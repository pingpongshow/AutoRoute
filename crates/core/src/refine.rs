//! Post-route taut-polyline refinement: replaces staircase runs of grid
//! vertices with straight (any-angle) segments wherever the straight segment
//! clears all other copper by the exact pair clearance. Endpoints (pad
//! centers, via positions, layer-change junctions) are never moved, so
//! connectivity is untouched; the DRC can verify the result independently.

use crate::board::{Board, NetClass, Pad};
use crate::geometry::{poly_signed_dist, P};
use crate::grid::effective_classes;
use crate::router::RouteResult;

const SAFETY: f64 = 2.0; // um beyond the exact clearance
const HASH_CELL: f64 = 2000.0; // um

struct Item {
    net: Option<usize>,
    layer: Option<usize>, // None = all layers (via)
    a: P,
    b: P,
    half_width: f64,
    clearance: f64,
}

struct SpatialHash {
    cell: f64,
    map: std::collections::HashMap<(i64, i64), Vec<u32>>,
}

impl SpatialHash {
    fn build(items: &[Item], inflate: f64) -> SpatialHash {
        let mut map: std::collections::HashMap<(i64, i64), Vec<u32>> = Default::default();
        for (i, it) in items.iter().enumerate() {
            let (lo, hi) = seg_bbox(it.a, it.b, it.half_width + inflate);
            let (cx0, cy0) = (key(lo[0]), key(lo[1]));
            let (cx1, cy1) = (key(hi[0]), key(hi[1]));
            for cy in cy0..=cy1 {
                for cx in cx0..=cx1 {
                    map.entry((cx, cy)).or_default().push(i as u32);
                }
            }
        }
        SpatialHash { cell: HASH_CELL, map }
    }

    fn query(&self, lo: P, hi: P, out: &mut Vec<u32>) {
        out.clear();
        let (cx0, cy0) = (key(lo[0]), key(lo[1]));
        let (cx1, cy1) = (key(hi[0]), key(hi[1]));
        for cy in cy0..=cy1 {
            for cx in cx0..=cx1 {
                if let Some(v) = self.map.get(&(cx, cy)) {
                    out.extend_from_slice(v);
                }
            }
        }
        out.sort_unstable();
        out.dedup();
        let _ = self.cell;
    }
}

fn key(v: f64) -> i64 {
    (v / HASH_CELL).floor() as i64
}

fn seg_bbox(a: P, b: P, pad: f64) -> (P, P) {
    (
        [a[0].min(b[0]) - pad, a[1].min(b[1]) - pad],
        [a[0].max(b[0]) + pad, a[1].max(b[1]) + pad],
    )
}

fn class_of<'a>(classes: &'a [NetClass], board: &Board, net: usize) -> &'a NetClass {
    &classes[board.nets[net].class.min(classes.len() - 1)]
}

/// Straighten all routed polylines in place. Returns the number of vertices
/// removed.
pub fn refine(board: &Board, result: &mut RouteResult) -> usize {
    let classes = effective_classes(board);
    let mut removed = 0usize;

    // Boundary margin check helper.
    let inside_boundary = |p: P, margin: f64| -> bool {
        board.boundary.len() < 3 || poly_signed_dist(p, &board.boundary) <= -margin
    };

    let net_count = result.nets.len();
    for pass in 0..3 {
        let mut pass_removed = 0usize;
        for ni in 0..net_count {
            let net = result.nets[ni].net;
            let my_class = class_of(&classes, board, net);
            let my_hw = |seg_w: f64| seg_w / 2.0;

            // Items: everyone else's copper (tracks + vias).
            let mut items: Vec<Item> = Vec::new();
            for rn in result.nets.iter() {
                if rn.net == net {
                    continue;
                }
                let cl = class_of(&classes, board, rn.net);
                for seg in &rn.segs {
                    for w in seg.pts.windows(2) {
                        items.push(Item {
                            net: Some(rn.net),
                            layer: Some(seg.layer),
                            a: w[0],
                            b: w[1],
                            half_width: seg.width / 2.0,
                            clearance: cl.clearance,
                        });
                    }
                }
                for v in &rn.vias {
                    items.push(Item {
                        net: Some(rn.net),
                        layer: None,
                        a: *v,
                        b: *v,
                        half_width: cl.via_diameter / 2.0,
                        clearance: cl.clearance,
                    });
                }
            }
            // My own vias also constrain my own... no: same net, no clearance.
            let hash = SpatialHash::build(&items, 0.0);

            // Foreign pads, hashed too (coarsely by bbox).
            let pads: Vec<(&Pad, f64)> = board
                .pads
                .iter()
                .filter(|p| p.net != Some(net))
                .map(|p| {
                    let pc = match p.net {
                        Some(n) => class_of(&classes, board, n).clearance,
                        None => classes[0].clearance,
                    };
                    (p, pc)
                })
                .collect();

            let mut cand_ids: Vec<u32> = Vec::new();
            let nsegs = result.nets[ni].segs.len();
            for si in 0..nsegs {
                let (layer, width) = {
                    let s = &result.nets[ni].segs[si];
                    (s.layer, s.width)
                };
                let hw = my_hw(width);
                let boundary_margin = my_class.clearance + hw;

                let mut pts = std::mem::take(&mut result.nets[ni].segs[si].pts);
                let mut i = 0;
                while i + 2 < pts.len() {
                    // Try far-to-near shortcut targets, geometrically decreasing.
                    let mut j = pts.len() - 1;
                    let mut done = false;
                    while j > i + 1 {
                        let a = pts[i];
                        let b = pts[j];
                        if candidate_clear(
                            a, b, layer, hw, my_class, &items, &hash, &pads,
                            &mut cand_ids, boundary_margin, &inside_boundary,
                        ) {
                            let cut = j - i - 1;
                            pts.drain(i + 1..j);
                            pass_removed += cut;
                            done = true;
                            break;
                        }
                        let span = j - i;
                        j = i + (span / 2).max(1);
                        if j <= i + 1 {
                            break;
                        }
                    }
                    if !done {
                        i += 1;
                    } else {
                        i += 1;
                    }
                }
                result.nets[ni].segs[si].pts = pts;
            }
        }
        removed += pass_removed;
        if pass_removed == 0 {
            break;
        }
        let _ = pass;
    }
    // Recompute lengths.
    for rn in result.nets.iter_mut() {
        rn.length = rn
            .segs
            .iter()
            .map(|s| {
                s.pts
                    .windows(2)
                    .map(|w| ((w[0][0] - w[1][0]).powi(2) + (w[0][1] - w[1][1]).powi(2)).sqrt())
                    .sum::<f64>()
            })
            .sum();
    }
    removed
}

#[allow(clippy::too_many_arguments)]
fn candidate_clear(
    a: P,
    b: P,
    layer: usize,
    hw: f64,
    my_class: &NetClass,
    items: &[Item],
    hash: &SpatialHash,
    pads: &[(&Pad, f64)],
    cand_ids: &mut Vec<u32>,
    boundary_margin: f64,
    inside_boundary: &dyn Fn(P, f64) -> bool,
) -> bool {
    // Widest interaction reach we might need for the hash query.
    let reach = hw + my_class.clearance + 2500.0;
    let (lo, hi) = seg_bbox(a, b, reach);
    hash.query(lo, hi, cand_ids);
    for &id in cand_ids.iter() {
        let it = &items[id as usize];
        if let Some(l) = it.layer {
            if l != layer {
                continue;
            }
        }
        let req = hw + it.half_width + my_class.clearance.max(it.clearance) + SAFETY;
        let d = crate::drc::seg_seg_distance(a, b, it.a, it.b);
        if d < req {
            return false;
        }
    }
    // Pads on this layer (sampled signed distance).
    let len = ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
    let steps = ((len / 150.0).ceil() as usize).clamp(1, 256);
    for (pad, pad_clearance) in pads.iter() {
        if !pad.layers.contains(&layer) {
            continue;
        }
        let req = hw + my_class.clearance.max(*pad_clearance) + SAFETY;
        let (plo, phi) = pad.shape.bbox();
        let (clo, chi) = seg_bbox(a, b, req);
        if phi[0] < clo[0] || plo[0] > chi[0] || phi[1] < clo[1] || plo[1] > chi[1] {
            continue;
        }
        for s in 0..=steps {
            let t = s as f64 / steps as f64;
            let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
            if pad.shape.signed_dist(p) < req {
                return false;
            }
        }
    }
    // Board outline.
    for s in 0..=steps {
        let t = s as f64 / steps as f64;
        let p = [a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t];
        if !inside_boundary(p, boundary_margin - SAFETY) {
            return false;
        }
    }
    true
}
