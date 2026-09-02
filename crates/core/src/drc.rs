//! Post-route geometric DRC: exact clearance checks on the emitted geometry,
//! independent of the router's grid bookkeeping.

use crate::board::{pair_clearance, Board, NetClass, Pad};
use crate::geometry::{dist, P};
use crate::grid::effective_classes;
use crate::router::RouteResult;
use rayon::prelude::*;

fn class_of<'a>(classes: &'a [NetClass], board: &Board, net: usize) -> &'a NetClass {
    &classes[board.nets[net].class.min(classes.len() - 1)]
}

fn pad_clearance(classes: &[NetClass], board: &Board, pad: &Pad) -> f64 {
    match pad.net {
        Some(n) => class_of(classes, board, n).clearance,
        None => classes[0].clearance,
    }
}

#[derive(Debug, Clone)]
pub struct Violation {
    pub net_a: usize,
    pub net_b: usize,
    pub layer: Option<usize>,
    pub at: P,
    pub gap: f64,
    pub required: f64,
    pub kind: &'static str,
}

/// Distance between two segments (crate-internal helper for refinement).
pub(crate) fn seg_seg_distance(a1: P, a2: P, b1: P, b2: P) -> f64 {
    seg_seg_closest(a1, a2, b1, b2).0
}

/// Closest points between two segments: returns (distance, point on a, point on b).
fn seg_seg_closest(a1: P, a2: P, b1: P, b2: P) -> (f64, P, P) {
    fn closest_on(p: P, a: P, b: P) -> P {
        let ab = [b[0] - a[0], b[1] - a[1]];
        let len2 = ab[0] * ab[0] + ab[1] * ab[1];
        if len2 < 1e-12 {
            return a;
        }
        let t = (((p[0] - a[0]) * ab[0] + (p[1] - a[1]) * ab[1]) / len2).clamp(0.0, 1.0);
        [a[0] + ab[0] * t, a[1] + ab[1] * t]
    }
    if segs_intersect(a1, a2, b1, b2) {
        // Use the crossing point for both.
        let d1 = orient(b1, b2, a1);
        let d2 = orient(b1, b2, a2);
        let t = if (d1 - d2).abs() > 1e-12 { d1 / (d1 - d2) } else { 0.5 };
        let x = [a1[0] + (a2[0] - a1[0]) * t, a1[1] + (a2[1] - a1[1]) * t];
        return (0.0, x, x);
    }
    let mut best = (f64::INFINITY, a1, b1);
    for (p, on_b, sa, sb) in [
        (a1, true, b1, b2),
        (a2, true, b1, b2),
        (b1, false, a1, a2),
        (b2, false, a1, a2),
    ] {
        let q = closest_on(p, sa, sb);
        let d = crate::geometry::dist(p, q);
        if d < best.0 {
            best = if on_b { (d, p, q) } else { (d, q, p) };
        }
    }
    best
}

fn orient(a: P, b: P, c: P) -> f64 {
    (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
}

fn segs_intersect(a1: P, a2: P, b1: P, b2: P) -> bool {
    let d1 = orient(b1, b2, a1);
    let d2 = orient(b1, b2, a2);
    let d3 = orient(a1, a2, b1);
    let d4 = orient(a1, a2, b2);
    ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
}

struct Item {
    net: usize,
    class: usize,
    layer: Option<usize>, // None = spans all layers (via)
    a: P,
    b: P,
    half_width: f64,
    kind: &'static str,
}

/// Check all net-to-net copper clearances in the routed result.
/// Returns violations where the copper gap is below `board.rules.clearance`
/// (with a small tolerance for grid rounding).
pub fn check(board: &Board, result: &RouteResult, tolerance: f64) -> Vec<Violation> {
    let classes = effective_classes(board);
    let mut items: Vec<Item> = Vec::new();
    for rn in &result.nets {
        let ci = board.nets[rn.net].class.min(classes.len() - 1);
        let class = &classes[ci];
        for seg in &rn.segs {
            for w in seg.pts.windows(2) {
                items.push(Item {
                    net: rn.net,
                    class: ci,
                    layer: Some(seg.layer),
                    a: w[0],
                    b: w[1],
                    half_width: seg.width / 2.0,
                    kind: "track",
                });
            }
        }
        for v in &rn.vias {
            items.push(Item {
                net: rn.net,
                class: ci,
                layer: None,
                a: *v,
                b: *v,
                half_width: class.via_diameter / 2.0,
                kind: "via",
            });
        }
    }

    let out: Vec<Violation> = (0..items.len())
        .into_par_iter()
        .flat_map_iter(|i| {
            let items = &items;
            let board = &*board;
            let classes = &classes;
            ((i + 1)..items.len()).filter_map(move |j| {
                let (x, y) = (&items[i], &items[j]);
                if x.net == y.net {
                    return None;
                }
                if let (Some(a), Some(b)) = (x.layer, y.layer) {
                    if a != b {
                        return None;
                    }
                }
                let (d, px, py) = seg_seg_closest(x.a, x.b, y.a, y.b);
                let gap = d - x.half_width - y.half_width;
                let clearance =
                    pair_clearance(&classes, &board.pair_clearances, x.class, y.class);
                if gap >= clearance - tolerance {
                    return None;
                }
                // Copper fully inside its own net's pad adds nothing beyond
                // the pad itself: spacing there is pad placement, not routing
                // (pad-entry stubs are snapped to pad centers).
                let subsumed = |net: usize, p: P, hw: f64| {
                    board.pads.iter().any(|op| {
                        !op.is_pour && op.net == Some(net) && op.shape.signed_dist(p) <= -hw
                    })
                };
                if subsumed(x.net, px, x.half_width) || subsumed(y.net, py, y.half_width) {
                    return None;
                }
                if std::env::var_os("AUTOROUTE_DEBUG").is_some() {
                    eprintln!(
                        "debug drc tt: nets {}({}) vs {}({}) layers {:?}/{:?} segs {:?}-{:?} vs {:?}-{:?} d {d:.1} hw {}/{}",
                        x.net, x.kind, y.net, y.kind, x.layer, y.layer, x.a, x.b, y.a, y.b, x.half_width, y.half_width
                    );
                }
                Some(Violation {
                    net_a: x.net,
                    net_b: y.net,
                    layer: x.layer.or(y.layer),
                    at: [(px[0] + py[0]) / 2.0, (px[1] + py[1]) / 2.0],
                    gap,
                    required: clearance,
                    kind: if x.kind == "via" || y.kind == "via" {
                        "via-track"
                    } else {
                        "track-track"
                    },
                })
            })
        })
        .collect();
    out
}

/// Check via barrels against pads of other nets.
pub fn check_via_pads(board: &Board, result: &RouteResult, tolerance: f64) -> Vec<Violation> {
    let classes = effective_classes(board);
    let mut out = Vec::new();
    for rn in &result.nets {
        let class = class_of(&classes, board, rn.net);
        let via_r = class.via_diameter / 2.0;
        for v in &rn.vias {
            for pad in &board.pads {
                if pad.net == Some(rn.net) || pad.is_pour {
                    continue;
                }
                let clearance = class.clearance.max(pad_clearance(&classes, board, pad));
                let gap = pad.shape.signed_dist(*v) - via_r;
                if gap < clearance - tolerance {
                    out.push(Violation {
                        net_a: rn.net,
                        net_b: pad.net.unwrap_or(usize::MAX),
                        layer: None,
                        at: *v,
                        gap,
                        required: clearance,
                        kind: "via-pad",
                    });
                }
            }
        }
    }
    out
}

/// Check routed copper against pads of other nets.
pub fn check_pads(board: &Board, result: &RouteResult, tolerance: f64) -> Vec<Violation> {
    let classes = effective_classes(board);
    let mut out = Vec::new();
    for rn in &result.nets {
        let net_clearance = class_of(&classes, board, rn.net).clearance;
        for seg in &rn.segs {
            for pad in &board.pads {
                if pad.net == Some(rn.net) || pad.is_pour {
                    continue;
                }
                let clearance = net_clearance.max(pad_clearance(&classes, board, pad));
                if !pad.layers.contains(&seg.layer) {
                    continue;
                }
                for w in seg.pts.windows(2) {
                    // Sample along the segment against the pad's signed distance.
                    let steps = ((dist(w[0], w[1]) / 100.0).ceil() as usize).clamp(1, 64);
                    for s in 0..=steps {
                        let t = s as f64 / steps as f64;
                        let p = [
                            w[0][0] + (w[1][0] - w[0][0]) * t,
                            w[0][1] + (w[1][1] - w[0][1]) * t,
                        ];
                        let gap = pad.shape.signed_dist(p) - seg.width / 2.0;
                        // Wire copper fully inside one of its own net's pads
                        // adds nothing; spacing there is pad-to-pad placement,
                        // not routing.
                        let subsumed = || {
                            board.pads.iter().any(|op| {
                                op.net == Some(rn.net)
                                    && op.shape.signed_dist(p) <= -seg.width / 2.0
                            })
                        };
                        if gap < clearance - tolerance && !subsumed() {
                            if std::env::var_os("AUTOROUTE_DEBUG").is_some() {
                                eprintln!(
                                    "debug drc: pad {} {:?} vs seg L{} {:?}..{:?} at {:?} gap {gap:.1}",
                                    pad.id,
                                    pad.shape,
                                    seg.layer,
                                    w[0],
                                    w[1],
                                    p
                                );
                            }
                            out.push(Violation {
                                net_a: rn.net,
                                net_b: pad.net.unwrap_or(usize::MAX),
                                layer: Some(seg.layer),
                                at: p,
                                gap,
                                required: clearance,
                                kind: "track-pad",
                            });
                            break;
                        }
                    }
                }
            }
        }
    }
    out
}
