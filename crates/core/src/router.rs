//! Grid router: multi-layer A* maze search per net, wrapped in a
//! PathFinder-style negotiated-congestion rip-up-and-reroute loop.
//!
//! Clearance model: each committed route "stamps" a footprint disc around its
//! centerline sized `width + clearance` (vias: `via_r + width/2 + clearance`).
//! Another net's centerline entering that footprint means the two nets are
//! closer than DRC allows; such overlaps are conflicts that later iterations
//! resolve by pricing the contested cells up (history + present congestion).

use std::cmp::Ordering;
use std::collections::BinaryHeap;
use std::sync::atomic::{AtomicBool, Ordering as AtomicOrdering};

use crate::board::{Board, NetClass};
use crate::geometry::P;
use crate::grid::{effective_classes, Grid, PadCells};
use rayon::prelude::*;

#[derive(Debug, Clone)]
pub struct RouterConfig {
    /// Grid pitch, um. If None, derived from the board rules.
    pub pitch: Option<f64>,
    /// Maximum rip-up/reroute iterations.
    pub max_iterations: usize,
    /// Extra cost for inserting a via, in um of equivalent track length.
    pub via_cost: f64,
    /// Cost bonus for continuing straight (penalty for turning), um per turn.
    pub turn_penalty: f64,
    /// History cost added to contested cells each iteration, in grid pitches.
    pub history_increment: f64,
    /// Present-congestion price growth factor per iteration.
    pub present_growth: f64,
    /// Present-congestion price cap, in grid pitches.
    pub present_cap: f64,
    /// Class names to leave unrouted (their pads still act as obstacles).
    /// Matched case-insensitively as substrings of the class name.
    pub skip_classes: Vec<String>,
    /// When non-empty, route only nets matching one of these patterns
    /// ("*" wildcard supported).
    pub only_nets: Vec<String>,
    /// Post-completion cleanup passes: each routed net is rerouted alone and
    /// the new route kept only if it is strictly better (fewer vias/shorter).
    pub optimize_passes: usize,
    /// Preferred routing direction per layer: 'h', 'v', or 'a' (any).
    /// None = automatic (inner layers alternate h/v on 3+ layer boards).
    pub layer_dirs: Option<String>,
}

impl Default for RouterConfig {
    fn default() -> Self {
        RouterConfig {
            pitch: None,
            max_iterations: 40,
            via_cost: 5000.0,
            turn_penalty: 60.0,
            history_increment: 1.5,
            present_growth: 2.0,
            present_cap: 120.0,
            skip_classes: Vec::new(),
            only_nets: Vec::new(),
            optimize_passes: 1,
            layer_dirs: None,
        }
    }
}

impl RouterConfig {
    pub fn effective_pitch(&self, board: &Board) -> f64 {
        self.pitch.unwrap_or_else(|| {
            // Two grid-adjacent tracks sit 2*pitch apart, and must clear
            // width + clearance. A factor just above 0.5 keeps the grid-legal
            // spacing close to the true DRC minimum (little wasted capacity)
            // without conflict-testing exactly at the boundary. Derive from
            // the finest class so its nets get full resolution.
            let finest = effective_classes(board)
                .iter()
                .map(|c| c.width + c.clearance)
                .fold(f64::INFINITY, f64::min);
            (finest * 0.53).clamp(80.0, 500.0)
        })
    }
}

#[derive(Debug, Clone)]
pub struct RouteSeg {
    pub layer: usize,
    pub width: f64,
    pub pts: Vec<P>,
}

#[derive(Debug, Clone, Default)]
pub struct RoutedNet {
    pub net: usize,
    pub segs: Vec<RouteSeg>,
    pub vias: Vec<P>,
    /// Total routed track length, um.
    pub length: f64,
    pub failed: bool,
    pub in_conflict: bool,
}

#[derive(Debug, Clone, Default)]
pub struct RouteResult {
    pub nets: Vec<RoutedNet>,
    pub iterations: usize,
    pub failed_nets: usize,
    pub conflict_nets: usize,
    pub complete: bool,
}

pub struct Progress<'a> {
    pub iteration: usize,
    pub nets_done: usize,
    pub nets_total: usize,
    /// True for the callback fired after conflict analysis at the end of an
    /// iteration; only then are `result.conflict_nets`/`failed_nets` current.
    pub end_of_iteration: bool,
    pub result: &'a RouteResult,
}

/// Internal bookkeeping for one net's committed route. Stamp lists are per
/// bucket (outer index), since each bucket's field gets its own radii.
#[derive(Default, Clone)]
struct NetState {
    /// Wire centerline cells (tested against the wire-entrant field).
    centerline: Vec<u32>,
    /// Via barrel cells on every layer (tested against the via-entrant field).
    via_centers: Vec<u32>,
    /// Deduped footprint cells currently stamped into `usage`, per bucket.
    stamp: Vec<Vec<u32>>,
    /// Deduped footprint cells currently stamped into `vusage`, per bucket.
    vstamp: Vec<Vec<u32>>,
    /// Deduped soft-margin cells currently stamped into `aura`, per bucket.
    aura_stamp: Vec<Vec<u32>>,
    /// Deduped no-diagonal cells currently stamped into `via_tight`, per bucket.
    vt_stamp: Vec<Vec<u32>>,
    /// Raw cell paths of each routed leg, in order (for geometry extraction).
    legs: Vec<Leg>,
    routed: bool,
}

/// One routed leg: its raw cell path plus the pads it terminates at (used to
/// snap wire endpoints to exact pad centers during geometry extraction).
#[derive(Clone)]
struct Leg {
    cells: Vec<u32>,
    start_pad: Option<usize>,
    end_pad: Option<usize>,
}

struct Search {
    g: Vec<f32>,
    gen: Vec<u32>,
    came: Vec<u32>,
    tgen: Vec<u32>,
    cur_gen: u32,
    cur_tgen: u32,
}

#[derive(PartialEq)]
struct HeapEntry {
    f: f64,
    c: u32,
}
impl Eq for HeapEntry {}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // Reverse: BinaryHeap is a max-heap, we want the smallest f on top.
        other.f.total_cmp(&self.f)
    }
}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

const NO_CAME: u32 = u32::MAX;

/// Stamp discs for one stamping class into every bucket's fields
/// (each inner Vec is indexed by bucket).
struct ClassDiscs {
    /// Wire-entrant congestion, stamped by wires / vias (row spans).
    wire_usage: Vec<Vec<(i32, i32, i32)>>,
    via_usage: Vec<Vec<(i32, i32, i32)>>,
    /// Via-entrant congestion, stamped by wires (orthogonal / diagonal cells)
    /// and vias.
    wire_vusage: Vec<Vec<(i32, i32, i32)>>,
    wire_vusage_diag: Vec<Vec<(i32, i32, i32)>>,
    via_vusage: Vec<Vec<(i32, i32, i32)>>,
    /// Soft margin (one cell beyond the wire-entrant footprint).
    wire_aura: Vec<Vec<(i32, i32, i32)>>,
    via_aura: Vec<Vec<(i32, i32, i32)>>,
    /// No-diagonal zone around vias.
    via_tight: Vec<Vec<(i32, i32, i32)>>,
}

pub struct Router<'a> {
    board: &'a Board,
    cfg: RouterConfig,
    grid: Grid,
    pad_cells: PadCells,
    /// Net class parameters (from the board, or a synthesized default).
    classes: Vec<NetClass>,
    /// Net index -> bucket index (into `grid.buckets`).
    net_bucket: Vec<usize>,
    /// Wire-entrant congestion per (bucket, cell): what a WIRE centerline of
    /// that bucket entering a cell is too close to.
    usage: Vec<u16>,
    /// Via-entrant congestion per (bucket, cell): what a VIA barrel of that
    /// bucket placed at a cell is too close to. Without this separate field,
    /// via placement is blind to wires that are legal for wires but too close
    /// for a barrel, and every via placement shoves innocent nets around.
    vusage: Vec<u16>,
    /// Soft congestion per (bucket, cell): like `usage` but one cell wider.
    /// Skimming the exact edge of a foreign footprint is legal but fragile; a
    /// small price on the aura makes routes keep margin where space allows,
    /// which stops the negotiation from churning on knife-edge spacings.
    aura: Vec<u16>,
    /// No-diagonal counts around committed vias per (bucket, cell): a
    /// 45-degree wire segment can dip up to half a diagonal step closer to a
    /// via than its endpoints, so within this zone wires move orthogonally.
    via_tight: Vec<u16>,
    /// History cost, shared across buckets (single plane).
    hist: Vec<f32>,
    /// Differential-pair partner per net (by name convention).
    pub pairs: Vec<Option<usize>>,
    /// Per-layer (horizontal-move penalty, vertical-move penalty), um.
    dir_pen: Vec<(f64, f64)>,
    /// Attraction counts (single plane): cells near an already-routed pair
    /// partner get a cost discount so the second net hugs the first.
    attract: Vec<u16>,
    attract_w: f64,
    /// True while a pair partner's attraction field is stamped.
    attract_on: bool,
    states: Vec<NetState>,
    /// Stamp discs indexed by stamping class.
    discs: Vec<ClassDiscs>,
    /// Per net: sorted terminal (own pad interior) cells. Centerline cells in
    /// here are protected by hard blocking, not by footprint stamps.
    net_terms: Vec<Vec<u32>>,
    p_fac: f64,
    aura_w: f64,
    search: Search,
}


impl<'a> Router<'a> {
    pub fn new(board: &'a Board, cfg: RouterConfig) -> Router<'a> {
        let pitch = cfg.effective_pitch(board);
        let (grid, pad_cells) = Grid::build(board, pitch);
        let ncells = grid.ncells;
        let nb = grid.buckets.len();
        let classes = effective_classes(board);
        let net_bucket: Vec<usize> = board
            .nets
            .iter()
            .map(|n| grid.class_bucket[n.class.min(classes.len() - 1)])
            .collect();

        // Stamp radii keyed by (stamping class, entering bucket): each equals
        // the exact minimum legal center-to-center distance for that pair
        // (pair clearance = max of the two classes'). Fanout of fine-pitch
        // parts lives or dies on tens of um of slack, so no padding here; the
        // sub-cell dip of diagonal wire segments is handled by no-diagonal
        // zones and by the padded diagonal-cell vusage disc instead.
        let quant = 0.5 * std::f64::consts::SQRT_2 * pitch;
        let discs: Vec<ClassDiscs> = classes
            .iter()
            .map(|k| {
                let wk = k.width / 2.0;
                let vrk = k.via_diameter / 2.0;
                let mut cd = ClassDiscs {
                    wire_usage: Vec::new(),
                    via_usage: Vec::new(),
                    wire_vusage: Vec::new(),
                    wire_vusage_diag: Vec::new(),
                    via_vusage: Vec::new(),
                    wire_aura: Vec::new(),
                    via_aura: Vec::new(),
                    via_tight: Vec::new(),
                };
                for bp in &grid.buckets {
                    let cc = k.clearance.max(bp.clearance);
                    let wb = bp.width / 2.0;
                    let vrb = bp.via_diameter / 2.0;
                    cd.wire_usage.push(disc_spans((wk + cc + wb) / pitch));
                    cd.via_usage.push(disc_spans((vrk + cc + wb) / pitch));
                    cd.wire_vusage.push(disc_spans((wk + cc + vrb) / pitch));
                    cd.wire_vusage_diag
                        .push(disc_spans((wk + cc + vrb + quant) / pitch));
                    cd.via_vusage.push(disc_spans((vrk + cc + vrb) / pitch));
                    cd.wire_aura.push(disc_spans((wk + cc + wb) / pitch + 1.0));
                    cd.via_aura.push(disc_spans((vrk + cc + wb) / pitch + 1.0));
                    cd.via_tight
                        .push(disc_spans((vrk + cc + wb + quant) / pitch));
                }
                cd
            })
            .collect();
        let nnets = board.nets.len();
        let mut net_terms: Vec<Vec<u32>> = vec![Vec::new(); nnets];
        for (pi, pad) in board.pads.iter().enumerate() {
            if let Some(net) = pad.net {
                net_terms[net].extend_from_slice(&pad_cells[pi]);
            }
        }
        for t in &mut net_terms {
            t.sort_unstable();
            t.dedup();
        }
        let dir_pen = layer_dir_penalties(&cfg, grid.nl, pitch);
        let empty_state = NetState {
            stamp: vec![Vec::new(); nb],
            vstamp: vec![Vec::new(); nb],
            aura_stamp: vec![Vec::new(); nb],
            vt_stamp: vec![Vec::new(); nb],
            ..Default::default()
        };
        Router {
            board,
            cfg,
            grid,
            pad_cells,
            classes,
            net_bucket,
            usage: vec![0; nb * ncells],
            vusage: vec![0; nb * ncells],
            aura: vec![0; nb * ncells],
            via_tight: vec![0; nb * ncells],
            hist: vec![0.0; ncells],
            pairs: detect_pairs(
                &board.nets.iter().map(|n| n.name.clone()).collect::<Vec<_>>(),
            ),
            dir_pen,
            attract: vec![0; ncells],
            attract_w: 0.6 * pitch,
            attract_on: false,
            states: vec![empty_state; nnets],
            discs,
            net_terms,
            p_fac: 0.0,
            aura_w: 0.35 * pitch,
            search: Search {
                g: vec![0.0; ncells],
                gen: vec![0; ncells],
                came: vec![NO_CAME; ncells],
                tgen: vec![0; ncells],
                cur_gen: 0,
                cur_tgen: 0,
            },
        }
    }

    #[inline]
    fn bi(&self, b: usize, c: u32) -> usize {
        b * self.grid.ncells + c as usize
    }

    pub fn grid_info(&self) -> (usize, usize, usize, f64) {
        (self.grid.nl, self.grid.nx, self.grid.ny, self.grid.pitch)
    }

    /// Run the full negotiation loop. `progress` is called after every routed
    /// net and every iteration; `cancel` stops at the next net boundary.
    pub fn route(
        &mut self,
        cancel: &AtomicBool,
        progress: &mut dyn FnMut(&Progress),
    ) -> RouteResult {
        let pitch = self.grid.pitch;
        let mut result = RouteResult {
            nets: (0..self.board.nets.len())
                .map(|i| RoutedNet { net: i, ..Default::default() })
                .collect(),
            ..Default::default()
        };

        // Route shorter nets first: they have fewer detour options.
        #[allow(unused_mut)]
        let mut order: Vec<usize> = (0..self.board.nets.len())
            .filter(|&n| self.routable(n))
            .collect();
        order.sort_by(|&a, &b| {
            self.net_extent(a)
                .total_cmp(&self.net_extent(b))
        });
        // Keep differential-pair partners adjacent so the second is routed
        // right after (and attracted to) the first.
        {
            let mut seen = vec![false; self.board.nets.len()];
            let mut reordered = Vec::with_capacity(order.len());
            for &n in &order {
                if seen[n] {
                    continue;
                }
                seen[n] = true;
                reordered.push(n);
                if let Some(m) = self.pairs[n] {
                    if !seen[m] && self.routable(m) {
                        seen[m] = true;
                        reordered.push(m);
                    }
                }
            }
            order = reordered;
        }
        let total = order.len();

        self.p_fac = 0.5 * pitch;
        let hist_inc = (self.cfg.history_increment * pitch) as f32;

        let mut targets: Vec<usize> = order.clone();
        // Plateau detection: consecutive iterations with an identical
        // (conflicted + failed) set mean the negotiation has reached a stable
        // equilibrium and further iterations are wasted work.
        let mut last_next: Vec<usize> = Vec::new();
        let mut plateau = 0usize;
        for iter in 0..self.cfg.max_iterations {
            result.iterations = iter + 1;

            // Rip up and reroute one net at a time, so each rerouted net sees
            // the present congestion of every other net (PathFinder style).
            let mut done = 0usize;
            for &n in &targets {
                if cancel.load(AtomicOrdering::Relaxed) {
                    self.finalize(&mut result);
                    return result;
                }
                // A net that failed outright (no path exists at any price)
                // will keep failing until the board around it changes; probing
                // it every iteration costs a full-grid search each time, so
                // only retry failed-but-unconflicted nets every 4th iteration.
                if iter > 0 && !self.states[n].routed && iter % 4 != 0 && !self.in_conflict(n) {
                    continue;
                }
                self.rip_up(n);
                let attracted = self.stamp_attraction(n);
                self.route_net(n);
                if attracted {
                    self.clear_attraction();
                }
                self.extract_geometry(n, &mut result.nets[n]);
                done += 1;
                progress(&Progress {
                    iteration: iter,
                    nets_done: done,
                    nets_total: total,
                    end_of_iteration: false,
                    result: &result,
                });
            }

            // Find conflicted or failed nets for the next round.
            let mut next: Vec<usize> = Vec::new();
            for &n in &order {
                let conflicted = self.in_conflict(n);
                result.nets[n].in_conflict = conflicted;
                if conflicted || !self.states[n].routed {
                    next.push(n);
                }
            }
            self.finalize(&mut result);
            progress(&Progress {
                iteration: iter,
                nets_done: total,
                nets_total: total,
                end_of_iteration: true,
                result: &result,
            });
            if next.is_empty() {
                result.complete = true;
                break;
            }
            if next == last_next {
                plateau += 1;
                // The same nets have been stuck for a while: equilibrium.
                if plateau >= 6 {
                    break;
                }
            } else {
                plateau = 0;
                last_next = next.clone();
            }

            // Charge history on contested cells (any bucket); raise the
            // present-sharing price.
            let ncells = self.grid.ncells;
            for c in 0..ncells {
                let contested = (0..self.grid.buckets.len())
                    .any(|b| self.usage[b * ncells + c] >= 2);
                if contested {
                    self.hist[c] += hist_inc;
                }
            }
            self.p_fac = (self.p_fac * self.cfg.present_growth).min(self.cfg.present_cap * pitch);
            // Reroute everything while the design is still fluid: parked nets
            // holding prime corridors must be able to move aside. Once the
            // conflict set is small, focus on just those nets.
            targets = if next.len() * 4 > total {
                order.clone()
            } else {
                next
            };
        }

        if !result.complete {
            self.repair_clusters(cancel, &mut result);
        }
        if self.cfg.optimize_passes > 0 && !cancel.load(AtomicOrdering::Relaxed) {
            self.optimize(&order, cancel, &mut result);
        }
        self.finalize(&mut result);
        if std::env::var_os("AUTOROUTE_DEBUG").is_some() {
            self.dump_conflicts(15);
        }
        result
    }

    /// Debug helper: print a sample of conflicted cells with their combatants.
    pub fn dump_conflicts(&self, limit: usize) {
        let mut shown = 0;
        for n in 0..self.states.len() {
            if !self.routable(n) || !self.in_conflict(n) {
                continue;
            }
            let st = &self.states[n];
            let b = self.net_bucket[n];
            let wire_hit = st.centerline.iter().find(|&&c| self.usage[self.bi(b, c)] >= 2);
            let via_hit = st.via_centers.iter().find(|&&c| self.vusage[self.bi(b, c)] >= 2);
            if let Some(&c) = wire_hit.or(via_hit) {
                let kind = if wire_hit.is_some() { "wire" } else { "via" };
                let (l, _, _) = self.grid.decompose(c);
                let p = self.grid.cell_center(c);
                let others: Vec<usize> = (0..self.states.len())
                    .filter(|&m| {
                        m != n
                            && (self.states[m].stamp[b].binary_search(&c).is_ok()
                                || self.states[m].vstamp[b].binary_search(&c).is_ok())
                    })
                    .collect();
                eprintln!(
                    "conflict: net {n} ({}) {kind} cell L{l} ({:.0},{:.0}) vs nets {:?}",
                    self.board.nets[n].name, p[0], p[1], others
                );
                shown += 1;
            }
            if shown >= limit {
                return;
            }
        }
    }

    /// If net n's pair partner is already routed, discount cells near the
    /// partner's centerline (same layer) so n runs coupled to it.
    fn stamp_attraction(&mut self, n: usize) -> bool {
        let Some(m) = self.pairs[n] else { return false };
        if !self.states[m].routed || self.states[m].centerline.is_empty() {
            return false;
        }
        // Target coupling distance: adjacent legal spacing for the pair.
        let k = self.board.nets[n].class.min(self.classes.len() - 1);
        let cl = &self.classes[k];
        let radius = (cl.width + cl.clearance) * 1.6 / self.grid.pitch;
        let disc = disc_offsets(radius);
        let centerline = self.states[m].centerline.clone();
        for &c in &centerline {
            let (l, x, y) = self.grid.decompose(c);
            for &(dx, dy) in &disc {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;
                if nx < 0 || ny < 0 || nx as usize >= self.grid.nx || ny as usize >= self.grid.ny {
                    continue;
                }
                let i = self.grid.idx(l, nx as usize, ny as usize) as usize;
                self.attract[i] = self.attract[i].saturating_add(1);
            }
        }
        self.attract_on = true;
        true
    }

    fn clear_attraction(&mut self) {
        self.attract.fill(0);
        self.attract_on = false;
    }

    /// Local repair: conflicted nets often deadlock in pairs/small clusters
    /// where the global negotiation's fixed order can't find the solution.
    /// Rip up each cluster and retry it in several orders; keep the best.
    fn repair_clusters(&mut self, cancel: &AtomicBool, result: &mut RouteResult) {
        let conflicted: Vec<usize> = (0..self.states.len())
            .filter(|&n| self.routable(n) && self.in_conflict(n))
            .collect();
        if conflicted.is_empty() {
            return;
        }
        // Cluster nets that fight over the same cells (stamp covering another
        // conflicted net's centerline, in either one's bucket).
        let mut parent: Vec<usize> = (0..conflicted.len()).collect();
        fn find(p: &mut Vec<usize>, i: usize) -> usize {
            if p[i] != i {
                let r = find(p, p[i]);
                p[i] = r;
            }
            p[i]
        }
        for i in 0..conflicted.len() {
            for j in (i + 1)..conflicted.len() {
                let (n, m) = (conflicted[i], conflicted[j]);
                let bn = self.net_bucket[n];
                let overlap = self.states[n].centerline.iter().chain(&self.states[n].via_centers).any(|c| {
                    self.states[m].stamp[bn].binary_search(c).is_ok()
                        || self.states[m].vstamp[bn].binary_search(c).is_ok()
                });
                if overlap {
                    let (a, b) = (find(&mut parent, i), find(&mut parent, j));
                    parent[a] = b;
                }
            }
        }
        let mut clusters: std::collections::HashMap<usize, Vec<usize>> = Default::default();
        for i in 0..conflicted.len() {
            let r = find(&mut parent, i);
            clusters.entry(r).or_default().push(conflicted[i]);
        }

        for cluster in clusters.values() {
            if cluster.len() < 2 || cluster.len() > 8 {
                continue;
            }
            if cancel.load(AtomicOrdering::Relaxed) {
                return;
            }
            let snapshot = |slf: &Self| -> Vec<(Vec<Leg>, bool)> {
                cluster
                    .iter()
                    .map(|&n| (slf.states[n].legs.clone(), slf.states[n].routed))
                    .collect()
            };
            let restore = |slf: &mut Self, snap: &[(Vec<Leg>, bool)]| {
                for (&n, (legs, routed)) in cluster.iter().zip(snap.iter()) {
                    slf.rip_up(n);
                    for leg in legs.clone() {
                        slf.commit_leg(n, leg);
                    }
                    slf.states[n].routed = *routed;
                }
            };
            let count = |slf: &Self| -> usize {
                cluster
                    .iter()
                    .filter(|&&n| slf.in_conflict(n) || !slf.states[n].routed)
                    .count()
            };

            let original = snapshot(self);
            let mut best_count = count(self);
            let mut best_snap: Option<Vec<(Vec<Leg>, bool)>> = None;
            let k = cluster.len();
            let mut orders: Vec<Vec<usize>> = vec![{
                let mut r = cluster.clone();
                r.reverse();
                r
            }];
            for rot in 1..k.min(4) {
                let mut o = cluster.clone();
                o.rotate_left(rot);
                orders.push(o);
            }
            for ord in orders {
                for &n in &ord {
                    self.rip_up(n);
                }
                for &n in &ord {
                    self.route_net(n);
                }
                let c = count(self);
                if c < best_count {
                    best_count = c;
                    best_snap = Some(snapshot(self));
                    if c == 0 {
                        break;
                    }
                }
                restore(self, &original);
            }
            if let Some(snap) = best_snap {
                restore(self, &snap);
                for &n in cluster.iter() {
                    let idx = n;
                    let mut rn = std::mem::take(&mut result.nets[idx]);
                    rn.net = idx;
                    self.extract_geometry(idx, &mut rn);
                    rn.in_conflict = self.in_conflict(idx);
                    result.nets[idx] = rn;
                }
            }
        }
    }

    /// Cleanup: reroute each net alone against the final board state; keep
    /// the new route only if it is strictly better and conflict-free.
    fn optimize(&mut self, order: &[usize], cancel: &AtomicBool, result: &mut RouteResult) {
        let metric = |st: &NetState, grid: &Grid| -> (usize, f64) {
            let mut vias = 0usize;
            let mut len = 0.0;
            for leg in &st.legs {
                for w in leg.cells.windows(2) {
                    if layer_change(grid, w[0], w[1]) {
                        vias += 1;
                    } else {
                        let a = grid.cell_center(w[0]);
                        let b = grid.cell_center(w[1]);
                        len += ((a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)).sqrt();
                    }
                }
            }
            (vias, len)
        };
        for pass in 0..self.cfg.optimize_passes {
            let mut improved = 0usize;
            for &n in order {
                if cancel.load(AtomicOrdering::Relaxed) {
                    return;
                }
                if !self.states[n].routed || self.in_conflict(n) {
                    continue;
                }
                let old_legs = self.states[n].legs.clone();
                let (old_vias, old_len) = metric(&self.states[n], &self.grid);
                self.rip_up(n);
                let attracted = self.stamp_attraction(n);
                self.route_net(n);
                if attracted {
                    self.clear_attraction();
                }
                let (new_vias, new_len) = metric(&self.states[n], &self.grid);
                let better = self.states[n].routed
                    && !self.in_conflict(n)
                    && (new_vias < old_vias
                        || (new_vias == old_vias && new_len < old_len - 1.0));
                if better {
                    improved += 1;
                    self.extract_geometry(n, &mut result.nets[n]);
                } else {
                    // Restore the previous route.
                    self.rip_up(n);
                    for leg in old_legs {
                        self.commit_leg(n, leg);
                    }
                    self.states[n].routed = true;
                    self.extract_geometry(n, &mut result.nets[n]);
                }
            }
            if improved == 0 {
                break;
            }
            let _ = pass;
        }
    }

    fn finalize(&self, result: &mut RouteResult) {
        result.failed_nets = self.states.iter().enumerate()
            .filter(|(n, s)| self.routable(*n) && !s.routed)
            .count();
        result.conflict_nets = (0..self.states.len())
            .filter(|&n| self.routable(n) && self.in_conflict(n))
            .count();
        result.complete = result.failed_nets == 0 && result.conflict_nets == 0;
    }

    fn routable(&self, n: usize) -> bool {
        if self.board.nets[n].pads.len() < 2 || self.board.nets[n].fixed {
            return false;
        }
        if !self.cfg.skip_classes.is_empty() {
            let cname = self.classes[self.board.nets[n].class.min(self.classes.len() - 1)]
                .name
                .to_lowercase();
            if self
                .cfg
                .skip_classes
                .iter()
                .any(|s| !s.is_empty() && cname.contains(&s.to_lowercase()))
            {
                return false;
            }
        }
        if !self.cfg.only_nets.is_empty() {
            let name = &self.board.nets[n].name;
            if !self.cfg.only_nets.iter().any(|p| glob_match(p, name)) {
                return false;
            }
        }
        true
    }

    fn net_extent(&self, n: usize) -> f64 {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for &pi in &self.board.nets[n].pads {
            let c = self.board.pads[pi].shape.center();
            for k in 0..2 {
                lo[k] = lo[k].min(c[k]);
                hi[k] = hi[k].max(c[k]);
            }
        }
        (hi[0] - lo[0]) + (hi[1] - lo[1])
    }

    fn rip_up(&mut self, n: usize) {
        let nb = self.grid.buckets.len();
        let ncells = self.grid.ncells;
        let st = &mut self.states[n];
        for b in 0..nb {
            for &c in &st.stamp[b] {
                let i = b * ncells + c as usize;
                self.usage[i] = self.usage[i].saturating_sub(1);
            }
            for &c in &st.vstamp[b] {
                let i = b * ncells + c as usize;
                self.vusage[i] = self.vusage[i].saturating_sub(1);
            }
            for &c in &st.aura_stamp[b] {
                let i = b * ncells + c as usize;
                self.aura[i] = self.aura[i].saturating_sub(1);
            }
            for &c in &st.vt_stamp[b] {
                let i = b * ncells + c as usize;
                self.via_tight[i] = self.via_tight[i].saturating_sub(1);
            }
        }
        *st = NetState {
            stamp: vec![Vec::new(); nb],
            vstamp: vec![Vec::new(); nb],
            aura_stamp: vec![Vec::new(); nb],
            vt_stamp: vec![Vec::new(); nb],
            ..Default::default()
        };
    }

    fn in_conflict(&self, n: usize) -> bool {
        let b = self.net_bucket[n];
        let st = &self.states[n];
        st.centerline.iter().any(|&c| self.usage[self.bi(b, c)] >= 2)
            || st.via_centers.iter().any(|&c| self.vusage[self.bi(b, c)] >= 2)
    }

    /// Route one net: greedy incremental Steiner tree over its pads.
    fn route_net(&mut self, n: usize) {
        let pads: Vec<usize> = self.board.nets[n].pads.clone();
        if pads.len() < 2 {
            self.states[n].routed = true;
            return;
        }

        // Start from the pad closest to the net centroid.
        let centers: Vec<P> = pads.iter().map(|&pi| self.board.pads[pi].shape.center()).collect();
        let centroid = [
            centers.iter().map(|c| c[0]).sum::<f64>() / centers.len() as f64,
            centers.iter().map(|c| c[1]).sum::<f64>() / centers.len() as f64,
        ];
        let mut remaining: Vec<usize> = (0..pads.len()).collect();
        let start = remaining
            .iter()
            .copied()
            .min_by(|&a, &b| {
                crate::geometry::dist(centers[a], centroid)
                    .total_cmp(&crate::geometry::dist(centers[b], centroid))
            })
            .unwrap();
        remaining.retain(|&i| i != start);

        let mut tree: Vec<u32> = self.pad_cells[pads[start]].clone();
        let mut tree_centers: Vec<P> = vec![centers[start]];
        let mut legs: Vec<Leg> = Vec::new();
        let start_pad = pads[start];
        let mut ok = true;

        while !remaining.is_empty() {
            // Nearest unconnected pad to the current tree.
            let (ri, _) = remaining
                .iter()
                .enumerate()
                .min_by(|(_, &a), (_, &b)| {
                    let da = tree_centers.iter().map(|t| crate::geometry::dist(*t, centers[a]))
                        .fold(f64::INFINITY, f64::min);
                    let db = tree_centers.iter().map(|t| crate::geometry::dist(*t, centers[b]))
                        .fold(f64::INFINITY, f64::min);
                    da.total_cmp(&db)
                })
                .map(|(i, &p)| (i, p))
                .unwrap();
            let pad_local = remaining.remove(ri);
            let target_cells = self.pad_cells[pads[pad_local]].clone();

            // Try a window around sources+targets first; a full-grid sweep
            // only when the local search fails.
            let window = self.search_window(&tree, &target_cells, 64);
            let attempt = self
                .astar(n as i32, self.net_bucket[n], &tree, &target_cells, window)
                .or_else(|| {
                    if window.is_some() {
                        self.astar(n as i32, self.net_bucket[n], &tree, &target_cells, None)
                    } else {
                        None
                    }
                });
            match attempt {
                Some(path) => {
                    // Commit only after every leg is routed: a net is allowed
                    // to run close to itself, so its own earlier legs must not
                    // show up as congestion while routing the later ones.
                    tree.extend_from_slice(&path);
                    tree_centers.push(centers[pad_local]);
                    legs.push(Leg {
                        cells: path,
                        start_pad: if legs.is_empty() { Some(start_pad) } else { None },
                        end_pad: Some(pads[pad_local]),
                    });
                }
                None => {
                    ok = false;
                    if std::env::var_os("AUTOROUTE_DEBUG").is_some() {
                        eprintln!(
                            "debug: net {n} failed leg to pad {} ({} tree cells, {} target cells)",
                            self.board.pads[pads[pad_local]].id,
                            tree.len(),
                            target_cells.len()
                        );
                    }
                    tree_centers.push(centers[pad_local]);
                }
            }
        }
        for leg in legs {
            self.commit_leg(n, leg);
        }
        self.states[n].routed = ok;
    }

    /// Commit one leg: record wire centerlines and via barrels, and stamp the
    /// wire-entrant, via-entrant, and soft-margin fields.
    /// Bounding window (inflated by `margin` cells) around two cell sets.
    fn search_window(
        &self,
        a: &[u32],
        b: &[u32],
        margin: usize,
    ) -> Option<(usize, usize, usize, usize)> {
        let mut x0 = usize::MAX;
        let mut x1 = 0usize;
        let mut y0 = usize::MAX;
        let mut y1 = 0usize;
        for &c in a.iter().chain(b.iter()) {
            let (_, x, y) = self.grid.decompose(c);
            x0 = x0.min(x);
            x1 = x1.max(x);
            y0 = y0.min(y);
            y1 = y1.max(y);
        }
        if x0 == usize::MAX {
            return None;
        }
        let wx0 = x0.saturating_sub(margin);
        let wy0 = y0.saturating_sub(margin);
        let wx1 = (x1 + margin).min(self.grid.nx - 1);
        let wy1 = (y1 + margin).min(self.grid.ny - 1);
        // A window covering nearly the whole grid buys nothing.
        if (wx1 - wx0) * (wy1 - wy0) * 2 > self.grid.nx * self.grid.ny {
            return None;
        }
        Some((wx0, wx1, wy0, wy1))
    }

    fn commit_leg(&mut self, n: usize, leg: Leg) {
        let path: &[u32] = &leg.cells;
        let nb = self.grid.buckets.len();
        let cd = &self.discs[self.board.nets[n].class.min(self.classes.len() - 1)];
        let grid = &self.grid;
        let net_terms = &self.net_terms[n];

        // Classify path cells once (bucket-independent).
        let diag = |a: u32, b: u32| -> bool {
            let (la, xa, ya) = grid.decompose(a);
            let (lb, xb, yb) = grid.decompose(b);
            la == lb && xa != xb && ya != yb
        };
        let mut centerline: Vec<u32> = Vec::new();
        let mut via_centers: Vec<u32> = Vec::new();
        // (kind, l, x, y): kind 0 = wire orthogonal, 1 = wire diagonal, 2 = via.
        let mut cells_info: Vec<(u8, usize, usize, usize)> = Vec::new();
        for (i, &c) in path.iter().enumerate() {
            let (l, x, y) = grid.decompose(c);
            let is_via = (i > 0 && layer_change(grid, path[i - 1], c))
                || (i + 1 < path.len() && layer_change(grid, c, path[i + 1]));
            if is_via {
                for ll in 0..grid.nl {
                    via_centers.push(grid.idx(ll, x, y));
                }
                cells_info.push((2, l, x, y));
            } else {
                // Inside our own pad copper the hard block already keeps
                // foreign copper at pad clearance; wire-to-wire spacing does
                // not apply. Stamping or conflict-testing here would create
                // unresolvable false conflicts with wires legally skimming
                // the pad.
                if net_terms.binary_search(&c).is_ok() {
                    continue;
                }
                centerline.push(c);
                let is_diag = (i > 0 && diag(path[i - 1], c))
                    || (i + 1 < path.len() && diag(c, path[i + 1]));
                cells_info.push((if is_diag { 1 } else { 0 }, l, x, y));
            }
        }

        // Build the per-bucket footprint lists in parallel.
        let (nx_max, ny_max) = (grid.nx, grid.ny);
        let built: Vec<(Vec<u32>, Vec<u32>, Vec<u32>, Vec<u32>)> = (0..nb)
            .into_par_iter()
            .map(|b| {
                let mut st: Vec<u32> = Vec::new();
                let mut vs: Vec<u32> = Vec::new();
                let mut au: Vec<u32> = Vec::new();
                let mut vt: Vec<u32> = Vec::new();
                // Push whole contiguous cell runs per disc row.
                let push_disc = |list: &mut Vec<u32>, ll: usize, x: usize, y: usize, d: &[(i32, i32, i32)]| {
                    for &(dy, sx0, sx1) in d {
                        let ny = y as i32 + dy;
                        if ny < 0 || ny >= ny_max as i32 {
                            continue;
                        }
                        let a = (x as i32 + sx0).max(0);
                        let b = (x as i32 + sx1).min(nx_max as i32 - 1);
                        if b < a {
                            continue;
                        }
                        let base = grid.idx(ll, 0, ny as usize);
                        list.extend(base + a as u32..=base + b as u32);
                    }
                };
                for &(kind, l, x, y) in &cells_info {
                    if kind == 2 {
                        for ll in 0..grid.nl {
                            push_disc(&mut st, ll, x, y, &cd.via_usage[b]);
                            push_disc(&mut vs, ll, x, y, &cd.via_vusage[b]);
                            push_disc(&mut au, ll, x, y, &cd.via_aura[b]);
                            push_disc(&mut vt, ll, x, y, &cd.via_tight[b]);
                        }
                    } else {
                        let vd = if kind == 1 { &cd.wire_vusage_diag[b] } else { &cd.wire_vusage[b] };
                        push_disc(&mut st, l, x, y, &cd.wire_usage[b]);
                        push_disc(&mut vs, l, x, y, vd);
                        push_disc(&mut au, l, x, y, &cd.wire_aura[b]);
                    }
                }
                (st, vs, au, vt)
            })
            .collect();
        let mut stamp: Vec<Vec<u32>> = Vec::with_capacity(nb);
        let mut vstamp: Vec<Vec<u32>> = Vec::with_capacity(nb);
        let mut aura: Vec<Vec<u32>> = Vec::with_capacity(nb);
        let mut vtight: Vec<Vec<u32>> = Vec::with_capacity(nb);
        for (st, vs, au, vt) in built {
            stamp.push(st);
            vstamp.push(vs);
            aura.push(au);
            vtight.push(vt);
        }
        centerline.sort_unstable();
        centerline.dedup();
        via_centers.sort_unstable();
        via_centers.dedup();
        // Merge with what this net already stamped (multi-leg nets).
        let ncells = self.grid.ncells;
        let st = &mut self.states[n];
        for b in 0..nb {
            for (new_list, old_list, field) in [
                (&mut stamp[b], &mut st.stamp[b], &mut self.usage),
                (&mut vstamp[b], &mut st.vstamp[b], &mut self.vusage),
                (&mut aura[b], &mut st.aura_stamp[b], &mut self.aura),
                (&mut vtight[b], &mut st.vt_stamp[b], &mut self.via_tight),
            ] {
                new_list.sort_unstable();
                new_list.dedup();
                for &c in new_list.iter() {
                    if old_list.binary_search(&c).is_err() {
                        field[b * ncells + c as usize] += 1;
                    }
                }
                *old_list = merge_sorted(old_list, new_list);
            }
        }
        st.centerline = merge_sorted(&st.centerline, &centerline);
        st.via_centers = merge_sorted(&st.via_centers, &via_centers);
        st.legs.push(leg);
    }

    /// Multi-source, multi-target A* for net `net`.
    fn astar(
        &mut self,
        net: i32,
        bucket: usize,
        sources: &[u32],
        targets: &[u32],
        window: Option<(usize, usize, usize, usize)>,
    ) -> Option<Vec<u32>> {
        let grid = &self.grid;
        let bo = bucket * grid.ncells;
        let s = &mut self.search;
        s.cur_gen = s.cur_gen.wrapping_add(1);
        s.cur_tgen = s.cur_tgen.wrapping_add(1);
        let pitch = grid.pitch;
        let diag = pitch * std::f64::consts::SQRT_2;

        // Target bbox (in cell coords) for the heuristic.
        let mut txa = i64::MAX;
        let mut txb = i64::MIN;
        let mut tya = i64::MAX;
        let mut tyb = i64::MIN;
        let mut any_target = false;
        for &t in targets {
            if !grid.passable(bucket, t, net) {
                continue;
            }
            s.tgen[t as usize] = s.cur_tgen;
            let (_, x, y) = grid.decompose(t);
            txa = txa.min(x as i64);
            txb = txb.max(x as i64);
            tya = tya.min(y as i64);
            tyb = tyb.max(y as i64);
            any_target = true;
        }
        if !any_target {
            return None;
        }
        let hfn = |x: usize, y: usize| -> f64 {
            let dx = if (x as i64) < txa { txa - x as i64 } else if (x as i64) > txb { x as i64 - txb } else { 0 };
            let dy = if (y as i64) < tya { tya - y as i64 } else if (y as i64) > tyb { y as i64 - tyb } else { 0 };
            let (lo, hi) = if dx < dy { (dx, dy) } else { (dy, dx) };
            lo as f64 * diag + (hi - lo) as f64 * pitch
        };

        let mut heap: BinaryHeap<HeapEntry> = BinaryHeap::new();
        for &c in sources {
            if !grid.passable(bucket, c, net) {
                continue;
            }
            if s.tgen[c as usize] == s.cur_tgen {
                return Some(vec![c]); // source already touches target
            }
            s.g[c as usize] = 0.0;
            s.gen[c as usize] = s.cur_gen;
            s.came[c as usize] = NO_CAME;
            let (_, x, y) = grid.decompose(c);
            heap.push(HeapEntry { f: hfn(x, y), c });
        }
        if heap.is_empty() {
            return None;
        }

        const DIRS: [(i32, i32); 8] = [
            (1, 0), (-1, 0), (0, 1), (0, -1),
            (1, 1), (1, -1), (-1, 1), (-1, -1),
        ];

        while let Some(HeapEntry { f, c }) = heap.pop() {
            let gc = s.g[c as usize] as f64;
            let (l, x, y) = grid.decompose(c);
            // Stale-entry check. g is stored as f32, so allow generous slack:
            // an f32 ulp at board-size magnitudes is ~0.002 um and would
            // otherwise discard valid entries and cut off whole regions.
            if f > gc + hfn(x, y) + 0.5 {
                continue;
            }
            if s.tgen[c as usize] == s.cur_tgen {
                // Reconstruct.
                let mut path = vec![c];
                let mut cur = c;
                while s.came[cur as usize] != NO_CAME {
                    cur = s.came[cur as usize];
                    path.push(cur);
                }
                path.reverse();
                return Some(path);
            }

            // Direction we arrived from (for the turn penalty).
            let arr_dir: Option<(i32, i32)> = {
                let p = s.came[c as usize];
                if p == NO_CAME {
                    None
                } else {
                    let (pl, px, py) = grid.decompose(p);
                    if pl != l {
                        None
                    } else {
                        Some((x as i32 - px as i32, y as i32 - py as i32))
                    }
                }
            };

            // Planar moves.
            for &(dx, dy) in &DIRS {
                let nx = x as i32 + dx;
                let ny = y as i32 + dy;
                if nx < 0 || ny < 0 || nx as usize >= grid.nx || ny as usize >= grid.ny {
                    continue;
                }
                let (nxu, nyu) = (nx as usize, ny as usize);
                if let Some((wx0, wx1, wy0, wy1)) = window {
                    if nxu < wx0 || nxu > wx1 || nyu < wy0 || nyu > wy1 {
                        continue;
                    }
                }
                let ncell = grid.idx(l, nxu, nyu);
                if !grid.passable(bucket, ncell, net) {
                    continue;
                }
                if dx != 0 && dy != 0 {
                    // A 45-degree segment can dip closer to copper than its
                    // endpoints: forbid diagonals in the tight band around
                    // pads/boundary and near committed vias.
                    if grid.is_tight(bucket, c)
                        || grid.is_tight(bucket, ncell)
                        || self.via_tight[bo + c as usize] > 0
                        || self.via_tight[bo + ncell as usize] > 0
                    {
                        continue;
                    }
                    // No corner cutting through blocked cells.
                    let a = grid.idx(l, nxu, y);
                    let b = grid.idx(l, x, nyu);
                    if !grid.passable(bucket, a, net) || !grid.passable(bucket, b, net) {
                        continue;
                    }
                }
                let step = if dx != 0 && dy != 0 { diag } else { pitch };
                // Preferred-direction penalty for this layer (diagonals pay half).
                let (hp, vp) = self.dir_pen[l];
                let dpen = if dx != 0 && dy != 0 {
                    (hp + vp) * 0.5
                } else if dx != 0 {
                    hp
                } else {
                    vp
                };
                // Differential-pair coupling: discount cells near the routed
                // partner (never below a fraction of the step, so costs stay
                // positive) and mildly penalize leaving the corridor.
                let a = self.attract[ncell as usize] as f64;
                let bonus = (a * self.attract_w).min(step * 0.8);
                let corridor = if self.attract_on && a == 0.0 { 1.2 * pitch } else { 0.0 };
                let mut ng = gc
                    + step
                    + dpen
                    - bonus
                    + corridor
                    + self.hist[ncell as usize] as f64
                    + self.usage[bo + ncell as usize] as f64 * self.p_fac
                    + self.aura[bo + ncell as usize] as f64 * self.aura_w;
                if let Some(ad) = arr_dir {
                    if ad != (dx, dy) {
                        ng += self.cfg.turn_penalty;
                    }
                }
                if s.gen[ncell as usize] != s.cur_gen || (ng as f32) < s.g[ncell as usize] {
                    s.gen[ncell as usize] = s.cur_gen;
                    s.g[ncell as usize] = ng as f32;
                    s.came[ncell as usize] = c;
                    heap.push(HeapEntry { f: ng + hfn(nxu, nyu), c: ncell });
                }
            }

            // Via move: the barrel needs via-radius clearance on EVERY layer
            // (via_ok), but wire-margin passability only where the wire
            // actually runs — the entry layer (already true for `c`) and the
            // exit layer (checked per target below). Demanding wire margins on
            // intermediate layers would forbid vias in the tight under-IC
            // moats where fat nets in particular need them.
            if grid.nl > 1 {
                let all_ok = (0..grid.nl).all(|ll| grid.via_ok(bucket, grid.idx(ll, x, y), net));
                if all_ok {
                    // A committed via occupies the whole column (and a wide
                    // footprint), so price congestion across every layer here —
                    // otherwise the search is blind to most via conflicts.
                    let mut col_penalty = 0.0;
                    for ll in 0..grid.nl {
                        let cc = grid.idx(ll, x, y) as usize;
                        col_penalty += self.hist[cc] as f64
                            + self.vusage[bo + cc] as f64 * self.p_fac
                            + self.aura[bo + cc] as f64 * self.aura_w;
                    }
                    for nl in 0..grid.nl {
                        if nl == l {
                            continue;
                        }
                        let ncell = grid.idx(nl, x, y);
                        // The wire continues on the exit layer: wire margins
                        // must hold there.
                        if !grid.passable(bucket, ncell, net) {
                            continue;
                        }
                        let ng = gc + self.cfg.via_cost + col_penalty;
                        if s.gen[ncell as usize] != s.cur_gen || (ng as f32) < s.g[ncell as usize] {
                            s.gen[ncell as usize] = s.cur_gen;
                            s.g[ncell as usize] = ng as f32;
                            s.came[ncell as usize] = c;
                            heap.push(HeapEntry { f: ng + hfn(x, y), c: ncell });
                        }
                    }
                }
            }
        }
        None
    }

    /// Convert a net's raw cell legs into polylines and via positions.
    fn extract_geometry(&self, n: usize, out: &mut RoutedNet) {
        out.segs.clear();
        out.vias.clear();
        out.failed = !self.states[n].routed;
        let width = self.classes[self.board.nets[n].class.min(self.classes.len() - 1)].width;
        for leg in &self.states[n].legs {
            let cells = &leg.cells;
            let inside = |pad: usize, c: u32| -> bool {
                self.board.pads[pad]
                    .shape
                    .signed_dist(self.grid.cell_center(c))
                    < 0.0
            };
            let mut i = 0;
            while i < cells.len() {
                let (l, _, _) = self.grid.decompose(cells[i]);
                // Collect the run of cells on this layer.
                let mut j = i;
                while j + 1 < cells.len() {
                    let (l2, _, _) = self.grid.decompose(cells[j + 1]);
                    if l2 != l {
                        break;
                    }
                    j += 1;
                }
                // Terminal runs: drop the in-pad meander and connect straight
                // to the exact pad center instead. The replacement segment
                // stays inside the pad copper plus the pad's own clearance
                // zone, so it cannot approach foreign copper; an off-center
                // grid entry into a narrow pad would otherwise poke wire
                // copper sideways out of the pad.
                let mut a = i;
                let mut b = j;
                let (mut snap_front, mut snap_back) = (None, None);
                if i == 0 {
                    if let Some(sp) = leg.start_pad {
                        while a < b && inside(sp, cells[a]) {
                            a += 1;
                        }
                        snap_front = Some(self.board.pads[sp].shape.center());
                    }
                }
                if j + 1 == cells.len() {
                    if let Some(ep) = leg.end_pad {
                        while b > a && inside(ep, cells[b]) {
                            b -= 1;
                        }
                        snap_back = Some(self.board.pads[ep].shape.center());
                    }
                }
                let mut pts = simplify(&self.grid, &cells[a..=b]);
                if let Some(c) = snap_front {
                    if pts.first().map_or(true, |p| dist2(*p, c) > 1e-9) {
                        pts.insert(0, c);
                    }
                }
                if let Some(c) = snap_back {
                    if pts.last().map_or(true, |p| dist2(*p, c) > 1e-9) {
                        pts.push(c);
                    }
                }
                if pts.len() >= 2 {
                    // Neck-down: a track wider than its pad would overhang
                    // sideways into neighboring fine-pitch pins; enter narrow
                    // pads at the pad's own width and widen once clear.
                    let front_pad = if i == 0 { leg.start_pad } else { None };
                    let back_pad = if j + 1 == cells.len() { leg.end_pad } else { None };
                    emit_necked(
                        self.board,
                        &mut out.segs,
                        l,
                        width,
                        pts,
                        front_pad,
                        back_pad,
                    );
                }
                if j + 1 < cells.len() {
                    out.vias.push(self.grid.cell_center(cells[j]));
                }
                i = j + 1;
            }
        }
        // Dedup vias (same location reached by multiple legs).
        out.vias.sort_by(|a, b| a[0].total_cmp(&b[0]).then(a[1].total_cmp(&b[1])));
        out.vias.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-6 && (a[1] - b[1]).abs() < 1e-6);
        out.length = out
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
}

fn dist2(a: P, b: P) -> f64 {
    (a[0] - b[0]).powi(2) + (a[1] - b[1]).powi(2)
}

/// Push a run's polyline as up to three segments, necking the ends down to
/// the pad width where the net's track is wider than its terminal pad.
fn emit_necked(
    board: &Board,
    segs: &mut Vec<RouteSeg>,
    layer: usize,
    width: f64,
    pts: Vec<P>,
    front_pad: Option<usize>,
    back_pad: Option<usize>,
) {
    // Width that fits a pad: its bbox's short side (diameter for circles).
    let neck_width = |pi: usize| -> f64 {
        let (lo, hi) = board.pads[pi].shape.bbox();
        (hi[0] - lo[0]).min(hi[1] - lo[1]).max(100.0)
    };
    // Length of the necked section, measured along the path from the pad:
    // enough to clear the pad and its immediate neighborhood.
    let neck_reach = |pi: usize| -> f64 {
        let (lo, hi) = board.pads[pi].shape.bbox();
        let half_diag = 0.5 * ((hi[0] - lo[0]).powi(2) + (hi[1] - lo[1]).powi(2)).sqrt();
        half_diag + width
    };

    let mut pts = pts;
    let mut start = 0usize;
    let mut end_back = 0usize; // vertices consumed by the back neck
    if let Some(pi) = front_pad {
        let nw = neck_width(pi);
        if width > nw + 1.0 {
            let cut = split_at_distance(&mut pts, true, neck_reach(pi));
            if cut >= 1 {
                segs.push(RouteSeg {
                    layer,
                    width: nw.min(width),
                    pts: pts[..=cut].to_vec(),
                });
                start = cut;
            }
        }
    }
    if let Some(pi) = back_pad {
        let nw = neck_width(pi);
        if width > nw + 1.0 {
            let cut = split_at_distance(&mut pts, false, neck_reach(pi));
            if cut > start {
                segs.push(RouteSeg {
                    layer,
                    width: nw.min(width),
                    pts: pts[cut..].to_vec(),
                });
                end_back = pts.len() - 1 - cut;
            }
        }
    }
    let end = pts.len() - end_back;
    if end > start + 1 {
        segs.push(RouteSeg {
            layer,
            width,
            pts: pts[start..end].to_vec(),
        });
    }
}

/// Split a polyline at `dist` along its length from the front (or back),
/// inserting an interpolated vertex if needed. Returns the split vertex index.
fn split_at_distance(pts: &mut Vec<P>, from_front: bool, dist: f64) -> usize {
    if !from_front {
        pts.reverse();
        let k = split_at_distance(pts, true, dist);
        pts.reverse();
        return pts.len() - 1 - k;
    }
    let mut acc = 0.0;
    let mut k = 0;
    while k + 1 < pts.len() {
        let seglen = dist2(pts[k], pts[k + 1]).sqrt();
        if acc + seglen >= dist {
            let t = ((dist - acc) / seglen).clamp(0.0, 1.0);
            if t <= 1e-6 {
                return k;
            }
            if t >= 1.0 - 1e-6 {
                return k + 1;
            }
            let p = [
                pts[k][0] + (pts[k + 1][0] - pts[k][0]) * t,
                pts[k][1] + (pts[k + 1][1] - pts[k][1]) * t,
            ];
            pts.insert(k + 1, p);
            return k + 1;
        }
        acc += seglen;
        k += 1;
    }
    pts.len() - 1
}

/// Simple glob: "*" matches any run of characters.
pub fn glob_match(pattern: &str, name: &str) -> bool {
    fn rec(p: &[u8], n: &[u8]) -> bool {
        match p.first() {
            None => n.is_empty(),
            Some(b'*') => (0..=n.len()).any(|k| rec(&p[1..], &n[k..])),
            Some(&c) => n.first() == Some(&c) && rec(&p[1..], &n[1..]),
        }
    }
    rec(pattern.as_bytes(), name.as_bytes())
}

/// Detect differential pairs by KiCad naming conventions: names differing
/// only by a trailing +/- or _P/_N. Returns partner index per net.
pub fn detect_pairs(names: &[String]) -> Vec<Option<usize>> {
    use std::collections::HashMap;
    let mut partner = vec![None; names.len()];
    let mut by_base: HashMap<(String, bool), usize> = HashMap::new();
    let base_of = |name: &str| -> Option<(String, bool)> {
        if let Some(b) = name.strip_suffix('+') {
            Some((b.to_string(), true))
        } else if let Some(b) = name.strip_suffix('-') {
            Some((b.to_string(), false))
        } else if let Some(b) = name.strip_suffix("_P") {
            Some((b.to_string(), true))
        } else if let Some(b) = name.strip_suffix("_N") {
            Some((b.to_string(), false))
        } else {
            None
        }
    };
    for (i, name) in names.iter().enumerate() {
        if let Some((base, pos)) = base_of(name) {
            if let Some(&j) = by_base.get(&(base.clone(), !pos)) {
                partner[i] = Some(j);
                partner[j] = Some(i);
            } else {
                by_base.insert((base, pos), i);
            }
        }
    }
    partner
}

fn merge_sorted(a: &[u32], b: &[u32]) -> Vec<u32> {
    let mut v = Vec::with_capacity(a.len() + b.len());
    v.extend_from_slice(a);
    v.extend_from_slice(b);
    v.sort_unstable();
    v.dedup();
    v
}

fn layer_change(grid: &Grid, a: u32, b: u32) -> bool {
    let (la, _, _) = grid.decompose(a);
    let (lb, _, _) = grid.decompose(b);
    la != lb
}

/// Drop collinear intermediate points from a cell run.
fn simplify(grid: &Grid, cells: &[u32]) -> Vec<P> {
    let mut pts: Vec<P> = Vec::new();
    let mut last_dir: Option<(i64, i64)> = None;
    let mut last_xy: Option<(i64, i64)> = None;
    for &c in cells {
        let (_, x, y) = grid.decompose(c);
        let xy = (x as i64, y as i64);
        if let Some(prev) = last_xy {
            let dir = (xy.0 - prev.0, xy.1 - prev.1);
            if dir == (0, 0) {
                continue;
            }
            if last_dir == Some(dir) {
                // Extend the current run: replace the last point.
                let p = grid.cell_center(c);
                *pts.last_mut().unwrap() = p;
            } else {
                pts.push(grid.cell_center(c));
                last_dir = Some(dir);
            }
        } else {
            pts.push(grid.cell_center(c));
        }
        last_xy = Some(xy);
    }
    pts
}

/// Per-layer (h-move, v-move) penalties from the layer-direction config.
fn layer_dir_penalties(cfg: &RouterConfig, nl: usize, pitch: f64) -> Vec<(f64, f64)> {
    let pen = 0.35 * pitch;
    let dirs: Vec<char> = match &cfg.layer_dirs {
        Some(s) => s.chars().take(nl).collect(),
        None => (0..nl)
            .map(|l| {
                // Outer layers unconstrained; inner layers alternate h/v.
                if nl > 2 && l > 0 && l + 1 < nl {
                    if (l - 1) % 2 == 0 { 'h' } else { 'v' }
                } else {
                    'a'
                }
            })
            .collect(),
    };
    (0..nl)
        .map(|l| match dirs.get(l).copied().unwrap_or('a') {
            'h' | 'H' => (0.0, pen), // horizontal preferred: penalize vertical
            'v' | 'V' => (pen, 0.0),
            _ => (0.0, 0.0),
        })
        .collect()
}

/// Row spans of a filled disc: (dy, xmin, xmax) per row — lets stamping push
/// whole contiguous cell runs at once.
fn disc_spans(r: f64) -> Vec<(i32, i32, i32)> {
    let ri = r.ceil() as i32;
    let r2 = r * r;
    let mut v = Vec::new();
    for dy in -ri..=ri {
        let rem = r2 - (dy * dy) as f64 + 1e-9;
        if rem < 0.0 {
            continue;
        }
        let dx = rem.sqrt().floor() as i32;
        v.push((dy, -dx, dx));
    }
    v
}

/// Integer offsets of a filled disc of radius `r` (in cells).
fn disc_offsets(r: f64) -> Vec<(i32, i32)> {
    let ri = r.ceil() as i32;
    let r2 = r * r;
    let mut v = Vec::new();
    for dy in -ri..=ri {
        for dx in -ri..=ri {
            if (dx * dx + dy * dy) as f64 <= r2 + 1e-9 {
                v.push((dx, dy));
            }
        }
    }
    v
}

/// Convenience wrapper: build a router and run it to completion.
pub fn route_board(
    board: &Board,
    cfg: RouterConfig,
    cancel: &AtomicBool,
    progress: &mut dyn FnMut(&Progress),
) -> RouteResult {
    Router::new(board, cfg).route(cancel, progress)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::*;
    use std::sync::atomic::AtomicBool;

    fn pad(id: &str, x: f64, y: f64, net: usize) -> Pad {
        Pad {
            id: id.into(),
            shape: PadShape::Circle { center: [x, y], diameter: 1000.0 },
            layers: vec![0, 1],
            net: Some(net),
        }
    }

    fn board_two_nets() -> Board {
        // Two nets whose straight-line paths cross: forces negotiation or a via.
        Board {
            name: "t".into(),
            layers: vec!["F.Cu".into(), "B.Cu".into()],
            boundary: vec![[0.0, 0.0], [20000.0, 0.0], [20000.0, 20000.0], [0.0, 20000.0]],
            pads: vec![
                pad("A-1", 3000.0, 3000.0, 0),
                pad("A-2", 17000.0, 17000.0, 0),
                pad("B-1", 17000.0, 3000.0, 1),
                pad("B-2", 3000.0, 17000.0, 1),
            ],
            nets: vec![
                Net { name: "N0".into(), pads: vec![0, 1], class: 0, fixed: false },
                Net { name: "N1".into(), pads: vec![2, 3], class: 0, fixed: false },
            ],
            rules: Rules::default(),
            classes: Vec::new(),
        }
    }

    #[test]
    fn test_route_crossing_nets() {
        let board = board_two_nets();
        let cancel = AtomicBool::new(false);
        let result = route_board(&board, RouterConfig::default(), &cancel, &mut |_| {});
        assert!(result.complete, "expected complete routing, got {result:?}");
        assert_eq!(result.failed_nets, 0);
        assert_eq!(result.conflict_nets, 0);
        for rn in &result.nets {
            assert!(!rn.segs.is_empty(), "net {} has no segments", rn.net);
        }
    }

    #[test]
    fn test_cross_class_clearance() {
        // A fat high-clearance net and a thin default net whose straight
        // paths cross: the result must respect the pair clearance (max of the
        // two classes) or report a conflict — never emit silent violations.
        let mut board = board_two_nets();
        board.classes = vec![
            NetClass {
                name: "default".into(),
                width: 250.0,
                clearance: 200.0,
                via_diameter: 800.0,
                via_name: "Via[0-1]_800:400_um".into(),
            },
            NetClass {
                name: "HV".into(),
                width: 2000.0,
                clearance: 800.0,
                via_diameter: 800.0,
                via_name: "Via[0-1]_800:400_um".into(),
            },
        ];
        board.nets[1].class = 1;
        let cancel = AtomicBool::new(false);
        let result = route_board(&board, RouterConfig::default(), &cancel, &mut |_| {});
        // The HV net's tracks come out 2000 wide, except pad-entry necks
        // (its pads are only 1000 wide, so entries neck down to fit).
        assert!(result.nets[1]
            .segs
            .iter()
            .any(|s| (s.width - 2000.0).abs() < 1e-9));
        assert!(result.nets[1]
            .segs
            .iter()
            .all(|s| (s.width - 2000.0).abs() < 1e-9 || (s.width - 1000.0).abs() < 1e-9));
        let mut v = crate::drc::check(&board, &result, 5.0);
        v.extend(crate::drc::check_pads(&board, &result, 5.0));
        if result.complete {
            assert!(
                v.is_empty(),
                "complete result must be DRC-clean, got {} violations: {:?}",
                v.len(),
                &v[..v.len().min(3)]
            );
        }
    }

    #[test]
    fn test_glob_and_pairs() {
        assert!(glob_match("ETH_*", "ETH_TD+"));
        assert!(glob_match("*GND*", "RX GND"));
        assert!(!glob_match("ETH_*", "SPI_CLK"));
        let names: Vec<String> = ["/ETH_TD+", "/ETH_TD-", "/CLK", "/A_P", "/A_N"]
            .iter()
            .map(|s| s.to_string())
            .collect();
        let pairs = detect_pairs(&names);
        assert_eq!(pairs[0], Some(1));
        assert_eq!(pairs[1], Some(0));
        assert_eq!(pairs[2], None);
        assert_eq!(pairs[3], Some(4));
    }

    #[test]
    fn test_only_nets_filter() {
        let board = board_two_nets();
        let cancel = AtomicBool::new(false);
        let cfg = RouterConfig {
            only_nets: vec!["N0".into()],
            ..Default::default()
        };
        let result = route_board(&board, cfg, &cancel, &mut |_| {});
        assert!(!result.nets[0].segs.is_empty());
        assert!(result.nets[1].segs.is_empty());
    }

    #[test]
    fn test_multi_pad_net() {
        // A 4-pad ground-style net plus a crossing 2-pad net.
        let mut board = board_two_nets();
        board.pads.push(pad("C1-1", 3000.0, 10000.0, 0));
        board.pads.push(pad("C2-1", 17000.0, 10000.0, 0));
        board.nets[0].pads = vec![0, 1, 4, 5];
        let cancel = AtomicBool::new(false);
        let result = route_board(&board, RouterConfig::default(), &cancel, &mut |_| {});
        assert!(result.complete, "multi-pad board should route: {result:?}");
        assert!(!result.nets[0].segs.is_empty());
    }

    #[test]
    fn test_single_net_straight() {
        let mut board = board_two_nets();
        board.nets.truncate(1);
        board.pads.truncate(2);
        let cancel = AtomicBool::new(false);
        let result = route_board(&board, RouterConfig::default(), &cancel, &mut |_| {});
        assert!(result.complete);
        // A straight diagonal route should not need vias.
        assert!(result.nets[0].vias.is_empty());
    }
}
