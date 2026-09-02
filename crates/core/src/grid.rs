//! Uniform routing grid: rasterizes the board (boundary + pads) into per-layer
//! cell ownership, and records which cells belong to each pad (terminals).
//!
//! Net classes make clearances pair-dependent (width/2 + width/2 + max of the
//! two clearances), so static obstacles are rasterized once per "bucket" — a
//! distinct (width, clearance, via diameter) combination among the classes.
//! Each net reads only its own bucket's fields.

use crate::board::{pair_clearance, Board, NetClass, Pad};
use crate::geometry::{poly_signed_dist, P};
use rayon::prelude::*;

/// Cell ownership markers in `Grid::block`.
pub const FREE: i32 = -1;
/// Blocked for every net (outside boundary, or contested by multiple nets).
pub const HARD: i32 = -2;

#[derive(Debug, Clone, PartialEq)]
pub struct BucketParams {
    pub width: f64,
    pub clearance: f64,
    pub via_diameter: f64,
}

pub struct Grid {
    pub nl: usize,
    pub nx: usize,
    pub ny: usize,
    /// Cell pitch, um.
    pub pitch: f64,
    /// Center of cell (0, 0), um.
    pub x0: f64,
    pub y0: f64,
    /// Cells per bucket-plane (nl * nx * ny).
    pub ncells: usize,
    /// Distinct rule combinations; every net maps to one via `class_bucket`.
    pub buckets: Vec<BucketParams>,
    /// Class index -> bucket index.
    pub class_bucket: Vec<usize>,
    /// Bucket index -> representative class index.
    pub bucket_class: Vec<usize>,
    /// Wire ownership per (bucket, layer, y, x): FREE, HARD, or a net index
    /// (passable only for that net). Margins are bucket-specific.
    block: Vec<i32>,
    /// Cells within half a diagonal step of the exact clearance margin for
    /// that bucket. Passable, but diagonal moves are forbidden: a 45-degree
    /// segment between two legal cell centers can pass up to that much closer
    /// to copper than the centers themselves, while orthogonal steps cannot.
    tight: Vec<bool>,
    /// Via placement ownership per bucket, inflated by that bucket's via
    /// radius: a via barrel needs `via_r + clearance` to a foreign pad edge.
    via_block: Vec<i32>,
}

/// For each pad (parallel to `Board::pads`): the grid cells inside the pad copper.
pub type PadCells = Vec<Vec<u32>>;

#[inline]
fn merge_owner(cur: i32, owner: i32) -> i32 {
    match cur {
        HARD => HARD,
        FREE => owner,
        c if c == owner => c,
        // Two different nets both claim this cell: nobody may use it.
        _ => HARD,
    }
}

/// Deduplicate class rule combinations into buckets. With pair-clearance
/// overrides in play, classes with identical parameters may still differ in
/// pair rules, so dedup is disabled and buckets map 1:1 to classes.
pub fn make_buckets(
    classes: &[NetClass],
    dedup: bool,
) -> (Vec<BucketParams>, Vec<usize>, Vec<usize>) {
    let mut buckets: Vec<BucketParams> = Vec::new();
    let mut class_bucket = Vec::with_capacity(classes.len());
    let mut bucket_class: Vec<usize> = Vec::new();
    for (ci, cl) in classes.iter().enumerate() {
        let p = BucketParams {
            width: cl.width,
            clearance: cl.clearance,
            via_diameter: cl.via_diameter,
        };
        let b = match buckets.iter().position(|q| dedup && *q == p) {
            Some(b) => b,
            None => {
                buckets.push(p);
                bucket_class.push(ci);
                buckets.len() - 1
            }
        };
        class_bucket.push(b);
    }
    (buckets, class_bucket, bucket_class)
}

impl Grid {
    #[inline]
    pub fn idx(&self, l: usize, x: usize, y: usize) -> u32 {
        ((l * self.ny + y) * self.nx + x) as u32
    }

    #[inline]
    pub fn decompose(&self, c: u32) -> (usize, usize, usize) {
        let c = c as usize;
        let x = c % self.nx;
        let rest = c / self.nx;
        (rest / self.ny, x, rest % self.ny)
    }

    #[inline]
    pub fn cell_center(&self, c: u32) -> P {
        let (_, x, y) = self.decompose(c);
        [self.x0 + x as f64 * self.pitch, self.y0 + y as f64 * self.pitch]
    }

    /// Nearest cell (x, y) for a point, clamped into the grid.
    pub fn cell_of(&self, p: P) -> (usize, usize) {
        let x = ((p[0] - self.x0) / self.pitch).round().max(0.0) as usize;
        let y = ((p[1] - self.y0) / self.pitch).round().max(0.0) as usize;
        (x.min(self.nx - 1), y.min(self.ny - 1))
    }

    #[inline]
    fn bi(&self, b: usize, c: u32) -> usize {
        b * self.ncells + c as usize
    }

    #[inline]
    pub fn passable(&self, b: usize, c: u32, net: i32) -> bool {
        let v = self.block[self.bi(b, c)];
        v == FREE || v == net
    }

    /// May net (of bucket `b`) place a via barrel at this cell (per layer)?
    #[inline]
    pub fn via_ok(&self, b: usize, c: u32, net: i32) -> bool {
        let v = self.via_block[self.bi(b, c)];
        v == FREE || v == net
    }

    #[inline]
    pub fn is_tight(&self, b: usize, c: u32) -> bool {
        self.tight[self.bi(b, c)]
    }

    /// Rasterize a board. `pitch` in um.
    pub fn build(board: &Board, pitch: f64) -> (Grid, PadCells) {
        let classes = effective_classes(board);
        let (buckets, class_bucket, bucket_class) =
            make_buckets(&classes, board.pair_clearances.is_empty());
        let nb = buckets.len();

        let (lo, hi) = board.bbox();
        let quant = 0.5 * std::f64::consts::SQRT_2 * pitch;
        let x0 = lo[0] - pitch;
        let y0 = lo[1] - pitch;
        let nx = (((hi[0] - lo[0]) + 2.0 * pitch) / pitch).ceil() as usize + 1;
        let ny = (((hi[1] - lo[1]) + 2.0 * pitch) / pitch).ceil() as usize + 1;
        let nl = board.layers.len().max(1);
        let ncells = nl * nx * ny;

        let mut grid = Grid {
            nl,
            nx,
            ny,
            pitch,
            x0,
            y0,
            ncells,
            buckets,
            class_bucket,
            bucket_class,
            block: vec![HARD; nb * ncells],
            tight: vec![false; nb * ncells],
            // HARD by default so vias cannot land outside the boundary; the
            // boundary pass frees the interior per bucket.
            via_block: vec![HARD; nb * ncells],
        };

        // Free the interior of the boundary polygon per bucket (margins are
        // bucket-specific); the band just inside the margin is tight, and via
        // placement needs the via radius to the edge.
        if board.boundary.len() >= 3 {
            // Signed distances to the outline, computed in parallel.
            let sds: Vec<f64> = (0..nx * ny)
                .into_par_iter()
                .map(|i| {
                    let (x, y) = (i % nx, i / nx);
                    let p = [x0 + x as f64 * pitch, y0 + y as f64 * pitch];
                    poly_signed_dist(p, &board.boundary)
                })
                .collect();
            for y in 0..ny {
                for x in 0..nx {
                    let sd = sds[y * nx + x];
                    for b in 0..nb {
                        let bp = &grid.buckets[b];
                        let margin = bp.clearance + bp.width / 2.0;
                        let via_margin = bp.clearance + bp.via_diameter / 2.0;
                        if sd <= -margin {
                            let is_tight = sd > -(margin + quant);
                            for l in 0..nl {
                                let c = grid.idx(l, x, y);
                                let i = grid.bi(b, c);
                                grid.block[i] = FREE;
                                grid.tight[i] = is_tight;
                            }
                        }
                        if sd <= -via_margin {
                            for l in 0..nl {
                                let i = grid.bi(b, grid.idx(l, x, y));
                                grid.via_block[i] = FREE;
                            }
                        }
                    }
                }
            }
        } else {
            grid.block.fill(FREE);
            grid.via_block.fill(FREE);
        }

        // Rasterize pads as (net-owned or hard) obstacles per bucket. The
        // expensive part — signed distances — is sampled in parallel; the
        // grid writes are applied serially.
        let pad_meta: Vec<(f64, Vec<(f64, f64)>, f64)> = board
            .pads
            .iter()
            .map(|pad| {
                let pad_class = match pad.net {
                    Some(n) => board.nets[n].class.min(classes.len() - 1),
                    None => 0,
                };
                let pad_clearance = classes[pad_class].clearance;
                let margins: Vec<(f64, f64)> = grid
                    .buckets
                    .iter()
                    .enumerate()
                    .map(|(b, bp)| {
                        let c = pair_clearance(
                            &classes,
                            &board.pair_clearances,
                            pad_class,
                            grid.bucket_class[b],
                        );
                        (c + bp.width / 2.0, c + bp.via_diameter / 2.0)
                    })
                    .collect();
                let outer = margins
                    .iter()
                    .map(|(m, vm)| (m + quant).max(*vm))
                    .fold(0.0_f64, f64::max);
                (pad_clearance, margins, outer)
            })
            .collect();
        let geom = (pitch, x0, y0, nx, ny);
        let samples: Vec<Vec<(usize, usize, f64)>> = board
            .pads
            .par_iter()
            .zip(pad_meta.par_iter())
            .map(|(pad, (_, _, outer))| sample_pad(pad, *outer, geom))
            .collect();
        for ((pad, (_, margins, _)), cells) in
            board.pads.iter().zip(pad_meta.iter()).zip(samples.iter())
        {
            if pad.is_pour {
                continue;
            }
            let owner = match pad.net {
                Some(n) => n as i32,
                None => HARD,
            };
            let layers: Vec<usize> =
                pad.layers.iter().copied().filter(|&l| l < grid.nl).collect();
            for &(x, y, sd) in cells {
                for &l in &layers {
                    let c = grid.idx(l, x, y);
                    for (b, &(margin, via_margin)) in margins.iter().enumerate() {
                        let i = grid.bi(b, c);
                        if sd <= via_margin {
                            grid.via_block[i] = merge_owner(grid.via_block[i], owner);
                        }
                        if sd > margin + quant {
                            continue;
                        }
                        if sd > margin {
                            grid.tight[i] = true;
                            continue;
                        }
                        grid.block[i] = merge_owner(grid.block[i], owner);
                    }
                }
            }
        }

        // Collect terminal cells: cells strictly inside pad copper, forced
        // passable in the pad's own bucket for its own net even if a
        // neighbor's inflation overlapped them.
        let mut pad_cells: PadCells = vec![Vec::new(); board.pads.len()];
        for (pi, pad) in board.pads.iter().enumerate() {
            let net = match pad.net {
                Some(n) => n as i32,
                None => continue,
            };
            let nb_own = grid.class_bucket
                [board.nets[net as usize].class.min(grid.class_bucket.len() - 1)];
            let cells = &mut pad_cells[pi];
            for &(x, y, sd) in &samples[pi] {
                if sd <= 0.0 {
                    for &l in &pad.layers {
                        if l >= grid.nl {
                            continue;
                        }
                        let c = grid.idx(l, x, y);
                        if pad.is_pour {
                            // Pour interiors are terminals wherever they are
                            // already passable — never forced, never owned.
                            if grid.passable(nb_own, c, net) {
                                cells.push(c);
                            }
                            continue;
                        }
                        let i = grid.bi(nb_own, c);
                        // A terminal forced out of contested/blocked space sits
                        // inside some foreign pad's clearance margin: exits
                        // from it must be orthogonal (diagonals dip closer).
                        let prev = grid.block[i];
                        if prev != FREE && prev != net {
                            grid.tight[i] = true;
                        }
                        grid.block[i] = net;
                        cells.push(c);
                    }
                }
            }
            // Grid too coarse to land a cell inside the pad: force the nearest cell.
            if cells.is_empty() && !pad.is_pour {
                let cp = pad.shape.center();
                let (x, y) = grid.cell_of(cp);
                for &l in &pad.layers {
                    if l >= grid.nl {
                        continue;
                    }
                    let c = grid.idx(l, x, y);
                    let i = grid.bi(nb_own, c);
                    let prev = grid.block[i];
                    if prev != FREE && prev != net {
                        grid.tight[i] = true;
                    }
                    grid.block[i] = net;
                    cells.push(c);
                }
            }
        }

        (grid, pad_cells)
    }

}

/// Cells within `margin` of the pad shape, with signed distances.
/// `geom` = (pitch, x0, y0, nx, ny).
fn sample_pad(
    pad: &Pad,
    margin: f64,
    geom: (f64, f64, f64, usize, usize),
) -> Vec<(usize, usize, f64)> {
    let (pitch, x0, y0, nx, ny) = geom;
    let (lo, hi) = pad.shape.bbox();
    let xa = (((lo[0] - margin) - x0) / pitch).floor().max(0.0) as usize;
    let ya = (((lo[1] - margin) - y0) / pitch).floor().max(0.0) as usize;
    let xb = ((((hi[0] + margin) - x0) / pitch).ceil() as usize).min(nx - 1);
    let yb = ((((hi[1] + margin) - y0) / pitch).ceil() as usize).min(ny - 1);
    let mut out = Vec::new();
    for y in ya..=yb {
        for x in xa..=xb {
            let p = [x0 + x as f64 * pitch, y0 + y as f64 * pitch];
            let sd = pad.shape.signed_dist(p);
            if sd <= margin {
                out.push((x, y, sd));
            }
        }
    }
    out
}

/// The board's classes, or a synthesized default class when none are set.
pub fn effective_classes(board: &Board) -> Vec<NetClass> {
    if board.classes.is_empty() {
        vec![NetClass {
            name: "default".into(),
            width: board.rules.width,
            clearance: board.rules.clearance,
            via_diameter: board.rules.via_diameter,
            via_name: board.rules.via_name.clone(),
        }]
    } else {
        board.classes.clone()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::board::*;

    fn test_board() -> Board {
        Board {
            name: "t".into(),
            layers: vec!["F.Cu".into(), "B.Cu".into()],
            boundary: vec![[0.0, 0.0], [10000.0, 0.0], [10000.0, 10000.0], [0.0, 10000.0]],
            pads: vec![
                Pad {
                    id: "R1-1".into(),
                    shape: PadShape::Circle { center: [2000.0, 5000.0], diameter: 1000.0 },
                    layers: vec![0, 1],
                    net: Some(0),
                    is_pour: false,
                },
                Pad {
                    id: "R1-2".into(),
                    shape: PadShape::Circle { center: [8000.0, 5000.0], diameter: 1000.0 },
                    layers: vec![0, 1],
                    net: Some(0),
                    is_pour: false,
                },
            ],
            nets: vec![Net { name: "N1".into(), pads: vec![0, 1], class: 0, fixed: false }],
            rules: Rules::default(),
            classes: Vec::new(),
            pair_clearances: Vec::new(),
        }
    }

    #[test]
    fn test_build_grid() {
        let board = test_board();
        let (grid, pad_cells) = Grid::build(&board, 250.0);
        assert_eq!(grid.nl, 2);
        assert_eq!(grid.buckets.len(), 1);
        assert!(!pad_cells[0].is_empty());
        assert!(!pad_cells[1].is_empty());
        // Pad cells are owned by net 0 in its bucket.
        for &c in &pad_cells[0] {
            assert!(grid.passable(0, c, 0));
            assert!(!grid.passable(0, c, 1));
        }
        // A mid-board free cell exists.
        let (x, y) = grid.cell_of([5000.0, 2000.0]);
        assert!(grid.passable(0, grid.idx(0, x, y), 99));
        // Outside the boundary is hard-blocked.
        let (x, y) = grid.cell_of([-900.0, -900.0]);
        assert!(!grid.passable(0, grid.idx(0, x, y), 99));
    }

    #[test]
    fn test_two_buckets() {
        let mut board = test_board();
        board.classes = vec![
            NetClass {
                name: "default".into(),
                width: 250.0,
                clearance: 200.0,
                via_diameter: 800.0,
                via_name: "v".into(),
            },
            NetClass {
                name: "power".into(),
                width: 1000.0,
                clearance: 300.0,
                via_diameter: 800.0,
                via_name: "v".into(),
            },
        ];
        let (grid, _) = Grid::build(&board, 250.0);
        assert_eq!(grid.buckets.len(), 2);
        // A cell near a pad passable for the thin bucket but not the fat one:
        // margin_thin = 200+125 = 325, margin_fat = max(300,200)+500 = 800.
        let p = [2000.0 + 500.0 + 600.0, 5000.0]; // 600 from pad edge
        let (x, y) = grid.cell_of(p);
        let c = grid.idx(0, x, y);
        assert!(grid.passable(0, c, 5));
        assert!(!grid.passable(1, c, 5));
    }
}
