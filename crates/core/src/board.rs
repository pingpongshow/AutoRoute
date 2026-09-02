//! Flattened board model: everything in absolute coordinates (f64 micrometers),
//! independent of any input file format.

use crate::geometry::{self, P};

#[derive(Debug, Clone)]
pub struct Rules {
    /// Default track width, um.
    pub width: f64,
    /// Minimum copper-to-copper clearance, um.
    pub clearance: f64,
    /// Via pad diameter, um.
    pub via_diameter: f64,
    /// Via drill diameter, um.
    pub via_drill: f64,
    /// Padstack name of the via (echoed into the SES file).
    pub via_name: String,
}

impl Default for Rules {
    fn default() -> Self {
        Rules {
            width: 250.0,
            clearance: 200.0,
            via_diameter: 800.0,
            via_drill: 400.0,
            via_name: "Via[0-1]_800:400_um".to_string(),
        }
    }
}

/// A pad shape in absolute board coordinates.
#[derive(Debug, Clone)]
pub enum PadShape {
    Circle { center: P, diameter: f64 },
    Polygon { pts: Vec<P> },
    /// An open polyline with round caps/joins (Specctra "path" shape, used for ovals).
    Path { width: f64, pts: Vec<P> },
}

impl PadShape {
    /// Signed distance from `p` to the copper: negative inside, positive outside.
    pub fn signed_dist(&self, p: P) -> f64 {
        match self {
            PadShape::Circle { center, diameter } => geometry::dist(p, *center) - diameter / 2.0,
            PadShape::Polygon { pts } => geometry::poly_signed_dist(p, pts),
            PadShape::Path { width, pts } => {
                if pts.len() == 1 {
                    return geometry::dist(p, pts[0]) - width / 2.0;
                }
                let mut best = f64::INFINITY;
                for w in pts.windows(2) {
                    let d = geometry::seg_dist(p, w[0], w[1]);
                    if d < best {
                        best = d;
                    }
                }
                best - width / 2.0
            }
        }
    }

    /// Axis-aligned bounding box (min, max).
    pub fn bbox(&self) -> (P, P) {
        match self {
            PadShape::Circle { center, diameter } => {
                let r = diameter / 2.0;
                ([center[0] - r, center[1] - r], [center[0] + r, center[1] + r])
            }
            PadShape::Polygon { pts } => bbox_of(pts, 0.0),
            PadShape::Path { width, pts } => bbox_of(pts, width / 2.0),
        }
    }

    /// Representative center point of the shape.
    pub fn center(&self) -> P {
        let (lo, hi) = self.bbox();
        [(lo[0] + hi[0]) / 2.0, (lo[1] + hi[1]) / 2.0]
    }

    /// Apply mirror (x -> -x), then rotation (deg CCW), then translation.
    pub fn transform(&self, mirror_x: bool, deg: f64, at: P) -> PadShape {
        let tp = |p: P| -> P {
            let p = if mirror_x { [-p[0], p[1]] } else { p };
            let p = geometry::rotate(p, deg);
            [p[0] + at[0], p[1] + at[1]]
        };
        match self {
            PadShape::Circle { center, diameter } => PadShape::Circle {
                center: tp(*center),
                diameter: *diameter,
            },
            PadShape::Polygon { pts } => PadShape::Polygon {
                pts: pts.iter().map(|p| tp(*p)).collect(),
            },
            PadShape::Path { width, pts } => PadShape::Path {
                width: *width,
                pts: pts.iter().map(|p| tp(*p)).collect(),
            },
        }
    }
}

fn bbox_of(pts: &[P], pad: f64) -> (P, P) {
    let mut lo = [f64::INFINITY; 2];
    let mut hi = [f64::NEG_INFINITY; 2];
    for p in pts {
        for k in 0..2 {
            lo[k] = lo[k].min(p[k] - pad);
            hi[k] = hi[k].max(p[k] + pad);
        }
    }
    (lo, hi)
}

#[derive(Debug, Clone)]
pub struct Pad {
    /// "REF-PINNUM", e.g. "R1-2". Empty for keepout obstacles.
    pub id: String,
    pub shape: PadShape,
    /// Copper layer indices this pad exists on.
    pub layers: Vec<usize>,
    /// Net index into `Board::nets`, or None if unconnected (pure obstacle).
    pub net: Option<usize>,
    /// A copper pour region: a routable terminal for its net (stubs and
    /// stitching vias may end anywhere inside), but NOT an obstacle for other
    /// nets — the pour refills around foreign copper.
    pub is_pour: bool,
}

/// A net class: per-class routing rules. Class 0 is always the default class.
#[derive(Debug, Clone)]
pub struct NetClass {
    pub name: String,
    /// Track width, um.
    pub width: f64,
    /// Clearance, um. The clearance between two items is the max of their
    /// classes' clearances (KiCad semantics).
    pub clearance: f64,
    /// Via pad diameter, um.
    pub via_diameter: f64,
    /// Via padstack name (echoed into the SES file).
    pub via_name: String,
}

#[derive(Debug, Clone)]
pub struct Net {
    pub name: String,
    /// Indices into `Board::pads`.
    pub pads: Vec<usize>,
    /// Index into `Board::classes`.
    pub class: usize,
    /// Net has pre-existing routing that must be preserved: it is not
    /// (re)routed, and its copper acts as an obstacle for other nets.
    pub fixed: bool,
}

#[derive(Debug, Clone)]
pub struct Board {
    pub name: String,
    /// Signal layer names in stackup order (e.g. ["F.Cu", "B.Cu"]).
    pub layers: Vec<String>,
    /// Board outline polygon, absolute um.
    pub boundary: Vec<P>,
    pub pads: Vec<Pad>,
    pub nets: Vec<Net>,
    /// Default rules (also class 0's rules).
    pub rules: Rules,
    /// Net classes; index 0 is the default class.
    pub classes: Vec<NetClass>,
    /// Pair clearance overrides (class_a, class_b, um) from conditional
    /// design rules; effective pair clearance = max(a, b, override).
    pub pair_clearances: Vec<(usize, usize, f64)>,
}

/// Effective clearance between two classes.
pub fn pair_clearance(
    classes: &[NetClass],
    overrides: &[(usize, usize, f64)],
    a: usize,
    b: usize,
) -> f64 {
    let mut c = classes[a.min(classes.len() - 1)]
        .clearance
        .max(classes[b.min(classes.len() - 1)].clearance);
    for &(x, y, v) in overrides {
        if (x == a && y == b) || (x == b && y == a) {
            c = c.max(v);
        }
    }
    c
}

impl Board {
    /// Ensure `classes` has at least the default class derived from `rules`.
    pub fn ensure_default_class(&mut self) {
        if self.classes.is_empty() {
            self.classes.push(NetClass {
                name: "default".into(),
                width: self.rules.width,
                clearance: self.rules.clearance,
                via_diameter: self.rules.via_diameter,
                via_name: self.rules.via_name.clone(),
            });
        }
    }

    #[inline]
    pub fn net_class(&self, net: usize) -> &NetClass {
        &self.classes[self.nets[net].class.min(self.classes.len() - 1)]
    }

    /// Class of a pad's net (default class for unconnected pads).
    #[inline]
    pub fn pad_class(&self, pad: &Pad) -> &NetClass {
        match pad.net {
            Some(n) => self.net_class(n),
            None => &self.classes[0],
        }
    }
    pub fn bbox(&self) -> (P, P) {
        let mut lo = [f64::INFINITY; 2];
        let mut hi = [f64::NEG_INFINITY; 2];
        for p in &self.boundary {
            for k in 0..2 {
                lo[k] = lo[k].min(p[k]);
                hi[k] = hi[k].max(p[k]);
            }
        }
        for pad in &self.pads {
            let (plo, phi) = pad.shape.bbox();
            for k in 0..2 {
                lo[k] = lo[k].min(plo[k]);
                hi[k] = hi[k].max(phi[k]);
            }
        }
        (lo, hi)
    }
}
