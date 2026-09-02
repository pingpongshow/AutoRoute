//! Render a routed board (Board + SES geometry) to a standalone SVG.

use autoroute_core::board::{Board, PadShape};
use autoroute_dsn::sexpr;
use std::fmt::Write as _;

const LAYER_COLORS: [&str; 6] = ["#d64949", "#4d7fd6", "#57a64a", "#d69e40", "#9e5ed6", "#40bebe"];

pub struct SesWire {
    pub layer: usize,
    pub width: f64,
    pub pts: Vec<[f64; 2]>,
}

pub struct SesNet {
    pub name: String,
    pub wires: Vec<SesWire>,
    pub vias: Vec<[f64; 2]>,
}

/// Parse the routes out of a SES file we (or freerouting) wrote.
pub fn parse_ses(text: &str, board: &Board) -> Result<Vec<SesNet>, String> {
    let root = sexpr::parse(text)?;
    let routes = root.child("routes").ok_or("no (routes) in session")?;
    let res = routes
        .child("resolution")
        .and_then(|r| r.f64_arg(1))
        .unwrap_or(10.0);
    let scale = 1.0 / res; // resolution units -> um
    let net_out = routes.child("network_out").ok_or("no (network_out)")?;
    let mut nets = Vec::new();
    for net in net_out.children("net") {
        let mut sn = SesNet {
            name: net.str_arg(0).unwrap_or("").to_string(),
            wires: Vec::new(),
            vias: Vec::new(),
        };
        for wire in net.children("wire") {
            if let Some(path) = wire.child("path") {
                let layer_name = path.str_arg(0).unwrap_or("");
                let layer = board
                    .layers
                    .iter()
                    .position(|l| l == layer_name)
                    .unwrap_or(0);
                let width = path.f64_arg(1).unwrap_or(0.0) * scale;
                let coords: Vec<f64> = path
                    .args()
                    .iter()
                    .skip(2)
                    .filter_map(|a| a.as_atom().and_then(|s| s.parse::<f64>().ok()))
                    .collect();
                let pts: Vec<[f64; 2]> = coords
                    .chunks_exact(2)
                    .map(|xy| [xy[0] * scale, xy[1] * scale])
                    .collect();
                if pts.len() >= 2 {
                    sn.wires.push(SesWire { layer, width, pts });
                }
            }
        }
        for via in net.children("via") {
            if let (Some(x), Some(y)) = (via.f64_arg(1), via.f64_arg(2)) {
                sn.vias.push([x * scale, y * scale]);
            }
        }
        nets.push(sn);
    }
    Ok(nets)
}

pub fn render_svg(board: &Board, nets: &[SesNet]) -> String {
    let (lo, hi) = board.bbox();
    let pad = 2000.0;
    let (x0, y0) = (lo[0] - pad, -(hi[1] + pad));
    let (w, h) = (hi[0] - lo[0] + 2.0 * pad, hi[1] - lo[1] + 2.0 * pad);
    // SVG y grows down; board y grows up, so flip: sy = -y.
    let mut s = String::new();
    writeln!(
        s,
        r#"<svg xmlns="http://www.w3.org/2000/svg" viewBox="{x0:.0} {y0:.0} {w:.0} {h:.0}" width="1200">"#
    )
    .unwrap();
    writeln!(s, r##"<rect x="{x0:.0}" y="{y0:.0}" width="{w:.0}" height="{h:.0}" fill="#181a1e"/>"##).unwrap();

    // Board outline.
    if board.boundary.len() >= 3 {
        let pts: Vec<String> = board
            .boundary
            .iter()
            .map(|p| format!("{:.0},{:.0}", p[0], -p[1]))
            .collect();
        writeln!(
            s,
            r##"<polygon points="{}" fill="none" stroke="#b4b45a" stroke-width="120"/>"##,
            pts.join(" ")
        )
        .unwrap();
    }

    // Tracks, bottom layers first so top is drawn on top.
    let mut order: Vec<usize> = (0..board.layers.len()).collect();
    order.reverse();
    for &layer in &order {
        let color = LAYER_COLORS[layer % LAYER_COLORS.len()];
        for net in nets {
            for wire in &net.wires {
                if wire.layer != layer {
                    continue;
                }
                let pts: Vec<String> = wire
                    .pts
                    .iter()
                    .map(|p| format!("{:.0},{:.0}", p[0], -p[1]))
                    .collect();
                writeln!(
                    s,
                    r#"<polyline points="{}" fill="none" stroke="{color}" stroke-width="{:.0}" stroke-opacity="0.85" stroke-linecap="round" stroke-linejoin="round"/>"#,
                    pts.join(" "),
                    wire.width
                )
                .unwrap();
            }
        }
    }

    // Pads.
    for p in &board.pads {
        let color = if p.layers.len() > 1 {
            "#c8aa3c"
        } else {
            LAYER_COLORS[p.layers.first().copied().unwrap_or(0) % LAYER_COLORS.len()]
        };
        match &p.shape {
            PadShape::Circle { center, diameter } => {
                writeln!(
                    s,
                    r#"<circle cx="{:.0}" cy="{:.0}" r="{:.0}" fill="{color}" fill-opacity="0.9"/>"#,
                    center[0],
                    -center[1],
                    diameter / 2.0
                )
                .unwrap();
            }
            PadShape::Polygon { pts } => {
                let pp: Vec<String> = pts.iter().map(|q| format!("{:.0},{:.0}", q[0], -q[1])).collect();
                writeln!(s, r#"<polygon points="{}" fill="{color}" fill-opacity="0.9"/>"#, pp.join(" "))
                    .unwrap();
            }
            PadShape::Path { width, pts } => {
                let pp: Vec<String> = pts.iter().map(|q| format!("{:.0},{:.0}", q[0], -q[1])).collect();
                writeln!(
                    s,
                    r#"<polyline points="{}" fill="none" stroke="{color}" stroke-width="{:.0}" stroke-opacity="0.9" stroke-linecap="round"/>"#,
                    pp.join(" "),
                    width
                )
                .unwrap();
            }
        }
    }

    // Vias.
    for net in nets {
        for v in &net.vias {
            let r = board.rules.via_diameter / 2.0;
            writeln!(
                s,
                r##"<circle cx="{:.0}" cy="{:.0}" r="{r:.0}" fill="#bebebe"/><circle cx="{:.0}" cy="{:.0}" r="{:.0}" fill="#282828"/>"##,
                v[0], -v[1], v[0], -v[1], r * 0.45
            )
            .unwrap();
        }
    }
    writeln!(s, "</svg>").unwrap();
    s
}
