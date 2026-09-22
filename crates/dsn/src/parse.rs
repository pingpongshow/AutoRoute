//! Convert a Specctra DSN document (as exported by KiCad) into the flattened
//! `autoroute_core::Board` model.

use std::collections::HashMap;

use autoroute_core::board::{Board, Net, NetClass, Pad, PadShape, Rules};
use autoroute_core::geometry::P;

use crate::sexpr::{parse as parse_sexpr, SExpr};

#[derive(Debug, Clone)]
struct PinDef {
    padstack: String,
    id: String,
    at: P,
    rotation: f64,
}

#[derive(Debug, Clone)]
struct ImageDef {
    pins: Vec<PinDef>,
    /// (layer name, shape) obstacles from footprint keepouts.
    keepouts: Vec<(String, PadShape)>,
}

#[derive(Debug, Clone)]
struct PadstackDef {
    /// (layer name, shape in padstack-local coordinates)
    shapes: Vec<(String, PadShape)>,
}

pub struct ParsedDsn {
    pub board: Board,
    pub warnings: Vec<String>,
    /// Number of nets with pre-existing wiring that was preserved.
    pub fixed_nets: usize,
    /// Number of nets left to their copper pours (planes).
    pub poured_nets: usize,
}

/// Parse with default options: pre-existing wiring is preserved (wired nets
/// are fixed, their copper becomes obstacles).
pub fn parse_dsn(text: &str) -> Result<ParsedDsn, String> {
    parse_dsn_opts(text, true)
}

pub fn parse_dsn_opts(text: &str, keep_wiring: bool) -> Result<ParsedDsn, String> {
    let root = parse_sexpr(text)?;
    if root.name() != Some("pcb") && root.name() != Some("PCB") {
        return Err(format!("expected (pcb ...), got {:?}", root.name()));
    }
    let mut warnings = Vec::new();

    let name = root.str_arg(0).unwrap_or("board").to_string();

    // Unit scale to um.
    let scale = unit_scale(&root);

    let structure = root
        .child("structure")
        .ok_or("missing (structure ...)")?;

    // --- Layers (signal only), honoring explicit index properties if present.
    let mut layers: Vec<(usize, String)> = Vec::new();
    for (order, layer) in structure.children("layer").enumerate() {
        let lname = layer.str_arg(0).unwrap_or("").to_string();
        let is_signal = layer
            .child("type")
            .and_then(|t| t.str_arg(0))
            .map(|t| t == "signal")
            .unwrap_or(true);
        if !is_signal {
            continue;
        }
        let index = layer
            .child("property")
            .and_then(|p| p.child("index"))
            .and_then(|i| i.f64_arg(0))
            .map(|f| f as usize)
            .unwrap_or(order);
        layers.push((index, lname));
    }
    layers.sort_by_key(|(i, _)| *i);
    let layer_names: Vec<String> = layers.into_iter().map(|(_, n)| n).collect();
    if layer_names.is_empty() {
        return Err("no signal layers found".into());
    }
    let layer_index: HashMap<String, usize> = layer_names
        .iter()
        .enumerate()
        .map(|(i, n)| (n.clone(), i))
        .collect();

    // --- Boundary.
    let mut boundary: Vec<P> = Vec::new();
    if let Some(b) = structure.child("boundary") {
        if let Some(path) = b.child("path") {
            // (path pcb <width> x y x y ...)
            let args = path.args();
            let mut coords = Vec::new();
            for a in args.iter().skip(2) {
                if let Some(v) = a.as_atom().and_then(|s| s.parse::<f64>().ok()) {
                    coords.push(v * scale);
                }
            }
            for xy in coords.chunks_exact(2) {
                boundary.push([xy[0], xy[1]]);
            }
        } else if let Some(rect) = b.child("rect") {
            let x1 = rect.f64_arg(1).unwrap_or(0.0) * scale;
            let y1 = rect.f64_arg(2).unwrap_or(0.0) * scale;
            let x2 = rect.f64_arg(3).unwrap_or(0.0) * scale;
            let y2 = rect.f64_arg(4).unwrap_or(0.0) * scale;
            boundary = vec![[x1, y1], [x2, y1], [x2, y2], [x1, y2]];
        }
    }
    // Drop consecutive duplicate points (KiCad closes the polygon).
    boundary.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9);
    if boundary.len() >= 2 {
        let (f, l) = (boundary[0], *boundary.last().unwrap());
        if (f[0] - l[0]).abs() < 1e-9 && (f[1] - l[1]).abs() < 1e-9 {
            boundary.pop();
        }
    }

    // --- Rules.
    let mut rules = Rules::default();
    if let Some(rule) = structure.child("rule") {
        if let Some(w) = rule.child("width").and_then(|w| w.f64_arg(0)) {
            rules.width = w * scale;
        }
        // Take the plain (clearance x) — the first one without a (type ...) arg,
        // else the first clearance at all.
        let mut best: Option<f64> = None;
        for c in rule.children("clearance") {
            let has_type = c.child("type").is_some();
            if let Some(v) = c.f64_arg(0) {
                if !has_type {
                    best = Some(v);
                    break;
                }
                if best.is_none() {
                    best = Some(v);
                }
            }
        }
        if let Some(c) = best {
            rules.clearance = c * scale;
        }
    }
    if let Some(via) = structure.child("via") {
        if let Some(vn) = via.str_arg(0) {
            rules.via_name = vn.to_string();
        }
    }

    // --- Library: padstacks and images.
    let library = root.child("library").ok_or("missing (library ...)")?;
    let mut padstacks: HashMap<String, PadstackDef> = HashMap::new();
    for ps in library.children("padstack") {
        let ps_name = ps.str_arg(0).unwrap_or("").to_string();
        let mut shapes = Vec::new();
        for shape in ps.children("shape") {
            if let Some(inner) = shape.items().get(1) {
                if let Some((lname, s)) = parse_shape(inner, scale) {
                    shapes.push((lname, s));
                }
            }
        }
        padstacks.insert(ps_name, PadstackDef { shapes });
    }

    // Via geometry from its padstack (drill is not in the DSN; guess from name).
    if let Some(vps) = padstacks.get(&rules.via_name) {
        let mut dia: f64 = 0.0;
        for (_, s) in &vps.shapes {
            if let PadShape::Circle { diameter, .. } = s {
                dia = dia.max(*diameter);
            }
        }
        if dia > 0.0 {
            rules.via_diameter = dia;
        }
    }
    if let Some(drill) = via_drill_from_name(&rules.via_name) {
        rules.via_drill = drill * scale_for_name(&rules.via_name, scale);
    } else {
        rules.via_drill = rules.via_diameter / 2.0;
    }

    let mut images: HashMap<String, ImageDef> = HashMap::new();
    for img in library.children("image") {
        let img_name = img.str_arg(0).unwrap_or("").to_string();
        let mut pins = Vec::new();
        for pin in img.children("pin") {
            // (pin PADSTACK [(rotate deg)] PIN_ID X Y)
            let mut padstack = String::new();
            let mut rotation = 0.0;
            let mut atoms: Vec<&str> = Vec::new();
            for (i, item) in pin.args().iter().enumerate() {
                match item {
                    SExpr::Atom(a) => {
                        if i == 0 {
                            padstack = a.clone();
                        } else {
                            atoms.push(a);
                        }
                    }
                    SExpr::List(_) => {
                        if item.name() == Some("rotate") {
                            rotation = item.f64_arg(0).unwrap_or(0.0);
                        }
                    }
                }
            }
            if atoms.len() < 3 {
                warnings.push(format!("image {img_name}: malformed pin, skipped"));
                continue;
            }
            let y = atoms.pop().unwrap().parse::<f64>().unwrap_or(0.0) * scale;
            let x = atoms.pop().unwrap().parse::<f64>().unwrap_or(0.0) * scale;
            let id = atoms.join("-");
            pins.push(PinDef { padstack, id, at: [x, y], rotation });
        }
        let mut keepouts = Vec::new();
        for ko in img.children("keepout") {
            // (keepout "" (shape...)) — shape may be direct or wrapped.
            for item in ko.args() {
                if let SExpr::List(_) = item {
                    if let Some((lname, s)) = parse_shape(item, scale) {
                        keepouts.push((lname, s));
                    }
                }
            }
        }
        images.insert(img_name, ImageDef { pins, keepouts });
    }

    // --- Placement: flatten pads.
    let placement = root.child("placement").ok_or("missing (placement ...)")?;
    let mut pads: Vec<Pad> = Vec::new();
    let mut pad_by_id: HashMap<String, usize> = HashMap::new();
    for comp in placement.children("component") {
        let img_name = comp.str_arg(0).unwrap_or("");
        let image = match images.get(img_name) {
            Some(i) => i,
            None => {
                warnings.push(format!("component image not found: {img_name}"));
                continue;
            }
        };
        for place in comp.children("place") {
            let reference = place.str_arg(0).unwrap_or("?").to_string();
            let px = place.f64_arg(1).unwrap_or(0.0) * scale;
            let py = place.f64_arg(2).unwrap_or(0.0) * scale;
            let side = place.str_arg(3).unwrap_or("front").to_string();
            let rot = place.f64_arg(4).unwrap_or(0.0);
            let back = side == "back";

            for pin in &image.pins {
                let ps = match padstacks.get(&pin.padstack) {
                    Some(p) => p,
                    None => {
                        warnings.push(format!("padstack not found: {}", pin.padstack));
                        continue;
                    }
                };
                // Collect the layer set and take the first shape as geometry.
                let mut pad_layers: Vec<usize> = Vec::new();
                let mut shape_local: Option<PadShape> = None;
                for (lname, s) in &ps.shapes {
                    let resolved: Vec<usize> = if let Some(&li) = layer_index.get(lname) {
                        vec![li]
                    } else {
                        // "signal"/unknown layer applies to all copper layers.
                        (0..layer_names.len()).collect()
                    };
                    for li in resolved {
                        let li = if back { layer_names.len() - 1 - li } else { li };
                        if !pad_layers.contains(&li) {
                            pad_layers.push(li);
                        }
                    }
                    if shape_local.is_none() {
                        shape_local = Some(s.clone());
                    }
                }
                let shape_local = match shape_local {
                    Some(s) => s,
                    None => continue,
                };
                // Compose: pin-level rotation, pin offset, then place transform.
                let shifted = shape_local
                    .transform(false, pin.rotation, pin.at);
                let abs = shifted.transform(back, if back { -rot } else { rot }, [px, py]);
                let id = format!("{reference}-{}", pin.id);
                pad_by_id.insert(id.clone(), pads.len());
                pads.push(Pad { id, shape: abs, layers: pad_layers, net: None , is_pour: false});
            }
            // Footprint keepouts become unconnected hard obstacles.
            for (lname, shape) in &image.keepouts {
                let layers: Vec<usize> = if let Some(&li) = layer_index.get(lname) {
                    let li = if back { layer_names.len() - 1 - li } else { li };
                    vec![li]
                } else {
                    (0..layer_names.len()).collect()
                };
                let abs = shape.transform(back, if back { -rot } else { rot }, [px, py]);
                pads.push(Pad { id: String::new(), shape: abs, layers, net: None , is_pour: false});
            }
        }
    }

    // --- Network.
    let mut nets: Vec<Net> = Vec::new();
    let mut net_index: HashMap<String, usize> = HashMap::new();
    if let Some(network) = root.child("network") {
        for net in network.children("net") {
            let net_name = net.str_arg(0).unwrap_or("").to_string();
            let mut net_pads = Vec::new();
            if let Some(pins) = net.child("pins") {
                for pin in pins.args() {
                    if let Some(pid) = pin.as_atom() {
                        if let Some(&pi) = pad_by_id.get(pid) {
                            net_pads.push(pi);
                        } else {
                            warnings.push(format!("net {net_name}: pin {pid} not found"));
                        }
                    }
                }
            }
            let net_idx = nets.len();
            // A pin can only belong to one net; keep the first assignment.
            net_pads.retain(|&pi| {
                if let Some(prev) = pads[pi].net {
                    warnings.push(format!(
                        "net {net_name}: pin {} already on net {}, ignored",
                        pads[pi].id, nets[prev].name
                    ));
                    false
                } else {
                    pads[pi].net = Some(net_idx);
                    true
                }
            });
            net_index.insert(net_name.clone(), net_idx);
            nets.push(Net { name: net_name, pads: net_pads, class: 0, fixed: false });
        }
    }

    // --- Net classes. Class 0 is the default (from the structure rules);
    // additional (class ...) entries carry their own width/clearance/via and
    // list their member nets.
    let mut classes: Vec<NetClass> = vec![NetClass {
        name: "default".into(),
        width: rules.width,
        clearance: rules.clearance,
        via_diameter: rules.via_diameter,
        via_name: rules.via_name.clone(),
    }];
    if let Some(network) = root.child("network") {
        for cls in network.children("class") {
            let cls_name = cls.str_arg(0).unwrap_or("").to_string();
            let mut nc = NetClass {
                name: cls_name.clone(),
                ..classes[0].clone()
            };
            if let Some(rule) = cls.child("rule") {
                if let Some(w) = rule.child("width").and_then(|w| w.f64_arg(0)) {
                    nc.width = w * scale;
                }
                if let Some(c) = rule.child("clearance").and_then(|c| c.f64_arg(0)) {
                    nc.clearance = c * scale;
                }
            }
            if let Some(via) = cls.child("circuit").and_then(|c| c.child("use_via")) {
                if let Some(vn) = via.str_arg(0) {
                    nc.via_name = vn.to_string();
                    if let Some(vps) = padstacks.get(vn) {
                        let mut dia: f64 = 0.0;
                        for (_, s) in &vps.shapes {
                            if let PadShape::Circle { diameter, .. } = s {
                                dia = dia.max(*diameter);
                            }
                        }
                        if dia > 0.0 {
                            nc.via_diameter = dia;
                        }
                    }
                }
            }
            // Member nets: bare atoms after the class name (lists are rules).
            let members: Vec<&str> = cls
                .args()
                .iter()
                .skip(1)
                .filter_map(|a| a.as_atom())
                .collect();
            let class_idx = classes.len();
            let mut used = false;
            for m in &members {
                if let Some(&ni) = net_index.get(*m) {
                    nets[ni].class = class_idx;
                    used = true;
                } else if !m.is_empty() {
                    warnings.push(format!("class {cls_name}: unknown net {m}"));
                }
            }
            if used || !members.is_empty() {
                classes.push(nc);
            }
        }
    }

    // --- Structure-level keepouts: unconnected hard obstacles.
    for ko in structure.children("keepout") {
        for item in ko.args() {
            if let SExpr::List(_) = item {
                if let Some((lname, s)) = parse_shape(item, scale) {
                    let layers: Vec<usize> = layer_index
                        .get(&lname)
                        .map(|&l| vec![l])
                        .unwrap_or_else(|| (0..layer_names.len()).collect());
                    pads.push(Pad { id: String::new(), shape: s, layers, net: None , is_pour: false});
                }
            }
        }
    }

    // --- Track-only keepouts (KiCad "tracks not allowed" rule areas) export as
    // `wire_keepout`, which this router does not model: obstacles block wires
    // and vias alike, so honouring one would also forbid every via inside it.
    // Warn rather than drop it silently — copper WILL be routed through.
    for ko in structure.children("wire_keepout") {
        let layer = ko
            .child("polygon")
            .and_then(|p| p.str_arg(0))
            .unwrap_or("?")
            .to_string();
        warnings.push(format!(
            "wire_keepout on {layer} ignored: tracks WILL be routed through it \
             (track-only keepouts are not modelled)"
        ));
    }

    // --- Copper pours: each plane becomes a routable pour region for its
    // net — pads not already inside it get stitched to it with stubs and
    // vias — while never blocking other nets (the pour refills around them).
    let mut poured_nets = 0usize;
    if keep_wiring {
        let mut seen_pour: Vec<bool> = vec![false; nets.len()];
        for plane in structure.children("plane") {
            let net_name = match plane.str_arg(0) {
                Some(n) => n,
                None => continue,
            };
            let ni = match net_index.get(net_name) {
                Some(&ni) => ni,
                None => continue,
            };
            let poly = match plane.child("polygon") {
                Some(p) => p,
                None => continue,
            };
            let lname = poly.str_arg(0).unwrap_or("");
            let layer = match layer_index.get(lname) {
                Some(&l) => l,
                None => continue,
            };
            let coords: Vec<f64> = poly
                .args()
                .iter()
                .skip(2)
                .filter_map(|a| a.as_atom().and_then(|s| s.parse::<f64>().ok()))
                .collect();
            let mut pts: Vec<P> = coords
                .chunks_exact(2)
                .map(|xy| [xy[0] * scale, xy[1] * scale])
                .collect();
            pts.dedup_by(|a, b| (a[0] - b[0]).abs() < 1e-9 && (a[1] - b[1]).abs() < 1e-9);
            if pts.len() < 3 {
                continue;
            }
            let pad_idx = pads.len();
            pads.push(Pad {
                id: format!("pour-{net_name}"),
                shape: PadShape::Polygon { pts },
                layers: vec![layer],
                net: Some(ni),
                is_pour: true,
            });
            nets[ni].pads.push(pad_idx);
            if !seen_pour[ni] {
                seen_pour[ni] = true;
                poured_nets += 1;
            }
        }

        // A pad already sitting inside its own net's pour, on a layer that pour
        // occupies, is connected by the zone fill itself — KiCad joins it when
        // the zone is filled. Stitching it anyway emits a redundant stub, and a
        // stub touching a pad whose neighbour is closer than the class clearance
        // cannot be legal at any width. Drop those pads from the routing target
        // list; they stay in `pads` as obstacles.
        let pour_polys: Vec<(usize, usize, PadShape)> = pads
            .iter()
            .enumerate()
            .filter(|(_, p)| p.is_pour)
            .filter_map(|(i, p)| p.net.map(|n| (i, n, p.shape.clone())))
            .collect();
        if !pour_polys.is_empty() {
            let mut covered = 0usize;
            for (ni, net) in nets.iter_mut().enumerate() {
                if !seen_pour[ni] {
                    continue;
                }
                net.pads.retain(|&pi| {
                    let pad = &pads[pi];
                    if pad.is_pour {
                        return true;
                    }
                    let inside = pour_polys.iter().any(|(qi, qn, shape)| {
                        *qn == ni
                            && pads[*qi].layers.iter().any(|l| pad.layers.contains(l))
                            && shape.signed_dist(pad.shape.center()) < 0.0
                    });
                    if inside {
                        covered += 1;
                    }
                    !inside
                });
            }
            if covered > 0 {
                warnings.push(format!(
                    "{covered} pads already inside their net's pour: left to the zone fill, not stitched"
                ));
            }
        }
    }

    // --- Existing wiring: preserved as net-owned obstacles; wired nets are
    // fixed (not rerouted). KiCad's SES import would otherwise delete a
    // wired net's existing tracks when the session touches that net.
    let mut fixed_nets = 0usize;
    if keep_wiring {
        if let Some(wiring) = root.child("wiring") {
            let mut newly_fixed: Vec<bool> = vec![false; nets.len()];
            for item in wiring.items() {
                match item.name() {
                    Some("wire") => {
                        let path = match item.child("path") {
                            Some(p) => p,
                            None => continue,
                        };
                        let lname = path.str_arg(0).unwrap_or("");
                        let layer = match layer_index.get(lname) {
                            Some(&l) => l,
                            None => continue,
                        };
                        let w = path.f64_arg(1).unwrap_or(0.0) * scale;
                        let coords: Vec<f64> = path
                            .args()
                            .iter()
                            .skip(2)
                            .filter_map(|a| a.as_atom().and_then(|s| s.parse::<f64>().ok()))
                            .collect();
                        let pts: Vec<P> = coords
                            .chunks_exact(2)
                            .map(|xy| [xy[0] * scale, xy[1] * scale])
                            .collect();
                        let net = item
                            .child("net")
                            .and_then(|n| n.str_arg(0))
                            .and_then(|nm| net_index.get(nm).copied());
                        if let Some(ni) = net {
                            newly_fixed[ni] = true;
                        }
                        // One obstacle per segment keeps rasterization bboxes
                        // tight even for long meandering tracks.
                        for sw in pts.windows(2) {
                            pads.push(Pad {
                                id: String::new(),
                                shape: PadShape::Path { width: w, pts: sw.to_vec() },
                                layers: vec![layer],
                                net,
                                is_pour: false,
                            });
                        }
                        if pts.len() == 1 {
                            pads.push(Pad {
                                id: String::new(),
                                shape: PadShape::Path { width: w, pts },
                                layers: vec![layer],
                                net,
                                is_pour: false,
                            });
                        }
                    }
                    Some("via") => {
                        // (via PADSTACK x y (net "..") ...)
                        let ps_name = item.str_arg(0).unwrap_or("");
                        let x = item.f64_arg(1).unwrap_or(0.0) * scale;
                        let y = item.f64_arg(2).unwrap_or(0.0) * scale;
                        let mut dia = rules.via_diameter;
                        if let Some(vps) = padstacks.get(ps_name) {
                            for (_, s) in &vps.shapes {
                                if let PadShape::Circle { diameter, .. } = s {
                                    dia = dia.max(*diameter);
                                }
                            }
                        }
                        let net = item
                            .child("net")
                            .and_then(|n| n.str_arg(0))
                            .and_then(|nm| net_index.get(nm).copied());
                        if let Some(ni) = net {
                            newly_fixed[ni] = true;
                        }
                        pads.push(Pad {
                            id: String::new(),
                            shape: PadShape::Circle { center: [x, y], diameter: dia },
                            layers: (0..layer_names.len()).collect(),
                            net,
                            is_pour: false,
                        });
                    }
                    _ => {}
                }
            }
            for (ni, f) in newly_fixed.into_iter().enumerate() {
                if f {
                    nets[ni].fixed = true;
                    fixed_nets += 1;
                }
            }
        }
    }

    Ok(ParsedDsn {
        board: Board {
            name,
            layers: layer_names,
            boundary,
            pads,
            nets,
            rules,
            classes,
            pair_clearances: Vec::new(),
        },
        warnings,
        fixed_nets,
        poured_nets,
    })
}

fn unit_scale(root: &SExpr) -> f64 {
    let unit = root
        .child("unit")
        .and_then(|u| u.str_arg(0))
        .or_else(|| root.child("resolution").and_then(|r| r.str_arg(0)))
        .unwrap_or("um");
    match unit {
        "mm" => 1000.0,
        "cm" => 10000.0,
        "mil" => 25.4,
        "inch" => 25400.0,
        _ => 1.0, // um
    }
}

/// KiCad via padstack names look like "Via[0-1]_800:400_um" (diameter:drill).
fn via_drill_from_name(name: &str) -> Option<f64> {
    let part = name.split('_').nth(1)?;
    let drill = part.split(':').nth(1)?;
    drill.parse().ok()
}

fn scale_for_name(name: &str, default_scale: f64) -> f64 {
    if name.ends_with("_um") {
        1.0
    } else if name.ends_with("_mm") {
        1000.0
    } else {
        default_scale
    }
}

/// Parse one shape form: (circle L d [cx cy]) | (rect L x1 y1 x2 y2)
/// | (path L width x y x y ...) | (polygon L aperture x y ...)
fn parse_shape(e: &SExpr, scale: f64) -> Option<(String, PadShape)> {
    let lname = e.str_arg(0)?.to_string();
    match e.name()? {
        "circle" => {
            let d = e.f64_arg(1)? * scale;
            let cx = e.f64_arg(2).unwrap_or(0.0) * scale;
            let cy = e.f64_arg(3).unwrap_or(0.0) * scale;
            Some((lname, PadShape::Circle { center: [cx, cy], diameter: d }))
        }
        "rect" => {
            let x1 = e.f64_arg(1)? * scale;
            let y1 = e.f64_arg(2)? * scale;
            let x2 = e.f64_arg(3)? * scale;
            let y2 = e.f64_arg(4)? * scale;
            Some((
                lname,
                PadShape::Polygon {
                    pts: vec![[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                },
            ))
        }
        "path" => {
            let width = e.f64_arg(1)? * scale;
            let mut pts = Vec::new();
            let coords: Vec<f64> = e
                .args()
                .iter()
                .skip(2)
                .filter_map(|a| a.as_atom().and_then(|s| s.parse::<f64>().ok()))
                .collect();
            for xy in coords.chunks_exact(2) {
                pts.push([xy[0] * scale, xy[1] * scale]);
            }
            if pts.is_empty() {
                return None;
            }
            Some((lname, PadShape::Path { width, pts }))
        }
        "polygon" => {
            let mut pts = Vec::new();
            let coords: Vec<f64> = e
                .args()
                .iter()
                .skip(2)
                .filter_map(|a| a.as_atom().and_then(|s| s.parse::<f64>().ok()))
                .collect();
            for xy in coords.chunks_exact(2) {
                pts.push([xy[0] * scale, xy[1] * scale]);
            }
            if pts.len() < 3 {
                return None;
            }
            Some((lname, PadShape::Polygon { pts }))
        }
        _ => None,
    }
}
