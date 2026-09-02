//! Write a Specctra session (.ses) file that KiCad can import
//! (File > Import > Specctra Session).
//!
//! Coordinates are written at (resolution um 10): integer tenths of a um,
//! matching what freerouting emits and what KiCad's importer expects.

use autoroute_core::board::Board;
use autoroute_core::grid::effective_classes;
use autoroute_core::router::RouteResult;
use std::fmt::Write as _;

const RES: f64 = 10.0; // units per um

fn q(name: &str) -> String {
    let plain = !name.is_empty()
        && name
            .chars()
            .all(|c| c.is_ascii_alphanumeric() || matches!(c, '_' | '.' | '-' | '/' | '+'));
    if plain {
        name.to_string()
    } else {
        format!("\"{name}\"")
    }
}

fn coord(v: f64) -> i64 {
    (v * RES).round() as i64
}

pub fn write_ses(board: &Board, result: &RouteResult, design_name: &str) -> String {
    let mut s = String::new();
    let base = design_name.trim_end_matches(".dsn");
    writeln!(s, "(session {}", q(&format!("{base}.ses"))).unwrap();
    writeln!(s, "  (base_design {})", q(&format!("{base}.dsn"))).unwrap();
    writeln!(s, "  (placement\n    (resolution um {})\n  )", RES as i64).unwrap();
    writeln!(s, "  (was_is\n  )").unwrap();
    writeln!(s, "  (routes ").unwrap();
    writeln!(s, "    (resolution um {})", RES as i64).unwrap();
    writeln!(s, "    (parser").unwrap();
    writeln!(s, "      (host_cad \"AutoRoute\")").unwrap();
    writeln!(s, "      (host_version \"0.1.0\")").unwrap();
    writeln!(s, "    )").unwrap();

    // Via padstack definitions: one per distinct class via.
    let classes = effective_classes(board);
    let mut vias_out: Vec<(String, f64)> = Vec::new();
    for cl in &classes {
        if !vias_out.iter().any(|(n, _)| n == &cl.via_name) {
            vias_out.push((cl.via_name.clone(), cl.via_diameter));
        }
    }
    writeln!(s, "    (library_out ").unwrap();
    for (vname, vdia) in &vias_out {
        writeln!(s, "      (padstack {}", q(vname)).unwrap();
        for layer in &board.layers {
            writeln!(
                s,
                "        (shape\n          (circle {} {} 0 0)\n        )",
                layer,
                coord(*vdia)
            )
            .unwrap();
        }
        writeln!(s, "        (attach off)").unwrap();
        writeln!(s, "      )").unwrap();
    }
    writeln!(s, "    )").unwrap();

    writeln!(s, "    (network_out ").unwrap();
    for rn in &result.nets {
        if rn.segs.is_empty() && rn.vias.is_empty() {
            continue;
        }
        let name = &board.nets[rn.net].name;
        writeln!(s, "      (net {}", q(name)).unwrap();
        for seg in &rn.segs {
            let layer = board
                .layers
                .get(seg.layer)
                .map(|s| s.as_str())
                .unwrap_or("F.Cu");
            let mut line = format!(
                "        (wire\n          (path {} {}",
                layer,
                coord(seg.width)
            );
            for p in &seg.pts {
                write!(line, "\n            {} {}", coord(p[0]), coord(p[1])).unwrap();
            }
            line.push_str("\n          )\n        )");
            writeln!(s, "{line}").unwrap();
        }
        let via_name = &classes[board.nets[rn.net].class.min(classes.len() - 1)].via_name;
        for v in &rn.vias {
            writeln!(
                s,
                "        (via {} {} {}\n        )",
                q(via_name),
                coord(v[0]),
                coord(v[1])
            )
            .unwrap();
        }
        writeln!(s, "      )").unwrap();
    }
    writeln!(s, "    )").unwrap();
    writeln!(s, "  )").unwrap();
    writeln!(s, ")").unwrap();
    s
}
