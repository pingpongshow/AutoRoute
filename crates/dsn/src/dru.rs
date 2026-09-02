//! Minimal .kicad_dru (custom design rules) reader.
//!
//! Supports unconditional global minimums — the common case:
//!   (rule "name" (constraint clearance (min 0.127mm)))
//!   (rule "name" (constraint track_width (min 0.2mm)))
//! Rules with (condition ...) clauses or other constraint types are reported
//! as unsupported and ignored.

use crate::sexpr;

#[derive(Debug, Default, Clone)]
pub struct DruMins {
    /// Minimum copper clearance, um.
    pub clearance: Option<f64>,
    /// Minimum track width, um.
    pub track_width: Option<f64>,
    /// Names of rules that were ignored.
    pub unsupported: Vec<String>,
}

fn parse_len(s: &str) -> Option<f64> {
    let s = s.trim();
    if let Some(v) = s.strip_suffix("mm") {
        v.parse::<f64>().ok().map(|v| v * 1000.0)
    } else if let Some(v) = s.strip_suffix("mil") {
        v.parse::<f64>().ok().map(|v| v * 25.4)
    } else if let Some(v) = s.strip_suffix("in") {
        v.parse::<f64>().ok().map(|v| v * 25400.0)
    } else {
        s.parse::<f64>().ok().map(|v| v * 1000.0) // bare numbers are mm in dru files
    }
}

pub fn parse_dru(text: &str) -> Result<DruMins, String> {
    // The file is a sequence of forms, not one root form: wrap it.
    let wrapped = format!("(dru {text})");
    let root = sexpr::parse(&wrapped)?;
    let mut out = DruMins::default();
    for rule in root.children("rule") {
        let name = rule.str_arg(0).unwrap_or("").to_string();
        if rule.child("condition").is_some() {
            out.unsupported.push(name);
            continue;
        }
        let mut used = false;
        for con in rule.children("constraint") {
            let kind = con.str_arg(0).unwrap_or("");
            let min = con
                .child("min")
                .and_then(|m| m.str_arg(0))
                .and_then(parse_len);
            match (kind, min) {
                ("clearance", Some(v)) => {
                    out.clearance = Some(out.clearance.map_or(v, |c: f64| c.max(v)));
                    used = true;
                }
                ("track_width", Some(v)) => {
                    out.track_width = Some(out.track_width.map_or(v, |c: f64| c.max(v)));
                    used = true;
                }
                // Hole/annular/connection rules don't affect track routing.
                ("hole_size", _) | ("hole_clearance", _) | ("annular_width", _)
                | ("connection_width", _) => {
                    used = true;
                }
                _ => {}
            }
        }
        if !used {
            out.unsupported.push(name);
        }
    }
    Ok(out)
}

/// Look for a .kicad_dru next to a .kicad_pcb.
pub fn sibling_dru(board: &std::path::Path) -> Option<std::path::PathBuf> {
    let p = board.with_extension("kicad_dru");
    p.exists().then_some(p)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_parse_dru() {
        let text = r#"(version 1)
(rule "Minimum clearance 1"
    (constraint clearance (min 0.127mm))
)
(rule "hole" (constraint hole_size (min 0.254mm)))
(rule "fancy" (condition "A.NetClass == 'HV'") (constraint clearance (min 1mm)))
"#;
        let d = parse_dru(text).unwrap();
        assert_eq!(d.clearance, Some(127.0));
        assert_eq!(d.unsupported, vec!["fancy".to_string()]);
    }
}
