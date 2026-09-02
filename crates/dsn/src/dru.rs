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
    /// Conditional pair rules: (class A, class B or None = any, clearance um)
    /// from conditions like "A.NetClass == 'X' && B.NetClass == 'Y'".
    pub pair_rules: Vec<(String, Option<String>, f64)>,
    /// Names of rules that were ignored.
    pub unsupported: Vec<String>,
}

/// Extract netclass names from a condition of the supported shape.
fn parse_condition(cond: &str) -> Option<(String, Option<String>)> {
    let mut a: Option<String> = None;
    let mut b: Option<String> = None;
    for part in cond.split("&&") {
        let part = part.trim();
        let (side, rest) = if let Some(r) = part.strip_prefix("A.NetClass") {
            (0, r)
        } else if let Some(r) = part.strip_prefix("B.NetClass") {
            (1, r)
        } else {
            return None; // unsupported clause
        };
        let rest = rest.trim().strip_prefix("==")?.trim();
        let name = rest
            .trim_matches(|c| c == '\'' || c == '"')
            .to_string();
        if side == 0 {
            a = Some(name);
        } else {
            b = Some(name);
        }
    }
    match (a, b) {
        (Some(a), b) => Some((a, b)),
        (None, Some(b)) => Some((b, None)),
        (None, None) => None,
    }
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
        if let Some(cond) = rule.child("condition").and_then(|c| c.str_arg(0)) {
            let min = rule
                .children("constraint")
                .find(|c| c.str_arg(0) == Some("clearance"))
                .and_then(|c| c.child("min"))
                .and_then(|m| m.str_arg(0))
                .and_then(parse_len);
            match (parse_condition(cond), min) {
                (Some((a, b)), Some(v)) => out.pair_rules.push((a, b, v)),
                _ => out.unsupported.push(name),
            }
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
        assert!(d.unsupported.is_empty());
        assert_eq!(d.pair_rules, vec![("HV".to_string(), None, 1000.0)]);
    }
}
