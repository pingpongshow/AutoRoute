//! Specctra DSN reading and SES writing for AutoRoute.

pub mod dru;
pub mod kicad;
pub mod parse;
pub mod ses;
pub mod sexpr;

pub use parse::{parse_dsn, parse_dsn_opts, ParsedDsn};
pub use ses::write_ses;

#[cfg(test)]
mod tests {
    use super::*;
    use autoroute_core::{route_board, RouterConfig};
    use std::sync::atomic::AtomicBool;

    /// A minimal hand-written 2-layer DSN: two 2-pad resistors, two nets.
    const DEMO: &str = r#"
(pcb "demo.dsn"
  (parser
    (string_quote ")
    (space_in_quoted_tokens on)
    (host_cad "KiCad's Pcbnew")
  )
  (resolution um 10)
  (unit um)
  (structure
    (layer F.Cu (type signal) (property (index 0)))
    (layer B.Cu (type signal) (property (index 1)))
    (boundary
      (path pcb 0 0 0 30000 0 30000 -20000 0 -20000 0 0)
    )
    (via "Via[0-1]_800:400_um")
    (rule (width 250) (clearance 200.1))
  )
  (placement
    (component "R_0805"
      (place R1 4000 -4000 front 0 (PN 10k))
      (place R2 4000 -16000 front 90 (PN 22k))
    )
  )
  (library
    (image "R_0805"
      (pin RoundRect[T]Pad_1025x1400_um 1 -937.5 0)
      (pin RoundRect[T]Pad_1025x1400_um 2 937.5 0)
    )
    (padstack RoundRect[T]Pad_1025x1400_um
      (shape (rect F.Cu -512.5 -700 512.5 700))
      (attach off)
    )
    (padstack "Via[0-1]_800:400_um"
      (shape (circle F.Cu 800 0 0))
      (shape (circle B.Cu 800 0 0))
      (attach off)
    )
  )
  (network
    (net "Net-(R1-Pad2)"
      (pins R1-2 R2-1)
    )
    (net "GND"
      (pins R1-1 R2-2)
    )
    (class kicad_default "Net-(R1-Pad2)"
      (circuit (use_via "Via[0-1]_800:400_um"))
      (rule (width 250) (clearance 200.1))
    )
    (class POWER GND
      (circuit (use_via "Via[0-1]_800:400_um"))
      (rule (width 500) (clearance 300))
    )
  )
  (wiring)
)
"#;

    #[test]
    fn test_parse_demo() {
        let parsed = parse_dsn(DEMO).unwrap();
        let b = &parsed.board;
        assert_eq!(b.layers, vec!["F.Cu", "B.Cu"]);
        assert_eq!(b.boundary.len(), 4);
        assert_eq!(b.pads.len(), 4);
        assert_eq!(b.nets.len(), 2);
        assert_eq!(b.nets[0].pads.len(), 2);
        assert!((b.rules.width - 250.0).abs() < 1e-9);
        assert!((b.rules.via_diameter - 800.0).abs() < 1e-9);
        // R2 is rotated 90 degrees: its pads should be vertically stacked.
        let p3 = b.pads[2].shape.center();
        let p4 = b.pads[3].shape.center();
        assert!((p3[0] - p4[0]).abs() < 1.0, "rotated pads share x: {p3:?} {p4:?}");
        assert!((p3[1] - p4[1]).abs() > 1000.0);
    }

    #[test]
    fn test_netclasses() {
        let parsed = parse_dsn(DEMO).unwrap();
        let b = &parsed.board;
        // default + kicad_default + POWER
        assert_eq!(b.classes.len(), 3);
        let gnd = b.nets.iter().position(|n| n.name == "GND").unwrap();
        let sig = b.nets.iter().position(|n| n.name == "Net-(R1-Pad2)").unwrap();
        let gnd_class = &b.classes[b.nets[gnd].class];
        assert_eq!(gnd_class.name, "POWER");
        assert!((gnd_class.width - 500.0).abs() < 1e-9);
        assert!((gnd_class.clearance - 300.0).abs() < 1e-9);
        assert!((b.classes[b.nets[sig].class].width - 250.0).abs() < 1e-9);

        // Route and confirm the GND tracks come out at the class width.
        let cancel = AtomicBool::new(false);
        let result = route_board(&parsed.board, RouterConfig::default(), &cancel, &mut |_| {});
        assert!(result.complete, "classed demo should route: {result:?}");
        let gnd_route = &result.nets[gnd];
        assert!(gnd_route.segs.iter().all(|s| (s.width - 500.0).abs() < 1e-9));
        let ses = write_ses(&parsed.board, &result, "demo.dsn");
        assert!(ses.contains("(path F.Cu 5000") || ses.contains("(path B.Cu 5000"),
            "SES should carry 500um-wide GND tracks");
    }

    #[test]
    fn test_keep_existing_wiring() {
        // GND already routed: it must be fixed (not rerouted) and its copper
        // must act as an obstacle for the other net.
        let demo_wired = DEMO.replace(
            "  (wiring)",
            r#"  (wiring
    (wire (path F.Cu 500 3062 -4000 3062 -16937) (net "GND") (type route))
  )"#,
        );
        let parsed = parse_dsn(&demo_wired).unwrap();
        assert_eq!(parsed.fixed_nets, 1);
        let gnd = parsed.board.nets.iter().position(|n| n.name == "GND").unwrap();
        assert!(parsed.board.nets[gnd].fixed);
        // Obstacle pad(s) were added for the wire.
        assert!(parsed.board.pads.len() > 4);

        let cancel = AtomicBool::new(false);
        let result = route_board(&parsed.board, RouterConfig::default(), &cancel, &mut |_| {});
        assert!(result.complete, "signal net should still route: {result:?}");
        // The fixed net emitted no new geometry.
        assert!(result.nets[gnd].segs.is_empty());
        // And with keep_wiring off, both nets route from scratch.
        let parsed2 = parse_dsn_opts(&demo_wired, false).unwrap();
        assert_eq!(parsed2.fixed_nets, 0);
        assert!(!parsed2.board.nets[gnd].fixed);
    }

    #[test]
    fn test_pour_connected_net() {
        // GND has a copper pour: it is left to the pour, not routed.
        let demo_poured = DEMO.replace(
            "    (via \"Via[0-1]_800:400_um\")",
            "    (plane GND (polygon F.Cu 0 0 0 30000 0 30000 -20000 0 -20000))\n    (via \"Via[0-1]_800:400_um\")",
        );
        let parsed = parse_dsn(&demo_poured).unwrap();
        assert_eq!(parsed.poured_nets, 1);
        let gnd = parsed.board.nets.iter().position(|n| n.name == "GND").unwrap();
        assert!(parsed.board.nets[gnd].fixed);
        let cancel = AtomicBool::new(false);
        let result = route_board(&parsed.board, RouterConfig::default(), &cancel, &mut |_| {});
        assert!(result.complete);
        assert!(result.nets[gnd].segs.is_empty());
    }

    #[test]
    fn test_route_demo_and_write_ses() {
        let parsed = parse_dsn(DEMO).unwrap();
        let cancel = AtomicBool::new(false);
        let result = route_board(&parsed.board, RouterConfig::default(), &cancel, &mut |_| {});
        assert!(result.complete, "demo board should route: {result:?}");
        let ses = write_ses(&parsed.board, &result, "demo.dsn");
        assert!(ses.contains("(session"));
        assert!(ses.contains("network_out"));
        assert!(ses.contains("\"Net-(R1-Pad2)\""));
        // The SES must itself be a valid s-expression.
        crate::sexpr::parse(&ses).unwrap();
    }
}
