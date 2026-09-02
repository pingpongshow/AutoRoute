//! AutoRoute CLI: route a Specctra DSN file and write a .ses session
//! importable by KiCad (File > Import > Specctra Session).

use std::path::PathBuf;
use std::sync::atomic::AtomicBool;
use std::time::Instant;

use clap::Parser;

use autoroute_core::{route_board, RouterConfig};
use autoroute_dsn::{parse_dsn_opts, write_ses};

mod svg;

#[derive(Parser)]
#[command(name = "autoroute", version, about = "PCB autorouter (Specctra DSN in, SES out)")]
struct Args {
    /// Input .dsn file (export from KiCad: File > Export > Specctra DSN)
    input: PathBuf,

    /// Output .ses file (default: input with .ses extension)
    #[arg(short, long)]
    output: Option<PathBuf>,

    /// Routing grid pitch in um (default: derived from design rules)
    #[arg(long)]
    grid: Option<f64>,

    /// Maximum rip-up/reroute iterations
    #[arg(long, default_value_t = 40)]
    iterations: usize,

    /// Via cost in um of equivalent track length
    #[arg(long, default_value_t = 5000.0)]
    via_cost: f64,

    /// Print per-iteration progress
    #[arg(short, long)]
    verbose: bool,

    /// Run a geometric DRC on the routed result and report violations
    #[arg(long)]
    check: bool,

    /// Comma-separated class names to leave unrouted (e.g. power/zone nets:
    /// "HV_TX,POWER,+VIN"); their pads still act as obstacles
    #[arg(long, value_delimiter = ',')]
    skip_classes: Vec<String>,

    /// Cap all net-class clearances at this value (um). Large class
    /// clearances otherwise forbid ANY copper near that class's pads
    /// (KiCad max-of-pair semantics), which often makes signal escapes
    /// impossible next to power pads.
    #[arg(long)]
    clearance_cap: Option<f64>,

    /// Ignore pre-existing routing and copper pours; route every net from
    /// scratch. By default, already-routed nets are preserved as obstacles
    /// and pour-connected nets are left to their pours.
    #[arg(long)]
    reroute_wired: bool,

    /// Route only nets matching these comma-separated patterns
    /// ("*" wildcard allowed, e.g. "ETH_*,/RX*")
    #[arg(long, value_delimiter = ',')]
    nets: Vec<String>,

    /// Post-completion cleanup passes: reroute each net alone to straighten
    /// tracks and drop unnecessary vias
    #[arg(long, default_value_t = 1)]
    optimize: usize,

    /// Apply minimums from a .kicad_dru file (auto-detected next to a
    /// .kicad_pcb input)
    #[arg(long)]
    dru: Option<PathBuf>,

    /// Instead of routing, render an existing SES over the DSN to SVG
    #[arg(long, value_name = "SES")]
    render_ses: Option<PathBuf>,

    /// SVG output path (with --render-ses; default: SES path with .svg)
    #[arg(long)]
    svg: Option<PathBuf>,
}

fn main() {
    let args = Args::parse();

    // Direct .kicad_pcb input: export a DSN via KiCad's own converter first,
    // and remember to write the routed result back at the end.
    let is_kicad = args
        .input
        .extension()
        .map(|e| e == "kicad_pcb")
        .unwrap_or(false);
    let (dsn_path, kicad_board) = if is_kicad {
        let dsn = args.input.with_extension("autoroute.dsn");
        eprintln!("exporting Specctra DSN via KiCad...");
        if let Err(e) = autoroute_dsn::kicad::export_dsn(&args.input, &dsn) {
            eprintln!("error: {e}");
            std::process::exit(1);
        }
        (dsn, Some(args.input.clone()))
    } else {
        (args.input.clone(), None)
    };

    let text = match std::fs::read_to_string(&dsn_path) {
        Ok(t) => t,
        Err(e) => {
            eprintln!("error: cannot read {}: {e}", dsn_path.display());
            std::process::exit(1);
        }
    };

    let mut parsed = match parse_dsn_opts(&text, !args.reroute_wired) {
        Ok(p) => p,
        Err(e) => {
            eprintln!("error: failed to parse DSN: {e}");
            std::process::exit(1);
        }
    };
    for w in &parsed.warnings {
        eprintln!("warning: {w}");
    }
    // Design-rule minimums from .kicad_dru (explicit or sibling of the board).
    let dru_path = args.dru.clone().or_else(|| {
        kicad_board
            .as_ref()
            .and_then(|b| autoroute_dsn::dru::sibling_dru(b))
    });
    if let Some(dp) = dru_path {
        match std::fs::read_to_string(&dp).map_err(|e| e.to_string()).and_then(|t| {
            autoroute_dsn::dru::parse_dru(&t)
        }) {
            Ok(mins) => {
                for name in &mins.unsupported {
                    eprintln!("dru: rule \"{name}\" not supported, ignored");
                }
                for cl in &mut parsed.board.classes {
                    if let Some(c) = mins.clearance {
                        if cl.clearance < c {
                            eprintln!("dru: class {} clearance {} -> {} um", cl.name, cl.clearance, c);
                            cl.clearance = c;
                        }
                    }
                    if let Some(w) = mins.track_width {
                        if cl.width < w {
                            eprintln!("dru: class {} width {} -> {} um", cl.name, cl.width, w);
                            cl.width = w;
                        }
                    }
                }
            }
            Err(e) => eprintln!("warning: cannot read dru {}: {e}", dp.display()),
        }
    }

    if let Some(cap) = args.clearance_cap {
        for cl in &mut parsed.board.classes {
            if cl.clearance > cap {
                eprintln!(
                    "clearance cap: class {} {} -> {} um",
                    cl.name, cl.clearance, cap
                );
                cl.clearance = cap;
            }
        }
    }
    let board = &parsed.board;

    if let Some(ses_path) = &args.render_ses {
        let ses_text = match std::fs::read_to_string(ses_path) {
            Ok(t) => t,
            Err(e) => {
                eprintln!("error: cannot read {}: {e}", ses_path.display());
                std::process::exit(1);
            }
        };
        let nets = match svg::parse_ses(&ses_text, board) {
            Ok(n) => n,
            Err(e) => {
                eprintln!("error: failed to parse SES: {e}");
                std::process::exit(1);
            }
        };
        // With --check, DRC the SES geometry itself (works on any session
        // file, including freerouting's).
        if args.check {
            let mut rr = autoroute_core::RouteResult::default();
            for sn in &nets {
                let net = match board.nets.iter().position(|n| n.name == sn.name) {
                    Some(i) => i,
                    None => {
                        eprintln!("warning: SES net {} not in board", sn.name);
                        continue;
                    }
                };
                let mut rn = autoroute_core::RoutedNet { net, ..Default::default() };
                for w in &sn.wires {
                    rn.segs.push(autoroute_core::RouteSeg {
                        layer: w.layer,
                        width: w.width,
                        pts: w.pts.clone(),
                    });
                }
                rn.vias = sn.vias.clone();
                rr.nets.push(rn);
            }
            let tol = 5.0;
            let mut v = autoroute_core::drc::check(board, &rr, tol);
            v.extend(autoroute_core::drc::check_pads(board, &rr, tol));
            v.extend(autoroute_core::drc::check_via_pads(board, &rr, tol));
            eprintln!("DRC on SES: {} violations", v.len());
            let mut by_kind: std::collections::HashMap<&str, usize> = Default::default();
            for viol in &v {
                *by_kind.entry(viol.kind).or_default() += 1;
            }
            for (k, n) in by_kind {
                eprintln!("  {k}: {n}");
            }
            for viol in v.iter().take(15) {
                eprintln!(
                    "  {} {} vs {} at ({:.0}, {:.0}): gap {:.1} < {:.1}",
                    viol.kind,
                    board.nets.get(viol.net_a).map(|n| n.name.as_str()).unwrap_or("?"),
                    board.nets.get(viol.net_b).map(|n| n.name.as_str()).unwrap_or("?"),
                    viol.at[0],
                    viol.at[1],
                    viol.gap,
                    viol.required
                );
            }
        }
        let out = args
            .svg
            .clone()
            .unwrap_or_else(|| ses_path.with_extension("svg"));
        let svg_text = svg::render_svg(board, &nets);
        if let Err(e) = std::fs::write(&out, svg_text) {
            eprintln!("error: cannot write {}: {e}", out.display());
            std::process::exit(1);
        }
        eprintln!("wrote {}", out.display());
        return;
    }

    let routable = board
        .nets
        .iter()
        .filter(|n| n.pads.len() >= 2 && !n.fixed)
        .count();
    eprintln!(
        "board: {} layers, {} pads, {} nets ({} routable)",
        board.layers.len(),
        board.pads.len(),
        board.nets.len(),
        routable
    );
    if parsed.fixed_nets > 0 {
        eprintln!(
            "keeping existing routing on {} nets (use --reroute-wired to route from scratch)",
            parsed.fixed_nets
        );
    }
    if parsed.poured_nets > 0 {
        eprintln!("leaving {} nets to their copper pours", parsed.poured_nets);
    }

    let cfg = RouterConfig {
        pitch: args.grid,
        max_iterations: args.iterations,
        via_cost: args.via_cost,
        skip_classes: args.skip_classes.clone(),
        only_nets: args.nets.clone(),
        optimize_passes: args.optimize,
        ..Default::default()
    };
    eprintln!("grid pitch: {:.0} um", cfg.effective_pitch(board));

    let cancel = AtomicBool::new(false);
    let t0 = Instant::now();
    let verbose = args.verbose;
    let result = route_board(board, cfg, &cancel, &mut |p| {
        if verbose && p.end_of_iteration {
            eprintln!(
                "iteration {}: {} nets, {} in conflict, {} failed",
                p.iteration + 1,
                p.nets_total,
                p.result.conflict_nets,
                p.result.failed_nets
            );
        }
    });
    let dt = t0.elapsed();

    let total_vias: usize = result.nets.iter().map(|n| n.vias.len()).sum();
    eprintln!(
        "routed in {:.2}s: {} iterations, {} vias, {} failed nets, {} conflicted nets",
        dt.as_secs_f64(),
        result.iterations,
        total_vias,
        result.failed_nets,
        result.conflict_nets
    );

    if args.check {
        let tol = 5.0; // um, grid rounding tolerance
        let mut v = autoroute_core::drc::check(board, &result, tol);
        v.extend(autoroute_core::drc::check_pads(board, &result, tol));
        v.extend(autoroute_core::drc::check_via_pads(board, &result, tol));
        if v.is_empty() {
            eprintln!("DRC: clean ({} tolerance um)", tol);
        } else {
            eprintln!("DRC: {} violations", v.len());
            for viol in v.iter().take(20) {
                eprintln!(
                    "  {} {} vs {} layer {:?} at ({:.0}, {:.0}): gap {:.1} < {:.1} um",
                    viol.kind,
                    board.nets.get(viol.net_a).map(|n| n.name.as_str()).unwrap_or("?"),
                    board.nets.get(viol.net_b).map(|n| n.name.as_str()).unwrap_or("?"),
                    viol.layer,
                    viol.at[0],
                    viol.at[1],
                    viol.gap,
                    viol.required
                );
            }
        }
    }

    let design_name = dsn_path
        .file_name()
        .map(|s| s.to_string_lossy().to_string())
        .unwrap_or_else(|| "board.dsn".to_string());
    let ses = write_ses(board, &result, &design_name);
    let out = args
        .output
        .unwrap_or_else(|| args.input.with_extension("ses"));
    if let Err(e) = std::fs::write(&out, ses) {
        eprintln!("error: cannot write {}: {e}", out.display());
        std::process::exit(1);
    }
    eprintln!("wrote {}", out.display());

    // Write the routed result back into a copy of the .kicad_pcb.
    if let Some(kicad_pcb) = kicad_board {
        let stem = kicad_pcb
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "board".into());
        let routed = kicad_pcb.with_file_name(format!("{stem}_routed.kicad_pcb"));
        eprintln!("importing SES back via KiCad...");
        match autoroute_dsn::kicad::import_ses(&kicad_pcb, &out, &routed) {
            Ok(()) => eprintln!("wrote {}", routed.display()),
            Err(e) => eprintln!("error: SES write-back failed: {e}"),
        }
    }

    if !result.complete {
        std::process::exit(2);
    }
}
