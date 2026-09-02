//! AutoRoute GUI: load a Specctra DSN, watch the router work live, export SES.

#![cfg_attr(not(debug_assertions), windows_subsystem = "windows")]

use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

use eframe::egui::{self, Color32, Pos2, Rect, Stroke, Vec2};

use autoroute_core::{Board, PadShape, RouteResult, Router, RouterConfig};
use autoroute_dsn::{parse_dsn, write_ses};

fn main() -> eframe::Result {
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_inner_size([1280.0, 840.0])
            .with_title("AutoRoute"),
        ..Default::default()
    };
    eframe::run_native(
        "AutoRoute",
        options,
        Box::new(|_cc| {
            let mut app = App::default();
            if let Some(path) = std::env::args().nth(1) {
                app.open_file(PathBuf::from(path));
            }
            Ok(Box::new(app))
        }),
    )
}

#[derive(Clone, Default)]
struct Shared {
    result: Option<RouteResult>,
    violations: Vec<autoroute_core::drc::Violation>,
    status: String,
    running: bool,
}

#[derive(Default)]
struct App {
    board: Option<Arc<Board>>,
    board_path: Option<PathBuf>,
    /// Original .kicad_pcb when the board was opened from one.
    kicad_path: Option<PathBuf>,
    shared: Arc<Mutex<Shared>>,
    cancel: Arc<AtomicBool>,
    // View transform: screen = pan + world * zoom (y flipped).
    pan: Vec2,
    zoom: f32,
    fitted: bool,
    grid_pitch: String,
    status: String,
    show_layers: Vec<bool>,
    /// Net-name filter ("*" glob); matching nets highlighted, rest dimmed.
    net_filter: String,
    show_ratsnest: bool,
    /// World point to center on next frame (from clicking a DRC row).
    goto_point: Option<[f64; 2]>,
    show_drc_panel: bool,
}

const LAYER_COLORS: [Color32; 6] = [
    Color32::from_rgb(214, 73, 73),   // F.Cu red
    Color32::from_rgb(77, 127, 214),  // B.Cu blue
    Color32::from_rgb(87, 166, 74),   // In1
    Color32::from_rgb(214, 158, 64),  // In2
    Color32::from_rgb(158, 94, 214),  // In3
    Color32::from_rgb(64, 190, 190),  // In4
];

fn layer_color(l: usize) -> Color32 {
    LAYER_COLORS[l % LAYER_COLORS.len()]
}

impl App {
    fn open_file(&mut self, path: PathBuf) {
        // .kicad_pcb: convert to DSN via KiCad's own exporter first.
        self.kicad_path = None;
        let path = if path.extension().map(|e| e == "kicad_pcb").unwrap_or(false) {
            self.kicad_path = Some(path.clone());
            let dsn = std::env::temp_dir().join(format!(
                "{}.autoroute.dsn",
                path.file_stem().map(|s| s.to_string_lossy().to_string()).unwrap_or_default()
            ));
            match autoroute_dsn::kicad::export_dsn(&path, &dsn) {
                Ok(()) => dsn,
                Err(e) => {
                    self.status = format!("KiCad export failed: {e}");
                    return;
                }
            }
        } else {
            path
        };
        match std::fs::read_to_string(&path) {
            Ok(text) => match parse_dsn(&text) {
                Ok(parsed) => {
                    let nl = parsed.board.layers.len();
                    self.status = format!(
                        "{}: {} layers, {} pads, {} nets{}",
                        path.file_name().map(|s| s.to_string_lossy()).unwrap_or_default(),
                        nl,
                        parsed.board.pads.len(),
                        parsed.board.nets.len(),
                        if parsed.warnings.is_empty() {
                            String::new()
                        } else {
                            format!(" ({} warnings)", parsed.warnings.len())
                        }
                    );
                    self.show_layers = vec![true; nl];
                    self.board = Some(Arc::new(parsed.board));
                    self.board_path = Some(path);
                    self.fitted = false;
                    *self.shared.lock().unwrap() = Shared::default();
                }
                Err(e) => self.status = format!("parse error: {e}"),
            },
            Err(e) => self.status = format!("read error: {e}"),
        }
    }

    fn start_routing(&mut self, ctx: &egui::Context) {
        let board = match &self.board {
            Some(b) => b.clone(),
            None => return,
        };
        {
            let mut sh = self.shared.lock().unwrap();
            if sh.running {
                return;
            }
            sh.running = true;
            sh.status = "routing...".into();
        }
        self.cancel.store(false, Ordering::Relaxed);
        let shared = self.shared.clone();
        let cancel = self.cancel.clone();
        let ctx = ctx.clone();
        let pitch: Option<f64> = self.grid_pitch.trim().parse().ok().filter(|p: &f64| *p > 10.0);

        std::thread::spawn(move || {
            let cfg = RouterConfig { pitch, ..Default::default() };
            let mut router = Router::new(&board, cfg);
            let (nl, nx, ny, p) = router.grid_info();
            {
                let mut sh = shared.lock().unwrap();
                sh.status = format!("grid {nx}x{ny}x{nl} @ {p:.0} um");
            }
            let t0 = Instant::now();
            let mut last_push = Instant::now() - Duration::from_secs(1);
            let result = router.route(&cancel, &mut |prog| {
                if last_push.elapsed() > Duration::from_millis(33) {
                    let mut sh = shared.lock().unwrap();
                    sh.result = Some(prog.result.clone());
                    sh.status = format!(
                        "iteration {} — net {}/{} — conflicts {} failed {}",
                        prog.iteration + 1,
                        prog.nets_done,
                        prog.nets_total,
                        prog.result.conflict_nets,
                        prog.result.failed_nets,
                    );
                    drop(sh);
                    ctx.request_repaint();
                    last_push = Instant::now();
                }
            });
            let mut result = result;
            if !cancel.load(Ordering::Relaxed) {
                autoroute_core::refine::refine(&board, &mut result);
            }
            let violations = {
                let tol = 5.0;
                let mut v = autoroute_core::drc::check(&board, &result, tol);
                v.extend(autoroute_core::drc::check_pads(&board, &result, tol));
                v.extend(autoroute_core::drc::check_via_pads(&board, &result, tol));
                v
            };
            let mut sh = shared.lock().unwrap();
            sh.violations = violations;
            let vias: usize = result.nets.iter().map(|n| n.vias.len()).sum();
            sh.status = if cancel.load(Ordering::Relaxed) {
                "stopped".to_string()
            } else if result.complete {
                format!(
                    "done in {:.2}s — {} iterations, {} vias",
                    t0.elapsed().as_secs_f64(),
                    result.iterations,
                    vias
                )
            } else {
                format!(
                    "incomplete after {} iterations — {} failed, {} conflicted",
                    result.iterations, result.failed_nets, result.conflict_nets
                )
            };
            sh.result = Some(result);
            sh.running = false;
            drop(sh);
            ctx.request_repaint();
        });
    }

    fn export_ses(&mut self) {
        let (board, path) = match (&self.board, &self.board_path) {
            (Some(b), Some(p)) => (b.clone(), p.clone()),
            _ => return,
        };
        let result = match &self.shared.lock().unwrap().result {
            Some(r) => r.clone(),
            None => {
                self.status = "nothing routed yet".into();
                return;
            }
        };
        let default_name = path
            .with_extension("ses")
            .file_name()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "board.ses".into());
        let dest = rfd::FileDialog::new()
            .set_file_name(&default_name)
            .set_directory(path.parent().unwrap_or(std::path::Path::new(".")))
            .save_file();
        if let Some(dest) = dest {
            let design = path
                .file_name()
                .map(|s| s.to_string_lossy().to_string())
                .unwrap_or_else(|| "board.dsn".into());
            let ses = write_ses(&board, &result, &design);
            match std::fs::write(&dest, ses) {
                Ok(_) => self.status = format!("wrote {}", dest.display()),
                Err(e) => self.status = format!("write error: {e}"),
            }
        }
    }

    fn export_kicad(&mut self) {
        let (board, kicad) = match (&self.board, &self.kicad_path) {
            (Some(b), Some(k)) => (b.clone(), k.clone()),
            _ => return,
        };
        let result = match &self.shared.lock().unwrap().result {
            Some(r) => r.clone(),
            None => return,
        };
        let stem = kicad
            .file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "board".into());
        let dest = rfd::FileDialog::new()
            .set_file_name(format!("{stem}_routed.kicad_pcb"))
            .set_directory(kicad.parent().unwrap_or(std::path::Path::new(".")))
            .save_file();
        let Some(dest) = dest else { return };
        let ses_tmp = std::env::temp_dir().join(format!("{stem}.autoroute.ses"));
        let ses = write_ses(&board, &result, &format!("{stem}.kicad_pcb"));
        if let Err(e) = std::fs::write(&ses_tmp, ses) {
            self.status = format!("write error: {e}");
            return;
        }
        match autoroute_dsn::kicad::import_ses(&kicad, &ses_tmp, &dest) {
            Ok(()) => self.status = format!("wrote {}", dest.display()),
            Err(e) => self.status = format!("KiCad write-back failed: {e}"),
        }
    }

    fn fit_view(&mut self, board: &Board, rect: Rect) {
        let (lo, hi) = board.bbox();
        let w = (hi[0] - lo[0]).max(1.0) as f32;
        let h = (hi[1] - lo[1]).max(1.0) as f32;
        let zoom = (rect.width() / w).min(rect.height() / h) * 0.92;
        self.zoom = zoom;
        let cx = (lo[0] + hi[0]) as f32 / 2.0;
        let cy = (lo[1] + hi[1]) as f32 / 2.0;
        // screen = pan + (x, -y) * zoom
        self.pan = rect.center().to_vec2() - Vec2::new(cx * zoom, -cy * zoom);
        self.fitted = true;
    }

    #[inline]
    fn to_screen(&self, p: [f64; 2]) -> Pos2 {
        Pos2::new(
            self.pan.x + p[0] as f32 * self.zoom,
            self.pan.y - p[1] as f32 * self.zoom,
        )
    }

    fn draw_board(
        &self,
        painter: &egui::Painter,
        board: &Board,
        result: Option<&RouteResult>,
        violations: &[autoroute_core::drc::Violation],
    ) {
        let z = self.zoom;
        let filter = self.net_filter.trim().to_string();
        let net_dim = |net: usize| -> bool {
            !filter.is_empty()
                && !board
                    .nets
                    .get(net)
                    .map(|n| autoroute_core::router::glob_match(&filter, &n.name))
                    .unwrap_or(false)
        };
        // Boundary.
        if board.boundary.len() >= 2 {
            let mut pts: Vec<Pos2> = board.boundary.iter().map(|p| self.to_screen(*p)).collect();
            pts.push(pts[0]);
            painter.add(egui::Shape::line(
                pts,
                Stroke::new(1.5, Color32::from_rgb(180, 180, 90)),
            ));
        }

        let layer_visible = |l: usize| self.show_layers.get(l).copied().unwrap_or(true);

        // Routed tracks below pads.
        if let Some(result) = result {
            for rn in &result.nets {
                for seg in &rn.segs {
                    if !layer_visible(seg.layer) {
                        continue;
                    }
                    let mut color = layer_color(seg.layer);
                    if rn.in_conflict {
                        color = Color32::from_rgb(255, 40, 220);
                    }
                    if net_dim(rn.net) {
                        color = color.gamma_multiply(0.15);
                    }
                    let stroke = Stroke::new((seg.width as f32 * z).max(1.0), color);
                    let pts: Vec<Pos2> = seg.pts.iter().map(|p| self.to_screen(*p)).collect();
                    painter.add(egui::Shape::line(pts, stroke));
                }
                for v in &rn.vias {
                    let c = self.to_screen(*v);
                    let r = (board.rules.via_diameter as f32 / 2.0 * z).max(2.0);
                    painter.circle_filled(c, r, Color32::from_rgb(190, 190, 190));
                    painter.circle_filled(c, r * 0.45, Color32::from_rgb(40, 40, 40));
                }
            }
        }

        // Pads.
        for pad in &board.pads {
            let visible = pad.layers.iter().any(|&l| layer_visible(l));
            if !visible {
                continue;
            }
            if pad.is_pour {
                if let PadShape::Polygon { pts } = &pad.shape {
                    let spts: Vec<Pos2> = pts.iter().map(|p| self.to_screen(*p)).collect();
                    let c = layer_color(pad.layers[0]).gamma_multiply(0.15);
                    painter.add(egui::Shape::convex_polygon(spts, c, Stroke::NONE));
                }
                continue;
            }
            let color = if pad.layers.len() > 1 {
                Color32::from_rgb(200, 170, 60) // through-hole gold
            } else {
                layer_color(pad.layers[0]).gamma_multiply(0.85)
            };
            match &pad.shape {
                PadShape::Circle { center, diameter } => {
                    painter.circle_filled(
                        self.to_screen(*center),
                        (*diameter as f32 / 2.0 * z).max(1.0),
                        color,
                    );
                }
                PadShape::Polygon { pts } => {
                    let spts: Vec<Pos2> = pts.iter().map(|p| self.to_screen(*p)).collect();
                    painter.add(egui::Shape::convex_polygon(spts, color, Stroke::NONE));
                }
                PadShape::Path { width, pts } => {
                    let spts: Vec<Pos2> = pts.iter().map(|p| self.to_screen(*p)).collect();
                    painter.add(egui::Shape::line(
                        spts,
                        Stroke::new((*width as f32 * z).max(1.0), color),
                    ));
                }
            }
        }

        // Ratsnest: chain pad centers of nets with no routed geometry.
        if self.show_ratsnest {
            let ghost = Stroke::new(1.0, Color32::from_rgba_unmultiplied(200, 200, 200, 90));
            for (ni, net) in board.nets.iter().enumerate() {
                if net.fixed || net.pads.len() < 2 {
                    continue;
                }
                let routed = result
                    .and_then(|r| r.nets.get(ni))
                    .map(|rn| !rn.segs.is_empty())
                    .unwrap_or(false);
                if routed {
                    continue;
                }
                let centers: Vec<Pos2> = net
                    .pads
                    .iter()
                    .filter(|&&pi| !board.pads[pi].is_pour)
                    .map(|&pi| self.to_screen(board.pads[pi].shape.center()))
                    .collect();
                for w in centers.windows(2) {
                    painter.line_segment([w[0], w[1]], ghost);
                }
            }
        }

        // DRC violation markers.
        for v in violations {
            let c = self.to_screen(v.at);
            painter.circle_stroke(c, 7.0, Stroke::new(2.0, Color32::from_rgb(255, 60, 60)));
        }
    }
}

impl eframe::App for App {
    fn update(&mut self, ctx: &egui::Context, _frame: &mut eframe::Frame) {
        let shared_snapshot = self.shared.lock().unwrap().clone();

        egui::TopBottomPanel::top("toolbar").show(ctx, |ui| {
            ui.horizontal(|ui| {
                if ui.button("Open…").clicked() {
                    if let Some(path) = rfd::FileDialog::new()
                        .add_filter("Boards", &["dsn", "kicad_pcb"])
                        .pick_file()
                    {
                        self.open_file(path);
                    }
                }
                let can_route = self.board.is_some() && !shared_snapshot.running;
                if ui.add_enabled(can_route, egui::Button::new("▶ Route")).clicked() {
                    self.start_routing(ctx);
                }
                if ui
                    .add_enabled(shared_snapshot.running, egui::Button::new("■ Stop"))
                    .clicked()
                {
                    self.cancel.store(true, Ordering::Relaxed);
                }
                if ui
                    .add_enabled(
                        shared_snapshot.result.is_some() && !shared_snapshot.running,
                        egui::Button::new("Export SES…"),
                    )
                    .clicked()
                {
                    self.export_ses();
                }
                if self.kicad_path.is_some()
                    && ui
                        .add_enabled(
                            shared_snapshot.result.is_some() && !shared_snapshot.running,
                            egui::Button::new("Export routed .kicad_pcb…"),
                        )
                        .clicked()
                {
                    self.export_kicad();
                }
                ui.separator();
                ui.label("grid (um):");
                ui.add(egui::TextEdit::singleline(&mut self.grid_pitch).desired_width(60.0))
                    .on_hover_text("Grid pitch in um; empty = auto from design rules");
                if ui.button("Fit").clicked() {
                    self.fitted = false;
                }
                ui.separator();
                ui.label("net:");
                ui.add(egui::TextEdit::singleline(&mut self.net_filter).desired_width(90.0))
                    .on_hover_text("Highlight nets matching this glob (e.g. ETH_*)");
                ui.checkbox(&mut self.show_ratsnest, "ratsnest");
                ui.checkbox(&mut self.show_drc_panel, "DRC");
                ui.separator();
                if let Some(board) = &self.board {
                    let names: Vec<String> = board.layers.clone();
                    for (l, name) in names.iter().enumerate() {
                        if self.show_layers.len() <= l {
                            self.show_layers.push(true);
                        }
                        let mut on = self.show_layers[l];
                        let text = egui::RichText::new(name).color(layer_color(l));
                        if ui.checkbox(&mut on, text).changed() {
                            self.show_layers[l] = on;
                        }
                    }
                }
            });
        });

        egui::TopBottomPanel::bottom("status").show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(&self.status);
                if !shared_snapshot.status.is_empty() {
                    ui.separator();
                    ui.label(&shared_snapshot.status);
                }
            });
        });

        if self.show_drc_panel {
            egui::SidePanel::right("drc").min_width(230.0).show(ctx, |ui| {
                ui.heading(format!("DRC: {}", shared_snapshot.violations.len()));
                egui::ScrollArea::vertical().show(ui, |ui| {
                    let board = self.board.clone();
                    for v in shared_snapshot.violations.iter() {
                        let name = |n: usize| {
                            board
                                .as_ref()
                                .and_then(|b| b.nets.get(n))
                                .map(|nn| nn.name.clone())
                                .unwrap_or_else(|| "?".into())
                        };
                        let label = format!(
                            "{} {} / {}\n  gap {:.0} < {:.0} um",
                            v.kind,
                            name(v.net_a),
                            name(v.net_b),
                            v.gap,
                            v.required
                        );
                        if ui.selectable_label(false, label).clicked() {
                            self.goto_point = Some(v.at);
                        }
                    }
                    if shared_snapshot.violations.is_empty() {
                        ui.label("clean");
                    }
                });
            });
        }

        egui::CentralPanel::default()
            .frame(egui::Frame::default().fill(Color32::from_rgb(24, 26, 30)))
            .show(ctx, |ui| {
                let (response, painter) =
                    ui.allocate_painter(ui.available_size(), egui::Sense::click_and_drag());
                let rect = response.rect;

                if let Some(board) = self.board.clone() {
                    if !self.fitted {
                        self.fit_view(&board, rect);
                    }
                    if let Some(gp) = self.goto_point.take() {
                        self.zoom = self.zoom.max(0.05);
                        self.pan = rect.center().to_vec2()
                            - Vec2::new(gp[0] as f32 * self.zoom, -(gp[1] as f32) * self.zoom);
                    }
                    // Pan.
                    if response.dragged() {
                        self.pan += response.drag_delta();
                    }
                    // Zoom around the pointer.
                    let (scroll, zoom_mult, hover) = ui.input(|i| {
                        (i.raw_scroll_delta.y, i.zoom_delta(), i.pointer.hover_pos())
                    });
                    if response.hovered() {
                        let mut factor = zoom_mult;
                        if scroll.abs() > 0.0 {
                            factor *= (scroll * 0.003).exp();
                        }
                        if (factor - 1.0).abs() > 1e-4 {
                            if let Some(hp) = hover {
                                self.pan = hp.to_vec2() + (self.pan - hp.to_vec2()) * factor;
                                self.zoom *= factor;
                            }
                        }
                    }
                    self.draw_board(
                        &painter,
                        &board,
                        shared_snapshot.result.as_ref(),
                        &shared_snapshot.violations,
                    );
                } else {
                    painter.text(
                        rect.center(),
                        egui::Align2::CENTER_CENTER,
                        "Open a Specctra DSN file (KiCad: File > Export > Specctra DSN)",
                        egui::FontId::proportional(16.0),
                        Color32::GRAY,
                    );
                }
            });
    }
}
