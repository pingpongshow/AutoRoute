//! Direct .kicad_pcb import/export by driving KiCad's bundled Python
//! (`pcbnew` module) headlessly. This reuses KiCad's own Specctra converters,
//! so board interpretation always matches KiCad exactly.
//!
//! Requires a KiCad installation; set `KICAD_PYTHON` to override discovery.

use std::path::{Path, PathBuf};
use std::process::Command;

/// Locate a Python interpreter that can `import pcbnew`.
pub fn find_kicad_python() -> Result<PathBuf, String> {
    let mut candidates: Vec<PathBuf> = Vec::new();
    if let Some(p) = std::env::var_os("KICAD_PYTHON") {
        candidates.push(PathBuf::from(p));
    }
    // macOS app bundle.
    candidates.push(PathBuf::from(
        "/Applications/KiCad/KiCad.app/Contents/Frameworks/Python.framework/Versions/Current/bin/python3",
    ));
    // Linux / PATH pythons with pcbnew installed system-wide.
    candidates.push(PathBuf::from("python3"));
    candidates.push(PathBuf::from("/usr/bin/python3"));

    for cand in candidates {
        let ok = Command::new(&cand)
            .args(["-c", "import pcbnew"])
            .output()
            .map(|o| o.status.success())
            .unwrap_or(false);
        if ok {
            return Ok(cand);
        }
    }
    Err(
        "no Python with the pcbnew module found; install KiCad or set KICAD_PYTHON \
         to KiCad's bundled python3"
            .into(),
    )
}

fn run_pcbnew_script(script: &str, args: &[&Path]) -> Result<(), String> {
    let python = find_kicad_python()?;
    let mut cmd = Command::new(&python);
    cmd.arg("-c").arg(script);
    for a in args {
        cmd.arg(a);
    }
    let out = cmd
        .output()
        .map_err(|e| format!("failed to run {}: {e}", python.display()))?;
    if !out.status.success() {
        let stderr = String::from_utf8_lossy(&out.stderr);
        // Filter the harmless headless-wxWidgets assert noise.
        let msg: String = stderr
            .lines()
            .filter(|l| !l.contains("create wxApp") && !l.contains("stdpbase"))
            .collect::<Vec<_>>()
            .join("\n");
        return Err(format!("pcbnew script failed: {msg}"));
    }
    Ok(())
}

/// Export a .kicad_pcb to a Specctra DSN file.
pub fn export_dsn(board: &Path, dsn_out: &Path) -> Result<(), String> {
    run_pcbnew_script(
        r#"
import sys
import pcbnew
board = pcbnew.LoadBoard(sys.argv[1])
ok = pcbnew.ExportSpecctraDSN(board, sys.argv[2])
sys.exit(0 if ok else 1)
"#,
        &[board, dsn_out],
    )
}

/// Import a routed SES session into a copy of the board, saved to `out`.
pub fn import_ses(board: &Path, ses: &Path, out: &Path) -> Result<(), String> {
    run_pcbnew_script(
        r#"
import sys
import pcbnew
board = pcbnew.LoadBoard(sys.argv[1])
ok = pcbnew.ImportSpecctraSES(board, sys.argv[2])
if not ok:
    sys.exit(1)
pcbnew.SaveBoard(sys.argv[3], board)
"#,
        &[board, ses, out],
    )
}
