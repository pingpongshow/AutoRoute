"""AutoRoute KiCad action plugin.

Adds an "AutoRoute board" button to pcbnew that:
  1. exports the current board to Specctra DSN,
  2. runs the AutoRoute binary on it,
  3. imports the resulting SES session back into the board.

Install: copy this file (and optionally the `autoroute` binary) into your
KiCad scripting/plugins directory, e.g. on macOS:
    ~/Documents/KiCad/9.0/scripting/plugins/
Set the binary location below or put `autoroute` on your PATH.
"""

import os
import shutil
import subprocess
import tempfile

import pcbnew
import wx

# Path to the autoroute binary; falls back to PATH lookup.
AUTOROUTE_BIN = os.environ.get("AUTOROUTE_BIN", "autoroute")


class AutoRoutePlugin(pcbnew.ActionPlugin):
    def defaults(self):
        self.name = "AutoRoute board"
        self.category = "Routing"
        self.description = "Route the board with AutoRoute (external Specctra router)"
        self.show_toolbar_button = True
        icon = os.path.join(os.path.dirname(__file__), "autoroute_icon.png")
        self.icon_file_name = icon if os.path.exists(icon) else ""

    def Run(self):
        board = pcbnew.GetBoard()
        binary = AUTOROUTE_BIN
        if not os.path.isabs(binary) and shutil.which(binary) is None:
            local = os.path.join(os.path.dirname(__file__), "autoroute")
            if os.path.exists(local):
                binary = local
            else:
                wx.MessageBox(
                    "autoroute binary not found.\n"
                    "Put it on PATH, next to this plugin, or set AUTOROUTE_BIN.",
                    "AutoRoute",
                    wx.ICON_ERROR,
                )
                return

        tmpdir = tempfile.mkdtemp(prefix="autoroute_")
        dsn_path = os.path.join(tmpdir, "board.dsn")
        ses_path = os.path.join(tmpdir, "board.ses")

        try:
            if not pcbnew.ExportSpecctraDSN(board, dsn_path):
                wx.MessageBox("Specctra DSN export failed.", "AutoRoute", wx.ICON_ERROR)
                return

            proc = subprocess.run(
                [binary, dsn_path, "-o", ses_path],
                capture_output=True,
                text=True,
                timeout=3600,
            )
            log = (proc.stderr or "") + (proc.stdout or "")
            if proc.returncode not in (0, 2) or not os.path.exists(ses_path):
                wx.MessageBox(
                    "AutoRoute failed:\n\n" + log[-2000:], "AutoRoute", wx.ICON_ERROR
                )
                return

            if not pcbnew.ImportSpecctraSES(board, ses_path):
                wx.MessageBox("SES import failed.", "AutoRoute", wx.ICON_ERROR)
                return

            pcbnew.Refresh()
            if proc.returncode == 2:
                wx.MessageBox(
                    "Routing finished INCOMPLETE (some nets failed or conflict).\n"
                    "Partial result imported.\n\n" + log[-1000:],
                    "AutoRoute",
                    wx.ICON_WARNING,
                )
            else:
                wx.MessageBox(
                    "Routing complete.\n\n" + log[-1000:], "AutoRoute", wx.ICON_INFORMATION
                )
        except subprocess.TimeoutExpired:
            wx.MessageBox("AutoRoute timed out.", "AutoRoute", wx.ICON_ERROR)
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)


AutoRoutePlugin().register()
