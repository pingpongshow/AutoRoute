"""AutoRoute KiCad action plugin.

Adds an "AutoRoute board" toolbar button to pcbnew that opens an options
dialog, then: exports the board to Specctra DSN, runs the AutoRoute binary,
and imports the resulting SES session back into the live board (undoable).

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


class OptionsDialog(wx.Dialog):
    def __init__(self, parent):
        super().__init__(parent, title="AutoRoute options")
        grid = wx.FlexGridSizer(0, 2, 6, 8)

        grid.Add(wx.StaticText(self, label="Grid pitch (um, blank = auto):"))
        self.grid_pitch = wx.TextCtrl(self, value="120")
        grid.Add(self.grid_pitch, flag=wx.EXPAND)

        grid.Add(wx.StaticText(self, label="Clearance cap (um, blank = none):"))
        self.clearance_cap = wx.TextCtrl(self, value="")
        grid.Add(self.clearance_cap, flag=wx.EXPAND)

        grid.Add(wx.StaticText(self, label="Skip classes (comma separated):"))
        self.skip_classes = wx.TextCtrl(self, value="")
        grid.Add(self.skip_classes, flag=wx.EXPAND)

        grid.Add(wx.StaticText(self, label="Route only nets (glob, blank = all):"))
        self.only_nets = wx.TextCtrl(self, value="")
        grid.Add(self.only_nets, flag=wx.EXPAND)

        self.keep_wiring = wx.CheckBox(self, label="Keep existing routing / pours")
        self.keep_wiring.SetValue(True)
        grid.Add(self.keep_wiring)
        grid.Add(wx.StaticText(self, label=""))

        self.optimize = wx.CheckBox(self, label="Post-route optimization pass")
        self.optimize.SetValue(True)
        grid.Add(self.optimize)
        grid.Add(wx.StaticText(self, label=""))

        buttons = self.CreateButtonSizer(wx.OK | wx.CANCEL)
        outer = wx.BoxSizer(wx.VERTICAL)
        outer.Add(grid, flag=wx.ALL | wx.EXPAND, border=12)
        outer.Add(buttons, flag=wx.ALL | wx.ALIGN_RIGHT, border=8)
        self.SetSizerAndFit(outer)

    def flags(self):
        out = []
        if self.grid_pitch.GetValue().strip():
            out += ["--grid", self.grid_pitch.GetValue().strip()]
        if self.clearance_cap.GetValue().strip():
            out += ["--clearance-cap", self.clearance_cap.GetValue().strip()]
        if self.skip_classes.GetValue().strip():
            out += ["--skip-classes", self.skip_classes.GetValue().strip()]
        if self.only_nets.GetValue().strip():
            out += ["--nets", self.only_nets.GetValue().strip()]
        if not self.keep_wiring.GetValue():
            out += ["--reroute-wired"]
        if not self.optimize.GetValue():
            out += ["--optimize", "0"]
        return out


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

        dlg = OptionsDialog(None)
        if dlg.ShowModal() != wx.ID_OK:
            dlg.Destroy()
            return
        flags = dlg.flags()
        dlg.Destroy()

        tmpdir = tempfile.mkdtemp(prefix="autoroute_")
        dsn_path = os.path.join(tmpdir, "board.dsn")
        ses_path = os.path.join(tmpdir, "board.ses")

        try:
            if not pcbnew.ExportSpecctraDSN(board, dsn_path):
                wx.MessageBox("Specctra DSN export failed.", "AutoRoute", wx.ICON_ERROR)
                return

            # Pass the project's .kicad_dru if it exists.
            brd_file = board.GetFileName()
            if brd_file:
                dru = os.path.splitext(brd_file)[0] + ".kicad_dru"
                if os.path.exists(dru):
                    flags += ["--dru", dru]

            proc = subprocess.run(
                [binary, dsn_path, "-o", ses_path] + flags,
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
                    "Partial result imported.\n\n" + log[-1200:],
                    "AutoRoute",
                    wx.ICON_WARNING,
                )
            else:
                wx.MessageBox(
                    "Routing complete.\n\n" + log[-1200:], "AutoRoute", wx.ICON_INFORMATION
                )
        except subprocess.TimeoutExpired:
            wx.MessageBox("AutoRoute timed out.", "AutoRoute", wx.ICON_ERROR)
        finally:
            shutil.rmtree(tmpdir, ignore_errors=True)


AutoRoutePlugin().register()
