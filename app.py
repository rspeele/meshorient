"""frame2solid — GUI for turning a pistol-frame scan into a clean,
watertight 'subtraction solid' for grip transplants.

Run:  python app.py            (optionally: python app.py myproject.json)

Steps (radio buttons, top right):
  1 Load/Orient — load scan, PCA auto-orient, fix with 90-deg rotations/flips.
                  Goal: X = bore axis, Y = across frame (symmetry plane Y=0),
                  Z = vertical. Views show the three projections.
  2 Silhouette  — extract the side-view OUTER outline. Windows and holes are
                  filled automatically. Export DXF for CAD tracing if wanted.
  3 Regions     — thickness heatmap from the scan. Drag rectangles to assign
                  plateau widths (suggested from measurement; override in the
                  width box, which edits the LAST region). Base width applies
                  everywhere else.
  4 Extras      — add clearance solids: drag = tilted box (mag path, levers),
                  click = Y-cylinder (grip screws, pins). Dims via text boxes
                  (they edit the LAST extra).
  5 Build       — voxel SDF build -> watertight STL. Inspect cross-sections
                  with the slider before saving.
"""
from __future__ import annotations

import json
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.widgets import (Button, TextBox, RadioButtons,
                                RectangleSelector, Slider)
from matplotlib.patches import Polygon as MplPolygon, Circle as MplCircle

import core
from meshio_lite import load_mesh, save_stl

STEPS = ["1 Load/Orient", "2 Silhouette", "3 Regions", "4 Extras", "5 Build"]


def _try_filedialog(save=False, initial=""):
    """File dialog via tkinter when available (Windows), else None."""
    try:
        import tkinter as tk
        from tkinter import filedialog
        root = tk.Tk()
        root.withdraw()
        if save:
            p = filedialog.asksaveasfilename(initialfile=initial)
        else:
            p = filedialog.askopenfilename()
        root.destroy()
        return p or None
    except Exception:
        return None


class App:
    def __init__(self):
        self.fig = plt.figure("frame2solid", figsize=(15, 8.5))

        # ---------- state
        self.scan_path = ""
        self.orig_verts = None
        self.orig_faces = None
        self.M = np.eye(3)            # orientation applied to scan
        self.offset = np.zeros(3)     # translation applied after M
        self.verts = None             # oriented verts (cache)
        self.sil = None               # core.Silhouette
        self.wmap = None              # (wmap, x0, z0, px)
        self.base_width = None
        self.regions = []
        self.extras = []
        self.result = None            # (verts, faces, report, sdf_pack)
        self.step = 0
        self.extra_kind = "box (drag)"
        self._radio_guard = False

        # ---------- axes
        self.ax_main = self.fig.add_axes([0.05, 0.16, 0.58, 0.76])
        self.ax_side = self.fig.add_axes([0.05, 0.55, 0.27, 0.37])
        self.ax_front = self.fig.add_axes([0.36, 0.55, 0.27, 0.37])
        self.ax_top = self.fig.add_axes([0.05, 0.10, 0.27, 0.37])
        self.orient_axes = [self.ax_side, self.ax_front, self.ax_top]

        self.txt_status = self.fig.text(0.05, 0.035, "Load a scan to begin.",
                                        fontsize=10, family="monospace",
                                        va="bottom", wrap=True)

        # ---------- widget layout helpers
        self._wy = 0.90

        def slot(h=0.045, dy=0.012, split=None):
            self._wy -= h + dy
            if split is None:
                return [0.67, self._wy, 0.31, h]
            x0, x1 = split
            return [x0, self._wy, x1, h]

        # ---------- step selector
        ax = self.fig.add_axes([0.67, 0.76, 0.31, 0.17])
        ax.set_title("frame2solid", fontsize=11, weight="bold")
        self.radio_step = RadioButtons(ax, STEPS, active=0)
        self.radio_step.on_clicked(self._on_step)
        self._wy = 0.745

        # ---------- step 1 widgets
        self.w1 = []
        r = slot()
        self.tb_path = TextBox(self.fig.add_axes(r), "scan ",
                               initial="synthetic_frame_scan.stl")
        self.w1.append(self.tb_path)
        r = slot(split=(0.67, 0.145))
        self.bt_browse = Button(self.fig.add_axes(r), "Browse…")
        self.bt_browse.on_clicked(self._on_browse)
        self.bt_load = Button(self.fig.add_axes([0.835, r[1], 0.145, r[3]]),
                              "Load scan")
        self.bt_load.on_clicked(self._on_load)
        self.w1 += [self.bt_browse, self.bt_load]
        r = slot()
        self.bt_pca = Button(self.fig.add_axes(r), "Auto-orient (PCA)")
        self.bt_pca.on_clicked(self._on_pca)
        self.w1.append(self.bt_pca)
        r = slot(split=(0.67, 0.095))
        self.bt_rx = Button(self.fig.add_axes(r), "rot X 90°")
        self.bt_ry = Button(self.fig.add_axes([0.775, r[1], 0.095, r[3]]), "rot Y 90°")
        self.bt_rz = Button(self.fig.add_axes([0.88, r[1], 0.095, r[3]]), "rot Z 90°")
        self.bt_rx.on_clicked(lambda e: self._rotate("x"))
        self.bt_ry.on_clicked(lambda e: self._rotate("y"))
        self.bt_rz.on_clicked(lambda e: self._rotate("z"))
        self.w1 += [self.bt_rx, self.bt_ry, self.bt_rz]
        r = slot(split=(0.67, 0.145))
        self.bt_flipx = Button(self.fig.add_axes(r), "flip X (180° about Z)")
        self.bt_flipx.on_clicked(lambda e: self._rotate("z", 180))
        self.bt_center = Button(self.fig.add_axes([0.835, r[1], 0.145, r[3]]),
                                "re-center")
        self.bt_center.on_clicked(self._on_center)
        self.w1 += [self.bt_flipx, self.bt_center]

        self._wy = 0.745
        # ---------- step 2 widgets
        self.w2 = []
        r = slot(split=(0.67, 0.095))
        self.tb_px = TextBox(self.fig.add_axes(r), "px mm ", initial="0.15")
        self.tb_close = TextBox(self.fig.add_axes([0.805, r[1], 0.06, r[3]]),
                                "close ", initial="1.5")
        self.tb_simp = TextBox(self.fig.add_axes([0.92, r[1], 0.055, r[3]]),
                               "simp ", initial="0.3")
        self.w2 += [self.tb_px, self.tb_close, self.tb_simp]
        r = slot(split=(0.67, 0.145))
        self.bt_extract = Button(self.fig.add_axes(r), "Extract silhouette")
        self.bt_extract.on_clicked(self._on_extract)
        self.bt_dxf = Button(self.fig.add_axes([0.835, r[1], 0.145, r[3]]),
                             "Export DXF")
        self.bt_dxf.on_clicked(self._on_dxf)
        self.w2 += [self.bt_extract, self.bt_dxf]

        self._wy = 0.745
        # ---------- step 3 widgets
        self.w3 = []
        r = slot(split=(0.67, 0.13))
        self.tb_base = TextBox(self.fig.add_axes(r), "base w ", initial="")
        self.tb_base.on_submit(self._on_base_w)
        self.tb_rw = TextBox(self.fig.add_axes([0.87, r[1], 0.10, r[3]]),
                             "last w ", initial="")
        self.tb_rw.on_submit(self._on_region_w)
        self.w3 += [self.tb_base, self.tb_rw]
        r = slot(split=(0.67, 0.145))
        self.bt_rdel = Button(self.fig.add_axes(r), "Delete last region")
        self.bt_rdel.on_clicked(self._on_region_del)
        self.bt_rclr = Button(self.fig.add_axes([0.835, r[1], 0.145, r[3]]),
                              "Clear regions")
        self.bt_rclr.on_clicked(self._on_region_clr)
        self.w3 += [self.bt_rdel, self.bt_rclr]

        self._wy = 0.745
        # ---------- step 4 widgets
        self.w4 = []
        r = slot(h=0.075)
        ax4 = self.fig.add_axes(r)
        self.radio_extra = RadioButtons(ax4, ["box (drag)", "cyl-Y (click)"],
                                        active=0)
        self.radio_extra.on_clicked(self._on_extra_kind)
        self.w4.append(self.radio_extra)
        r = slot(split=(0.67, 0.085))
        self.tb_yw = TextBox(self.fig.add_axes(r), "y-width ", initial="20")
        self.tb_tilt = TextBox(self.fig.add_axes([0.80, r[1], 0.06, r[3]]),
                               "tilt° ", initial="0")
        self.tb_dia = TextBox(self.fig.add_axes([0.92, r[1], 0.055, r[3]]),
                              "dia ", initial="5")
        self.tb_yw.on_submit(self._on_extra_edit)
        self.tb_tilt.on_submit(self._on_extra_edit)
        self.tb_dia.on_submit(self._on_extra_edit)
        self.w4 += [self.tb_yw, self.tb_tilt, self.tb_dia]
        r = slot(split=(0.67, 0.10))
        self.tb_ylen = TextBox(self.fig.add_axes(r), "cyl len ", initial="60")
        self.tb_ylen.on_submit(self._on_extra_edit)
        self.bt_edel = Button(self.fig.add_axes([0.80, r[1], 0.08, r[3]]),
                              "Del last")
        self.bt_edel.on_clicked(self._on_extra_del)
        self.bt_eclr = Button(self.fig.add_axes([0.895, r[1], 0.08, r[3]]),
                              "Clear")
        self.bt_eclr.on_clicked(self._on_extra_clr)
        self.w4 += [self.tb_ylen, self.bt_edel, self.bt_eclr]

        self._wy = 0.745
        # ---------- step 5 widgets
        self.w5 = []
        r = slot(split=(0.67, 0.10))
        self.tb_vox = TextBox(self.fig.add_axes(r), "voxel ", initial="0.3")
        self.tb_clr = TextBox(self.fig.add_axes([0.85, r[1], 0.10, r[3]]),
                              "clearance ", initial="0.15")
        self.w5 += [self.tb_vox, self.tb_clr]
        r = slot()
        self.bt_build = Button(self.fig.add_axes(r), "BUILD solid")
        self.bt_build.on_clicked(self._on_build)
        self.w5.append(self.bt_build)
        r = slot(h=0.035)
        self.sl_sec = Slider(self.fig.add_axes(r), "section ", -1.0, 1.0,
                             valinit=0.0)
        self.sl_sec.on_changed(lambda v: self._draw())
        self.w5.append(self.sl_sec)
        r = slot(h=0.06)
        self.radio_sec = RadioButtons(self.fig.add_axes(r),
                                      ["side section (Y=…)", "cross section (X=…)"],
                                      active=0)
        self.radio_sec.on_clicked(lambda l: self._draw())
        self.w5.append(self.radio_sec)
        r = slot()
        self.tb_out = TextBox(self.fig.add_axes(r), "out ",
                              initial="frame_solid.stl")
        self.w5.append(self.tb_out)
        r = slot()
        self.bt_save = Button(self.fig.add_axes(r), "Save STL")
        self.bt_save.on_clicked(self._on_save)
        self.w5.append(self.bt_save)

        # ---------- always-visible project row
        self.tb_proj = TextBox(self.fig.add_axes([0.67, 0.075, 0.31, 0.045]),
                               "proj ", initial="project.json")
        self.bt_psave = Button(self.fig.add_axes([0.67, 0.02, 0.145, 0.045]),
                               "Save project")
        self.bt_pload = Button(self.fig.add_axes([0.835, 0.02, 0.145, 0.045]),
                               "Load project")
        self.bt_psave.on_clicked(self._on_proj_save)
        self.bt_pload.on_clicked(self._on_proj_load)

        # ---------- selectors / events
        self.rsel = RectangleSelector(self.ax_main, self._on_rect,
                                      useblit=False, button=[1],
                                      interactive=False, minspanx=1,
                                      minspany=1, spancoords="data")
        self.rsel.set_active(False)
        self.fig.canvas.mpl_connect("button_press_event", self._on_click)

        self._set_step(0)

    # ================================================== step handling
    @staticmethod
    def _enable(w, flag):
        # Disable hidden widgets so they can't intercept clicks aimed at the
        # widgets drawn in the same screen position on another step.
        # RadioButtons.set_active(index) means "select option", so set the
        # underlying Widget active flag directly for those.
        from matplotlib.widgets import RadioButtons as _RB
        if isinstance(w, _RB):
            w._active = flag
        elif hasattr(w, "set_active"):
            w.set_active(flag)

    def _widgets_for(self, i):
        return [self.w1, self.w2, self.w3, self.w4, self.w5][i]

    def _set_step(self, i):
        self.step = i
        if STEPS.index(self.radio_step.value_selected) != i:
            self._radio_guard = True
            try:
                self.radio_step.set_active(i)
            finally:
                self._radio_guard = False
        for group in (self.w1, self.w2, self.w3, self.w4, self.w5):
            for w in group:
                w.ax.set_visible(False)
                self._enable(w, False)
        for w in self._widgets_for(i):
            w.ax.set_visible(True)
            self._enable(w, True)
        show_orient = (i == 0)
        for ax in self.orient_axes:
            ax.set_visible(show_orient)
        self.ax_main.set_visible(not show_orient)
        self.rsel.set_active(i in (2, 3))
        if i == 2 and self.sil is not None and self.wmap is None:
            self._compute_wmap()
        self._draw()

    def _on_step(self, label):
        if self._radio_guard:
            return
        self._set_step(STEPS.index(label))

    def _status(self, msg):
        import textwrap
        self.txt_status.set_text("\n".join(textwrap.wrap(msg, 92)[:3]))
        self.fig.canvas.draw_idle()

    # ================================================== step 1: load/orient
    def _on_browse(self, _):
        if self.step != 0:
            return
        p = _try_filedialog()
        if p:
            self.tb_path.set_val(p)
            self._on_load(None)

    def _on_load(self, _):
        if self.step != 0:
            return
        path = self.tb_path.text.strip()
        if not os.path.isfile(path):
            self._status(f"File not found: {path}")
            return
        try:
            v, f = load_mesh(path)
        except Exception as e:
            self._status(f"Load failed: {e}")
            return
        self.scan_path = path
        self.orig_verts, self.orig_faces = v, f
        self.M = np.eye(3)
        self.offset = np.zeros(3)
        self._apply_transform(center=True)
        self.sil = None
        self.wmap = None
        self.result = None
        self._status(f"Loaded {os.path.basename(path)}: "
                     f"{len(v)} verts, {len(f)} tris. Orient so that "
                     f"X=length, Y=across (thin), Z=height.")
        self._draw()

    def _apply_transform(self, center=False):
        self.verts = self.orig_verts @ self.M.T
        if center:
            lo, hi = self.verts.min(0), self.verts.max(0)
            self.offset = -(lo + hi) / 2
        self.verts = self.verts + self.offset

    def _on_pca(self, _):
        if self.orig_verts is None:
            return
        self.M = core.auto_orient(self.orig_verts)
        self._apply_transform(center=True)
        self._invalidate()
        self._status("PCA auto-orient applied. Check all three views; fix "
                     "with the 90° buttons if axes are swapped/flipped.")
        self._draw()

    def _rotate(self, axis, deg=90):
        if self.orig_verts is None:
            return
        self.M = core.rot_matrix(axis, deg) @ self.M
        self._apply_transform(center=True)
        self._invalidate()
        self._draw()

    def _on_center(self, _):
        if self.orig_verts is None:
            return
        self._apply_transform(center=True)
        self._invalidate()
        self._draw()

    def _invalidate(self):
        self.sil = None
        self.wmap = None
        self.result = None

    # ================================================== step 2: silhouette
    def _on_extract(self, _):
        if self.verts is None:
            self._status("Load a scan first (step 1).")
            return
        try:
            px = float(self.tb_px.text)
            cl = float(self.tb_close.text)
            sp = float(self.tb_simp.text)
            self.sil = core.extract_silhouette(self.verts, self.orig_faces,
                                               px=px, close_mm=cl,
                                               simplify_mm=sp)
        except Exception as e:
            self._status(f"Silhouette failed: {e}")
            return
        self.wmap = None
        self.result = None
        self._status(f"Silhouette OK: {len(self.sil.polygon)} outline points. "
                     f"Interior windows/holes filled automatically.")
        self._draw()

    def _on_dxf(self, _):
        if self.sil is None:
            self._status("Extract a silhouette first.")
            return
        out = os.path.splitext(self.scan_path or "outline")[0] + "_outline.dxf"
        core.export_outline_dxf(self.sil, out)
        self._status(f"DXF outline written: {out}")

    def _compute_wmap(self):
        self.wmap = core.width_map(self.verts, self.sil, px=0.6)
        if self.base_width is None:
            s = core.suggest_width(*self.wmap, rect=None, q=50)
            self.base_width = round(s, 2) if s else 20.0
            self.tb_base.set_val(str(self.base_width))

    # ================================================== step 3: regions
    def _on_base_w(self, text):
        try:
            self.base_width = float(text)
            self.result = None
            self._draw()
        except ValueError:
            pass

    def _on_region_w(self, text):
        if not self.regions:
            return
        try:
            self.regions[-1]["width"] = float(text)
            self.result = None
            self._draw()
        except ValueError:
            pass

    def _on_region_del(self, _):
        if self.regions:
            self.regions.pop()
            self.result = None
            self._draw()

    def _on_region_clr(self, _):
        self.regions = []
        self.result = None
        self._draw()

    def add_region(self, x0, z0, x1, z1):
        if self.wmap is None:
            return
        s = core.suggest_width(*self.wmap, rect=(x0, z0, x1, z1))
        w = round(s, 2) if s else (self.base_width or 20.0)
        self.regions.append({"x0": round(x0, 2), "z0": round(z0, 2),
                             "x1": round(x1, 2), "z1": round(z1, 2),
                             "width": w})
        self.tb_rw.set_val(str(w))
        self.result = None
        self._status(f"Region {len(self.regions)}: measured width ≈ {w} mm "
                     f"(95th pct). Override in 'last w' box.")
        self._draw()

    # ================================================== step 4: extras
    def _on_extra_kind(self, label):
        self.extra_kind = label

    def add_extra_box(self, x0, z0, x1, z1):
        try:
            yw = float(self.tb_yw.text)
            tilt = float(self.tb_tilt.text)
        except ValueError:
            yw, tilt = 20.0, 0.0
        self.extras.append({"kind": "box", "x0": round(x0, 2),
                            "z0": round(z0, 2), "x1": round(x1, 2),
                            "z1": round(z1, 2), "ywidth": yw,
                            "tilt_deg": tilt})
        self.result = None
        self._status(f"Extra {len(self.extras)}: box, y-width {yw}, "
                     f"tilt {tilt}°. Edit via text boxes (applies to last).")
        self._draw()

    def add_extra_cyl(self, x, z):
        try:
            dia = float(self.tb_dia.text)
            ylen = float(self.tb_ylen.text)
        except ValueError:
            dia, ylen = 5.0, 60.0
        self.extras.append({"kind": "cyl_y", "x": round(x, 2),
                            "z": round(z, 2), "dia": dia, "ylen": ylen})
        self.result = None
        self._status(f"Extra {len(self.extras)}: Y-cylinder ⌀{dia} × {ylen}.")
        self._draw()

    def _on_extra_edit(self, _text):
        if not self.extras:
            return
        e = self.extras[-1]
        try:
            if e["kind"] == "box":
                e["ywidth"] = float(self.tb_yw.text)
                e["tilt_deg"] = float(self.tb_tilt.text)
            else:
                e["dia"] = float(self.tb_dia.text)
                e["ylen"] = float(self.tb_ylen.text)
            self.result = None
            self._draw()
        except ValueError:
            pass

    def _on_extra_del(self, _):
        if self.extras:
            self.extras.pop()
            self.result = None
            self._draw()

    def _on_extra_clr(self, _):
        self.extras = []
        self.result = None
        self._draw()

    # ================================================== selectors
    def _on_rect(self, eclick, erelease):
        x0, z0 = eclick.xdata, eclick.ydata
        x1, z1 = erelease.xdata, erelease.ydata
        if None in (x0, z0, x1, z1):
            return
        if self.step == 2:
            self.add_region(x0, z0, x1, z1)
        elif self.step == 3 and self.extra_kind.startswith("box"):
            self.add_extra_box(x0, z0, x1, z1)

    def _on_click(self, event):
        if (self.step == 3 and self.extra_kind.startswith("cyl")
                and event.inaxes == self.ax_main and event.button == 1
                and self.fig.canvas.toolbar is not None
                and getattr(self.fig.canvas.toolbar, "mode", "") == ""):
            if event.xdata is not None:
                self.add_extra_cyl(event.xdata, event.ydata)
        elif (self.step == 3 and self.extra_kind.startswith("cyl")
                and event.inaxes == self.ax_main and event.button == 1
                and self.fig.canvas.toolbar is None):
            if event.xdata is not None:
                self.add_extra_cyl(event.xdata, event.ydata)

    # ================================================== step 5: build
    def _on_build(self, _):
        if self.sil is None:
            self._status("Need a silhouette first (step 2).")
            return
        if self.base_width is None:
            self._status("Set a base width first (step 3).")
            return
        try:
            vox = float(self.tb_vox.text)
            clr = float(self.tb_clr.text)
        except ValueError:
            self._status("Bad voxel/clearance value.")
            return
        self._status("Building… (large grids can take a minute)")
        self.fig.canvas.draw()
        try:
            v, f, rep, pack = core.build_solid(
                self.sil, self.base_width, self.regions, self.extras,
                clearance=clr, voxel=vox, return_sdf=True)
        except MemoryError:
            self._status("Out of memory — increase voxel size.")
            return
        except Exception as e:
            self._status(f"Build failed: {e}")
            return
        self.result = (v, f, rep, pack)
        wt = "WATERTIGHT ✓" if rep["watertight"] else "NOT watertight ✗"
        self._status(f"Built: {rep['faces']} tris, {rep['volume_cm3']:.1f} cm³, "
                     f"voxel {vox} mm, clearance {clr} mm — {wt}. "
                     f"Grid {rep['grid']} ({rep['grid_mem_mb']} MB). "
                     f"Inspect sections, then Save STL.")
        self._draw()

    def _on_save(self, _):
        if self.result is None:
            self._status("Build first.")
            return
        out = self.tb_out.text.strip() or "frame_solid.stl"
        v, f, rep, _ = self.result
        save_stl(out, v, f)
        self._status(f"Saved {out}  ({rep['faces']} tris, "
                     f"{'watertight' if rep['watertight'] else 'NOT watertight'})")

    # ================================================== project I/O
    def _on_proj_save(self, _):
        d = {"scan_path": self.scan_path,
             "M": self.M.tolist(), "offset": self.offset.tolist(),
             "sil": {"px": self.tb_px.text, "close": self.tb_close.text,
                     "simplify": self.tb_simp.text},
             "base_width": self.base_width,
             "regions": self.regions, "extras": self.extras,
             "build": {"voxel": self.tb_vox.text,
                       "clearance": self.tb_clr.text,
                       "out": self.tb_out.text}}
        path = self.tb_proj.text.strip() or "project.json"
        with open(path, "w") as fh:
            json.dump(d, fh, indent=2)
        self._status(f"Project saved: {path}")

    def _on_proj_load(self, _):
        path = self.tb_proj.text.strip()
        if not os.path.isfile(path):
            self._status(f"No such project file: {path}")
            return
        with open(path) as fh:
            d = json.load(fh)
        self.tb_px.set_val(d["sil"]["px"])
        self.tb_close.set_val(d["sil"]["close"])
        self.tb_simp.set_val(d["sil"]["simplify"])
        self.tb_vox.set_val(d["build"]["voxel"])
        self.tb_clr.set_val(d["build"]["clearance"])
        self.tb_out.set_val(d["build"].get("out", "frame_solid.stl"))
        self.base_width = d.get("base_width")
        if self.base_width:
            self.tb_base.set_val(str(self.base_width))
        self.regions = d.get("regions", [])
        self.extras = d.get("extras", [])
        sp = d.get("scan_path", "")
        if sp and os.path.isfile(sp):
            self.tb_path.set_val(sp)
            v, f = load_mesh(sp)
            self.scan_path = sp
            self.orig_verts, self.orig_faces = v, f
            self.M = np.asarray(d["M"])
            self.offset = np.asarray(d["offset"])
            self._apply_transform(center=False)
            self.sil = None
            self.wmap = None
            self.result = None
            self._status(f"Project loaded; scan reloaded ({len(v)} verts). "
                         f"Re-run step 2 (Extract) to continue.")
        else:
            self._status("Project loaded (scan file not found — load manually).")
        self._draw()

    # ================================================== drawing
    def _hist2d(self, ax, a, b, title, xlabel, ylabel):
        ax.clear()
        if self.verts is not None:
            h, xe, ye = np.histogram2d(a, b, bins=300)
            ax.imshow((h.T > 0), origin="lower", cmap="gray_r",
                      extent=(xe[0], xe[-1], ye[0], ye[-1]), aspect="equal")
        ax.set_title(title, fontsize=9)
        ax.set_xlabel(xlabel, fontsize=8)
        ax.set_ylabel(ylabel, fontsize=8)
        ax.tick_params(labelsize=7)

    def _draw(self):
        if self.step == 0:
            if self.verts is not None:
                v = self.verts
                self._hist2d(self.ax_side, v[:, 0], v[:, 2],
                             "SIDE view (X-Z) — this becomes the outline",
                             "X (bore) mm", "Z (height) mm")
                self._hist2d(self.ax_front, v[:, 1], v[:, 2],
                             "FRONT view (Y-Z) — should look thin",
                             "Y (across) mm", "Z mm")
                self._hist2d(self.ax_top, v[:, 0], v[:, 1],
                             "TOP view (X-Y) — symmetric about Y=0",
                             "X mm", "Y mm")
            else:
                for ax in self.orient_axes:
                    ax.clear()
            self.fig.canvas.draw_idle()
            return

        ax = self.ax_main
        ax.clear()

        if self.step == 1:      # silhouette
            if self.sil is not None:
                ex = self.sil.world_extent()
                ax.imshow(self.sil.mask, origin="lower", extent=ex,
                          cmap="gray_r", alpha=0.9)
                p = self.sil.polygon
                ax.plot(np.r_[p[:, 0], p[0, 0]], np.r_[p[:, 1], p[0, 1]],
                        "r-", lw=1.5)
                ax.set_title("Outer silhouette (dark) — interior windows filled")
            elif self.verts is not None:
                v = self.verts
                ax.plot(v[::17, 0], v[::17, 2], ",", color="0.6")
                ax.set_title("Press 'Extract silhouette'")
            ax.set_aspect("equal")

        elif self.step == 2:    # regions
            if self.wmap is not None:
                wm, x0, z0, px = self.wmap
                h, w = wm.shape
                im = ax.imshow(wm, origin="lower", cmap="viridis",
                               extent=(x0, x0 + w * px, z0, z0 + h * px))
                ax.set_title(f"Thickness map — base {self.base_width} mm; "
                             "drag rectangles for plateau regions")
            elif self.sil is not None:
                self._compute_wmap()
                self._draw()
                return
            else:
                ax.set_title("Extract a silhouette first (step 2)")
            if self.sil is not None:
                p = self.sil.polygon
                ax.plot(np.r_[p[:, 0], p[0, 0]], np.r_[p[:, 1], p[0, 1]],
                        "w-", lw=1)
            self._draw_regions(ax)
            ax.set_aspect("equal")

        elif self.step == 3:    # extras
            if self.sil is not None:
                ex = self.sil.world_extent()
                ax.imshow(self.sil.mask, origin="lower", extent=ex,
                          cmap="gray_r", alpha=0.35)
            self._draw_regions(ax, faint=True)
            for i, e in enumerate(self.extras):
                if e["kind"] == "box":
                    cx, cz = (e["x0"] + e["x1"]) / 2, (e["z0"] + e["z1"]) / 2
                    hx, hz = abs(e["x1"] - e["x0"]) / 2, abs(e["z1"] - e["z0"]) / 2
                    a = np.deg2rad(e.get("tilt_deg", 0))
                    c, s = np.cos(a), np.sin(a)
                    pts = np.array([[-hx, -hz], [hx, -hz], [hx, hz], [-hx, hz]])
                    rot = pts @ np.array([[c, -s], [s, c]]).T + [cx, cz]
                    ax.add_patch(MplPolygon(rot, closed=True, fill=False,
                                            ec="tab:red", lw=2))
                    ax.text(cx, cz, f"#{i+1} y±{e['ywidth']/2:.0f}",
                            color="tab:red", fontsize=8, ha="center")
                else:
                    ax.add_patch(MplCircle((e["x"], e["z"]), e["dia"] / 2,
                                           fill=False, ec="tab:blue", lw=2))
                    ax.text(e["x"], e["z"] + e["dia"] / 2 + 1,
                            f"#{i+1} ⌀{e['dia']}", color="tab:blue",
                            fontsize=8, ha="center")
            ax.set_title("Extras: drag = box, click = Y-cylinder "
                         "(kind set at right)")
            ax.set_aspect("equal")

        elif self.step == 4:    # build
            if self.result is not None:
                _, _, rep, pack = self.result
                sdf, (xlo, ylo, zlo), vox = pack
                t = self.sl_sec.val   # -1..1
                if self.radio_sec.value_selected.startswith("side"):
                    y = t * (sdf.shape[1] * vox / 2)
                    m, mex = core.sdf_slice_y(pack, y)
                    ax.imshow(m.T, origin="lower",
                              extent=(mex[0], mex[1], mex[2], mex[3]),
                              cmap="copper")
                    ax.set_title(f"Solid section at Y = {y:+.1f} mm "
                                 "(bright = material to subtract)")
                    ax.set_xlabel("X mm"); ax.set_ylabel("Z mm")
                else:
                    x = t * (sdf.shape[0] * vox / 2)
                    m, mex = core.sdf_slice_x(pack, x)
                    ax.imshow(m.T, origin="lower",
                              extent=(mex[0], mex[1], mex[2], mex[3]),
                              cmap="copper")
                    ax.set_title(f"Cross-section at X = {x:+.1f} mm")
                    ax.set_xlabel("Y mm"); ax.set_ylabel("Z mm")
            else:
                ax.set_title("Press BUILD")
            ax.set_aspect("equal")

        self.fig.canvas.draw_idle()

    def _draw_regions(self, ax, faint=False):
        colors = ["tab:orange", "tab:green", "tab:red", "tab:purple",
                  "tab:brown", "tab:pink"]
        alpha = 0.5 if faint else 1.0
        for i, r in enumerate(self.regions):
            c = colors[i % len(colors)]
            x0, x1 = min(r["x0"], r["x1"]), max(r["x0"], r["x1"])
            z0, z1 = min(r["z0"], r["z1"]), max(r["z0"], r["z1"])
            ax.add_patch(plt.Rectangle((x0, z0), x1 - x0, z1 - z0, fill=False,
                                       ec=c, lw=2, alpha=alpha))
            if not faint:
                ax.text(x0 + 1, z0 + 1, f"#{i+1}: {r['width']}mm", color=c,
                        fontsize=9)


def main():
    app = App()
    if len(sys.argv) > 1 and sys.argv[1].endswith(".json"):
        app.tb_proj.set_val(sys.argv[1])
        app._on_proj_load(None)
    plt.show()
    return app


if __name__ == "__main__":
    main()
