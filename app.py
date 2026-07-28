"""frame2solid — GUI for turning a pistol-frame scan into a clean,
watertight 'subtraction solid' for grip transplants.

Run:  python app.py            (optionally: python app.py myproject.json)

Steps (radio buttons, top right):
  1 Load/Orient — load scan, PCA auto-orient, fix with 90-deg rotations/flips.
                  Goal: X = bore axis, Y = across frame (symmetry plane Y=0),
                  Z = vertical. Views show the three projections.
  2 Silhouette  — extract the side-view OUTER outline. Windows and holes are
                  filled automatically. Export DXF for CAD tracing if wanted.
  3 Tiers       — two views, one per side of the frame (the left one is
                  mirrored: it is the frame seen from its left). Each shows
                  only its own side's tiers. "Pick base" + a click on the
                  THINNEST part sets the base tier (the whole silhouette).
                  Every further click adds a tier to whichever view you
                  clicked: its outline is everything standing proud of the
                  tier below it (anywhere on the frame), extruded out to the
                  clicked thickness. 3-5 tiers per side is typical. Tiers are
                  listed at the right: click a row (or right-click a view) to
                  select one; edit its height / noise margin / growth in the
                  boxes and press APPLY (or Enter) to commit.
  4 Extras      — add clearance solids: drag = tilted box (mag path, levers),
                  click = Y-cylinder (grip screws, pins). Dims via text boxes
                  (they edit the LAST extra); y-mid offsets it off the
                  centreline for one-sided reliefs.
  5 Build       — two ways out of the same tier model. "Export CAD (DXF)"
                  writes the sketches and their extrusion depths for
                  SolveSpace/OpenSCAD (no build needed — this is the lossless
                  one). "BUILD solid" is the voxel SDF preview and printable
                  watertight STL; inspect cross-sections with the slider.
"""
from __future__ import annotations

import contextlib
import json
import os
import sys

import numpy as np
import matplotlib
import matplotlib.pyplot as plt
from matplotlib.path import Path as MplPath
from matplotlib.widgets import (Button, TextBox, RadioButtons,
                                RectangleSelector, Slider)
from matplotlib.patches import Polygon as MplPolygon, Circle as MplCircle

import core
from meshio_lite import load_mesh, save_stl

STEPS = ["1 Load/Orient", "2 Silhouette", "3 Tiers", "4 Extras", "5 Build"]
SIDE_KEYS = ["left", "right"]
SIDE_TAG = {"left": "L ", "right": " R"}


class ListBox:
    """Minimal clickable list for matplotlib (no tkinter, testable headless).

    Rows are drawn top-down in their own axes; a click selects a row and
    calls on_select(index). Hidden/disabled like the other step widgets via
    .ax and set_active().
    """

    ROWS = 11

    def __init__(self, fig, rect, on_select, title=""):
        self.ax = fig.add_axes(rect)
        self.ax.set_navigate(False)
        self.ax.set_xticks([])
        self.ax.set_yticks([])
        for sp in self.ax.spines.values():
            sp.set_color("0.75")
        self.on_select = on_select
        self.title = title
        self.items = []
        self.sel = None
        self.top = 0
        self.active = True
        self._rh = 1.0 / (self.ROWS + 1)
        fig.canvas.mpl_connect("button_press_event", self._on_click)
        fig.canvas.mpl_connect("scroll_event", self._on_scroll)
        self.draw()

    # matplotlib-widget-compatible enable/disable
    def set_active(self, flag):
        self.active = bool(flag)

    def set_items(self, items, sel=None):
        self.items = list(items)
        self.sel = sel
        if sel is not None:
            if sel < self.top:
                self.top = sel
            elif sel >= self.top + self.ROWS:
                self.top = sel - self.ROWS + 1
        self.top = max(0, min(self.top, max(0, len(self.items) - self.ROWS)))
        self.draw()

    def draw(self):
        ax = self.ax
        ax.clear()
        ax.set_xticks([]); ax.set_yticks([])
        ax.set_xlim(0, 1); ax.set_ylim(0, 1)
        rh = self._rh
        ax.text(0.015, 1 - rh * 0.5, self.title, fontsize=8, weight="bold",
                va="center", color="0.25")
        if not self.items:
            ax.text(0.015, 1 - rh * 1.6, "(none yet)", fontsize=8,
                    color="0.55", va="center", family="monospace")
        for i, txt in enumerate(self.items[self.top:self.top + self.ROWS]):
            idx = self.top + i
            y = 1 - rh * (i + 1.5)
            if idx == self.sel:
                ax.add_patch(plt.Rectangle((0, y - rh * 0.5), 1, rh,
                                           color="#ffe08a", zorder=0))
            ax.text(0.015, y, txt, fontsize=8, family="monospace",
                    va="center", zorder=1)
        n_hidden = len(self.items) - self.top - self.ROWS
        if n_hidden > 0:
            ax.text(0.985, rh * 0.5, f"+{n_hidden} more (scroll)", fontsize=7,
                    color="0.55", va="center", ha="right")

    def _row_at(self, event):
        if not self.active or not self.ax.get_visible():
            return None
        if event.inaxes is not self.ax or event.ydata is None:
            return None
        row = int((1.0 - event.ydata) / self._rh) - 1
        if row < 0:
            return None
        idx = self.top + row
        return idx if 0 <= idx < len(self.items) else None

    def _on_click(self, event):
        idx = self._row_at(event)
        if idx is not None:
            self.on_select(idx)

    def _on_scroll(self, event):
        if not self.active or not self.ax.get_visible():
            return
        if event.inaxes is not self.ax:
            return
        self.top = max(0, min(self.top - int(event.step),
                              max(0, len(self.items) - self.ROWS)))
        self.draw()
        self.ax.figure.canvas.draw_idle()


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
        self.maps = None              # core.ThicknessMaps (two-sided)
        self.base_thickness = None
        self.base_y0 = 0.0            # mid-plane of the base slab
        self.regions = []
        self.sel = None               # index of the selected region
        self.extras = []
        self.result = None            # (verts, faces, report, sdf_pack)
        self.step = 0
        self.extra_kind = "box (drag)"
        self._radio_guard = False
        self._sync = False            # suppress widget callbacks while syncing
        self._pick_base_armed = False

        # ---------- axes
        self.ax_main = self.fig.add_axes([0.05, 0.16, 0.58, 0.76])
        self.ax_side = self.fig.add_axes([0.05, 0.55, 0.27, 0.37])
        self.ax_front = self.fig.add_axes([0.36, 0.55, 0.27, 0.37])
        self.ax_top = self.fig.add_axes([0.05, 0.10, 0.27, 0.37])
        self.orient_axes = [self.ax_side, self.ax_front, self.ax_top]
        # step 3 shows the two sides as separate views. The left one is
        # mirrored in X, so it is what you see standing on the frame's left —
        # not the right-hand view with the far side showing through.
        self.ax_tier_L = self.fig.add_axes([0.045, 0.16, 0.29, 0.76])
        self.ax_tier_R = self.fig.add_axes([0.345, 0.16, 0.29, 0.76])
        self.tier_axes = {"left": self.ax_tier_L, "right": self.ax_tier_R}

        self.txt_status = self.fig.text(0.05, 0.035, "Load a scan to begin.",
                                        fontsize=10, family="monospace",
                                        va="bottom", wrap=True)
        self.txt_busy = self.fig.text(
            0.635, 0.955, "", fontsize=10, family="monospace", weight="bold",
            ha="right", va="center", color="#7a3e00", visible=False,
            bbox=dict(fc="#ffd97a", ec="#c99a2e", boxstyle="round,pad=0.35"))

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
        r = slot(split=(0.715, 0.055))
        self.tb_base = TextBox(self.fig.add_axes(r), "base T ", initial="")
        self.tb_y0 = TextBox(self.fig.add_axes([0.805, r[1], 0.045, r[3]]),
                             "y0 ", initial="0")
        self.bt_pickbase = Button(self.fig.add_axes([0.865, r[1], 0.115, r[3]]),
                                  "Pick base")
        self.bt_pickbase.on_clicked(self._on_pick_base)
        self.w3 += [self.tb_base, self.tb_y0, self.bt_pickbase]

        r = slot(split=(0.715, 0.055))
        self.tb_add = TextBox(self.fig.add_axes(r), "+mm ", initial="")
        self.tb_over = TextBox(self.fig.add_axes([0.825, r[1], 0.045, r[3]]),
                               "over ", initial="0.3")
        self.tb_grow = TextBox(self.fig.add_axes([0.925, r[1], 0.055, r[3]]),
                               "grow ", initial="0")
        # deliberately NO on_submit: matplotlib fires that when a box merely
        # loses focus, so tabbing between boxes would kick off a re-cut each
        # time. Edits land on Apply (button or Enter) and nowhere else.
        self.w3 += [self.tb_add, self.tb_over, self.tb_grow]

        r = slot(h=0.075)
        self.bt_apply = Button(self.fig.add_axes([0.67, r[1], 0.145, 0.075]),
                               "APPLY\n(or press Enter)")
        self.bt_apply.on_clicked(self._apply_edits)
        self.w3.append(self.bt_apply)
        self.bt_prev = Button(self.fig.add_axes([0.825, r[1] + 0.041,
                                                 0.07, 0.033]), "< prev")
        self.bt_next = Button(self.fig.add_axes([0.905, r[1] + 0.041,
                                                 0.07, 0.033]), "next >")
        self.bt_rdel = Button(self.fig.add_axes([0.825, r[1] + 0.003,
                                                 0.07, 0.033]), "Delete")
        self.bt_rclr = Button(self.fig.add_axes([0.905, r[1] + 0.003,
                                                 0.07, 0.033]), "Clear")
        self.bt_prev.on_clicked(lambda e: self._step_sel(-1))
        self.bt_next.on_clicked(lambda e: self._step_sel(+1))
        self.bt_rdel.on_clicked(self._on_region_del)
        self.bt_rclr.on_clicked(self._on_region_clr)
        self.w3 += [self.bt_prev, self.bt_next, self.bt_rdel, self.bt_rclr]

        r = slot(split=(0.735, 0.05))
        self.tb_mpx = TextBox(self.fig.add_axes(r), "map px ", initial="0.5")
        self.bt_remeas = Button(self.fig.add_axes([0.81, r[1], 0.17, r[3]]),
                                "Re-measure scan")
        self.bt_remeas.on_clicked(self._on_remeasure)
        self.w3 += [self.tb_mpx, self.bt_remeas]

        self.lst = ListBox(self.fig, [0.67, 0.14, 0.31, 0.315], self._select,
                           "thickness tiers  (base is the thinnest part)")
        self.w3.append(self.lst)

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
        self.tb_yc = TextBox(self.fig.add_axes([0.845, r[1], 0.055, r[3]]),
                             "y-mid ", initial="0")
        self.tb_yc.on_submit(self._on_extra_edit)
        self.w4 += [self.tb_ylen, self.tb_yc]
        r = slot(split=(0.67, 0.145))
        self.bt_edel = Button(self.fig.add_axes(r), "Delete last extra")
        self.bt_edel.on_clicked(self._on_extra_del)
        self.bt_eclr = Button(self.fig.add_axes([0.835, r[1], 0.145, r[3]]),
                              "Clear extras")
        self.bt_eclr.on_clicked(self._on_extra_clr)
        self.w4 += [self.bt_edel, self.bt_eclr]

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
        r = slot(split=(0.67, 0.145))
        self.bt_save = Button(self.fig.add_axes(r), "Save STL")
        self.bt_save.on_clicked(self._on_save)
        self.bt_cad = Button(self.fig.add_axes([0.835, r[1], 0.145, r[3]]),
                             "Export CAD (DXF)")
        self.bt_cad.on_clicked(self._on_export_cad)
        self.w5 += [self.bt_save, self.bt_cad]

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
        self.fig.canvas.mpl_connect("key_press_event", self._on_key)

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
        show_tiers = (i == 2)
        for ax in self.orient_axes:
            ax.set_visible(show_orient)
        for ax in self.tier_axes.values():
            ax.set_visible(show_tiers)
        self.ax_main.set_visible(not show_orient and not show_tiers)
        self.rsel.set_active(i == 3)     # drag-a-box is step 4 only now
        self._pick_base_armed = False
        if show_tiers:
            if self.sil is not None and self.maps is None:
                with self._busy("measuring scan…"):
                    self._compute_maps()
            self._refresh_list()
        self._draw()

    def _on_step(self, label):
        if self._radio_guard:
            return
        self._set_step(STEPS.index(label))

    def _status(self, msg):
        import textwrap
        self.txt_status.set_text("\n".join(textwrap.wrap(msg, 92)[:3]))
        self.fig.canvas.draw_idle()

    @contextlib.contextmanager
    def _busy(self, msg="applying…"):
        """Badge the figure while a slow update runs.

        The view underneath is the stale one until the work finishes — that
        is the point: it says "this is old, the new one is coming".
        """
        self.txt_busy.set_text(f" {msg} ")
        self.txt_busy.set_visible(True)
        try:
            self.fig.canvas.draw()
            self.fig.canvas.flush_events()
            yield
        finally:
            self.txt_busy.set_visible(False)

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
        self._invalidate()
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
        self.maps = None
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
        self.maps = None
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

    def _compute_maps(self):
        if self.sil is None or self.verts is None:
            return
        try:
            mpx = float(self.tb_mpx.text)
        except ValueError:
            mpx = 0.5
        self.maps = core.measure_maps(self.verts, self.orig_faces, self.sil,
                                      px=mpx)
        if self.base_thickness is None:
            t, y0 = core.base_from_maps(self.maps)
            self.base_thickness = round(t, 2) if t else 20.0
            self.base_y0 = round(y0, 2)
        self._sync_boxes()

    # ================================================== step 3: regions
    def _selected(self):
        if self.sel is None or not 0 <= self.sel < len(self.regions):
            return None
        return self.regions[self.sel]

    def _region_label(self, i):
        r = self.regions[i]
        if core.region_kind(r) == "rect":
            return f"{i+1:2d} LR  {r['width']:6.2f}mm total (rectangle)"
        area = r.get("area_mm2")
        area = f"{area:6.0f}mm2" if area else "  EMPTY "
        fo = ("   ?" if self.base_thickness is None else
              f"{core.tier_offset(r, self.base_thickness, self.base_y0):5.2f}")
        return (f"{i+1:2d} {SIDE_TAG[r['side']]} +{r['add_mm']:5.2f} "
                f"face{fo} over{r['over_mm']:g} grow{r['grow_mm']:g}{area}")

    def _refresh_list(self):
        self.lst.set_items([self._region_label(i)
                            for i in range(len(self.regions))], self.sel)

    def _index_of(self, obj):
        for i, r in enumerate(self.regions):
            if r is obj:
                return i
        return None

    def _sort_regions(self):
        """Keep tiers in stacking order (side, then height) — thresholds
        derive from the ordering, so it must not depend on click order."""
        keep = self._selected()
        self.regions = core.sort_tiers(self.regions)
        self.sel = self._index_of(keep) if keep is not None else None

    def _resegment(self):
        if self.maps is None or self.base_thickness is None:
            return
        core.segment_all(self.maps, self.regions, self.base_thickness,
                         self.base_y0)

    def _coverage_note(self):
        """One-line 'how much is still under-thick' summary."""
        if self.maps is None or self.base_thickness is None:
            return ""
        cov = core.coverage_report(self.maps, self.regions,
                                   self.base_thickness, self.base_y0)
        bits = []
        for side in ("left", "right"):
            c = cov[side]
            if c["short_mm"] > c["tol_mm"] and c["area_mm2"] >= 1.0:
                bits.append(f"{side[0].upper()} {c['short_mm']:.2f}mm over "
                            f"{c['area_mm2']:.0f}mm² (worst at X={c['at'][0]:g},"
                            f" Z={c['at'][1]:g})")
        return ("  Still under-thick: " + "; ".join(bits) if bits
                else "  Tiers now cover the whole scan.")

    def _sync_boxes(self):
        """Push state into the text boxes (they have no submit callbacks)."""
        self._sync = True
        try:
            if self.base_thickness is not None:
                self.tb_base.set_val(f"{self.base_thickness:g}")
            self.tb_y0.set_val(f"{self.base_y0:g}")
            r = self._selected()
            if r is None:
                self.tb_add.set_val("")
            elif core.region_kind(r) == "rect":
                self.tb_add.set_val(f"{r['width']:g}")
            else:
                self.tb_add.set_val(f"{r['add_mm']:g}")
                self.tb_over.set_val(f"{r['over_mm']:g}")
                self.tb_grow.set_val(f"{r['grow_mm']:g}")
        finally:
            self._sync = False

    def _select(self, i):
        if i is not None and not 0 <= i < len(self.regions):
            i = None
        self.sel = i
        self._sync_boxes()
        self._refresh_list()
        self._draw()

    def _step_sel(self, d):
        if not self.regions:
            return
        i = 0 if self.sel is None else (self.sel + d) % len(self.regions)
        self._select(i)

    def _region_at(self, x, z, side=None):
        """Index of the tallest tier on `side` containing world point (x, z)."""
        for i in range(len(self.regions) - 1, -1, -1):
            r = self.regions[i]
            if side is not None and r.get("side", side) != side:
                continue
            if core.region_kind(r) == "rect":
                if (min(r["x0"], r["x1"]) <= x <= max(r["x0"], r["x1"])
                        and min(r["z0"], r["z1"]) <= z <= max(r["z0"], r["z1"])):
                    return i
            elif any(MplPath(np.asarray(p)).contains_point((x, z))
                     for p in r.get("polys", [])):
                return i
        return None

    def _on_pick_base(self, _):
        if self.step != 2:
            return
        if self.maps is None:
            self._status("Extract a silhouette first (step 2).")
            return
        self._pick_base_armed = True
        self._status("Click the THINNEST part of the frame — its measured "
                     "thickness becomes the base tier, and every other tier "
                     "stacks outward from it.")

    def pick_base_at(self, x, z):
        self._pick_base_armed = False
        s = core.sample_maps(self.maps, x, z)
        if s is None:
            self._status("No scan data there — click inside the frame.")
            return
        self.base_thickness = round(s["total"], 2)
        self.base_y0 = round((s["hR"] - s["hL"]) / 2, 2)
        self.result = None
        with self._busy("setting base…"):
            self._resegment()           # every tier measures from the base
            self._sync_boxes()
            self._refresh_list()
        self._status(f"Base tier {self.base_thickness} mm, mid-plane y0 "
                     f"{self.base_y0:+g} mm (faces {-s['hL']:+.2f} / "
                     f"{s['hR']:+.2f}) from X={x:.1f}, Z={z:.1f}. Now click "
                     f"each thicker feature to add a tier.{self._coverage_note()}")
        self._draw()

    def add_region(self, x, z, side="right"):
        """Add a tier on `side`, picked at world XZ."""
        if self.maps is None:
            self._status("Extract a silhouette first (step 2).")
            return
        if self.base_thickness is None:
            self._status("Set the base thickness first.")
            return
        try:
            over = float(self.tb_over.text)
            grow = float(self.tb_grow.text)
        except ValueError:
            over, grow = 0.3, 0.0
        with self._busy("adding tier…"):
            t = core.make_tier(self.maps, x, z, side, self.base_thickness,
                               self.base_y0, over_mm=over, grow_mm=grow,
                               tiers=self.regions)
            if t is not None:
                self.regions.append(t)
                self._sort_regions()
                self.sel = self._index_of(t)
                self._resegment()       # a new tier re-cuts its neighbours
                self.result = None
                self._sync_boxes()
                self._refresh_list()
                note = self._coverage_note()
        if t is None:
            self._status("No scan data there — click inside the frame.")
            return
        m = t["measured"]
        self._status(f"{side} tier +{t['add_mm']:.2f} mm, {t['area_mm2']:.0f} "
                     f"mm² — scan faces here {-m['hL']:+.2f} / {m['hR']:+.2f}, "
                     f"total {m['total']:.2f} mm. Everything proud of the tier "
                     f"below is roped in.{note}")
        self._draw()

    def _apply_edits(self, _=None):
        """Commit the text boxes — the Apply button, or Enter in any of them.

        Nothing in step 3 re-cuts on focus change, so you can tab around the
        boxes freely and pay for the update exactly once, when you say so.
        """
        if self.step != 2:
            return
        changes = []
        try:
            base_t = float(self.tb_base.text)
            base_y0 = float(self.tb_y0.text)
        except ValueError:
            self._status("Base thickness / y0 must be numbers.")
            return
        if (base_t, base_y0) != (self.base_thickness, self.base_y0):
            self.base_thickness, self.base_y0 = base_t, base_y0
            changes.append(f"base {base_t:g} mm @ y0 {base_y0:+g}")

        r = self._selected()
        if r is not None and self.tb_add.text.strip():
            try:
                val = float(self.tb_add.text)
            except ValueError:
                self._status("Tier thickness must be a number.")
                return
            if core.region_kind(r) == "rect":
                if val != r["width"]:
                    r["width"] = val
                    changes.append(f"rectangle {val:g} mm")
            else:
                try:
                    over = float(self.tb_over.text)
                    grow = float(self.tb_grow.text)
                except ValueError:
                    self._status("over / grow must be numbers.")
                    return
                if (val, over, grow) != (r["add_mm"], r["over_mm"],
                                         r["grow_mm"]):
                    r["add_mm"], r["over_mm"], r["grow_mm"] = val, over, grow
                    changes.append(f"tier {r['side']} +{val:g} mm "
                                   f"(over {over:g}, grow {grow:g})")
        if not changes:
            self._status("Nothing to apply — the boxes match the model.")
            return
        with self._busy():
            self._sort_regions()
            self._resegment()
            self.result = None
            self._sync_boxes()
            self._refresh_list()
            note = self._coverage_note()
        self._status("Applied: " + "; ".join(changes) + "." + note)
        self._draw()

    def _on_key(self, event):
        """Enter applies, wherever the keyboard focus happens to be."""
        if event.key in ("enter", "return") and self.step == 2:
            self._apply_edits()

    def _on_remeasure(self, _):
        if self.sil is None:
            self._status("Extract a silhouette first (step 2).")
            return
        with self._busy("re-measuring scan…"):
            self._compute_maps()
            self._resegment()
            self.result = None
            self._sync_boxes()
            self._refresh_list()
            note = self._coverage_note()
        self._status(f"Thickness maps re-measured at {self.maps.px} mm/px; "
                     f"{len(self.regions)} tier(s) re-cut.{note}")
        self._draw()

    def _on_region_del(self, _):
        if self.step != 2 or not self.regions:
            return
        i = self.sel if self.sel is not None else len(self.regions) - 1
        with self._busy("deleting tier…"):
            if 0 <= i < len(self.regions):
                self.regions.pop(i)
            self.sel = min(i, len(self.regions) - 1) if self.regions else None
            self.result = None
            self._resegment()    # the tier above inherits a lower threshold
        self._select(self.sel)

    def _on_region_clr(self, _):
        if self.step != 2:
            return
        self.regions = []
        self.sel = None
        self.result = None
        self._select(None)

    # ================================================== step 4: extras
    def _on_extra_kind(self, label):
        self.extra_kind = label

    def _extra_yc(self):
        try:
            return float(self.tb_yc.text)
        except ValueError:
            return 0.0

    def add_extra_box(self, x0, z0, x1, z1):
        try:
            yw = float(self.tb_yw.text)
            tilt = float(self.tb_tilt.text)
        except ValueError:
            yw, tilt = 20.0, 0.0
        yc = self._extra_yc()
        self.extras.append({"kind": "box", "x0": round(x0, 2),
                            "z0": round(z0, 2), "x1": round(x1, 2),
                            "z1": round(z1, 2), "ywidth": yw,
                            "tilt_deg": tilt, "yc": yc})
        self.result = None
        self._status(f"Extra {len(self.extras)}: box, y {yc:+g} ± {yw / 2:g}, "
                     f"tilt {tilt}°. Edit via text boxes (applies to last).")
        self._draw()

    def add_extra_cyl(self, x, z):
        try:
            dia = float(self.tb_dia.text)
            ylen = float(self.tb_ylen.text)
        except ValueError:
            dia, ylen = 5.0, 60.0
        yc = self._extra_yc()
        self.extras.append({"kind": "cyl_y", "x": round(x, 2),
                            "z": round(z, 2), "dia": dia, "ylen": ylen,
                            "yc": yc})
        self.result = None
        self._status(f"Extra {len(self.extras)}: Y-cylinder ⌀{dia} × {ylen} "
                     f"centred at y {yc:+g}.")
        self._draw()

    def _on_extra_edit(self, _text):
        if not self.extras:
            return
        e = self.extras[-1]
        try:
            e["yc"] = float(self.tb_yc.text)
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
        if self.step == 3 and self.extra_kind.startswith("box"):
            self.add_extra_box(x0, z0, x1, z1)

    def _toolbar_idle(self):
        tb = self.fig.canvas.toolbar
        return tb is None or getattr(tb, "mode", "") == ""

    def _on_click(self, event):
        if not self._toolbar_idle():
            return
        if event.xdata is None or event.ydata is None:
            return
        x, z = event.xdata, event.ydata
        if self.step == 2:                       # tiers: one view per side
            side = next((s for s, ax in self.tier_axes.items()
                         if event.inaxes is ax), None)
            if side is None:
                return
            if event.button == 3:
                self._select(self._region_at(x, z, side))
            elif event.button == 1:
                if self._pick_base_armed:
                    self.pick_base_at(x, z)
                else:
                    self.add_region(x, z, side)
        elif (self.step == 3 and event.inaxes is self.ax_main
                and event.button == 1 and self.extra_kind.startswith("cyl")):
            self.add_extra_cyl(x, z)

    # ================================================== step 5: build
    def _on_build(self, _):
        if self.sil is None:
            self._status("Need a silhouette first (step 2).")
            return
        if self.base_thickness is None:
            self._status("Set a base thickness first (step 3).")
            return
        try:
            vox = float(self.tb_vox.text)
            clr = float(self.tb_clr.text)
        except ValueError:
            self._status("Bad voxel/clearance value.")
            return
        self._status("Building… (large grids can take a minute)")
        try:
            with self._busy("building solid…"):
                v, f, rep, pack = core.build_solid(
                    self.sil, self.base_thickness, self.regions, self.extras,
                    clearance=clr, voxel=vox, return_sdf=True,
                    base_y0=self.base_y0)
        except MemoryError:
            self._status("Out of memory — increase voxel size.")
            return
        except Exception as e:
            self._status(f"Build failed: {e}")
            return
        self.result = (v, f, rep, pack)
        wt = "WATERTIGHT ✓" if rep["watertight"] else "NOT watertight ✗"
        skipped = (f" {rep['regions_skipped']} region(s) had no footprint and "
                   f"were skipped." if rep.get("regions_skipped") else "")
        self._status(f"Built: {rep['faces']} tris, {rep['volume_cm3']:.1f} cm³, "
                     f"voxel {vox} mm, clearance {clr} mm — {wt}. "
                     f"Grid {rep['grid']} ({rep['grid_mem_mb']} MB).{skipped} "
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

    def _on_export_cad(self, _):
        """Export the tier model as sketches — no voxel build needed."""
        if self.sil is None:
            self._status("Need a silhouette first (step 2).")
            return
        if self.base_thickness is None:
            self._status("Set the base thickness first (step 3).")
            return
        try:
            clr = float(self.tb_clr.text)
        except ValueError:
            clr = 0.0
        folder = os.path.splitext(self.tb_out.text.strip()
                                  or "frame_solid.stl")[0] + "_cad"
        try:
            with self._busy("exporting sketches…"):
                model = core.sketch_model(self.sil, self.base_thickness,
                                          self.regions, self.extras,
                                          base_y0=self.base_y0, clearance=clr)
                files = core.export_cad(folder, model, meta={
                    "scan": self.scan_path, "clearance": clr,
                    "base_y0": self.base_y0})
        except Exception as e:
            self._status(f"CAD export failed: {e}")
            return
        self._status(f"Exported {len(model)} sketches to {folder}\\ "
                     f"({len(files)} files): one DXF each plus "
                     f"all_sketches.dxf, build.txt (extrusion depths) and "
                     f"assembly.scad. Clearance {clr:g} mm is baked in — "
                     f"set it to 0 for nominal outlines.")

    # ================================================== project I/O
    def _on_proj_save(self, _):
        d = {"scan_path": self.scan_path,
             "M": self.M.tolist(), "offset": self.offset.tolist(),
             "sil": {"px": self.tb_px.text, "close": self.tb_close.text,
                     "simplify": self.tb_simp.text},
             "base_thickness": self.base_thickness,
             "base_y0": self.base_y0,
             "map_px": self.tb_mpx.text,
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
        self.tb_mpx.set_val(str(d.get("map_px", "0.5")))
        # base_width is the pre-two-sided key name
        self.base_thickness = d.get("base_thickness", d.get("base_width"))
        self.base_y0 = float(d.get("base_y0", 0.0))
        self.regions = d.get("regions", [])
        self.extras = d.get("extras", [])
        self.sel = len(self.regions) - 1 if self.regions else None
        self._sync_boxes()
        self._refresh_list()
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
            self.maps = None
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

        if self.step == 2:      # tiers — one independent view per side
            self._draw_tiers()
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
                    yc = e.get("yc", 0.0)
                    ax.text(cx, cz, f"#{i+1} y{yc:+g}±{e['ywidth']/2:.0f}",
                            color="tab:red", fontsize=8, ha="center")
                else:
                    ax.add_patch(MplCircle((e["x"], e["z"]), e["dia"] / 2,
                                           fill=False, ec="tab:blue", lw=2))
                    ax.text(e["x"], e["z"] + e["dia"] / 2 + 1,
                            f"#{i+1} ⌀{e['dia']}", color="tab:blue",
                            fontsize=8, ha="center")
            ax.set_title("Extras: drag = box, click = Y-cylinder "
                         "(kind set at right; y-mid offsets it off centre)")
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

    def _draw_tiers(self):
        """Step 3: the two sides as separate views, each with its own tiers."""
        if self.maps is None and self.sil is not None:
            with self._busy("measuring scan…"):
                self._compute_maps()
        for side, ax in self.tier_axes.items():
            ax.clear()
            ax.tick_params(labelsize=7)
            if self.maps is None:
                ax.set_title("Extract a silhouette first (step 2)", fontsize=10)
                continue
            m = self.maps.face_map(side)
            finite = m[np.isfinite(m)]
            vmin, vmax = (np.percentile(finite, [2, 98]) if finite.size
                          else (0.0, 1.0))
            if vmax - vmin < 1e-6:
                vmax = vmin + 1.0
            ax.imshow(m, origin="lower", cmap="viridis", vmin=vmin, vmax=vmax,
                      extent=self.maps.world_extent())
            p = self.sil.polygon
            ax.plot(np.r_[p[:, 0], p[0, 0]], np.r_[p[:, 1], p[0, 1]],
                    "w-", lw=1)
            self._draw_regions(ax, side=side)

            n = sum(1 for t in self.regions if t.get("side") == side)
            if self.base_thickness is None:
                base = "base not set"
            else:
                base = (f"base face {core.base_offset(side, self.base_thickness, self.base_y0):.2f}"
                        f" mm")
            hint = ("CLICK THE THINNEST PART TO SET THE BASE"
                    if self._pick_base_armed else
                    "click = add a tier here · right-click = select")
            ax.set_title(f"{side.upper()} side ({'-' if side == 'left' else '+'}Y)"
                         f"  ·  {n} tier(s)  ·  {base}\n{hint}", fontsize=9)
            ax.set_aspect("equal")
            if side == "left":
                # mirrored: this is the view standing on the frame's left
                ax.invert_xaxis()

    def _draw_regions(self, ax, side=None, faint=False):
        colors = ["tab:orange", "tab:red", "tab:cyan", "tab:purple",
                  "tab:brown", "tab:pink", "yellow", "lime"]
        alpha = 0.5 if faint else 1.0
        shown = 0
        for i, r in enumerate(self.regions):
            if side is not None and r.get("side", side) != side:
                continue
            c = colors[i % len(colors)]
            sel = (not faint) and i == self.sel
            lw = 3.0 if sel else 1.8
            if core.region_kind(r) == "rect":
                x0, x1 = min(r["x0"], r["x1"]), max(r["x0"], r["x1"])
                z0, z1 = min(r["z0"], r["z1"]), max(r["z0"], r["z1"])
                ax.add_patch(plt.Rectangle((x0, z0), x1 - x0, z1 - z0,
                                           fill=False, ec=c, lw=lw,
                                           alpha=alpha))
                lx, lz = x0 + 1, z0 + 1
                label = f"#{i+1}: {r['width']}mm"
            else:
                polys = r.get("polys") or []
                if not polys:
                    continue
                for p in polys:
                    p = np.asarray(p, float)
                    if sel:
                        ax.add_patch(MplPolygon(p, closed=True, fc=c,
                                                ec="none", alpha=0.25))
                    ax.add_patch(MplPolygon(p, closed=True, fill=False, ec=c,
                                            lw=lw, alpha=alpha))
                ax.plot([r["x"]], [r["z"]], "+", color=c, ms=9, mew=2,
                        alpha=alpha)
                # label at the pick point, stepped so tiers picked in the
                # same place don't write on top of each other
                lx, lz = r["x"] + 1.5, r["z"] + 1.5 + 3.5 * shown
                label = (f"#{i+1} {SIDE_TAG[r['side']].strip()} "
                         f"+{r['add_mm']:g}mm")
            shown += 1
            if not faint:
                ax.text(lx, lz, label, color=c, fontsize=8,
                        weight="bold" if sel else "normal")


def main():
    app = App()
    if len(sys.argv) > 1 and sys.argv[1].endswith(".json"):
        app.tb_proj.set_val(sys.argv[1])
        app._on_proj_load(None)
    plt.show()
    return app


if __name__ == "__main__":
    main()
