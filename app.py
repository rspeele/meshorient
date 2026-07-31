"""frame2solid — GUI for turning a pistol-frame scan into the CAD sketches
of a 'subtraction solid' for grip transplants.

Run:  python app.py            (optionally: python app.py myproject.json)

Steps (radio buttons, top right):
  1 Load/Orient — load scan, PCA auto-orient, fix with 90-deg rotations/flips.
                  Goal: X = bore axis, Y = across frame (mid-plane near Y=0),
                  Z = vertical. Views show the three projections.
  2 Level       — square the scan up. PCA leaves a fraction of a degree of
                  tilt, which is enough to make one end of a flat side read
                  thicker than the other. Click 3+ points on a face that
                  really is flat (right side of the frame) and LEVEL rotates
                  the scan so that face is perpendicular to Y. The oriented
                  scan can be written back out as an STL here — a rigid
                  transform, nothing resampled — which is worth having for
                  any other work you do on the scan.
  3 Silhouette  — extract the side-view OUTER outline. Windows and holes are
                  filled automatically. Export DXF for CAD tracing if wanted.
  4 Tiers       — two views, one per side of the frame (the left one is
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
  5 Export      — write the model out: one DXF per sketch plus the
                  extrusion table and an OpenSCAD assembly. The view shows
                  every outline that will be written and its Y range.
                  Downstream, OpenSCAD makes the STL (or the finished grip)
                  and SolveSpace/VCarve take the DXFs directly.
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
from matplotlib.widgets import Button, TextBox, RadioButtons
from matplotlib.patches import Polygon as MplPolygon, Circle as MplCircle

import core
from meshio_lite import load_mesh, save_stl

def _blit_widget(w):
    """Repaint just this widget's own axes, not the whole figure."""
    canvas = w.ax.get_figure(root=True).canvas
    if getattr(canvas, "supports_blit", False):
        try:
            w.ax.draw_artist(w.ax)
            canvas.blit(w.ax.bbox)
            return
        except Exception:
            pass                       # backend can't blit — fall back
    canvas.draw_idle()


def _repaint_widget_only(method):
    """Run a widget method with canvas.draw() redirected to a blit.

    Reuses matplotlib's own logic rather than reimplementing it; only the
    repaint at the end is swapped. Full draws still happen if the figure has
    never been rendered (nothing to blit onto).
    """
    def wrapper(self, *args, **kwargs):
        fig = self.ax.get_figure(root=True)
        canvas = fig.canvas
        real_draw = canvas.draw

        def cheap_draw(*a, **k):
            if fig._get_renderer() is None:
                return real_draw(*a, **k)
            _blit_widget(self)

        canvas.draw = cheap_draw
        try:
            return method(self, *args, **kwargs)
        finally:
            canvas.draw = real_draw
    wrapper._f2s_patched = True
    return wrapper


def _patch_matplotlib_textbox():
    """Work around matplotlib TextBox problems (checked against 3.11.0).

    1. `_resize` is connected to 'resize_event' but decorated with the
       mouse-event reparenting wrapper, which reads event.inaxes. A
       ResizeEvent has none, so every window resize raised AttributeError
       once per text box — fifteen tracebacks per drag here.

    2. TextBox repaints by calling canvas.draw(), a full figure render. This
       figure takes ~200 ms to render, and matplotlib calls stop_typing() on
       every box that was NOT the one clicked, so a single click into a text
       box cost 7 full renders — over a second before the cursor appeared,
       and another render per keystroke. Button already blits its hover
       repaint (`useblit`); TextBox simply never got the same treatment. So:
         - stop_typing: return early when there is nothing to stop, which is
           the case for every box except the one you were typing in;
         - _motion (hover tint) and _rendercursor (click, and every
           keystroke): repaint that widget's axes only.

    Each patch is skipped if it has already been applied or if upstream has
    changed the code out from under it.
    """
    tb = getattr(TextBox, "_resize", None)
    inner = getattr(tb, "__wrapped__", None)
    if inner is not None:
        TextBox._resize = inner

    if not getattr(TextBox.stop_typing, "_f2s_patched", False):
        _stop_typing = TextBox.stop_typing

        def stop_typing(self):
            # not typing and no cursor showing: the original would change
            # nothing and then repaint the whole figure anyway
            if not self.capturekeystrokes and not self.cursor.get_visible():
                return
            _stop_typing(self)

        stop_typing._f2s_patched = True
        TextBox.stop_typing = _repaint_widget_only(stop_typing)

    for name in ("_motion", "_rendercursor"):
        method = getattr(TextBox, name, None)
        if method is not None and not getattr(method, "_f2s_patched", False):
            setattr(TextBox, name, _repaint_widget_only(method))


_patch_matplotlib_textbox()

STEPS = ["1 Load/Orient", "2 Level", "3 Silhouette", "4 Tiers", "5 Export"]
S_LOAD, S_LEVEL, S_SIL, S_TIERS, S_EXPORT = range(len(STEPS))


def step_ref(i):
    """'step 3 (Silhouette)'.

    Never hand-write a step number in a message: the steps have been
    renumbered twice and the stale references only surface when a user hits
    that particular error.
    """
    num, _, name = STEPS[i].partition(" ")
    return f"step {num} ({name})"


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
        # NOT plt.figure("frame2solid"): a string num is a figure *label*, so
        # a second App would be handed the first one's figure and pile its
        # widgets onto that canvas — both apps' widgets then answer every
        # click. The name belongs on the window, not on the figure identity.
        self.fig = plt.figure(figsize=(15, 8.5))
        if self.fig.canvas.manager is not None:
            self.fig.canvas.manager.set_window_title("frame2solid")

        # ---------- state
        self.scan_path = ""
        self.orig_verts = None
        self.orig_faces = None
        self.M = np.eye(3)            # orientation applied to scan
        self.offset = np.zeros(3)     # translation applied after M
        self.verts = None             # oriented verts (cache)
        self.sil = None               # core.Silhouette
        self.level_pts = []           # [x,y,z] picks for the Level step
        self.level_maps = None        # coarse maps for the Level view
        self._pre_level = None        # (M, offset, picks) for Undo level
        self.maps = None              # core.ThicknessMaps (two-sided)
        self.base_thickness = None
        self.base_y0 = 0.0            # mid-plane of the base slab
        self.regions = []
        self.sel = None               # index of the selected region
        self.model = None             # cached sketch model for export
        self.step = 0
        self._radio_guard = False
        self._sync = False            # suppress widget callbacks while syncing
        self._pick_base_armed = False

        # ---------- axes
        self.ax_main = self.fig.add_axes([0.05, 0.16, 0.58, 0.76])
        self.ax_side = self.fig.add_axes([0.05, 0.55, 0.27, 0.37])
        self.ax_front = self.fig.add_axes([0.36, 0.55, 0.27, 0.37])
        self.ax_top = self.fig.add_axes([0.05, 0.10, 0.27, 0.37])
        self.orient_axes = [self.ax_side, self.ax_front, self.ax_top]
        # the Level step borrows the front and top views (that is where the
        # two components of a residual tilt are visible) and shrinks the
        # main view to make room, so it re-positions all three
        self.POS_MAIN = [0.05, 0.16, 0.58, 0.76]
        self.POS_O_FRONT = [0.36, 0.55, 0.27, 0.37]
        self.POS_O_TOP = [0.05, 0.10, 0.27, 0.37]
        self.POS_L_MAIN = [0.05, 0.34, 0.45, 0.58]
        self.POS_L_FRONT = [0.525, 0.34, 0.11, 0.58]
        self.POS_L_TOP = [0.05, 0.15, 0.45, 0.19]
        # the Tiers step shows the two sides as separate views. The left
        # one is mirrored in X, so it is what you see standing on the
        # frame's left, not the right-hand view with the far side through.
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
        self.w1 = self.w_load = []
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
        # ---------- step 2 widgets (Level)
        self.w_level = []
        r = slot()
        self.bt_level = Button(self.fig.add_axes(r),
                               "LEVEL — square up on the picked points")
        self.bt_level.on_clicked(self._on_level)
        self.w_level.append(self.bt_level)
        r = slot(split=(0.67, 0.145))
        self.bt_lvl_clear = Button(self.fig.add_axes(r), "Clear points")
        self.bt_lvl_clear.on_clicked(self._on_level_clear)
        self.bt_lvl_undo = Button(self.fig.add_axes([0.835, r[1], 0.145, r[3]]),
                                  "Undo level")
        self.bt_lvl_undo.on_clicked(self._on_level_undo)
        self.w_level += [self.bt_lvl_clear, self.bt_lvl_undo]
        r = slot()
        self.bt_lvl_stl = Button(self.fig.add_axes(r),
                                 "Export oriented STL (for Blender etc.)")
        self.bt_lvl_stl.on_clicked(self._on_export_oriented)
        self.w_level.append(self.bt_lvl_stl)

        self._wy = 0.745
        # ---------- step 3 widgets (Silhouette)
        self.w2 = self.w_sil = []
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
        # ---------- step 4 widgets (Tiers)
        self.w3 = self.w_tier = []
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
        # ---------- step 5 widgets (Export)
        self.w4 = self.w_export = []
        r = slot(split=(0.78, 0.10))
        self.tb_clr = TextBox(self.fig.add_axes(r), "clearance mm ",
                              initial="0.15")
        self.w4.append(self.tb_clr)
        r = slot(split=(0.78, 0.10))
        self.tb_epx = TextBox(self.fig.add_axes(r), "export px ", initial="0.2")
        self.w4.append(self.tb_epx)
        r = slot(split=(0.78, 0.10))
        self.tb_cfit = TextBox(self.fig.add_axes(r), "circle fit mm ",
                               initial="0.3")
        self.w4.append(self.tb_cfit)
        r = slot()
        self.tb_out = TextBox(self.fig.add_axes(r), "name ",
                              initial="frame_solid")
        self.w4.append(self.tb_out)
        r = slot(h=0.06)
        self.bt_cad = Button(self.fig.add_axes(r), "EXPORT CAD (DXF + SCAD)")
        self.bt_cad.on_clicked(self._on_export_cad)
        self.w4.append(self.bt_cad)

        # ---------- always-visible project row
        self.tb_proj = TextBox(self.fig.add_axes([0.67, 0.075, 0.31, 0.045]),
                               "proj ", initial="project.json")
        self.bt_psave = Button(self.fig.add_axes([0.67, 0.02, 0.145, 0.045]),
                               "Save project")
        self.bt_pload = Button(self.fig.add_axes([0.835, 0.02, 0.145, 0.045]),
                               "Load project")
        self.bt_psave.on_clicked(self._on_proj_save)
        self.bt_pload.on_clicked(self._on_proj_load)

        self._groups = [self.w_load, self.w_level, self.w_sil, self.w_tier,
                        self.w_export]

        # ---------- events
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
        return self._groups[i]

    def _set_step(self, i):
        self.step = i
        if STEPS.index(self.radio_step.value_selected) != i:
            self._radio_guard = True
            try:
                self.radio_step.set_active(i)
            finally:
                self._radio_guard = False
        for group in self._groups:
            for w in group:
                w.ax.set_visible(False)
                self._enable(w, False)
        for w in self._widgets_for(i):
            w.ax.set_visible(True)
            self._enable(w, True)
        show_orient = (i == S_LOAD)
        show_level = (i == S_LEVEL)
        show_tiers = (i == S_TIERS)
        self.ax_side.set_visible(show_orient)
        for ax in (self.ax_front, self.ax_top):
            ax.set_visible(show_orient or show_level)
        if show_level:
            self.ax_main.set_position(self.POS_L_MAIN)
            self.ax_front.set_position(self.POS_L_FRONT)
            self.ax_top.set_position(self.POS_L_TOP)
        else:
            self.ax_main.set_position(self.POS_MAIN)
            self.ax_front.set_position(self.POS_O_FRONT)
            self.ax_top.set_position(self.POS_O_TOP)
        for ax in self.tier_axes.values():
            ax.set_visible(show_tiers)
        self.ax_main.set_visible(not show_orient and not show_tiers)
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
        if self.step != S_LOAD:
            return
        p = _try_filedialog()
        if p:
            self.tb_path.set_val(p)
            self._on_load(None)

    def _on_load(self, _):
        if self.step != S_LOAD:
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
        self.level_pts = []
        self._pre_level = None
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
        self.level_pts = []
        self._invalidate()
        self._status("PCA auto-orient applied. Check all three views; fix "
                     "with the 90° buttons if axes are swapped/flipped.")
        self._draw()

    def _rotate(self, axis, deg=90):
        if self.orig_verts is None:
            return
        self.M = core.rot_matrix(axis, deg) @ self.M
        self._apply_transform(center=True)
        self.level_pts = []
        self._invalidate()
        self._draw()

    def _on_center(self, _):
        if self.orig_verts is None:
            return
        self._apply_transform(center=True)
        self.level_pts = []
        self._invalidate()
        self._draw()

    def _invalidate(self):
        """Drop everything derived from the oriented scan."""
        self.sil = None
        self.maps = None
        self.level_maps = None
        self.model = None

    # ================================================== step 2: level
    def _level_map(self):
        """Coarse right-face map for the Level view (no silhouette yet)."""
        if self.level_maps is None and self.verts is not None:
            with self._busy("measuring surface…"):
                self.level_maps = core.measure_maps(
                    self.verts, self.orig_faces, None, px=0.8)
        return self.level_maps

    def _level_fit(self):
        """(tilt_deg, rms_mm) of the current picks, or None if under 3."""
        if len(self.level_pts) < 3:
            return None
        _, tilt, rms = core.level_rotation(np.asarray(self.level_pts, float))
        return tilt, rms

    def pick_level_point(self, x, z):
        """Record where the scan's RIGHT face sits under a clicked XZ point."""
        maps = self._level_map()
        if maps is None:
            self._status(f"Load a scan first — {step_ref(S_LOAD)}.")
            return
        s = core.sample_maps(maps, x, z, r_mm=1.2)
        if s is None:
            self._status("No scan surface there — click on the frame.")
            return
        self.level_pts.append([float(x), float(s["hR"]), float(z)])
        fit = self._level_fit()
        extra = ""
        if fit:
            extra = (f"  Plane through {len(self.level_pts)} points: tilt "
                     f"{fit[0]:.2f}°, points coplanar to ±{fit[1]:.3f} mm. "
                     f"Press LEVEL.")
        self._status(f"Point {len(self.level_pts)} at X={x:.1f}, Z={z:.1f} — "
                     f"right face y={s['hR']:+.3f}.{extra}")
        self._draw()

    def _on_level_clear(self, _):
        if self.step != S_LEVEL:
            return
        self.level_pts = []
        self._status("Picks cleared.")
        self._draw()

    def _on_level(self, _):
        if self.step != S_LEVEL:
            return
        if len(self.level_pts) < 3:
            self._status("Pick at least 3 points on a flat part of the "
                         "frame's right side first.")
            return
        pts = np.asarray(self.level_pts, float)
        R, tilt, rms = core.level_rotation(pts)
        self._pre_level = (self.M.copy(), self.offset.copy(), pts.copy())
        off_old = self.offset.copy()
        with self._busy("levelling…"):
            self.M = R @ self.M
            self._apply_transform(center=True)
            # carry the picks into the new frame, so you can level again
            self.level_pts = ((pts - off_old) @ R.T + self.offset).tolist()
            self._invalidate()
        after = self._level_fit()
        self._status(f"Levelled: rotated {tilt:.3f}° so the picked face is "
                     f"square to Y. The {len(pts)} points were coplanar to "
                     f"±{rms:.3f} mm (that is your scan's own flatness) and "
                     f"now sit within {after[1] * 2:.3f} mm of one Y. "
                     f"Re-extract the silhouette: {step_ref(S_SIL)}.")
        self._draw()

    def _on_export_oriented(self, _):
        """Write the scan back out with the orientation baked in.

        A rigid transform of the vertices — same triangles, same topology,
        nothing resampled — so sharp edges and open boundaries survive
        exactly. Getting a scan square is the fiddly part; this hands the
        result to Blender/MeshMixer so the rest of the work happens on a
        datum that already means something.
        """
        if self.step != S_LEVEL:
            return
        if self.verts is None or self.orig_faces is None:
            self._status(f"Load a scan first — {step_ref(S_LOAD)}.")
            return
        out = os.path.splitext(self.scan_path or "scan")[0] + "_oriented.stl"
        try:
            with self._busy("writing STL…"):
                save_stl(out, self.verts, self.orig_faces)
        except OSError as e:
            self._status(f"Could not write {out}: {e}")
            return
        self._status(f"Wrote {out} — the same {len(self.orig_faces)} triangles, "
                     f"rigidly transformed: X along the bore, Y across the "
                     f"frame, Z up, centred on the bounding box. Nothing was "
                     f"resampled, so edges and hole boundaries are untouched.")

    def _on_level_undo(self, _):
        if self.step != S_LEVEL or self._pre_level is None:
            return
        self.M, self.offset, pts = self._pre_level
        self._pre_level = None
        self.verts = self.orig_verts @ self.M.T + self.offset
        self.level_pts = pts.tolist()
        self._invalidate()
        self._status("Levelling undone — orientation is back as it was.")
        self._draw()

    # ================================================== step 3: silhouette
    def _on_extract(self, _):
        if self.verts is None:
            self._status(f"Load a scan first — {step_ref(S_LOAD)}.")
            return
        try:
            px = float(self.tb_px.text)
            cl = float(self.tb_close.text)
            sp = float(self.tb_simp.text)
            with self._busy("extracting silhouette…"):
                self.sil = core.extract_silhouette(self.verts, self.orig_faces,
                                                   px=px, close_mm=cl,
                                                   simplify_mm=sp)
        except Exception as e:
            self._status(f"Silhouette failed: {e}")
            return
        self.maps = None
        self.model = None
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
        # the maps just changed (new scan, new orientation, new silhouette),
        # so every tier outline cut from the old ones is stale
        self._resegment()
        self._sync_boxes()

    # ================================================== step 4: tiers
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
        if self.step != S_TIERS:
            return
        if self.maps is None:
            self._status(f"Extract a silhouette first — {step_ref(S_SIL)}.")
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
        self.model = None
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
            self._status(f"Extract a silhouette first — {step_ref(S_SIL)}.")
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
                self.model = None
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

        Nothing in the Tiers step re-cuts on focus change, so you can tab
        around the boxes freely and pay for the update exactly once, when
        you say so.
        """
        if self.step != S_TIERS:
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
            self.model = None
            self._sync_boxes()
            self._refresh_list()
            note = self._coverage_note()
        self._status("Applied: " + "; ".join(changes) + "." + note)
        self._draw()

    def _on_key(self, event):
        """Enter applies, wherever the keyboard focus happens to be."""
        if event.key in ("enter", "return") and self.step == S_TIERS:
            self._apply_edits()

    def _on_remeasure(self, _):
        if self.sil is None:
            self._status(f"Extract a silhouette first — {step_ref(S_SIL)}.")
            return
        with self._busy("re-measuring scan…"):
            self._compute_maps()
            self._resegment()
            self.model = None
            self._sync_boxes()
            self._refresh_list()
            note = self._coverage_note()
        self._status(f"Thickness maps re-measured at {self.maps.px} mm/px; "
                     f"{len(self.regions)} tier(s) re-cut.{note}")
        self._draw()

    def _on_region_del(self, _):
        if self.step != S_TIERS or not self.regions:
            return
        i = self.sel if self.sel is not None else len(self.regions) - 1
        with self._busy("deleting tier…"):
            if 0 <= i < len(self.regions):
                self.regions.pop(i)
            self.sel = min(i, len(self.regions) - 1) if self.regions else None
            self.model = None
            self._resegment()    # the tier above inherits a lower threshold
        self._select(self.sel)

    def _on_region_clr(self, _):
        if self.step != S_TIERS:
            return
        self.regions = []
        self.sel = None
        self.model = None
        self._select(None)

    # ================================================== clicks
    def _toolbar_idle(self):
        tb = self.fig.canvas.toolbar
        return tb is None or getattr(tb, "mode", "") == ""

    def _on_click(self, event):
        if not self._toolbar_idle():
            return
        if event.xdata is None or event.ydata is None:
            return
        x, z = event.xdata, event.ydata
        if self.step == S_LEVEL:
            if event.inaxes is self.ax_main and event.button == 1:
                self.pick_level_point(x, z)
            return
        if self.step == S_TIERS:                 # one view per side
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

    # ================================================== step 5: export
    def _sketch_model(self, regions=None):
        """The model as sketches + extrusion depths (cached for the preview)."""
        if regions is None and self.model is not None:
            return self.model
        if self.sil is None or self.base_thickness is None:
            return None
        try:
            clr = float(self.tb_clr.text)
        except ValueError:
            clr = 0.0
        try:
            cfit = max(0.0, float(self.tb_cfit.text))
        except ValueError:
            cfit = 0.0
        model = core.sketch_model(
            self.sil, self.base_thickness,
            self.regions if regions is None else regions,
            base_y0=self.base_y0, clearance=clr, circle_tol=cfit)
        self.model = model
        return model

    def _refined_regions(self):
        """Re-cut the tiers on a finer raster than step 4 works at.

        Step 4 has to stay responsive, so it measures at `map px` (0.5 mm by
        default) — plenty to pick tiers on, but it quantises every tier
        boundary to that grid. The export can afford better: re-measure, re-cut
        COPIES of the tiers, and leave the interactive state alone. Returns
        (regions, note).
        """
        try:
            epx = float(self.tb_epx.text)
        except ValueError:
            return self.regions, ""
        if self.maps is None or epx <= 0 or epx >= self.maps.px:
            return self.regions, ""
        fine = core.measure_maps(self.verts, self.orig_faces, self.sil, px=epx)
        regions = [dict(r) for r in self.regions]     # segment_tier replaces
        core.segment_all(fine, regions, self.base_thickness, self.base_y0)
        # Refining must not change WHAT is included, only where its edges
        # land. If a tier's footprint moves by more than a few percent the
        # finer measurement disagrees with what was reviewed, so keep the
        # coarse outline for that tier and say so. The allowance is 5% OR the
        # conservative-raster bias the finer measurement is supposed to shed,
        # whichever is larger — see core.raster_bias_bound, because a flat
        # percentage punishes small high-perimeter islands for getting
        # measured better.
        drifted = []
        for old, new in zip(self.regions, regions):
            if core.region_kind(new) != "tier":
                continue
            a0 = old.get("area_mm2") or 0.0
            a1 = new.get("area_mm2") or 0.0
            allow = max(0.05 * a0, core.raster_bias_bound(
                old.get("polys") or [], self.maps.px, epx))
            if a0 > 0 and abs(a1 - a0) > allow:
                new["polys"] = old.get("polys", [])
                new["area_mm2"] = a0
                drifted.append(f"{new['side']} +{new['add_mm']:g}")
        note = f" Re-cut at {epx:g} mm/px."
        if drifted:
            note += (f" {len(drifted)} tier(s) ({', '.join(drifted)}) changed "
                     f"footprint by more than the raster can account for and "
                     f"kept their step-4 outlines — try a coarser export px.")
        return regions, note

    def _on_export_cad(self, _):
        if self.sil is None:
            self._status(f"Need a silhouette first — {step_ref(S_SIL)}.")
            return
        if self.base_thickness is None:
            self._status(f"Set the base thickness first — {step_ref(S_TIERS)}.")
            return
        try:
            clr = float(self.tb_clr.text)
        except ValueError:
            self._status("Clearance must be a number.")
            return
        folder = os.path.splitext(self.tb_out.text.strip()
                                  or "frame_solid")[0] + "_cad"
        try:
            with self._busy("re-cutting tiers, exporting…"):
                self.model = None            # clearance may have changed
                regions, note = self._refined_regions()
                model = self._sketch_model(regions)
                files = core.export_cad(folder, model, meta={
                    "scan": self.scan_path, "clearance": clr,
                    "base_y0": self.base_y0})
        except Exception as e:
            self._status(f"CAD export failed: {e}")
            return
        pts = sum(len(l) for s in model for l in s["loops"])
        circles = sum(len(s.get("circles", ())) for s in model)
        cnote = (f" {circles} round island(s) went out as true DXF circles."
                 if circles else "")
        self._status(f"Exported {len(model)} sketches ({pts} outline points) to "
                     f"{folder}\\ ({len(files)} files): one DXF each plus "
                     f"all_sketches.dxf, build.txt (the extrusion table) and "
                     f"assembly.scad.{note}{cnote} Clearance {clr:g} mm is "
                     f"baked in — set it to 0 for nominal outlines.")
        self._draw()

    # ================================================== project I/O
    def _on_proj_save(self, _):
        d = {"scan_path": self.scan_path,
             "M": self.M.tolist(), "offset": self.offset.tolist(),
             "sil": {"px": self.tb_px.text, "close": self.tb_close.text,
                     "simplify": self.tb_simp.text},
             "base_thickness": self.base_thickness,
             "base_y0": self.base_y0,
             "level_pts": [list(map(float, p)) for p in self.level_pts],
             "map_px": self.tb_mpx.text,
             "regions": self.regions,
             "export": {"clearance": self.tb_clr.text,
                        "export_px": self.tb_epx.text,
                        "circle_fit": self.tb_cfit.text,
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
        exp = d.get("export", d.get("build", {}))     # "build" is the old key
        self.tb_clr.set_val(str(exp.get("clearance", "0.15")))
        self.tb_epx.set_val(str(exp.get("export_px", "0.2")))
        self.tb_cfit.set_val(str(exp.get("circle_fit", "0.3")))
        self.tb_out.set_val(str(exp.get("out", "frame_solid")))
        self.tb_mpx.set_val(str(d.get("map_px", "0.5")))
        # base_width is the pre-two-sided key name
        self.base_thickness = d.get("base_thickness", d.get("base_width"))
        self.base_y0 = float(d.get("base_y0", 0.0))
        self.level_pts = [list(p) for p in d.get("level_pts", [])]
        self.regions = d.get("regions", [])
        # Extras (boxes/cylinders for the magazine path and grip screws) were
        # dropped: that job is done in Blender now. Old projects still load —
        # their extras are simply not part of the model any more, and saying
        # so beats silently exporting a solid that is missing them.
        n_extras = len(d.get("extras") or [])
        dropped = (f" {n_extras} extra(s) in this project were ignored — "
                   f"extras are gone; add those clearances in Blender."
                   if n_extras else "")
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
            self.model = None
            self._status(f"Project loaded; scan reloaded ({len(v)} verts). "
                         f"Re-run {step_ref(S_SIL)} to continue.{dropped}")
        else:
            self._status("Project loaded (scan file not found — load "
                         "manually)." + dropped)
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
        if self.step == S_LOAD:
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

        if self.step == S_TIERS:   # tiers — one independent view per side
            self._draw_tiers()
            self.fig.canvas.draw_idle()
            return

        ax = self.ax_main
        ax.clear()

        if self.step == S_LEVEL:
            maps = self._level_map()
            if maps is not None:
                m = maps.hR
                finite = m[np.isfinite(m)]
                vmin, vmax = (np.percentile(finite, [2, 98]) if finite.size
                              else (0.0, 1.0))
                if vmax - vmin < 1e-6:
                    vmax = vmin + 1.0
                ax.imshow(m, origin="lower", cmap="coolwarm", vmin=vmin,
                          vmax=vmax, extent=maps.world_extent())
                for i, p in enumerate(self.level_pts):
                    ax.plot([p[0]], [p[2]], "kx", ms=11, mew=2.5)
                    ax.text(p[0] + 1.5, p[2] + 1.5, f"{i+1}: {p[1]:+.2f}",
                            fontsize=8, weight="bold")
                fit = self._level_fit()
                verdict = (f"tilt {fit[0]:.2f}°, coplanar to ±{fit[1]:.3f} mm"
                           if fit else
                           f"{len(self.level_pts)}/3 points picked")
                ax.set_title(
                    f"RIGHT face height, {vmin:.1f}–{vmax:.1f} mm — a tilt "
                    f"reads as a gradient\n"
                    f"click 3+ points on one flat face · {verdict}",
                    fontsize=9)
            else:
                ax.set_title(f"Load a scan first — {step_ref(S_LOAD)}")
            ax.set_aspect("equal")
            self._draw_level_aux()

        elif self.step == S_SIL:      # silhouette
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

        elif self.step == S_EXPORT:
            model = self._sketch_model()
            if model:
                colours = {"base": "0.35", "left": "tab:blue",
                           "right": "tab:red"}
                for i, s in enumerate(model):
                    c = colours.get(s.get("side", s["kind"]), "tab:orange")
                    for loop in s["loops"]:
                        p = np.asarray(loop, float)
                        ax.add_patch(MplPolygon(p, closed=True, fill=False,
                                                ec=c, lw=1.6, alpha=0.9))
                    for cx, cz, rr in s.get("circles", ()):
                        ax.add_patch(MplCircle((cx, cz), rr, fill=False,
                                               ec=c, lw=1.6))
                    # the outlines nest, so label them in a column rather
                    # than on top of each other at the frame's corner
                    ax.text(0.99, 0.97 - 0.035 * i,
                            f"{s['key']:<16} y {s['y_lo']:+7.2f} .."
                            f"{s['y_hi']:+7.2f}", color=c, fontsize=7.5,
                            family="monospace", ha="right", va="top",
                            transform=ax.transAxes)
                ax.autoscale_view()
                ax.set_title(f"{len(model)} sketches to export — every outline "
                             f"and its extrusion range in Y\n"
                             f"grey = base · blue = left tiers · red = right "
                             f"tiers", fontsize=9)
                ax.set_xlabel("X mm"); ax.set_ylabel("Z mm")
            else:
                ax.set_title("Set a base thickness and some tiers first")
            ax.set_aspect("equal")

        self.fig.canvas.draw_idle()

    def _draw_level_aux(self):
        """Front and top views on the Level step.

        A residual tilt splits into exactly two visible components: about X
        (leans the frame in the FRONT view) and about Z (leans it in the TOP
        view). Both get the Y=0 datum as a dotted line and, once three points
        are picked, the fitted plane's trace in red — level it and the red
        line lands parallel to the datum.
        """
        v = self.verts
        if v is None:
            return
        self._hist2d(self.ax_front, v[:, 1], v[:, 2], "", "Y (across) mm",
                     "Z mm")
        self._hist2d(self.ax_top, v[:, 0], v[:, 1], "", "X mm",
                     "Y (across) mm")
        self.ax_front.axvline(0, color="#00a5ff", ls="--", lw=1.4, zorder=5)
        self.ax_top.axhline(0, color="#00a5ff", ls="--", lw=1.4, zorder=5)

        about_x = about_z = None
        if len(self.level_pts) >= 3:
            pts = np.asarray(self.level_pts, float)
            n, c, _ = core.fit_plane(pts)
            if n[1] < 0:
                n = -n
            if abs(n[1]) > 1e-9:
                xb, zb = pts[:, 0].mean(), pts[:, 2].mean()
                zz = np.array([v[:, 2].min(), v[:, 2].max()])
                self.ax_front.plot(
                    c[1] - (n[0] * (xb - c[0]) + n[2] * (zz - c[2])) / n[1],
                    zz, "-", color="tab:red", lw=1.8, zorder=6)
                xx = np.array([v[:, 0].min(), v[:, 0].max()])
                self.ax_top.plot(
                    xx,
                    c[1] - (n[0] * (xx - c[0]) + n[2] * (zb - c[2])) / n[1],
                    "-", color="tab:red", lw=1.8, zorder=6)
                about_x = np.degrees(np.arctan2(n[2], n[1]))
                about_z = np.degrees(np.arctan2(n[0], n[1]))
            for p in pts:
                self.ax_front.plot([p[1]], [p[2]], "kx", ms=7, mew=1.8)
                self.ax_top.plot([p[0]], [p[1]], "kx", ms=7, mew=1.8)

        self.ax_front.set_title(
            "FRONT (Y-Z)" + (f" — tilt about X {about_x:+.2f}°"
                             if about_x is not None else ""), fontsize=9)
        self.ax_top.set_title(
            "TOP (X-Y)" + (f" — tilt about Z {about_z:+.2f}°"
                           if about_z is not None else ""), fontsize=9)

    def _draw_tiers(self):
        """The Tiers step: one view per side, each with its own tiers."""
        if self.maps is None and self.sil is not None:
            with self._busy("measuring scan…"):
                self._compute_maps()
        for side, ax in self.tier_axes.items():
            ax.clear()
            ax.tick_params(labelsize=7)
            if self.maps is None:
                ax.set_title(f"Extract a silhouette first — {step_ref(S_SIL)}",
                             fontsize=10)
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

    def _draw_regions(self, ax, side=None):
        colors = ["tab:orange", "tab:red", "tab:cyan", "tab:purple",
                  "tab:brown", "tab:pink", "yellow", "lime"]
        shown = 0
        for i, r in enumerate(self.regions):
            if side is not None and r.get("side", side) != side:
                continue
            c = colors[i % len(colors)]
            sel = i == self.sel
            lw = 3.0 if sel else 1.8
            if core.region_kind(r) == "rect":
                x0, x1 = min(r["x0"], r["x1"]), max(r["x0"], r["x1"])
                z0, z1 = min(r["z0"], r["z1"]), max(r["z0"], r["z1"])
                ax.add_patch(plt.Rectangle((x0, z0), x1 - x0, z1 - z0,
                                           fill=False, ec=c, lw=lw))
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
                                            lw=lw))
                ax.plot([r["x"]], [r["z"]], "+", color=c, ms=9, mew=2)
                # label at the pick point, stepped so tiers picked in the
                # same place don't write on top of each other
                lx, lz = r["x"] + 1.5, r["z"] + 1.5 + 3.5 * shown
                label = (f"#{i+1} {SIDE_TAG[r['side']].strip()} "
                         f"+{r['add_mm']:g}mm")
            shown += 1
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
