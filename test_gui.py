"""Headless GUI test: drive the app programmatically, screenshot each step."""
import os

import matplotlib
matplotlib.use("Agg")

import numpy as np
import app as appmod
import core
from meshio_lite import load_mesh


a = appmod.App()


class FakeClick:
    """A matplotlib button_press_event, near enough to drive the app."""

    def __init__(self, ax, x, y, button=1):
        self.inaxes, self.xdata, self.ydata, self.button = ax, x, y, button

# --- messages that point at another step must name the right one. The steps
# have been renumbered twice and a stale "(step 2)" only surfaces when a user
# happens to hit that error, so derive them from STEPS via step_ref().
assert "(step " not in open("app.py", encoding="utf-8").read(), \
    "hand-written step number in a message — use step_ref(S_*)"
assert appmod.step_ref(appmod.S_SIL) == "step 3 (Silhouette)"
_probe = appmod.App()
_probe._set_step(appmod.S_TIERS)
_probe._on_pick_base(None)          # nothing extracted yet
assert appmod.step_ref(appmod.S_SIL) in _probe.txt_status.get_text(), \
    _probe.txt_status.get_text()
_probe._set_step(appmod.S_LEVEL)
_probe.pick_level_point(0, 0)       # nothing loaded yet
assert appmod.step_ref(appmod.S_LOAD) in _probe.txt_status.get_text(), \
    _probe.txt_status.get_text()
print("step references OK")

# --- resizing the window must be silent. matplotlib 3.11 decorates
# TextBox._resize with the mouse-event wrapper, so a ResizeEvent used to
# raise AttributeError once per text box (headless, mpl re-raises rather
# than printing, so this asserts by not blowing up).
from matplotlib.backend_bases import ResizeEvent
ResizeEvent("resize_event", a.fig.canvas)._process()
print("resize event clean")

# --- step 1: load + orient
a.tb_path.set_val("synthetic_frame_scan.stl")
a._on_load(None)
assert a.verts is not None
a.fig.savefig("gui_step1_orient.png", dpi=100)

# exercise rotation buttons round-trip
v0 = a.verts.copy()
a._rotate("y"); a._rotate("y"); a._rotate("y"); a._rotate("y")
assert np.allclose(a.verts, v0, atol=1e-9), "4x 90deg rotation should be identity"

# --- step 2: level. Introduce a tilt PCA would have left behind, then
# square it back up off three points on the grip's flat right wall.
a._set_step(appmod.S_LEVEL)
a.M = core.rot_matrix("x", 1.2) @ core.rot_matrix("z", -0.6) @ a.M
a._apply_transform(center=True)
a._invalidate()
flat = [(-30, 20), (30, 20), (-30, -50), (30, -50)]     # grip right wall
for x, z in flat:
    a._on_click(FakeClick(a.ax_main, x, z + 5.0))
assert len(a.level_pts) == 4
tilt_before, rms_before = a._level_fit()
print(f"level: tilt {tilt_before:.3f}° before, picks coplanar to "
      f"±{rms_before:.3f} mm")
assert 1.0 < tilt_before < 2.0, "should see the tilt we introduced"
a.fig.savefig("gui_step2_level.png", dpi=100)
a._on_level(None)
tilt_after, _ = a._level_fit()
print(f"level: tilt {tilt_after:.3f}° after")
assert tilt_after < 0.05, "levelling should square the picked face to Y"
a.fig.savefig("gui_step2_level_after.png", dpi=100)
# the front/top views are on this step too, so the correction is visible
assert a.ax_front.get_visible() and a.ax_top.get_visible()
assert not a.ax_side.get_visible(), "side view belongs to step 1"
assert len(a.ax_front.lines) >= 2, "datum + fitted plane trace"
# and the flat wall now reads the same thickness end to end
lvl = a._level_map()
ys = [core.sample_maps(lvl, x, z + 5.0)["hR"] for x, z in flat]
print("right-face heights after levelling:", [round(v, 3) for v in ys])
assert max(ys) - min(ys) < 0.15, ys
# the oriented scan can be written back out, orientation baked in
a.scan_path = "gui_oriented_probe.stl"          # don't clobber the input
a._on_export_oriented(None)
assert os.path.isfile("gui_oriented_probe_oriented.stl")
_v2, _f2 = load_mesh("gui_oriented_probe_oriented.stl")
assert len(_f2) == len(a.orig_faces), "must be a rigid transform, not a remesh"
assert np.allclose(_v2.min(0), a.verts.min(0), atol=1e-3)
assert np.allclose(_v2.max(0), a.verts.max(0), atol=1e-3)
# ... and it comes back already square: the flat wall reads one thickness
_m2 = core.measure_maps(_v2, _f2, None, px=0.8)
_ys = [core.sample_maps(_m2, x, z + 5.0)["hR"] for x, z in flat]
print("re-imported oriented STL, right-face heights:",
      [round(v, 3) for v in _ys])
assert max(_ys) - min(_ys) < 0.15, _ys
a.scan_path = "synthetic_frame_scan.stl"

# undo puts the orientation back
a._on_level_undo(None)
assert a._level_fit()[0] > 1.0
a._on_level(None)                       # re-level for the rest of the run

# --- step 3: silhouette
a._set_step(appmod.S_SIL)
a._on_extract(None)
assert a.sil is not None
a._on_dxf(None)
a.fig.savefig("gui_step3_silhouette.png", dpi=100)

# --- step 4: thickness tiers (two-sided, click driven)
a._set_step(appmod.S_TIERS)
assert a.maps is not None and a.base_thickness is not None
print("auto base:", a.base_thickness, "y0:", a.base_y0)

# the app re-centres the scan on its bbox, so world Z is shifted
zmax, zmin = a.verts[:, 2].max(), a.verts[:, 2].min()
Z = lambda z_authored: z_authored + 5.0      # authored Z -> centred Z
assert abs(zmax - Z(50)) < 0.5, "unexpected centring"

# base = the THINNEST part of the frame, here the front tang. The frame is
# asymmetric (right-side boss), so the bbox mid-plane is not the frame's and
# base_y0 has to absorb that.
a._on_pick_base(None)
assert a._pick_base_armed
a.pick_base_at(-70, Z(10))
assert not a._pick_base_armed
print("picked base:", a.base_thickness, "y0:", a.base_y0)
assert abs(a.base_thickness - 12) < 0.5
assert abs(a.base_y0 + 0.5) < 0.2, "mid-plane offset should be about -0.5 mm"

# tiers: one view per side, and a click lands in whichever view you clicked
a.tb_over.set_val("0.3")
a.tb_grow.set_val("0")
for side in ("left", "right"):
    a._on_click(FakeClick(a.tier_axes[side], 25, Z(-30)))   # grip walls, +5
    a._on_click(FakeClick(a.tier_axes[side], 0, Z(40)))     # rail,       +7
a._on_click(FakeClick(a.tier_axes["right"], 5, Z(4)))       # boss, right only
assert len(a.regions) == 5
assert [t["side"] for t in a.regions].count("right") == 3
# a click in the left view can only ever make a left tier
assert all(t["side"] == "left" for t in a.regions
           if abs(t["x"] - 25) < 1 and t["side"] == "left")
for t in a.regions:
    assert t["polys"], f"tier {t} came out empty"
print("tiers:", [(t["side"], t["add_mm"], t["area_mm2"]) for t in a.regions])

# tiers are kept in stacking order (side, then height), not click order
assert [t["side"] for t in a.regions] == ["left", "left", "right",
                                          "right", "right"]
adds = [t["add_mm"] for t in a.regions]
assert adds == sorted(adds[:2]) + sorted(adds[2:])
for t in a.regions:
    exp = {"left": [5.0, 7.0], "right": [5.0, 7.0, 8.0]}[t["side"]]
    assert min(abs(t["add_mm"] - e) for e in exp) < 0.4, t
a.fig.savefig("gui_step4_tiers.png", dpi=100)

# the tier stack must cover the whole scan
cov = core.coverage_report(a.maps, a.regions, a.base_thickness, a.base_y0)
print("coverage:", cov)
assert max(cov[s]["short_mm"] for s in ("left", "right")) < 0.4
assert "cover the whole scan" in a._coverage_note()

# each view draws only its own side's tiers
a._draw()
for side, ax in a.tier_axes.items():
    drawn = [t.get_text() for t in ax.texts if "+" in t.get_text()]
    want = sum(1 for t in a.regions if t["side"] == side)
    assert len(drawn) == want, (side, drawn)
    assert all(appmod.SIDE_TAG[side].strip() in d for d in drawn), (side, drawn)
# the left view is mirrored — it is the frame seen from its left
lo, hi = a.tier_axes["left"].get_xlim()
assert lo > hi, "left view should have X inverted"
assert a.tier_axes["right"].get_xlim()[0] < a.tier_axes["right"].get_xlim()[1]

# selection: any tier, not just the last one
a._select(0)
assert a.sel == 0 and a.tb_add.text == f"{a.regions[0]['add_mm']:g}"
a._step_sel(+1)
assert a.sel == 1
# right-clicking a view selects the tallest tier there — and only that side's
assert a._region_at(5, Z(4), "right") == 4, "boss tier should win over rail"
assert a._region_at(5, Z(4), "left") == 0, "left view sees only left tiers"
assert a._region_at(-70, Z(10), "right") is None, "the tang is base, not a tier"
# clicking a list row selects too
a.lst.on_select(4)
assert a.sel == 4

# editing is explicit: typing alone changes nothing until Apply
before = a.regions[4]["area_mm2"]
a.tb_grow.set_val("2")
assert a.regions[4]["area_mm2"] == before, "no re-cut before Apply"
a._apply_edits()
assert a.regions[4]["area_mm2"] > before, "grow_mm must dilate the outline"
a.tb_grow.set_val("0")
a._apply_edits()
a.tb_add.set_val("8.4")            # caliper override; outline must not move
area = a.regions[4]["area_mm2"]
a._apply_edits()
assert a.regions[4]["add_mm"] == 8.4
assert abs(a.regions[4]["area_mm2"] - area) < 1e-9
# Enter applies too, and applying twice is a no-op
a.tb_add.set_val("8")
a._on_key(type("E", (), {"key": "enter"})())
assert a.regions[4]["add_mm"] == 8.0
a._apply_edits()
assert "Nothing to apply" in a.txt_status.get_text()
# the busy badge is up *while* the re-cut runs, and gone afterwards
assert not a.txt_busy.get_visible()
seen = {}
_orig_segment_all = core.segment_all
core.segment_all = lambda *args, **kw: (
    seen.update(up=a.txt_busy.get_visible(), text=a.txt_busy.get_text()),
    _orig_segment_all(*args, **kw))[1]
a.tb_grow.set_val("1")
a._apply_edits()
core.segment_all = _orig_segment_all
assert seen["up"] and "applying" in seen["text"], seen
assert not a.txt_busy.get_visible()
a.tb_grow.set_val("0")
a._apply_edits()

# Interacting with a text box must not repaint the whole figure. matplotlib
# calls TextBox.stop_typing() on every box that was NOT clicked, and each of
# those did a full canvas.draw(): 7 renders (~1.5 s on this figure) before
# the cursor even appeared, plus one per keystroke.
from matplotlib.backend_bases import MouseEvent, KeyEvent
_draws = [0]
_real_draw = a.fig.canvas.draw
a.fig.canvas.draw = lambda *x, **k: (_draws.__setitem__(0, _draws[0] + 1),
                                     _real_draw(*x, **k))[1]
_bb = a.tb_over.ax.get_window_extent()
_x, _y = _bb.x0 + _bb.width / 2, _bb.y0 + _bb.height / 2
MouseEvent("motion_notify_event", a.fig.canvas, _x, _y)._process()   # hover
MouseEvent("button_press_event", a.fig.canvas, _x, _y, button=1)._process()
for _ch in "45":
    KeyEvent("key_press_event", a.fig.canvas, _ch)._process()
a.fig.canvas.draw = _real_draw
assert a.tb_over.text.endswith("45"), a.tb_over.text   # the keys landed
assert _draws[0] == 0, f"{_draws[0]} full figure redraws for one click + 2 keys"
print("widget interaction does no full redraws")
a.tb_over.stop_typing()
a.tb_over.set_val("0.3")

# deleting a middle tier re-cuts the one above it (its threshold drops)
a._select(3)                        # right rail tier
rail_thr = a.regions[3]["threshold_mm"]
boss_thr = a.regions[4]["threshold_mm"]
a._on_region_del(None)
assert len(a.regions) == 4
boss = a.regions[-1]
assert boss["side"] == "right" and abs(boss["add_mm"] - 8.0) < 0.4
assert boss["threshold_mm"] < boss_thr, "boss should now cut from the tier below"
assert abs(boss["threshold_mm"] - rail_thr) < 1e-6
a._on_click(FakeClick(a.tier_axes["right"], 0, Z(40)))   # put the rail back
assert len(a.regions) == 5

# base thickness / y0 also wait for Apply
a.tb_base.set_val("12.5")
assert a.base_thickness != 12.5
a._apply_edits()
assert a.base_thickness == 12.5
a.tb_base.set_val(f"{12.09:g}")
a._apply_edits()
a.fig.savefig("gui_step4_tiers.png", dpi=100)

# --- step 5: extras
a._set_step(appmod.S_EXTRAS)
a.tb_yw.set_val("20"); a.tb_tilt.set_val("8"); a.tb_yc.set_val("0")
a.add_extra_box(-25, zmin - 35, 15, zmax - 75)
a.tb_dia.set_val("5"); a.tb_ylen.set_val("40")
a.add_extra_cyl(55, zmax - 75)
assert a.extras[-1]["yc"] == 0.0
a.fig.savefig("gui_step5_extras.png", dpi=100)

# --- step 6: CAD export — the tool's actual output
a._set_step(appmod.S_EXPORT)
a.tb_clr.set_val("0.15")
a.tb_out.set_val("gui_frame_solid")
a._on_export_cad(None)
cad = "gui_frame_solid_cad"
assert os.path.isdir(cad)
want = ["00_base.dxf", "all_sketches.dxf", "build.txt", "assembly.scad"]
assert all(os.path.isfile(os.path.join(cad, f) )for f in want), os.listdir(cad)
dxfs = [f for f in os.listdir(cad) if f.endswith(".dxf")]
assert len(dxfs) == 1 + 5 + 2 + 1, dxfs      # base + tiers + extras + combined
sheet = open(os.path.join(cad, "build.txt"), encoding="utf-8").read()
assert "R3_plus" in sheet and "clearance     : 0.15" in sheet
print("CAD export OK:", sorted(os.listdir(cad)))
# the export step previews every sketch it is about to write
assert a.model is not None and len(a.model) == 1 + 5 + 2
a.fig.savefig("gui_step6_export.png", dpi=100)

# --- project save/load round trip
a.tb_proj.set_val("test_project.json")
a._on_proj_save(None)
b = appmod.App()
b.tb_proj.set_val("test_project.json")
b._on_proj_load(None)
assert len(b.regions) == 5 and len(b.extras) == 2
assert abs(b.base_y0 - a.base_y0) < 1e-9
assert all(t.get("polys") for t in b.regions), "outlines must survive the JSON"
assert b.orig_verts is not None
b._on_extract(None)
assert b.sil is not None
# a reloaded project re-exports without re-measuring anything
b._set_step(appmod.S_EXPORT)
b.tb_out.set_val("gui_reload")
b._on_export_cad(None)
assert os.path.isfile(os.path.join("gui_reload_cad", "build.txt"))
print("project round-trip OK")

# --- legacy projects (drag-rectangle regions) still load and build
legacy = appmod.App()
legacy.tb_proj.set_val("sample_project.json")
legacy._on_proj_load(None)
assert legacy.base_thickness and legacy.regions
assert core.region_kind(legacy.regions[0]) == "rect"
legacy._on_extract(None)
legacy.tb_out.set_val("gui_legacy")
legacy._on_export_cad(None)
assert os.path.isfile(os.path.join("gui_legacy_cad", "build.txt"))
print("legacy rectangle project OK")

print("ALL GUI TESTS PASSED")
