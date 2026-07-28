"""Headless GUI test: drive the app programmatically, screenshot each step."""
import matplotlib
matplotlib.use("Agg")

import numpy as np
import app as appmod
import core
from meshio_lite import load_mesh, watertight_report

SIDE = {s: appmod.SIDE_LABELS[i] for i, s in enumerate(appmod.SIDE_KEYS)}

a = appmod.App()

# --- step 1: load + orient
a.tb_path.set_val("synthetic_frame_scan.stl")
a._on_load(None)
assert a.verts is not None
a.fig.savefig("gui_step1_orient.png", dpi=100)

# exercise rotation buttons round-trip
v0 = a.verts.copy()
a._rotate("y"); a._rotate("y"); a._rotate("y"); a._rotate("y")
assert np.allclose(a.verts, v0, atol=1e-9), "4x 90deg rotation should be identity"

# --- step 2: silhouette
a._set_step(1)
a._on_extract(None)
assert a.sil is not None
a._on_dxf(None)
a.fig.savefig("gui_step2_silhouette.png", dpi=100)

# --- step 3: thickness tiers (two-sided, click driven)
a._set_step(2)
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

# tiers: grip walls and rail on both sides, then the right-only boss
a.tb_over.set_val("0.3")
a.tb_grow.set_val("0")
a._on_side(SIDE["both"])
a.add_region(25, Z(-30))          # grip walls  -> +5 mm each side
a.add_region(0, Z(40))            # rail        -> +7 mm each side
a._on_side(SIDE["right"])
a.add_region(5, Z(4))             # trigger-bar boss -> +8 mm, right only
assert len(a.regions) == 5, "both-sided picks add one tier per side"
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
a.fig.savefig("gui_step3_regions.png", dpi=100)

# the tier stack must cover the whole scan
cov = core.coverage_report(a.maps, a.regions, a.base_thickness, a.base_y0)
print("coverage:", cov)
assert max(cov[s]["short_mm"] for s in ("left", "right")) < 0.4
assert "cover the whole scan" in a._coverage_note()

# selection: any tier, not just the last one
a._select(0)
assert a.sel == 0 and a.tb_add.text == f"{a.regions[0]['add_mm']:g}"
a._step_sel(+1)
assert a.sel == 1
# right-clicking the map selects the tallest tier under the cursor
assert a._region_at(5, Z(4)) == 4, "the boss tier should win over the rail"
assert a._region_at(-70, Z(10)) is None, "the tang is base, not a tier"
# clicking a list row selects too
a.lst.on_select(4)
assert a.sel == 4

# editing the selected tier re-cuts it in place
before = a.regions[4]["area_mm2"]
a.tb_grow.set_val("2")
a._on_region_edit("2")
assert a.regions[4]["area_mm2"] > before, "grow_mm must dilate the outline"
a.tb_grow.set_val("0")
a._on_region_edit("0")
a.tb_add.set_val("8.4")            # caliper override; outline must not move
area = a.regions[4]["area_mm2"]
a._on_region_edit("8.4")
assert a.regions[4]["add_mm"] == 8.4
assert abs(a.regions[4]["area_mm2"] - area) < 1e-9
a.tb_add.set_val("8"); a._on_region_edit("8")

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
a.add_region(0, Z(40), side="right")   # put the rail tier back
assert len(a.regions) == 5
a.fig.savefig("gui_step3_regions.png", dpi=100)

# --- step 4: extras
a._set_step(3)
a.tb_yw.set_val("20"); a.tb_tilt.set_val("8"); a.tb_yc.set_val("0")
a.add_extra_box(-25, zmin - 35, 15, zmax - 75)
a.tb_dia.set_val("5"); a.tb_ylen.set_val("40")
a.add_extra_cyl(55, zmax - 75)
assert a.extras[-1]["yc"] == 0.0
a.fig.savefig("gui_step4_extras.png", dpi=100)

# --- step 5: build + save
a._set_step(4)
a.tb_vox.set_val("0.35")
a.tb_clr.set_val("0.15")
a._on_build(None)
assert a.result is not None
bverts, _, rep, _ = a.result
print("build report:", rep)
assert rep["watertight"]
assert rep["regions_skipped"] == 0

# the boss must come out one-sided: right face pushed out, left face at the
# grip tier
m = ((bverts[:, 0] > -5) & (bverts[:, 0] < 5)
     & (bverts[:, 2] > Z(3)) & (bverts[:, 2] < Z(5)))
lo, hi = bverts[m][:, 1].min(), bverts[m][:, 1].max()
print(f"boss faces in the built solid: {lo:+.2f} .. {hi:+.2f} "
      f"(expect ~-11.65 .. +13.65)")
assert abs(lo - (-11.5 - 0.15)) < 0.35
assert abs(hi - (13.5 + 0.15)) < 0.35
a.fig.savefig("gui_step5_build.png", dpi=100)

# section radio + slider redraws
a.sl_sec.set_val(0.4)
a.fig.savefig("gui_step5_build_section.png", dpi=100)

a.tb_out.set_val("gui_frame_solid.stl")
a._on_save(None)

# verify saved STL round-trips watertight
v, f = load_mesh("gui_frame_solid.stl")
print("saved STL:", watertight_report(v, f))
assert watertight_report(v, f)["watertight"]

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
# a reloaded project rebuilds to the same thing without re-measuring
b._set_step(4)
b.tb_vox.set_val("0.35")
b._on_build(None)
assert b.result is not None and b.result[2]["watertight"]
print("project round-trip OK")

# --- legacy projects (drag-rectangle regions) still load and build
legacy = appmod.App()
legacy.tb_proj.set_val("sample_project.json")
legacy._on_proj_load(None)
assert legacy.base_thickness and legacy.regions
assert core.region_kind(legacy.regions[0]) == "rect"
legacy._on_extract(None)
legacy.tb_vox.set_val("0.5")
legacy._on_build(None)
assert legacy.result is not None and legacy.result[2]["watertight"]
print("legacy rectangle project OK")

print("ALL GUI TESTS PASSED")
