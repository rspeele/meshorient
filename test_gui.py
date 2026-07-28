"""Headless GUI test: drive the app programmatically, screenshot each step."""
import matplotlib
matplotlib.use("Agg")

import numpy as np
import app as appmod
from meshio_lite import load_mesh, watertight_report

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

# --- step 3: regions
a._set_step(2)
assert a.wmap is not None and a.base_width is not None
print("auto base width:", a.base_width)
zoff = 0.0  # synthetic loads centered: recompute region coords from bbox
zmax = a.verts[:, 2].max()
zmin = a.verts[:, 2].min()
# rail = top 20 mm band, tang = left front block
a.add_region(-90, zmax - 22, 90, zmax + 2)
a.add_region(-90, zmax - 62, -41, zmax - 21)
print("regions:", a.regions)
a.fig.savefig("gui_step3_regions.png", dpi=100)

# --- step 4: extras
a._set_step(3)
a.tb_yw.set_val("20"); a.tb_tilt.set_val("8")
a.add_extra_box(-25, zmin - 35, 15, zmax - 75)
a.tb_dia.set_val("5"); a.tb_ylen.set_val("40")
a.add_extra_cyl(55, zmax - 75)
a.fig.savefig("gui_step4_extras.png", dpi=100)

# --- step 5: build + save
a._set_step(4)
a.tb_vox.set_val("0.35")
a.tb_clr.set_val("0.15")
a._on_build(None)
assert a.result is not None
_, _, rep, _ = a.result
print("build report:", rep)
assert rep["watertight"]
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
assert len(b.regions) == 2 and len(b.extras) == 2
assert b.orig_verts is not None
b._on_extract(None)
assert b.sil is not None
print("project round-trip OK")
print("ALL GUI TESTS PASSED")
