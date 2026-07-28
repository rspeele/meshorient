"""Headless end-to-end test of the frame2solid core pipeline."""
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from meshio_lite import load_mesh
import core

# ---- load synthetic scan
verts, faces = load_mesh("synthetic_frame_scan.stl")
print(f"loaded: {len(verts)} verts, {len(faces)} tris")

# scan is already in datum orientation; still exercise the helpers
R = core.auto_orient(verts)
verts_o = verts  # synthetic is already in datum pose (Y=0 symmetry plane)

# ---- silhouette
sil = core.extract_silhouette(verts_o, faces, px=0.15, close_mm=1.5)
print(f"silhouette: mask {sil.mask.shape}, polygon {len(sil.polygon)} pts")

# verify the magwell window was filled: the filled mask must have no holes
import cv2
cnts, hier = cv2.findContours(sil.mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
inner = sum(1 for h in hier[0] if h[3] != -1) if hier is not None else 0
print(f"interior holes in silhouette: {inner}  (expect 0)")
assert inner == 0

core.export_outline_dxf(sil, "outline.dxf")

# ---- width map + suggestions
wmap, wx0, wz0, wpx = core.width_map(verts_o, sil, px=0.6)
s_rail = core.suggest_width(wmap, wx0, wz0, wpx, rect=(-80, 32, 80, 48))
s_grip = core.suggest_width(wmap, wx0, wz0, wpx, rect=(-30, -55, 30, -20))
s_tang = core.suggest_width(wmap, wx0, wz0, wpx, rect=(-80, -5, -45, 25))
print(f"suggested widths: rail={s_rail:.2f} (exp ~26), "
      f"grip={s_grip:.2f} (exp ~22), tang={s_tang:.2f} (exp ~12)")
assert abs(s_rail - 26) < 1.0 and abs(s_grip - 22) < 1.0 and abs(s_tang - 12) < 1.0

# ---- regions & extras
base_w = s_grip
regions = [
    {"x0": -86, "z0": 29, "x1": 86, "z1": 52, "width": s_rail},
    {"x0": -86, "z0": -11, "x1": -41, "z1": 31, "width": s_tang},
]
extras = [
    {"kind": "cyl_y", "x": 55, "z": -20, "dia": 5.0, "ylen": 40.0},   # grip screw
    {"kind": "box", "x0": -25, "z0": -95, "x1": 15, "z1": -20,
     "ywidth": 20.0, "tilt_deg": 8.0},                                # mag path
]

# ---- build
CLEAR = 0.15
bverts, bfaces, report, sdf_pack = core.build_solid(
    sil, base_w, regions, extras, clearance=CLEAR, voxel=0.3, return_sdf=True)
print("report:", report)
assert report["watertight"], "output must be watertight"

# ---- verify plateau widths on the output mesh (rail should be ~26 + 2*clr)
def width_at(v, xlo, xhi, zlo, zhi):
    m = (v[:, 0] > xlo) & (v[:, 0] < xhi) & (v[:, 2] > zlo) & (v[:, 2] < zhi)
    return v[m][:, 1].max() - v[m][:, 1].min()

w_rail = width_at(bverts, -10, 10, 38, 44)
w_grip = width_at(bverts, 15, 30, -50, -30)
w_tang = width_at(bverts, -75, -60, 5, 20)
print(f"output widths: rail={w_rail:.2f} (exp ~{s_rail+2*CLEAR:.2f}), "
      f"grip={w_grip:.2f} (exp ~{base_w+2*CLEAR:.2f}), "
      f"tang={w_tang:.2f} (exp ~{s_tang+2*CLEAR:.2f})")
assert abs(w_rail - (s_rail + 2 * CLEAR)) < 0.7
assert abs(w_grip - (base_w + 2 * CLEAR)) < 0.7
assert abs(w_tang - (s_tang + 2 * CLEAR)) < 0.7

core.save_solid("frame_solid.stl", bverts, bfaces)
print("saved frame_solid.stl")

# ---- preview images -------------------------------------------------
fig, axes = plt.subplots(2, 3, figsize=(16, 9))
ex = sil.world_extent()

ax = axes[0, 0]
ax.imshow(sil.mask, origin="lower", extent=ex, cmap="gray")
ax.plot(np.r_[sil.polygon[:, 0], sil.polygon[0, 0]],
        np.r_[sil.polygon[:, 1], sil.polygon[0, 1]], "r-", lw=1)
ax.set_title("1. Outer silhouette (window auto-filled)")

ax = axes[0, 1]
h, w = wmap.shape
im = ax.imshow(wmap, origin="lower", cmap="viridis",
               extent=(wx0, wx0 + w * wpx, wz0, wz0 + h * wpx))
fig.colorbar(im, ax=ax, label="measured width (mm)")
ax.set_title("2. Thickness map from scan")

ax = axes[0, 2]
ax.imshow(sil.mask, origin="lower", extent=ex, cmap="gray", alpha=0.4)
colors = ["tab:orange", "tab:green", "tab:red"]
for i, r in enumerate(regions):
    ax.add_patch(plt.Rectangle((min(r["x0"], r["x1"]), min(r["z0"], r["z1"])),
                               abs(r["x1"] - r["x0"]), abs(r["z1"] - r["z0"]),
                               fill=False, ec=colors[i % 3], lw=2))
    ax.text(min(r["x0"], r["x1"]) + 2, min(r["z0"], r["z1"]) + 2,
            f'{r["width"]:.1f}mm', color=colors[i % 3], fontsize=9)
ax.set_title(f"3. Plateau regions (base {base_w:.1f} mm)")

ax = axes[1, 0]
m, mex = core.sdf_slice_y(sdf_pack, 0.0)
ax.imshow(m.T, origin="lower", extent=(mex[0], mex[1], mex[2], mex[3]),
          cmap="gray")
ax.set_title("4. Result section at Y=0 (note mag path + screw)")

ax = axes[1, 1]
m, mex = core.sdf_slice_x(sdf_pack, 0.0)
ax.imshow(m.T, origin="lower", extent=(mex[0], mex[1], mex[2], mex[3]),
          cmap="gray")
ax.set_aspect("equal")
ax.set_title("5. Cross-section at X=0 (plateaus + mag path)")

ax = axes[1, 2]
m, mex = core.sdf_slice_x(sdf_pack, 55.0)
ax.imshow(m.T, origin="lower", extent=(mex[0], mex[1], mex[2], mex[3]),
          cmap="gray")
ax.set_aspect("equal")
ax.set_title("6. Cross-section at X=55 (grip screw hole boss)")

for ax in axes.flat:
    ax.set_xlabel("mm"); ax.set_ylabel("mm")
fig.suptitle("frame2solid pipeline — synthetic frame test", fontsize=14)
fig.tight_layout()
fig.savefig("pipeline_preview.png", dpi=110)
print("saved pipeline_preview.png")
