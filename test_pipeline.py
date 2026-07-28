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
verts_o = verts  # synthetic is already in datum pose (Y=0 mid-plane)

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

# ---- two-sided surface maps
maps = core.measure_maps(verts_o, faces, sil, px=0.5)

# the boss is right-side only: the two faces must disagree there
s_boss = core.sample_maps(maps, 5, 4)
s_grip = core.sample_maps(maps, 25, -30)
print(f"faces at the boss: L={-s_boss['hL']:+.2f} R={s_boss['hR']:+.2f} "
      f"(expect -11 / +14);  plain grip wall: L={-s_grip['hL']:+.2f} "
      f"R={s_grip['hR']:+.2f} (expect -11 / +11)")
assert abs(s_boss["hL"] - 11) < 0.5 and abs(s_boss["hR"] - 14) < 0.5
assert abs(s_grip["hL"] - 11) < 0.5 and abs(s_grip["hR"] - 11) < 0.5

# ---- base tier: the THINNEST part of the frame (here the front tang)
s_tang = core.sample_maps(maps, -70, 10)
base = round(s_tang["total"], 2)
base_y0 = round((s_tang["hR"] - s_tang["hL"]) / 2, 2)
print(f"base tier (clicked on the tang): {base:.2f} mm @ y0 {base_y0:+.2f} "
      f"(expect 12 @ 0)")
assert abs(base - 12) < 0.5 and abs(base_y0) < 0.3

# ---- tiers: each click ropes in everything proud of the tier below it
tiers = []
for side, (x, z), name in [("right", (25, -30), "grip"),
                           ("left", (25, -30), "grip"),
                           ("right", (0, 40), "rail"),
                           ("left", (0, 40), "rail"),
                           ("right", (5, 4), "boss")]:
    t = core.make_tier(maps, x, z, side, base, base_y0, tiers=tiers)
    t["name"] = name
    tiers.append(t)
    core.segment_all(maps, tiers, base, base_y0)
tiers = core.sort_tiers(tiers)
for t in tiers:
    print(f"  {t['name']:4s} {t['side']:>5s}  +{t['add_mm']:5.2f} mm "
          f"(face {core.tier_offset(t, base, base_y0):5.2f})  "
          f"cuts at {t['threshold_mm']:5.2f}  "
          f"{t['area_mm2']:7.1f} mm² in {len(t['polys'])} island(s)")
    assert t["polys"], f"{t['name']}: tier outline is empty"

by = {(t["name"], t["side"]): t for t in tiers}
assert abs(by[("grip", "right")]["add_mm"] - 5.0) < 0.4     # 11 - 12/2
assert abs(by[("rail", "right")]["add_mm"] - 7.0) < 0.4     # 13 - 6
assert abs(by[("boss", "right")]["add_mm"] - 8.0) < 0.4     # 14 - 6

# a tier is a superset of the one above it: grip > rail > boss
assert (by[("grip", "right")]["area_mm2"] > by[("rail", "right")]["area_mm2"]
        > by[("boss", "right")]["area_mm2"])
# ... and the boss tier catches the boss only, the rail tier rail+boss
assert abs(by[("boss", "right")]["area_mm2"] - 45 * 8) / (45 * 8) < 0.30
assert len(by[("rail", "right")]["polys"]) == 2, "rail tier = rail + boss"

# the rail's through-hole must be swallowed, like the silhouette swallows
# the magwell window
from matplotlib.path import Path as _Path
assert any(_Path(np.asarray(p)).contains_point((57, 37))
           for p in by[("rail", "right")]["polys"]), \
    "tier outlines must fill interior holes"

# growth is a Minkowski dilation of the tier outline
grown = dict(by[("boss", "right")], grow_mm=2.0)
core.segment_tier(maps, tiers + [grown], grown, base, base_y0)
print(f"  boss: grow 0 -> {by[('boss', 'right')]['area_mm2']:.0f} mm², "
      f"grow 2 mm -> {grown['area_mm2']:.0f} mm²")
assert grown["area_mm2"] > by[("boss", "right")]["area_mm2"]

# ---- coverage: the tier stack must not leave the frame under-thick
cov = core.coverage_report(maps, tiers, base, base_y0)
print("coverage:", cov)
for side in ("left", "right"):
    assert cov[side]["short_mm"] < 0.4, f"{side} left under-thick"
# drop the top tier and the boss becomes under-thick by ~1 mm
part = [t for t in tiers if t["name"] != "boss"]
short = core.coverage_report(maps, part, base, base_y0)["right"]
print("coverage without the boss tier:", short)
assert 0.8 < short["short_mm"] < 1.3
regions = tiers

extras = [
    {"kind": "cyl_y", "x": 55, "z": -20, "dia": 5.0, "ylen": 40.0},   # grip screw
    {"kind": "box", "x0": -25, "z0": -95, "x1": 15, "z1": -20,
     "ywidth": 20.0, "tilt_deg": 8.0},                                # mag path
]

# ---- build
CLEAR = 0.15
bverts, bfaces, report, sdf_pack = core.build_solid(
    sil, base, regions, extras, clearance=CLEAR, voxel=0.3, return_sdf=True,
    base_y0=base_y0)
print("report:", report)
assert report["watertight"], "output must be watertight"
assert report["regions_skipped"] == 0


# ---- verify the built faces, per side
def faces_at(v, xlo, xhi, zlo, zhi):
    m = (v[:, 0] > xlo) & (v[:, 0] < xhi) & (v[:, 2] > zlo) & (v[:, 2] < zhi)
    return v[m][:, 1].min(), v[m][:, 1].max()


for name, box, exp_l, exp_r in [
        ("rail", (-10, 10, 38, 44), -13.0, 13.0),      # tier 2 both sides
        ("grip", (15, 30, -50, -30), -11.0, 11.0),     # tier 1 both sides
        ("tang", (-75, -60, 5, 20), -6.0, 6.0),        # base tier
        ("boss", (-5, 5, 3, 5), -11.0, 14.0)]:         # tier 3, RIGHT ONLY
    lo, hi = faces_at(bverts, *box)
    print(f"output {name}: y {lo:+.2f} .. {hi:+.2f}  "
          f"(expect {exp_l - CLEAR:+.2f} .. {exp_r + CLEAR:+.2f})")
    assert abs(lo - (exp_l - CLEAR)) < 0.35, name
    assert abs(hi - (exp_r + CLEAR)) < 0.35, name

core.save_solid("frame_solid.stl", bverts, bfaces)
print("saved frame_solid.stl")

# ---- preview images -------------------------------------------------
fig, axes = plt.subplots(2, 3, figsize=(16, 9))
ex = sil.world_extent()
mex = maps.world_extent()

ax = axes[0, 0]
ax.imshow(sil.mask, origin="lower", extent=ex, cmap="gray")
ax.plot(np.r_[sil.polygon[:, 0], sil.polygon[0, 0]],
        np.r_[sil.polygon[:, 1], sil.polygon[0, 1]], "r-", lw=1)
ax.set_title("1. Outer silhouette (window auto-filled)")

for ax, side, title in ((axes[0, 1], "left", "2. LEFT face (-Y)"),
                        (axes[0, 2], "right", "3. RIGHT face (+Y)")):
    im = ax.imshow(maps.face_map(side), origin="lower", cmap="viridis",
                   extent=mex)
    fig.colorbar(im, ax=ax, label="mm from mid-plane")
    ax.set_title(f"{title} — note the boss on the right only")

ax = axes[1, 0]
ax.imshow(sil.mask, origin="lower", extent=ex, cmap="gray", alpha=0.35)
colors = ["tab:orange", "tab:red", "tab:cyan", "tab:purple", "yellow"]
for i, r in enumerate(t for t in tiers if t["side"] == "right"):
    for p in r["polys"]:
        p = np.asarray(p)
        ax.add_patch(plt.Polygon(p, closed=True, fill=False, ec=colors[i],
                                 lw=2))
    ax.plot([r["x"]], [r["z"]], "+", color=colors[i], ms=9, mew=2)
    ax.text(r["x"], r["z"] + 2, f'+{r["add_mm"]:.1f}mm', color=colors[i],
            fontsize=9)
ax.set_title(f"4. RIGHT-side tier stack (base {base:.1f} mm)")

ax = axes[1, 1]
m, sex = core.sdf_slice_y(sdf_pack, 12.5)
ax.imshow(m.T, origin="lower", extent=(sex[0], sex[1], sex[2], sex[3]),
          cmap="gray")
ax.set_title("5. Section at Y=+12.5 (rail + boss only reach this far right)")

ax = axes[1, 2]
m, sex = core.sdf_slice_x(sdf_pack, 5.0)
ax.imshow(m.T, origin="lower", extent=(sex[0], sex[1], sex[2], sex[3]),
          cmap="gray")
ax.set_aspect("equal")
ax.axvline(0, color="tab:red", lw=0.8)
ax.set_title("6. Cross-section at X=5 (asymmetric: boss on +Y)")

for ax in axes.flat:
    ax.set_xlabel("mm"); ax.set_ylabel("mm")
fig.suptitle("frame2solid pipeline — synthetic frame test", fontsize=14)
fig.tight_layout()
fig.savefig("pipeline_preview.png", dpi=110)
print("saved pipeline_preview.png")
print("ALL PIPELINE TESTS PASSED")
