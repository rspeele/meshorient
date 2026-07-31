"""Headless end-to-end test of the frame2solid core pipeline."""
import os

import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from meshio_lite import load_mesh
import core


def read_dxf_entities(path):
    """Minimal DXF reader — enough to prove what we wrote is readable.

    Returns (segments, circles): LINE endpoint pairs and (x, y, r) circles.
    """
    with open(path) as fh:
        tok = [t.strip() for t in fh]
    pairs = list(zip(tok[0::2], tok[1::2]))
    segs, circles, cur, kind = [], [], {}, None
    allowed = {"LINE", "CIRCLE"}
    for code, val in pairs:
        if code == "0":
            if kind == "LINE":
                segs.append(((cur["10"], cur["20"]), (cur["11"], cur["21"])))
            elif kind == "CIRCLE":
                circles.append((cur["10"], cur["20"], cur["40"]))
            kind = val if val in allowed else None
            cur = {}
            assert val not in ("POLYLINE", "VERTEX", "SEQEND", "LWPOLYLINE"), \
                f"{path}: {val} is not portable — OpenSCAD rejects it"
        elif kind and code in ("10", "20", "11", "21", "40"):
            cur[code] = float(val)
    return segs, circles


def read_dxf_polylines(path):
    """Reassemble the LINE segments back into closed loops, in order."""
    segs, _ = read_dxf_entities(path)
    loops, cur, prev_end = [], [], None
    for a, b in segs:
        if cur and np.hypot(*np.subtract(a, prev_end)) > 1e-6:
            loops.append(np.asarray(cur, float))       # a break in the chain
            cur = []
        cur.append(a)
        prev_end = b
        if np.hypot(*np.subtract(b, cur[0])) < 1e-6:   # ring closed
            loops.append(np.asarray(cur, float))
            cur, prev_end = [], None
    if cur:
        loops.append(np.asarray(cur, float))
    return loops

# ---- load synthetic scan
verts, faces = load_mesh("synthetic_frame_scan.stl")
print(f"loaded: {len(verts)} verts, {len(faces)} tris")

# scan is already in datum orientation; still exercise the helpers
R = core.auto_orient(verts)
verts_o = verts  # synthetic is already in datum pose (Y=0 mid-plane)

# ---- the silhouette must be the UNION of the projected triangles.
# cv2.fillPoly applies even-odd across contours passed in one call, so a
# closed surface (front triangle + back triangle over the same pixel) used to
# cancel itself out. A plain cube is the minimal case: 12 triangles, and the
# whole projection is covered exactly twice.
_cube_v = np.array([[0, 0, 0], [20, 0, 0], [20, 10, 0], [0, 10, 0],
                    [0, 0, 30], [20, 0, 30], [20, 10, 30], [0, 10, 30]], float)
_cube_f = np.array([[0, 1, 2], [0, 2, 3], [4, 6, 5], [4, 7, 6],
                    [0, 4, 5], [0, 5, 1], [1, 5, 6], [1, 6, 2],
                    [2, 6, 7], [2, 7, 3], [3, 7, 4], [3, 4, 0]], np.int64)
_sil = core.extract_silhouette(_cube_v, _cube_f, px=0.15, close_mm=0.5,
                               min_area_mm2=1.0)
_p = np.asarray(_sil.polygon)
_area = abs(float(np.dot(_p[:, 0], np.roll(_p[:, 1], -1))
                  - np.dot(_p[:, 1], np.roll(_p[:, 0], -1)))) / 2.0
print(f"cube silhouette: {_area:.1f} mm2 (expect 20x30 = 600), "
      f"{len(_p)} pts, bbox x[{_p[:,0].min():.2f},{_p[:,0].max():.2f}] "
      f"z[{_p[:,1].min():.2f},{_p[:,1].max():.2f}]")
assert abs(_area - 600) / 600 < 0.05, "silhouette must be the union, not parity"
assert _p[:, 0].min() > -0.5 and _p[:, 0].max() < 20.5
assert _p[:, 1].min() > -0.5 and _p[:, 1].max() < 30.5

# ---- a tier boundary is measured on the coarse thickness map, so a straight
# feature edge arrives as a staircase. It must come back out straight: the
# simplify tolerance is tied to the MAP pixel, and the snap-to-profile runs
# first so protecting the profile no longer forces a fine tolerance. (Seen in
# OpenSCAD: the top edge of a plain rectangular block came out visibly jagged.)
_m = np.zeros((470, 470), np.uint8)
cv2.rectangle(_m, (33, 33), (436, 436), 255, -1)          # 5..65 mm at 0.15
_sil2 = core.Silhouette(_m, 0.0, 0.0, 0.15,
                        np.array([[5., 5.], [65., 5.], [65., 65.], [5., 65.]]))
_xs = np.arange(10.0, 60.01, 0.5)                          # 0.5 mm map pixels
_zs = np.round((15 + (_xs - 10) * 23 / 50) / 0.5) * 0.5    # ~25 deg staircase
_stair = []
for _i in range(len(_xs) - 1):
    _stair += [[_xs[_i], _zs[_i]], [_xs[_i + 1], _zs[_i]]]
_stair = np.asarray(_stair)
_clipped = core._clip_to_silhouette(
    [np.vstack([_stair, [[60., 58.], [10., 58.]]])], _sil2, simplify_mm=1.0)
_q = max(_clipped, key=len)
_edge = _q[(_q[:, 1] < 55) & (_q[:, 0] > 9) & (_q[:, 0] < 61)]
_c = _edge.mean(0)
_n = np.linalg.svd(_edge - _c)[2][1]
_wander = np.abs((_edge - _c) @ _n).max()
print(f"staircased edge: {len(_stair)} pts in -> {len(_edge)} out, "
      f"wander {_wander:.3f} mm (input steps were 0.5 mm)")
assert len(_edge) <= 6, f"{len(_edge)} points left on a straight edge"
assert _wander < 0.4, _wander

# ---- measure_maps rasterises the triangles; that has to be EXACT.
# It replaced scattering random points over each triangle, which left 26% of
# in-outline pixels empty at 0.5 mm/px and 52% at 0.1 — the finer the raster
# the emptier — and the grey-close that patched those holes is what put a
# staircase on every diagonal tier boundary. One big diagonal triangle of
# known area is the whole invariant: coverage converges on the true area from
# ABOVE (rasterising is conservative, so the excess is perimeter x px/2) and
# it must converge, not wander. A window sized from each bucket's lower bound
# instead of its largest member silently truncated triangles bigger than a
# pixel or two, which cost 8 mm off the end of a 39 mm feature.
_tri_v = np.array([[0., 5., 0.], [40., 5., 0.], [0., 5., 30.]])
_tri_f = np.array([[0, 1, 2]])
_prev_err = 1e9
for _px in (1.0, 0.5, 0.25, 0.1):
    _, _hi = core._raster_minmax(_tri_v, _tri_f, _px, -1.0, -1.0,
                                 int(42 / _px), int(32 / _px))
    _cov = (_hi > -1e8)
    _area, _err = _cov.sum() * _px ** 2, 0.0
    _err = _area - 600.0
    _peri = (40 + 30 + 50) * _px / 2
    print(f"  raster of a 600 mm2 diagonal triangle at {_px} mm/px: "
          f"{_area:7.1f} mm2 ({_err:+5.1f}, conservative bound {_peri:+.1f})")
    assert 0 <= _err <= 1.3 * _peri, "coverage must be exact to the half pixel"
    assert _err < _prev_err, "a finer raster must not be less accurate"
    _prev_err = _err
    assert np.allclose(_hi[_cov], 5.0), "interpolated Y must be the plane's"

# ---- levelling: a plane fit through picked points squares the scan up
flat = np.array([[-30.0, 11.0, 20.0], [30.0, 11.0, 20.0],
                 [-30.0, 11.0, -50.0], [30.0, 11.0, -50.0]])
tilt_R = core.rot_matrix("x", 1.3) @ core.rot_matrix("z", -0.7)
tilted = flat @ tilt_R.T
R, tilt, rms = core.level_rotation(tilted)
fixed = tilted @ R.T
print(f"level: {tilt:.4f} deg of tilt found, picks coplanar to {rms:.2e} mm, "
      f"y spread after {np.ptp(fixed[:, 1]):.2e} mm")
assert rms < 1e-9, "a perfect plane must fit perfectly"
assert np.ptp(fixed[:, 1]) < 1e-9, "levelled points must share one Y"
assert abs(tilt - 1.478) < 0.01          # combined 1.3 deg X + 0.7 deg Z
# levelling only removes tilt: it must not spin the frame about Y
assert abs((R @ tilt_R)[1, 1] - 1.0) < 1e-9
try:
    core.level_rotation(flat[:2])
except ValueError:
    pass
else:
    raise AssertionError("two points cannot define a plane")

# ---- silhouette
sil = core.extract_silhouette(verts_o, faces, px=0.15, close_mm=1.5)
print(f"silhouette: mask {sil.mask.shape}, polygon {len(sil.polygon)} pts")

# verify the magwell window was filled: the filled mask must have no holes
cnts, hier = cv2.findContours(sil.mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
inner = sum(1 for h in hier[0] if h[3] != -1) if hier is not None else 0
print(f"interior holes in silhouette: {inner}  (expect 0)")
assert inner == 0

core.export_outline_dxf(sil, "outline.dxf")

# ---- two-sided surface maps
maps = core.measure_maps(verts_o, faces, sil, px=0.5)

# the boss is right-side only: the two faces must disagree there
s_boss = core.sample_maps(maps, 5, 4)
s_grip = core.sample_maps(maps, 30, 20)
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
for side, (x, z), name in [("right", (30, 20), "grip"),
                           ("left", (30, 20), "grip"),
                           ("right", (24, -35), "rib"),    # diagonal strip
                           ("right", (0, 40), "rail"),
                           ("left", (0, 40), "rail"),
                           ("right", (5, 4), "boss"),
                           ("left", (25, 15), "rboss")]:   # round island
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

# A tall feature is built as several stacked extrusions — every tier below it
# also covers it. Where those tiers share a wall they must share it EXACTLY,
# or the steps between them show up as terracing on what should be one flat
# face (user-reported, seen in OpenSCAD).
def _gaps(P, refs):
    A = np.vstack([np.asarray(r) for r in refs])
    B = np.vstack([np.roll(np.asarray(r), -1, 0) for r in refs])
    d = B - A
    L2 = np.einsum("ij,ij->i", d, d)
    L2 = np.where(L2 > 0, L2, 1.0)
    tt = np.clip((P @ d.T - np.einsum("mj,mj->m", A, d)) / L2, 0, 1)
    o = P[:, None, :] - (A[None] + tt[:, :, None] * d[None])
    return np.sqrt(np.einsum("nmj,nmj->nm", o, o).min(1))


for _side in ("left", "right"):
    _stack = [t for t in tiers if t["side"] == _side]
    for _lo, _up in zip(_stack, _stack[1:]):
        _p = np.vstack([np.asarray(p) for p in _up["polys"]])
        _g = _gaps(_p, [np.asarray(p) for p in _lo["polys"]])
        _shared = _g[_g < 1.25]                      # the snap tolerance
        print(f"  {_side} +{_lo['add_mm']:.2f} -> +{_up['add_mm']:.2f}: "
              f"{len(_shared)}/{len(_p)} vertices on the lower outline, "
              f"worst offset {(_shared.max() if len(_shared) else 0):.4f} mm")
        assert not len(_shared) or _shared.max() < 0.05, \
            "stacked tiers must share their common wall exactly"

# Measuring the same scan more finely must give the SAME model, only with
# better-resolved edges — that is what lets the export re-cut at a finer
# raster than step 4 works at. _fill_gaps' kernels were in pixels, so a finer
# map bridged less dropout (81% -> 61% coverage of the profile on a real
# scan) and every tier shrank; finer was worse, not better.
_inside = maps.sil_mask > 0
_cov = np.isfinite(maps.hR)[_inside].mean()
_fine = core.measure_maps(verts_o, faces, sil, px=0.2)
_cov_f = np.isfinite(_fine.hR)[_fine.sil_mask > 0].mean()
_ref = [dict(t) for t in tiers]
core.segment_all(_fine, _ref, base, base_y0)
print(f"  resolution independence: coverage {100*_cov:.1f}% at "
      f"{maps.px} mm/px vs {100*_cov_f:.1f}% at {_fine.px} mm/px")
for _o, _n in zip(tiers, _ref):
    _p0 = sum(len(np.asarray(p)) for p in _o["polys"])
    _p1 = sum(len(np.asarray(p)) for p in _n["polys"])
    # The tolerance is ASYMMETRIC, because measure_maps' conservative
    # rasterisation biases every outline outward by ~half a pixel: refining the
    # raster is SUPPOSED to shed that, and the amount it sheds is
    # perimeter * dpx/2 — which for a small round island is a large fraction of
    # its area (the 8 mm boss goes 55.5 -> 51.4 mm2, true 50.3). Growing is
    # still capped at 6%: a finer raster resolving MORE frame would mean the
    # coarse map had missed some, which is the regression this test exists for
    # (fixed pixel kernels once cost 81% -> 61% of the profile).
    _shed = core.raster_bias_bound(_o["polys"], maps.px, _fine.px)
    _d = _n["area_mm2"] - _o["area_mm2"]
    print(f"    {_n['name']:4s} {_n['side']:>5s}: area {_o['area_mm2']:8.1f} -> "
          f"{_n['area_mm2']:8.1f} mm2 ({_d:+6.1f}, may shed {_shed:5.1f}), "
          f"{_p0:3d} -> {_p1:4d} pts")
    assert -_shed - 0.06 * _o["area_mm2"] < _d < 0.06 * _o["area_mm2"], \
        f"{_n['name']} changed area when re-measured finely"
# (point counts can go either way: these synthetic tiers are axis-aligned
# rectangles, which a finer raster describes with FEWER points, not more.
# On an organic scan boundary it is the other way round.)
assert abs(_cov_f - _cov) < 0.05, (_cov, _cov_f)

# The export's drift guard has to use that same allowance. Numbers from the
# real project that exposed it: a 30 mm-proud rod end, 91.8 mm2 of footprint
# with 77.5 mm of perimeter, legitimately sheds 14.1% going from 0.5 to
# 0.2 mm/px. A flat 5% guard reverted it to the coarse outline, which was too
# crude for fit_circle — so the same rod through the same hole came out round
# on one side of the frame and faceted on the other.
# 36.2 x 2.53 has exactly that tier's measured area AND perimeter — the bound
# scales with perimeter, so a compact stand-in would understate it badly
_rod = np.array([[0., 0.], [36.2, 0.], [36.2, 2.535], [0., 2.535]])
_rod_a, _rod_p = core._poly_area(_rod), float(np.linalg.norm(
    np.roll(_rod, -1, 0) - _rod, axis=1).sum())
assert abs(_rod_a - 91.8) < 0.5 and abs(_rod_p - 77.5) < 0.5, (_rod_a, _rod_p)
_bound = core.raster_bias_bound([_rod], 0.5, 0.2)
print(f"  drift allowance for a {_rod_a:.1f} mm2 / {_rod_p:.1f} mm island, "
      f"0.5 -> 0.2 mm/px: {_bound:.1f} mm2 ({100 * _bound / _rod_a:.1f}%) "
      f"vs a flat 5%")
assert _bound / _rod_a > 0.15, "must cover the 14.1% a real rod end shed"
# ...but a wholesale collapse must still trip it. When _fill_gaps was sized in
# pixels a tier fell from 21405 to 12746 mm2 at a finer raster; the bias bound
# for a footprint that large is a couple of percent, nowhere near 40%.
_big = np.array([[0., 0.], [200., 0.], [200., 107.], [0., 107.]])
assert core.raster_bias_bound([_big], 0.5, 0.2) < 0.05 * core._poly_area(_big)
assert core.raster_bias_bound([_rod], 0.2, 0.5) == 0.0, "finer-only"

# a small round island (a screw-clearance cylinder) must not be flattened to a
# triangle by the coarse tolerance a long straight edge needs
_disc = np.stack([12 + 1.75 * np.cos(np.linspace(0, 2*np.pi, 33)[:-1]),
                  8 + 1.75 * np.sin(np.linspace(0, 2*np.pi, 33)[:-1])], 1)
_kept = core._clip_to_silhouette([_disc], _sil2, simplify_mm=1.0)[0]
_a_in, _a_out = core._poly_area(_disc), core._poly_area(_kept)
print(f"  3.5 mm circle: 32 pts -> {len(_kept)} pts, area {_a_in:.2f} -> "
      f"{_a_out:.2f} mm2 ({100*(_a_out/_a_in - 1):+.1f}%)")
assert len(_kept) >= 8, f"circle collapsed to {len(_kept)} points"
assert abs(_a_out / _a_in - 1) < 0.05, "simplification must keep the footprint"

# the rail's through-hole must be swallowed, like the silhouette swallows
# the magwell window
from matplotlib.path import Path as _Path
assert any(_Path(np.asarray(p)).contains_point((57, 37))
           for p in by[("rail", "right")]["polys"]), \
    "tier outlines must fill interior holes"

# A tier that runs out to the edge of the frame must SHARE that edge with the
# base outline. Stopping a fraction of a millimetre short leaves a ledge in
# the built solid that no amount of grow_mm can close, because growing then
# clips back to the same inset boundary.
sil_path = _Path(np.asarray(sil.polygon))


def in_tier(t, q):
    return any(_Path(np.asarray(p)).contains_point(q) for p in t["polys"])


edge = ([(x, -60 + 0.3) for x in range(-30, 31, 10)]     # grip bottom edge
        + [(39.6, z) for z in range(-50, 21, 10)])       # grip rear edge
for name, t in (("grip", by[("grip", "right")]),
                ("grip grown", dict(by[("grip", "right")], grow_mm=1.5))):
    if "grown" in name:
        core.segment_tier(maps, tiers + [t], t, base, base_y0)
    missed = [q for q in edge if not in_tier(t, q)]
    print(f"  {name}: reaches the frame edge at {len(edge) - len(missed)}"
          f"/{len(edge)} probes 0.3 mm inside the outline")
    assert not missed, (name, missed)
    # ... and must not spill outside it either
    out = [q for p in t["polys"] for q in p
           if not sil_path.contains_point(q, radius=0.02)
           and not sil_path.contains_point(q, radius=-0.02)]
    assert not out, (name, out[:4])

# growth is a Minkowski dilation of the tier outline
grown = dict(by[("boss", "right")], grow_mm=2.0)
core.segment_tier(maps, tiers + [grown], grown, base, base_y0)
print(f"  boss: grow 0 -> {by[('boss', 'right')]['area_mm2']:.0f} mm², "
      f"grow 2 mm -> {grown['area_mm2']:.0f} mm²")
assert grown["area_mm2"] > by[("boss", "right")]["area_mm2"]

# ---- NON-AXIS-ALIGNED features. Rectangles cannot catch staircasing of a
# diagonal boundary or over-simplification of a curved one, which is where
# every artefact in this area has actually shown up.
def _island_at(t, cx, cz):
    for p in t["polys"]:
        p = np.asarray(p)
        if (p[:, 0].min() - 1 < cx < p[:, 0].max() + 1
                and p[:, 1].min() - 1 < cz < p[:, 1].max() + 1):
            return p
    raise AssertionError(f"no island of tier {t['name']} near ({cx}, {cz})")


_rib = _island_at(by[("rib", "right")], 24, -35)
_u = np.array([20.0, 34.0]); _u /= np.linalg.norm(_u)
_nrm = np.array([-_u[1], _u[0]]) * 2.0
_strip = np.array([[14, -52] + _nrm, [34, -18] + _nrm,
                   [34, -18] - _nrm, [14, -52] - _nrm])
_off = _gaps(_rib, [_strip])
print(f"  diagonal rib: {len(_rib)} pts, area {core._poly_area(_rib):.1f} mm2 "
      f"(true 157.6), vertices within {_off.max():.3f} mm of the true strip")
assert len(_rib) <= 9, f"{len(_rib)} points on a 4-corner strip — staircasing?"
assert _off.max() < 1.0, "rib outline wandered off the real feature"
# The bias is DIRECTIONAL, not symmetric. measure_maps rasterises
# conservatively — a pixel takes any triangle overlapping its square — so a
# feature comes out about half a pixel oversized per side, which on a 4 mm-wide
# rib at 0.5 mm/px is ~12% of area. That is the safe direction (a fat
# subtraction solid costs grip wall; a thin one fouls the frame) and it is why
# the rib must never measure UNDER true area. Testing pixel centres instead
# put it 7.7% under, and the excess has to shrink as the raster gets finer.
_a_rib = core._poly_area(_rib)
_rib_f = _island_at([t for t in _ref if t["name"] == "rib"][0], 24, -35)
_a_rib_f = core._poly_area(_rib_f)
print(f"  rib area bias: {_a_rib - 157.6:+.1f} mm2 at {maps.px} mm/px -> "
      f"{_a_rib_f - 157.6:+.1f} mm2 at {_fine.px} mm/px")
assert 157.6 <= _a_rib < 1.15 * 157.6, _a_rib
assert 157.6 <= _a_rib_f < _a_rib, "the outward bias must shrink with px"

_rb = _island_at(by[("rboss", "left")], 25, 15)
_a_rb = core._poly_area(_rb)
_rb_f = _island_at([t for t in _ref if t["name"] == "rboss"][0], 25, 15)
_a_rb_f = core._poly_area(_rb_f)
_r_true = np.pi * 16
print(f"  round boss: {len(_rb)} pts, area {_a_rb:.1f} mm2 at {maps.px} mm/px -> "
      f"{_a_rb_f:.1f} at {_fine.px} (true circle r=4 -> {_r_true:.1f})")
assert len(_rb) >= 10, f"circle flattened to {len(_rb)} points"
# Same directional bias as the rib, and worst on a small island: an 8 mm circle
# has 25 mm of perimeter around 50 mm2 of area, so half a pixel outward at
# 0.5 mm/px is +12%. Over is safe, under is not, and it has to converge.
assert _r_true <= _a_rb < 1.15 * _r_true, _a_rb
assert _r_true <= _a_rb_f < _a_rb, "the outward bias must shrink with px"

# ---- coverage: the tier stack must not leave the frame under-thick
cov = core.coverage_report(maps, tiers, base, base_y0)
print("coverage as picked:",
      {k: (v["short_mm"], v["area_mm2"]) for k, v in cov.items()})
if max(cov[s]["short_mm"] for s in ("left", "right")) > 0.4:
    # The diagonal rib's end comes up a sliver short: a dropped triangle
    # there drags the measured surface just under the threshold. That is what
    # coverage_report is for, and `grow` is the remedy — exercise both rather
    # than asserting the wart is or is not present.
    by[("rib", "right")]["grow_mm"] = 0.5
    core.segment_all(maps, tiers, base, base_y0)
    cov = core.coverage_report(maps, tiers, base, base_y0)
    print("coverage after growing the rib 0.5 mm:",
          {k: (v["short_mm"], v["area_mm2"]) for k, v in cov.items()})
for side in ("left", "right"):
    assert cov[side]["short_mm"] < 0.4, f"{side} left under-thick"
# drop the top tier and the boss becomes under-thick by ~1 mm
part = [t for t in tiers if t["name"] != "boss"]
short = core.coverage_report(maps, part, base, base_y0)["right"]
print("coverage without the boss tier:", short)
assert 0.8 < short["short_mm"] < 1.3
regions = tiers

# ---- CAD: the model as sketches + extrusion depths, which IS the output
CLEAR = 0.15
CFIT = 0.3
model = core.sketch_model(sil, base, regions, base_y0=base_y0,
                          clearance=CLEAR, circle_tol=CFIT)
assert len(model) == 1 + len(tiers)
print("sketch model:")
for s in model:
    print(f"  {s['key']:<24} y {s['y_lo']:+7.2f} .. {s['y_hi']:+7.2f}  "
          f"{len(s['loops'])} loop(s) {len(s['circles'])} circle(s)  "
          f"{s['label']}")

# ---- round islands become true CIRCLE entities. A 3-4 mm screw-clearance disc
# measured on a fine raster only supports about a dozen vertices, so a polygon
# can never be rounder than that; a CIRCLE is exact, is one object to drag in
# SolveSpace, and lets OpenSCAD tessellate at $fn. The synthetic's left-side
# boss is the case: a true circle of r=4 at (25, 15).
_lb = [s for s in model if s.get("side") == "left" and s["circles"]]
# TWO of them: the boss stands proud of the tier below, so that tier covers it
# as well, and both describe the same disc. That is the shared-wall case in its
# new form — as polygons those two walls once disagreed by 0.967 mm.
assert len(_lb) == 2, f"expected the boss in its tier and the one below: {_lb}"
assert all(len(s["circles"]) == 1 for s in _lb)
_c0, _c1 = (np.asarray(s["circles"][0]) for s in _lb)
print(f"  stacked circle walls agree to {np.abs(_c0 - _c1).max():.4f} mm")
assert np.abs(_c0 - _c1).max() < 0.05, (_c0, _c1)
_cx, _cz, _cr = _lb[-1]["circles"][0]
print(f"  circle from the scan: centre ({_cx:.2f}, {_cz:.2f}) r {_cr:.3f} "
      f"(true 25, 15, r 4 + {CLEAR} clearance)")
assert np.hypot(_cx - 25, _cz - 15) < 0.25, (_cx, _cz)
# r = true + the raster's conservative half pixel + clearance, and never under
assert 4.0 + CLEAR <= _cr < 4.0 + CLEAR + 0.6, _cr
# clearance goes on the radius analytically, not by re-rasterising the disc
_nom = core.sketch_model(sil, base, regions, base_y0=base_y0,
                         clearance=0.0, circle_tol=CFIT)
_nr = [s for s in _nom if s.get("side") == "left" and s["circles"]][-1]
assert abs((_cr - _nr["circles"][0][2]) - CLEAR) < 1e-9, "clearance is r + c"
# the circle must keep the polygon's footprint: that is how its radius is set
_poly_rb = _island_at(by[("rboss", "left")], 25, 15)
assert abs(np.pi * _nr["circles"][0][2] ** 2
           - core._poly_area(_poly_rb)) < 1e-6, "footprint must be preserved"
# and it is opt-out: circle_tol = 0 leaves everything a polygon
assert not any(s["circles"] for s in
               core.sketch_model(sil, base, regions, base_y0=base_y0,
                                 clearance=CLEAR, circle_tol=0.0)
               if s["kind"] == "tier")
# the detector must not turn angular features into circles
assert core.fit_circle(np.array([[0., 0.], [3., 0.], [3., 3.], [0., 3.]])) is None
_hex = np.stack([1.7 * np.cos(np.linspace(0, 2 * np.pi, 7)[:-1]),
                 1.7 * np.sin(np.linspace(0, 2 * np.pi, 7)[:-1])], 1)
assert core.fit_circle(_hex) is None, "a hexagon is not a circle"
for _t in tiers:
    for _p in _t["polys"]:
        # the boss is legitimately round wherever it appears, and it appears in
        # its own tier AND every tier below it; nothing else here is
        if np.hypot(*(np.asarray(_p).mean(0) - [25, 15])) > 6:
            assert core.fit_circle(_p, CFIT) is None, \
                f"{_t['name']} island wrongly read as a circle"

# every tier extrudes off the same plane, and clearance is grown in
for s in model:
    if s["kind"] == "tier":
        assert base_y0 in (s["y_lo"], s["y_hi"]), "tiers share one workplane"
by_key = {s["key"]: s for s in model}
base_sk = by_key["00_base"]
assert abs((base_sk["y_hi"] - base_sk["y_lo"]) - (base + 2 * CLEAR)) < 1e-6
# right tier 3 is the boss: its outer face must be the scan's, plus clearance
boss_sk = max((s for s in model if s.get("side") == "right"),
              key=lambda s: s["y_hi"])          # the tallest right tier
assert abs(boss_sk["y_hi"] - (14.0 + CLEAR)) < 0.4, boss_sk["y_hi"]
# the clearance-grown outline must enclose the nominal one
nominal = core.sketch_model(sil, base, regions, base_y0=base_y0,
                            clearance=0.0)
n_boss = max((s for s in nominal if s.get("side") == "right"),
             key=lambda s: s["y_hi"])
poly_g = _Path(boss_sk["loops"][0])
assert all(poly_g.contains_point(p) for p in n_boss["loops"][0]), \
    "clearance must grow the outline outward"


# the exported sketches must describe the solid we asked for
def cad_faces_at(model, x, z):
    """Y range of the union of the extruded sketches at a point."""
    lo, hi = np.inf, -np.inf
    for s in model:
        inside = any(_Path(l).contains_point((x, z)) for l in s["loops"])
        inside = inside or any(np.hypot(x - cx, z - cz) <= r
                               for cx, cz, r in s.get("circles", ()))
        if inside:
            lo, hi = min(lo, s["y_lo"]), max(hi, s["y_hi"])
    return lo, hi


for name, (x, z), exp_l, exp_r in [("rail", (0, 40), -13.0, 13.0),
                                   # clear of the rib, the boss and the round
                                   # boss — every one of which is proud here
                                   ("grip", (30, 20), -11.0, 11.0),
                                   ("tang", (-70, 10), -6.0, 6.0),
                                   ("boss", (0, 4), -11.0, 14.0)]:
    lo, hi = cad_faces_at(model, x, z)
    print(f"CAD {name}: y {lo:+.2f} .. {hi:+.2f}  "
          f"(expect {exp_l - CLEAR:+.2f} .. {exp_r + CLEAR:+.2f})")
    assert abs(lo - (exp_l - CLEAR)) < 0.35, name
    assert abs(hi - (exp_r + CLEAR)) < 0.35, name

files = core.export_cad("frame_solid_cad", model, meta={
    "scan": "synthetic_frame_scan.stl", "clearance": CLEAR,
    "base_y0": base_y0})
print(f"wrote {len(files)} CAD files to frame_solid_cad/")
assert all(os.path.isfile(f) for f in files)

# the DXFs must read back as the same closed loops, and must contain only
# entities every importer handles (OpenSCAD rejects POLYLINE/LWPOLYLINE)
for s in model:
    path = os.path.join("frame_solid_cad", s["key"] + ".dxf")
    segs, circles = read_dxf_entities(path)
    assert len(circles) == len(s.get("circles", ())), s["key"]
    # a circle must survive as a CIRCLE with its exact centre and radius —
    # that is the entire point of recognising it rather than emitting facets
    for got, want in zip(circles, s.get("circles", ())):
        assert np.allclose(got, want, atol=1e-6), (s["key"], got, want)
    if not s["loops"]:
        assert circles, f"{s['key']} exported nothing at all"
        continue
    loops = read_dxf_polylines(path)
    assert len(loops) == len(s["loops"]), (s["key"], len(loops))
    for a, b in zip(loops, s["loops"]):
        assert np.allclose(a, b, atol=1e-3), s["key"]
    assert len(segs) == sum(len(l) for l in s["loops"]), "loops must close"
read_dxf_entities(os.path.join("frame_solid_cad", "all_sketches.dxf"))
print("DXF round-trip OK (LINE/CIRCLE only)")

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
for sk in model:
    c = {"base": "0.4", "left": "tab:blue",
         "right": "tab:red"}.get(sk.get("side", sk["kind"]), "tab:orange")
    for loop in sk["loops"]:
        ax.add_patch(plt.Polygon(np.asarray(loop), closed=True, fill=False,
                                 ec=c, lw=1.4))
    for cx, cz, rr in sk.get("circles", ()):
        ax.add_patch(plt.Circle((cx, cz), rr, fill=False, ec=c, lw=1.4))
ax.autoscale_view()
ax.set_title(f"5. The {len(model)} exported sketches (grey base, blue L, red R)")


def cad_section_x(model, x, ys, zs):
    """Y-Z cross-section of the extruded sketches."""
    img = np.zeros((len(zs), len(ys)), bool)
    probe = np.column_stack([np.full_like(zs, x), zs])
    for sk in model:
        hit = np.zeros(len(zs), bool)
        for loop in sk["loops"]:
            hit |= _Path(np.asarray(loop)).contains_points(probe)
        for cx, cz, rr in sk.get("circles", ()):
            hit |= np.hypot(x - cx, zs - cz) <= rr
        img |= hit[:, None] & ((ys >= sk["y_lo"]) & (ys <= sk["y_hi"]))[None, :]
    return img


ax = axes[1, 2]
ys = np.linspace(-20, 20, 400)
zs = np.linspace(sil.polygon[:, 1].min(), sil.polygon[:, 1].max(), 500)
ax.imshow(cad_section_x(model, 5.0, ys, zs), origin="lower",
          extent=(ys[0], ys[-1], zs[0], zs[-1]), cmap="gray")
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
