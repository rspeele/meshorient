"""frame2solid core pipeline.

Coordinate convention (after the Orient step):
    X = along the bore / frame length
    Y = across the frame (left-right); Y = 0 is the frame's mid-plane
    Z = vertical (grip height)
The "side view" is the XZ plane, looking along +Y. All units are mm.

Pipeline:
    scan mesh -> orient -> side-view silhouette (outer contour only, windows
    filled) -> two-sided surface maps (where the scan's left and right faces
    sit, per side-view pixel) -> thickness tiers -> extras (screw holes,
    magazine path, ...) -> sketch_model() + export_cad() -> one DXF per
    sketch plus the extrusion depths, which is the model exactly.

The output is CAD, not mesh: the tier model is a sketch-and-extrude model,
so it goes out losslessly as 2D outlines. Downstream, OpenSCAD turns
assembly.scad into an STL (or the finished grip) and SolveSpace/VCarve take
the DXFs directly.

The solid is NOT symmetric about Y=0. It is bounded by two independent face
surfaces yL(x,z) <= y <= yR(x,z), built from a stack of thickness TIERS per
side. The real frame's fuzzy, continuously varying thickness is quantised
into 3-5 flat tiers per side, always rounding UP:

    base       — the whole silhouette, at the thickness of the THINNEST part
                 of the frame, centred on base_y0
    tier       — picked by clicking a thicker feature. Its outline is
                 everything on that side standing proud of the tier below it
                 (by more than over_mm, to ride out scanner noise), wherever
                 it is on the frame; that outline is extruded out to this
                 tier's own height, add_mm proud of the base.

So each tier is a superset of the one above it, and they stack outward. A
feature only 0.5 mm proud of the tier below still gets pulled all the way up
to the next tier — deliberately: a too-thick subtraction solid only costs
grip wall thickness, a too-thin one means the grip fouls the frame.
core.coverage_report() measures what the tier stack still leaves short.
"""
from __future__ import annotations

import os

import numpy as np
import cv2

from meshio_lite import (save_dxf, save_dxf_polyline, dxf_polyline,
                         dxf_circle)


# ================================================================ orient

def rot_matrix(axis: str, deg: float) -> np.ndarray:
    a = np.deg2rad(deg)
    c, s = np.cos(a), np.sin(a)
    if axis == "x":
        return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])
    if axis == "y":
        return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])
    if axis == "z":
        return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
    raise ValueError(axis)


def auto_orient(verts: np.ndarray) -> np.ndarray:
    """PCA orientation guess: longest axis -> X, thinnest -> Y, middle -> Z.

    Returns a 3x3 rotation matrix R such that verts @ R.T is oriented.
    90-degree ambiguities are expected; the GUI provides flip/rotate buttons.
    """
    c = verts - verts.mean(axis=0)
    cov = np.cov(c.T)
    evals, evecs = np.linalg.eigh(cov)          # ascending
    e_small, e_mid, e_large = evecs[:, 0], evecs[:, 1], evecs[:, 2]
    R = np.stack([e_large, e_small, e_mid], axis=0)  # rows: new x, y, z
    if np.linalg.det(R) < 0:
        R[1] = -R[1]
    return R


def center_verts(verts: np.ndarray) -> np.ndarray:
    """Center so bbox center is at origin (puts symmetry plane near Y=0)."""
    lo, hi = verts.min(axis=0), verts.max(axis=0)
    return verts - (lo + hi) / 2.0


def fit_plane(points):
    """Least-squares plane through >= 3 points.

    Returns (unit normal, centroid, rms distance of the points from it).
    """
    P = np.asarray(points, float)
    if len(P) < 3:
        raise ValueError("need at least 3 points to fit a plane")
    c = P.mean(axis=0)
    _, _, vt = np.linalg.svd(P - c)
    n = vt[-1]
    return n, c, float(np.sqrt(np.mean(((P - c) @ n) ** 2)))


def level_rotation(points, axis=1):
    """Smallest rotation that squares a picked face up to the world axes.

    PCA gets the orientation close, but a fraction of a degree of residual
    tilt is enough to make one end of a flat side read thicker than the
    other. Pick points on a face that really is flat and this rotates the
    scan so that face becomes perpendicular to `axis` (default Y).

    Returns (R, tilt_deg, rms_mm) — rms is how coplanar the picks actually
    were, i.e. whether they were a fair sample of one flat face.
    """
    n, _, rms = fit_plane(points)
    target = np.zeros(3)
    target[axis] = 1.0
    if n @ target < 0:
        n = -n
    v = np.cross(n, target)
    s = float(np.linalg.norm(v))
    if s < 1e-12:
        return np.eye(3), 0.0, rms
    k = v / s
    K = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    th = float(np.arctan2(s, float(n @ target)))
    R = np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K)
    return R, float(np.degrees(th)), rms


# ============================================================= silhouette

class Silhouette:
    """Raster + polygon of the side-view outer silhouette."""

    def __init__(self, mask, x0, z0, px, polygon):
        self.mask = mask          # uint8 HxW, rows = Z (row 0 = min Z), cols = X
        self.x0 = x0              # world X of column 0 center
        self.z0 = z0              # world Z of row 0 center
        self.px = px              # mm per pixel
        self.polygon = polygon    # (N,2) world XZ, closed implied

    def world_extent(self):
        h, w = self.mask.shape
        return (self.x0 - self.px / 2, self.x0 + (w - 0.5) * self.px,
                self.z0 - self.px / 2, self.z0 + (h - 0.5) * self.px)


def _fill_union(img, polys, value=255, convex=False):
    """Fill the UNION of `polys` — one cv2 call each, deliberately.

    cv2.fillPoly applies the even-odd rule ACROSS every contour handed to it
    in a single call, so overlapping polygons cancel instead of merging. Given
    a whole mesh's triangles at once that erases every pixel covered an even
    number of times — which is most of them, since a closed surface projects
    front-face-plus-back-face onto the same pixel. A plain cube came out with
    nothing but its diagonals, and on a real scan the silhouette lost whole
    limbs. Do not "optimise" this back into one call.
    """
    fill = cv2.fillConvexPoly if convex else None
    for p in polys:
        if fill is not None:
            fill(img, p, value)
        else:
            cv2.fillPoly(img, [p], value)


def extract_silhouette(verts, faces, px=0.15, close_mm=1.5,
                       simplify_mm=0.3, min_area_mm2=25.0) -> Silhouette:
    """Project mesh to the XZ plane, keep only the outer contour.

    Interior holes (magwell windows, pin holes) vanish automatically because
    only the outermost contour is retained (cv2.RETR_EXTERNAL).
    """
    x, z = verts[:, 0], verts[:, 2]
    pad = 3.0
    x0, x1 = x.min() - pad, x.max() + pad
    z0, z1 = z.min() - pad, z.max() + pad
    w = int(np.ceil((x1 - x0) / px)) + 1
    h = int(np.ceil((z1 - z0) / px)) + 1
    img = np.zeros((h, w), np.uint8)

    if len(faces) > 0:
        tv = verts[faces]                                      # (M,3,3)
        pts = np.empty((len(faces), 3, 2), np.float64)
        pts[:, :, 0] = (tv[:, :, 0] - x0) / px                 # col = X
        pts[:, :, 1] = (tv[:, :, 2] - z0) / px                 # row = Z
        _fill_union(img, np.round(pts).astype(np.int32), convex=True)
    else:  # point cloud fallback
        ci = np.clip(np.round((x - x0) / px).astype(int), 0, w - 1)
        ri = np.clip(np.round((z - z0) / px).astype(int), 0, h - 1)
        img[ri, ci] = 255
        k = max(3, int(round(2.0 / px)) | 1)
        img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))

    # close small gaps / scanner speckle
    k = max(3, int(round(close_mm / px)) | 1)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)
    img = cv2.morphologyEx(img, cv2.MORPH_OPEN,
                           cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)))

    contours, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        raise ValueError("No silhouette found — is the mesh empty?")
    contours = [c for c in contours if cv2.contourArea(c) * px * px >= min_area_mm2]
    if not contours:
        raise ValueError("Silhouette too small — check units (mm expected)")
    outer = max(contours, key=cv2.contourArea)

    eps_px = max(1.0, simplify_mm / px)
    approx = cv2.approxPolyDP(outer, eps_px, True).reshape(-1, 2).astype(np.float64)

    mask = np.zeros_like(img)
    cv2.fillPoly(mask, [np.round(approx).astype(np.int32)], 255)

    poly_world = np.empty_like(approx)
    poly_world[:, 0] = x0 + approx[:, 0] * px
    poly_world[:, 1] = z0 + approx[:, 1] * px
    return Silhouette(mask, x0, z0, px, poly_world)


def export_outline_dxf(sil: Silhouette, path: str):
    save_dxf_polyline(path, sil.polygon, closed=True)


# =========================================================== surface maps

SIDES = ("left", "right")


class ThicknessMaps:
    """Where the scan's left and right faces sit, per side-view pixel.

    hL and hR are distances from the Y=0 mid-plane, positive outward, so the
    scan occupies -hL <= y <= +hR and its total thickness is hL + hR.
    NaN where the scan gave no measurement or the pixel is outside the
    silhouette.  Rows = Z (row 0 = min Z), cols = X.
    """

    def __init__(self, hL, hR, sil_mask, x0, z0, px, sil=None):
        self.hL = hL
        self.hR = hR
        self.sil_mask = sil_mask     # silhouette at this resolution, 0/255
        self.sil = sil               # ... and at its own, for exact clipping
        self.x0, self.z0, self.px = x0, z0, px

    @property
    def total(self):
        return self.hL + self.hR

    def face_map(self, side):
        """The map a tier on this side is segmented against."""
        if side == "left":
            return self.hL
        if side == "right":
            return self.hR
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")

    def world_extent(self):
        h, w = self.hL.shape
        return (self.x0 - self.px / 2, self.x0 + (w - 0.5) * self.px,
                self.z0 - self.px / 2, self.z0 + (h - 0.5) * self.px)

    def rc(self, x, z):
        """World XZ -> (row, col), clipped to the map."""
        h, w = self.hL.shape
        c = int(np.clip(round((x - self.x0) / self.px), 0, w - 1))
        r = int(np.clip(round((z - self.z0) / self.px), 0, h - 1))
        return r, c


def _minmax_by_cell(lin, vals, n):
    """Per-cell min and max of vals grouped by lin (inf/-inf where empty)."""
    lo = np.full(n, np.inf)
    hi = np.full(n, -np.inf)
    order = np.argsort(lin, kind="stable")
    ls, vs = lin[order], vals[order]
    starts = np.flatnonzero(np.r_[True, ls[1:] != ls[:-1]])
    keys = ls[starts]
    lo[keys] = np.minimum.reduceat(vs, starts)
    hi[keys] = np.maximum.reduceat(vs, starts)
    return lo, hi


def _fill_gaps(m, sil_small, k=5):
    """Fill scanner dropouts by grey-closing; keep NaN outside the outline."""
    nan = ~np.isfinite(m)
    filled = np.where(nan, 0.0, m).astype(np.float32)
    closed = cv2.morphologyEx(filled, cv2.MORPH_CLOSE, np.ones((k, k), np.uint8))
    out = np.where(nan, closed, m)
    out[nan & (closed <= 0)] = np.nan        # nothing nearby to borrow from
    # A hole in the outer face lets an inner one (a magwell wall, say) show
    # through as a pit. A 3x3 grey close pulls those back up to their
    # neighbours without moving real edges.
    ok = np.isfinite(out)
    lo = np.nanmin(out) if ok.any() else 0.0
    dense = np.where(ok, out, lo).astype(np.float32)
    out = np.where(ok, cv2.morphologyEx(dense, cv2.MORPH_CLOSE,
                                        np.ones((3, 3), np.uint8)), np.nan)
    out[sil_small == 0] = np.nan
    return out


def _surface_samples(verts, faces, px, seed=0, max_per_tri=4096):
    """Points scattered over the triangles, ~2 per pixel of projected area.

    Sampling the surface rather than just the vertices keeps the maps free of
    holes when the mesh is tessellated coarser than the map resolution — a
    hole there splits a tier's outline in two.
    """
    if faces is None or len(faces) == 0:
        return verts
    tv = verts[faces]                                    # (M,3,3)
    a = tv[:, 1][:, [0, 2]] - tv[:, 0][:, [0, 2]]        # XZ edge vectors
    b = tv[:, 2][:, [0, 2]] - tv[:, 0][:, [0, 2]]
    area_px = 0.5 * np.abs(a[:, 0] * b[:, 1] - a[:, 1] * b[:, 0]) / (px * px)
    n = np.clip(np.ceil(2 * area_px), 1, max_per_tri).astype(np.int64)
    idx = np.repeat(np.arange(len(faces)), n)
    rng = np.random.default_rng(seed)                    # deterministic
    r1, r2 = rng.random(len(idx)), rng.random(len(idx))
    s = np.sqrt(r1)
    w = np.stack([1 - s, s * (1 - r2), s * r2], axis=1)  # barycentric
    pts = np.einsum("ij,ijk->ik", w, tv[idx])
    return np.vstack([verts, pts])


def measure_maps(verts, faces, sil: Silhouette = None, px=0.5) -> ThicknessMaps:
    """Measure the scan's left/right face positions over the side view.

    With no silhouette (the Level step, which runs before one exists) the
    extent comes from the scan's own bounding box and nothing is masked out.
    """
    pts = _surface_samples(verts, faces, px)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    if sil is not None:
        ex = sil.world_extent()
    else:
        pad = 3.0
        ex = (x.min() - pad, x.max() + pad, z.min() - pad, z.max() + pad)
    x0, z0 = ex[0], ex[2]
    w = int(np.ceil((ex[1] - x0) / px)) + 1
    h = int(np.ceil((ex[3] - z0) / px)) + 1
    ci = np.clip(((x - x0) / px).round().astype(int), 0, w - 1)
    ri = np.clip(((z - z0) / px).round().astype(int), 0, h - 1)
    ymin, ymax = _minmax_by_cell(ri * w + ci, y, h * w)

    hL = (-ymin).reshape(h, w)               # positive outward, both sides
    hR = ymax.reshape(h, w)
    sil_small = (cv2.resize(sil.mask, (w, h), interpolation=cv2.INTER_NEAREST)
                 if sil is not None else np.full((h, w), 255, np.uint8))
    return ThicknessMaps(_fill_gaps(hL, sil_small), _fill_gaps(hR, sil_small),
                         sil_small, x0, z0, px, sil)


def sample_maps(maps: ThicknessMaps, x, z, r_mm=1.0):
    """Robust (median) sample of the maps around a world XZ point.

    Returns {"hL","hR","total"} in mm, or None if there is no data there.
    """
    r, c = maps.rc(x, z)
    k = max(0, int(round(r_mm / maps.px)))
    h, w = maps.hL.shape
    sl = (slice(max(0, r - k), min(h, r + k + 1)),
          slice(max(0, c - k), min(w, c + k + 1)))
    out = {}
    for name, m in (("hL", maps.hL), ("hR", maps.hR)):
        vals = m[sl]
        vals = vals[np.isfinite(vals)]
        if vals.size == 0:
            return None
        out[name] = float(np.median(vals))
    out["total"] = out["hL"] + out["hR"]
    return out


def median_thickness(maps: ThicknessMaps):
    """Median total thickness over the whole outline (base-thickness guess)."""
    vals = maps.total
    vals = vals[np.isfinite(vals)]
    return float(np.median(vals)) if vals.size else None


def base_from_maps(maps: ThicknessMaps):
    """Whole-outline guess at (base thickness, base mid-plane offset)."""
    tot, off = maps.total, (maps.hR - maps.hL) / 2.0
    tot, off = tot[np.isfinite(tot)], off[np.isfinite(off)]
    if not tot.size:
        return None, 0.0
    return float(np.median(tot)), float(np.median(off))


def base_faces(base_thickness, base_y0=0.0):
    """(yR, yL) of the base slab: base_y0 ± base_thickness/2."""
    return base_y0 + base_thickness / 2.0, base_y0 - base_thickness / 2.0


def _snap_to_polygon(poly, ref, tol):
    """Pull vertices lying within tol of `ref`'s boundary exactly onto it.

    Where a tier runs out to the edge of the frame, its outline and the base
    outline must be the same line — a tenth of a millimetre of disagreement
    shows up as a step in the built solid.
    """
    A = np.asarray(ref, float)
    B = np.roll(A, -1, axis=0)
    d = B - A
    L2 = np.einsum("ij,ij->i", d, d)
    L2 = np.where(L2 > 0, L2, 1.0)
    out = np.array(poly, float)
    for i, p in enumerate(out):
        t = np.clip(np.einsum("ij,ij->i", p - A, d) / L2, 0.0, 1.0)
        proj = A + t[:, None] * d
        off = p - proj
        k = int(np.argmin(np.einsum("ij,ij->i", off, off)))
        if np.hypot(*off[k]) <= tol:
            out[i] = proj[k]
    return out


def _clip_to_silhouette(polys, sil: Silhouette, simplify_mm=0.2,
                        min_area_mm2=2.0, snap_tol=0.35):
    """Trim tier outlines to the frame profile, at the profile's resolution.

    Doing this on the coarse thickness-map grid left every tier a fraction
    of a millimetre short of the outline — a visible ledge that no amount of
    grow_mm could close, because growing then clipped back to the same
    inset boundary.
    """
    if sil is None or not polys:
        return polys
    img = np.zeros(sil.mask.shape, np.uint8)
    conts = []
    for p in polys:
        p = np.asarray(p, float)
        q = np.empty_like(p)
        q[:, 0] = (p[:, 0] - sil.x0) / sil.px        # col = x
        q[:, 1] = (p[:, 1] - sil.z0) / sil.px        # row = z
        conts.append(np.round(q).astype(np.int32))
    _fill_union(img, conts)
    img = cv2.bitwise_and(img, sil.mask)

    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        if float(cv2.contourArea(c)) * sil.px ** 2 < min_area_mm2:
            continue
        a = cv2.approxPolyDP(c, max(1.0, simplify_mm / sil.px), True)
        a = a.reshape(-1, 2).astype(np.float64)
        if len(a) < 3:
            continue
        world = np.stack([sil.x0 + a[:, 0] * sil.px,
                          sil.z0 + a[:, 1] * sil.px], axis=1)
        out.append(_snap_to_polygon(world, sil.polygon, snap_tol))
    return out


def region_kind(region):
    """'tier' (click-picked thickness tier) or 'rect' (legacy rectangle)."""
    if region.get("kind") == "rect" or ("width" in region and "x0" in region):
        return "rect"
    return "tier"


def base_offset(side, base_thickness, base_y0=0.0):
    """How far the base's face on `side` stands off the mid-plane."""
    half = float(base_thickness) / 2.0
    return half + base_y0 if side == "right" else half - base_y0


def tier_offset(tier, base_thickness, base_y0=0.0):
    """How far this tier's face stands off the mid-plane (base + add_mm)."""
    return (base_offset(tier["side"], base_thickness, base_y0)
            + float(tier.get("add_mm", 0.0)))


def _prev_offset(tiers, tier, base_thickness, base_y0=0.0):
    """The offset of the tier immediately below this one, on its own side.

    That is the threshold this tier segments against: everything standing
    proud of the tier below gets pulled into this tier's outline.
    """
    side = tier["side"]
    fo = tier_offset(tier, base_thickness, base_y0)
    prev = base_offset(side, base_thickness, base_y0)
    for u in tiers:
        if u is tier or region_kind(u) != "tier" or u.get("side") != side:
            continue
        f = tier_offset(u, base_thickness, base_y0)
        if prev < f < fo:
            prev = f
    return prev


def make_tier(maps: ThicknessMaps, x, z, side, base_thickness, base_y0=0.0,
              over_mm=0.3, grow_mm=0.0, add_mm=None, tiers=()):
    """Build a thickness tier from a click at world XZ.

    add_mm — how far this tier stands proud of the base, on its own side.
    Defaults to the scan's own value under the click, and may be overridden
    (calipers beat the scanner) without changing which pixels it covers: the
    footprint always comes from the tier below it, not from this number.

    Returns the tier dict, or None if the scan has no data at that point.
    """
    if side not in SIDES:
        raise ValueError(f"side must be one of {SIDES}, got {side!r}")
    s = sample_maps(maps, x, z)
    if s is None:
        return None
    if add_mm is None:
        here = s["hR"] if side == "right" else s["hL"]
        add_mm = here - base_offset(side, base_thickness, base_y0)
    t = {"kind": "tier", "side": side,
         "x": round(float(x), 2), "z": round(float(z), 2),
         "add_mm": round(float(add_mm), 2),
         "over_mm": float(over_mm), "grow_mm": float(grow_mm),
         "measured": {k: round(v, 2) for k, v in s.items()}}
    segment_tier(maps, list(tiers) + [t], t, base_thickness, base_y0)
    return t


def segment_tier(maps: ThicknessMaps, tiers, tier, base_thickness,
                 base_y0=0.0, close_mm=1.0, min_area_mm2=2.0,
                 simplify_mm=0.3):
    """(Re)compute a tier's outline from the scan; sets tier["polys"].

    Everything standing proud of the tier below — by more than over_mm, to
    ride out scanner noise — is roped in, wherever it is on the frame, then
    interior holes are filled (outer contours only, the same trick the
    silhouette uses) and the outline is grown by grow_mm.

    Returns the total footprint area in mm² (0.0 if nothing matched).
    """
    tier["polys"] = []
    tier["area_mm2"] = 0.0
    if region_kind(tier) != "tier":
        return None
    px = maps.px
    m = maps.face_map(tier["side"])
    thr = (_prev_offset(tiers, tier, base_thickness, base_y0)
           + float(tier.get("over_mm", 0.3)))

    img = np.zeros(m.shape, np.uint8)
    img[np.isfinite(m) & (m > thr)] = 255
    k = max(3, int(round(close_mm / px)) | 1)
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    # Opening kills speckle, but along the silhouette edge the map is only
    # partly covered by the frame, so opening there would nibble a sliver off
    # the outline and leave that rim of the solid under-thick. Keep the raw
    # threshold on that rim.
    rim = cv2.bitwise_and(maps.sil_mask, cv2.bitwise_not(
        cv2.erode(maps.sil_mask, np.ones((5, 5), np.uint8))))
    img = cv2.bitwise_or(cv2.morphologyEx(img, cv2.MORPH_OPEN, kern),
                         cv2.bitwise_and(img, rim))
    img = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kern)     # bridge dropouts

    grow = float(tier.get("grow_mm", 0.0))
    if grow > 0:
        kk = max(3, int(round(2 * grow / px)) | 1)
        img = cv2.dilate(img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE,
                                                        (kk, kk)))
        if maps.sil is None:                 # no profile to clip against
            img = cv2.bitwise_and(img, maps.sil_mask)
    elif maps.sil is not None:
        # The maps stop at the silhouette rendered on this coarse grid, which
        # sits inside the real profile. Let the mask spill over that edge —
        # but ONLY outside it, so interior tier boundaries do not fatten —
        # and let the clip below put it back on the real profile.
        spill = cv2.bitwise_and(
            cv2.dilate(img, np.ones((5, 5), np.uint8)),
            cv2.bitwise_not(maps.sil_mask))
        img = cv2.bitwise_or(img, spill)

    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polys = []
    for c in cnts:
        if float(cv2.contourArea(c)) * px * px < min_area_mm2:
            continue
        approx = cv2.approxPolyDP(c, max(1.0, simplify_mm / px), True)
        approx = approx.reshape(-1, 2).astype(np.float64)
        if len(approx) < 3:
            continue
        polys.append(np.stack([maps.x0 + approx[:, 0] * px,
                               maps.z0 + approx[:, 1] * px], axis=1))
    # a grown outline may now stick out past the frame; trim it there, at
    # the profile's own resolution so the two share an edge exactly
    polys = _clip_to_silhouette(polys, maps.sil, min_area_mm2=min_area_mm2)

    total = 0.0
    for p in polys:
        x, y = np.asarray(p).T
        total += abs(float(np.dot(x, np.roll(y, -1)) -
                           np.dot(y, np.roll(x, -1)))) / 2.0
    tier["polys"] = [[[round(a, 3), round(b, 3)] for a, b in p] for p in polys]
    tier["area_mm2"] = round(total, 1)
    tier["threshold_mm"] = round(thr, 3)
    return total


def segment_all(maps: ThicknessMaps, regions, base_thickness, base_y0=0.0):
    """Re-segment every tier — thresholds are relative, so they all move."""
    for t in regions:
        if region_kind(t) == "tier":
            segment_tier(maps, regions, t, base_thickness, base_y0)


def sort_tiers(regions):
    """Tiers ordered by side then thickness (the order they stack in)."""
    return sorted(regions, key=lambda r: (
        0 if region_kind(r) == "rect" else 1,
        r.get("side", ""), float(r.get("add_mm", 0.0))))


def _raster_polys_map(polys, maps: ThicknessMaps):
    """Rasterize world-XZ polygons in map orientation (rows = Z, cols = X)."""
    img = np.zeros(maps.hL.shape, np.uint8)
    conts = []
    for p in polys:
        p = np.asarray(p, np.float64)
        if len(p) < 3:
            continue
        q = np.empty_like(p)
        q[:, 0] = (p[:, 0] - maps.x0) / maps.px      # col = x
        q[:, 1] = (p[:, 1] - maps.z0) / maps.px      # row = z
        conts.append(np.round(q).astype(np.int32))
    _fill_union(img, conts)
    return img > 0


def coverage_report(maps: ThicknessMaps, regions, base_thickness, base_y0=0.0,
                    tol_mm=0.25):
    """How much of the real frame the tier model still leaves under-thick.

    Tiers round thickness UP, so anything left short is a place where the
    subtraction solid is thinner than the scan — i.e. where the transplanted
    grip would foul the frame. Returns per side:
        {"short_mm": worst shortfall, "at": (x, z),
         "area_mm2": area short by more than tol_mm}
    tol_mm exists because a scan is noisy: the maps take the outermost sample
    per pixel, so a flat face reads a tenth of a millimetre proud of itself.
    """
    out = {}
    for side in ("left", "right"):
        m = maps.face_map(side)
        built = np.full(m.shape, base_offset(side, base_thickness, base_y0),
                        np.float32)
        for t in regions:
            if region_kind(t) != "tier" or t.get("side") != side:
                continue
            if not t.get("polys"):
                continue
            mask = _raster_polys_map(t["polys"], maps)
            fo = tier_offset(t, base_thickness, base_y0)
            np.maximum(built, np.where(mask, fo, -np.inf).astype(np.float32),
                       out=built)
        # a tier's outline has to cut somewhere inside the boundary pixel, so
        # forgive a one-pixel transition band; real gaps are far wider
        built = cv2.dilate(built, np.ones((3, 3), np.uint8))
        short = np.where(np.isfinite(m), m - built, -np.inf)
        i = int(np.argmax(short))
        r, c = np.unravel_index(i, short.shape)
        out[side] = {"short_mm": round(float(short[r, c]), 2),
                     "at": (round(float(maps.x0 + c * maps.px), 1),
                            round(float(maps.z0 + r * maps.px), 1)),
                     "area_mm2": round(float((short > tol_mm).sum())
                                       * maps.px ** 2, 1),
                     "tol_mm": tol_mm}
    return out


# ---- legacy helpers (rectangle workflow; kept for old projects/scripts)

def width_map(verts, faces, sil: Silhouette, px=0.6):
    """Total measured width per side-view pixel. Returns (wmap, x0, z0, px)."""
    maps = measure_maps(verts, faces, sil, px=px)
    return maps.total, maps.x0, maps.z0, maps.px


def suggest_width(wmap, x0, z0, px, rect=None, q=95):
    """Suggested plateau width for a world-coords rect (x0,z0,x1,z1)."""
    m = wmap
    if rect is not None:
        rx0, rz0, rx1, rz1 = rect
        c0 = max(0, int((min(rx0, rx1) - x0) / px))
        c1 = min(m.shape[1], int((max(rx0, rx1) - x0) / px) + 1)
        r0 = max(0, int((min(rz0, rz1) - z0) / px))
        r1 = min(m.shape[0], int((max(rz0, rz1) - z0) / px) + 1)
        m = m[r0:r1, c0:c1]
    vals = m[np.isfinite(m)]
    if vals.size == 0:
        return None
    return float(np.percentile(vals, q))


# ============================================================ CAD export
#
# The tier model IS a sketch-and-extrude model: a handful of closed outlines,
# each extruded a known distance along Y. That exports losslessly to DXF.

def _grow_loops(loops, mm, simplify_mm=0.1):
    """Minkowski-grow closed loops by mm (rasterised; mm <= 0 is a no-op)."""
    loops = [np.asarray(l, float) for l in loops if len(np.asarray(l)) >= 3]
    if mm <= 0 or not loops:
        return loops
    px = float(np.clip(mm / 4.0, 0.02, 0.1))
    pts = np.vstack(loops)
    # snap the raster to a shared lattice: the base outline and a tier that
    # runs out to it must come back from this with the SAME offset edge, and
    # they only do if they were rasterised on the same grid
    x0 = np.floor((pts[:, 0].min() - mm - 5 * px) / px) * px
    z0 = np.floor((pts[:, 1].min() - mm - 5 * px) / px) * px
    x1, z1 = pts[:, 0].max() + mm + 5 * px, pts[:, 1].max() + mm + 5 * px
    w = int(np.ceil((x1 - x0) / px)) + 1
    h = int(np.ceil((z1 - z0) / px)) + 1
    img = np.zeros((h, w), np.uint8)
    _fill_union(img, [np.round(np.stack([(l[:, 0] - x0) / px,
                                         (l[:, 1] - z0) / px], 1)
                               ).astype(np.int32) for l in loops])
    k = max(3, int(round(2 * mm / px)) | 1)
    img = cv2.dilate(img, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k)))
    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        a = cv2.approxPolyDP(c, max(1.0, simplify_mm / px), True)
        a = a.reshape(-1, 2).astype(np.float64)
        if len(a) >= 3:
            out.append(np.stack([x0 + a[:, 0] * px, z0 + a[:, 1] * px], 1))
    return out


def _box_loop(e, pad=0.0):
    """The four corners of a box extra, in world XZ."""
    cx, cz = (e["x0"] + e["x1"]) / 2, (e["z0"] + e["z1"]) / 2
    hx = abs(e["x1"] - e["x0"]) / 2 + pad
    hz = abs(e["z1"] - e["z0"]) / 2 + pad
    a = np.deg2rad(e.get("tilt_deg", 0.0))
    c, s = np.cos(a), np.sin(a)
    p = np.array([[-hx, -hz], [hx, -hz], [hx, hz], [-hx, hz]])
    return p @ np.array([[c, -s], [s, c]]).T + [cx, cz]


def sketch_model(sil: Silhouette, base_thickness, regions, extras,
                 base_y0=0.0, clearance=0.0):
    """The solid as a list of sketches with extrusion ranges.

    Each entry: {"key", "kind", "label", "loops" (world XZ), "y_lo", "y_hi"}.
    Every tier extrudes from the same plane - y = base_y0 - so in CAD they
    all share one workplane and differ only in depth. The base straddles it.
    `clearance` (mm per side) is grown into the outlines and the depths, so
    what comes out is the finished subtraction solid.
    """
    c = float(clearance)
    T = float(base_thickness)
    yR_b, yL_b = base_faces(T, base_y0)
    model = [{"key": "00_base", "kind": "base",
              "label": f"base tier - whole silhouette, {T:.2f} mm thick",
              "loops": _grow_loops([np.asarray(sil.polygon, float)], c),
              "y_lo": yL_b - c, "y_hi": yR_b + c}]

    n = {"left": 0, "right": 0}
    for t in sort_tiers(regions):
        if region_kind(t) == "rect":
            w = float(t["width"])
            loops = _grow_loops([np.array([
                [t["x0"], t["z0"]], [t["x1"], t["z0"]],
                [t["x1"], t["z1"]], [t["x0"], t["z1"]]], float)], c)
            model.append({"key": f"rect{len(model):02d}", "kind": "rect",
                          "label": f"legacy rectangle, {w:.2f} mm thick",
                          "loops": loops, "y_lo": base_y0 - w / 2 - c,
                          "y_hi": base_y0 + w / 2 + c})
            continue
        if not t.get("polys"):
            continue
        side = t["side"]
        n[side] += 1
        fo = tier_offset(t, T, base_y0)
        loops = _grow_loops([np.asarray(p, float) for p in t["polys"]], c)
        if side == "right":
            y_lo, y_hi = base_y0, fo + c
        else:
            y_lo, y_hi = -fo - c, base_y0
        model.append({
            "key": (f"{side[0].upper()}{n[side]}_plus"
                    f"{t['add_mm']:.2f}mm".replace(".", "_")),
            "kind": "tier", "side": side,
            "label": (f"{side} tier {n[side]} - {t['add_mm']:+.2f} mm proud "
                      f"of base, face at y={fo if side == 'right' else -fo:+.2f}"),
            "loops": loops, "y_lo": y_lo, "y_hi": y_hi})

    for i, e in enumerate(extras, 1):
        yc = e.get("yc", 0.0)
        if e["kind"] == "box":
            hw = e["ywidth"] / 2 + c
            model.append({"key": f"X{i}_box", "kind": "extra",
                          "label": f"extra {i} - box, tilt "
                                   f"{e.get('tilt_deg', 0.0):g} deg",
                          "loops": [_box_loop(e, c)],
                          "y_lo": yc - hw, "y_hi": yc + hw})
        elif e["kind"] == "cyl_y":
            hw = e["ylen"] / 2 + c
            model.append({"key": f"X{i}_cyl", "kind": "extra",
                          "label": f"extra {i} - cylinder dia {e['dia']:g} mm",
                          "circle": (e["x"], e["z"], e["dia"] / 2 + c),
                          "loops": [], "y_lo": yc - hw, "y_hi": yc + hw})
    return model


def _sketch_entities(s, layer=None):
    ents = [dxf_polyline(loop, True, layer or s["key"]) for loop in s["loops"]]
    if "circle" in s:
        x, z, r = s["circle"]
        ents.append(dxf_circle(x, z, r, layer or s["key"]))
    return ents


def export_cad(folder, model, meta=None):
    """Write the sketch model as DXFs + a build sheet + an OpenSCAD assembly.

    One DXF per sketch (import, place on a workplane, extrude to the depth on
    the build sheet), plus all_sketches.dxf with one layer per sketch for a
    single import. Returns the list of files written.
    """
    meta = meta or {}
    os.makedirs(folder, exist_ok=True)
    written = []
    # what a previous export left here, so renamed sketches don't linger as
    # stale DXFs. Only ever removes files this tool wrote itself.
    stamp = os.path.join(folder, ".frame2solid_files")
    previous = []
    if os.path.isfile(stamp):
        with open(stamp, encoding="utf-8") as f:
            previous = [ln.strip() for ln in f if ln.strip()]

    for s in model:
        p = os.path.join(folder, s["key"] + ".dxf")
        save_dxf(p, _sketch_entities(s, layer="0"))
        written.append(p)

    p = os.path.join(folder, "all_sketches.dxf")
    save_dxf(p, [e for s in model for e in _sketch_entities(s)])
    written.append(p)

    # ---- build sheet
    L = ["frame2solid - CAD export", "=" * 60, ""]
    if meta.get("scan"):
        L.append(f"scan          : {meta['scan']}")
    L += [f"clearance     : {meta.get('clearance', 0.0):g} mm per side "
          f"(already grown into these outlines and depths)",
          f"base mid-plane: y0 = {meta.get('base_y0', 0.0):+.3f} mm",
          "",
          "The DXFs use only LINE and CIRCLE entities (R12). Closed outlines",
          "are rings of line segments: OpenSCAD refuses POLYLINE and would",
          "need an R13+ file for LWPOLYLINE, and SolveSpace splits polylines",
          "into segments on import anyway.",
          "",
          "Units are mm. The sketches lie in the frame's XZ plane:",
          "    DXF x = frame X (along the bore)",
          "    DXF y = frame Z (vertical)",
          "and extrude along frame Y (across the frame, + = right).",
          "",
          "Every tier extrudes from the SAME plane, y = y0, so in CAD they",
          "share one workplane and differ only in depth and direction.",
          "The base straddles it. Union everything.",
          "",
          f"{'file':<26} {'from y':>9} {'to y':>9} {'depth':>8}  sketch",
          "-" * 100]
    for s in model:
        L.append(f"{s['key'] + '.dxf':<26} {s['y_lo']:>+9.3f} "
                 f"{s['y_hi']:>+9.3f} {s['y_hi'] - s['y_lo']:>8.3f}  "
                 f"{s['label']}")
    L += ["", "SolveSpace:",
          "  1. New sketch in a workplane on the XZ plane (Y = 0 normal).",
          "  2. File > Import... the .dxf you want (or all_sketches.dxf for",
          "     the lot, then delete what you don't need).",
          "  3. Tweak the outline - that's the point of exporting it.",
          "  4. New Group > Extrude, set the depth from the table, direction",
          "     + for right-side tiers and - for left-side ones, and offset",
          "     the group's workplane to y0 if it isn't there already.",
          "  5. Union the groups; export the result as STL for the boolean.",
          "",
          "OpenSCAD: open assembly.scad - it already does all of the above.",
          ""]
    p = os.path.join(folder, "build.txt")
    with open(p, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    written.append(p)

    # ---- OpenSCAD assembly: the model rebuilt exactly, and the
    #      quickest route to an STL if you want one
    S = ["// frame2solid - subtraction solid, rebuilt from the exported",
         "// sketches. Render this to get an STL.",
         "// X = bore, Y = across the frame, Z = vertical. Units mm.",
         "$fn = 64;", "",
         "module sketch(file, y_lo, y_hi) {",
         "    translate([0, y_hi, 0]) rotate([90, 0, 0])",
         "        linear_extrude(height = y_hi - y_lo) import(file);",
         "}", "",
         "module frame_solid() {", "    union() {"]
    for s in model:
        S.append(f'        sketch("{s["key"]}.dxf", {s["y_lo"]:.4f}, '
                 f'{s["y_hi"]:.4f});   // {s["label"]}')
    S += ["    }", "}", "",
          "frame_solid();", "",
          "// The grip transplant itself - point it at your donor grip:",
          "// difference() {",
          '//     import("donor_grip.stl", convexity = 10);',
          "//     frame_solid();", "// }", ""]
    p = os.path.join(folder, "assembly.scad")
    with open(p, "w", encoding="utf-8") as f:
        f.write("\n".join(S))
    written.append(p)

    names = [os.path.basename(p) for p in written]
    for old in previous:
        if old not in names:
            try:
                os.remove(os.path.join(folder, old))
            except OSError:
                pass
    with open(stamp, "w", encoding="utf-8") as f:
        f.write("\n".join(names) + "\n")
    return written
