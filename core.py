"""frame2solid core pipeline.

Coordinate convention (after the Orient step):
    X = along the bore / frame length
    Y = across the frame (left-right); Y = 0 is the frame's mid-plane
    Z = vertical (grip height)
The "side view" is the XZ plane, looking along +Y. All units are mm.

Pipeline:
    scan mesh -> orient -> side-view silhouette (outer contour only, windows
    filled) -> two-sided surface maps (where the scan's left and right faces
    sit, per side-view pixel) -> thickness tiers -> sketch_model() +
    export_cad() -> one DXF per sketch plus the extrusion depths, which is
    the model exactly.

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


def _tess_mm(verts, faces):
    """The scan's own triangle scale (median edge length), in mm.

    A dropped face leaves a hole this big, so it is the right yardstick for
    how far _fill_gaps has to reach — a fixed millimetre guess is either too
    small for a coarse scan or needlessly destructive on a fine one.
    """
    if faces is None or len(faces) == 0:
        return 0.0
    tv = verts[faces]
    e = np.linalg.norm(tv[:, [1, 2, 0]] - tv, axis=2)
    return float(np.median(e))


def _grey_close_masked(m, ok, kern):
    """Grey close that IGNORES invalid pixels instead of reading them as data.

    Returns (closed, reached), reached being where the kernel found any valid
    pixel at all.

    The masking is the whole point. Filling the invalid pixels with the map's
    minimum and closing over the lot — which is what this used to do — lets
    the erosion half of the close drag that minimum a full kernel radius INTO
    valid data. Every dropout and the entire silhouette rim came back short,
    and enlarging the kernel to match the scan's tessellation made it worse
    rather than better: the synthetic's diagonal rib lost 8 mm off the end
    that runs out to the frame's rear edge. Dilate with the invalid pixels at
    -inf so they cannot raise anything, then erode with whatever the dilation
    did not reach at +inf so it cannot lower anything.
    """
    NEG, POS = -1e9, 1e9
    d = cv2.dilate(np.where(ok, m, NEG).astype(np.float32), kern)
    reached = d > NEG / 2
    e = cv2.erode(np.where(reached, d, POS).astype(np.float32), kern)
    return e, e < POS / 2


def _fill_gaps(m, sil_small, px, gap_mm=2.5, pit_mm=1.0, tess_mm=0.0):
    """Fill scanner dropouts by grey-closing; keep NaN outside the outline.

    The kernels are sized in MILLIMETRES, not pixels. They used to be fixed
    pixel counts, which quietly made this whole function resolution
    dependent: it bridged 2.5 mm of dropout at px=0.5 but only 0.75 mm at
    px=0.15, so re-measuring a scan more finely LOST coverage (81% -> 61% of
    the profile on a real scan) and every tier shrank. Anything specified in
    pixels here has to be re-derived from px.

    They also scale with the scan's TESSELLATION. A dropped triangle punches a
    hole one triangle wide, which a coarse raster cannot resolve but a fine
    one can: the synthetic's 2% dropped faces left 16% of the diagonal rib's
    top face reading the wall behind it at px=0.1 (all of it survived at 0.5),
    and the rib came out 20% under area. Reach has to follow the mesh, not the
    raster.

    ELLIPSE kernels, never square. A square structuring element quantises a
    diagonal edge to its own size, and since these kernels are millimetres
    across that put a ~2.5 mm staircase on tier boundaries which no raster
    resolution could remove (user-reported, via OpenSCAD). An ellipse rounds
    concave corners by its radius and leaves everything else where it was.
    """
    reach = max(1.0, 2.0 * tess_mm)
    kern = lambda mm: cv2.getStructuringElement(
        cv2.MORPH_ELLIPSE, ((max(3, int(round(mm / px)) | 1),) * 2))
    ok = np.isfinite(m)
    filled, reached = _grey_close_masked(m, ok, kern(max(gap_mm, reach)))
    out = np.where(ok, m, np.where(reached, filled, np.nan))
    # A hole in the outer face lets an inner one (a magwell wall, say) show
    # through as a pit. A grey close pulls those back up to their neighbours
    # without moving real edges.
    ok = np.isfinite(out)
    pit, reached = _grey_close_masked(out, ok, kern(max(pit_mm, reach)))
    out = np.where(ok, np.where(reached, pit, out), np.nan)
    out[sil_small == 0] = np.nan
    return out


def _raster_minmax(verts, faces, px, x0, z0, w, h, chunk=1 << 22):
    """Per-pixel min/max of Y over the triangles, by RASTERISING them.

    Scan-converts every triangle's XZ projection and interpolates Y at each
    pixel centre it covers, exactly the way extract_silhouette rasterises
    with fillPoly. Coverage is therefore EXACT: a pixel has data iff the
    surface actually projects onto its centre, at any px, and a straight
    edge comes out as a one-pixel staircase.

    This replaced scattering ~2 random points per pixel over each triangle.
    Random placement is Poisson, so it left 26% of in-outline pixels empty at
    px=0.5 and 52% at px=0.1 — finer rasters were emptier, not sharper. Those
    holes then got patched by _fill_gaps' grey-close, and since a square
    structuring element quantises a diagonal edge to its own size, every tier
    boundary inherited a ~2.5 mm staircase that no amount of px could fix
    (user-reported, via OpenSCAD; the offending feature was a plain
    rectangular prism). Do not go back to point sampling: the cost here is
    O(covered pixels), the same order, and it is exact.

    Rasterisation is CONSERVATIVE — a pixel takes a triangle that overlaps
    its square at all, not just one covering its centre. Two reasons. It errs
    thick, which is this tool's standing rule. And it keeps the half-pixel
    bookkeeping the tier code was tuned against: the old sampler's per-pixel
    max over random points biased every feature outward by about half a
    pixel, which cancelled findContours' inset. Testing centres alone dropped
    that bias and the synthetic's 4 mm diagonal rib came out 7.7% under area.
    The overlap test restores it deliberately. The test is exact — the
    Minkowski sum of a triangle and the pixel square is the three edge
    half-planes offset by each edge's support, plus the expanded bounds.
    """
    lo = np.full(h * w, np.inf)
    hi = np.full(h * w, -np.inf)
    if faces is None or len(faces) == 0:
        return lo, hi
    tv = verts[faces]                                    # (M,3,3)
    u = (tv[:, :, 0] - x0) / px                          # pixel coordinates;
    v = (tv[:, :, 2] - z0) / px                          # pixel c is centred
    y = tv[:, :, 1]                                      # exactly on u == c
    # Pixels whose SQUARE the triangle can reach, so the bounds grow by the
    # half-pixel the overlap test below allows.
    c0 = np.maximum(np.ceil(u.min(1) - 0.5), 0).astype(np.int64)
    c1 = np.minimum(np.floor(u.max(1) + 0.5), w - 1).astype(np.int64)
    r0 = np.maximum(np.ceil(v.min(1) - 0.5), 0).astype(np.int64)
    r1 = np.minimum(np.floor(v.max(1) + 0.5), h - 1).astype(np.int64)
    e0u, e0v = u[:, 1] - u[:, 0], v[:, 1] - v[:, 0]
    e1u, e1v = u[:, 2] - u[:, 0], v[:, 2] - v[:, 0]
    det = e0u * e1v - e0v * e1u
    sgn = np.where(det >= 0, 1.0, -1.0)
    # Each edge's half-plane relaxed by the pixel square's support along its
    # normal. Distance is edge_fn / |edge| and the support is
    # 0.5*(|edge_u| + |edge_v|) / |edge|, so the |edge| cancels.
    sl0 = 0.5 * (np.abs(e0u) + np.abs(e0v))                  # edge p0->p1
    sl2 = 0.5 * (np.abs(e1u) + np.abs(e1v))                  # edge p0->p2
    sl1 = 0.5 * (np.abs(e1u - e0u) + np.abs(e1v - e0v))      # edge p1->p2
    ylo3, yhi3 = y.min(1), y.max(1)
    bw, bh = c1 - c0 + 1, r1 - r0 + 1
    live = np.flatnonzero((bw > 0) & (bh > 0) & (np.abs(det) > 1e-12))
    if live.size == 0:
        return lo, hi

    idx_parts, y_parts = [], []
    size = np.maximum(bw[live], bh[live])
    # Bucket by bounding-box size so each batch is one rectangular array.
    # Most triangles in a scan land in the smallest bucket.
    order = np.argsort(size, kind="stable")
    ssort = size[order]
    edges = np.searchsorted(ssort, 2 ** np.arange(1, 32), side="left")
    for lo_i, hi_i in zip(np.r_[0, edges], np.r_[edges, live.size]):
        if hi_i <= lo_i:
            continue
        grp = live[order[lo_i:hi_i]]
        # The window is the bucket's LARGEST member. Sizing it by the bucket's
        # lower bound instead truncated every triangle above that bound, which
        # silently dropped the far end of any triangle bigger than a pixel or
        # two — 8 mm off the end of the synthetic's diagonal rib, whose outer
        # face is two 39 mm triangles.
        s = int(ssort[hi_i - 1])
        step = max(1, chunk // (s * s))
        for b in range(0, grp.size, step):
            t = grp[b:b + step]
            du = np.arange(s)
            U = c0[t][:, None, None] + du[None, None, :]        # (n,s,s)
            V = r0[t][:, None, None] + du[None, :, None]
            pu = U - u[t, 0][:, None, None]
            pv = V - v[t, 0][:, None, None]
            d = det[t][:, None, None]
            sg = sgn[t][:, None, None]
            g2 = pu * e1v[t][:, None, None] - pv * e1u[t][:, None, None]
            g0 = e0u[t][:, None, None] * pv - e0v[t][:, None, None] * pu
            g1 = d - g0 - g2
            ok = ((g0 * sg >= -sl0[t][:, None, None])
                  & (g1 * sg >= -sl1[t][:, None, None])
                  & (g2 * sg >= -sl2[t][:, None, None])
                  & (U <= c1[t][:, None, None]) & (V <= r1[t][:, None, None]))
            if not ok.any():
                continue
            # Clamped so a pixel just outside the triangle cannot extrapolate
            # Y beyond the range the triangle actually spans.
            yv = np.clip(y[t, 0][:, None, None]
                         + (g2 / d) * (y[t, 1] - y[t, 0])[:, None, None]
                         + (g0 / d) * (y[t, 2] - y[t, 0])[:, None, None],
                         ylo3[t][:, None, None], yhi3[t][:, None, None])
            idx_parts.append((V * w + U)[ok])
            y_parts.append(yv[ok])
    if not idx_parts:
        return lo, hi
    return _minmax_by_cell(np.concatenate(idx_parts),
                           np.concatenate(y_parts), h * w)


def measure_maps(verts, faces, sil: Silhouette = None, px=0.5) -> ThicknessMaps:
    """Measure the scan's left/right face positions over the side view.

    With no silhouette (the Level step, which runs before one exists) the
    extent comes from the scan's own bounding box and nothing is masked out.
    """
    if sil is not None:
        ex = sil.world_extent()
    else:
        pad = 3.0
        ex = (verts[:, 0].min() - pad, verts[:, 0].max() + pad,
              verts[:, 2].min() - pad, verts[:, 2].max() + pad)
    x0, z0 = ex[0], ex[2]
    w = int(np.ceil((ex[1] - x0) / px)) + 1
    h = int(np.ceil((ex[3] - z0) / px)) + 1
    ymin, ymax = _raster_minmax(verts, faces, px, x0, z0, w, h)

    hL = (-ymin).reshape(h, w)               # positive outward, both sides
    hR = ymax.reshape(h, w)
    sil_small = (cv2.resize(sil.mask, (w, h), interpolation=cv2.INTER_NEAREST)
                 if sil is not None else np.full((h, w), 255, np.uint8))
    tess = _tess_mm(verts, faces)
    return ThicknessMaps(_fill_gaps(hL, sil_small, px, tess_mm=tess),
                         _fill_gaps(hR, sil_small, px, tess_mm=tess),
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


_SIMPLIFY_FLOOR = 0.10        # mm; finer than this just traces the raster
_CIRCLE_REL_TOL = 0.12        # a loop may stray this much of its own radius
                              # and still be called a circle. This is the
                              # SHAPE guard, and it is what stops a small
                              # square boss being read as a disc: a square's
                              # boundary sits 25% of r_equivalent off its own
                              # best-fit circle, a rasterised disc 6-8%.
                              # The other guard is absolute (see fit_circle).
_SIMPLIFY_AREA_TOL = 0.02     # accept the coarsest tolerance costing this much
                              # footprint. Measured: 0.05 drops a 3.4 mm
                              # screw-clearance circle from 12 points to 8,
                              # 0.01 blows a 4 mm-wide straight-sided rib up
                              # from 5 points to 30. Faceted cylinders were
                              # NOT this number's fault — see segment_tier,
                              # which used to pre-simplify at 1.5 px before
                              # this pass ever ran.


def _poly_area(P):
    """Absolute area of a closed polygon (shoelace)."""
    x, y = np.asarray(P, float).T
    return abs(float(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))) / 2.0


def fit_circle(poly, tol_mm=0.3, min_pts=8, min_r=0.6):
    """(cx, cz, r) if this loop really is a circle, else None.

    Grip-screw and pin clearances come off the scan as polygonised discs, and
    a 3.4 mm circle measured on a 0.2 mm raster only supports about a dozen
    vertices — that is the information limit, not a simplification failure.
    Recognising the circle lets the DXF carry one CIRCLE entity instead, which
    is exact, is one draggable object in SolveSpace rather than twelve points,
    and lets OpenSCAD tessellate it at $fn instead of inheriting our facets.
    The clearance offset also becomes r + c rather than a rasterised dilation.

    The boundary is sampled at vertices AND edge midpoints, which is the whole
    trick: every regular polygon has its vertices exactly on a circle, so a
    vertex-only test calls a square a circle. Midpoints are what separate
    them — for a regular n-gon they sit a sagitta inside the vertices, so the
    residual against the best-fit circle scales as 1/n^2.

    The radius comes from the AREA, not from the fit, so swapping the polygon
    for the circle preserves the footprint exactly — the same rule
    simplification follows everywhere else here.

    Both guards are needed. `tol_mm` is ABSOLUTE and belongs on the raster's
    scale — about one map pixel, because a loop that hugs a circle to within a
    pixel IS a circle as far as the measurement can say. Measured deviations
    of real polygonised discs are half a pixel: 0.258 mm at px=0.5, 0.139 mm
    at px=0.2. `_CIRCLE_REL_TOL` is the shape guard; without it a coarse
    tol_mm would swallow a small square boss whole.

    `min_pts` is 8 because the deviation test alone cannot separate a regular
    HEXAGON (10.0% of r) from an octagon (5.4%) by any margin worth trusting,
    and a hex recess is a plausible thing to meet on a frame. Every
    polygonised disc measured here clears 8 comfortably — 12-13 points on the
    real scan, 16-18 on the synthetic.
    """
    P = np.asarray(poly, float)
    if len(P) < min_pts:
        return None
    area = _poly_area(P)
    r = float(np.sqrt(area / np.pi)) if area > 0 else 0.0
    if r < min_r:
        return None
    S = np.vstack([P, (P + np.roll(P, -1, axis=0)) / 2.0])
    # algebraic (Kasa) least squares: x^2+y^2 = D*x + E*y + F, centre at D/2,E/2
    try:
        sol = np.linalg.lstsq(np.column_stack([S[:, 0], S[:, 1],
                                               np.ones(len(S))]),
                              (S ** 2).sum(1), rcond=None)[0]
    except np.linalg.LinAlgError:
        return None
    cx, cz = float(sol[0]) / 2.0, float(sol[1]) / 2.0
    dev = np.abs(np.linalg.norm(S - [cx, cz], axis=1) - r).max()
    if dev > tol_mm or dev > _CIRCLE_REL_TOL * r:
        return None
    return cx, cz, r


def _split_circles(polys, tol_mm):
    """Partition loops into (polygons, circles) — circles as (cx, cz, r)."""
    loops, circles = [], []
    for p in polys:
        fit = fit_circle(p, tol_mm) if tol_mm > 0 else None
        (circles if fit else loops).append(fit if fit else p)
    return loops, circles


def _pin_corners(P, refs, tol):
    """Put reference CORNERS exactly on the outline; return their indices.

    A tier that runs out to the frame's profile should use the profile's own
    corner, not rediscover it. approxPolyDP is free to cut across a corner —
    it only has to stay within eps of the contour — so the base tier and a
    tier stacked on it rounded the same profile corner differently and their
    walls stopped matching (0.72 mm apart on the synthetic). Pinning the
    corner and simplifying only the runs BETWEEN pinned points keeps corners
    shared and still lets the straight stretches collapse to two points.
    """
    prot = {}
    for r in refs:
        for c in np.asarray(r, float):
            d = np.linalg.norm(P - c, axis=1)
            j = int(np.argmin(d))
            if d[j] <= tol and j not in prot:
                prot[j] = c
    for j, c in prot.items():
        P[j] = c
    return P, np.array(sorted(prot), dtype=int)


def _simplify_protected(P, prot, eps):
    """approxPolyDP each run between pinned vertices, so those survive."""
    if len(prot) == 0:
        return cv2.approxPolyDP(P.astype(np.float32).reshape(-1, 1, 2),
                                eps, True).reshape(-1, 2)
    n, out = len(P), []
    for k, i0 in enumerate(prot):
        i1 = prot[(k + 1) % len(prot)]
        idx = np.arange(i0, i1 + (n if i1 <= i0 else 0) + 1) % n
        run = P[idx]
        seg = (run if len(run) < 3 else
               cv2.approxPolyDP(run.astype(np.float32).reshape(-1, 1, 2),
                                eps, False).reshape(-1, 2))
        out.append(seg[:-1])                 # the next run repeats this point
    return np.vstack(out)


def _snap_to_polygon(poly, refs, tol):
    """Pull vertices within tol of any reference boundary exactly onto it.

    Two outlines that nearly coincide must coincide EXACTLY, or the step
    between them shows up as an artefact in the built solid. Two cases:
    a tier running out to the edge of the frame (ref = the silhouette), and a
    tier sharing a wall with the tier below it (ref = that tier's outline) —
    a tall feature is built as several stacked extrusions, and if their
    footprints disagree by a fraction of a millimetre you get a terraced wall
    instead of one clean face.
    """
    P = np.asarray(poly, float)
    if isinstance(refs, np.ndarray):
        refs = [refs]
    rr = [np.asarray(r, float) for r in refs if len(np.asarray(r)) >= 2]
    if not rr or not len(P):
        return P.copy()
    A = np.vstack(rr)
    B = np.vstack([np.roll(r, -1, axis=0) for r in rr])
    d = B - A
    L2 = np.einsum("ij,ij->i", d, d)
    L2 = np.where(L2 > 0, L2, 1.0)
    Ad = np.einsum("mj,mj->m", A, d)
    out = P.copy()
    # nearest point on every segment for every vertex, in blocks: this runs on
    # the raw contour (thousands of points), so it can be neither a Python
    # loop nor one giant N x M temporary
    for i in range(0, len(P), 4096):
        Q = P[i:i + 4096]
        t = np.clip((Q @ d.T - Ad) / L2, 0.0, 1.0)
        proj = A[None, :, :] + t[:, :, None] * d[None, :, :]
        off = Q[:, None, :] - proj
        dist2 = np.einsum("nmj,nmj->nm", off, off)
        k = np.argmin(dist2, axis=1)
        rows = np.arange(len(Q))
        close = dist2[rows, k] <= tol * tol
        blk = out[i:i + 4096]
        blk[close] = proj[rows, k][close]
    return out


def _clip_to_silhouette(polys, sil: Silhouette, simplify_mm=0.2,
                        min_area_mm2=2.0, snap_tol=0.35, refs=None):
    """Trim tier outlines to the frame profile, at the profile's resolution.

    Doing this on the coarse thickness-map grid left every tier a fraction
    of a millimetre short of the outline — a visible ledge that no amount of
    grow_mm could close, because growing then clipped back to the same
    inset boundary.

    Order matters: SNAP to the profile first, then simplify. A tier boundary
    running along the profile becomes collinear once snapped, so simplifying
    can only drop redundant points from it, never pull it off the line. Doing
    it the other way round forces a simplify tolerance fine enough to protect
    the profile (0.2 mm), which then faithfully reproduces the thickness
    map's own 0.5 mm staircase — a straight feature edge came out visibly
    jagged (user-reported, seen in OpenSCAD). `simplify_mm` should therefore
    be set from the MAP resolution, which is what limits the boundary's
    accuracy, not from the silhouette's.
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

    if refs is None:
        refs = [np.asarray(sil.polygon)]
    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    out = []
    for c in cnts:
        if float(cv2.contourArea(c)) * sil.px ** 2 < min_area_mm2:
            continue
        a = c.reshape(-1, 2).astype(np.float64)
        world = np.stack([sil.x0 + a[:, 0] * sil.px,
                          sil.z0 + a[:, 1] * sil.px], axis=1)
        world = _snap_to_polygon(world, refs, snap_tol)
        # Pin the PROFILE's corners only. Pinning the tier below as well
        # propagates its vertex pattern up the stack, and a small round island
        # it happened to describe as an octagon then forced every tier above it
        # to be an octagon too.
        world, prot = _pin_corners(world, [np.asarray(sil.polygon)], snap_tol)
        # Tolerance PER ISLAND, chosen by how much area it costs. One number
        # cannot serve a 100 mm straight edge (wants ~1 mm, or it traces the
        # map's staircase), a 3 mm screw-clearance circle (1 mm makes it a
        # trapezoid) and a 2 mm-wide rib (which a size-based rule would
        # flatten). Take the coarsest tolerance that keeps the footprint, so
        # simplification can never quietly shrink a feature.
        target = _poly_area(world)
        cap = max(simplify_mm, _SIMPLIFY_FLOOR)
        best = None
        for eps in (cap, cap / 2, cap / 4, cap / 8, _SIMPLIFY_FLOOR):
            a = _simplify_protected(world, prot, eps)
            if len(a) < 3:
                continue
            best = a
            if (target <= 0
                    or abs(_poly_area(a) - target) <= _SIMPLIFY_AREA_TOL * target):
                break
        if best is None:
            continue
        world = best
        # snapping again costs nothing and pins any vertex the simplifier
        # nudged off a shared boundary back onto it
        out.append(_snap_to_polygon(world.astype(np.float64), refs, snap_tol))
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


def _prev_tier(tiers, tier, base_thickness, base_y0=0.0):
    """The tier immediately below this one on its own side, or None."""
    side = tier["side"]
    fo = tier_offset(tier, base_thickness, base_y0)
    best, best_f = None, base_offset(side, base_thickness, base_y0)
    for u in tiers:
        if u is tier or region_kind(u) != "tier" or u.get("side") != side:
            continue
        f = tier_offset(u, base_thickness, base_y0)
        if best_f < f < fo:
            best, best_f = u, f
    return best


def _prev_offset(tiers, tier, base_thickness, base_y0=0.0):
    """The offset of the tier immediately below this one, on its own side.

    That is the threshold this tier segments against: everything standing
    proud of the tier below gets pulled into this tier's outline.
    """
    prev = _prev_tier(tiers, tier, base_thickness, base_y0)
    return (tier_offset(prev, base_thickness, base_y0) if prev is not None
            else base_offset(tier["side"], base_thickness, base_y0))


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
        # ONE pixel, not two: sil_mask is a nearest-neighbour downsample of
        # the real profile, so it can only be inset by up to a pixel, and a
        # wider dilation also spreads the mask TANGENTIALLY along the profile.
        # That put a tier a pixel past the end of its own footprint down the
        # frame's front edge, 0.45 mm proud of the tier below it — which the
        # snap could not repair, because the vertex was already exactly on the
        # silhouette and snapping takes the nearest reference.
        spill = cv2.bitwise_and(
            cv2.dilate(img, np.ones((3, 3), np.uint8)),
            cv2.bitwise_not(maps.sil_mask))
        img = cv2.bitwise_or(img, spill)

    cnts, _ = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    polys = []
    for c in cnts:
        if float(cv2.contourArea(c)) * px * px < min_area_mm2:
            continue
        # Barely simplify here — HALF a pixel. This runs before the careful
        # per-island pass in _clip_to_silhouette, and that pass can only drop
        # points, never restore ones already thrown away: at simplify_mm/px
        # (1.5 px at the export raster) it decided a 3.4 mm screw-clearance
        # circle was an octagon, and no tolerance downstream could round it
        # back out. The contour is already run-length collapsed by
        # CHAIN_APPROX_SIMPLE, so this costs almost nothing.
        approx = cv2.approxPolyDP(c, 0.5, True)
        approx = approx.reshape(-1, 2).astype(np.float64)
        if len(approx) < 3:
            continue
        polys.append(np.stack([maps.x0 + approx[:, 0] * px,
                               maps.z0 + approx[:, 1] * px], axis=1))
    # A grown outline may now stick out past the frame; trim it there, at the
    # profile's own resolution so the two share an edge exactly. Snap to the
    # profile AND to the tier below: a tall feature is built as several
    # stacked extrusions, and their walls have to be the same wall or the
    # steps between them show as terracing. Simplify to twice the map pixel
    # at most — the boundary is a thickness step measured on that grid, so it
    # is only known to about a pixel.
    refs = []
    if maps.sil is not None:
        refs.append(np.asarray(maps.sil.polygon))
    below = _prev_tier(tiers, tier, base_thickness, base_y0)
    if below is not None:
        refs += [np.asarray(p) for p in (below.get("polys") or [])]
    # snap_tol scales with the map pixel: each contour's position is only
    # known to about a pixel, so two contours can disagree by a couple of
    # pixels from noise alone. Inside that they are the same wall; a genuine
    # difference (a ramped surface, where terracing is the correct answer) is
    # much larger than this.
    polys = _clip_to_silhouette(polys, maps.sil, min_area_mm2=min_area_mm2,
                                simplify_mm=2.0 * px, refs=refs or None,
                                snap_tol=max(0.35, 2.5 * px))

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
    """Re-segment every tier — thresholds are relative, so they all move.

    In stacking order, because each tier snaps its shared wall onto the tier
    below and so needs that one's outline to exist already.
    """
    for t in sort_tiers(regions):
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


def raster_bias_bound(polys, px_coarse, px_fine, safety=1.5):
    """How much footprint a FINER raster may legitimately shed, in mm².

    measure_maps rasterises conservatively, so every outline carries about half
    a pixel of outward bias — worth perimeter * px / 2 of area. Re-measuring
    more finely sheds the difference, and that is the whole point: it is the
    measurement getting better, not the model changing.

    It has to be a bound and not a percentage because the bias scales with
    PERIMETER while the guard compares AREA, so it is proportionally huge for a
    small island with a long boundary. A 30 mm-proud rod end (92 mm², 78 mm of
    perimeter) legitimately shrinks 14% going from 0.5 to 0.2 mm/px, which a
    flat 5% guard reverted to the coarse outline — and the coarse outline was
    too crude for fit_circle to recognise, so the same rod came out round on
    one side of the frame and faceted on the other (user-reported, via
    OpenSCAD). `safety` covers the coarse outline's simplified perimeter
    underestimating the true one.
    """
    peri = sum(float(np.linalg.norm(np.roll(np.asarray(p, float), -1, axis=0)
                                    - np.asarray(p, float), axis=1).sum())
               for p in polys)
    return safety * peri * max(0.0, float(px_coarse) - float(px_fine)) / 2.0


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
        # Judge only pixels properly inside the profile. The map reaches up to
        # two pixels PAST it — one from the grid's padding, half from
        # measure_maps' conservative rasterisation — and a tier is clipped to
        # the profile, so that rim can never be covered by one. Left in, it
        # reported the rail's 13 mm face as a 7 mm hole over a 1-2 px ring.
        # Whether tiers actually reach the profile is a separate question, and
        # snapping already answers it exactly.
        inside = cv2.erode(maps.sil_mask, np.ones((5, 5), np.uint8))
        short = np.where(np.isfinite(m) & (inside > 0), m - built, -np.inf)
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


def sketch_model(sil: Silhouette, base_thickness, regions,
                 base_y0=0.0, clearance=0.0, circle_tol=0.0):
    """The solid as a list of sketches with extrusion ranges.

    Each entry: {"key", "kind", "label", "loops" (world XZ),
    "circles" [(cx, cz, r)], "y_lo", "y_hi"}. Every tier extrudes from the
    same plane - y = base_y0 - so in CAD they all share one workplane and
    differ only in depth. The base straddles it. `clearance` (mm per side) is
    grown into the outlines and the depths, so what comes out is the finished
    subtraction solid.

    `circle_tol` > 0 turns tier islands that really are circles into CIRCLE
    entities (see fit_circle). It runs BEFORE the clearance offset, so those
    grow as r + clearance — exact, where _grow_loops would re-rasterise the
    disc and hand back a fresh set of facets.
    """
    c = float(clearance)
    T = float(base_thickness)
    yR_b, yL_b = base_faces(T, base_y0)
    model = [{"key": "00_base", "kind": "base",
              "label": f"base tier - whole silhouette, {T:.2f} mm thick",
              "loops": _grow_loops([np.asarray(sil.polygon, float)], c),
              "circles": [], "y_lo": yL_b - c, "y_hi": yR_b + c}]

    n = {"left": 0, "right": 0}
    for t in sort_tiers(regions):
        if region_kind(t) == "rect":
            w = float(t["width"])
            loops = _grow_loops([np.array([
                [t["x0"], t["z0"]], [t["x1"], t["z0"]],
                [t["x1"], t["z1"]], [t["x0"], t["z1"]]], float)], c)
            model.append({"key": f"rect{len(model):02d}", "kind": "rect",
                          "label": f"legacy rectangle, {w:.2f} mm thick",
                          "loops": loops, "circles": [],
                          "y_lo": base_y0 - w / 2 - c,
                          "y_hi": base_y0 + w / 2 + c})
            continue
        if not t.get("polys"):
            continue
        side = t["side"]
        n[side] += 1
        fo = tier_offset(t, T, base_y0)
        polys, circles = _split_circles([np.asarray(p, float)
                                         for p in t["polys"]], circle_tol)
        loops = _grow_loops(polys, c) if polys else []
        circles = [(cx, cz, r + c) for cx, cz, r in circles]
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
            "loops": loops, "circles": circles,
            "y_lo": y_lo, "y_hi": y_hi})

    return model


def _sketch_entities(s, layer=None):
    lay = layer or s["key"]
    return ([dxf_polyline(loop, True, lay) for loop in s["loops"]]
            + [dxf_circle(x, z, r, lay) for x, z, r in s.get("circles", ())])


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
          ""]
    n_circ = sum(len(s.get("circles", ())) for s in model)
    if n_circ:
        L += [f"{n_circ} round island(s) are true CIRCLE entities, not",
              "polygons - grip-screw and pin clearances, recognised from the",
              "scan. They carry an exact centre and radius (clearance already",
              "added to the radius), so SolveSpace gives you one circle to",
              "drag and dimension, and OpenSCAD tessellates them at $fn",
              "instead of inheriting the raster's facets. Listed under",
              "'circles' below. Set 'circle fit mm' to 0 to export them as",
              "polygons like everything else.",
              ""]
    L += [
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
    if n_circ:
        L += ["", f"circles ({n_circ}) - centre and radius as exported, "
                  f"clearance included", "-" * 100]
        for s in model:
            for cx, cz, r in s.get("circles", ()):
                L.append(f"{s['key'] + '.dxf':<26} centre "
                         f"({cx:+9.3f}, {cz:+9.3f})  r {r:7.3f}  "
                         f"dia {2 * r:7.3f}")
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
