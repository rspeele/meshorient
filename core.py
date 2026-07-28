"""frame2solid core pipeline.

Coordinate convention (after the Orient step):
    X = along the bore / frame length
    Y = across the frame (left-right); the frame's symmetry plane is Y = 0
    Z = vertical (grip height)
The "side view" is the XZ plane, looking along +Y. All units are mm.

Pipeline:
    scan mesh -> orient -> side-view silhouette (outer contour only, windows
    filled) -> width (thickness) map -> plateau regions -> extras (screw
    holes, magazine path, ...) -> SDF voxel build -> watertight STL.
"""
from __future__ import annotations

import numpy as np
import cv2
from skimage import measure

from meshio_lite import save_stl, save_dxf_polyline, watertight_report


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
        cv2.fillPoly(img, np.round(pts).astype(np.int32), 255)
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


# ============================================================== width map

def width_map(verts, sil: Silhouette, px=0.6):
    """Measured left-right width of the scan per side-view pixel.

    Returns (wmap, x0, z0, px): wmap in mm, NaN where unmeasured/outside.
    """
    x, y, z = verts[:, 0], verts[:, 1], verts[:, 2]
    ex = sil.world_extent()
    x0, z0 = ex[0], ex[2]
    w = int(np.ceil((ex[1] - x0) / px)) + 1
    h = int(np.ceil((ex[3] - z0) / px)) + 1
    ci = np.clip(((x - x0) / px).round().astype(int), 0, w - 1)
    ri = np.clip(((z - z0) / px).round().astype(int), 0, h - 1)
    lin = ri * w + ci
    ymin = np.full(h * w, np.inf)
    ymax = np.full(h * w, -np.inf)
    np.minimum.at(ymin, lin, y)
    np.maximum.at(ymax, lin, y)
    wm = (ymax - ymin).reshape(h, w)
    wm[~np.isfinite(wm)] = np.nan

    # fill small gaps with grey-closing
    filled = wm.copy()
    nanmask = np.isnan(filled)
    filled[nanmask] = 0
    k = np.ones((5, 5), np.uint8)
    closed = cv2.morphologyEx(filled.astype(np.float32), cv2.MORPH_CLOSE, k)
    wm = np.where(nanmask, closed, wm)
    wm[wm <= 0] = np.nan

    # mask to silhouette
    sil_small = cv2.resize(sil.mask, (w, h), interpolation=cv2.INTER_NEAREST)
    wm[sil_small == 0] = np.nan
    return wm, x0, z0, px


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


# ================================================================= build

def clean_mesh(verts, faces, tol=1e-3):
    """Weld vertices closer than tol (mm), drop degenerate & duplicate faces.

    Keeps the mesh robust through float32 STL round-trips and downstream
    boolean engines.
    """
    key = np.round(verts / tol).astype(np.int64)
    _, first, inv = np.unique(key.view([("", key.dtype)] * 3),
                              return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    verts2 = verts[first]
    faces2 = inv[faces]
    ok = ((faces2[:, 0] != faces2[:, 1]) & (faces2[:, 1] != faces2[:, 2])
          & (faces2[:, 0] != faces2[:, 2]))
    faces2 = faces2[ok]
    fkey = np.sort(faces2, axis=1)
    _, fidx = np.unique(fkey.view([("", fkey.dtype)] * 3), return_index=True)
    faces2 = faces2[np.sort(fidx)]
    return verts2, faces2.astype(np.int64)

def _box_sdf(X, Y, Z, x0, z0, x1, z1, ywidth, tilt_deg=0.0):
    """SDF of a box drawn as a side-view rect, extruded ±ywidth/2,
    optionally tilted (rotated in the XZ plane about the rect center)."""
    cx, cz = (x0 + x1) / 2, (z0 + z1) / 2
    hx, hz = abs(x1 - x0) / 2, abs(z1 - z0) / 2
    a = np.deg2rad(tilt_deg)
    c, s = np.cos(a), np.sin(a)
    Xr = (X - cx) * c + (Z - cz) * s
    Zr = -(X - cx) * s + (Z - cz) * c
    d = np.maximum(np.abs(Xr) - hx, np.abs(Zr) - hz)
    return np.maximum(d, np.abs(Y) - ywidth / 2)


def _cyl_y_sdf(X, Y, Z, cx, cz, dia, ylen):
    d = np.sqrt((X - cx) ** 2 + (Z - cz) ** 2) - dia / 2
    return np.maximum(d, np.abs(Y) - ylen / 2)


def extra_sdf(extra, X, Y, Z):
    if extra["kind"] == "box":
        return _box_sdf(X, Y, Z, extra["x0"], extra["z0"], extra["x1"],
                        extra["z1"], extra["ywidth"], extra.get("tilt_deg", 0.0))
    if extra["kind"] == "cyl_y":
        return _cyl_y_sdf(X, Y, Z, extra["x"], extra["z"],
                          extra["dia"], extra["ylen"])
    raise ValueError(f"Unknown extra kind: {extra['kind']}")


def extra_bounds(extra):
    """(x0,x1),(y0,y1),(z0,z1) world bounds of an extra."""
    if extra["kind"] == "box":
        cx, cz = (extra["x0"] + extra["x1"]) / 2, (extra["z0"] + extra["z1"]) / 2
        hx = abs(extra["x1"] - extra["x0"]) / 2
        hz = abs(extra["z1"] - extra["z0"]) / 2
        r = np.hypot(hx, hz)  # conservative for tilt
        hw = extra["ywidth"] / 2
        return (cx - r, cx + r), (-hw, hw), (cz - r, cz + r)
    if extra["kind"] == "cyl_y":
        r = extra["dia"] / 2
        hw = extra["ylen"] / 2
        return (extra["x"] - r, extra["x"] + r), (-hw, hw), \
               (extra["z"] - r, extra["z"] + r)
    raise ValueError(extra["kind"])


def build_solid(sil: Silhouette, base_width: float, regions, extras,
                clearance=0.15, voxel=0.3, return_sdf=False):
    """Build the watertight subtraction solid.

    regions: list of {"x0","z0","x1","z1","width"} — later entries override
             earlier ones where they overlap (painter's order).
    extras:  list of extra dicts (see extra_sdf).
    clearance: dilation in mm applied to the whole solid (fit clearance).

    Returns (verts, faces, report[, sdf_pack]).
    """
    pad = clearance + 3 * voxel + 1.0

    # ---- bounds
    poly = sil.polygon
    xlo, xhi = poly[:, 0].min(), poly[:, 0].max()
    zlo, zhi = poly[:, 1].min(), poly[:, 1].max()
    wmax = base_width + 0.0
    for r in regions:
        wmax = max(wmax, r["width"])
    ylo, yhi = -wmax / 2, wmax / 2
    for e in extras:
        (ex0, ex1), (ey0, ey1), (ez0, ez1) = extra_bounds(e)
        xlo, xhi = min(xlo, ex0), max(xhi, ex1)
        ylo, yhi = min(ylo, ey0), max(yhi, ey1)
        zlo, zhi = min(zlo, ez0), max(zhi, ez1)
    xlo, xhi = xlo - pad, xhi + pad
    ylo, yhi = ylo - pad, yhi + pad
    zlo, zhi = zlo - pad, zhi + pad

    xs = np.arange(xlo, xhi + voxel, voxel)
    ys = np.arange(ylo, yhi + voxel, voxel)
    zs = np.arange(zlo, zhi + voxel, voxel)
    nx, ny, nz = len(xs), len(ys), len(zs)
    mem_mb = nx * ny * nz * 4 / 1e6

    # ---- 2D signed distance to silhouette polygon, on the (x,z) build grid
    # raster: rows = x index, cols = z index  (so d2[ix, iz])
    img = np.zeros((nx, nz), np.uint8)
    pp = np.empty((len(poly), 2), np.float64)
    pp[:, 0] = (poly[:, 1] - zlo) / voxel        # col = z
    pp[:, 1] = (poly[:, 0] - xlo) / voxel        # row = x
    cv2.fillPoly(img, [np.round(pp).astype(np.int32)], 255)
    d_in = cv2.distanceTransform(img, cv2.DIST_L2, 5)
    d_out = cv2.distanceTransform(255 - img, cv2.DIST_L2, 5)
    d2 = (d_out - d_in) * voxel                  # + outside, - inside

    # ---- width per (x,z) cell
    W = np.full((nx, nz), base_width, np.float32)
    for r in regions:
        i0 = np.clip(int((min(r["x0"], r["x1"]) - xlo) / voxel), 0, nx - 1)
        i1 = np.clip(int((max(r["x0"], r["x1"]) - xlo) / voxel) + 1, 0, nx)
        k0 = np.clip(int((min(r["z0"], r["z1"]) - zlo) / voxel), 0, nz - 1)
        k1 = np.clip(int((max(r["z0"], r["z1"]) - zlo) / voxel) + 1, 0, nz)
        W[i0:i1, k0:k1] = r["width"]

    # ---- 3D SDF: frame prism = intersection(outline extrusion, |y| < w/2)
    Yabs = np.abs(ys)[None, :, None].astype(np.float32)
    sdf = np.maximum(d2[:, None, :].astype(np.float32),
                     Yabs - W[:, None, :] / 2)

    # ---- extras (union)
    if extras:
        X3 = xs[:, None, None].astype(np.float32)
        Y3 = ys[None, :, None].astype(np.float32)
        Z3 = zs[None, None, :].astype(np.float32)
        for e in extras:
            np.minimum(sdf, extra_sdf(e, X3, Y3, Z3).astype(np.float32), out=sdf)

    # ---- clearance dilation
    if clearance:
        sdf -= clearance

    # ---- seal the domain boundary so marching cubes closes the surface
    big = np.float32(10 * voxel)
    sdf[0, :, :] = big; sdf[-1, :, :] = big
    sdf[:, 0, :] = big; sdf[:, -1, :] = big
    sdf[:, :, 0] = big; sdf[:, :, -1] = big

    verts, faces, _, _ = measure.marching_cubes(sdf, level=0.0,
                                                spacing=(voxel, voxel, voxel))
    verts = verts + np.array([xlo, ylo, zlo])
    faces = faces.astype(np.int64)
    verts, faces = clean_mesh(verts, faces)

    # ---- outward orientation (positive volume)
    v = verts[faces]
    vol6 = np.einsum("ij,ij->i", v[:, 0], np.cross(v[:, 1], v[:, 2])).sum()
    if vol6 < 0:
        faces = faces[:, ::-1]
        vol6 = -vol6

    report = watertight_report(verts, faces)
    report["volume_cm3"] = float(vol6 / 6.0 / 1000.0)
    report["voxel_mm"] = voxel
    report["clearance_mm"] = clearance
    report["grid"] = (nx, ny, nz)
    report["grid_mem_mb"] = round(mem_mb, 1)

    if return_sdf:
        return verts, faces, report, (sdf, (xlo, ylo, zlo), voxel)
    return verts, faces, report


def save_solid(path, verts, faces):
    save_stl(path, verts, faces)


# ------------------------------------------------------------- previews

def sdf_slice_y(sdf_pack, y_mm=0.0):
    """Cross-section mask at a given Y (for preview). Returns (mask, extent)
    with extent = (xmin, xmax, zmin, zmax) for imshow."""
    sdf, (xlo, ylo, zlo), voxel = sdf_pack
    iy = int(round((y_mm - ylo) / voxel))
    iy = np.clip(iy, 0, sdf.shape[1] - 1)
    m = (sdf[:, iy, :] < 0)          # (nx, nz)
    extent = (xlo, xlo + voxel * sdf.shape[0], zlo, zlo + voxel * sdf.shape[2])
    return m, extent


def sdf_slice_x(sdf_pack, x_mm):
    """Cross-section at a given X station: returns (mask (ny,nz), extent
    (ymin,ymax,zmin,zmax))."""
    sdf, (xlo, ylo, zlo), voxel = sdf_pack
    ix = int(round((x_mm - xlo) / voxel))
    ix = np.clip(ix, 0, sdf.shape[0] - 1)
    m = (sdf[ix, :, :] < 0)
    extent = (ylo, ylo + voxel * sdf.shape[1], zlo, zlo + voxel * sdf.shape[2])
    return m, extent
