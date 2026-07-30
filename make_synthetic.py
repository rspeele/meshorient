"""Generate a synthetic 'frame scan' for testing frame2solid.

Mock frame (mm), deliberately awkward like a real scan:
  - top rail block: X -85..85, Z 30..50, width 26  (with a small through-hole)
  - grip block:     X -40..40, Z -60..30, width 22, hollow magwell with a
                    through-WINDOW (X -20..10, Z -40..-10) in both side walls
  - front tang:     X -85..-40, Z -10..30, width 12
  - trigger-bar boss: X -20..25, Z 0..8, on the RIGHT side ONLY (Y 11..14),
                    i.e. the frame is asymmetric there — 25 mm total, of
                    which 14 is right of the mid-plane
  - diagonal rib:   a 4 mm-wide strip at ~30 deg across the grip's right
                    wall, 1.5 mm proud (Y 11..12.5). NOT axis aligned, so it
                    exercises staircasing and over-simplification of tier
                    boundaries — rectangles cannot.
  - round boss:     an 8 mm circle on the LEFT wall at X=25 Z=15, 3.5 mm
                    proud (Y -14.5..-11) — a different height from the
                    right-side boss on purpose, so the Y bbox stays
                    asymmetric and base_y0 has work to do. Curved boundary,
                    small island.
  - vertex jitter 0.05 mm, 2% of triangles randomly dropped (scan holes)
"""
import numpy as np
from meshio_lite import save_stl

rng = np.random.default_rng(7)
V, F = [], []


def add_quad(p0, p1, p2, p3, spacing=1.2):
    """One quad as triangles, subdivided to scan-like vertex density."""
    nu = max(1, int(np.ceil(np.linalg.norm(p1 - p0) / spacing)))
    nv = max(1, int(np.ceil(np.linalg.norm(p3 - p0) / spacing)))
    for i in range(nu):
        for j in range(nv):
            a = p0 + (p1 - p0) * i / nu + (p3 - p0) * j / nv
            b = p0 + (p1 - p0) * (i + 1) / nu + (p3 - p0) * j / nv
            c = p0 + (p1 - p0) * (i + 1) / nu + (p3 - p0) * (j + 1) / nv
            d = p0 + (p1 - p0) * i / nu + (p3 - p0) * (j + 1) / nv
            i0 = len(V)
            V.extend([a, b, c, d])
            F.append([i0, i0 + 1, i0 + 2])
            F.append([i0, i0 + 2, i0 + 3])


def add_box(x0, y0, z0, x1, y1, z1, spacing=1.2):
    """Axis-aligned box as triangles."""
    corners = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]],
                       float)
    for q in [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
              (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]:
        add_quad(*corners[list(q)], spacing=spacing)


def add_prism(poly_xz, y0, y1, spacing=1.2):
    """Extrude an arbitrary convex XZ polygon along Y.

    This is how the NON-AXIS-ALIGNED features get built. Everything else here
    is axis-aligned, which makes the synthetic scan useless for catching the
    artefacts that only show up on diagonal and curved tier boundaries —
    staircasing, over-simplification, terracing between stacked tiers.
    """
    P = np.asarray(poly_xz, float)
    n = len(P)
    for i in range(n):                                   # side walls
        a, b = P[i], P[(i + 1) % n]
        add_quad(np.array([a[0], y0, a[1]]), np.array([b[0], y0, b[1]]),
                 np.array([b[0], y1, b[1]]), np.array([a[0], y1, a[1]]),
                 spacing=spacing)
    for y in (y0, y1):                                   # caps, fan-triangulated
        for i in range(1, n - 1):
            i0 = len(V)
            V.extend([np.array([P[0][0], y, P[0][1]]),
                      np.array([P[i][0], y, P[i][1]]),
                      np.array([P[i + 1][0], y, P[i + 1][1]])])
            F.append([i0, i0 + 1, i0 + 2])


def strip_xz(a, b, half_width):
    """Rectangle of half_width either side of the line a->b, in XZ."""
    a, b = np.asarray(a, float), np.asarray(b, float)
    u = (b - a) / np.linalg.norm(b - a)
    nrm = np.array([-u[1], u[0]]) * half_width
    return [a + nrm, b + nrm, b - nrm, a - nrm]


def circle_xz(cx, cz, r, n=48):
    t = np.linspace(0, 2 * np.pi, n, endpoint=False)
    return np.stack([cx + r * np.cos(t), cz + r * np.sin(t)], axis=1)


# --- top rail block (width 26) with a small through-hole near X=57, Z=37
# left/right walls of the rail, split around the hole
for (ylo, yhi) in [(-13.0, -3.0), (3.0, 13.0)]:
    # main run, minus hole zone x 54..60, z 34..40 (through both walls)
    add_box(-85, ylo, 30, 54, yhi, 50)
    add_box(60, ylo, 30, 85, yhi, 50)
    add_box(54, ylo, 30, 60, yhi, 34)
    add_box(54, ylo, 40, 60, yhi, 50)
add_box(-85, -3, 30, 85, 3, 50)   # solid core of rail

# --- grip block: hollow magwell, side walls with a through-window
# side walls y in [8,11] and [-11,-8]; window X -20..10, Z -40..-10
for (ylo, yhi) in [(8.0, 11.0), (-11.0, -8.0)]:
    add_box(-40, ylo, -60, -20, yhi, 30)          # front of window
    add_box(10, ylo, -60, 40, yhi, 30)            # rear of window
    add_box(-20, ylo, -60, 10, yhi, -40)          # below window
    add_box(-20, ylo, -10, 10, yhi, 30)           # above window
add_box(-40, -11, -60, -34, 11, 30)               # front strap
add_box(34, -11, -60, 40, 11, 30)                 # rear strap

# --- front tang (width 12)
add_box(-85, -6, -10, -40, 6, 30)

# --- trigger-bar boss: right side only, sits proud of the grip's right wall
add_box(-20, 11, 0, 25, 14, 8)

# --- NON-AXIS-ALIGNED features, so the tests cover diagonal and curved tier
#     boundaries and not just rectangles.
# a diagonal rib down the right side of the grip, 1.5 mm proud of the wall
add_prism(strip_xz((14, -52), (34, -18), 2.0), 11.0, 12.5)
# a round boss on the LEFT side, 3.5 mm proud of that wall. Deliberately
# NOT the same height as the right-side boss (14.0): the frame has to stay
# asymmetric about its Y bounding-box centre, because that is what makes
# base_y0 do any work.
add_prism(circle_xz(25, 15, 4.0), -14.5, -11.0)

V = np.asarray(V, float)
F = np.asarray(F, np.int64)

# scan-like imperfections
V += rng.normal(0, 0.05, V.shape)
keep = rng.random(len(F)) > 0.02
F = F[keep]

save_stl("synthetic_frame_scan.stl", V, F)
print(f"synthetic_frame_scan.stl: {len(V)} verts, {len(F)} tris")
