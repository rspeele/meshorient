"""Generate a synthetic 'frame scan' for testing frame2solid.

Mock frame (mm), deliberately awkward like a real scan:
  - top rail block: X -85..85, Z 30..50, width 26  (with a small through-hole)
  - grip block:     X -40..40, Z -60..30, width 22, hollow magwell with a
                    through-WINDOW (X -20..10, Z -40..-10) in both side walls
  - front tang:     X -85..-40, Z -10..30, width 12
  - trigger-bar boss: X -20..25, Z 0..8, on the RIGHT side ONLY (Y 11..14),
                    i.e. the frame is asymmetric there — 25 mm total, of
                    which 14 is right of the mid-plane
  - vertex jitter 0.05 mm, 2% of triangles randomly dropped (scan holes)
"""
import numpy as np
from meshio_lite import save_stl

rng = np.random.default_rng(7)
V, F = [], []


def add_box(x0, y0, z0, x1, y1, z1, spacing=1.2):
    """Axis-aligned box as triangles, subdivided to scan-like vertex density."""
    corners = np.array([[x0, y0, z0], [x1, y0, z0], [x1, y1, z0], [x0, y1, z0],
                        [x0, y0, z1], [x1, y0, z1], [x1, y1, z1], [x0, y1, z1]],
                       float)
    quads = [(0, 1, 2, 3), (4, 7, 6, 5), (0, 4, 5, 1),
             (1, 5, 6, 2), (2, 6, 7, 3), (3, 7, 4, 0)]
    for q in quads:
        p0, p1, p2, p3 = corners[list(q)]
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

V = np.asarray(V, float)
F = np.asarray(F, np.int64)

# scan-like imperfections
V += rng.normal(0, 0.05, V.shape)
keep = rng.random(len(F)) > 0.02
F = F[keep]

save_stl("synthetic_frame_scan.stl", V, F)
print(f"synthetic_frame_scan.stl: {len(V)} verts, {len(F)} tris")
