"""Minimal mesh I/O: STL (binary+ASCII), OBJ, PLY (ascii + binary_little_endian).

No external mesh libraries required — numpy only.
Meshes are (vertices: float64 (N,3), faces: int64 (M,3)).
"""
from __future__ import annotations

import struct
import numpy as np


# ---------------------------------------------------------------- loading

def load_mesh(path: str):
    """Load a mesh file by extension. Returns (vertices, faces).

    faces may be an empty (0,3) array for point-cloud-only PLY files.
    """
    p = path.lower()
    if p.endswith(".stl"):
        return _load_stl(path)
    if p.endswith(".obj"):
        return _load_obj(path)
    if p.endswith(".ply"):
        return _load_ply(path)
    raise ValueError(f"Unsupported mesh format: {path} (use STL, OBJ, or PLY)")


def _load_stl(path):
    with open(path, "rb") as f:
        header = f.read(5)
    if header[:5] == b"solid":
        # Possibly ASCII, but some binary files also start with 'solid'.
        try:
            return _load_stl_ascii(path)
        except Exception:
            pass
    return _load_stl_binary(path)


def _load_stl_binary(path):
    with open(path, "rb") as f:
        f.read(80)
        (ntri,) = struct.unpack("<I", f.read(4))
        data = np.fromfile(f, dtype=np.uint8, count=ntri * 50)
    if data.size != ntri * 50:
        raise ValueError("Truncated binary STL")
    rec = data.reshape(ntri, 50)
    tri = rec[:, 12:48].copy().view("<f4").reshape(ntri, 3, 3).astype(np.float64)
    verts = tri.reshape(-1, 3)
    faces = np.arange(len(verts), dtype=np.int64).reshape(-1, 3)
    return _weld(verts, faces)


def _load_stl_ascii(path):
    verts = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            line = line.strip()
            if line.startswith("vertex"):
                parts = line.split()
                verts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    if not verts or len(verts) % 3 != 0:
        raise ValueError("Not a valid ASCII STL")
    verts = np.asarray(verts, dtype=np.float64)
    faces = np.arange(len(verts), dtype=np.int64).reshape(-1, 3)
    return _weld(verts, faces)


def _load_obj(path):
    verts, faces = [], []
    with open(path, "r", errors="replace") as f:
        for line in f:
            if line.startswith("v "):
                p = line.split()
                verts.append([float(p[1]), float(p[2]), float(p[3])])
            elif line.startswith("f "):
                idx = [int(tok.split("/")[0]) for tok in line.split()[1:]]
                idx = [i - 1 if i > 0 else len(verts) + i for i in idx]
                for k in range(1, len(idx) - 1):  # fan-triangulate
                    faces.append([idx[0], idx[k], idx[k + 1]])
    v = np.asarray(verts, dtype=np.float64)
    f = np.asarray(faces, dtype=np.int64) if faces else np.zeros((0, 3), np.int64)
    return v, f


_PLY_TYPES = {
    "char": "i1", "int8": "i1", "uchar": "u1", "uint8": "u1",
    "short": "i2", "int16": "i2", "ushort": "u2", "uint16": "u2",
    "int": "i4", "int32": "i4", "uint": "u4", "uint32": "u4",
    "float": "f4", "float32": "f4", "double": "f8", "float64": "f8",
}


def _load_ply(path):
    with open(path, "rb") as f:
        # ---- header
        line = f.readline().strip()
        if line != b"ply":
            raise ValueError("Not a PLY file")
        fmt = None
        elements = []  # (name, count, [(propname, dtype) or ('list', ctype, itype, name)])
        cur = None
        while True:
            line = f.readline()
            if not line:
                raise ValueError("Unexpected EOF in PLY header")
            tok = line.decode("ascii", "replace").split()
            if not tok:
                continue
            if tok[0] == "format":
                fmt = tok[1]
            elif tok[0] == "element":
                cur = (tok[1], int(tok[2]), [])
                elements.append(cur)
            elif tok[0] == "property":
                if tok[1] == "list":
                    cur[2].append(("list", _PLY_TYPES[tok[2]], _PLY_TYPES[tok[3]], tok[4]))
                else:
                    cur[2].append((tok[2], _PLY_TYPES[tok[1]]))
            elif tok[0] == "end_header":
                break
        if fmt == "ascii":
            return _ply_ascii_body(f, elements)
        if fmt == "binary_little_endian":
            return _ply_binary_body(f, elements, "<")
        if fmt == "binary_big_endian":
            return _ply_binary_body(f, elements, ">")
        raise ValueError(f"Unsupported PLY format: {fmt}")


def _ply_ascii_body(f, elements):
    verts, faces = None, []
    for name, count, props in elements:
        if name == "vertex":
            cols = [p[0] for p in props]
            data = []
            for _ in range(count):
                vals = f.readline().split()
                data.append([float(x) for x in vals[: len(cols)]])
            arr = np.asarray(data, dtype=np.float64)
            verts = arr[:, [cols.index("x"), cols.index("y"), cols.index("z")]]
        elif name == "face":
            for _ in range(count):
                vals = [int(x) for x in f.readline().split()]
                n, idx = vals[0], vals[1:]
                for k in range(1, n - 1):
                    faces.append([idx[0], idx[k], idx[k + 1]])
        else:
            for _ in range(count):
                f.readline()
    fc = np.asarray(faces, dtype=np.int64) if faces else np.zeros((0, 3), np.int64)
    return verts, fc


def _ply_binary_body(f, elements, endian):
    verts, faces = None, []
    for name, count, props in elements:
        if name == "vertex" and all(p[0] != "list" for p in props):
            dt = np.dtype([(p[0], endian + p[1]) for p in props])
            arr = np.frombuffer(f.read(dt.itemsize * count), dtype=dt)
            verts = np.stack(
                [arr["x"], arr["y"], arr["z"]], axis=1).astype(np.float64)
        elif name == "face":
            # Assume single list property (vertex_indices) — the common case.
            lp = [p for p in props if p[0] == "list"][0]
            cdt = np.dtype(endian + lp[1])
            idt = np.dtype(endian + lp[2])
            for _ in range(count):
                (n,) = np.frombuffer(f.read(cdt.itemsize), dtype=cdt)
                idx = np.frombuffer(f.read(idt.itemsize * int(n)), dtype=idt)
                for k in range(1, int(n) - 1):
                    faces.append([int(idx[0]), int(idx[k]), int(idx[k + 1])])
        else:
            # Skip unknown fixed-size element
            size = sum(np.dtype(endian + p[1]).itemsize for p in props if p[0] != "list")
            if any(p[0] == "list" for p in props):
                raise ValueError(f"Cannot skip PLY element '{name}' with list property")
            f.read(size * count)
    fc = np.asarray(faces, dtype=np.int64) if faces else np.zeros((0, 3), np.int64)
    return verts, fc


def _weld(verts, faces, decimals=6):
    """Merge duplicate vertices (triangle-soup formats like STL)."""
    key = np.round(verts, decimals)
    _, first, inv = np.unique(
        key.view([("", key.dtype)] * 3), return_index=True, return_inverse=True)
    inv = inv.reshape(-1)
    return verts[first], inv[faces].astype(np.int64)


# ---------------------------------------------------------------- saving

def save_stl(path, verts, faces):
    """Write a binary STL."""
    verts = np.asarray(verts, dtype=np.float64)
    faces = np.asarray(faces, dtype=np.int64)
    tri = verts[faces].astype("<f4")                    # (M,3,3)
    a = tri[:, 1] - tri[:, 0]
    b = tri[:, 2] - tri[:, 0]
    n = np.cross(a, b)
    norm = np.linalg.norm(n, axis=1, keepdims=True)
    norm[norm == 0] = 1.0
    n = (n / norm).astype("<f4")
    rec = np.zeros(len(faces), dtype=np.dtype([
        ("normal", "<f4", 3), ("v", "<f4", (3, 3)), ("attr", "<u2")]))
    rec["normal"] = n
    rec["v"] = tri
    with open(path, "wb") as f:
        f.write(b"frame2solid".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(faces)))
        rec.tofile(f)


def save_dxf_polyline(path, points_xy, closed=True):
    """Write a minimal R12 DXF containing one POLYLINE from Nx2 points."""
    lines = ["0", "SECTION", "2", "ENTITIES", "0", "POLYLINE", "8", "0",
             "66", "1", "70", "1" if closed else "0"]
    for x, y in points_xy:
        lines += ["0", "VERTEX", "8", "0", "10", f"{x:.4f}", "20", f"{y:.4f}"]
    lines += ["0", "SEQEND", "0", "ENDSEC", "0", "EOF"]
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


# ---------------------------------------------------------------- checks

def watertight_report(verts, faces):
    """Check that every edge is shared by exactly two triangles."""
    e = np.concatenate([faces[:, [0, 1]], faces[:, [1, 2]], faces[:, [2, 0]]])
    e = np.sort(e, axis=1)
    uniq, counts = np.unique(e, axis=0, return_counts=True)
    boundary = int((counts == 1).sum())
    nonmanifold = int((counts > 2).sum())
    return {
        "vertices": int(len(verts)),
        "faces": int(len(faces)),
        "boundary_edges": boundary,
        "nonmanifold_edges": nonmanifold,
        "watertight": boundary == 0 and nonmanifold == 0,
    }
