# frame2solid

Turns a 3D scan of a pistol frame into a clean, watertight **subtraction
solid** — the thing you boolean-subtract from a donor grip exterior to make a
transplanted grip. Fills magwell windows and pin holes automatically, rebuilds
the frame as flat-sided plateau extrusions at calipered widths, lets you add
clearance solids (magazine path, grip screws, levers), applies a fit-clearance
offset, and exports a guaranteed-watertight STL.

## Install (Windows)

1. Install Python 3.10+ from https://python.org (check "Add to PATH").
2. In a command prompt, in this folder:

       pip install -r requirements.txt
       python app.py

The GUI needs no other software. Try it first with the included
`synthetic_frame_scan.stl` (a mock frame with a magwell window and three
thicknesses) — the default scan path points at it.

## Coordinate convention

After the Orient step your scan must sit like this (all mm):

- **X** = along the bore / frame length
- **Y** = across the frame (left-right); symmetry plane at **Y = 0**
- **Z** = vertical

The side view (X-Z) is what becomes the outline. If you've already aligned the
scan in MeshMixer/Blender, export it that way and skip PCA; otherwise
"Auto-orient (PCA)" plus the 90° buttons gets you there in a few clicks.

## The five steps

**1 · Load/Orient.** Load STL/OBJ/PLY. Three projection views update as you
rotate. You're done when the SIDE view shows the classic frame profile, the
FRONT view looks thin, and the TOP view is symmetric about Y=0. "re-center"
puts the bounding-box center at the origin (it runs automatically after each
rotation).

**2 · Silhouette.** Extracts the side-view **outer** contour. Interior
windows, pin holes, and lightening cuts vanish automatically — only the
outermost outline is kept, which is exactly the "fill it solid" behavior you
want in a subtraction tool. Parameters: `px` raster resolution (0.15 mm is
fine), `close` closes gaps up to this size (scanner dropouts), `simp` outline
simplification tolerance. "Export DXF" writes the outline for tracing in
SolveSpace if you'd rather rebuild in CAD.

**3 · Regions.** Shows a thickness heatmap measured from the scan (left-right
width per side-view pixel). The **base width** (prefilled with the scan's
median) applies to the whole outline; drag rectangles over areas that are
thicker or thinner — rails, bosses, tangs — and each gets the measured 95th-
percentile width, which you can override in the "last w" box (calipers beat
the scanner; trust your measurement). Later rectangles override earlier ones
where they overlap.

**4 · Extras.** Clearance solids beyond the frame itself, in the same
coordinates. **Drag** to add a box (set its across-width and tilt first — e.g.
a magazine insertion path angled with the grip, drawn from above the magwell
down past where any grip could reach). **Click** to add a Y-axis cylinder
(grip screw holes, pin punches — set diameter/length first). The text boxes
edit the **last** extra added. Everything here gets unioned with the frame
before subtraction, so nothing you build later can block a magazine or bury a
screw.

**5 · Build.** Voxel signed-distance-field build: outline extruded per-region
to its width, extras unioned, everything dilated by `clearance`, then meshed.
This engine cannot produce a broken boolean — the output is checked and
reported watertight. Inspect side- and cross-sections with the slider, then
Save STL. Voxel 0.3 mm is a good preview; 0.15–0.2 mm for the final (watch the
grid-memory readout — halving voxel size ≈ 8× the RAM).

"Save project" stores everything (orientation, regions, extras, parameters) in
a JSON you can reload later or start from for the next scan of the same frame:
`python app.py myframe.json`.

## Using the output

Subtract `frame_solid.stl` from your donor grip mesh:

- **Blender 4.5+**: Boolean modifier, Difference, solver "Manifold" (both
  meshes watertight — this tool guarantees its half).
- **OpenSCAD** (current release, Manifold backend): a 3-line
  `difference() { import(...); import(...); }` script makes the recipe
  repeatable per grip.
- MeshMixer's boolean will usually cope too, since the hard part (a clean
  subtraction solid) is already done — but the two above are faster and
  stricter.

Clearance guidance: the `clearance` parameter is per-side. ~0.10–0.20 mm for
FDM prints, ~0.05–0.10 mm for CNC hardwood plus finish allowance. Print a
test-fit before cutting wood; edit one number and rebuild.

## Notes & limits

- The rebuild is 2.5D: flat-sided plateau extrusions symmetric about Y=0. Edge
  rounds and chamfers on the real frame become sharp corners — a superset of
  the frame, which for a subtraction solid just means harmless extra internal
  clearance. Genuinely non-2.5D features (angled dovetails, tapered magwell
  mouths) — cover them with an oversized box/cylinder extra.
- Widths are measured from the scan but noisy; override with caliper numbers.
- The silhouette keeps only the largest outer contour: crop turntable junk and
  disconnected debris from the scan beforehand (MeshMixer select+discard).
- Files: `app.py` (GUI), `core.py` (pipeline, importable for scripting),
  `meshio_lite.py` (STL/OBJ/PLY I/O), `make_synthetic.py` +
  `synthetic_frame_scan.stl` (test data), `test_pipeline.py` / `test_gui.py`
  (self-tests; run with `python test_pipeline.py`).
