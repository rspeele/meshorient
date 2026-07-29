# frame2solid

Turns a 3D scan of a pistol frame into a clean, watertight **subtraction
solid** — the thing you boolean-subtract from a donor grip exterior to make a
transplanted grip. Fills magwell windows and pin holes automatically, rebuilds
the frame as a stack of flat **thickness tiers, independently per side** (so a
trigger-bar relief on the right doesn't fatten the left), lets you add
clearance solids (magazine path, grip screws, levers), applies a fit-clearance
offset, and exports a guaranteed-watertight STL.

## Install (Windows)

1. Install Python 3.10+ from https://python.org (check "Add to PATH").
2. In a command prompt, in this folder:

       pip install -r requirements.txt
       python app.py

The GUI needs no other software. Try it first with the included
`synthetic_frame_scan.stl` (a mock frame with a magwell window, three
thicknesses and a right-side-only boss) — the default scan path points at it.

## Coordinate convention

After the Orient step your scan must sit like this (all mm):

- **X** = along the bore / frame length
- **Y** = across the frame (left-right); mid-plane near **Y = 0** (it needn't
  be exact — step 2 squares the scan up and step 4's `y0` measures where it
  actually is)
- **Z** = vertical

The side view (X-Z) is what becomes the outline. If you've already aligned the
scan in MeshMixer/Blender, export it that way and skip PCA; otherwise
"Auto-orient (PCA)" plus the 90° buttons gets you there in a few clicks.

## The six steps

**1 · Load/Orient.** Load STL/OBJ/PLY. Three projection views update as you
rotate. You're done when the SIDE view shows the classic frame profile, the
FRONT view looks thin, and the TOP view is symmetric about Y=0. "re-center"
puts the bounding-box center at the origin (it runs automatically after each
rotation).

**2 · Level.** PCA gets you close but leaves a fraction of a degree of tilt,
and that's enough to make the trigger guard read thicker than the backstrap
even when they're identical. The view here is the **height of the scan's
right-hand surface**, so residual tilt shows up as a gradient sliding across
a face you know is flat.

Click **3 or more points on one face that really is flat** — spread them
out, the corners of the frame's right side are ideal. Each pick reports the
Y it found, and once you have three the title tells you the tilt angle and
how coplanar your picks actually were (if that number is large, one of them
isn't on the flat). **LEVEL** applies the smallest rotation that squares
that face up to Y; it only removes tilt, it never spins the frame about Y.
Level again to confirm it now reads 0.00°. "Undo level" puts the
orientation back.

**3 · Silhouette.** Extracts the side-view **outer** contour. Interior
windows, pin holes, and lightening cuts vanish automatically — only the
outermost outline is kept, which is exactly the "fill it solid" behavior you
want in a subtraction tool. Parameters: `px` raster resolution (0.15 mm is
fine), `close` closes gaps up to this size (scanner dropouts), `simp` outline
simplification tolerance. "Export DXF" writes the outline for tracing in
SolveSpace if you'd rather rebuild in CAD.

**4 · Tiers.** The real frame's bumpy, continuously varying thickness gets
simplified into a handful of flat **thickness tiers per side** — typically
3–5. You get **two views, one per side**: each shows where that side's
surface sits (so bosses and reliefs stand out) and only that side's tiers.
The left view is mirrored, so it's the frame as seen standing on its left —
not the right-hand view with the far side showing through.

- **Pick base**, then click the *thinnest* part of the frame (usually the
  magazine housing). That's the base tier: the whole silhouette, extruded to
  that thickness. `y0` is the mid-plane it's centred on, measured at your
  click — this matters, because an asymmetric frame's true mid-plane is not
  where step 1's bounding-box centring put it.
- Then **click each thicker feature, in whichever view's side it's on**. A
  tier's outline is *everything on that side standing proud of the tier
  below it*, anywhere on the frame — so one click on the frame body ropes in
  every trigger-bar relief, spring channel and boss above the base at once,
  and extrudes them all to the thickness you clicked. Click a fatter area
  for the next tier up, and so on.
- `+mm` is how far proud of the base that tier stands (prefilled from the
  scan, override with calipers — the outline doesn't move when you do).
  `over` is how much proud of the tier below a feature must be to get roped
  in: a noise margin, 0.3 mm suits most scanners. `grow` Minkowski-grows the
  outline for parts that have to *move* — a trigger bar needs room fore, aft
  and up, not just clearance on its face.
- Edits to those boxes (and to `base T` / `y0`) commit when you press
  **APPLY** or Enter, and never before — so you can tab between boxes
  without paying for a re-cut each time, and you can force one without
  having to click somewhere else. A badge says `applying…` while it works;
  what's on screen until it clears is the old view.
- Every tier is listed at the right; click a row (or right-click a view) to
  select one and edit or delete it. Tiers keep themselves in stacking order.
- The status line reports what the stack still leaves **under-thick**, with
  the worst spot's coordinates. Zero means the tiers cover the whole scan;
  anything significant means you need another tier there.

Thickness always rounds **up**: a feature only 0.5 mm proud of a tier gets
pulled up to the next one. Too thick only costs grip wall thickness, too thin
means the grip fouls the frame.

**5 · Extras.** Clearance solids beyond the frame itself, in the same
coordinates. **Drag** to add a box (set its across-width, tilt and `y-mid`
first — e.g. a magazine insertion path angled with the grip, drawn from above
the magwell down past where any grip could reach). **Click** to add a Y-axis
cylinder (grip screw holes, pin punches — set diameter/length first). `y-mid`
offsets an extra off the centreline, for reliefs that belong on one side only.
The text boxes edit the **last** extra added. Everything here gets unioned
with the frame before subtraction, so nothing you build later can block a
magazine or bury a screw.

**6 · Build & export.** Two ways out, from the same tier model:

**Export CAD (DXF)** is the one to reach for. The tier model *is* a
sketch-and-extrude model, so it exports losslessly: one DXF per sketch (base
outline, each tier, each extra) into `<out>_cad/`, plus `all_sketches.dxf`
with one layer per sketch if you'd rather import once. `build.txt` is the
build sheet — the exact Y range and depth for every sketch. Every tier
extrudes from the *same* plane (`y0`), so in SolveSpace they share one
workplane and differ only in depth and direction. Import, tweak the outlines
by hand — which is the point — extrude, union. `assembly.scad` does the whole
thing already if you'd rather drive it from OpenSCAD, and includes the
`difference()` against your donor grip, commented out. The `clearance` value
is grown into the outlines and depths; set it to 0 for nominal geometry.
This needs no build — it works straight off step 4.

**BUILD solid** is the voxel signed-distance-field path: outline extruded to
the tier stack on each side, extras unioned, everything dilated by
`clearance`, then meshed. Use it as a preview (the section slider) and for a
quick printable STL; it can't produce a broken boolean — the output is
checked and reported watertight — but it's a mesh, not CAD. Inspect side- and
cross-sections with the slider, then Save STL. Voxel 0.3 mm is a good preview;
0.15–0.2 mm for the final (watch the grid-memory readout — halving voxel size
≈ 8× the RAM).

"Save project" stores everything (orientation, regions, extras, parameters) in
a JSON you can reload later or start from for the next scan of the same frame:
`python app.py myframe.json`.

## Using the output

**Via CAD (preferred).** Take `<out>_cad/` into SolveSpace, adjust the
outlines where the scan was ropey, extrude per `build.txt`, and you have a
real parametric model you can revise later. Or open `assembly.scad` in
OpenSCAD (Manifold backend), uncomment the `difference()` at the bottom and
point it at your donor grip — exact geometry, no voxels, seconds to run.

**Via mesh.** Subtract `frame_solid.stl` from your donor grip mesh:

- **Blender 4.5+**: Boolean modifier, Difference, solver "Manifold" (both
  meshes watertight — this tool guarantees its half).
- MeshMixer's boolean will usually cope too, since the hard part (a clean
  subtraction solid) is already done — but Blender is faster and stricter.

Clearance guidance: the `clearance` parameter is per-side. ~0.10–0.20 mm for
FDM prints, ~0.05–0.10 mm for CNC hardwood plus finish allowance. Print a
test-fit before cutting wood; edit one number and rebuild.

## Notes & limits

- The rebuild is 2.5D per side: each tier is a flat extrusion, and the two
  sides are independent (the solid is not symmetric about Y=0). Edge rounds
  and chamfers on the real frame become steps — a superset of the frame,
  which for a subtraction solid just means harmless extra internal clearance.
  Genuinely non-2.5D features (angled dovetails, tapered magwell mouths) —
  cover them with an oversized box/cylinder extra.
- A tier's outline is clipped to the silhouette, so `grow` cannot push it
  past the frame profile; use an extra where the frame exits the grip.
- Thicknesses are measured from the scan but noisy; override with caliper
  numbers. The outline a tier covers never depends on that override.
- The silhouette keeps only the largest outer contour: crop turntable junk and
  disconnected debris from the scan beforehand (MeshMixer select+discard).
- Files: `app.py` (GUI), `core.py` (pipeline, importable for scripting),
  `meshio_lite.py` (STL/OBJ/PLY I/O), `make_synthetic.py` +
  `synthetic_frame_scan.stl` (test data), `test_pipeline.py` / `test_gui.py`
  (self-tests; run with `python test_pipeline.py`).
