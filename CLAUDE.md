# frame2solid — project context for Claude

## What this is

A Python GUI tool that converts a 3D scan of a pistol frame into a clean,
watertight "subtraction solid" for grip transplant projects: the user scans a
donor grip (e.g. Glock) and a host frame (e.g. Sako Triace / Pardini target
pistol), and boolean-subtracts the solid this tool produces from the donor
grip exterior to get a grip that fits the host frame. Downstream tools:
MeshMixer (registration, clamshell splitting), Blender/OpenSCAD (the boolean),
VCarve (CNC), FDM printing for fit prototypes.

## Architecture & key decisions

- `core.py` — the pipeline, pure functions, importable headless.
- `app.py` — matplotlib-widget GUI (5 steps via radio buttons). matplotlib
  widgets (NOT tkinter) were chosen so the GUI is testable headless with the
  Agg backend; on Windows it runs on the default TkAgg backend.
- `meshio_lite.py` — self-contained STL/OBJ/PLY read, binary STL + minimal
  R12 DXF write, watertightness check. No trimesh (deliberate).
- Dependencies are ONLY numpy/scipy/scikit-image/opencv-python/matplotlib
  (see requirements.txt). Do not add trimesh/manifold3d/meshlib/shapely
  without discussing — the constraint was plain pip wheels, no mesh stack.
- Geometry engine is a voxel signed-distance field, not mesh booleans:
  - side-view outer silhouette (cv2 RETR_EXTERNAL) auto-fills magwell
    windows/pin holes — this is a core feature, keep it;
  - frame = extrusion of outline with piecewise-constant width W(x,z)
    ("plateau regions", painter's-order rectangles over a measured
    thickness map);
  - extras (clearance solids: tilted boxes for magazine path, Y-cylinders
    for grip screws) are analytic SDFs unioned in (np.minimum);
  - fit clearance = subtract constant from SDF (per-side dilation);
  - skimage marching_cubes at level 0, then clean_mesh() (weld @1e-3 mm,
    drop degenerate/dup faces — REQUIRED for STL float32 round-trip to stay
    watertight; this was a real bug).
- Coordinate convention everywhere: X = bore/length, Y = across frame
  (symmetry plane Y=0, solids symmetric about it), Z = vertical. Side view =
  XZ. Units mm.
- SDF grid memory scales (1/voxel)^3; report includes grid_mem_mb. 0.3 mm
  preview, 0.15–0.2 mm final.

## GUI gotchas (learned the hard way)

- All five steps' widgets occupy the SAME panel coordinates; step switching
  hides other steps' widgets. Hidden matplotlib widgets STILL receive clicks —
  they must also be deactivated. See App._enable(): RadioButtons.set_active(i)
  means "select option i", so those get `w._active = flag` instead. If you add
  a widget to any step, add it to that step's wN list or it will intercept
  clicks on every other step.
- RadioButtons.set_active also fires callbacks — _radio_guard prevents
  recursion when syncing the step radio programmatically.
- Text boxes that edit "the last region/extra" are a deliberate v1
  simplification; a selection model is a known possible upgrade.

## Testing

- `python make_synthetic.py` — regenerates the synthetic frame scan (mock
  frame with magwell window, 3 widths 26/22/12 mm, vertex jitter, 2% dropped
  faces).
- `python test_pipeline.py` — end-to-end core test with assertions (window
  filled, suggested widths ±1 mm, output watertight, plateau widths within
  0.7 mm incl. 2×clearance) + pipeline_preview.png.
- `python test_gui.py` — drives the GUI headless (Agg), screenshots each
  step, asserts build watertight, STL round-trip watertight, project JSON
  round-trip.
- Run both after ANY change to core.py or app.py. There is also a click-
  routing regression concern: clicking "Extract silhouette" in step 2 must NOT
  trigger step 1's Browse dialog (overlapping hidden widgets).

## Known limitations / roadmap candidates

- 2.5D only: plateau extrusions symmetric about Y=0. Non-2.5D features are
  covered by oversized extras. Fine for the subtraction use-case.
- Silhouette keeps only largest outer contour; scan junk must be pre-cropped.
- Possible next features: select/edit individual regions & extras (not just
  last), polygon (non-rectangular) regions, asymmetric widths (Y offset per
  region), extras along arbitrary axes, an "extend region beyond silhouette"
  helper for where the frame exits the grip, per-extra clearance opt-out,
  OpenSCAD difference-script generator for the final grip boolean.
- The user's broader workflow is documented in grip-transplant-workflow.md
  (may be in a parent folder): scan → this tool → registration in MeshMixer →
  boolean (Blender 4.5 Manifold solver or OpenSCAD+Manifold) → wall-thickness
  check → printed fit prototype → clamshell split → 2-sided CNC in VCarve.
