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
  - the solid is TWO-SIDED: bounded by two independent face surfaces
    yL(x,z) <= y <= yR(x,z), built from a stack of THICKNESS TIERS per side
    (see below). Not symmetric about Y=0;
  - extras (clearance solids: tilted boxes for magazine path, Y-cylinders
    for grip screws) are analytic SDFs unioned in (np.minimum); each has a
    `yc` mid-offset so it can sit on one side only;
  - fit clearance = subtract constant from SDF (per-side dilation);
  - skimage marching_cubes at level 0, then clean_mesh() (weld @1e-3 mm,
    drop degenerate/dup faces — REQUIRED for STL float32 round-trip to stay
    watertight; this was a real bug).
- The tier model (core of the tool — read core.py's module docstring):
  - `measure_maps()` measures where the scan's left and right faces sit per
    side-view pixel (hL, hR, positive outward from Y=0). It samples the
    TRIANGLE SURFACES, not just vertices — a coarsely tessellated face
    otherwise leaves holes that split a tier's outline in two;
  - base tier = the whole silhouette at the thickness of the THINNEST part
    of the frame, centred on `base_y0` (which is NOT 0 when the frame is
    asymmetric: step 1 centres on the bbox, which is the wrong plane then,
    and `base_y0` is the only way to fix it);
  - each further tier is picked by clicking a thicker feature. Its outline
    is EVERYTHING on that side standing proud of the tier below it (by more
    than `over_mm`, a scanner-noise margin), wherever it is on the frame —
    so a tier is a superset of the one above and they stack outward (np
    max/min, order-independent). Typically 3-5 tiers per side;
  - `add_mm` (how far proud of the base) is what the user edits; the
    footprint comes from the tier BELOW, never from add_mm, so a caliper
    override cannot make a tier's outline vanish;
  - deliberately rounds thickness UP: a too-thick subtraction solid only
    costs grip wall thickness, a too-thin one means the grip fouls the
    frame. `coverage_report()` measures what the stack still leaves short —
    that is the "have I picked enough tiers?" number;
  - half-pixel bookkeeping is subtle and was tuned against the tests: the
    outline is NOT auto-dilated (sampling bias outward ≈ contour inset), the
    speckle-killing MORPH_OPEN is skipped on the silhouette rim (it would
    nibble a sliver off and leave that rim under-thick), and
    coverage_report dilates the built map by 1 px before comparing.
- Legacy `{"x0","z0","x1","z1","width"}` rectangle regions still build (old
  projects, sample_project.json) — they assign in list order rather than
  stacking. `core.region_kind()` tells them apart.
- Coordinate convention everywhere: X = bore/length, Y = across frame
  (mid-plane near Y=0, but the solid is NOT symmetric about it), Z =
  vertical. Side view = XZ. Units mm.
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
  recursion when syncing the step radio programmatically. TextBox.set_val
  fires on_submit too: App._sync guards every programmatic box update.
- Step 3 has a hand-rolled `ListBox` (matplotlib has no list widget): text
  rows in their own axes, click-to-select via button_press_event, scroll to
  page. It quacks like a widget (.ax, set_active) so _set_step hides and
  deactivates it with everything else.
- The step-3 side radio is the WORKING side (which map is drawn, where the
  next click lands). It is deliberately NOT synced to the selected tier —
  doing that silently dropped "both sides" back to one side after a
  both-sided pick. A tier belongs to one side's stack for good.
- Text boxes in step 4 still edit "the last extra" — a deliberate v1
  simplification. Step 3's tiers have a real selection model now; extras
  are the obvious next candidate for the same treatment.

## Testing

- `python make_synthetic.py` — regenerates the synthetic frame scan (mock
  frame with magwell window, widths 26/22/12 mm, a RIGHT-SIDE-ONLY
  trigger-bar boss making the frame asymmetric, vertex jitter, 2% dropped
  faces).
- `python test_pipeline.py` — end-to-end core test with assertions (window
  filled, tier heights ±0.4 mm, tiers nest, interior holes swallowed,
  coverage clean and correctly flagging a missing tier, output watertight,
  built faces per side within 0.35 mm incl. clearance) + pipeline_preview.png.
- `python test_gui.py` — drives the GUI headless (Agg), screenshots each
  step, asserts tier stacking order, selection/edit/delete of any tier (not
  just the last), the asymmetric built faces, build watertight, STL
  round-trip, project JSON round-trip, and that legacy rectangle projects
  still build.
- Run both after ANY change to core.py or app.py. There is also a click-
  routing regression concern: clicking "Extract silhouette" in step 2 must NOT
  trigger step 1's Browse dialog (overlapping hidden widgets).

## Known limitations / roadmap candidates

- 2.5D per side: each tier is a flat extrusion, so draft/rounds/chamfers
  become steps. Genuinely non-2.5D features are covered by oversized extras.
  Fine for the subtraction use-case (it errs thick).
- Tier outlines are clipped to the silhouette; a tier cannot extend past the
  frame profile even after grow_mm. Use an extra for that.
- Silhouette keeps only largest outer contour; scan junk must be pre-cropped.
- Possible next features: select/edit individual extras (tiers have this
  now), extras along arbitrary axes, an "extend beyond silhouette" helper
  for where the frame exits the grip, per-extra clearance opt-out, a
  coverage heat-map overlay in step 3, OpenSCAD difference-script generator
  for the final grip boolean.
- The user's broader workflow is documented in grip-transplant-workflow.md
  (may be in a parent folder): scan → this tool → registration in MeshMixer →
  boolean (Blender 4.5 Manifold solver or OpenSCAD+Manifold) → wall-thickness
  check → printed fit prototype → clamshell split → 2-sided CNC in VCarve.
