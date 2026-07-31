# frame2solid — project context for Claude

## What this is

A Python GUI tool that converts a 3D scan of a pistol frame into a clean,
"subtraction solid" for grip transplant projects: the user scans a
donor grip (e.g. Glock) and a host frame (e.g. Sako Triace / Pardini target
pistol), and boolean-subtracts the solid this tool produces from the donor
grip exterior to get a grip that fits the host frame. Downstream tools:
MeshMixer (registration, clamshell splitting), OpenSCAD (the boolean and
the STL), SolveSpace (editing the exported profiles), VCarve (CNC), FDM
printing for fit prototypes.

## What the user actually uses it for (as of 2026-07-31)

Their scanning workflow now usually yields a manifold scan first try, and
they are fluent enough in Blender to do clearances and corner clean-up with
cubes on the raw scan. So the tool's day-to-day job has narrowed to:

  1. scan, make watertight (MeshMixer if needed);
  2. **open in frame2solid, PCA-orient, level in step 2, re-export the
     oriented STL** — this is the part no other tool of theirs does as well,
     and it is the reason the tool is still in the workflow;
  3. clean up / add clearances in Blender;
  4. subtract the frame from the scanned grip mesh in Blender.

Steps 1-2 are therefore the load-bearing feature. Tiers/silhouette/CAD
export stay because the DXFs are the right thing for VCarve and SolveSpace
and the feature is well developed — but do not assume the user is exercising
them on every project. The **Extras** step (tilted boxes for the magazine
path, Y-cylinders for grip screws) was REMOVED on 2026-07-31: it never
earned its keep, and a cube in Blender does the same job better. Do not
reintroduce it. Old project JSONs still load; their `extras` key is ignored
and `_on_proj_load` says so in the status line.

## Architecture & key decisions

- `core.py` — the pipeline, pure functions, importable headless.
- `app.py` — matplotlib-widget GUI (5 steps via radio buttons; step indices
  are the S_* constants, never bare numbers). matplotlib
  widgets (NOT tkinter) were chosen so the GUI is testable headless with the
  Agg backend; on Windows it runs on the default TkAgg backend.
- `meshio_lite.py` — self-contained STL/OBJ/PLY read, binary STL + R12 DXF
  write. No trimesh (deliberate). The DXF writer emits ONLY `LINE` and
  `CIRCLE`: closed outlines go out as rings of segments. This is not an
  aesthetic choice — OpenSCAD rejects R12 POLYLINE/VERTEX/SEQEND outright
  ("Unsupported DXF Entity", confirmed by the user), and LWPOLYLINE would
  mean an R13+ file with entity handles. OpenSCAD restitches coincident
  endpoints into closed paths and SolveSpace explodes polylines into
  segments on import, so neither loses anything. Do not "tidy" this back
  into polylines. save_stl survives for make_synthetic.py and for the Level
  step's oriented-scan re-export — the tool's own output is DXF.
- Dependencies are ONLY numpy/opencv-python/matplotlib (see
  requirements.txt). Do not add trimesh/manifold3d/meshlib/shapely without
  discussing — the constraint was plain pip wheels, no mesh stack. scipy and
  scikit-image went out with the voxel build; keep it that way.
- The export re-cuts the tiers on a finer raster than step 4 uses
  (`App._refined_regions`, `export px`, default 0.2 mm vs the 0.5 mm working
  map). It works on COPIES so the interactive state is untouched, and if a
  tier's area moves by more than it should it keeps its step-4 outline and says
  so — refining is meant to move edges, not change what is included.
  The allowance is `max(5%, core.raster_bias_bound(...))`, NOT a flat 5%.
  Conservative rasterisation puts half a pixel of outward bias on every
  outline, worth `perimeter * px / 2` of area, and shedding it is the finer
  measurement doing its job. Because that bias scales with PERIMETER while the
  guard compares AREA, it is proportionally huge for a small island with a long
  boundary: a 30 mm-proud rod end (91.8 mm², 77.5 mm of perimeter) legitimately
  loses 14.1% between 0.5 and 0.2 mm/px. A flat 5% guard reverted it to the
  coarse outline, whose islands were too crude for `fit_circle` — so the same
  rod, one body passed straight through the frame, exported as a true circle on
  one side and a faceted polygon on the other (user-reported, via OpenSCAD).
  A wholesale collapse still trips the guard: the bound is a couple of percent
  for a large footprint, nowhere near the 40% a broken `_fill_gaps` once cost.
  test_pipeline pins both ends of that with the real measured numbers, and
  test_gui asserts no tier gets reverted on the synthetic.
- Round tier islands are exported as true DXF `CIRCLE` entities, not polygons
  (`fit_circle`, `_split_circles`, `circle_tol` on `sketch_model`, "circle fit
  mm" in the Export step, 0 = off). A 3.5 mm screw clearance measured on a 0.2 mm
  raster only supports ~12 vertices — that is the information limit, so no
  amount of tolerance tuning makes a polygon rounder. Detection details that
  matter: sample the boundary at vertices AND EDGE MIDPOINTS (every regular
  polygon has its vertices exactly on a circle, so a vertex-only test calls a
  square a circle); take the radius from the AREA so the swap preserves the
  footprint; and use BOTH an absolute tolerance on the raster's scale (real
  polygonised discs sit half a pixel off — 0.258 mm at px=0.5, 0.139 mm at
  px=0.2) and a relative one (`_CIRCLE_REL_TOL`) to stop a small square boss
  being read as a disc. `min_pts` is 8 because deviation alone cannot separate
  a hexagon (10.0% of r) from an octagon (5.4%). Detection runs BEFORE the
  clearance offset so circles grow as `r + c`, exactly, instead of being
  re-rasterised by `_grow_loops` into a fresh set of facets. A tier and the
  tier below it both cover a boss, so the same disc is emitted twice — that is
  correct, and the two agree to 0.02 mm (as polygons they once differed by
  0.967 mm). NOT field-verified through OpenSCAD/SolveSpace yet, like the rest
  of the DXF output; the box exists so it can be switched off.
- The output is CAD, not mesh. `sketch_model()` + `export_cad()`: the tier
  model IS a sketch-and-extrude model, so it exports losslessly as one DXF
  per sketch (base outline, each tier) + `all_sketches.dxf` (one
  layer each) + `build.txt` (the extrusion table) + `assembly.scad`
  (OpenSCAD rebuild, exact geometry, with the donor-grip `difference()`
  commented in). Every tier extrudes from the same plane y=base_y0, so CAD
  needs one workplane. `clearance` is grown into the outlines (rasterised
  Minkowski) and the depths.
  There WAS a voxel SDF build (build_solid + marching cubes -> STL). It was
  removed once the CAD export was exact: OpenSCAD renders assembly.scad to
  an STL far better than a voxel grid could, and the DXFs are what
  SolveSpace and VCarve want. Do not bring it back — if you need a preview,
  extrude the sketches, do not re-raster the model.
- The silhouette (cv2 RETR_EXTERNAL) auto-fills magwell windows/pin holes —
  this is a core feature, keep it.
- NEVER hand more than one contour to a single `cv2.fillPoly` call. It
  applies the EVEN-ODD rule across them, so overlapping polygons cancel
  instead of merging. `extract_silhouette` used to pass the whole triangle
  array in one call, which erased every pixel covered an even number of
  times — i.e. most of them, since a closed surface projects a front and a
  back triangle onto the same pixel. A plain cube raised "Silhouette too
  small"; on a real 760k-tri scan the outline silently lost 21 mm in X and
  50 mm in Z (user-reported: geometry they had added in Blender vanished).
  Fill one polygon per call via `core._fill_union`, which is also FASTER
  than the broken single call (0.86 s vs 1.07 s on that scan, because it
  fills far less area twice). test_pipeline's cube case guards this.
- The solid is TWO-SIDED: bounded by two independent face surfaces
  yL(x,z) <= y <= yR(x,z), built from a stack of THICKNESS TIERS per side
  (see below). Not symmetric about Y=0.
- The Level step (2) exists because PCA leaves a few tenths of a degree of
  tilt, and that alone makes one end of a flat side measure thicker than the
  other — which the two-sided tier model then bakes in. The user clicks 3+
  points on a face that really is flat; `core.level_rotation()` fits a plane
  (SVD) and returns the SMALLEST rotation squaring it to Y (Rodrigues about
  n x y), so it removes tilt without spinning the frame about Y. Its view is
  the right-face height map, where a tilt reads as a gradient, plus the
  front (Y-Z) and top (X-Y) views borrowed from step 1 — the two components
  of a tilt are visible one per view, each titled with its own angle, with
  the fitted plane's trace in red against a blue Y=0 datum. Picks are
  world-space, so any re-orientation in step 1 clears them; levelling itself
  carries them into the new frame so you can level twice and see 0.00°.
  Step 2 re-positions ax_main/ax_front/ax_top (POS_L_* vs POS_O_*) rather
  than owning duplicate axes.
  Step 2 also re-exports the scan (`_on_export_oriented` -> save_stl of
  self.verts + self.orig_faces, `<scan>_oriented.stl`): a rigid transform,
  no resampling, so it is useful for the user's Blender/MeshMixer work even
  when they are not building a solid. This is the only remaining caller of
  save_stl outside make_synthetic.py.
- The tier model (core of the tool — read core.py's module docstring):
  - `measure_maps()` measures where the scan's left and right faces sit per
    side-view pixel (hL, hR, positive outward from Y=0). It RASTERISES the
    triangles (`_raster_minmax`, scan conversion with Y interpolated at each
    pixel), the same way `extract_silhouette` rasterises with fillPoly. That
    is why the two now produce outlines of the same quality, which they did
    not before. Do NOT go back to scattering sample points over the
    triangles: random placement is Poisson, so it left 26% of in-outline
    pixels empty at px=0.5 and 52% at px=0.1 — the finer the raster, the
    emptier — and the grey-close that patched those holes is what put a
    ~2.5 mm staircase on every diagonal tier boundary (user-reported, via
    OpenSCAD, on a feature that was a plain rectangular prism). Rasterising
    costs the same order (O(covered pixels)) and is exact. On the user's
    761 k-triangle scan it is ~1.3 s at px=0.5 against 0.4 s for the old
    sampler — worth it, and the busy badge covers it;
  - rasterisation is CONSERVATIVE: a pixel takes any triangle overlapping its
    square, not just one covering its centre (exact test — Minkowski sum of
    triangle and pixel square = the three edge half-planes offset by each
    edge's support). Features therefore come out ~half a pixel oversized per
    side, which is the errs-thick direction and is also the bias the tier code
    was tuned against. Testing centres alone put the synthetic's 4 mm diagonal
    rib 7.7% UNDER area — i.e. too thin, the dangerous direction. The excess
    must shrink as px shrinks; test_pipeline asserts both;
  - `_raster_minmax` buckets triangles by bounding-box size and rasterises
    each bucket as one array. The window must be the bucket's LARGEST member —
    sizing it from the bucket's lower bound silently truncated every triangle
    above that bound, and took 8 mm off the end of the synthetic's rib, whose
    outer face is two 39 mm triangles. test_pipeline rasterises one big
    diagonal triangle of known area to pin this down;
  - measuring the SAME scan at a different px must give the same model with
    better-resolved edges — that invariant is what lets the export re-cut
    finer than step 4 works at. Everything in `_fill_gaps` and
    `segment_tier` is therefore sized in MILLIMETRES and divided by px.
    `_fill_gaps` used fixed pixel kernels, so it bridged 2.5 mm of dropout at
    px=0.5 but 0.75 mm at px=0.15: coverage of the profile fell 81% -> 61% on
    a real scan and every tier shrank, so a finer raster made the output
    WORSE (39 fragmented islands instead of 5). Denser sampling does not fix
    that — it is not a sampling problem. test_pipeline asserts tier areas
    hold within 6% across resolutions. `_fill_gaps`' reach also scales with
    the SCAN'S TESSELLATION (`_tess_mm`, median edge length): a dropped face
    punches a hole one triangle wide, which a coarse raster cannot resolve
    and a fine one can. Its kernels are ELLIPSE, never square — a square
    structuring element quantises a diagonal edge to its own size, and these
    kernels are millimetres across. And it grey-closes with the invalid
    pixels MASKED (`_grey_close_masked`): filling them with the map's minimum
    and closing over the lot lets the erosion half drag that minimum a full
    kernel radius INTO valid data, so every dropout and the whole profile rim
    came back short. Measured on the real scan, finer is now genuinely
    better — 86.8% of the tier outlines are straight to 0.1 mm at px=0.2
    against 37.9% before, from 354 points instead of 3208;
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
    outline is NOT auto-dilated (the raster's deliberate outward half-pixel ≈
    contour inset), the speckle-killing MORPH_OPEN is skipped on the
    silhouette rim (it would nibble a sliver off and leave that rim
    under-thick), the `spill` that lets a tier reach past the coarse
    `sil_mask` dilates by ONE pixel not two (two also spreads the mask
    TANGENTIALLY along the profile, which put a tier a pixel past the end of
    its own footprint and 0.45 mm proud of the tier below — and the snap
    cannot repair that, because the vertex is already exactly on the
    silhouette and snapping takes the NEAREST reference), and
    `coverage_report` dilates the built map by 1 px and judges only pixels
    two px inside the profile. That last one matters: the map reaches up to
    two pixels past the profile (one from grid padding, half from conservative
    rasterisation) and a tier is clipped to the profile, so that rim can never
    be covered — left in, it reported the rail's 13 mm face as a 7 mm hole;
  - a tier is cut on the coarse map grid but clipped to the profile on the
    SILHOUETTE's grid (`_clip_to_silhouette`). Inside that: SNAP to
    `sil.polygon` first (vertices within 0.35 mm), THEN simplify, then snap
    again. The order is load-bearing — snapped runs are collinear, so
    simplifying can only drop redundant points from them, never pull them off
    the profile. That frees the tolerance to be `2 * maps.px` (the map pixel
    is what limits the boundary's accuracy, since it is a thickness step
    measured on that grid). Simplifying first forced a tolerance fine enough
    to protect the profile, which then faithfully traced the map's 0.5 mm
    staircase: a straight block edge came out visibly jagged in OpenSCAD
    (user-reported). Interpolating the height map does NOT fix that — the
    boundary is a step edge and max-per-pixel sampling pins it to ±1 pixel
    however finely you resample; the answer is to not imply more precision
    than the measurement has. test_pipeline feeds a synthetic 0.5 mm
    staircase through and asserts it comes out as 2 points. Clipping on the
    map grid instead left every tier ~0.7 mm short of the profile — a ledge
    in the built solid that grow_mm could never close, because growing then
    clipped back to the same inset boundary (user-reported, via OpenSCAD).
    The snap references are the silhouette AND the tier BELOW (`_prev_tier`),
    because a tall feature is built as several stacked extrusions and their
    shared wall has to be one wall — otherwise the sub-millimetre
    disagreements terrace it (user-reported). `snap_tol` is `2.5 * maps.px`:
    each contour's position is known to about a pixel, so two contours can
    differ by a couple of pixels from noise alone, while a genuine difference
    (a ramped surface, where terracing IS the right answer) is far larger.
    `segment_all` therefore runs in stacking order.
  - the simplify tolerance is chosen PER ISLAND as the coarsest of
    (cap, cap/2, cap/4, cap/8, 0.10 mm) that keeps the island's area within
    `_SIMPLIFY_AREA_TOL` (0.5%), so simplification can never quietly shrink a
    feature. One fixed number cannot serve a 100 mm straight edge (needs ~1 mm
    or it traces the staircase), a 3 mm screw-clearance circle (1 mm makes it
    a trapezoid) and a 2 mm rib (a size-based rule flattens it). A 32-point
    circle survives as 11 points, area within 2.2%. 0.02 is measured, not
    guessed: 0.05 drops a 3.4 mm screw-clearance circle from 12 points to 8,
    and 0.01 blows a 4 mm-wide straight-sided rib up from 5 points to 30.
  - `segment_tier` barely simplifies its own contour before handing it to
    `_clip_to_silhouette` — HALF a pixel. It used to use `simplify_mm / px`
    (1.5 px at the export raster), and the careful per-island pass downstream
    can only drop points, never restore ones already thrown away: that is what
    made screw-clearance cylinders octagons ("cylinders get rather brutally
    simplified to trapezoids"). Loosening the area tolerance was NOT the fix
    and made it worse. The contour is already run-length collapsed by
    CHAIN_APPROX_SIMPLE, so keeping it costs almost nothing.
  - the PROFILE's corners are pinned into a tier's boundary before simplifying
    (`_pin_corners`), and each run BETWEEN pinned points is simplified
    separately (`_simplify_protected`, approxPolyDP with closed=False so the
    ends survive). approxPolyDP is otherwise free to cut across a corner — it
    only has to stay within eps of the contour — so the base tier and a tier
    stacked on it rounded the same profile corner differently and their walls
    stopped matching (0.72 mm apart). Pin the silhouette ONLY, not the tier
    below: pinning the tier below propagates its vertex pattern up the stack,
    and a small round island it happened to describe as an octagon then forced
    every tier above it to be an octagon too.
    For the same reason `_grow_loops` snaps its raster to a shared lattice:
    the base and a tier that meets it must come out of the clearance offset
    with the same edge.
- Legacy `{"x0","z0","x1","z1","width"}` rectangle regions still build (old
  projects, sample_project.json) — they assign in list order rather than
  stacking. `core.region_kind()` tells them apart.
- Coordinate convention everywhere: X = bore/length, Y = across frame
  (mid-plane near Y=0, but the solid is NOT symmetric about it), Z =
  vertical. Side view = XZ. Units mm.

## GUI gotchas (learned the hard way)

- All five steps' widgets occupy the SAME panel coordinates; step switching
  hides other steps' widgets. Hidden matplotlib widgets STILL receive clicks —
  they must also be deactivated. See App._enable(): RadioButtons.set_active(i)
  means "select option i", so those get `w._active = flag` instead. If you add
  a widget to any step, add it to that step's wN list or it will intercept
  clicks on every other step.
- Any message that points the user at another step must build the reference
  with `app.step_ref(S_*)` ("step 3 (Silhouette)"), never a hand-written
  number. The steps have been renumbered twice and stale references only
  surface when a user hits that specific error. test_gui asserts app.py
  contains no literal "(step ".
- `app._patch_matplotlib_textbox()` fixes two matplotlib TextBox problems
  at import (checked against 3.11.0); both are covered by test_gui, which
  fires real MouseEvent/KeyEvent/ResizeEvent objects:
  - `_resize` is connected to 'resize_event' but decorated with the
    mouse-event reparenting wrapper, which reads event.inaxes. ResizeEvent
    has none, so every window resize printed one AttributeError traceback
    per text box. Restores the undecorated method.
  - TextBox repaints via canvas.draw() — a FULL figure render, ~200 ms
    here. matplotlib calls stop_typing() on every box that was NOT the one
    clicked, so one click into a text box cost 7 full renders (~1.5 s
    before the cursor appeared) plus one per keystroke. Button already
    blits its repaint (`useblit=True`); TextBox never got the same
    treatment. The patch makes stop_typing return early when there is
    nothing to stop, and routes _motion/_rendercursor through
    `_repaint_widget_only`, which redirects canvas.draw() to a blit of that
    widget's own axes. Measured after: 0 full renders for a click, 20 ms.
  - If you add a widget-heavy step, keep an eye on this: the whole figure
    is ~240 text artists and text is 60% of a render.
- App() must NOT use `plt.figure("frame2solid")`. A string `num` is a
  figure LABEL, so a second App is handed the first one's figure and piles
  its ~50 axes onto that canvas — both apps' widgets then answer every
  click (this bit the tests, which build three Apps). The name goes on the
  window via canvas.manager.set_window_title().
- RadioButtons.set_active also fires callbacks — _radio_guard prevents
  recursion when syncing the step radio programmatically. TextBox.set_val
  fires on_submit too: App._sync guards every programmatic box update.
- The Tiers step has a hand-rolled `ListBox` (matplotlib has no list widget): text
  rows in their own axes, click-to-select via button_press_event, scroll to
  page. It quacks like a widget (.ax, set_active) so _set_step hides and
  deactivates it with everything else.
- The Tiers step has TWO axes (self.tier_axes), one per side, instead of one map
  with a side selector. Each renders only its own side's tiers and a click
  lands on the side you clicked — there is no "working side" state and no
  "both" option (the user rejected both; a tier belongs to one side for
  good). ax_tier_L is X-inverted so it reads as the frame seen from its
  left, not the right-hand view with the far side showing through.
- The Tiers step's text boxes have NO on_submit. matplotlib fires that on focus
  loss as well as Enter, so tabbing between boxes used to trigger a re-cut
  each time — and there was no way to force one without leaving the box.
  Edits commit only via App._apply_edits (the APPLY button, or Enter via
  App._on_key). Keep it that way if you add boxes to the Tiers step.
- Slow operations wrap in `with self._busy(...)`: it drops a badge on the
  figure and forces a synchronous draw+flush first, so the user sees "this
  view is stale, a new one is coming". The stale view underneath is
  deliberate.

## Testing

- `python make_synthetic.py` — regenerates the synthetic frame scan (mock
  frame with magwell window, widths 26/22/12 mm, a RIGHT-SIDE-ONLY
  trigger-bar boss, vertex jitter, 2% dropped faces) plus two deliberately
  NON-AXIS-ALIGNED features: a diagonal rib across the right grip wall and a
  round boss on the left. Rectangles cannot catch staircasing of a diagonal
  tier boundary or over-simplification of a curved one, which is where every
  artefact in this area has actually shown up — a fixed synthetic of boxes
  passed happily while a real scan came out jagged. `add_prism()` extrudes an
  arbitrary XZ polygon, so add awkward shapes rather than more boxes.
  The two bosses are DIFFERENT heights on purpose (right +14.0, left -14.5)
  so the Y bounding-box centre is not the frame's mid-plane and `base_y0` has
  work to do; keep it that way.
  Probe points in the tests have to dodge all of these — several assertions
  broke when the rib landed near a "plain grip wall" reference point.
- `python test_pipeline.py` — end-to-end core test with assertions (window
  filled, tier heights ±0.4 mm, tiers nest, interior holes swallowed,
  coverage clean and correctly flagging a missing tier, a tier sharing the
  frame's edge exactly where it reaches it, the exported sketches giving the
  right per-side faces within 0.35 mm incl. clearance, and the DXFs reading
  back as the same closed loops using only LINE/CIRCLE) + pipeline_preview.png.
- `python test_gui.py` — drives the GUI headless (Agg), screenshots each
  step, asserts levelling recovers a deliberately introduced 1.3° tilt
  (and Undo puts it back), tier stacking order, per-side views drawing only
  their own tiers, selection/edit/delete of any tier (not just the last),
  that edits do nothing until Apply, that the busy badge is up *during* the
  re-cut, the CAD export writing every sketch without needing anything
  else, project JSON round-trip, and that legacy rectangle projects still
  export — including sample_project.json, whose two now-obsolete `extras`
  must be dropped AND reported in the status line, not silently ignored.
- Run both after ANY change to core.py or app.py. There is also a click-
  routing regression concern: clicking "Extract silhouette" in step 3 must NOT
  trigger step 1's Browse dialog (overlapping hidden widgets).

## Known limitations / roadmap candidates

- 2.5D per side: each tier is a flat extrusion, so draft/rounds/chamfers
  become steps. Genuinely non-2.5D features (angled dovetails, tapered
  magwell mouths) are a cube in Blender now, subtracted from the donor grip
  alongside this solid. Fine for the subtraction use-case (it errs thick).
- Tier outlines are clipped to the silhouette; a tier cannot extend past the
  frame profile even after grow_mm. Extend it in Blender.
- Silhouette keeps only largest outer contour; scan junk must be pre-cropped.
- The DXF export is verified here only against the format spec and the
  round-trip reader in test_pipeline.py (no SolveSpace/OpenSCAD available).
  What IS confirmed from the field: OpenSCAD rejected the earlier POLYLINE
  output ("Unsupported DXF Entity"). The LINE/CIRCLE replacement has not yet
  been run through OpenSCAD or SolveSpace. If an importer complains, entity
  types are the first place to look — test_pipeline asserts nothing but
  LINE/CIRCLE is emitted.
- Possible next features: a coverage heat-map overlay in the Tiers step,
  ARCS in the DXF (full circles are done — fillets and rounded slot ends are
  the obvious next case, and R12 ARC is as portable as CIRCLE). Anything
  that makes steps 1-2 faster or more certain is worth more than anything
  that adds to steps 3-5 — see "What the user actually uses it for".
- The user's broader workflow is documented in grip-transplant-workflow.md
  (may be in a parent folder): scan → this tool → registration in MeshMixer →
  boolean (Blender 4.5 Manifold solver or OpenSCAD+Manifold) → wall-thickness
  check → printed fit prototype → clamshell split → 2-sided CNC in VCarve.
