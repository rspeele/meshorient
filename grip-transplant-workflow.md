# Grip Transplant Workflow

**Putting the external shape of one pistol's grip around the frame of another**

*Example used throughout: donor = Glock grip exterior, host = Sako Triace / Pardini-style frame nub. The workflow is symmetric to any donor/host pair where the host frame fits inside the donor envelope.*

---

## 1. Architecture: three solids, one boolean

Everything downstream gets easier if you commit to this structure:

1. **Donor exterior** — a scan-derived mesh of the outside of the donor grip. Stays a mesh forever. Never gets converted to CAD. Organic shapes belong in mesh space.
2. **Host subtraction solid** — a *rebuilt*, clean, deliberately-oversimplified solid representing the host frame **plus** every functional clearance volume (magazine path, controls, screws). This is the reusable asset: build it once per frame, use it against every donor shape you ever scan.
3. **Result** — donor exterior minus host subtraction solid, computed in a robust boolean engine, then split/prepared for manufacturing.

The cardinal rule: **never boolean raw scan against raw scan.** One input (the frame) gets rebuilt into a clean solid; the other (the grip) gets watertighted but stays organic; and the boolean itself runs in an engine that tolerates mesh input (Manifold-based or voxel-based — Section 7).

Why the frame is the one that gets rebuilt: frames are prismatic (flat-sided, 2–3 thicknesses, simple side profile), so rebuilding is cheap and gives you exact dimensions, filled windows, and parametric clearance. Grips are organic, so rebuilding them in CAD would be miserable and pointless.

---

## 2. Scanning and cleanup

### Donor grip
- Scan the assembled pistol or bare frame — you only need the **exterior** surface of the gripping area. Don't fight to capture undercuts inside the trigger guard or magwell; you'll cut those regions off anyway.
- Decide where the donor envelope *ends*: typically you keep the wrap from just below the slide/upper down through the heel, and cut it off with a plane or box at the top (where the host's upper takes over) and possibly the front (trigger guard region, which belongs to the host).
- Cleanup target: **watertight, manifold, single shell**. The fastest route is a voxel remesh (Blender: Remesh modifier, Voxel mode, size 0.2–0.4 mm) — it eats holes, self-intersections, and scanner junk in one step at the cost of slightly softening detail. If you need to preserve texture (stippling), repair instead of remeshing: MeshLib's healing tools or Blender's 3D-Print toolbox → Make Manifold.

### Host frame
- Scan the **bare frame**, stripped of grips and ideally with controls removed or noted.
- This scan is only a *reference* for reconstruction — it never touches the final boolean — so noise tolerance is high. What matters is that the side profile and the few thickness values are recoverable.
- **Caliper the thicknesses.** For a 2.5D rebuild you need maybe 3–6 width numbers (frame body, rails, trigger guard root, any bosses). A $20 caliper beats the scanner for these every time. Use the scan for the *profile*, calipers for the *widths*.

---

## 3. Registration — the underrated hard part

Before any boolean, the host frame and donor exterior must be positioned relative to each other **in the pose they'll occupy in the finished gun**. This is a design decision, not a geometry operation, and it deserves its own explicit step:

1. The **host frame position is fixed** — it's determined by the gun's mechanics. Import the frame solid at a known origin (recommendation: symmetry plane = XZ plane, bore axis parallel to X, some repeatable datum like the trigger pivot or magwell front face at the origin).
2. Align the donor exterior's symmetry plane to the same XZ plane.
3. Now slide/rotate the donor shape in that plane until the ergonomics are right: **trigger reach** (distance from backstrap to trigger face), **grip angle** relative to bore, and **bore height** above the web of the hand. For a target pistol these are exactly the parameters you care about, and this is your one chance to tune them — you can even deliberately change the effective grip angle of the "Glock" by rotating the donor shell.
4. Record the transform. Bake it into the exported donor mesh so the boolean stage sees pre-aligned inputs.

Do this visually in Blender with both meshes loaded and a side-view reference. Two sanity checks before proceeding:

- **Containment**: everywhere you need grip material, the frame solid must be inside the donor envelope with enough margin. Quick check: boolean *intersect* the frame solid with the donor shell — any place the frame pokes out of the grip surface (other than intentionally, e.g., trigger guard, top) will be visible.
- **Wall thickness preview**: the gap between donor exterior and frame solid is your future wall. Slim donor + wide host = paper walls. Section it in a few places now (Section 8 has the systematic version).

---

## 4. Building the host subtraction solid

Two paths; they produce the same asset. Start with the manual path for your first frame — it doubles as the spec for automating.

### 4a. Manual path (SolveSpace, using the scan as reference)

1. **Extract the side silhouette from the scan.** In Blender: orient the frame scan to the datum pose, then either (a) render an orthographic side view at known scale and use it as a bitmap underlay, or (b) better, project the mesh to the side plane and export a DXF outline. A 10-line Python script (trimesh: project vertices to plane, rasterize, contour with OpenCV or skimage, simplify with shapely) gets you a clean DXF; so does Blender's "flatten to plane + limited dissolve + export DXF" dance. SolveSpace imports DXF directly as construction geometry to trace over.
2. **Trace the outline** in a SolveSpace sketch with lines and arcs. Trace only the **outer contour** — skip every window, pin hole, and pocket. This is where the magwell-window problem solves itself: an outer-contour trace *is* the filled solid. Deliberately overshoot where the frame exits the grip (extend the profile a few mm above the grip top line and below the heel) so the subtraction cuts cleanly through the surface rather than leaving a knife-edge membrane.
3. **Partition the profile into thickness regions** and extrude each symmetric about the center plane at its calipered width: e.g., main body 22 mm, rail block 25 mm, lower tang 12 mm. Two or three extrusions covers most target-pistol frames, which is exactly the pattern you identified.
4. **Union the extrusions.** SolveSpace note: its NURBS booleans can be temperamental on tangent/coplanar faces. Two mitigations: nudge region boundaries so faces overlap rather than exactly abut (make each extrusion 0.5 mm proud into its neighbor), and if a union still fails, switch the group's "force NURBS to triangle mesh" option — you're exporting STL anyway, so losing the NURBS representation costs nothing.
5. **Skip the fillets.** Real frames have edge rounds; your prismatic rebuild is a *superset* of the frame at those edges. Since this solid is being subtracted, the superset just means a little extra internal clearance at corners nobody will ever see or feel. This is a feature: it kills the entire most-tedious category of CAD work.
6. **Handle the non-2.5D exceptions** (dovetails, angled surfaces, tapered magwell mouths) as additional primitive solids unioned on, or just bound them with an oversized box — again, oversubtracting internally is almost always fine.
7. Export STL at fine tessellation.

### 4b. Near-automated path (the custom tool)

The manual path above is a fixed pattern of operations, which makes it a good automation target. Pipeline spec for a Python tool (all pieces are pip-installable and well-maintained):

```
frame_scan.stl
  → load + repair              (trimesh / meshlib)
  → align to datum pose         [MANUAL CHECKPOINT: confirm/adjust axes]
  → rasterize side projection   (project verts to XZ, occupancy grid ~0.1 mm/px)
  → outer contour only          (OpenCV findContours, RETR_EXTERNAL —
                                 interior holes/windows vanish automatically)
  → simplify + smooth outline   (shapely simplify + optional arc fitting)
  → thickness map               (cast rays ±Y across the grid; local width
                                 = right-hit minus left-hit)
  → quantize widths to plateaus (k-means or manual list from calipers)
                                [MANUAL CHECKPOINT: review plateau map,
                                 override values with caliper numbers]
  → extrude each plateau region symmetric about XZ  (shapely polygon
                                 → trimesh extrude)
  → union                       (manifold3d — exact, fast)
  → dilate by clearance         (meshlib offset, voxel-based; e.g. +0.15 mm)
  → frame_solid.stl
```

- `RETR_EXTERNAL` in the contour step is the one-liner that fills magwell windows — the same insight as tracing only the outer contour by hand.
- The two manual checkpoints (pose confirmation, plateau review) are cheap and worth keeping; everything else runs unattended.
- Per-frame configuration lives in a small YAML: datum transform, plateau widths, clearance value, and the list of extra clearance solids (Section 5). Rerunning the tool on a new scan of the same frame family is then a one-command affair.

I can build this tool; the manual path output for your first frame becomes its regression test.

### Which path when

Manual SolveSpace: first frame, frames with tricky geometry, or when you want exact drawn dimensions. Automated: second-and-later frames, quick experiments, rescans. Both produce the same artifact (`frame_solid.stl` + clearance kit), so they're interchangeable downstream.

---

## 5. The clearance kit: what to add beyond the frame

The frame alone is not enough to subtract — the finished grip must also stay out of the way of everything that *moves* or *needs finger access*. Model each of these as a dumb primitive (box, cylinder, extruded profile, swept prism) in the frame's datum coordinates, and keep them in a per-frame "clearance kit" file that gets unioned with the frame solid at boolean time:

- **Magazine path**: extrude the magwell's internal footprint along the full insertion direction, from above the feed position to well below the grip's heel, so nothing you build can ever intrude into the magazine's travel — including drop-free ejection if you care about it.
- **Magazine release**: button travel volume plus a finger-access cone/scallop on the reachable side.
- **Trigger and trigger guard**: the trigger's full adjustment and travel envelope; on target pistols with adjustable shoes, use the envelope of all adjustment positions.
- **Slide/bolt/moving upper**: whatever reciprocates or opens, swept along its travel, with generous margin.
- **Levers and pins**: safety, slide stop, takedown pins (including punch access if you'll ever detail-strip with grips on).
- **Attachment features**: this one is *additive to your thinking* but still subtractive geometry — grip screw through-holes, counterbores for screw heads, and access to the frame's threaded bosses. Target pistols typically hang the grip on one or two machine screws; model those as cylinders in the kit. If the host uses a palm-shelf rail or clamp hardware, model its pocket too.
- **Sight radius / hand clearance**: anything the donor shape might block that the host needs (e.g., a low-mounted counterweight rail).

The kit is where per-gun knowledge accumulates. It's maybe an hour of primitive modeling per frame, done once.

---

## 6. Fit clearance strategy

You need the frame to actually slide into the printed/machined grip. Options, best first:

1. **SDF/voxel dilation of the frame solid** (MeshLib `offsetMesh`, or Blender remesh tricks): geometrically correct offset that handles concave corners properly. Recommended values: **+0.10–0.20 mm per side for FDM prints** (printers over-extrude inward on holes), **+0.05–0.10 mm for CNC in hardwood** plus whatever your finish (oil vs. film) adds. Make this a parameter, not a baked-in dimension.
2. **Draw-in clearance** in the CAD rebuild (extrude at width + 2×clearance): fine, but it couples the asset to one manufacturing process. Keep the CAD nominal and offset at pipeline time instead.
3. Vertex-normal "shrink/fatten" offsets: avoid — they self-intersect at concave features and corrupt exactly the corners you care about.

Plan on dialing this empirically with test coupons (Section 8) rather than trusting the first number.

---

## 7. The boolean itself

State of the art has genuinely improved; slow-and-flaky is no longer the norm. Four good engines, in rough order of fit for your workflow:

1. **OpenSCAD (2025+ releases, Manifold backend)** — sleeper pick for you specifically. A per-project `.scad` file that does `difference() { import("donor.stl"); union() { import("frame_solid.stl"); /* clearance kit primitives, parametric */ } }` is a *readable, versionable recipe* for each grip, and the clearance kit can live directly in OpenSCAD as parametric cubes/cylinders instead of a separate mesh file. With the Manifold backend this runs in seconds even on scan-sized meshes.
2. **Blender 4.5+ Boolean modifier, "Manifold" solver** — new solver added in 4.5, much faster and more robust than the old Exact solver, with the constraint that inputs must be watertight manifold meshes (which your Section 2 cleanup guarantees). Best when you want to eyeball and tweak alignment interactively in the same file.
3. **Python: trimesh + manifold3d, or MeshLib** — for the automated pipeline. MeshLib additionally offers voxel-based booleans: convert both solids to a signed distance field, subtract, re-mesh. This route is essentially unkillable — it does not care about self-intersections, near-coplanar faces, or degenerate triangles — at the cost of resampling everything at the chosen voxel size (0.1–0.15 mm is plenty below printer/router resolution).
4. **Voxel fallback for anything that resists**: any time an exact boolean fails or produces slivers, voxel-remesh both inputs at 0.15 mm and redo it. This converts every exotic failure mode into "worked, slightly smoothed."

Failure modes to know: exact engines choke on *coplanar overlapping faces* — which your deliberately-proud extrusions and dilated clearance offset already prevent (nothing in this workflow produces exactly-touching faces, and that's intentional). If you ever see zero-thickness membranes in a result, the cause is a subtraction solid that ends exactly at the grip surface — extend it through (Section 4a step 2).

---

## 8. Validation before cutting anything expensive

1. **Wall thickness map**: compute thickness across the result (MeshLib has this; slicers show it implicitly in preview). Set a floor: roughly ≥2.5–3 mm for load-bearing hardwood walls, ≥1.6–2 mm for a printed prototype in PETG/PA. Thin spots mean the donor/host pairing or registration needs adjusting — better to learn that now.
2. **Section views**: cut the model on 3–4 planes (Blender bisect or slicer preview) and inspect the frame pocket, screw bosses, and mag channel.
3. **Tolerance coupon**: before printing whole grips, print a small block containing just the frame's cross-section pocket at 3 clearance values (e.g., +0.05/+0.15/+0.25 mm). Ten-minute print, tells you your scanner+printer system's true offset.
4. **Full printed prototype**: print, fit the frame, insert a magazine, dry-fire, check control access. Iterate registration/clearance here — printing is your cheap loop. Only cut wood once a print fits and handles right.
5. Keep a small changelog per iteration (clearance used, what bound, what rattled) — with scans in the loop it's easy to lose track of which number produced which fit.

---

## 9. Manufacturing notes

### Printing
Print the fit prototypes in the final one-piece geometry — printers don't care about internal pockets. Orient with the bore axis vertical-ish so the frame pocket's critical surfaces are walls, not bridged ceilings.

### CNC (VCarve)
A wraparound grip with an internal frame-shaped pocket is not machinable as one piece — the pocket is trapped. The standard solution, and how commercial anatomical target grips are typically handled:

1. **Split the result into left/right halves** along the frame's center plane (one more boolean, with a half-space/box). Now each half's internal geometry is a pocket opening toward the flat mating face — machinable from two sides.
2. Per half, two-sided setup in VCarve: **inner face up first** (machine the frame pocket and the flat mating face into the top of the blank), then flip on registration dowels to carve the organic exterior. Dowel holes in the waste margin, machined in op 1, are your registration.
3. The flat mating faces join by glue (permanent) or by making the grip screws pass through both halves into the frame (serviceable). A shallow key/boss pair on the mating faces, added as one more boolean pair, kills shear alignment issues.
4. VCarve consumes the STL directly for 3D finishing toolpaths; keep exported STLs dense (0.05–0.1 mm chord tolerance) so toolpath quality isn't limited by tessellation.
5. Leave the donor scan's surface texture out of the CNC version (remesh smooth) — you'll checker/stipple the wood by hand better than a scanned Glock texture will carve.

---

## 10. Suggested build order

1. Scan + clean donor grip and host frame (Section 2).
2. Manual SolveSpace rebuild of the host frame (4a) + clearance kit (5) → `frame_solid.stl`.
3. Registration session in Blender (3); bake transform.
4. First boolean via OpenSCAD or Blender Manifold (7); validate (8); print; iterate clearance.
5. Split + CNC once a print passes (9).
6. Then, with one full manual pass as ground truth, build the automation tool (4b) so frames two through N cost an evening instead of a week.

### Concrete tool/version notes (as of mid-2026)

- Blender ≥ 4.5: Boolean modifier gained the **Manifold** solver; also the voxel Remesh modifier for watertighting.
- OpenSCAD: use a current release/snapshot with the **Manifold** backend enabled — orders of magnitude faster on imported STLs than the old CGAL path.
- Python: `pip install trimesh manifold3d meshlib shapely opencv-python` covers the entire automated pipeline (booleans, robust offsets, voxel ops, contouring).
- SolveSpace: DXF import for tracing works well; force-to-mesh option sidesteps NURBS boolean fragility.
- MeshLab: still handy for scan repair and measurement, optional.
