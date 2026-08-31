# meshorient

Gets a raw 3D scan **square**, then writes it back out.

That is the whole tool. Getting a scan onto a datum that means something is
the fiddly part of any scan-to-CAD job — and everything downstream (Blender
booleans, MeshMixer registration, CNC setup) is easier once it is done.

    dotnet run --project MeshOrient.App -- myscan.stl

Reads STL (binary + ASCII), OBJ and PLY. **Export STL** writes up to two
files beside the source: `<name>_oriented.stl` — always, the pristine scan
rigidly transformed, nothing resampled, no flattens even if some were
applied — and `<name>_cleaned.stl` when flattens are baked in, with the same
orientation. The cleanup is never the price of the raw geometry.

**Loading does not move the model.** Centring on the bbox happens on the
FIRST orientation command, not before — so a scan you only flatten and
re-export comes back in the exact coordinate frame it arrived in, ready to
drop onto other objects registered against it. The export status names the
frame it wrote ("original coordinates" vs "the oriented datum"). Note
Y-Centerline is a deliberate translation, so it counts as orientation.

## The window

Three fixed orthographic panels down the left, one orbiting 3D view on the
right. The ortho panels re-frame themselves to the whole model after every
change, so they are a live check on an orientation you are actively editing.

| | shows | on screen |
|---|---|---|
| **R** | the right side (−Y face) | +X right, +Z up |
| **T** | the top (+Z face) | +X right, +Y up |
| **B** | the back (−X face) | −Y right, +Z up |
| **3D** | orbit | drag orbit · middle-drag pan · wheel zoom · click pick · right-click unpick |

Blender-style numpad in the 3D view: **1** the right side (as the R panel),
**3** the front (+X end-on), **7** top, **9** flip to the opposite view,
**2/4/6/8** 15° orbit steps, **5** toggle orthographic/perspective.

These are genuine orthographic views, not mirrored to look familiar. A
mirrored side view is a real trap — it turns a right-hand part into a
left-hand one and nothing on screen says so — so each panel spells out which
way its axes run.

### Which way round

The world is right-handed, so naming three views leaves no free choice. Fix
the part's long axis along +X and up along +Z, and its right side is
**forced** to −Y, because right = forward × up:

    +X = forward               -X = back
    +Z = up                    -Z = down
    -Y = the part's RIGHT      +Y = the part's left

Orient the scan so the top shows in T and the back shows in B, and R then
genuinely shows the right side with the front of the part running off to the
right of frame.

## Three moves, no modes

There is no stage selector. **One pick list, and every button reads it** —
Orient Face to Side wants 3+ points, Y-Spin Face to Level wants 2+. Nothing has to be told
which you meant, so nothing can be set wrong. Each button greys itself out
until it has enough points, so a button that is offered is a button that
works, and the readout at bottom right always says what *both* would do with
the picks you have:

    4 picks · to-side 1.312° (±0.007 mm) · y-spin 0.529° (±0.284 mm)

Those two residuals are more use than a mode label: points spread over a flat
side fit a plane tightly and a side-view line badly, and points along a top
edge do the reverse — so the numbers tell you which face you are actually on.

The typical run is: **Auto-orient → pick a flat side → Orient Face to Side →
Clear picks → pick a flat top → Y-Spin Face to Level → (optionally Flatten
Face and Y-Centerline) → Export STL.** Nothing enforces that order; the 90°
buttons, Undo and Export all work at any point.

Each move narrows the freedom the one before it left, and each reports how
good your picks actually were as well as what it did.

**Coarse.** `Auto-orient` runs PCA: longest principal axis → X, thinnest
→ Y, middle → Z. Then the 90° and Flip buttons fix the quarter-turn
ambiguities PCA cannot resolve. Expect PCA to leave about a degree in the X-Z
plane on an L-shaped part — that is what Y-Spin Face to Level is for.

**Orient Face to Side.** Click **3 or more points on one face that really
is flat** in the 3D view — a flat side wall is ideal. A red marker drops on
each. A plane is fitted and the *smallest* rotation that squares it to Y is
applied: it removes tilt without spinning the model about Y, so the coarse
orientation stays put.

The readout gives the tilt and the **RMS coplanarity of your picks**. That
second number is the one to watch: it is your scan's own flatness plus your
aim, and a large value means one pick missed the flat and the fit is not to be
trusted. The readout drops to 0.000° the moment it lands — picks are stored in
model space, so they travel with the mesh.

The fitted plane draws in red against a blue Y = 0 datum in the T and B
panels. Square it and the red line lands parallel to the blue one. A number
saying "0.000°" asks to be believed; two parallel lines can be checked.

**Y-Spin Face to Level.** Click **2 or more points on a flat top or bottom
reference** and the model spins until that face is level and the long axis
runs straight down X. Two picks define the line exactly; more average out
the error in each, which is the point of picking several.

This rotates **about Y**, not about X. That is forced, not a preference:
once a side face is square to Y, rotating about X or Z tips it straight back
out of square. Rotation about the locked axis is the only remaining
freedom — and it is exactly the one that swings the long axis up and down in
the side view. A 10° rotation about X moves the side face's normal from
`(0, 1, 0)` to `(0, 0.985, 0.174)`; about Y it stays `(0, 1, 0)` exactly.
`Straightening_leaves_the_locked_axis_exactly_alone` in `OrientTests` is what
stops anyone "fixing" this later.

Re-orienting invalidates a previous spin, so do them in that order — but
nothing stops you going back and forth.

**Y-Centerline.** Pick point(s) on the RIGHT face and on the LEFT face —
one each is enough, more per side average the scan noise out — and the model
TRANSLATES in Y so those faces sit symmetric about the XZ plane, ready to
slice down the middle into two symmetric halves. The picks must separate into
exactly two tight Y-groups (±0.5 mm); anything else is rejected with the
group count, so a stray pick cannot silently drag the centreline. Do this
LAST: rotations re-centre on the bbox and undo it (Undo also undoes it).

## FLATTEN — de-noise a flat face onto its true plane

Scan noise on a face you know is flat can be removed here instead of
eyeballed in Blender: pick 3+ points on the flat as usual, press
**Flatten Face**, inspect the preview, tweak, **APPLY**.

The naive "snap everything within k of the plane" fails two ways, and the
implementation exists to avoid both:

- **Flood fill, not the infinite plane.** The snap only reaches faces
  connected to your picks, walking face-to-face, so geometry elsewhere that
  happens to intersect the plane is never touched. The walk is gated by
  surface normal (within 30° of the plane, orientation-agnostic — scan
  winding is not trustworthy) as well as by distance, which is what stops it
  smearing a skirt up the base of every adjoining wall. Every pick seeds its
  own island: for a face broken into multiple strips (serrations, grooves),
  put one pick per strip and all of them flatten to the single common plane
  in one operation.
- **Feathering in distance space, not perimeter space.** Snap strength is a
  smoothstep of each vertex's own |distance|: full inside the `floor` (the
  noise band), fading to zero at the `ceiling` (k). A true flat — including
  one ending at a sharp box edge — sits entirely inside the noise band, so it
  snaps at full strength right up to the arris. A gentle curve leaves the
  band smoothly, so the correction fades smoothly instead of printing a
  crease at the k-boundary.

**Preview before commit.** Captured faces highlight **green** in all four
panels. Pockets completely surrounded by the capture but not part of it
highlight **yellow** — enclaves. A yellow patch is either a genuine feature
(leave it), a spot the normal gate rejected (raise the floor), or a dent
deeper than the ceiling; the point is that it is never invisible. The
**force-flatten enclaves** checkbox (off by default) snaps the yellow
pockets onto the plane wholesale, however far out their verts sit — for
scanner blobs and dents living inside a flat. The feathered band that leads
into a forced enclave is pulled to full strength with it (the "halo", also
shown yellow — the highlight is exactly the force's reach), which prevents
a raised ring forming around the erased blob. Detection is unchanged; only
what happens to enclaves changes, and the status line says how many verts it
will force. The status line reports verts, islands, RMS before → after, max
move, and enclave count. Nothing moves until APPLY; Esc or Discard drops the
preview; Undo reverses an apply exactly.

**The two boxes.** `floor` pre-fills from your picks' plane-fit RMS ×3 — the
scan's measured noise — and `ceiling` defaults to 0.3 mm. Arrow keys step
±0.1 mm above 0.1 and halve/double below it (0.21 → 0.11 → 0.055 → 0.0275,
and exactly back up); the preview recomputes on arrow press, Enter, tab-out,
the button, or a pick edit — never on a drag, and the button greys when the
shown preview is already current. The plane itself is refined from the whole
captured region (IRLS), so the picks only need to be roughly placed.

The plane is fitted wherever the face lies — flattening neither requires nor
disturbs orientation. Run it after squaring and the face becomes a true
Y = const datum. After an APPLY the export message stops claiming "nothing
resampled" and says how many verts were flattened instead.

**Snap to the true axis plane.** When the picks' plane sits within a couple
of degrees of a true axis-normal plane (in world space — the orientation the
export will use), a **snap to true Y plane** checkbox appears (the axis
letter is live), checked by default. A face that close to an axis is
evidently *meant* to be axis-true, and the fit residual is pick noise, not
design — but flattening onto the fitted plane leaves each such face flat yet
a fraction of a degree off its siblings, which is exactly what makes a cube
boolean in Blender later sit with one corner proud of the face and another
below flush. Snapped, the verts are sent onto the plane with *exactly* the
axis normal (through the fitted origin), so every snapped flat comes out
truly parallel. Membership is unchanged — the flood, feather and refit still
judge the face against its own free fit, so the capture is identical either
way — and the status line reports how far off-axis the free fit was.
Visually the result is indistinguishable from the free fit; the CSG later is
not.

## Build

- `MeshOrient.Core` — all the maths. Mesh I/O, eigen/plane/line fitting, PCA,
  ray-mesh intersection, the alignment moves, and the flatten flood
  (exact-bit weld + face adjacency, cached per mesh). No Avalonia and no GL:
  that is what makes every measurement testable without a window.
- `MeshOrient.App` — Avalonia 12 + Silk.NET.OpenGL.
- `MeshOrient.Core.Tests` — MSTest, run with `dotnet test`.

One `OpenGlControlBase` with four `glViewport` passes, not four GL controls:
each Avalonia GL control owns its own context, so a 760k-triangle scan would
be uploaded four times. Re-orienting changes a uniform matrix, never the
vertex buffer, which is what keeps the rotate buttons instant on a big scan.

Picking is brute-force Möller-Trumbore across every triangle, chunked over
cores. No BVH: it runs on a *click*, not per frame, and a few milliseconds is
imperceptible where a tree to build and keep in sync is not.

## Tests

`dotnet test` — the main fixture is `synthetic_frame_scan.stl`, a mock part
with known dimensions and deliberate scan defects (0.05 mm vertex jitter, 2%
of triangles dropped). The load-bearing ones:

- **Orient Face to Side recovers a known tilt.** Introduce a 1.3° tilt, pick
  four points on the flat right wall, and it comes back to under 0.05° —
  measured 1.312° → 0.006°, with the wall then reading one thickness end to
  end within 0.03 mm.
- **Y-Spin Face to Level** recovers a known 7° roll to 0.026°, leaves the
  locked axis fixed to 1e-9, and levels the top reference to 0.07 mm.
- **PCA is pose-invariant**: the same scan presented in three wildly different
  poses auto-orients to bounding boxes agreeing to 0.01 mm. Repeatability
  matters more here than accuracy — if it drifted, two exports of the same
  part would land in different places.
- **The readout survives every pick count**, including a single pick on a
  freshly loaded scan, so an exception on the pick path can never take the
  window down.
- **FLATTEN's two fixes, as A/B tests.** A noisy box top flattens with its
  arris exactly on the plane and its wall untouched to the last bit; a
  flat-into-fillet strip flattened with the hard snap kinks 6× worse than
  with the feathering (0.19 vs 0.03 mm). A wall fixture with a boss, a rib, a
  through-window and a perpendicular flange floods in 70 ms and reports
  exactly two enclaves — the boss and the rib, never the window or flange.
- **The synthetic is cracked** (each quad's vertex copies jittered
  independently), so it cannot host a flood. It stays as the hostile-mesh
  fixture — adjacency must survive its cracks and holes — while flood tests
  run on indexed fixtures round-tripped through real STL, which is what
  scanner-cleanup output actually looks like.
- **The full run**, in `OrientStateTests`: load a deliberately crooked scan,
  auto-orient, pick four wall points, square twice (0.291° → 0.0000°), export,
  re-import and confirm the written STL is already square.

## Not here

Silhouettes, thickness tiers, DXF, clearance offsets, project files. This
tool orients meshes and stops.
