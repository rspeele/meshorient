# meshorient

Gets a raw 3D scan **square**, then writes it back out.

That is the whole tool. It is frame2solid's steps 1–2 rebuilt as a small F#
desktop app, because getting a scan onto a datum that means something is the
fiddly part of any scan-to-CAD job — and everything downstream (Blender
booleans, MeshMixer registration, CNC setup) is easier once it is done. The
tier/silhouette/DXF half of f2s stays in f2s.

    dotnet run --project MeshOrient.App -- myscan.stl

Reads STL (binary + ASCII), OBJ and PLY. Writes `<name>_oriented.stl` — a
rigid transform of the vertices, same triangle count, same topology, nothing
resampled, so sharp edges and open hole boundaries survive exactly.

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

These are genuine orthographic views, not mirrored to look familiar. A
mirrored side view is a real trap — it turns a right-hand part into a
left-hand one and nothing on screen says so — so each panel spells out which
way its axes run.

### Which way round

The world is right-handed, so naming three views leaves no free choice. Fix
the bore along +X and up along +Z, and the gun's right side is **forced** to
−Y, because right = bore × up:

    +X = muzzle / forward      -X = back
    +Z = up                    -Z = down
    -Y = the gun's RIGHT       +Y = the gun's left

Orient the scan so the top shows in T and the back shows in B, and R then
genuinely shows the right side with the muzzle running off to the right of
frame. (The Right panel originally looked from +Y, which quietly contradicted
the Back panel: satisfying B forces the muzzle to +X, which makes +Y the
*left* side. The two labels could not both be true for any orientation.)

Note this is the opposite Y sense to f2s's synthetic scan, which puts its
"right side" trigger-bar boss at +Y and so implies a muzzle at −X. Nothing in
either tool depends on it — squaring works off whichever flat side you pick —
but it is why the two can disagree about which side is "right".

## Three stages

Each narrows the freedom the one before it left, and each reports how good
your picks actually were as well as what it did.

**1 · Coarse.** `Auto-orient` runs PCA: longest principal axis → X, thinnest
→ Y, middle → Z. Then the 90° and Flip buttons fix the quarter-turn
ambiguities PCA cannot resolve. Expect PCA to leave about a degree in the X-Z
plane on an L-shaped part like a frame — that is what stage 3 is for.

**2 · Square.** Click **3 or more points on one face that really is flat** in
the 3D view — the frame's right side is ideal. A red marker drops on each. A
plane is fitted and the *smallest* rotation that squares it to Y is applied:
it removes tilt without spinning the model about Y, so stage 1's work stays
put.

The readout gives the tilt and the **RMS coplanarity of your picks**. That
second number is the one to watch: it is your scan's own flatness plus your
aim, and a large value means one pick missed the flat and the fit is not to be
trusted. Press Square again and it should read 0.000° — picks are stored in
model space, so they travel with the mesh.

The fitted plane draws in red against a blue Y = 0 datum in the T and B
panels. Square it and the red line lands parallel to the blue one. A number
saying "0.000°" asks to be believed; two parallel lines can be checked.

**3 · Straighten.** Click **2 or more points on a flat top or bottom
reference** — a Glock slide top is the ideal case — and the model spins until
that face is level and the bore runs straight down X. Two picks define the
line exactly; more average out the error in each, which is the point of
picking several.

This rotates **about Y**, not about X. That is forced, not a preference:
once a side face is square to Y, rotating about X or Z tips it straight back
out of square. Rotation about the axis stage 2 locked is the only remaining
freedom — and it is exactly the one that swings the bore up and down in the
side view. A 10° rotation about X moves the side face's normal from
`(0, 1, 0)` to `(0, 0.985, 0.174)`; about Y it stays `(0, 1, 0)` exactly.
`StraightenTests.Straightening_leaves_the_locked_axis_exactly_alone` is what
stops anyone "fixing" this later.

Re-running stage 2 invalidates stage 3's result but keeps its picks, so you
can go back and forth.

## Build

- `MeshOrient.Core` — all the maths. Mesh I/O, eigen/plane/line fitting, PCA,
  ray-mesh intersection, the three stages. No Avalonia, no GL, no OpenGL: that
  is what makes every measurement testable without a window.
- `MeshOrient.App` — Avalonia 12 + Silk.NET.OpenGL. Same stack as CNCFlow.UI.
- `MeshOrient.Core.Tests` — MSTest, 40 tests, run with `dotnet test`.

One `OpenGlControlBase` with four `glViewport` passes, not four GL controls:
each Avalonia GL control owns its own context, so a 760k-triangle scan would
be uploaded four times. Re-orienting changes a uniform matrix, never the
vertex buffer, which is what keeps the rotate buttons instant on a big scan.

Picking is brute-force Möller-Trumbore across every triangle, chunked over
cores. No BVH: it runs on a *click*, not per frame, and a few milliseconds is
imperceptible where a tree to build and keep in sync is not.

## Tests

`dotnet test` — the fixture is frame2solid's `synthetic_frame_scan.stl`, a
mock frame with known dimensions and deliberate scan defects (0.05 mm vertex
jitter, 2% of triangles dropped). Sharing it means meshorient and f2s are
measured against the same object and can be cross-checked directly.

The load-bearing ones:

- **f2s's level regression, ported.** Introduce the same 1.3° tilt
  `test_gui.py` uses, pick four points on the flat right wall, and assert it
  comes back to under 0.05°. Measured: 1.312° → 0.006°, with the wall then
  reading one thickness end to end within 0.03 mm.
- **Straighten** recovers a known 7° roll to 0.026°, leaves the locked axis
  fixed to 1e-9, and levels the rail top to 0.07 mm.
- **PCA is pose-invariant**: the same scan presented in three wildly different
  poses auto-orients to bounding boxes agreeing to 0.01 mm. Repeatability
  matters more here than accuracy — if it drifted, two exports of the same
  part would land in different places.
- **The full run**, in `OrientStateTests`: load a deliberately crooked scan,
  auto-orient, pick four wall points, Square twice (0.291° → 0.0000°), export,
  re-import and confirm the written STL is already square.

## Not here

Silhouettes, thickness tiers, DXF, clearance offsets, project files. This
tool orients meshes and stops. Use frame2solid for the CAD path.
