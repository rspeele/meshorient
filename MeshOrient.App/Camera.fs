/// Cameras for the four panels.
///
/// Adapted from CNCFingers' `CNCFlow.Render.Camera` — the orbit maths, the
/// cursor-ray unprojection and the pan basis are the same problem solved
/// there, trimmed to what this tool needs and moved to +Z up in millimetres.
module MeshOrient.App.Camera

open System
open System.Numerics
open Avalonia
open MeshOrient.Core

/// The four panels. `Right`, `Top` and `Back` are fixed orthographic views
/// auto-framed to the whole model; `Free` is the orbiting 3D view where
/// picking happens.
type ViewKind =
    | Right
    | Top
    | Back
    | Free

    /// The letter drawn in the panel corner, matching the sketch this was
    /// designed from. Not `Tag` — F# already generates that on a union.
    member k.Label =
        match k with
        | Right -> "R" | Top -> "T" | Back -> "B" | Free -> "3D"

    member k.Caption =
        match k with
        // Genuine orthographic views, so the on-screen axis directions are
        // whatever standing in that position actually gives you. Spelled out
        // because a mirrored side view is a real trap: it turns a right-hand
        // part into a left-hand one and nothing on screen says so.
        | Right -> "RIGHT side (-Y)   +X right, +Z up"
        | Top -> "TOP (+Z)   +X right, +Y up"
        | Back -> "BACK (-X)   -Y right, +Z up"
        | Free -> "drag: orbit · middle-drag: pan · wheel: zoom · click: pick · right-click: unpick"

/// One panel: which view it shows and where it sits, in control (DIP)
/// coordinates with the origin top-left, as Avalonia reports pointers.
type Panel = { Kind : ViewKind; Rect : Rect }

/// Left column of three stacked orthographic panels, one large 3D panel to the
/// right — the layout from the sketch this was designed against.
///
/// A pure function of the control's size rather than something the renderer
/// records as a side effect, so the window can place its panel captions
/// without having to wait for a frame to be drawn first.
let panelLayout (w : float) (h : float) : Panel[] =
    let leftW = Math.Clamp(w * 0.28, 140.0, 340.0)
    let rowH = h / 3.0
    [|  { Kind = Right; Rect = Rect(0.0, 0.0, leftW, rowH) }
        { Kind = Top; Rect = Rect(0.0, rowH, leftW, rowH) }
        { Kind = Back; Rect = Rect(0.0, rowH * 2.0, leftW, h - rowH * 2.0) }
        { Kind = Free; Rect = Rect(leftW, 0.0, w - leftW, h) } |]

/// Eye direction (from the model toward the camera) and up vector per view.
///
/// THE HANDEDNESS TRAP, because it caught this file once already. The world is
/// right-handed, so naming three views leaves no free choice: fix the part's
/// long axis along +X and up along +Z, and its right side is FORCED to -Y,
/// since right = forward x up. It is NOT +Y.
///
/// The Right panel therefore looks from -Y. It looked from +Y at first, which
/// silently contradicted the Back panel looking from -X: putting the back of
/// the part in the Back panel forces the front to +X, which makes +Y the LEFT
/// side, so the "Right" panel showed the part's left (user-reported). The two
/// labels could not both be true at once for any orientation.
///
/// The convention that makes all three consistent, and the one assumed here:
///     +X = forward               -X = back
///     +Z = up                    -Z = down
///     -Y = the part's right      +Y = the part's left
/// Orient so the top shows in the Top panel and the back in the Back panel,
/// and the Right panel then genuinely shows the right side, with the front
/// running off to the right of frame.
let private orthoBasis (kind : ViewKind) =
    match kind with
    | Right -> Vector3(0.0f, -1.0f, 0.0f), Vector3(0.0f, 0.0f, 1.0f)
    | Top -> Vector3(0.0f, 0.0f, 1.0f), Vector3(0.0f, 1.0f, 0.0f)
    | Back -> Vector3(-1.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 1.0f)
    | Free -> Vector3(0.0f, -1.0f, 0.0f), Vector3(0.0f, 0.0f, 1.0f)

let private toV3 (v : Vec3) = Vector3(float32 v.X, float32 v.Y, float32 v.Z)

/// View and projection for a fixed orthographic panel, framed so the whole
/// model fits with a small margin whatever its current orientation.
///
/// Reframing on every transform is deliberate — it is what makes the three
/// small panels usable as a check on an orientation you are actively
/// changing.
let orthoMatrices (kind : ViewKind) (bounds : Bounds) (aspect : float32) =
    let dir, up = orthoBasis kind
    let centre = toV3 (Bounds.centre bounds)
    let diag = max 1e-3f (float32 (Bounds.diagonal bounds))
    let eye = centre + dir * (diag * 2.0f)
    let view = Matrix4x4.CreateLookAt(eye, centre, up)

    // Project the eight bbox corners into view space and fit the box to them.
    let lo, hi = toV3 bounds.Min, toV3 bounds.Max
    let mutable minX, maxX = Single.MaxValue, Single.MinValue
    let mutable minY, maxY = Single.MaxValue, Single.MinValue
    for i in 0 .. 7 do
        let c = Vector3((if i &&& 1 = 0 then lo.X else hi.X),
                        (if i &&& 2 = 0 then lo.Y else hi.Y),
                        (if i &&& 4 = 0 then lo.Z else hi.Z))
        let v = Vector3.Transform(c, view)
        minX <- min minX v.X; maxX <- max maxX v.X
        minY <- min minY v.Y; maxY <- max maxY v.Y
    let margin = 1.06f
    let mutable halfW = max 1e-3f ((maxX - minX) * 0.5f * margin)
    let mutable halfH = max 1e-3f ((maxY - minY) * 0.5f * margin)
    // Letterbox rather than stretch: the panels are used to judge whether
    // something is square, so a non-uniform scale would be a lie.
    if halfW / halfH < aspect then halfW <- halfH * aspect else halfH <- halfW / aspect
    let proj = Matrix4x4.CreateOrthographic(halfW * 2.0f, halfH * 2.0f, 0.01f, diag * 6.0f)
    view, proj

/// Orbital camera for the 3D panel. Angles in radians, distance in mm.
type Orbit =
    {   Target : Vector3
        Distance : float32
        Yaw : float32
        Pitch : float32
        FovY : float32
        /// Numpad-5, Blender style. Ortho sizes itself from Distance so the
        /// apparent zoom does not jump when toggling.
        Orthographic : bool }

let defaultOrbit =
    {   Target = Vector3.Zero
        Distance = 300.0f
        // Off the front-right quarter (+X, -Y), tilted down a little: enough
        // parallax to read the shape, and the same side the Right panel shows
        // already turned toward you, so the two views agree at a glance.
        Yaw = MathF.PI * -0.20f
        Pitch = MathF.PI * 0.12f
        FovY = MathF.PI * 0.25f
        Orthographic = false }

/// Frame the orbit camera on a model: centre on it and back off far enough
/// that the whole thing fits the vertical field of view.
let frameOrbit (bounds : Bounds) (orbit : Orbit) =
    let diag = max 1.0f (float32 (Bounds.diagonal bounds))
    { orbit with
        Target = toV3 (Bounds.centre bounds)
        Distance = diag * 0.75f / MathF.Tan(orbit.FovY * 0.5f) }

let eyePosition (c : Orbit) =
    let cosP, sinP = MathF.Cos c.Pitch, MathF.Sin c.Pitch
    let cosY, sinY = MathF.Cos c.Yaw, MathF.Sin c.Yaw
    c.Target + Vector3(cosP * cosY, cosP * sinY, sinP) * c.Distance

let orbitMatrices (c : Orbit) (aspect : float32) =
    let eye = eyePosition c
    // Up is the TANGENT of the orbit sphere, not a constant world-up: at
    // ordinary pitches LookAt orthonormalises both to the identical basis,
    // but a constant (0,0,1) degenerates when a numpad-7 preset puts the
    // camera exactly at the pole, and the tangent never does.
    let cosP, sinP = MathF.Cos c.Pitch, MathF.Sin c.Pitch
    let cosY, sinY = MathF.Cos c.Yaw, MathF.Sin c.Yaw
    let up = Vector3(-sinP * cosY, -sinP * sinY, cosP)
    let view = Matrix4x4.CreateLookAt(eye, c.Target, up)
    // Near/far scale with distance so a big scan and a small one both get
    // usable depth precision without any per-model tuning.
    let near = max 0.05f (c.Distance * 0.01f)
    let far = c.Distance * 10.0f
    let proj =
        if c.Orthographic then
            // Sized to the perspective frustum's height at the target, so
            // numpad-5 swaps projection without a zoom jump.
            let h = 2.0f * c.Distance * MathF.Tan(c.FovY * 0.5f)
            Matrix4x4.CreateOrthographic(h * aspect, h, near, far)
        else
            Matrix4x4.CreatePerspectiveFieldOfView(c.FovY, aspect, near, far)
    view, proj

/// Pitch is clamped short of the poles: passing straight over the top makes
/// the up vector degenerate and the view snap through 180 degrees.
let rotate (dYaw : float32) (dPitch : float32) (c : Orbit) =
    let limit = MathF.PI * 0.49f
    { c with Yaw = c.Yaw + dYaw; Pitch = Math.Clamp(c.Pitch + dPitch, -limit, limit) }

let zoom (factor : float32) (c : Orbit) =
    { c with Distance = Math.Clamp(c.Distance * factor, 0.5f, 100_000.0f) }

/// Drag the pivot across the screen plane. `dx`/`dy` are in pixels; the scale
/// converts to world units so panning tracks the cursor at any zoom.
let pan (dx : float32) (dy : float32) (viewportHeight : float32) (c : Orbit) =
    let eye = eyePosition c
    let forward = Vector3.Normalize(c.Target - eye)
    let worldUp = Vector3(0.0f, 0.0f, 1.0f)
    let right = Vector3.Normalize(Vector3.Cross(forward, worldUp))
    let up = Vector3.Normalize(Vector3.Cross(right, forward))
    let worldPerPixel = 2.0f * c.Distance * MathF.Tan(c.FovY * 0.5f) / max 1.0f viewportHeight
    { c with Target = c.Target + (right * -dx + up * dy) * worldPerPixel }

/// World-space ray through normalised device coordinates
/// (x and y both in [-1, 1], y up).
let cursorRay (view : Matrix4x4) (proj : Matrix4x4) (ndc : Vector2) =
    let mutable inv = Matrix4x4.Identity
    Matrix4x4.Invert(view * proj, &inv) |> ignore
    let unproject (z : float32) =
        let r = Vector4.Transform(Vector4(ndc.X, ndc.Y, z, 1.0f), inv)
        Vector3(r.X, r.Y, r.Z) / r.W
    let near = unproject 0.0f
    let far = unproject 1.0f
    let dir = Vector3.Normalize(far - near)
    {   Raycast.Origin = { X = float near.X; Y = float near.Y; Z = float near.Z }
        Raycast.Direction = { X = float dir.X; Y = float dir.Y; Z = float dir.Z } }
