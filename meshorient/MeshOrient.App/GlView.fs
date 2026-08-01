namespace MeshOrient.App

open System
open System.Numerics
open Avalonia
open Avalonia.Controls
open Avalonia.OpenGL
open Avalonia.OpenGL.Controls
open Silk.NET.OpenGL
open MeshOrient.Core
open MeshOrient.App.Camera

/// The four views, drawn into ONE OpenGL control.
///
/// Not four `OpenGlControlBase`s: each Avalonia GL control owns a separate
/// context, so a 760k-triangle scan would be uploaded four times and every
/// shader compiled four times. One control with four `glViewport` passes
/// shares a single vertex buffer and a single program, and the only thing that
/// differs between panels is a pair of matrices.
///
/// Pointer input does not arrive here — `OpenGlControlBase` has no Background,
/// so Avalonia's hit-tester treats it as transparent whatever is in the
/// framebuffer. The window wraps this in a Border and forwards events to the
/// `Handle*` members below.
type GlView(state : OrientState) as this =
    inherit OpenGlControlBase()

    let mutable gl : GL = Unchecked.defaultof<GL>
    let mutable programs : Scene.Programs voption = ValueNone
    let mutable meshBuf = Scene.empty
    let mutable sphereBuf = Scene.empty
    let mutable overlayBuf = Scene.empty
    /// Set when the mesh changed and the GPU copy is stale. The upload itself
    /// has to happen inside a render callback, where the context is current.
    let mutable meshDirty = false
    let mutable overlayDirty = true
    /// FLATTEN preview highlights: captured faces (green) and enclaves
    /// (yellow). Rebuilt when the state's preview stamp moves — a parameter
    /// tweak or pick edit — never per frame. Region-sized, so the rebuild is
    /// milliseconds even on an 800k-triangle scan.
    let mutable previewGreen = Scene.empty
    let mutable previewYellow = Scene.empty
    let mutable previewStampSeen = -1

    let mutable orbit = defaultOrbit

    // Pointer gesture state. A press that never moves more than a few pixels
    // is a pick; anything further is a drag. Without that distinction every
    // attempt to orbit would drop a pick where you started.
    let mutable dragButton = 0          // 0 none, 1 left, 2 middle
    let mutable dragStart = Point()
    let mutable lastPoint = Point()
    let mutable dragged = false

    let bg = Vector3(0.13f, 0.14f, 0.16f)
    let bgFree = Vector3(0.10f, 0.11f, 0.13f)
    let meshColour = Vector3(0.78f, 0.79f, 0.82f)
    let pickColour = Vector3(0.95f, 0.22f, 0.18f)
    let capturedColour = Vector3(0.30f, 0.78f, 0.38f)
    let enclaveColour = Vector3(0.93f, 0.83f, 0.20f)

    let pixelSize () =
        let scaling =
            match TopLevel.GetTopLevel this with
            | null -> 1.0
            | tl -> tl.RenderScaling
        let b = this.Bounds
        scaling, max 1 (int (b.Width * scaling)), max 1 (int (b.Height * scaling))

    /// World-space line geometry for the overlays: the datum the correction is
    /// judged against, and the trace of whatever is currently fitted.
    ///
    /// This is f2s's best idea from its Level step carried over — the fitted
    /// plane drawn in red against a blue Y=0 datum, so pressing LEVEL visibly
    /// snaps one parallel to the other. A number saying "0.00 degrees" asks to
    /// be believed; two parallel lines can be checked.
    let buildOverlay () =
        let verts = ResizeArray<Scene.LineVertex>()
        let b = state.Bounds
        if not b.IsEmpty then
            let lo, hi = b.Min, b.Max
            let c = Bounds.centre b
            let v3 (x : float) (y : float) (z : float) = Vector3(float32 x, float32 y, float32 z)
            let datum = Vector3(0.15f, 0.62f, 1.0f)
            let trace = Vector3(1.0f, 0.30f, 0.25f)
            let seg (a : Vector3) (bb : Vector3) (col : Vector3) =
                verts.Add(Scene.LineVertex(a, col))
                verts.Add(Scene.LineVertex(bb, col))
            // Pad so the lines run past the model and stay readable.
            let padX = (hi.X - lo.X) * 0.08 + 1.0
            let padZ = (hi.Z - lo.Z) * 0.08 + 1.0

            let picks = state.PicksWorld

            // No mode to switch on. Each panel gets the fit that belongs to
            // it, and they never collide because they live in different
            // panels: the plane fit SQUARE would use reads in Top and Back,
            // the side-view line fit STRAIGHTEN would use reads in Right.
            // Whichever face you actually picked, the panel you are looking
            // at is showing you the right thing.

            // --- for SQUARE: Y = 0 datum along X (reads in Top) and along Z
            //     (reads in Back), plus the fitted plane's two traces.
            seg (v3 (lo.X - padX) 0.0 c.Z) (v3 (hi.X + padX) 0.0 c.Z) datum
            seg (v3 c.X 0.0 (lo.Z - padZ)) (v3 c.X 0.0 (hi.Z + padZ)) datum
            if picks.Length >= 3 then
                let fit = Geometry.fitPlane picks
                let n = if fit.Normal.Y < 0.0 then -fit.Normal else fit.Normal
                if abs n.Y > 1e-9 then
                    let cen = fit.Centroid
                    // Where the fitted plane cuts a line of constant X (or of
                    // constant Z) — the same two traces f2s draws on its front
                    // and top views.
                    let yAt (x : float) (z : float) =
                        cen.Y - (n.X * (x - cen.X) + n.Z * (z - cen.Z)) / n.Y
                    let x0, x1 = lo.X - padX, hi.X + padX
                    seg (v3 x0 (yAt x0 cen.Z) c.Z) (v3 x1 (yAt x1 cen.Z) c.Z) trace
                    let z0, z1 = lo.Z - padZ, hi.Z + padZ
                    seg (v3 c.X (yAt cen.X z0) z0) (v3 c.X (yAt cen.X z1) z1) trace

            // --- for STRAIGHTEN: a level datum through the picks' own
            //     centroid (like-for-like, rather than against Z = 0) and the
            //     fitted side-view line.
            if picks.Length >= 2 then
                let fit = picks |> Array.map (fun p -> p.X, p.Z) |> Geometry.fitLine2
                seg (v3 (lo.X - padX) c.Y fit.CentroidY)
                    (v3 (hi.X + padX) c.Y fit.CentroidY) datum
                let t = (hi.X - lo.X) * 0.6 + padX
                seg (v3 (fit.CentroidX - fit.DirX * t) c.Y (fit.CentroidY - fit.DirY * t))
                    (v3 (fit.CentroidX + fit.DirX * t) c.Y (fit.CentroidY + fit.DirY * t)) trace
        verts.ToArray()

    let modelMatrix () =
        let r = state.Rotation
        let o = state.Offset
        // System.Numerics is row-vector, so the basis goes in as ROWS of the
        // upper 3x3 and the translation in the last row.
        Matrix4x4(float32 r.R0.X, float32 r.R1.X, float32 r.R2.X, 0.0f,
                  float32 r.R0.Y, float32 r.R1.Y, float32 r.R2.Y, 0.0f,
                  float32 r.R0.Z, float32 r.R1.Z, float32 r.R2.Z, 0.0f,
                  float32 o.X,    float32 o.Y,    float32 o.Z,    1.0f)

    let matricesFor (kind : ViewKind) (aspect : float32) =
        match kind with
        | Free -> orbitMatrices orbit aspect
        | k -> orthoMatrices k state.Bounds aspect

    /// Recomputed on demand rather than cached from the last render, so a
    /// click that arrives before the first frame still lands in the right
    /// panel.
    member this.Panels = panelLayout this.Bounds.Width this.Bounds.Height

    member _.Orbit with get () = orbit and set v = orbit <- v

    /// Point the 3D camera at whatever is loaded now.
    member _.FrameCamera() = orbit <- frameOrbit state.Bounds orbit

    // ---- Blender-style numpad navigation for the 3D panel

    /// Jump to an exact yaw/pitch (numpad 1/3/7 presets). Direct set, not
    /// `rotate`, so the top view can sit exactly at the pole — the orbit
    /// camera's tangent up-vector makes that safe.
    member this.SetOrbitAngles(yaw : float32, pitch : float32) =
        orbit <- { orbit with Yaw = yaw; Pitch = pitch }
        this.RequestNextFrameRendering()

    /// One numpad 2/4/6/8 step, through the same clamped path as a drag.
    member this.OrbitBy(dYaw : float32, dPitch : float32) =
        orbit <- rotate dYaw dPitch orbit
        this.RequestNextFrameRendering()

    /// Numpad 9: the exact opposite view (negated view direction).
    member this.FlipOrbit() =
        orbit <- { orbit with Yaw = orbit.Yaw + MathF.PI; Pitch = -orbit.Pitch }
        this.RequestNextFrameRendering()

    /// Numpad 5. Returns the new state so the window can announce it.
    member this.ToggleProjection() : bool =
        orbit <- { orbit with Orthographic = not orbit.Orthographic }
        this.RequestNextFrameRendering()
        orbit.Orthographic

    /// The mesh changed: re-upload on the next render, where the context is
    /// current. Re-orienting does NOT come through here — that is a uniform.
    member this.InvalidateMesh() =
        meshDirty <- true
        overlayDirty <- true
        this.RequestNextFrameRendering()

    member this.InvalidateOverlay() =
        overlayDirty <- true
        this.RequestNextFrameRendering()

    member this.PanelAt(p : Point) =
        this.Panels |> Array.tryFind (fun panel -> panel.Rect.Contains p)

    /// World-space ray through a point in control coordinates.
    member _.RayAt(panel : Panel, p : Point) =
        let r = panel.Rect
        let aspect = float32 (r.Width / max 1.0 r.Height)
        let view, proj = matricesFor panel.Kind aspect
        let ndc =
            Vector2(float32 ((p.X - r.X) / r.Width * 2.0 - 1.0),
                    float32 (1.0 - (p.Y - r.Y) / r.Height * 2.0))
        cursorRay view proj ndc

    // ------------------------------------------------------------ GL hooks

    override _.OnOpenGlInit(glInterface : GlInterface) =
        // Silk.NET does not create the context — Avalonia already did. This
        // just routes every GL entry point through Avalonia's loader.
        gl <- GL.GetApi(Func<string, nativeint>(fun name -> glInterface.GetProcAddress name))
        programs <- ValueSome(Scene.initPrograms gl)
        sphereBuf <- Scene.makeLitBuffer gl (Scene.sphereVertices ())
        meshDirty <- true

    override _.OnOpenGlDeinit(_ : GlInterface) =
        match programs with
        | ValueSome p ->
            Scene.disposeBuffer gl meshBuf
            Scene.disposeBuffer gl sphereBuf
            Scene.disposeBuffer gl overlayBuf
            Scene.disposeBuffer gl previewGreen
            Scene.disposeBuffer gl previewYellow
            meshBuf <- Scene.empty
            sphereBuf <- Scene.empty
            overlayBuf <- Scene.empty
            previewGreen <- Scene.empty
            previewYellow <- Scene.empty
            previewStampSeen <- -1
            Scene.disposePrograms gl p
            programs <- ValueNone
        | ValueNone -> ()

    override this.OnOpenGlRender(_ : GlInterface, _fb : int) =
        match programs with
        | ValueNone -> ()                     // OnOpenGlInit has not run yet
        | ValueSome progs ->

        if meshDirty then
            Scene.disposeBuffer gl meshBuf
            meshBuf <-
                match state.Mesh with
                | Some m -> Scene.makeLitBuffer gl (Scene.meshVertices m)
                | None -> Scene.empty
            meshDirty <- false

        if overlayDirty then
            Scene.disposeBuffer gl overlayBuf
            overlayBuf <- Scene.makeLineBuffer gl (buildOverlay ())
            overlayDirty <- false

        if previewStampSeen <> state.PreviewStamp then
            Scene.disposeBuffer gl previewGreen
            Scene.disposeBuffer gl previewYellow
            previewGreen <- Scene.empty
            previewYellow <- Scene.empty
            match state.Mesh, state.PreviewFaces with
            | Some m, Some (captured, enclaves) ->
                previewGreen <- Scene.makeLitBuffer gl (Scene.subsetVertices m captured)
                previewYellow <- Scene.makeLitBuffer gl (Scene.subsetVertices m enclaves)
            | _ -> ()
            previewStampSeen <- state.PreviewStamp

        let scaling, fbW, fbH = pixelSize ()
        let panels = this.Panels

        gl.Disable EnableCap.ScissorTest
        gl.Viewport(0, 0, uint32 fbW, uint32 fbH)
        // The gutter colour shows through between panels and reads as a border.
        gl.ClearColor(0.05f, 0.05f, 0.06f, 1.0f)
        gl.Clear(uint32 (GLEnum.ColorBufferBit ||| GLEnum.DepthBufferBit))
        gl.Enable EnableCap.DepthTest
        gl.Enable EnableCap.ScissorTest

        let model = modelMatrix ()
        let markerR = float32 state.MarkerRadius

        for panel in panels do
            let r = panel.Rect
            // Inset by one pixel per side so the gutter is visible, and flip
            // Y: Avalonia measures from the top, GL from the bottom.
            let px = int (r.X * scaling) + 1
            let pw = max 1 (int (r.Width * scaling) - 2)
            let ph = max 1 (int (r.Height * scaling) - 2)
            let py = fbH - int ((r.Y + r.Height) * scaling) + 1
            gl.Viewport(px, py, uint32 pw, uint32 ph)
            gl.Scissor(px, py, uint32 pw, uint32 ph)
            let c = if panel.Kind = Free then bgFree else bg
            gl.ClearColor(c.X, c.Y, c.Z, 1.0f)
            gl.Clear(uint32 (GLEnum.ColorBufferBit ||| GLEnum.DepthBufferBit))

            if meshBuf.VertexCount > 0 then
                let aspect = float32 (float pw / float ph)
                let view, proj = matricesFor panel.Kind aspect
                Scene.drawLit gl progs meshBuf model view proj meshColour

                // Flatten preview: the captured region in green, enclaves in
                // yellow, floated just off the surface with a polygon offset
                // so they read as paint rather than z-fighting with the mesh.
                if previewGreen.VertexCount > 0 || previewYellow.VertexCount > 0 then
                    gl.Enable EnableCap.PolygonOffsetFill
                    gl.PolygonOffset(-1.0f, -1.0f)
                    Scene.drawLit gl progs previewGreen model view proj capturedColour
                    Scene.drawLit gl progs previewYellow model view proj enclaveColour
                    gl.Disable EnableCap.PolygonOffsetFill

                for centre in state.MarkersWorld do
                    let m =
                        Matrix4x4.CreateScale markerR
                        * Matrix4x4.CreateTranslation(
                            Vector3(float32 centre.X, float32 centre.Y, float32 centre.Z))
                    Scene.drawLit gl progs sphereBuf m view proj pickColour

                // Overlays only in the panels they mean something in: the
                // SQUARE traces read in Top and Back (exactly the two views
                // f2s draws them on), the STRAIGHTEN trace in the side view
                // it is fitted in. The 3D panel stays clean.
                let wantsOverlay =
                    match panel.Kind with
                    | Top | Back | Right -> true
                    | Free -> false
                if wantsOverlay then
                    // Depth test off: a datum buried inside the model is no
                    // use as something to compare against.
                    gl.Disable EnableCap.DepthTest
                    Scene.drawLines gl progs overlayBuf (view * proj)
                    gl.Enable EnableCap.DepthTest

        gl.Disable EnableCap.ScissorTest

    // ------------------------------------------------------- input plumbing

    /// Returns true when a pick was added or removed, so the window can
    /// refresh its readout.
    member this.HandlePress(p : Point, button : int) : bool =
        dragStart <- p
        lastPoint <- p
        dragged <- false
        match this.PanelAt p with
        | Some panel when panel.Kind = Free ->
            if button = 3 then
                // Right-click removes the pick nearest what you clicked on.
                if state.RemovePickAlong(this.RayAt(panel, p)) then
                    this.InvalidateOverlay()
                    true
                else false
            else
                dragButton <- button
                false
        | _ -> false

    member this.HandleMove(p : Point) =
        if dragButton <> 0 then
            let dx = float32 (p.X - lastPoint.X)
            let dy = float32 (p.Y - lastPoint.Y)
            lastPoint <- p
            if abs (p.X - dragStart.X) > 3.0 || abs (p.Y - dragStart.Y) > 3.0 then
                dragged <- true
            if dragged then
                let h = float32 (match this.PanelAt p with Some pn -> pn.Rect.Height | None -> 600.0)
                orbit <-
                    if dragButton = 2 then pan dx dy h orbit
                    else rotate (-dx * 0.008f) (dy * 0.008f) orbit
                this.RequestNextFrameRendering()

    /// Returns Some world-point when the release was a click that landed a
    /// pick, so the window can report it.
    member this.HandleRelease(p : Point) : Vec3 option =
        let wasDrag = dragged
        let button = dragButton
        dragButton <- 0
        dragged <- false
        if wasDrag || button <> 1 then None
        else
            match this.PanelAt p with
            | Some panel when panel.Kind = Free ->
                match state.PickAt(this.RayAt(panel, p)) with
                | Some world -> this.InvalidateOverlay(); Some world
                | None -> None
            | _ -> None

    member this.HandleWheel(p : Point, delta : float) =
        match this.PanelAt p with
        | Some panel when panel.Kind = Free ->
            orbit <- zoom (if delta > 0.0 then 0.88f else 1.0f / 0.88f) orbit
            this.RequestNextFrameRendering()
        | _ -> ()
