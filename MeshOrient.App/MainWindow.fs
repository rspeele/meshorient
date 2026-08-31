namespace MeshOrient.App

open System
open System.IO
open System.Threading.Tasks
open Avalonia
open Avalonia.Controls
open Avalonia.Input
open Avalonia.Layout
open Avalonia.Media
open Avalonia.Platform.Storage
open MeshOrient.Core
open MeshOrient.App.Camera

type MainWindow() as this =
    inherit Window()

    let state = OrientState()
    let view = GlView(state)

    // Wrapping, not a StackPanel: a StackPanel hands its child unbounded width
    // in its own orientation, so a long status message runs straight under the
    // fit readout instead of being laid out beside it.
    let status = TextBlock(Margin = Thickness(10.0, 5.0), VerticalAlignment = VerticalAlignment.Center,
                           FontFamily = FontFamily "Consolas,Menlo,monospace", FontSize = 12.0,
                           TextWrapping = TextWrapping.Wrap, MaxLines = 2,
                           TextTrimming = TextTrimming.CharacterEllipsis)
    let fitLabel = TextBlock(Margin = Thickness(14.0, 5.0), VerticalAlignment = VerticalAlignment.Center,
                             FontFamily = FontFamily "Consolas,Menlo,monospace", FontSize = 12.0,
                             TextWrapping = TextWrapping.NoWrap,
                             Foreground = SolidColorBrush(Color.FromRgb(0xFFuy, 0xC0uy, 0x60uy)))

    /// Captions drawn over the GL surface. Avalonia text on top of the
    /// framebuffer, not glyphs rendered in GL — an entire text-rendering
    /// problem avoided for the cost of one Canvas.
    let captionCanvas = Canvas(IsHitTestVisible = false)
    let captions =
        [| Right; Top; Back; Free |]
        |> Array.map (fun kind ->
            let t = TextBlock(FontSize = 11.0, FontFamily = FontFamily "Consolas,Menlo,monospace",
                              Foreground = SolidColorBrush(Color.FromRgb(0x9Auy, 0xA4uy, 0xB0uy)),
                              IsHitTestVisible = false)
            captionCanvas.Children.Add t
            kind, t)

    let setStatus (msg : string) = status.Text <- msg

    let refreshFit () = fitLabel.Text <- state.CurrentFit()

    /// The two alignment buttons, held so their enabled state can follow the
    /// pick count. A button that is offered must be a button that works.
    let mutable btSquare : Button = null
    let mutable btStraighten : Button = null
    let mutable btCenterline : Button = null

    let refreshEnabled () =
        if not (isNull btSquare) then btSquare.IsEnabled <- state.CanSquare
        if not (isNull btStraighten) then btStraighten.IsEnabled <- state.CanStraighten
        if not (isNull btCenterline) then btCenterline.IsEnabled <- state.CanCenterline

    // ------------------------------------------------------------- flatten

    // The two parameter boxes. Values are millimetres; the textbox is the
    // source of truth (each arrow press parses, steps, reformats), and the
    // preview recomputes ONLY on discrete gestures — arrow press, tab-out,
    // Enter, an explicit button, or a pick edit. Never on a drag: a slider
    // that recomputes per pixel of travel is exactly the misfeature this
    // layout exists to avoid.
    let tbFloor = TextBox(Width = 64.0, Margin = Thickness(4.0, 0.0, 10.0, 0.0),
                          FontSize = 12.0, Watermark = "auto")
    let tbCeiling = TextBox(Width = 64.0, Text = "0.3",
                            Margin = Thickness(4.0, 0.0, 10.0, 0.0), FontSize = 12.0)
    let cbForce = CheckBox(Content = "force-flatten enclaves", FontSize = 12.0,
                           Margin = Thickness(0.0, 0.0, 10.0, 0.0),
                           VerticalAlignment = VerticalAlignment.Center)
    let forceOn () = cbForce.IsChecked.GetValueOrDefault false
    // Offered only while the picks' plane sits within a couple of degrees of
    // a true axis-normal plane — i.e. when the face is evidently MEANT to be
    // axis-true and the residual is pick noise. Checked by default: snapped
    // flats come out exactly parallel to each other, so cube booleans in
    // Blender later meet them flush instead of leaving a corner proud. The
    // content names the live axis; hidden means "no axis near, free fit".
    let cbSnap = CheckBox(Content = "snap to true Y plane", FontSize = 12.0,
                          IsChecked = Nullable true, IsVisible = false,
                          Margin = Thickness(0.0, 0.0, 10.0, 0.0),
                          VerticalAlignment = VerticalAlignment.Center)
    let snapOn () = cbSnap.IsChecked.GetValueOrDefault true
    let mutable btFlatten : Button = null
    let mutable btApply : Button = null
    let mutable btDiscard : Button = null

    let parsedParams () =
        match ParamStep.parse tbFloor.Text, ParamStep.parse tbCeiling.Text with
        | Some f, Some c when c > 0.0 -> Some(f, c)
        | _ -> None

    let refreshFlattenBar () =
        (match state.AxisSnapCandidate with
         | Some(ax, _) ->
             cbSnap.Content <- $"""snap to true {"XYZ"[ax]} plane"""
             cbSnap.IsVisible <- true
         | None -> cbSnap.IsVisible <- false)
        let current =
            match parsedParams () with
            | Some(f, c) -> state.PreviewIsCurrent(f, c, forceOn (), snapOn ())
            | None -> false
        if not (isNull btFlatten) then
            btFlatten.IsEnabled <- state.CanFlatten && not current
        if not (isNull btApply) then
            btApply.IsEnabled <- current
        if not (isNull btDiscard) then
            btDiscard.IsEnabled <- state.Preview.IsSome

    let refreshCaptions () =
        let panels = panelLayout view.Bounds.Width view.Bounds.Height
        for kind, t in captions do
            match panels |> Array.tryFind (fun p -> p.Kind = kind) with
            | Some panel ->
                t.Text <- $"{kind.Label}   {kind.Caption}"
                Canvas.SetLeft(t, panel.Rect.X + 6.0)
                Canvas.SetTop(t, panel.Rect.Y + 4.0)
                t.IsVisible <- true
            | None -> t.IsVisible <- false

    /// Everything that has to happen after the model moves or the picks change.
    let refresh () =
        view.InvalidateOverlay()
        refreshFit ()
        refreshEnabled ()
        refreshFlattenBar ()
        refreshCaptions ()

    let button (label : string) (tip : string) (handler : unit -> unit) =
        let b = Button(Content = label, Margin = Thickness(0.0, 0.0, 6.0, 0.0),
                       Padding = Thickness(10.0, 5.0))
        ToolTip.SetTip(b, tip)
        b.Click.Add(fun _ -> handler ())
        b

    let loadFrom (path : string) =
        try
            let m = state.Load path
            view.InvalidateMesh()
            view.FrameCamera()
            refresh ()
            let b = state.Bounds
            let s = Bounds.size b
            setStatus $"{Path.GetFileName path}: {m.TriangleCount} triangles, %.1f{s.X} x %.1f{s.Y} x %.1f{s.Z} mm. Auto-orient, then Square."
        with e ->
            setStatus $"Could not load {Path.GetFileName path}: {e.Message}"

    // ------------------------------------------------------------- actions

    let doAutoOrient () =
        if not state.HasMesh then setStatus "Load a scan first."
        else
            state.AutoOrient()
            view.FrameCamera()
            refresh ()
            let s = Bounds.size state.Bounds
            setStatus $"PCA applied: longest axis to X, thinnest to Y. Now %.1f{s.X} x %.1f{s.Y} x %.1f{s.Z} mm. Check the three views; fix any 90° swap with the rotate buttons."

    let doRotate (axis : int) (deg : float) () =
        if state.HasMesh then
            state.ApplyRotation(Mat3.rotDegrees axis deg)
            view.FrameCamera()
            refresh ()
            setStatus $"""Rotated %g{deg}° about {"XYZ"[axis]}."""

    let doSquare () =
        match state.ApplySquare() with
        | Error msg -> setStatus msg
        | Ok r ->
            view.FrameCamera()
            refresh ()
            setStatus $"Oriented to side: rotated %.3f{r.TiltDegrees}° so the picked face is perpendicular to Y. Your %d{state.Picks.Count} points were coplanar to ±%.3f{r.RmsMm} mm — that is the scan's own flatness. Next: Clear picks, then pick a flat top face and press Y-Spin Face to Level."

    let doStraighten () =
        match state.ApplyStraighten() with
        | Error msg -> setStatus msg
        | Ok r ->
            view.FrameCamera()
            refresh ()
            setStatus $"Straightened: spun %.3f{r.AngleDegrees}° about Y so the picked face is level. Points were collinear to ±%.3f{r.RmsMm} mm. The Y squaring is untouched — rotating about Y is the only move that leaves it alone."

    let doYCenterline () =
        match state.ApplyYCenterline() with
        | Error msg -> setStatus msg
        | Ok r ->
            view.FrameCamera()
            refresh ()
            setStatus $"Centred: shifted Y by %+.3f{r.ShiftY} mm — the two picked faces now sit at ±%.3f{r.HalfWidth} mm about Y=0, ready to slice down the middle. Do this LAST: any later rotation re-centres on the bbox and undoes it."

    let doUndo () =
        if state.Undo() then view.InvalidateMesh()
        view.FrameCamera()
        refresh ()
        setStatus "Undone."

    let previewStatus (pv : MeshOrient.Core.Flatten.Preview) =
        let enc =
            if pv.ForcedVertexCount > 0 then
                $" · %d{pv.EnclaveCount} enclave(s) in YELLOW will be FORCED flat (%d{pv.ForcedVertexCount} verts)"
            elif pv.EnclaveCount > 0 then
                $" · %d{pv.EnclaveCount} enclave(s) in YELLOW — inspect them"
            else ""
        let snap =
            match pv.SnapOffAngleDegrees, state.AxisSnapCandidate with
            | Some off, Some(ax, _) ->
                $""" · snapped to the true {"XYZ"[ax]} plane (free fit was %.2f{off}° off)"""
            | _ -> ""
        $"Preview: would flatten %d{pv.MovedVertexCount} verts across %d{pv.Islands} island(s) · RMS %.3f{pv.RmsBeforeMm} → %.3f{pv.RmsAfterMm} mm · max move %.3f{pv.MaxMoveMm} mm{enc}{snap}. Tweak floor/ceiling and APPLY when it looks right."

    /// Compute or refresh the preview from the boxes. The floor box pre-fills
    /// from the picks' own plane-fit RMS the first time — the scan's measured
    /// noise, not a guess — and is never overwritten once the user has typed.
    let doFlattenPreview () =
        if String.IsNullOrWhiteSpace tbFloor.Text then
            match state.SuggestedFloor with
            | Some f -> tbFloor.Text <- ParamStep.format f
            | None -> ()
        match parsedParams () with
        | None -> setStatus "Floor and ceiling must be numbers (mm), ceiling > 0."
        | Some(f, c) ->
            match state.ComputeFlattenPreview(f, c, forceOn (), snapOn ()) with
            | Error msg -> setStatus msg
            | Ok pv ->
                refresh ()
                setStatus (previewStatus pv)

    /// Re-preview after a discrete gesture, but only when a preview session
    /// is actually live — tweaking boxes before ever pressing FLATTEN does
    /// nothing, which is what makes the boxes safe to explore.
    let syncPreview () =
        if state.Preview.IsSome then
            if state.Picks.Count >= 3 then doFlattenPreview ()
            else
                state.DiscardPreview()
                refresh ()
                setStatus "Preview discarded — fewer than 3 picks left."
        else refreshFlattenBar ()

    let doApplyFlatten () =
        match state.ApplyFlatten() with
        | Error msg -> setStatus msg
        | Ok pv ->
            view.InvalidateMesh()
            refresh ()
            setStatus $"Flattened %d{pv.MovedVertexCount} verts (max move %.3f{pv.MaxMoveMm} mm, RMS now %.3f{pv.RmsAfterMm} mm). Picks kept — re-preview with new numbers, or Clear picks. Undo reverses this."

    let doDiscardPreview () =
        if state.Preview.IsSome then
            state.DiscardPreview()
            refresh ()
            setStatus "Preview discarded. Nothing was changed."

    /// Wire one parameter box: focus selects all, Up/Down arrow steps
    /// (linear above 0.1 mm, halving/doubling below), Enter and tab-out
    /// re-preview if anything changed.
    let wireParamBox (tb : TextBox) =
        tb.GotFocus.Add(fun _ ->
            Avalonia.Threading.Dispatcher.UIThread.Post(fun () -> tb.SelectAll()))
        tb.KeyDown.Add(fun e ->
            match e.Key with
            | Key.Up | Key.Down ->
                e.Handled <- true
                match ParamStep.stepText (e.Key = Key.Up) tb.Text with
                | Some t ->
                    tb.Text <- t
                    tb.SelectAll()
                    syncPreview ()
                | None -> ()
            | Key.Enter ->
                e.Handled <- true
                syncPreview ()
            | _ -> ())
        tb.LostFocus.Add(fun _ -> syncPreview ())

    let doClearPicks () =
        state.ClearPicks()
        refresh ()
        setStatus "Picks cleared."

    let doExport () =
        if not state.HasMesh then setStatus "Load a scan first."
        else
            try
                let oriented, cleaned = state.ExportStl()
                let frame =
                    if state.Rotation = MeshOrient.Core.Mat3.identity
                       && MeshOrient.Core.Vec3.length state.Offset < 1e-12 then
                        "in the scan's ORIGINAL coordinates (nothing was moved)"
                    else "on the oriented datum"
                match cleaned with
                | Some c ->
                    setStatus $"Wrote 2 files {frame}: {Path.GetFileName oriented} (orientation only — the scan untouched) and {Path.GetFileName c} (%d{state.FlattenedVertexCount} verts flattened). Same {state.TriangleCount} triangles in both."
                | None ->
                    setStatus $"Wrote {oriented} {frame} — the same {state.TriangleCount} triangles, rigidly transformed. Nothing resampled, so edges and hole boundaries are exactly as scanned."
            with e ->
                setStatus $"Export failed: {e.Message}"

    let openDialog () =
        task {
            let! files =
                this.StorageProvider.OpenFilePickerAsync(
                    FilePickerOpenOptions(
                        Title = "Open a scan",
                        AllowMultiple = false,
                        FileTypeFilter =
                            [| FilePickerFileType("Meshes",
                                Patterns = [| "*.stl"; "*.obj"; "*.ply" |]) |]))
            match Seq.tryHead files with
            | Some f -> loadFrom (f.Path.LocalPath)
            | None -> ()
        } :> Task

    do
        this.Title <- "meshorient"
        this.Width <- 1280.0
        this.Height <- 820.0
        this.Background <- SolidColorBrush(Color.FromRgb(0x1Euy, 0x1Fuy, 0x22uy))

        // No stage selector. Its only real job was routing clicks into one of
        // two pick lists, which is exactly how STRAIGHTEN came to report "no
        // points selected" with points on screen. One list, both buttons, and
        // each button greys itself out until it has what it needs.
        let toolbar = StackPanel(Orientation = Orientation.Horizontal, Margin = Thickness(10.0, 8.0))
        toolbar.Children.Add(button "Open…" "Load an STL, OBJ or PLY (or drag one onto the window)"
                                    (fun () -> openDialog () |> ignore))
        toolbar.Children.Add(button "Auto-orient" "PCA: longest axis to X, thinnest to Y" doAutoOrient)
        toolbar.Children.Add(button "X 90°" "Rotate 90° about X" (doRotate 0 90.0))
        toolbar.Children.Add(button "Y 90°" "Rotate 90° about Y" (doRotate 1 90.0))
        toolbar.Children.Add(button "Z 90°" "Rotate 90° about Z" (doRotate 2 90.0))
        toolbar.Children.Add(button "Flip" "Turn end for end (180° about Z)" (doRotate 2 180.0))
        toolbar.Children.Add(Border(Width = 1.0, Margin = Thickness(4.0, 2.0, 10.0, 2.0),
                                    Background = SolidColorBrush(Color.FromRgb(0x3Auy, 0x3Cuy, 0x42uy))))
        btSquare <- button "Orient Face to Side"
                        "Square the picked face to Y — needs 3+ points on one flat SIDE face"
                        doSquare
        btStraighten <- button "Y-Spin Face to Level"
                            "Spin about Y until the picked face is level — needs 2+ points on a flat TOP or BOTTOM face"
                            doStraighten
        btCenterline <- button "Y-Centerline"
                            ("Translate in Y so the two picked faces sit symmetric about Y=0 — pick point(s) on the "
                             + "RIGHT face and on the LEFT face (extra picks per side average the noise out). "
                             + "Do this last; rotations re-centre on the bbox.")
                            doYCenterline
        toolbar.Children.Add btSquare
        toolbar.Children.Add btStraighten
        toolbar.Children.Add btCenterline
        toolbar.Children.Add(button "Clear picks" "Drop every picked point" doClearPicks)
        toolbar.Children.Add(button "Undo" "Step back one orientation change" doUndo)
        toolbar.Children.Add(Border(Width = 1.0, Margin = Thickness(4.0, 2.0, 10.0, 2.0),
                                    Background = SolidColorBrush(Color.FromRgb(0x3Auy, 0x3Cuy, 0x42uy))))
        toolbar.Children.Add(button "Export STL"
                                    ("Write <name>_oriented.stl (orientation only, never flattened) and, when flattens "
                                     + "are applied, <name>_cleaned.stl (with them) beside the source")
                                    doExport)

        // ---- flatten bar: its own row so the preview loop reads as one unit
        let label (text : string) =
            TextBlock(Text = text, VerticalAlignment = VerticalAlignment.Center,
                      FontSize = 12.0, Margin = Thickness(0.0, 0.0, 2.0, 0.0),
                      Foreground = SolidColorBrush(Color.FromRgb(0x9Auy, 0xA4uy, 0xB0uy)))
        btFlatten <- button "Flatten Face"
                        ("Flood-fill the flat under your picks and PREVIEW the snap: captured faces green, "
                         + "surrounded-but-not-captured pockets yellow. Nothing moves until APPLY.")
                        doFlattenPreview
        btApply <- button "APPLY"
                       "Commit the previewed flatten to the mesh (Undo reverses it)"
                       doApplyFlatten
        btDiscard <- button "Discard"
                         "Drop the preview without changing anything (Esc)"
                         doDiscardPreview
        wireParamBox tbFloor
        wireParamBox tbCeiling
        ToolTip.SetTip(cbForce,
            "Snap the YELLOW pockets flat too, however far out their verts sit — for scanner "
            + "blobs and dents living inside the flat. Leave off when an enclave is a real feature.")
        ToolTip.SetTip(cbSnap,
            "Your picks' plane is within a fraction of a degree of a true axis-normal plane, "
            + "so it is presumably meant to BE one. Checked: flatten onto that exact plane, so "
            + "every such flat comes out parallel and later cube booleans sit flush. "
            + "Unchecked: flatten onto the fitted plane exactly as detected.")
        // A checkbox click is a discrete gesture, same class as an arrow press:
        // a live preview follows it.
        cbForce.IsCheckedChanged.Add(fun _ -> syncPreview ())
        cbSnap.IsCheckedChanged.Add(fun _ -> syncPreview ())
        ToolTip.SetTip(tbFloor,
            "Noise floor, mm: distances up to this snap fully. Pre-fills from your picks' "
            + "plane-fit RMS ×3. ↑/↓ steps ±0.1 above 0.1, halves/doubles below.")
        ToolTip.SetTip(tbCeiling,
            "Ceiling, mm: the capture limit k. The snap feathers to zero approaching it, "
            + "so curves stay smooth. ↑/↓ steps ±0.1 above 0.1, halves/doubles below.")
        let flattenBar = StackPanel(Orientation = Orientation.Horizontal,
                                    Margin = Thickness(10.0, 0.0, 10.0, 8.0))
        flattenBar.Children.Add btFlatten
        flattenBar.Children.Add(label "floor mm")
        flattenBar.Children.Add tbFloor
        flattenBar.Children.Add(label "ceiling mm")
        flattenBar.Children.Add tbCeiling
        flattenBar.Children.Add cbSnap
        flattenBar.Children.Add cbForce
        flattenBar.Children.Add btApply
        flattenBar.Children.Add btDiscard

        // OpenGlControlBase has no Background, so Avalonia's hit-tester treats
        // it as transparent no matter what is in the framebuffer. The Border
        // is what actually receives the pointer.
        let glHost = Border(Background = Brushes.Transparent)
        let glStack = Grid()
        glStack.Children.Add view
        glStack.Children.Add captionCanvas
        glHost.Child <- glStack

        let bottom = Grid(Background = SolidColorBrush(Color.FromRgb(0x17uy, 0x18uy, 0x1Buy)))
        bottom.ColumnDefinitions.Add(ColumnDefinition(GridLength(1.0, GridUnitType.Star)))
        bottom.ColumnDefinitions.Add(ColumnDefinition(GridLength.Auto))
        Grid.SetColumn(status, 0)
        bottom.Children.Add status
        Grid.SetColumn(fitLabel, 1)
        bottom.Children.Add fitLabel

        let root = Grid()
        root.RowDefinitions.Add(RowDefinition(GridLength.Auto))
        root.RowDefinitions.Add(RowDefinition(GridLength.Auto))
        root.RowDefinitions.Add(RowDefinition(GridLength(1.0, GridUnitType.Star)))
        root.RowDefinitions.Add(RowDefinition(GridLength.Auto))
        Grid.SetRow(toolbar, 0)
        root.Children.Add toolbar
        Grid.SetRow(flattenBar, 1)
        root.Children.Add flattenBar
        Grid.SetRow(glHost, 2)
        root.Children.Add glHost
        Grid.SetRow(bottom, 3)
        root.Children.Add bottom
        this.Content <- root

        // ------------------------------------------------------------ input

        // An exception out of a pointer handler is unhandled and takes the
        // window down with it, losing an orientation that may have taken
        // several minutes of picking to build. One did exactly that: the
        // readout threw on the very first pick. Report it and carry on — the
        // state is still good, and a message in the status bar is a bug
        // report rather than a vanished window.
        let guarded (what : string) (body : unit -> unit) =
            try body () with e -> setStatus $"{what} failed: {e.Message}"

        glHost.PointerPressed.Add(fun e ->
            guarded "Click" (fun () ->
                let p = e.GetCurrentPoint glHost
                let button =
                    if p.Properties.IsRightButtonPressed then 3
                    elif p.Properties.IsMiddleButtonPressed then 2
                    else 1
                if view.HandlePress(p.Position, button) then
                    refresh ()
                    setStatus $"Pick removed — %d{state.Picks.Count} left."
                    syncPreview ()))

        glHost.PointerMoved.Add(fun e ->
            guarded "Drag" (fun () -> view.HandleMove((e.GetCurrentPoint glHost).Position)))

        glHost.PointerReleased.Add(fun e ->
            guarded "Pick" (fun () ->
                match view.HandleRelease((e.GetCurrentPoint glHost).Position) with
                | Some world ->
                    refresh ()
                    setStatus $"Point %d{state.Picks.Count} at X %.2f{world.X}, Y %.2f{world.Y}, Z %.2f{world.Z}."
                    // A pick click is as deliberate a gesture as an arrow
                    // press: a live preview follows it rather than lying.
                    syncPreview ()
                | None -> ()))

        glHost.PointerWheelChanged.Add(fun e ->
            guarded "Zoom" (fun () ->
                view.HandleWheel((e.GetCurrentPoint glHost).Position, e.Delta.Y)))

        // Blender-style numpad navigation for the 3D panel. Guarded so keys
        // typed into a parameter box stay text entry: the routed event's
        // Source is the focused control.
        this.KeyDown.Add(fun e ->
            let inTextBox = (e.Source :? TextBox)
            if not inTextBox then
                let step = MathF.PI / 12.0f              // 15° per press, like Blender
                match e.Key with
                | Key.Escape -> e.Handled <- true; doDiscardPreview ()
                // Presets. Blender muscle memory mapped onto this world
                // (+X muzzle, +Z up, -Y the gun's right):
                //   1 = camera on -Y  -> the gun's RIGHT (same as the R panel)
                //   3 = camera on +X  -> muzzle-on
                //   7 = top, +Y up on screen, exactly at the pole (the orbit
                //       camera's tangent up-vector makes that legal)
                | Key.NumPad1 -> e.Handled <- true; view.SetOrbitAngles(-MathF.PI / 2.0f, 0.0f)
                | Key.NumPad3 -> e.Handled <- true; view.SetOrbitAngles(0.0f, 0.0f)
                | Key.NumPad7 -> e.Handled <- true; view.SetOrbitAngles(-MathF.PI / 2.0f, MathF.PI / 2.0f)
                | Key.NumPad9 -> e.Handled <- true; view.FlipOrbit()
                | Key.NumPad4 -> e.Handled <- true; view.OrbitBy(step, 0.0f)
                | Key.NumPad6 -> e.Handled <- true; view.OrbitBy(-step, 0.0f)
                | Key.NumPad8 -> e.Handled <- true; view.OrbitBy(0.0f, step)
                | Key.NumPad2 -> e.Handled <- true; view.OrbitBy(0.0f, -step)
                | Key.NumPad5 ->
                    e.Handled <- true
                    let ortho = view.ToggleProjection()
                    setStatus (if ortho then "3D view: orthographic." else "3D view: perspective.")
                | _ -> ())

        // Captions have to follow the panels, and the panels are laid out from
        // the control's size, so re-place them whenever that changes.
        glHost.LayoutUpdated.Add(fun _ -> refreshCaptions ())

        // Drag and drop a scan straight onto the window.
        DragDrop.SetAllowDrop(this, true)
        this.AddHandler(DragDrop.DragOverEvent, fun _ (e : DragEventArgs) ->
            e.DragEffects <-
                if e.DataTransfer.Contains DataFormat.File then DragDropEffects.Copy
                else DragDropEffects.None)
        this.AddHandler(DragDrop.DropEvent, fun _ (e : DragEventArgs) ->
            match e.DataTransfer.TryGetFile() with
            | null -> ()
            | f -> loadFrom f.Path.LocalPath)

        refreshEnabled ()
        setStatus "Open a scan (or drag one in). Then: Auto-orient · pick 3+ on a flat SIDE face → Orient Face to Side · Clear picks · pick 2+ on a flat TOP face → Y-Spin Face to Level · Export STL."

    /// Load a file named on the command line, once the window exists.
    member _.LoadInitial(path : string) =
        if File.Exists path then loadFrom path
        else setStatus $"No such file: {path}"
