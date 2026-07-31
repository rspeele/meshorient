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

    let refreshEnabled () =
        if not (isNull btSquare) then btSquare.IsEnabled <- state.CanSquare
        if not (isNull btStraighten) then btStraighten.IsEnabled <- state.CanStraighten

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
            setStatus $"Squared: rotated %.3f{r.TiltDegrees}° so the picked face is perpendicular to Y. Your %d{state.Picks.Count} points were coplanar to ±%.3f{r.RmsMm} mm — that is the scan's own flatness. Next: Clear picks, then pick a flat top face and press STRAIGHTEN."

    let doStraighten () =
        match state.ApplyStraighten() with
        | Error msg -> setStatus msg
        | Ok r ->
            view.FrameCamera()
            refresh ()
            setStatus $"Straightened: spun %.3f{r.AngleDegrees}° about Y so the picked face is level. Points were collinear to ±%.3f{r.RmsMm} mm. The Y squaring is untouched — rotating about Y is the only move that leaves it alone."

    let doUndo () =
        state.Undo()
        view.FrameCamera()
        refresh ()
        setStatus "Undone."

    let doClearPicks () =
        state.ClearPicks()
        refresh ()
        setStatus "Picks cleared."

    let doExport () =
        if not state.HasMesh then setStatus "Load a scan first."
        else
            try
                let out = state.ExportOrientedStl()
                setStatus $"Wrote {out} — the same {state.TriangleCount} triangles, rigidly transformed. Nothing resampled, so edges and hole boundaries are exactly as scanned."
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
        btSquare <- button "SQUARE"
                        "Square the picked face to Y — needs 3+ points on one flat SIDE face"
                        doSquare
        btStraighten <- button "STRAIGHTEN"
                            "Spin about Y until the picked face is level — needs 2+ points on a flat TOP or BOTTOM face"
                            doStraighten
        toolbar.Children.Add btSquare
        toolbar.Children.Add btStraighten
        toolbar.Children.Add(button "Clear picks" "Drop every picked point" doClearPicks)
        toolbar.Children.Add(button "Undo" "Step back one orientation change" doUndo)
        toolbar.Children.Add(Border(Width = 1.0, Margin = Thickness(4.0, 2.0, 10.0, 2.0),
                                    Background = SolidColorBrush(Color.FromRgb(0x3Auy, 0x3Cuy, 0x42uy))))
        toolbar.Children.Add(button "Export oriented STL" "Write <name>_oriented.stl beside the source" doExport)

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
        root.RowDefinitions.Add(RowDefinition(GridLength(1.0, GridUnitType.Star)))
        root.RowDefinitions.Add(RowDefinition(GridLength.Auto))
        Grid.SetRow(toolbar, 0)
        root.Children.Add toolbar
        Grid.SetRow(glHost, 1)
        root.Children.Add glHost
        Grid.SetRow(bottom, 2)
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
                    setStatus $"Pick removed — %d{state.Picks.Count} left."))

        glHost.PointerMoved.Add(fun e ->
            guarded "Drag" (fun () -> view.HandleMove((e.GetCurrentPoint glHost).Position)))

        glHost.PointerReleased.Add(fun e ->
            guarded "Pick" (fun () ->
                match view.HandleRelease((e.GetCurrentPoint glHost).Position) with
                | Some world ->
                    refresh ()
                    setStatus $"Point %d{state.Picks.Count} at X %.2f{world.X}, Y %.2f{world.Y}, Z %.2f{world.Z}."
                | None -> ()))

        glHost.PointerWheelChanged.Add(fun e ->
            guarded "Zoom" (fun () ->
                view.HandleWheel((e.GetCurrentPoint glHost).Position, e.Delta.Y)))

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
        setStatus "Open a scan (or drag one in). Then: Auto-orient · pick 3+ on a flat SIDE face and press SQUARE · Clear picks · pick 2+ on a flat TOP face and press STRAIGHTEN · Export."

    /// Load a file named on the command line, once the window exists.
    member _.LoadInitial(path : string) =
        if File.Exists path then loadFrom path
        else setStatus $"No such file: {path}"
