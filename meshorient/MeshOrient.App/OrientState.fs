/// The tool's whole model: a mesh, the rigid transform being built for it, and
/// the points picked for each alignment stage.
///
/// No Avalonia and no GL here, so every operation the buttons invoke can be
/// exercised without a window.
namespace MeshOrient.App

open System
open MeshOrient.Core

type Stage =
    /// PCA and the 90-degree buttons.
    | Coarse
    /// Pick 3+ on a flat side face; square it to Y.
    | Square
    /// Pick 2+ on a flat top or bottom face; spin about Y until it is level.
    | Straighten

/// A picked point, in MODEL space. The surface normal comes along so the
/// marker can be nudged clear of the face it sits on instead of z-fighting
/// with it.
type Pick = { Point : Vec3; Normal : Vec3 }

/// The axis the Square stage targets and the Straighten stage rotates about.
/// Hard-coded to Y (across the frame) — the core functions both take an axis
/// index, so exposing it is a UI change and nothing more.
[<AutoOpen>]
module private Constants =
    let lockedAxis = 1

type OrientState() =

    let mutable mesh : Mesh option = None
    let mutable rotation = Mat3.identity
    let mutable offset = Vec3.zero
    let mutable bounds = Bounds.empty
    let undo = System.Collections.Generic.Stack<Mat3 * Vec3>()

    // Picks live in MODEL space, not world space. That means a re-orientation
    // carries them along for free: press Square twice and the second reading
    // is 0.00 degrees, because the picks moved with the mesh they were taken
    // on. f2s has to do this by hand (`level_pts` are rewritten through every
    // transform); here it falls out of the representation.
    let squarePicks = ResizeArray<Pick>()
    let straightenPicks = ResizeArray<Pick>()

    let mutable stage = Coarse

    let reframe () =
        match mesh with
        | Some m -> bounds <- Mesh.transformedBounds rotation offset m
        | None -> bounds <- Bounds.empty

    /// Put the bounding-box centre back on the origin. Run after every
    /// rotation, as f2s does: it keeps the orthographic panels framed and the
    /// orbit pivot on the model instead of drifting off into space.
    let recentre () =
        match mesh with
        | Some m ->
            let b = Mesh.transformedBounds rotation Vec3.zero m
            offset <- -(Bounds.centre b)
            reframe ()
        | None -> ()

    member _.Mesh = mesh
    member _.HasMesh = mesh.IsSome
    member _.Rotation = rotation
    member _.Offset = offset
    member _.Bounds = bounds
    member _.Stage with get () = stage and set v = stage <- v
    member _.CanUndo = undo.Count > 0

    member _.SourcePath = match mesh with Some m -> m.SourcePath | None -> ""

    member _.TriangleCount = match mesh with Some m -> m.TriangleCount | None -> 0

    /// Picks for the stage currently selected, in model space.
    member _.ActivePicks =
        match stage with
        | Straighten -> straightenPicks
        | _ -> squarePicks

    member private _.ToWorld(p : Pick) = Mat3.apply rotation p.Point + offset

    /// Marker centres: the hit point nudged a little way out along the surface
    /// normal so the sphere sits proud of the face rather than half-buried in
    /// it, z-fighting.
    member this.ActiveMarkersWorld =
        let r = this.MarkerRadius * 0.5
        this.ActivePicks
        |> Seq.map (fun p -> Mat3.apply rotation (p.Point + p.Normal * r) + offset)
        |> Seq.toArray

    member this.ActivePicksWorld =
        this.ActivePicks |> Seq.map this.ToWorld |> Seq.toArray

    member this.SquarePicksWorld = squarePicks |> Seq.map this.ToWorld |> Seq.toArray

    /// How big a pick marker should be so it reads the same on a 20 mm part
    /// and a 400 mm one.
    member _.MarkerRadius =
        let d = Bounds.diagonal bounds
        if d <= 0.0 then 1.0 else d / 150.0

    // ------------------------------------------------------------ loading

    member _.Load(path : string) =
        let m = MeshIO.load path
        mesh <- Some m
        rotation <- Mat3.identity
        offset <- Vec3.zero
        squarePicks.Clear()
        straightenPicks.Clear()
        undo.Clear()
        recentre ()
        m

    // --------------------------------------------------------- transforms

    member private _.PushUndo() =
        undo.Push(rotation, offset)
        // A handful of steps is all anyone backtracks; unbounded growth on a
        // long session is not worth the memory.
        while undo.Count > 32 do undo.Pop() |> ignore

    /// Compose an extra world-space rotation on top of what we have, then
    /// re-centre. Picks are untouched: they are model-space, so they follow.
    member this.ApplyRotation(r : Mat3) =
        if mesh.IsSome then
            this.PushUndo()
            rotation <- Mat3.mul r rotation
            recentre ()

    member this.Undo() =
        if undo.Count > 0 then
            let r, o = undo.Pop()
            rotation <- r
            offset <- o
            reframe ()

    member this.AutoOrient() =
        match mesh with
        | None -> ()
        | Some m ->
            this.PushUndo()
            // PCA is a property of the mesh as it sits in MODEL space, so it
            // replaces the accumulated rotation rather than composing onto it.
            // Composing would mean pressing the button twice moved the model
            // twice, which is not what "auto-orient" can possibly mean.
            rotation <- Geometry.autoOrient m
            recentre ()

    member this.Recentre() =
        if mesh.IsSome then
            this.PushUndo()
            recentre ()

    // -------------------------------------------------------------- picks

    /// Where a world-space ray meets the mesh, in MODEL space.
    ///
    /// The ray is transformed into model space rather than the mesh into world
    /// space: the mesh is never moved, in memory or on the GPU — only the
    /// matrix around it — so this is the only direction that costs nothing.
    member private _.Trace(ray : Raycast.Ray) : Raycast.Hit option =
        match mesh with
        | None -> None
        | Some m ->
            let inv = Mat3.transpose rotation
            Raycast.intersect m
                {   Raycast.Origin = Mat3.apply inv (ray.Origin - offset)
                    Raycast.Direction = Mat3.apply inv ray.Direction }

    /// Cast a world-space ray at the mesh and record where it lands.
    /// Returns the world-space hit point, or None if the ray missed.
    member this.PickAt(ray : Raycast.Ray) : Vec3 option =
        match this.Trace ray with
        | None -> None
        | Some hit ->
            this.ActivePicks.Add { Point = hit.Point; Normal = hit.Normal }
            Some(Mat3.apply rotation hit.Point + offset)

    /// Remove whichever pick is nearest where this ray hits the mesh.
    member this.RemovePickAlong(ray : Raycast.Ray) : bool =
        match this.Trace ray with
        | None -> false
        | Some hit -> this.RemoveNearestPick(Mat3.apply rotation hit.Point + offset)

    /// Drop the pick nearest a world-space point, for right-click-to-remove.
    member this.RemoveNearestPick(world : Vec3) =
        let picks = this.ActivePicks
        if picks.Count = 0 then false
        else
            let mutable best, bestD = -1, infinity
            for i in 0 .. picks.Count - 1 do
                let d = Vec3.lengthSq (Mat3.apply rotation picks[i].Point + offset - world)
                if d < bestD then bestD <- d; best <- i
            picks.RemoveAt best
            true

    member this.ClearPicks() = this.ActivePicks.Clear()

    // ------------------------------------------------------------- stages

    /// Square the picked face to Y. Returns the measurement, or an error to
    /// show the user.
    member this.ApplySquare() : Result<Orient.SquareResult, string> =
        let picks = this.SquarePicksWorld
        if picks.Length < 3 then
            Error $"Pick at least 3 points on one flat side face first ({picks.Length} so far)."
        else
            let r = Orient.squareToAxis lockedAxis picks
            this.ApplyRotation r.Rotation
            Ok r

    /// Spin about Y until the picked top/bottom face is level.
    member this.ApplyStraighten() : Result<Orient.StraightenResult, string> =
        let picks = straightenPicks |> Seq.map this.ToWorld |> Seq.toArray
        if picks.Length < 2 then
            Error $"Pick at least 2 points on a flat top or bottom face first ({picks.Length} so far)."
        else
            let r = Orient.straightenAbout lockedAxis picks
            this.ApplyRotation r.Rotation
            Ok r

    /// The current fit for whichever stage is selected, for the live readout
    /// and the overlay traces. None when there are not enough picks yet.
    member this.CurrentFit() : string option =
        match stage with
        | Straighten when straightenPicks.Count >= 2 ->
            let r = Orient.straightenAbout lockedAxis (this.ActivePicksWorld)
            Some $"%d{straightenPicks.Count} picks · off level by %.3f{r.AngleDegrees}° · collinear to ±%.3f{r.RmsMm} mm"
        | Straighten -> None
        | _ when squarePicks.Count >= 3 ->
            let r = Orient.squareToAxis lockedAxis (this.SquarePicksWorld)
            let ax, az = Orient.tiltComponents (this.SquarePicksWorld)
            Some $"%d{squarePicks.Count} picks · tilt %.3f{r.TiltDegrees}° (about X %.2f{ax}°, about Z %.2f{az}°) · coplanar to ±%.3f{r.RmsMm} mm"
        | _ -> None

    // ------------------------------------------------------------- export

    member _.ExportOrientedStl() : string =
        match mesh with
        | None -> failwith "nothing loaded"
        | Some m ->
            let out = MeshIO.orientedPathFor m.SourcePath
            MeshIO.saveStlBinary out (Mesh.transform rotation offset m)
            out
