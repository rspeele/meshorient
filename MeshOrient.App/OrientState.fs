/// The tool's whole model: a mesh, the rigid transform being built for it, and
/// the points picked for whatever alignment comes next.
///
/// There is NO mode. One pick list, and every operation reads it: squaring
/// (Orient Face to Side) wants 3+ points on a flat side face, straightening
/// (Y-Spin Face to Level) wants 2+ on a flat top or bottom, and so on.
/// Nothing has to be told which you meant, so nothing can be set wrong — a
/// selector routing clicks into separate hidden lists would let a button read
/// a different list from the points on screen.
///
/// No Avalonia and no GL here, so every operation the buttons invoke can be
/// exercised without a window.
namespace MeshOrient.App

open System
open MeshOrient.Core

/// A picked point, in MODEL space. The surface normal comes along so the
/// marker can be nudged clear of the face it sits on instead of z-fighting
/// with it; the triangle index is the seed FLATTEN floods from.
type Pick = { Point : Vec3; Normal : Vec3; Triangle : int }

/// One reversible step. Rigid transforms store the nine numbers they replace;
/// a flatten stores only the vertices it moved (region-sized, so a deep stack
/// of them stays cheap — snapshotting the whole 2.4M-vert array would not).
type private UndoEntry =
    | Rigid of Mat3 * Vec3
    /// The (vertex index, old position) pairs to restore — one per soup
    /// copy — plus the canonical vertex count the flatten added to
    /// `flattenedVerts`, which is what undoing it must take back off.
    | Reshape of restore : (int * Vec3)[] * movedVertexCount : int

/// The axis squaring targets and straightening rotates about. Hard-coded to
/// Y — the core functions both take an axis index, so exposing it is a UI
/// change and nothing more.
[<AutoOpen>]
module private Constants =
    let lockedAxis = 1

    /// How many steps Undo can go back.
    let maxUndo = 32

type OrientState() =

    let mutable mesh : Mesh option = None
    let mutable rotation = Mat3.identity
    let mutable offset = Vec3.zero
    let mutable bounds = Bounds.empty
    /// Newest step last. A linked list rather than a Stack so the step that
    /// falls off when the history is full is the OLDEST one.
    let undo = System.Collections.Generic.LinkedList<UndoEntry>()

    // FLATTEN state. Adjacency is the exact-bit weld + face graph — built on
    // the first preview, cached for the life of the mesh (moving verts never
    // changes the topology it encodes). The preview is MODEL-space, so a
    // rotation leaves it valid — unless it snapped to a world axis, which
    // `setRotation` handles. Pick edits and parameter edits stale it too,
    // which is what the version counters track.
    let mutable adjacency : Flatten.Adjacency option = None
    let mutable preview : Flatten.Preview option = None
    let mutable previewParams = (nan, nan, false, false)
    let mutable previewPicksVersion = -1
    let mutable picksVersion = 0
    let mutable previewStamp = 0
    let mutable flattenedVerts = 0            // net total baked into the mesh
    /// The vertex array exactly as loaded. Flattens replace the working
    /// array (never mutate it), so holding the original reference costs
    /// nothing and lets the export always offer an un-mutated `_oriented`
    /// alongside the flattened `_cleaned`.
    let mutable pristineVerts : Vec3[] = [||]

    // Picks live in MODEL space, not world space. That means a re-orientation
    // carries them along for free: press Orient Face to Side twice and the
    // second reading is 0.000 degrees, because the picks moved with the mesh
    // they were taken on — it falls out of the representation.
    let picks = ResizeArray<Pick>()

    let picksChanged () = picksVersion <- picksVersion + 1

    let dropPreview () =
        if preview.IsSome then
            preview <- None
            previewStamp <- previewStamp + 1

    /// Every rotation change after load goes through here. A snapped preview
    /// is dropped: its snap direction was "true X/Y/Z" under the OLD rotation,
    /// and applying it after a turn would flatten onto a plane that is no
    /// longer axis-true — the very thing the snap exists to prevent.
    let setRotation (r : Mat3) =
        if r <> rotation then
            rotation <- r
            match preview with
            | Some pv when pv.SnapOffAngleDegrees.IsSome -> dropPreview ()
            | _ -> ()

    let reframe () =
        match mesh with
        | Some m -> bounds <- Mesh.transformedBounds rotation offset m
        | None -> bounds <- Bounds.empty

    /// Put the bounding-box centre back on the origin. Run after every
    /// ROTATION — and only then. Loading does NOT centre: a scan that is only
    /// flattened and re-exported must come back in the coordinate frame it
    /// arrived in, because the user's other Blender objects are registered
    /// against that frame. The views never need it — panels and orbit both
    /// frame on Bounds.centre wherever the model sits. The first orientation
    /// command is the moment the source frame stops being meaningful, so
    /// that is when centring starts.
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
    member _.CanUndo = undo.Count > 0

    /// Whether each operation has enough picks to run. The window greys its
    /// buttons on these, so a button is never offered that can only fail.
    member _.CanSquare = picks.Count >= 3
    member _.CanStraighten = picks.Count >= 2

    member _.SourcePath = match mesh with Some m -> m.SourcePath | None -> ""

    member _.TriangleCount = match mesh with Some m -> m.TriangleCount | None -> 0

    /// Every picked point, in model space. One list, both operations.
    member _.Picks = picks

    member private _.ToWorld(p : Pick) = Mat3.apply rotation p.Point + offset

    /// Marker centres: the hit point nudged a little way out along the surface
    /// normal so the sphere sits proud of the face rather than half-buried in
    /// it, z-fighting.
    member this.MarkersWorld =
        let r = this.MarkerRadius * 0.5
        picks
        |> Seq.map (fun p -> Mat3.apply rotation (p.Point + p.Normal * r) + offset)
        |> Seq.toArray

    member this.PicksWorld = picks |> Seq.map this.ToWorld |> Seq.toArray

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
        picks.Clear()
        picksChanged ()
        undo.Clear()
        adjacency <- None
        dropPreview ()
        flattenedVerts <- 0
        pristineVerts <- m.Vertices
        reframe ()                    // frame it where it IS — no centring
        m

    // --------------------------------------------------------- transforms

    member private _.PushUndo(entry : UndoEntry) =
        undo.AddLast entry |> ignore
        // A handful of steps is all anyone backtracks; unbounded growth on a
        // long session is not worth the memory.
        while undo.Count > maxUndo do undo.RemoveFirst()

    /// Compose an extra world-space rotation on top of what we have, then
    /// re-centre. Picks are untouched: they are model-space, so they follow.
    member this.ApplyRotation(r : Mat3) =
        if mesh.IsSome then
            this.PushUndo(Rigid(rotation, offset))
            setRotation (Mat3.mul r rotation)
            recentre ()

    /// Returns true when the mesh's VERTICES changed (a flatten was undone),
    /// so the caller knows the GPU copy is stale — a rigid undo only moves
    /// the model matrix and costs nothing.
    member this.Undo() : bool =
        if undo.Count = 0 then false
        else
            let entry = undo.Last.Value
            undo.RemoveLast()
            match entry with
            | Rigid(r, o) ->
                setRotation r
                offset <- o
                reframe ()
                false
            | Reshape(old, moved) ->
                match mesh with
                | None -> false
                | Some m ->
                    let verts = Array.copy m.Vertices
                    for (i, pos) in old do verts[i] <- pos
                    mesh <- Some { m with Vertices = verts }
                    flattenedVerts <- flattenedVerts - moved
                    dropPreview ()
                    reframe ()
                    true

    member this.AutoOrient() =
        match mesh with
        | None -> ()
        | Some m ->
            this.PushUndo(Rigid(rotation, offset))
            // PCA is a property of the mesh as it sits in MODEL space, so it
            // replaces the accumulated rotation rather than composing onto it.
            // Composing would mean pressing the button twice moved the model
            // twice, which is not what "auto-orient" can possibly mean.
            setRotation (Geometry.autoOrient m)
            recentre ()

    member this.Recentre() =
        if mesh.IsSome then
            this.PushUndo(Rigid(rotation, offset))
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
            picks.Add { Point = hit.Point; Normal = hit.Normal; Triangle = hit.Triangle }
            picksChanged ()
            Some(Mat3.apply rotation hit.Point + offset)

    /// Remove whichever pick is nearest where this ray hits the mesh.
    member this.RemovePickAlong(ray : Raycast.Ray) : bool =
        match this.Trace ray with
        | None -> false
        | Some hit -> this.RemoveNearestPick(Mat3.apply rotation hit.Point + offset)

    /// Drop the pick nearest a world-space point, for right-click-to-remove.
    member _.RemoveNearestPick(world : Vec3) =
        if picks.Count = 0 then false
        else
            let mutable best, bestD = -1, infinity
            for i in 0 .. picks.Count - 1 do
                let d = Vec3.lengthSq (Mat3.apply rotation picks[i].Point + offset - world)
                if d < bestD then bestD <- d; best <- i
            picks.RemoveAt best
            picksChanged ()
            true

    member _.ClearPicks() =
        picks.Clear()
        picksChanged ()
        dropPreview ()

    // ------------------------------------------------------- the two moves

    /// Square the picked face to Y. Returns the measurement, or the reason it
    /// cannot run.
    member this.ApplySquare() : Result<Orient.SquareResult, string> =
        let world = this.PicksWorld
        if world.Length < 3 then
            Error $"Orient Face to Side needs 3 or more points on one flat side face (%d{world.Length} picked)."
        else
            let r = Orient.squareToAxis lockedAxis world
            this.ApplyRotation r.Rotation
            Ok r

    /// Spin about Y until the picked top/bottom face is level.
    member this.ApplyStraighten() : Result<Orient.StraightenResult, string> =
        let world = this.PicksWorld
        if world.Length < 2 then
            Error $"Y-Spin Face to Level needs 2 or more points on a flat top or bottom face (%d{world.Length} picked)."
        else
            let r = Orient.straightenAbout lockedAxis world
            this.ApplyRotation r.Rotation
            Ok r

    member _.CanCenterline = picks.Count >= 2

    /// Translate in Y so the two picked faces sit symmetric about the XZ
    /// plane — pick point(s) on the right face and on the left face, then
    /// this. TRANSLATION ONLY, and deliberately not followed by a re-centre:
    /// re-centring on the bbox is exactly what this overrides. It IS undone
    /// by any later rotation (those re-centre), so it is a do-last step;
    /// Undo covers mistakes.
    member this.ApplyYCenterline() : Result<Orient.CenterlineResult, string> =
        if mesh.IsNone then Error "Load a scan first."
        elif picks.Count < 2 then
            Error $"Y-Centerline needs points on BOTH faces (%d{picks.Count} picked) — at least one on each side."
        else
            let ys = this.PicksWorld |> Array.map (fun p -> p.Y)
            match Orient.yCenterline 1.0 ys with
            | Error(Orient.WrongClusterCount n) ->
                Error($"Y-Centerline needs picks on exactly 2 Y-aligned faces — "
                      + $"these form %d{n} group(s). Pick some on the right face, some on the left.")
            | Error(Orient.ClusterTooLoose s) ->
                Error($"One group of picks spreads %.2f{s} mm in Y — more than one face, "
                      + "or a pick missed. Remove the stray (right-click) and retry.")
            | Ok r ->
                this.PushUndo(Rigid(rotation, offset))
                offset <- offset + Vec3.create 0.0 r.ShiftY 0.0
                reframe ()
                Ok r

    /// What each button would do with the picks as they stand.
    ///
    /// BOTH are reported, always. With one pick list both are live candidates,
    /// and the two residuals are what actually tell you which face you are on:
    /// points spread over a flat side fit a plane tightly and a side-view line
    /// badly, points along a top edge do the reverse. That is more use than a
    /// mode label, and it cannot be set to the wrong thing.
    member this.CurrentFit() : string =
        let world = this.PicksWorld
        // Each fit is computed ONLY inside the branch that has enough points
        // for it: `straightenAbout` throws on one pick, and this runs on every
        // pick, including the first. `let` is eager in F#, so binding both
        // fits up front and choosing afterwards would already be too late.
        if world.Length = 0 then ""
        elif world.Length = 1 then "1 pick · to-side needs 3 · y-spin needs 2"
        else
            let sq =
                if world.Length < 3 then "to-side needs 3"
                else
                    let r = Orient.squareToAxis lockedAxis world
                    $"to-side %.3f{r.TiltDegrees}° (±%.3f{r.RmsMm} mm)"
            let r = Orient.straightenAbout lockedAxis world
            $"%d{world.Length} picks · {sq} · y-spin %.3f{r.AngleDegrees}° (±%.3f{r.RmsMm} mm)"

    // ------------------------------------------------------------ flatten

    /// Bumped every time the preview appears, changes or vanishes — the GL
    /// view compares it to know when its highlight buffers are stale.
    member _.PreviewStamp = previewStamp

    member _.Preview = preview

    /// (green, yellow) face lists for the highlight overlay. Yellow is
    /// everything a force would flatten wholesale — enclaves plus their halo
    /// — and those halo faces leave the green list so the two buffers never
    /// overlap (identical geometry drawn twice at the same polygon offset
    /// z-fights, and the honest color is the force's reach anyway).
    member _.PreviewFaces =
        preview
        |> Option.map (fun pv ->
            if pv.HaloFaces.Length = 0 then pv.CapturedFaces, pv.EnclaveFaces
            else
                let halo = System.Collections.Generic.HashSet<int> pv.HaloFaces
                pv.CapturedFaces |> Array.filter (halo.Contains >> not),
                Array.append pv.EnclaveFaces pv.HaloFaces)

    member _.CanFlatten = mesh.IsSome && picks.Count >= 3

    /// True when the shown preview matches these parameters AND the picks it
    /// was computed from — i.e. pressing Update would change nothing. The
    /// window greys the button on this.
    member _.PreviewIsCurrent(floorMm : float, ceilingMm : float, forceEnclaves : bool,
                              snapToAxis : bool) =
        match preview with
        | None -> false
        | Some _ ->
            let (pf, pc, pforce, psnap) = previewParams
            previewPicksVersion = picksVersion && pforce = forceEnclaves && psnap = snapToAxis
            && abs (pf - floorMm) < 1e-12 && abs (pc - ceilingMm) < 1e-12

    /// The WORLD axis (0 = X, 1 = Y, 2 = Z) the picks' plane is nearly
    /// normal to, with how far off it the pick fit sits, in degrees — the
    /// "snap to true axis plane" offer. Within the threshold the face is
    /// evidently MEANT to be axis-true and the residual is pick noise, not
    /// design: snapping makes every such flat exactly parallel to the
    /// others, so cube booleans downstream (Blender CSG) sit flush instead
    /// of leaving one corner proud and another below.
    /// World space, not model space: "true Y" is a promise about the
    /// exported frame, which is the current orientation — the scan's own
    /// frame is arbitrary.
    member this.AxisSnapCandidate : (int * float) option =
        if picks.Count < 3 then None
        else
            let n = (Geometry.fitPlane this.PicksWorld).Normal
            let c = [| abs n.X; abs n.Y; abs n.Z |]
            let ax = if c[0] >= c[1] && c[0] >= c[2] then 0
                     elif c[1] >= c[2] then 1
                     else 2
            let off = acos (Math.Clamp(c[ax], 0.0, 1.0)) * 180.0 / Math.PI
            if off <= Flatten.axisSnapThresholdDegrees then Some(ax, off) else None

    /// Suggested noise floor: 3x the plane-fit RMS of the picks — the scan's
    /// own noise, measured off the very points the user just placed.
    member this.SuggestedFloor : float option =
        if picks.Count < 3 then None
        else
            let rms = (Geometry.fitPlane (this.PicksWorld)).Rms
            Some(max 0.01 (3.0 * rms))

    /// Compute (or refresh) the preview. Pure with respect to the mesh —
    /// nothing moves until ApplyFlatten.
    member this.ComputeFlattenPreview(floorMm : float, ceilingMm : float, forceEnclaves : bool,
                                      snapToAxis : bool)
            : Result<Flatten.Preview, string> =
        match mesh with
        | None -> Error "Load a scan first."
        | Some m ->
            if picks.Count < 3 then
                Error $"Flatten Face needs 3 or more points on the flat (%d{picks.Count} picked)."
            elif ceilingMm <= 0.0 then
                Error "The ceiling must be a positive distance in mm."
            else
                let adj =
                    match adjacency with
                    | Some a -> a
                    | None ->
                        let a = Flatten.buildAdjacency m
                        adjacency <- Some a
                        a
                let seeds = picks |> Seq.map (fun p -> p.Triangle) |> Seq.distinct |> Seq.toArray
                let pts = picks |> Seq.map (fun p -> p.Point) |> Seq.toArray
                let hint = picks |> Seq.fold (fun acc p -> acc + p.Normal) Vec3.zero
                // "True Y" is a world-space promise; the flatten runs in
                // model space, so the axis rides the inverse rotation in.
                // No candidate near = nothing to snap to, flag or no flag.
                let snapNormal =
                    if not snapToAxis then None
                    else
                        this.AxisSnapCandidate
                        |> Option.map (fun (ax, _) ->
                            Mat3.apply (Mat3.transpose rotation) (Vec3.axis ax))
                let pv =
                    Flatten.preview m adj seeds pts hint
                        { Flatten.defaults with
                            FloorMm = floorMm
                            CeilingMm = ceilingMm
                            ForceEnclaves = forceEnclaves
                            SnapNormal = snapNormal }
                preview <- Some pv
                previewParams <- (floorMm, ceilingMm, forceEnclaves, snapToAxis)
                previewPicksVersion <- picksVersion
                previewStamp <- previewStamp + 1
                Ok pv

    /// Commit the shown preview. Picks are KEPT — re-running the same flat at
    /// a different ceiling is the common follow-up, and clearing is one click.
    member this.ApplyFlatten() : Result<Flatten.Preview, string> =
        match mesh, preview with
        | _, None -> Error "Nothing to apply — press FLATTEN to preview first."
        | None, _ -> Error "Load a scan first."
        | Some m, Some pv ->
            let flattened, undoData = Flatten.apply m pv
            mesh <- Some flattened
            this.PushUndo(Reshape(undoData, pv.MovedVertexCount))
            flattenedVerts <- flattenedVerts + pv.MovedVertexCount
            dropPreview ()
            reframe ()
            Ok pv

    member _.DiscardPreview() = dropPreview ()

    // ------------------------------------------------------------- export

    /// Verts moved by flattens still baked into the current mesh. When this
    /// is non-zero the export is no longer a pure rigid transform of the scan
    /// and the status message must stop claiming it is.
    member _.FlattenedVertexCount = flattenedVerts

    /// Write up to TWO files beside the source:
    ///   `<name>_oriented.stl` — always: the PRISTINE scan, rigidly
    ///       transformed, no flattens even if some were applied;
    ///   `<name>_cleaned.stl`  — only when flattens are baked in: the same
    ///       orientation with the flattened vertices.
    /// Returns (orientedPath, cleanedPath option).
    member _.ExportStl() : string * string option =
        match mesh with
        | None -> failwith "nothing loaded"
        | Some m ->
            let oriented = MeshIO.orientedPathFor m.SourcePath
            MeshIO.saveStlBinary oriented
                (Mesh.transform rotation offset { m with Vertices = pristineVerts })
            let cleaned =
                if flattenedVerts > 0 then
                    let p = MeshIO.cleanedPathFor m.SourcePath
                    MeshIO.saveStlBinary p (Mesh.transform rotation offset m)
                    Some p
                else None
            oriented, cleaned
