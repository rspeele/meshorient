namespace MeshOrient.Core.Tests

open System
open System.IO
open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.Core

/// Purpose-built fixtures for the failure modes FLATTEN exists to avoid,
/// generated deterministically so every assertion is against known geometry.
///
/// The frame2solid synthetic is NOT usable here, and discovering that was one
/// of this suite's first findings: make_synthetic.py appends fresh corner
/// vertices per quad and jitters every copy independently, so adjacent quads
/// do not share exact positions — the mesh is cracked at every quad boundary
/// and a flood can never leave the quad it started in. Real inputs to this
/// tool are watertight MeshMixer exports with bit-identical shared vertices
/// (verified on the g21 scan), which these indexed fixtures model correctly.
module private FlattenFixtures =

    /// Deterministic gaussian jitter (Box-Muller over a seeded Random).
    let noise (rng : Random) (sigma : float) =
        let u1 = 1.0 - rng.NextDouble()
        let u2 = rng.NextDouble()
        sigma * sqrt (-2.0 * log u1) * cos (2.0 * Math.PI * u2)

    /// Indexed grid-of-quads surface with EXACT shared vertices, so the
    /// adjacency weld sees a proper surface. `keep` culls faces (windows).
    let gridMesh (ni : int) (nj : int) (pos : int -> int -> Vec3)
                 (keep : int -> int -> bool) : Mesh =
        let verts = Array.init (ni * nj) (fun k -> pos (k % ni) (k / ni))
        let idx = ResizeArray<int>()
        for j in 0 .. nj - 2 do
            for i in 0 .. ni - 2 do
                if keep i j then
                    let a, b = j * ni + i, j * ni + i + 1
                    let c, d = (j + 1) * ni + i + 1, (j + 1) * ni + i
                    idx.AddRange [ a; b; c ]
                    idx.AddRange [ a; c; d ]
        Mesh.create verts (idx.ToArray()) "fixture"

    /// An L: a flat top plate (z ~ 0) meeting a vertical wall at a SHARP
    /// edge, jittered in 3D. The fixture for "flattening the top must not
    /// round the arris or smear a skirt down the wall".
    let sharpL (sigma : float) : Mesh * int * int =
        let rng = Random 11
        let step = 0.4
        let nTop = 51
        let nWall = 21
        let nj = 26
        let ni = nTop + nWall - 1                      // shared edge column
        let pos i j =
            let y = float j * step
            let clean =
                if i < nTop then Vec3.create (float i * step) y 0.0
                else Vec3.create 20.0 y (-(float (i - nTop + 1) * step))
            clean + Vec3.create (noise rng sigma) (noise rng sigma) (noise rng sigma)
        gridMesh ni nj pos (fun _ _ -> true), nTop, ni

    /// A flat run rolling tangentially into a radius-60 fillet, jittered.
    /// The fixture for "flattening the flat must not print a crease where
    /// the curve starts". Wide (33 rows) so column means average the noise
    /// down and the metric sees the systematic kink, not the jitter.
    let curveStrip (sigma : float) : Mesh * int =
        let rng = Random 23
        let step = 0.35
        let r = 60.0
        let flatCols = [| for i in 0 .. 57 -> float i * step, 0.0 |]
        let arcCols =
            [| for k in 1 .. 120 ->
                let t = float k * step / r
                if t > 30.0 * Math.PI / 180.0 then None
                else Some(20.0 + r * sin t, r * (1.0 - cos t)) |]
            |> Array.choose id
        let cols = Array.append flatCols arcCols
        let nj = 33
        let pos i j =
            let (x, z) = cols[i]
            Vec3.create x (float j * 0.4) z
            + Vec3.create (noise rng sigma) (noise rng sigma) (noise rng sigma)
        gridMesh cols.Length nj pos (fun _ _ -> true), cols.Length

    /// The main fixture: a gun-wall stand-in in the XZ plane (normal +-Y),
    /// jittered, carrying every situation the flood has to get right:
    ///   - a rectangular WINDOW (missing faces) — must NOT read as an enclave
    ///   - a round BOSS raised 1.5 mm — must become an enclave
    ///   - a diagonal RIB raised 1.5 mm — must become a second enclave
    ///   - a perpendicular FLANGE along one edge — the "rest of the model",
    ///     which is what the largest-touching-component enclave rule needs
    ///     and what keeps the flood from capturing round the corner.
    /// Plate x 0..60, z 0..80, step 0.5; flange bends off x = 60.
    let wall (sigma : float) : Mesh =
        let rng = Random 31
        let step = 0.5
        let nPlate = 121                                // x 0..60
        let nFlange = 40                                // 20 mm of bend
        let ni = nPlate + nFlange
        let nj = 161                                    // z 0..80
        let inBoss x z = (x - 45.0) ** 2.0 + (z - 60.0) ** 2.0 < 25.0
        let inRib x z =
            // distance from the segment (10,55)-(30,75) under 2 mm
            let dx, dz = 20.0, 20.0
            let t = Math.Clamp(((x - 10.0) * dx + (z - 55.0) * dz) / (dx * dx + dz * dz), 0.0, 1.0)
            let px, pz = 10.0 + t * dx, 55.0 + t * dz
            (x - px) ** 2.0 + (z - pz) ** 2.0 < 4.0
        let pos i j =
            let z = float j * step
            let clean =
                if i < nPlate then
                    let x = float i * step
                    let y = if inBoss x z || inRib x z then 1.5 else 0.0
                    Vec3.create x y z
                else
                    // flange: perpendicular to the plate, off the x=60 edge
                    Vec3.create (60.0 + float (i - nPlate + 1) * step) 0.0 z
                    |> fun v -> Vec3.create 60.0 (float (i - nPlate + 1) * step) v.Z
            clean + Vec3.create (noise rng sigma) (noise rng sigma) (noise rng sigma)
        let keep i j =
            // the window: no faces over x 15..30, z 20..40
            let x, z = float i * step, float j * step
            not (i < nPlate && x >= 15.0 && x + step <= 30.0 && z >= 20.0 && z + step <= 40.0)
        gridMesh ni nj pos keep

    /// Face whose centroid is nearest (x, z) — a synthetic "pick" for the
    /// wall fixture (its plate lives in XZ).
    let faceNearXZ (m : Mesh) (x : float) (z : float) =
        let mutable best, bestD = 0, infinity
        for f in 0 .. m.TriangleCount - 1 do
            let struct (a, b, c) = Mesh.triangle m f
            let cen = (a + b + c) / 3.0
            let d = (cen.X - x) ** 2.0 + (cen.Z - z) ** 2.0 + cen.Y * cen.Y
            if d < bestD then bestD <- d; best <- f
        best

    /// Same, in XY, for the sharpL / curveStrip fixtures (surface at z ~ 0).
    let faceNearXY (m : Mesh) (x : float) (y : float) =
        let mutable best, bestD = 0, infinity
        for f in 0 .. m.TriangleCount - 1 do
            let struct (a, b, c) = Mesh.triangle m f
            let cen = (a + b + c) / 3.0
            let d = (cen.X - x) ** 2.0 + (cen.Y - y) ** 2.0 + cen.Z * cen.Z
            if d < bestD then bestD <- d; best <- f
        best

    let columnMeans (m : Mesh) (ni : int) =
        let nj = m.Vertices.Length / ni
        [| for i in 0 .. ni - 1 ->
            let mutable acc = 0.0
            for j in 0 .. nj - 1 do acc <- acc + m.Vertices[j * ni + i].Z
            acc / float nj |]

    let worstKink (means : float[]) =
        let mutable worst = 0.0
        for i in 1 .. means.Length - 2 do
            worst <- max worst (abs (means[i + 1] - 2.0 * means[i] + means[i - 1]))
        worst


[<TestClass>]
type FlattenTests () =

    let sigma = 0.02                                   // the g21's measured noise

    let previewOn (m : Mesh) (seeds : int[]) (pts : Vec3[]) (hint : Vec3) floor ceiling =
        let adj = Flatten.buildAdjacency m
        Flatten.preview m adj seeds pts hint
            { Flatten.defaults with FloorMm = floor; CeilingMm = ceiling }

    // ------------------------------------------------------------ adjacency

    [<TestMethod>]
    member _.Adjacency_pairs_every_interior_edge_of_a_grid () =
        let m, _, ni = FlattenFixtures.sharpL 0.0
        let adj = Flatten.buildAdjacency m
        Assert.AreEqual(m.Vertices.Length, adj.CanonicalCount, "indexed grid: nothing to weld")
        let holes = adj.Neighbor |> Array.filter (fun n -> n = -1) |> Array.length
        let rim = 2 * (ni - 1) + 2 * (m.Vertices.Length / ni - 1)
        Assert.AreEqual(rim, holes, "only the outer rim should be unpaired")

    /// The STL round trip a real scan takes: indexed surface -> binary STL
    /// (triangle soup, but with bit-identical shared coordinates, which is
    /// what MeshMixer emits — verified on the g21) -> load -> exact-bit weld
    /// reconstructs the surface. This is the property the whole flood stands
    /// on.
    [<TestMethod>]
    member _.Exact_bit_weld_reconstructs_a_surface_from_stl_soup () =
        let src = FlattenFixtures.wall sigma
        let p = Path.Combine(Path.GetTempPath(), $"meshorient_weld_{Guid.NewGuid():N}.stl")
        try
            MeshIO.saveStlBinary p src
            let soup = MeshIO.load p
            Assert.AreEqual(soup.Indices.Length, soup.Vertices.Length, "STL loads as soup")
            let adj = Flatten.buildAdjacency soup
            printfn "weld: %d soup verts -> %d canonical (source had %d)"
                    soup.Vertices.Length adj.CanonicalCount src.Vertices.Length
            // Every canonical vert that is USED comes back; the soup shrinks
            // by ~6x (interior verts are shared by six triangles).
            Assert.IsTrue(adj.CanonicalCount < soup.Vertices.Length / 4,
                          "the weld should collapse the soup substantially")
        finally
            if File.Exists p then File.Delete p

    [<TestMethod>]
    member _.Adjacency_tolerates_the_cracked_synthetic () =
        // The f2s synthetic is cracked at quad boundaries (independent jitter
        // per vertex copy) and full of deliberate holes. Not a useful flood
        // fixture — but adjacency must build on it without choking, because
        // hostile meshes are a thing.
        let m = Fixtures.synthetic.Value
        let sw = System.Diagnostics.Stopwatch.StartNew()
        let adj = Flatten.buildAdjacency m
        printfn "adjacency on %d tris: %d ms, %d verts -> %d canonical"
                m.TriangleCount sw.ElapsedMilliseconds m.Vertices.Length adj.CanonicalCount
        Assert.IsTrue(adj.CanonicalCount < m.Vertices.Length, "within-quad sharing still welds")
        Assert.IsTrue(adj.Neighbor |> Array.exists (fun n -> n = -1),
                      "cracks and holes must surface as unpaired edges, not crashes")

    // ------------------------------------------------------- the two fixes

    /// Sharp-edge preservation: with feathering in DISTANCE space, a true
    /// flat snaps at full strength right up to the arris, because every one
    /// of its verts sits inside the noise floor. Feathering by proximity to
    /// the region perimeter — the obvious alternative — would have left the
    /// outer millimetres only partially flattened and rounded the edge.
    [<TestMethod>]
    member _.Flattening_a_box_top_keeps_the_edge_sharp_and_the_wall_untouched () =
        let m, nTop, ni = FlattenFixtures.sharpL sigma
        let nj = m.Vertices.Length / ni
        let seed = FlattenFixtures.faceNearXY m 10.0 5.0
        let pts = [| Vec3.create 3.0 3.0 0.0; Vec3.create 17.0 3.0 0.0; Vec3.create 10.0 8.0 0.0 |]
        let pv = previewOn m [| seed |] pts Vec3.unitZ (5.0 * sigma) 0.3
        printfn "sharpL: %d captured, rms %.4f -> %.4f, max move %.4f"
                pv.CapturedFaces.Length pv.RmsBeforeMm pv.RmsAfterMm pv.MaxMoveMm
        let flat, _ = Flatten.apply m pv

        // The edge column belongs to the top: snapped exactly onto the plane
        // (floor = 5 sigma puts every top vert in the full-strength zone).
        let mutable edgeWorst = 0.0
        for j in 0 .. nj - 1 do
            edgeWorst <- max edgeWorst (abs flat.Vertices[j * ni + (nTop - 1)].Z)
        printfn "edge row |z| after: %.5f (was noise ~%.3f)" edgeWorst sigma
        Assert.IsTrue(edgeWorst < 0.01, $"edge row should be ON the plane, worst {edgeWorst}")

        // Wall verts below the edge: not captured, not moved at all.
        for j in 0 .. nj - 1 do
            for i in nTop .. ni - 1 do
                let k = j * ni + i
                Assert.AreEqual(m.Vertices[k].X, flat.Vertices[k].X, 1e-12, "wall vert moved")
                Assert.AreEqual(m.Vertices[k].Z, flat.Vertices[k].Z, 1e-12, "wall vert moved")

    /// The crease, as an A/B against the naive hard snap. floor = ceiling IS
    /// the hard snap (the weight degenerates to a step function), so the same
    /// machinery demonstrates both the artifact and its fix.
    [<TestMethod>]
    member _.Feathering_kills_the_crease_a_hard_snap_prints () =
        let run floor ceiling =
            let m, ni = FlattenFixtures.curveStrip sigma
            let seed = FlattenFixtures.faceNearXY m 5.0 3.0
            let pts = [| Vec3.create 2.0 1.0 0.0; Vec3.create 10.0 8.0 0.0; Vec3.create 18.0 3.0 0.0 |]
            let pv = previewOn m [| seed |] pts Vec3.unitZ floor ceiling
            let flat, _ = Flatten.apply m pv
            FlattenFixtures.worstKink (FlattenFixtures.columnMeans flat ni)
        let feathered = run (3.0 * sigma) 0.3
        let hard = run 0.3 0.3
        printfn "worst kink: feathered %.4f mm, hard snap %.4f mm (%.1fx)"
                feathered hard (hard / feathered)
        Assert.IsTrue(hard > 2.0 * feathered,
                      $"the hard snap should print a visibly sharper crease (hard {hard}, feathered {feathered})")
        Assert.IsTrue(feathered < 0.045, $"feathered transition should stay smooth, got {feathered}")

    // ----------------------------------------------------- the wall fixture

    [<TestMethod>]
    member _.Wall_floods_around_features_and_reports_them_as_enclaves () =
        let m = FlattenFixtures.wall sigma
        let seeds =
            [| FlattenFixtures.faceNearXZ m 5.0 5.0
               FlattenFixtures.faceNearXZ m 55.0 10.0
               FlattenFixtures.faceNearXZ m 5.0 75.0 |]
        let pts = seeds |> Array.map (fun f ->
            let struct (a, b, c) = Mesh.triangle m f
            (a + b + c) / 3.0)
        let sw = System.Diagnostics.Stopwatch.StartNew()
        let pv = previewOn m seeds pts Vec3.unitY (3.0 * sigma) 0.3
        printfn "wall: %d ms, %d captured, %d islands, %d verts, rms %.4f -> %.4f, %d enclave(s)"
                sw.ElapsedMilliseconds pv.CapturedFaces.Length pv.Islands
                pv.MovedVertexCount pv.RmsBeforeMm pv.RmsAfterMm pv.EnclaveCount

        // The whole plate minus window and feature footprints, and nothing
        // of the flange (it fails the normal gate at the bend).
        Assert.IsTrue(pv.CapturedFaces.Length > 30000, "should capture the plate")
        Assert.IsTrue(pv.CapturedFaces.Length < 37500, "must not capture the flange too")
        Assert.AreEqual(1, pv.Islands, "three seeds on one connected flat = one island")
        Assert.IsTrue(pv.RmsAfterMm < pv.RmsBeforeMm / 2.0,
                      $"flattening should at least halve the noise ({pv.RmsBeforeMm} -> {pv.RmsAfterMm})")

        // Exactly two enclaves: the boss and the rib. The window is missing
        // faces, not surrounded ones; the flange is the largest touching
        // complement component — the "rest of the model".
        Assert.AreEqual(2, pv.EnclaveCount, "boss + rib, nothing else")
        let enclaveNear (x, z) =
            pv.EnclaveFaces
            |> Array.exists (fun f ->
                let struct (a, b, c) = Mesh.triangle m f
                let cen = (a + b + c) / 3.0
                (cen.X - x) ** 2.0 + (cen.Z - z) ** 2.0 < 36.0)
        Assert.IsTrue(enclaveNear (45.0, 60.0), "the boss should be an enclave")
        Assert.IsTrue(enclaveNear (20.0, 65.0), "the rib should be an enclave")
        let enclaveInWindow =
            pv.EnclaveFaces
            |> Array.exists (fun f ->
                let struct (a, b, c) = Mesh.triangle m f
                let cen = (a + b + c) / 3.0
                cen.X > 15.0 && cen.X < 30.0 && cen.Z > 20.0 && cen.Z < 40.0 && abs cen.Y < 0.5)
        Assert.IsFalse(enclaveInWindow, "the window is a hole, not an enclave")

        // Locality: nothing on the boss top or the flange may move.
        let flat, undo = Flatten.apply m pv
        for (idx, _) in undo do
            Assert.IsTrue(m.Vertices[idx].Y < 1.0,
                          $"a raised-feature vert moved (idx {idx}, y {m.Vertices[idx].Y})")
        let mutable flangeMoved = 0
        for (idx, old) in undo do
            if m.Vertices[idx].X > 59.9 && m.Vertices[idx].Y > 0.5 then flangeMoved <- flangeMoved + 1
        Assert.AreEqual(0, flangeMoved, "the flange is not part of the flat")
        Assert.IsTrue(flat.Vertices.Length = m.Vertices.Length)

    // ----------------------------------------------------- state integration

    /// The full preview loop as the window drives it, on a mesh that took the
    /// same road a real scan takes (indexed -> STL soup -> weld).
    [<TestMethod>]
    member _.Preview_apply_undo_round_trip_through_the_app_state () =
        let stl = Path.Combine(Path.GetTempPath(), $"meshorient_wall_{Guid.NewGuid():N}.stl")
        MeshIO.saveStlBinary stl (FlattenFixtures.wall sigma)
        try
            let s = MeshOrient.App.OrientState()
            s.Load stl |> ignore
            // Picks by ray, exactly as clicks would arrive. Load re-centres
            // the model on its bbox, so fixture coordinates go through the
            // state's own offset to become world rays (rotation is identity).
            for (x, z) in [ 5.0, 5.0; 55.0, 10.0; 5.0, 75.0 ] do
                let o = s.Offset
                let hit = s.PickAt { Raycast.Origin = Vec3.create (x + o.X) 1000.0 (z + o.Z)
                                     Raycast.Direction = Vec3.create 0.0 -1.0 0.0 }
                Assert.IsTrue(hit.IsSome, $"pick at ({x}, {z}) should land")
            Assert.IsTrue s.CanFlatten
            let floor = s.SuggestedFloor.Value
            printfn "suggested floor from picks: %.4f mm" floor

            // Preview is pure: stamp ticks, currency tracks, nothing moves.
            let before = Array.copy s.Mesh.Value.Vertices
            let stamp0 = s.PreviewStamp
            match s.ComputeFlattenPreview(floor, 0.3) with
            | Error e -> Assert.Fail e
            | Ok pv ->
                printfn "preview: %d verts, %d enclaves" pv.MovedVertexCount pv.EnclaveCount
                Assert.IsTrue(pv.MovedVertexCount > 10000, "should preview the whole plate")
                Assert.AreNotEqual(stamp0, s.PreviewStamp, "stamp must tick for the GL view")
                Assert.IsTrue(s.PreviewIsCurrent(floor, 0.3))
                Assert.IsFalse(s.PreviewIsCurrent(floor, 0.4), "other params = stale")
                let untouched =
                    Array.forall2 (fun (a : Vec3) (b : Vec3) ->
                        a.X = b.X && a.Y = b.Y && a.Z = b.Z) before s.Mesh.Value.Vertices
                Assert.IsTrue(untouched, "preview must not move anything")

            // A new pick stales the preview — what is highlighted must always
            // be what APPLY would do.
            s.PickAt { Raycast.Origin = Vec3.create (40.0 + s.Offset.X) 1000.0 (10.0 + s.Offset.Z)
                       Raycast.Direction = Vec3.create 0.0 -1.0 0.0 } |> ignore
            Assert.IsFalse(s.PreviewIsCurrent(floor, 0.3), "pick edits must stale the preview")
            s.ComputeFlattenPreview(floor, 0.3) |> ignore

            // Apply: mesh moves, picks are KEPT, count recorded.
            let nPicks = s.Picks.Count
            match s.ApplyFlatten() with
            | Error e -> Assert.Fail e
            | Ok pv ->
                Assert.AreEqual(nPicks, s.Picks.Count, "picks must survive an apply")
                Assert.IsTrue(s.FlattenedVertexCount > 0)
                Assert.IsTrue(s.Preview.IsNone, "preview is consumed by apply")
                let moved =
                    Array.exists2 (fun (a : Vec3) (b : Vec3) -> a.Y <> b.Y)
                        before s.Mesh.Value.Vertices
                Assert.IsTrue(moved, "apply must actually move the mesh")
                printfn "applied: %d verts, max %.3f mm" pv.MovedVertexCount pv.MaxMoveMm

            // Undo: byte-identical restoration, reported as a MESH change so
            // the window re-uploads the VBO (a rigid undo would not).
            Assert.IsTrue(s.Undo(), "undoing a flatten must report a mesh change")
            let restored =
                Array.forall2 (fun (a : Vec3) (b : Vec3) ->
                    a.X = b.X && a.Y = b.Y && a.Z = b.Z) before s.Mesh.Value.Vertices
            Assert.IsTrue(restored, "undo must restore every vertex exactly")
            Assert.AreEqual(0, s.FlattenedVertexCount)
        finally
            if File.Exists stl then File.Delete stl

    /// Export STL writes BOTH truths: `_oriented` is the pristine scan
    /// (flattens deliberately absent), `_cleaned` has them baked in. The
    /// cleanup is never the price of the raw geometry.
    [<TestMethod>]
    member _.Export_writes_pristine_oriented_and_flattened_cleaned () =
        let stl = Path.Combine(Path.GetTempPath(), $"meshorient_exp_{Guid.NewGuid():N}.stl")
        MeshIO.saveStlBinary stl (FlattenFixtures.wall sigma)
        let mutable toDelete = [ stl ]
        try
            let s = MeshOrient.App.OrientState()
            s.Load stl |> ignore
            for (x, z) in [ 5.0, 5.0; 55.0, 10.0; 5.0, 75.0 ] do
                let o = s.Offset
                s.PickAt { Raycast.Origin = Vec3.create (x + o.X) 1000.0 (z + o.Z)
                           Raycast.Direction = Vec3.create 0.0 -1.0 0.0 } |> ignore
            s.ComputeFlattenPreview(3.0 * sigma, 0.3) |> ignore
            match s.ApplyFlatten() with
            | Error e -> Assert.Fail e
            | Ok _ -> ()

            let oriented, cleaned = s.ExportStl()
            Assert.IsTrue(cleaned.IsSome, "flattens applied -> a _cleaned file")
            toDelete <- oriented :: cleaned.Value :: toDelete
            Assert.IsTrue(oriented.EndsWith "_oriented.stl")
            Assert.IsTrue(cleaned.Value.EndsWith "_cleaned.stl")

            // Plate-noise spread in each file, measured about the median Y
            // (the export is bbox-centred, so absolute Y is shifted): the
            // oriented file must still carry the scan's jitter, the cleaned
            // one must not.
            let plateSpread (path : string) =
                let m = MeshIO.load path
                let ys = m.Vertices |> Array.map (fun v -> v.Y)
                let sorted = Array.sort ys
                let median = sorted[sorted.Length / 2]
                // +-0.3, not +-0.5: the flange's first row sits exactly
                // 0.5 mm off the plate and jitter tips half of it inside a
                // +-0.5 window, polluting the spread of BOTH files.
                let plate = ys |> Array.filter (fun y -> abs (y - median) < 0.3)
                let mean = Array.average plate
                sqrt (plate |> Array.averageBy (fun y -> (y - mean) ** 2.0))
            let rawSpread = plateSpread oriented
            let cleanSpread = plateSpread cleaned.Value
            printfn "plate noise: oriented %.4f mm, cleaned %.4f mm" rawSpread cleanSpread
            Assert.IsTrue(rawSpread > 0.012, $"_oriented must keep the scan's noise, got {rawSpread}")
            Assert.IsTrue(cleanSpread < rawSpread / 3.0,
                          $"_cleaned should be flat ({rawSpread} -> {cleanSpread})")
            let a, b = MeshIO.load oriented, MeshIO.load cleaned.Value
            Assert.AreEqual(a.TriangleCount, b.TriangleCount, "same triangles in both")
        finally
            for f in toDelete do
                if File.Exists f then File.Delete f

    [<TestMethod>]
    member _.Rigid_undo_still_reports_no_mesh_change () =
        let s = MeshOrient.App.OrientState()
        s.Load Fixtures.syntheticPath |> ignore
        s.ApplyRotation(Mat3.rotDegrees 0 30.0)
        Assert.IsFalse(s.Undo(), "a rigid undo moves the matrix, not the vertices")
