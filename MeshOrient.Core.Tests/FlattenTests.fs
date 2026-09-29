namespace MeshOrient.Core.Tests

open System
open System.IO
open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.Core

/// Purpose-built fixtures for the failure modes FLATTEN exists to avoid,
/// generated deterministically so every assertion is against known geometry.
///
/// The synthetic scan fixture is NOT usable here: it appends fresh corner
/// vertices per quad and jitters every copy independently, so adjacent quads
/// do not share exact positions — the mesh is cracked at every quad boundary
/// and a flood can never leave the quad it started in. Real inputs to this
/// tool are watertight MeshMixer exports with bit-identical shared vertices,
/// which these indexed fixtures model correctly.
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

    /// The main fixture: a side-wall stand-in in the XZ plane (normal +-Y),
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

    /// A flat plate with a smooth gaussian DOME (height 1.5, sigma 3 mm) in
    /// the middle. The dome's slope never exceeds ~17 degrees, so the normal
    /// gate passes ALL of it; the distance ceiling is what stops the flood,
    /// making the cap an enclave with a captured, feathered annulus running
    /// up to it — the exact "smooth gradient toward the enclave" geometry
    /// where force-flattening without a halo prints a ring.
    let dome (sigma : float) : Mesh =
        let rng = Random 41
        let step = 0.5
        let n = 61                                     // 30 x 30 mm plate
        let pos i j =
            let x, y = float i * step, float j * step
            let r2 = (x - 15.0) ** 2.0 + (y - 15.0) ** 2.0
            let z = 1.5 * exp (-r2 / (2.0 * 9.0))
            Vec3.create x y z
            + Vec3.create (noise rng sigma) (noise rng sigma) (noise rng sigma)
        gridMesh n n pos (fun _ _ -> true)

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

    let sigma = 0.02                                   // measured on a real scan

    let previewOn (m : Mesh) (seeds : int[]) (pts : Vec3[]) (hint : Vec3) floor ceiling =
        let adj = Flatten.buildAdjacency m
        Flatten.preview m adj seeds pts hint
            { Flatten.defaults with FloorMm = floor; CeilingMm = ceiling }

    /// The wall fixture written out as STL soup and loaded through the app
    /// state, the way a real scan arrives. Returns the state and the temp
    /// file, which the caller deletes.
    let wallState () =
        let stl = Path.Combine(Path.GetTempPath(), $"meshorient_wall_{Guid.NewGuid():N}.stl")
        MeshIO.saveStlBinary stl (FlattenFixtures.wall sigma)
        let s = MeshOrient.App.OrientState()
        s.Load stl |> ignore
        s, stl

    /// Three picks on the wall's plate, clear of the boss, rib and window.
    /// Fixture (model) coordinates go through the state's current transform,
    /// so this works whatever orientation the model is in.
    let pickPlate (s : MeshOrient.App.OrientState) =
        for (x, z) in [ 5.0, 5.0; 55.0, 10.0; 5.0, 75.0 ] do
            let w = Mat3.apply s.Rotation (Vec3.create x 0.0 z) + s.Offset
            let hit = s.PickAt { Raycast.Origin = Vec3.create w.X 1000.0 w.Z
                                 Raycast.Direction = Vec3.create 0.0 -1.0 0.0 }
            Assert.IsTrue(hit.IsSome, $"pick at ({x}, {z}) should land")

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
    /// what MeshMixer emits — verified on a real scan) -> load -> exact-bit weld
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
        // The synthetic is cracked at quad boundaries (independent jitter
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

    /// ForceEnclaves: the same wall, but the boss and rib — 1.5 mm proud,
    /// far beyond the ceiling — get squashed onto the plane wholesale. Their
    /// rim verts were already snapped by the normal pass, so the erased
    /// pocket meets the plate without a step.
    [<TestMethod>]
    member _.Force_flatten_squashes_the_enclaves_onto_the_plane () =
        let m = FlattenFixtures.wall sigma
        let adj = Flatten.buildAdjacency m
        let seeds =
            [| FlattenFixtures.faceNearXZ m 5.0 5.0
               FlattenFixtures.faceNearXZ m 55.0 10.0
               FlattenFixtures.faceNearXZ m 5.0 75.0 |]
        let pts = seeds |> Array.map (fun f ->
            let struct (a, b, c) = Mesh.triangle m f
            (a + b + c) / 3.0)
        let prm = { Flatten.defaults with FloorMm = 3.0 * sigma; CeilingMm = 0.3 }

        let plain = Flatten.preview m adj seeds pts Vec3.unitY prm
        let forced = Flatten.preview m adj seeds pts Vec3.unitY { prm with ForceEnclaves = true }
        printfn "forced: %d extra verts, max move %.3f (plain max %.3f)"
                forced.ForcedVertexCount forced.MaxMoveMm plain.MaxMoveMm

        // Same detection either way — the checkbox changes what HAPPENS to
        // the enclaves, not what counts as one.
        Assert.AreEqual(plain.EnclaveCount, forced.EnclaveCount)
        Assert.AreEqual(0, plain.ForcedVertexCount, "off by default: nothing forced")
        Assert.IsTrue(forced.ForcedVertexCount > 100,
                      $"boss + rib carry hundreds of verts, got {forced.ForcedVertexCount}")
        Assert.IsTrue(forced.MaxMoveMm > 1.2 && forced.MaxMoveMm < 2.0,
                      $"max move should be the 1.5 mm feature height, got {forced.MaxMoveMm}")
        Assert.IsTrue(plain.MaxMoveMm < 0.31, "without force, nothing beyond the ceiling moves")

        // After apply, every enclave-face vert sits ON the plane.
        let flat, _ = Flatten.apply m forced
        let mutable worst = 0.0
        for f in forced.EnclaveFaces do
            for e in 0 .. 2 do
                let v = flat.Vertices[m.Indices[f * 3 + e]]
                worst <- max worst (abs (Vec3.dot (v - forced.PlaneOrigin) forced.PlaneNormal))
        printfn "worst enclave-vert distance after forced apply: %.5f mm" worst
        Assert.IsTrue(worst < 1e-6, $"forced enclaves must be exactly flat, got {worst}")

        // And the flange still did not move: force reaches enclaves only.
        let flangeMoved =
            forced.Moves
            |> Array.exists (fun (struct (idx, _)) ->
                m.Vertices[idx].X > 59.9 && m.Vertices[idx].Y > 0.5)
        Assert.IsFalse(flangeMoved, "force must not leak beyond the enclaves")

    /// The ring regression. Approaching a forced enclave through a smooth
    /// gradient, the feather's residual GROWS toward the ceiling while the
    /// forced interior lands at zero — so without the halo, the verts that
    /// were NEARER the plane ended up FARTHER from it, printing a raised
    /// ring around the erased pocket. The halo pulls that
    /// approach band to full strength, and the whole dome must come out
    /// dead flat.
    [<TestMethod>]
    member _.Force_flatten_halo_kills_the_ring_around_a_gradual_enclave () =
        let m = FlattenFixtures.dome sigma
        let adj = Flatten.buildAdjacency m
        let seeds = [| FlattenFixtures.faceNearXY m 3.0 3.0
                       FlattenFixtures.faceNearXY m 27.0 27.0 |]
        let pts = [| Vec3.create 2.0 2.0 0.0; Vec3.create 27.0 3.0 0.0
                     Vec3.create 3.0 27.0 0.0 |]
        let prm = { Flatten.defaults with FloorMm = 3.0 * sigma; CeilingMm = 0.3 }

        // The geometry really is the drawn scenario: cap enclave, halo band.
        let forced = Flatten.preview m adj seeds pts Vec3.unitZ { prm with ForceEnclaves = true }
        printfn "dome: %d captured, %d enclave, %d halo faces, %d forced verts"
                forced.CapturedFaces.Length forced.EnclaveFaces.Length
                forced.HaloFaces.Length forced.ForcedVertexCount
        Assert.AreEqual(1, forced.EnclaveCount, "the dome cap is one enclave")
        Assert.IsTrue(forced.HaloFaces.Length > 50,
                      $"the feathered annulus should join the force, got {forced.HaloFaces.Length}")

        // After apply: NO ring. Every vert of every touched face — captured,
        // halo and enclave alike — sits exactly on the plane.
        let flat, _ = Flatten.apply m forced
        let residual (faces : int[]) =
            let mutable worst = 0.0
            for f in faces do
                for e in 0 .. 2 do
                    let v = flat.Vertices[m.Indices[f * 3 + e]]
                    worst <- max worst (abs (Vec3.dot (v - forced.PlaneOrigin) forced.PlaneNormal))
            worst
        let worstAll =
            max (residual forced.CapturedFaces)
                (max (residual forced.HaloFaces) (residual forced.EnclaveFaces))
        printfn "worst residual anywhere after forced apply: %.5f mm" worstAll
        // Not exactly zero: the halo BFS chains through feathered faces, and
        // noise can dip one face fully under the floor mid-annulus, breaking
        // the chain and leaving a just-past-floor vert its (tiny) feathered
        // residual. Sub-micron — 400x below the scan's own noise. The ring
        // this test exists to kill was ~0.3 mm.
        Assert.IsTrue(worstAll < 0.005, $"no visible ring may remain, got {worstAll}")

        // And the contrast that motivated the fix: WITHOUT the halo the
        // annulus keeps a residual approaching the ceiling — enclave interior
        // flat, its approach not. (Computed by re-running force with the same
        // params but measuring only the annulus faces the halo identified.)
        let plain = Flatten.preview m adj seeds pts Vec3.unitZ prm
        let plainFlat, _ = Flatten.apply m plain
        let mutable annulusWorst = 0.0
        for f in forced.HaloFaces do
            for e in 0 .. 2 do
                let v = plainFlat.Vertices[m.Indices[f * 3 + e]]
                annulusWorst <- max annulusWorst
                                    (abs (Vec3.dot (v - plain.PlaneOrigin) plain.PlaneNormal))
        printfn "same annulus without force: residual up to %.3f mm (the ring)" annulusWorst
        Assert.IsTrue(annulusWorst > 0.15,
                      "without force the annulus rightly keeps its feathered residual")

    // -------------------------------------------------------- axis snapping

    /// SnapNormal: a flat that is SUPPOSED to be axis-true comes out with a
    /// free-fit normal a fraction of a degree off — flat, but not parallel
    /// to its siblings, which is what left cube corners proud in downstream
    /// CSG. Snapped, the plane normal is the axis EXACTLY and the flatten
    /// still cleans the noise just as well.
    [<TestMethod>]
    member _.Snap_to_axis_yields_an_exactly_axis_normal_plane () =
        // Tilt the wall half a degree about X: the plate's true normal is
        // now measurably off +Y — a stand-in for the sub-degree residual a
        // real orientation leaves — and well inside the snap threshold.
        let m = Mesh.transform (Mat3.rotDegrees 0 0.5) Vec3.zero (FlattenFixtures.wall sigma)
        let adj = Flatten.buildAdjacency m
        let seeds =
            [| FlattenFixtures.faceNearXZ m 5.0 5.0
               FlattenFixtures.faceNearXZ m 55.0 10.0
               FlattenFixtures.faceNearXZ m 5.0 75.0 |]
        let pts = seeds |> Array.map (fun f ->
            let struct (a, b, c) = Mesh.triangle m f
            (a + b + c) / 3.0)
        let prm = { Flatten.defaults with FloorMm = 3.0 * sigma; CeilingMm = 0.3 }

        let free = Flatten.preview m adj seeds pts Vec3.unitY prm
        let snapped = Flatten.preview m adj seeds pts Vec3.unitY { prm with SnapNormal = Some Vec3.unitY }
        let offDeg (n : Vec3) = acos (Math.Clamp(abs n.Y, 0.0, 1.0)) * 180.0 / Math.PI
        printfn "free normal %.3f° off Y; snapped reports %.3f° and uses (%g, %g, %g)"
                (offDeg free.PlaneNormal)
                (defaultArg snapped.SnapOffAngleDegrees nan)
                snapped.PlaneNormal.X snapped.PlaneNormal.Y snapped.PlaneNormal.Z

        // The free fit follows the tilt; unsnapped, that is what you get.
        Assert.IsTrue(free.SnapOffAngleDegrees.IsNone, "no snap requested = none reported")
        Assert.IsTrue(offDeg free.PlaneNormal > 0.4 && offDeg free.PlaneNormal < 0.6,
                      $"free fit should track the 0.5° tilt, got {offDeg free.PlaneNormal}")

        // Snapped: the normal is the axis EXACTLY, and the off-angle report
        // matches what the free fit wanted.
        Assert.IsTrue(snapped.PlaneNormal.X = 0.0 && snapped.PlaneNormal.Z = 0.0
                      && abs snapped.PlaneNormal.Y = 1.0,
                      "snapped plane must be EXACTLY axis-normal")
        match snapped.SnapOffAngleDegrees with
        | None -> Assert.Fail "snap requested but not reported"
        | Some off -> Assert.IsTrue(off > 0.4 && off < 0.6,
                                    $"reported off-angle should be the tilt, got {off}")

        // Same capture, same cleaning power: the snap is not allowed to cost
        // anything the free fit delivered.
        Assert.AreEqual(free.EnclaveCount, snapped.EnclaveCount)
        Assert.IsTrue(snapped.RmsAfterMm < snapped.RmsBeforeMm / 2.0,
                      $"snapped flatten must still clean the noise ({snapped.RmsBeforeMm} -> {snapped.RmsAfterMm})")

        // And the payoff: every vert inside the noise floor of the flat's
        // OWN fit — the whole plate, tilt notwithstanding, because
        // membership is judged against the free plane — lands at ONE exact
        // Y value. Parallel to XZ, not merely flat.
        let mutable worst = 0.0
        let mutable zMin, zMax = infinity, -infinity
        for struct (idx, np) in snapped.Moves do
            let v = m.Vertices[idx]
            if abs (Vec3.dot (v - free.PlaneOrigin) free.PlaneNormal) <= 3.0 * sigma then
                worst <- max worst (abs (np.Y - snapped.PlaneOrigin.Y))
                zMin <- min zMin v.Z
                zMax <- max zMax v.Z
        printfn "worst Y spread of fully-snapped verts: %.2e mm (z span %.0f..%.0f)" worst zMin zMax
        Assert.IsTrue(worst < 1e-9, $"full-strength verts must share one exact Y, got {worst}")
        Assert.IsTrue(zMax - zMin > 70.0, "the exact-Y guarantee must span the whole plate")
        // Correcting the tilt is real movement at the plate's far ends:
        // ~0.35 mm at 0.5° over 80 mm, an order beyond any noise-sized move.
        Assert.IsTrue(snapped.MaxMoveMm > 0.25 && free.MaxMoveMm < 0.31,
                      $"the snap should visibly true the tilt (snapped {snapped.MaxMoveMm}, free {free.MaxMoveMm})")

    /// The state's side of the snap: the candidate is detected from the
    /// picks in WORLD space (the orientation the export will use), the
    /// checkbox flag rides into the preview, and the currency check treats
    /// the flag as a parameter like any other.
    [<TestMethod>]
    member _.Axis_snap_candidate_is_offered_and_applied_in_world_space () =
        let stl = Path.Combine(Path.GetTempPath(), $"meshorient_snap_{Guid.NewGuid():N}.stl")
        MeshIO.saveStlBinary stl (FlattenFixtures.wall sigma)
        try
            let s = MeshOrient.App.OrientState()
            s.Load stl |> ignore
            Assert.IsTrue(s.AxisSnapCandidate.IsNone, "no picks, no offer")
            for (x, z) in [ 5.0, 5.0; 55.0, 10.0; 5.0, 75.0 ] do
                let o = s.Offset
                s.PickAt { Raycast.Origin = Vec3.create (x + o.X) 1000.0 (z + o.Z)
                           Raycast.Direction = Vec3.create 0.0 -1.0 0.0 } |> ignore

            // The plate is Y-normal to within pick noise: the offer is Y.
            match s.AxisSnapCandidate with
            | None -> Assert.Fail "a Y-normal plate must produce a snap candidate"
            | Some(ax, off) ->
                printfn "candidate: axis %d, %.3f° off" ax off
                Assert.AreEqual(1, ax, "the plate is Y-normal")
                Assert.IsTrue(off < Flatten.axisSnapThresholdDegrees)

            match s.ComputeFlattenPreview(3.0 * sigma, 0.3, false, true) with
            | Error e -> Assert.Fail e
            | Ok pv ->
                Assert.IsTrue(pv.SnapOffAngleDegrees.IsSome, "snap flag must reach the core")
                Assert.IsTrue(pv.PlaneNormal.X = 0.0 && pv.PlaneNormal.Z = 0.0
                              && abs pv.PlaneNormal.Y = 1.0,
                              "rotation is identity here, so the snapped model-space normal is Y exactly")
                Assert.IsTrue(s.PreviewIsCurrent(3.0 * sigma, 0.3, false, true))
                Assert.IsFalse(s.PreviewIsCurrent(3.0 * sigma, 0.3, false, false),
                               "unchecking the snap box must stale the preview")

            // Snap off: same call, free fit, nothing reported.
            match s.ComputeFlattenPreview(3.0 * sigma, 0.3, false, false) with
            | Error e -> Assert.Fail e
            | Ok pv -> Assert.IsTrue(pv.SnapOffAngleDegrees.IsNone, "unchecked = free fit")

            // Rotate the model 30° about X: the picks ride along, their world
            // plane is now 30° off Y (and 60° off Z) — no axis is near, so
            // the offer must withdraw. World space, not model space.
            s.ApplyRotation(Mat3.rotDegrees 0 30.0)
            Assert.IsTrue(s.AxisSnapCandidate.IsNone,
                          "a plane far from every axis must not be offered a snap")
        finally
            if File.Exists stl then File.Delete stl

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
            // Picks by ray, exactly as clicks would arrive. Fixture
            // coordinates go through the state's own offset to become world
            // rays (it is zero on load, and rotation is identity).
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
            match s.ComputeFlattenPreview(floor, 0.3, false, false) with
            | Error e -> Assert.Fail e
            | Ok pv ->
                printfn "preview: %d verts, %d enclaves" pv.MovedVertexCount pv.EnclaveCount
                Assert.IsTrue(pv.MovedVertexCount > 10000, "should preview the whole plate")
                Assert.AreNotEqual(stamp0, s.PreviewStamp, "stamp must tick for the GL view")
                Assert.IsTrue(s.PreviewIsCurrent(floor, 0.3, false, false))
                Assert.IsFalse(s.PreviewIsCurrent(floor, 0.4, false, false), "other params = stale")
                Assert.IsFalse(s.PreviewIsCurrent(floor, 0.3, false, true), "snap toggle = stale")
                let untouched =
                    Array.forall2 (fun (a : Vec3) (b : Vec3) ->
                        a.X = b.X && a.Y = b.Y && a.Z = b.Z) before s.Mesh.Value.Vertices
                Assert.IsTrue(untouched, "preview must not move anything")

            // A new pick stales the preview — what is highlighted must always
            // be what APPLY would do.
            s.PickAt { Raycast.Origin = Vec3.create (40.0 + s.Offset.X) 1000.0 (10.0 + s.Offset.Z)
                       Raycast.Direction = Vec3.create 0.0 -1.0 0.0 } |> ignore
            Assert.IsFalse(s.PreviewIsCurrent(floor, 0.3, false, false), "pick edits must stale the preview")
            s.ComputeFlattenPreview(floor, 0.3, false, false) |> ignore

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
            s.ComputeFlattenPreview(3.0 * sigma, 0.3, false, false) |> ignore
            match s.ApplyFlatten() with
            | Error e -> Assert.Fail e
            | Ok _ -> ()

            let oriented, cleaned = s.ExportStl()
            Assert.IsTrue(cleaned.IsSome, "flattens applied -> a _cleaned file")
            toDelete <- oriented :: cleaned.Value :: toDelete
            Assert.IsTrue(oriented.EndsWith "_oriented.stl")
            Assert.IsTrue(cleaned.Value.EndsWith "_cleaned.stl")

            // Plate-noise spread in each file, measured about the median Y
            // (the plate's own level, whatever its absolute Y): the oriented
            // file must still carry the scan's jitter, the cleaned one must
            // not.
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

            // No orientation command was given, so the export must sit in the
            // SOURCE coordinate frame — the whole point of not centring on
            // load. The _oriented file is the input, byte-faithful.
            let src = MeshIO.load stl
            let sb, ab = Mesh.bounds src, Mesh.bounds a
            Assert.AreEqual(sb.Min.X, ab.Min.X, 1e-12, "no hidden translation")
            Assert.AreEqual(sb.Max.Y, ab.Max.Y, 1e-12, "no hidden translation")
            Assert.AreEqual(sb.Min.Z, ab.Min.Z, 1e-12, "no hidden translation")
        finally
            for f in toDelete do
                if File.Exists f then File.Delete f

    /// Undoing the second of two flattens must leave the first one counted:
    /// the count is what decides whether Export writes `_cleaned`, so if it
    /// drops to zero while a flatten is still baked in, that work is in no
    /// file at all.
    [<TestMethod>]
    member _.Undoing_one_of_two_flattens_keeps_the_other_in_the_export () =
        let s, stl = wallState ()
        let mutable toDelete = [ stl ]
        try
            pickPlate s
            let flatten ceiling =
                match s.ComputeFlattenPreview(3.0 * sigma, ceiling, false, false) with
                | Error e -> Assert.Fail e
                | Ok _ -> ()
                match s.ApplyFlatten() with
                | Error e -> Assert.Fail e; 0
                | Ok pv -> pv.MovedVertexCount
            let first = flatten 0.3
            let second = flatten 0.35
            Assert.AreEqual(first + second, s.FlattenedVertexCount)

            Assert.IsTrue(s.Undo(), "undoing a flatten is a mesh change")
            Assert.AreEqual(first, s.FlattenedVertexCount, "undo takes back only the second flatten")
            let oriented, cleaned = s.ExportStl()
            toDelete <- oriented :: Option.toList cleaned @ toDelete
            Assert.IsTrue(cleaned.IsSome, "the first flatten is still applied, so _cleaned must be written")

            Assert.IsTrue(s.Undo())
            Assert.AreEqual(0, s.FlattenedVertexCount, "both undone")
        finally
            for f in toDelete do
                if File.Exists f then File.Delete f

    /// A snapped preview targets "true Y" as it was under the rotation it was
    /// computed with. Once the model turns, that direction is no longer world
    /// Y, so the preview must not survive to be applied. An unsnapped preview
    /// is pure model space and does survive.
    [<TestMethod>]
    member _.Rotating_drops_a_snapped_preview_but_keeps_a_free_one () =
        let s, stl = wallState ()
        try
            // Tilt the plate 1.5 deg off world Y: inside the snap threshold.
            s.ApplyRotation(Mat3.rotDegrees 0 1.5)
            pickPlate s
            Assert.AreEqual(Some 1, s.AxisSnapCandidate |> Option.map fst, "Y snap should be offered")
            let floor = 3.0 * sigma

            // Free fit: survives a rotation.
            s.ComputeFlattenPreview(floor, 0.3, false, false) |> ignore
            s.ApplyRotation(Mat3.rotDegrees 2 90.0)
            Assert.IsTrue(s.PreviewIsCurrent(floor, 0.3, false, false), "a model-space preview outlives a turn")
            s.Undo() |> ignore

            // Snapped: dropped by the rotation, so it can never be applied stale.
            match s.ComputeFlattenPreview(floor, 0.3, false, true) with
            | Error e -> Assert.Fail e
            | Ok pv -> Assert.IsTrue(pv.SnapOffAngleDegrees.IsSome, "the preview should have snapped")
            match s.ApplySquare() with
            | Error e -> Assert.Fail e
            | Ok _ -> ()
            Assert.IsTrue(s.Preview.IsNone, "the rotation must drop the snapped preview")
            Assert.IsFalse(s.PreviewIsCurrent(floor, 0.3, false, true))
            match s.ApplyFlatten() with
            | Ok _ -> Assert.Fail "a snapped preview from before the rotation was applied"
            | Error _ -> ()

            // Re-previewed under the new rotation, the target is world Y exactly.
            match s.ComputeFlattenPreview(floor, 0.3, false, true) with
            | Error e -> Assert.Fail e
            | Ok pv ->
                let off = Fixtures.degreesBetween (Mat3.apply s.Rotation pv.PlaneNormal) Vec3.unitY
                printfn "re-previewed snap plane is %.2e deg off world Y" off
                Assert.IsTrue(off < 1e-6, $"snapped plane should be world Y, got {off} deg off")
        finally
            if File.Exists stl then File.Delete stl

    [<TestMethod>]
    member _.Rigid_undo_still_reports_no_mesh_change () =
        let s = MeshOrient.App.OrientState()
        s.Load Fixtures.syntheticPath |> ignore
        s.ApplyRotation(Mat3.rotDegrees 0 30.0)
        Assert.IsFalse(s.Undo(), "a rigid undo moves the matrix, not the vertices")
