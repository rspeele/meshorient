namespace MeshOrient.Core.Tests

open System
open System.IO
open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.Core
open MeshOrient.App

/// The app's state machine, driven the way the buttons drive it.
///
/// `OrientState` touches no Avalonia and no GL — it is the whole tool minus
/// the window — so the sequence a user actually performs can be run here
/// instead of being clicked through by hand every time something changes.
[<TestClass>]
type OrientStateTests () =

    let synthetic = Fixtures.syntheticPath

    /// The world-space ray the 3D view would build for a click that lands on
    /// the frame's right face above side-view (x, z).
    let rayFromRight (x : float) (z : float) =
        {   Raycast.Origin = Vec3.create x 1000.0 z
            Raycast.Direction = Vec3.create 0.0 -1.0 0.0 }

    let loaded () =
        let s = OrientState()
        s.Load synthetic |> ignore
        s

    [<TestMethod>]
    member _.Loading_centres_the_model_on_the_origin () =
        let s = loaded ()
        let c = Bounds.centre s.Bounds
        printfn "centre after load: %A" c
        Assert.AreEqual(0.0, c.X, 1e-6, "X")
        Assert.AreEqual(0.0, c.Y, 1e-6, "Y")
        Assert.AreEqual(0.0, c.Z, 1e-6, "Z")

    [<TestMethod>]
    member _.Rotating_recentres_so_the_panels_stay_framed () =
        let s = loaded ()
        s.ApplyRotation(Mat3.rotDegrees 0 37.0)
        let c = Bounds.centre s.Bounds
        Assert.AreEqual(0.0, Vec3.length c, 1e-6, "still centred after a rotation")
        // And the bbox really did change shape, i.e. the rotation applied.
        let size = Bounds.size s.Bounds
        Assert.IsTrue(size.Y > 40.0, $"a 37 deg roll should thicken Y, got {size.Y}")

    [<TestMethod>]
    member _.Auto_orient_replaces_rather_than_accumulates () =
        // Pressing the button twice must not move the model twice — "auto
        // orient" can only mean one destination.
        let s = loaded ()
        s.ApplyRotation(Mat3.rotDegrees 1 25.0)
        s.AutoOrient()
        let first = Bounds.size s.Bounds
        s.AutoOrient()
        let second = Bounds.size s.Bounds
        printfn "auto-orient once %A, twice %A" first second
        Assert.AreEqual(first.X, second.X, 1e-9)
        Assert.AreEqual(first.Y, second.Y, 1e-9)
        Assert.AreEqual(first.Z, second.Z, 1e-9)

    /// Picks are stored in MODEL space, so a later re-orientation carries them
    /// along instead of stranding them in mid-air. This is the property that
    /// makes "press Square twice and it reads 0.000" work.
    [<TestMethod>]
    member _.Picks_follow_the_model_through_a_re_orientation () =
        let s = loaded ()
        s.Stage <- Square
        match s.PickAt(rayFromRight 0.0 20.0) with
        | None -> Assert.Fail "expected the ray to hit the right wall"
        | Some before ->
            s.ApplyRotation(Mat3.rotDegrees 2 90.0)
            let after = s.ActivePicksWorld[0]
            printfn "pick before %A, after a 90 deg Z rotation %A" before after
            // Rotating about Z about the (centred) origin: the pick must land
            // where that same bit of surface landed, not stay put.
            Assert.AreEqual(-before.Y, after.X, 0.02, "pick X after rotation")
            Assert.AreEqual(before.X, after.Y, 0.02, "pick Y after rotation")
            Assert.AreEqual(before.Z, after.Z, 0.02, "pick Z unchanged by a Z rotation")

    [<TestMethod>]
    member _.Square_needs_three_picks_and_says_so () =
        let s = loaded ()
        s.Stage <- Square
        s.PickAt(rayFromRight 0.0 20.0) |> ignore
        match s.ApplySquare() with
        | Ok _ -> Assert.Fail "one pick cannot define a plane"
        | Error msg ->
            printfn "%s" msg
            Assert.IsTrue(msg.Contains "3", "the message should say how many are needed")

    /// The sequence a user actually performs, start to finish.
    [<TestMethod>]
    member _.Full_run_tilted_scan_to_square_exported_stl () =
        let s = OrientState()
        // A scan that arrives crooked, as they do.
        let tilted = Path.Combine(Path.GetTempPath(), $"meshorient_run_{Guid.NewGuid():N}.stl")
        let src = MeshIO.load synthetic
        let crooked =
            Mat3.mul (Mat3.rotDegrees 0 1.4) (Mat3.mul (Mat3.rotDegrees 2 -0.7) (Mat3.rotDegrees 1 12.0))
        MeshIO.saveStlBinary tilted (Mesh.transform crooked Vec3.zero src)
        try
            s.Load tilted |> ignore
            s.AutoOrient()

            // Square off four points on the flat right wall.
            s.Stage <- Square
            let mutable picked = 0
            for (x, z) in Fixtures.flatRightWallProbes do
                if (s.PickAt(rayFromRight x z)).IsSome then picked <- picked + 1
            printfn "picked %d of 4 wall points" picked
            Assert.IsTrue(picked >= 3, $"needed 3+ picks on the wall, landed {picked}")

            match s.ApplySquare() with
            | Error e -> Assert.Fail e
            | Ok r ->
                printfn "squared by %.3f deg, picks coplanar to +-%.4f mm" r.TiltDegrees r.RmsMm
                Assert.IsTrue(r.RmsMm < 0.15, $"picks should be on one flat face, rms {r.RmsMm}")

            // Press it again: a squared model must re-measure as square. The
            // picks came along because they are model-space.
            match s.ApplySquare() with
            | Error e -> Assert.Fail e
            | Ok again ->
                printfn "second Square reads %.4f deg" again.TiltDegrees
                Assert.IsTrue(again.TiltDegrees < 0.05,
                              $"a squared model must read square, got {again.TiltDegrees}")

            // Export, then check the written file is genuinely square by
            // measuring it the same way f2s's test does.
            let out = s.ExportOrientedStl()
            try
                Assert.IsTrue(File.Exists out, "export should have written a file")
                let back = MeshIO.load out
                Assert.AreEqual(src.TriangleCount, back.TriangleCount,
                                "export must be a rigid transform, not a remesh")
                let ys =
                    Fixtures.flatRightWallProbes
                    |> Array.map (fun (x, z) -> (Fixtures.rightFaceAt back x z).Y)
                printfn "re-imported oriented STL, right-face heights: %A"
                        (ys |> Array.map (fun v -> Math.Round(v, 3)))
                Assert.IsTrue(Array.max ys - Array.min ys < 0.15,
                              $"the exported scan should already be square, got {ys}")
            finally
                if File.Exists out then File.Delete out
        finally
            if File.Exists tilted then File.Delete tilted

    [<TestMethod>]
    member _.Undo_puts_the_orientation_back () =
        let s = loaded ()
        let before = Bounds.size s.Bounds
        s.ApplyRotation(Mat3.rotDegrees 1 33.0)
        let moved = Bounds.size s.Bounds
        Assert.IsTrue(abs (moved.X - before.X) > 1.0, "the rotation should have changed the bbox")
        s.Undo()
        let back = Bounds.size s.Bounds
        Assert.AreEqual(before.X, back.X, 1e-9, "X restored")
        Assert.AreEqual(before.Z, back.Z, 1e-9, "Z restored")

    [<TestMethod>]
    member _.Square_and_straighten_keep_separate_pick_lists () =
        // They reference different faces; sharing one list would mean each
        // stage silently consuming the other's points.
        let s = loaded ()
        s.Stage <- Square
        s.PickAt(rayFromRight 0.0 20.0) |> ignore
        s.PickAt(rayFromRight 30.0 20.0) |> ignore
        s.Stage <- Straighten
        Assert.AreEqual(0, s.ActivePicks.Count, "Straighten starts with its own empty list")
        s.PickAt({ Raycast.Origin = Vec3.create 0.0 0.0 1000.0
                   Raycast.Direction = Vec3.create 0.0 0.0 -1.0 }) |> ignore
        Assert.AreEqual(1, s.ActivePicks.Count)
        s.Stage <- Square
        Assert.AreEqual(2, s.ActivePicks.Count, "Square's picks are still there")

    [<TestMethod>]
    member _.Marker_radius_scales_with_the_model () =
        let s = loaded ()
        let big = s.MarkerRadius
        printfn "marker radius on a 200 mm frame: %.3f mm" big
        Assert.IsTrue(big > 0.5 && big < 5.0, $"expected a few mm, got {big}")
