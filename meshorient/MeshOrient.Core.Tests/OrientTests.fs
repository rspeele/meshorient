namespace MeshOrient.Core.Tests

open System
open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.Core

[<TestClass>]
type RaycastTests () =

    [<TestMethod>]
    member _.Finds_the_outer_right_wall () =
        let m = Fixtures.synthetic.Value
        // (0, 20) is plain grip wall: above the trigger-bar boss (Z 0..8),
        // clear of the magwell window and of the diagonal rib.
        let hit = Raycast.intersect m { Origin = Vec3.create 0.0 1000.0 20.0
                                        Direction = Vec3.create 0.0 -1.0 0.0 }
        match hit with
        | None -> Assert.Fail "expected to hit the grip's right wall"
        | Some h ->
            printfn "right wall at (0, 20): y = %.3f, normal %A" h.Point.Y h.Normal
            Assert.AreEqual(11.0, h.Point.Y, 0.25, "outer face of the right wall")
            Assert.IsTrue(h.Normal.Y > 0.9, $"normal should face the ray back, got {h.Normal}")

    [<TestMethod>]
    member _.Returns_nothing_when_the_ray_misses () =
        let m = Fixtures.synthetic.Value
        let hit = Raycast.intersect m { Origin = Vec3.create 500.0 1000.0 500.0
                                        Direction = Vec3.create 0.0 -1.0 0.0 }
        Assert.IsTrue(hit.IsNone, "a ray well clear of the scan must not hit anything")

    [<TestMethod>]
    member _.Takes_the_nearest_surface_not_just_any () =
        let m = Fixtures.synthetic.Value
        // The trigger-bar boss stands proud to Y = 14 over X -20..25, Z 0..8.
        // A ray there must stop on the boss, not carry on to the wall behind.
        let hit = Raycast.intersect m { Origin = Vec3.create 0.0 1000.0 4.0
                                        Direction = Vec3.create 0.0 -1.0 0.0 }
        match hit with
        | None -> Assert.Fail "expected to hit the trigger-bar boss"
        | Some h -> Assert.AreEqual(14.0, h.Point.Y, 0.25, "boss face, not the wall behind it")


[<TestClass>]
type SquareTests () =

    /// The regression f2s's `test_gui.py` runs, ported: introduce a tilt PCA
    /// would plausibly have left behind, then square it back up off four picks
    /// on the grip's flat right wall.
    [<TestMethod>]
    member _.Recovers_a_deliberately_introduced_tilt () =
        let tilt = Mat3.mul (Mat3.rotDegrees 0 1.2) (Mat3.rotDegrees 2 -0.6)
        let tilted = Mesh.transform tilt Vec3.zero Fixtures.synthetic.Value
        let picks =
            Fixtures.flatRightWallProbes
            |> Array.map (fun (x, z) -> Fixtures.rightFaceAt tilted x z)

        let before = Orient.squareToAxis 1 picks
        printfn "tilt before: %.3f deg, picks coplanar to +-%.4f mm"
                before.TiltDegrees before.RmsMm
        Assert.IsTrue(before.TiltDegrees > 1.0 && before.TiltDegrees < 2.0,
                      $"should see the ~1.34 deg tilt we introduced, got {before.TiltDegrees}")
        // The picks are on one flat face of a scan with 0.05 mm of jitter, so
        // this is the scan's own flatness and nothing more.
        Assert.IsTrue(before.RmsMm < 0.1, $"picks should be coplanar, rms was {before.RmsMm}")

        // Rotating the picks themselves must land them on one Y.
        let moved = picks |> Array.map (Mat3.apply before.Rotation)
        let spread = (moved |> Array.map (fun p -> p.Y) |> Array.max)
                     - (moved |> Array.map (fun p -> p.Y) |> Array.min)
        printfn "picked plane spread after squaring: %.4f mm" spread
        Assert.IsTrue(spread < 4.0 * before.RmsMm + 1e-6,
                      $"squared picks should share one Y, spread was {spread}")

        // And the mesh must actually move with them: re-measure the wall.
        let squared = Mesh.transform before.Rotation Vec3.zero tilted
        let ys =
            Fixtures.flatRightWallProbes
            |> Array.map (fun (x, z) -> (Fixtures.rightFaceAt squared x z).Y)
        printfn "right-face heights after squaring: %A" (ys |> Array.map (fun v -> Math.Round(v, 3)))
        Assert.IsTrue(Array.max ys - Array.min ys < 0.15,
                      $"the flat wall should read one thickness end to end, got {ys}")

        // Levelling a level model is a no-op — the "press it twice and it says
        // 0.00" property that makes the readout trustworthy.
        let again =
            Orient.squareToAxis 1
                (Fixtures.flatRightWallProbes
                 |> Array.map (fun (x, z) -> Fixtures.rightFaceAt squared x z))
        printfn "tilt after: %.4f deg" again.TiltDegrees
        Assert.IsTrue(again.TiltDegrees < 0.05,
                      $"a squared model must re-measure as square, got {again.TiltDegrees}")

    [<TestMethod>]
    member _.Squaring_does_not_spin_the_model_about_the_target_axis () =
        // The whole reason for Rodrigues-about-(n x y) rather than any old
        // rotation that happens to work: it must not undo stage 1's work.
        let tilt = Mat3.rotDegrees 0 3.0
        let tilted = Mesh.transform tilt Vec3.zero Fixtures.synthetic.Value
        let picks =
            Fixtures.flatRightWallProbes
            |> Array.map (fun (x, z) -> Fixtures.rightFaceAt tilted x z)
        let r = (Orient.squareToAxis 1 picks).Rotation
        // A pure tilt about X, corrected, must leave the bore direction alone.
        let bore = Mat3.apply r (Mat3.apply tilt Vec3.unitX)
        printfn "bore after squaring a 3 deg X-tilt: %A" bore
        Assert.IsTrue(Fixtures.degreesBetween bore Vec3.unitX < 0.2,
                      $"squaring should not have rotated the bore, got {bore}")

    [<TestMethod>]
    member _.Tilt_components_split_into_the_two_visible_leans () =
        // f2s draws these two numbers on the front and top views: a tilt about
        // X leans the model in the back view, one about Z leans it in the top.
        let tilted =
            Mesh.transform (Mat3.mul (Mat3.rotDegrees 0 1.2) (Mat3.rotDegrees 2 -0.6))
                           Vec3.zero Fixtures.synthetic.Value
        let picks =
            Fixtures.flatRightWallProbes
            |> Array.map (fun (x, z) -> Fixtures.rightFaceAt tilted x z)
        let aboutX, aboutZ = Orient.tiltComponents picks
        printfn "tilt components: about X %.3f deg, about Z %.3f deg" aboutX aboutZ
        Assert.AreEqual(1.2, aboutX, 0.1, "lean visible in the back view")
        Assert.AreEqual(0.6, abs aboutZ, 0.1, "lean visible in the top view")


[<TestClass>]
type CenterlineTests () =

    /// The user's own worked example: 3 picks within ±0.5 of one Y, 4 within
    /// ±0.5 of another — a clean two-cluster split, centred on the means.
    [<TestMethod>]
    member _.Two_clusters_centre_on_their_means () =
        let ys = [| -30.1; -29.9; -30.0; 9.8; 10.1; 10.0; 9.9 |]
        match Orient.yCenterline 1.0 ys with
        | Error e -> Assert.Fail $"expected a clean split, got {e}"
        | Ok r ->
            // c1 = -30.0, c2 = 9.95 -> shift +10.025, faces at +-19.975.
            Assert.AreEqual(10.025, r.ShiftY, 1e-9, "shift")
            Assert.AreEqual(19.975, r.HalfWidth, 1e-9, "half width")

    [<TestMethod>]
    member _.Two_points_are_two_clusters_of_one () =
        match Orient.yCenterline 1.0 [| 11.3; -10.7 |] with
        | Error e -> Assert.Fail $"two points on two faces must work, got {e}"
        | Ok r ->
            Assert.AreEqual(-0.3, r.ShiftY, 1e-9)
            Assert.AreEqual(11.0, r.HalfWidth, 1e-9)

    [<TestMethod>]
    member _.Three_groups_are_rejected_with_the_count () =
        match Orient.yCenterline 1.0 [| -30.0; -29.8; 0.0; 0.2; 10.0; 10.1 |] with
        | Error(Orient.WrongClusterCount n) -> Assert.AreEqual(3, n)
        | other -> Assert.Fail $"expected WrongClusterCount 3, got {other}"

    [<TestMethod>]
    member _.One_group_is_rejected_with_the_count () =
        match Orient.yCenterline 1.0 [| 10.0; 10.2; 10.4 |] with
        | Error(Orient.WrongClusterCount n) -> Assert.AreEqual(1, n)
        | other -> Assert.Fail $"expected WrongClusterCount 1, got {other}"

    /// A chain of sub-tolerance gaps can accumulate into a spread far wider
    /// than any face is noisy — that is a smeared selection, not a face, and
    /// letting it through would let one bad pick drag the centreline.
    [<TestMethod>]
    member _.A_loose_cluster_is_rejected_by_spread_not_just_gaps () =
        let ys = [| -30.0; 10.0; 10.9; 11.8; 12.7 |]     // gaps 0.9 -> one "cluster" 2.7 wide
        match Orient.yCenterline 1.0 ys with
        | Error(Orient.ClusterTooLoose s) -> Assert.AreEqual(2.7, s, 1e-9)
        | other -> Assert.Fail $"expected ClusterTooLoose, got {other}"


[<TestClass>]
type StraightenTests () =

    /// The second alignment: get the bore pointing straight down X off picks
    /// on a flat top reference (here the top rail, standing in for a slide
    /// top), WITHOUT disturbing the side-face squaring already done.
    [<TestMethod>]
    member _.Recovers_a_known_roll_from_top_face_picks () =
        let rolled = Mesh.transform (Mat3.rotDegrees 1 7.0) Vec3.zero Fixtures.synthetic.Value
        let picks = Fixtures.railTopProbes |> Array.map (fun (x, y) -> Fixtures.topFaceAt rolled x y)
        let r = Orient.straightenAbout 1 picks
        printfn "straighten: %.4f deg (expect -7), picks collinear to +-%.4f mm"
                r.AngleDegrees r.RmsMm
        Assert.AreEqual(-7.0, r.AngleDegrees, 0.05, "must undo the roll we applied")
        Assert.IsTrue(r.RmsMm < 0.1, $"rail top is flat; rms was {r.RmsMm}")

    /// The constraint that justifies rotating about Y rather than X. Rotating
    /// about X by any amount tips the side face straight back out of square,
    /// undoing stage 2; rotation about the locked axis is the only remaining
    /// freedom, and it is exactly the one that swings the bore in the side
    /// view. If someone "fixes" this to rotX, this test is what stops them.
    [<TestMethod>]
    member _.Straightening_leaves_the_locked_axis_exactly_alone () =
        let rolled = Mesh.transform (Mat3.rotDegrees 1 7.0) Vec3.zero Fixtures.synthetic.Value
        let picks = Fixtures.railTopProbes |> Array.map (fun (x, y) -> Fixtures.topFaceAt rolled x y)
        let r = Orient.straightenAbout 1 picks
        let yAfter = Mat3.apply r.Rotation Vec3.unitY
        printfn "locked axis after straightening: %A" yAfter
        Assert.AreEqual(0.0, Fixtures.degreesBetween yAfter Vec3.unitY, 1e-9,
                        "a rotation about Y must fix Y")
        // For contrast, and to document why: the same angle about X would not.
        let wrong = Mat3.apply (Mat3.rotDegrees 0 r.AngleDegrees) Vec3.unitY
        Assert.IsTrue(Fixtures.degreesBetween wrong Vec3.unitY > 6.0,
                      "rotating about X would have broken the squaring")

    [<TestMethod>]
    member _.Rail_top_comes_out_level_end_to_end () =
        let rolled = Mesh.transform (Mat3.rotDegrees 1 7.0) Vec3.zero Fixtures.synthetic.Value
        let picks = Fixtures.railTopProbes |> Array.map (fun (x, y) -> Fixtures.topFaceAt rolled x y)
        let r = Orient.straightenAbout 1 picks
        let straight = Mesh.transform r.Rotation Vec3.zero rolled
        let zs = Fixtures.railTopProbes |> Array.map (fun (x, y) -> (Fixtures.topFaceAt straight x y).Z)
        printfn "rail-top heights after straightening: %A" (zs |> Array.map (fun v -> Math.Round(v, 3)))
        Assert.IsTrue(Array.max zs - Array.min zs < 0.2,
                      $"the rail top should read one height end to end, got {zs}")

    [<TestMethod>]
    member _.Straightens_from_only_two_picks () =
        let rolled = Mesh.transform (Mat3.rotDegrees 1 -4.5) Vec3.zero Fixtures.synthetic.Value
        let picks =
            [| Fixtures.topFaceAt rolled -60.0 0.0; Fixtures.topFaceAt rolled 60.0 0.0 |]
        let r = Orient.straightenAbout 1 picks
        printfn "two-pick straighten: %.4f deg (expect 4.5)" r.AngleDegrees
        Assert.AreEqual(4.5, r.AngleDegrees, 0.1, "two picks are enough to define the line")

    [<TestMethod>]
    member _.Takes_the_small_correction_not_the_end_for_end_flip () =
        // The fitted line direction is sign-ambiguous. Taking the wrong sign
        // gives a rotation 180 degrees out, which spins the model end for end
        // while still making the reference face "level".
        let rolled = Mesh.transform (Mat3.rotDegrees 1 6.0) Vec3.zero Fixtures.synthetic.Value
        let picks = Fixtures.railTopProbes |> Array.map (fun (x, y) -> Fixtures.topFaceAt rolled x y)
        let r = Orient.straightenAbout 1 picks
        Assert.IsTrue(abs r.AngleDegrees < 90.0,
                      $"expected the minimal correction, got {r.AngleDegrees} deg")
        // Reversing the pick order must not change the answer.
        let reversed = Orient.straightenAbout 1 (Array.rev picks)
        Assert.AreEqual(r.AngleDegrees, reversed.AngleDegrees, 1e-9,
                        "the result must not depend on the order picks were made in")

    [<TestMethod>]
    member _.Straighten_needs_two_picks () =
        let threw =
            try Orient.straightenAbout 1 [| Vec3.zero |] |> ignore; false
            with :? ArgumentException -> true
        Assert.IsTrue(threw, "one pick cannot define a direction")
