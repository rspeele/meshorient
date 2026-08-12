namespace MeshOrient.Core.Tests

open System
open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.Core

[<TestClass>]
type GeometryTests () =

    [<TestMethod>]
    member _.Eigen_of_a_diagonal_matrix_is_sorted_ascending () =
        let m = Mat3.ofRows (Vec3.create 5.0 0.0 0.0) (Vec3.create 0.0 1.0 0.0)
                            (Vec3.create 0.0 0.0 3.0)
        let e = Geometry.eigenSymmetric3 m
        Assert.AreEqual(1.0, fst e[0], 1e-12)
        Assert.AreEqual(3.0, fst e[1], 1e-12)
        Assert.AreEqual(5.0, fst e[2], 1e-12)
        Assert.AreEqual(0.0, Fixtures.degreesBetween (snd e[0]) Vec3.unitY, 1e-6)
        Assert.AreEqual(0.0, Fixtures.degreesBetween (snd e[2]) Vec3.unitX, 1e-6)

    [<TestMethod>]
    member _.Eigen_handles_a_matrix_that_is_already_diagonal_without_dividing_by_zero () =
        // The Jacobi rotation divides by the off-diagonal element. An identity
        // matrix has none, so the guard has to hold.
        let e = Geometry.eigenSymmetric3 Mat3.identity
        for (v, _) in e do Assert.AreEqual(1.0, v, 1e-12)

    [<TestMethod>]
    member _.Fits_a_plane_through_exact_points () =
        // Points on z = 2x + 3y + 1, so the normal is (2, 3, -1) normalised.
        let pts =
            [| 0.0, 0.0; 1.0, 0.0; 0.0, 1.0; 2.0, -1.0; -3.0, 2.0 |]
            |> Array.map (fun (x, y) -> Vec3.create x y (2.0 * x + 3.0 * y + 1.0))
        let fit = Geometry.fitPlane pts
        Assert.AreEqual(0.0, fit.Rms, 1e-9, "exact points must fit exactly")
        let expected = Vec3.normalize (Vec3.create 2.0 3.0 -1.0)
        let angle = min (Fixtures.degreesBetween fit.Normal expected)
                        (Fixtures.degreesBetween fit.Normal -expected)
        Assert.AreEqual(0.0, angle, 1e-6, "normal direction")

    [<TestMethod>]
    member _.Plane_rms_reports_how_far_off_the_flat_a_pick_was () =
        let flat = [| Vec3.create 0.0 0.0 0.0; Vec3.create 10.0 0.0 0.0
                      Vec3.create 0.0 10.0 0.0; Vec3.create 10.0 10.0 0.0 |]
        Assert.AreEqual(0.0, (Geometry.fitPlane flat).Rms, 1e-9)
        // Lift one pick 1 mm off the flat — the sort of miss the number exists
        // to catch.
        let strayed = Array.copy flat
        strayed[3] <- Vec3.create 10.0 10.0 1.0
        let rms = (Geometry.fitPlane strayed).Rms
        printfn "one pick 1 mm off a 4-point flat -> rms %.4f mm" rms
        Assert.IsTrue(rms > 0.2, $"a 1 mm miss must show up in the rms, got {rms}")

    [<TestMethod>]
    member _.Fits_a_line_through_two_points () =
        let fit = Geometry.fitLine2 [| 0.0, 0.0; 3.0, 4.0 |]
        Assert.AreEqual(0.0, fit.Rms, 1e-12)
        Assert.AreEqual(0.6, abs fit.DirX, 1e-9)
        Assert.AreEqual(0.8, abs fit.DirY, 1e-9)

    /// The closed-form 2x2 eigenvector has two candidate expressions and one
    /// of them degenerates for an axis-aligned line. Both orientations must
    /// work or Straighten breaks on exactly the models it is easiest to aim at.
    [<TestMethod>]
    member _.Fits_axis_aligned_lines_in_both_orientations () =
        let horiz = Geometry.fitLine2 [| -5.0, 2.0; 0.0, 2.0; 7.0, 2.0 |]
        Assert.AreEqual(1.0, abs horiz.DirX, 1e-9, "horizontal line direction")
        Assert.AreEqual(0.0, abs horiz.DirY, 1e-9)
        Assert.AreEqual(0.0, horiz.Rms, 1e-9)
        let vert = Geometry.fitLine2 [| 2.0, -5.0; 2.0, 0.0; 2.0, 7.0 |]
        Assert.AreEqual(0.0, abs vert.DirX, 1e-9, "vertical line direction")
        Assert.AreEqual(1.0, abs vert.DirY, 1e-9)
        Assert.AreEqual(0.0, vert.Rms, 1e-9)

    [<TestMethod>]
    member _.Line_fit_averages_pick_error_rather_than_chasing_it () =
        // Five points along y = 0 with alternating +-0.1 error. A two-point
        // fit through the first and last would inherit their errors; the
        // least-squares fit through all five should come out flat.
        let pts = [| 0.0, 0.1; 10.0, -0.1; 20.0, 0.1; 30.0, -0.1; 40.0, 0.1 |]
        let fit = Geometry.fitLine2 pts
        let angle = atan2 (abs fit.DirY) (abs fit.DirX) * 180.0 / Math.PI
        printfn "5 picks with +-0.1 mm error over 40 mm -> %.4f deg, rms %.4f" angle fit.Rms
        Assert.IsTrue(angle < 0.1, $"averaged fit should be near flat, got {angle} deg")

    /// PCA has to identify which way the part points. It does NOT have to be
    /// exact: this frame is an L (a full-length rail with a grip hanging off
    /// the middle), and the principal axes of an L are not its bbox axes. The
    /// measured residual is ~1 degree of rotation in the X-Z plane, which
    /// inflates both extents — and which is precisely what the Straighten step
    /// exists to take out. Asserting a tight bbox here would be asserting
    /// something PCA never promised.
    [<TestMethod>]
    member _.Auto_orient_puts_length_on_x_and_thickness_on_y () =
        let scrambled =
            Mat3.mul (Mat3.rotDegrees 2 35.0)
                     (Mat3.mul (Mat3.rotDegrees 1 -50.0) (Mat3.rotDegrees 0 20.0))
        let m = Mesh.transform scrambled Vec3.zero Fixtures.synthetic.Value
        let oriented = Mesh.transform (Geometry.autoOrient m) Vec3.zero m
        let s = Bounds.size (Mesh.bounds oriented)
        printfn "auto-oriented bbox: %.1f x %.1f x %.1f mm (true 170 x 28.5 x 110)" s.X s.Y s.Z
        Assert.IsTrue(s.X > s.Z && s.Z > s.Y, $"expected X > Z > Y, got {s}")
        Assert.AreEqual(170.0, s.X, 5.0, "length along X")
        Assert.AreEqual(28.5, s.Y, 1.5, "thickness along Y")
        Assert.AreEqual(110.0, s.Z, 6.0, "height along Z")

    /// The property that matters more than accuracy: PCA must find the same
    /// intrinsic axes no matter how the scan happened to arrive. If it does
    /// not, two exports of the same part land in different places and nothing
    /// downstream can be repeated.
    [<TestMethod>]
    member _.Auto_orient_gives_the_same_answer_whatever_pose_the_scan_arrives_in () =
        let m = Fixtures.synthetic.Value
        let reference = Bounds.size (Mesh.bounds (Mesh.transform (Geometry.autoOrient m) Vec3.zero m))
        for (rx, ry, rz) in [ 20.0, -50.0, 35.0; -80.0, 10.0, 170.0; 5.0, 95.0, -120.0 ] do
            let pose = Mat3.mul (Mat3.rotDegrees 2 rz)
                                (Mat3.mul (Mat3.rotDegrees 1 ry) (Mat3.rotDegrees 0 rx))
            let posed = Mesh.transform pose Vec3.zero m
            let s = Bounds.size (Mesh.bounds (Mesh.transform (Geometry.autoOrient posed) Vec3.zero posed))
            printfn "pose (%g, %g, %g) -> %.2f x %.2f x %.2f" rx ry rz s.X s.Y s.Z
            Assert.AreEqual(reference.X, s.X, 0.5, $"X extent for pose ({rx}, {ry}, {rz})")
            Assert.AreEqual(reference.Y, s.Y, 0.5, $"Y extent for pose ({rx}, {ry}, {rz})")
            Assert.AreEqual(reference.Z, s.Z, 0.5, $"Z extent for pose ({rx}, {ry}, {rz})")

    /// The covariance is weighted by triangle area rather than built from raw
    /// vertices. On an evenly tessellated scan the two must agree — this pins
    /// that, so area weighting is an improvement in the uneven case rather
    /// than a different answer everywhere.
    [<TestMethod>]
    member _.Area_weighted_pca_agrees_with_vertex_pca_on_an_even_mesh () =
        let m = Fixtures.synthetic.Value
        // Independent reference: unweighted covariance of the raw vertices,
        // exactly what core.auto_orient does.
        let verts = m.Vertices
        let n = float verts.Length
        let mean = (Array.fold (+) Vec3.zero verts) / n
        let mutable xx, xy, xz, yy, yz, zz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        for v in verts do
            let d = v - mean
            xx <- xx + d.X * d.X
            xy <- xy + d.X * d.Y
            xz <- xz + d.X * d.Z
            yy <- yy + d.Y * d.Y
            yz <- yz + d.Y * d.Z
            zz <- zz + d.Z * d.Z
        let cov = Mat3.ofRows (Vec3.create xx xy xz) (Vec3.create xy yy yz) (Vec3.create xz yz zz)
        let e = Geometry.eigenSymmetric3 cov
        let vertexPca = Mat3.ofRows (snd e[2]) (snd e[0]) (snd e[1])
        let areaPca = Geometry.autoOrient m
        let worst =
            [| vertexPca.R0, areaPca.R0; vertexPca.R1, areaPca.R1; vertexPca.R2, areaPca.R2 |]
            |> Array.map (fun (a, b) ->
                // Axes are sign-ambiguous; compare as undirected lines.
                min (Fixtures.degreesBetween a b) (Fixtures.degreesBetween a -b))
            |> Array.max
        printfn "area-weighted vs vertex PCA: worst axis disagreement %.4f deg" worst
        Assert.IsTrue(worst < 1.0, $"expected agreement within 1 deg, got {worst}")
