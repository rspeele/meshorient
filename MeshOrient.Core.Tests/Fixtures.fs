namespace MeshOrient.Core.Tests

open System
open MeshOrient.Core

/// Shared test data and probes.
///
/// The fixture is `synthetic_frame_scan.stl` — a mock pistol
/// frame with known dimensions, deliberate scan defects (0.05 mm vertex
/// jitter, 2% of triangles dropped) and non-axis-aligned features.
///
/// Geometry that matters here (mm):
///   top rail   X -85..85, Z 30..50, Y -13..13   <- the flat TOP reference
///   grip walls X -40..40, Z -60..30, outer Y +-11
///   magwell window   X -20..10, Z -40..-10  (a hole in both walls)
///   trigger-bar boss X -20..25, Z 0..8, RIGHT side only, out to Y 14
///   diagonal rib     (14,-52)..(34,-18), right wall, out to Y 12.5
///   round boss       X 25 Z 15, LEFT wall, out to Y -14.5
module Fixtures =

    let syntheticPath = "synthetic_frame_scan.stl"

    /// Loaded once — it is ~760k triangles and every test wants the same one.
    let synthetic = lazy (MeshIO.load syntheticPath)

    /// Where the mesh's outermost surface sits along `dir` above the point
    /// `origin`, sampled over a small disc and keeping the FARTHEST-out hit.
    ///
    /// The cluster is not decoration: the synthetic drops 2% of its triangles
    /// at random, so a single ray falls through a hole often enough to make a
    /// test flaky, and when it does it hits the inner wall and reports a wildly
    /// wrong answer. Taking the outermost of several samples rides over the
    /// holes.
    let private sampleOuter (mesh : Mesh) (origin : Vec3) (dir : Vec3) (spread : Vec3 * Vec3) =
        let du, dv = spread
        let offsets =
            [| for a in -1 .. 1 do
                 for b in -1 .. 1 do
                     yield du * (float a * 0.5) + dv * (float b * 0.5) |]
        let hits =
            offsets
            |> Array.choose (fun o ->
                Raycast.intersect mesh { Origin = origin + o; Direction = dir })
        if hits.Length = 0 then None
        else
            // "Outermost" = smallest distance from the far-off ray origin.
            hits |> Array.minBy (fun h -> h.Distance) |> Some

    /// A point on the frame's RIGHT face (+Y) beneath side-view (x, z).
    /// Fails loudly rather than returning a silently wrong point: a test that
    /// probes a spot with no surface is a broken test, not a passing one.
    let rightFaceAt (mesh : Mesh) (x : float) (z : float) : Vec3 =
        let origin = Vec3.create x 1000.0 z
        match sampleOuter mesh origin (Vec3.create 0.0 -1.0 0.0)
                  (Vec3.create 1.0 0.0 0.0, Vec3.create 0.0 0.0 1.0) with
        | Some h -> h.Point
        | None -> failwithf "no right-face surface above (x=%g, z=%g)" x z

    /// A point on the frame's TOP face (+Z) above plan-view (x, y).
    let topFaceAt (mesh : Mesh) (x : float) (y : float) : Vec3 =
        let origin = Vec3.create x y 1000.0
        match sampleOuter mesh origin (Vec3.create 0.0 0.0 -1.0)
                  (Vec3.create 1.0 0.0 0.0, Vec3.create 0.0 1.0 0.0) with
        | Some h -> h.Point
        | None -> failwithf "no top surface above (x=%g, y=%g)" x y

    /// Four spots on the grip's flat right wall, chosen to dodge every
    /// deliberate awkwardness in the synthetic: clear of the magwell window,
    /// clear of the trigger-bar boss (X -20..25, Z 0..8) and clear of the
    /// diagonal rib (which runs X 14..34 over Z -52..-18).
    let flatRightWallProbes = [| -30.0, 20.0; 30.0, 20.0; -30.0, -50.0; 30.0, -50.0 |]

    /// Spots along the top rail, which is a single flat plate at Z = 50.
    let railTopProbes = [| -60.0, 0.0; -20.0, 0.0; 20.0, 0.0; 60.0, 0.0 |]

    let degreesBetween (a : Vec3) (b : Vec3) =
        let c = Vec3.dot (Vec3.normalize a) (Vec3.normalize b)
        acos (Math.Clamp(c, -1.0, 1.0)) * 180.0 / Math.PI
