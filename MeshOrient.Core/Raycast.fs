/// Ray/mesh intersection for picking points in the 3D view.
module MeshOrient.Core.Raycast

open System

type Ray =
    {   Origin : Vec3
        /// Expected to be unit length, so `Hit.Distance` is in model units.
        Direction : Vec3 }

type Hit =
    {   Distance : float
        Point : Vec3
        Triangle : int
        /// Unit face normal, flipped to face back along the ray. Scans have
        /// inconsistent winding often enough that trusting it would put pick
        /// markers inside the surface half the time.
        Normal : Vec3 }

/// Möller-Trumbore, double-sided.
let private hitTriangle (a : Vec3) (b : Vec3) (c : Vec3) (ray : Ray) : float =
    let e1 = b - a
    let e2 = c - a
    let pv = Vec3.cross ray.Direction e2
    let det = Vec3.dot e1 pv
    if abs det < 1e-12 then nan                     // ray parallel to the plane
    else
        let inv = 1.0 / det
        let tv = ray.Origin - a
        let u = Vec3.dot tv pv * inv
        if u < 0.0 || u > 1.0 then nan
        else
            let qv = Vec3.cross tv e1
            let v = Vec3.dot ray.Direction qv * inv
            if v < 0.0 || u + v > 1.0 then nan
            else
                let t = Vec3.dot e2 qv * inv
                if t > 1e-9 then t else nan

/// Nearest forward intersection of `ray` with `mesh`, or `None`.
///
/// Brute force over every triangle, chunked across cores. No BVH, deliberately:
/// this runs on a CLICK, not per frame. A 760k-triangle scan is a couple of
/// milliseconds spread over the cores, which is imperceptible, whereas a BVH is
/// a tree to build, keep in sync with every re-orientation, and get wrong.
let intersect (mesh : Mesh) (ray : Ray) : Hit option =
    let struct (t, tri) =
        Chunked.map mesh.TriangleCount (fun lo hi ->
            let mutable bt = infinity
            let mutable bi = -1
            for t in lo .. hi - 1 do
                let struct (a, b, c) = Mesh.triangle mesh t
                let d = hitTriangle a b c ray
                if not (Double.IsNaN d) && d < bt then
                    bt <- d
                    bi <- t
            struct (bt, bi))
        |> Array.fold (fun (struct (bt, _) as acc) (struct (t, i)) ->
            if i >= 0 && t < bt then struct (t, i) else acc) (struct (infinity, -1))
    if tri < 0 then None
    else
        let nrm = Mesh.faceNormal mesh tri
        // Point the normal back at the viewer regardless of winding.
        let nrm = if Vec3.dot nrm ray.Direction > 0.0 then -nrm else nrm
        Some { Distance = t
               Point = ray.Origin + ray.Direction * t
               Triangle = tri
               Normal = nrm }
