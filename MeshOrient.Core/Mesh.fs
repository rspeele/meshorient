namespace MeshOrient.Core

/// An axis-aligned bounding box. `Empty` is the identity for `expand`, so a
/// fold over an empty vertex array is well defined rather than a special case.
[<Struct>]
type Bounds =
    { Min : Vec3; Max : Vec3; IsEmpty : bool }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module Bounds =

    let empty = { Min = Vec3.zero; Max = Vec3.zero; IsEmpty = true }

    let expand (b : Bounds) (v : Vec3) =
        if b.IsEmpty then { Min = v; Max = v; IsEmpty = false }
        else { Min = Vec3.minOf b.Min v; Max = Vec3.maxOf b.Max v; IsEmpty = false }

    let ofPoints (points : Vec3 seq) = Seq.fold expand empty points

    let centre (b : Bounds) = if b.IsEmpty then Vec3.zero else (b.Min + b.Max) * 0.5
    let size (b : Bounds) = if b.IsEmpty then Vec3.zero else b.Max - b.Min
    let diagonal (b : Bounds) = Vec3.length (size b)


/// A triangle mesh: a vertex array plus a flat index array, three indices per
/// triangle.
///
/// STL arrives as a triangle soup (3N vertices, indices 0..3N-1) and is kept
/// that way — nothing here needs the duplicates welded: rendering wants flat
/// normals, the raycast walks triangles, and STL export writes triangles back
/// out. Welding would cost a sort over millions of vertices to make every
/// downstream step marginally slower.
type Mesh =
    {   Vertices : Vec3[]
        /// 3 entries per triangle, indexing `Vertices`.
        Indices : int[]
        /// Where it was loaded from. Used to name the export.
        SourcePath : string }

    member m.TriangleCount = m.Indices.Length / 3

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module Mesh =

    let create (vertices : Vec3[]) (indices : int[]) (path : string) =
        if indices.Length % 3 <> 0 then
            invalidArg "indices" $"index count {indices.Length} is not a multiple of 3"
        { Vertices = vertices; Indices = indices; SourcePath = path }

    let bounds (m : Mesh) = Bounds.ofPoints m.Vertices

    /// The three corners of triangle `t`.
    let inline triangle (m : Mesh) (t : int) =
        struct (m.Vertices[m.Indices[t * 3]],
                m.Vertices[m.Indices[t * 3 + 1]],
                m.Vertices[m.Indices[t * 3 + 2]])

    /// Rigid transform: rotate every vertex by `rotation`, then translate by
    /// `offset`. Triangles, winding and topology are untouched, which is the
    /// whole promise of this tool's export — nothing is resampled.
    let transform (rotation : Mat3) (offset : Vec3) (m : Mesh) =
        let out = Array.zeroCreate m.Vertices.Length
        System.Threading.Tasks.Parallel.For(0, m.Vertices.Length, fun i ->
            out[i] <- Mat3.apply rotation m.Vertices[i] + offset) |> ignore
        { m with Vertices = out }

    /// Bounds the mesh WOULD have under a rigid transform, without building
    /// the transformed vertex array.
    ///
    /// The app re-frames its three orthographic panels after every rotation,
    /// so this runs on every button press. Transforming the eight corners of
    /// the untransformed bounds instead would be cheaper but wrong — that box
    /// is a loose outer bound on a rotated mesh, and the panels would visibly
    /// zoom out as you turned the model.
    let transformedBounds (rotation : Mat3) (offset : Vec3) (m : Mesh) =
        let v = m.Vertices
        if v.Length = 0 then Bounds.empty
        else
            let chunks = max 1 (min 64 (System.Environment.ProcessorCount * 2))
            let per = (v.Length + chunks - 1) / chunks
            Array.Parallel.init chunks (fun ci ->
                let lo = ci * per
                let hi = min v.Length (lo + per)
                let mutable b = Bounds.empty
                for i in lo .. hi - 1 do
                    b <- Bounds.expand b (Mat3.apply rotation v[i] + offset)
                b)
            |> Array.fold (fun acc b ->
                if b.IsEmpty then acc
                elif acc.IsEmpty then b
                else { Min = Vec3.minOf acc.Min b.Min; Max = Vec3.maxOf acc.Max b.Max; IsEmpty = false })
                Bounds.empty

    /// Unit normal of triangle `t`, from the winding. Zero-area triangles —
    /// which real scans do contain — come back as the zero vector; callers
    /// that weight by area get the right answer for free, and the renderer
    /// draws a degenerate triangle as nothing anyway.
    let faceNormal (m : Mesh) (t : int) =
        let struct (a, b, c) = triangle m t
        Vec3.normalize (Vec3.cross (b - a) (c - a))

    /// Twice the area of triangle `t`, as a vector along its normal. Halving
    /// is left to the caller so area-weighted sums can skip it.
    let inline crossArea (m : Mesh) (t : int) =
        let struct (a, b, c) = triangle m t
        Vec3.cross (b - a) (c - a)

    /// Median edge length — a robust read on how finely the scan is
    /// tessellated. Used to size the pick marker so it reads the same on a
    /// coarse mesh and a dense one.
    let medianEdgeLength (m : Mesh) =
        if m.TriangleCount = 0 then 0.0
        else
            // One edge per triangle is a fair enough sample and keeps this
            // O(triangles) rather than O(3 x triangles) with a big sort.
            let lengths =
                Array.init m.TriangleCount (fun t ->
                    let struct (a, b, _) = triangle m t
                    Vec3.length (b - a))
            System.Array.Sort lengths
            lengths[lengths.Length / 2]
