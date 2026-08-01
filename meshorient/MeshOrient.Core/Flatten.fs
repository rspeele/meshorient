/// FLATTEN: remove scan noise on a picked flat face by snapping its vertices
/// onto the true plane.
///
/// The naive version — snap every vertex within k of the plane — fails two
/// ways (user-illustrated, both from real scans):
///
///   1. The infinite plane slices through unrelated geometry elsewhere on the
///      model, and snapping puts ripples in a surface that was never flat.
///      Fix: FLOOD FILL from the picked triangles, walking face-to-face, so
///      only the region you pointed at is touched. The walk is gated by a
///      NORMAL test as well as distance — without it the flood climbs the
///      first row of every adjoining wall (those verts sit within k of the
///      plane) and smears a skirt around each wall base.
///
///   2. Where the flat rolls into a gentle curve, a hard k-threshold snaps
///      one side of a line and not the other, printing a crease into a
///      surface that was smooth. Fix: feather in DISTANCE space, not
///      perimeter space. Snap strength is a smoothstep of the vertex's own
///      |distance|: full inside `floor` (the noise band), fading to zero at
///      `ceiling` (k). On a true flat everything is inside the noise band, so
///      a sharp box edge still gets the full snap right up to the arris —
///      feathering by distance-to-perimeter would have rounded it, which is
///      why that obvious alternative is wrong. On a curve the distance grows
///      smoothly, so the correction fades smoothly, and by the time the flood
///      hits its k-wall the correction is already zero: the wall stops
///      mattering.
///
/// Everything here is MODEL space and pure: preview computes what would move
/// and to where; the caller decides whether to commit.
module MeshOrient.Core.Flatten

open System
open System.Collections.Generic

// ------------------------------------------------------------------ adjacency

/// Face-to-face connectivity over the triangle soup, via an exact-bit weld.
///
/// STL is a soup on disk even when the mesh is watertight — MeshMixer (and
/// every repair tool checked) emits bit-identical coordinates for shared
/// vertices, so the weld needs no tolerance and cannot falsely merge a thin
/// wall. Built once per mesh and cached by the caller: positions move when a
/// flatten is applied, but coincident copies move identically, so the
/// topology this encodes never changes.
type Adjacency =
    {   /// vertex index -> canonical position id
        Canonical : int[]
        CanonicalCount : int
        /// CSR layout: canonical id c owns vertex indices
        /// CopyItems[CopyStart[c] .. CopyStart[c+1] - 1] — every coincident
        /// copy, so a snap can move all of them together instead of cracking
        /// the mesh open along triangle boundaries.
        CopyStart : int[]
        CopyItems : int[]
        /// face*3 + edge -> the face across that edge, or -1 at a hole.
        /// Holes are real (a scan with dropped triangles); the flood simply
        /// does not cross them.
        Neighbor : int[] }

let buildAdjacency (m : Mesh) : Adjacency =
    let v = m.Vertices
    let canonical = Array.zeroCreate v.Length
    let map = Dictionary<struct (float * float * float), int>(v.Length)
    let mutable next = 0
    for i in 0 .. v.Length - 1 do
        let p = v[i]
        let key = struct (p.X, p.Y, p.Z)
        match map.TryGetValue key with
        | true, id -> canonical[i] <- id
        | _ ->
            map[key] <- next
            canonical[i] <- next
            next <- next + 1

    // CSR of copies per canonical id.
    let start = Array.zeroCreate (next + 1)
    for i in 0 .. v.Length - 1 do
        start[canonical[i] + 1] <- start[canonical[i] + 1] + 1
    for c in 1 .. next do
        start[c] <- start[c] + start[c - 1]
    let items = Array.zeroCreate v.Length
    let cursor = Array.copy start
    for i in 0 .. v.Length - 1 do
        let c = canonical[i]
        items[cursor[c]] <- i
        cursor[c] <- cursor[c] + 1

    // Pair faces across shared (canonical) edges.
    let nf = m.TriangleCount
    let neighbor = Array.create (nf * 3) -1
    let pending = Dictionary<struct (int * int), struct (int * int)>(nf * 3 / 2)
    for f in 0 .. nf - 1 do
        for e in 0 .. 2 do
            let a = canonical[m.Indices[f * 3 + e]]
            let b = canonical[m.Indices[f * 3 + (e + 1) % 3]]
            if a <> b then                           // skip degenerate edges
                let key = struct (min a b, max a b)
                match pending.TryGetValue key with
                | true, struct (f2, e2) ->
                    neighbor[f * 3 + e] <- f2
                    neighbor[f2 * 3 + e2] <- f
                    // A third face on this edge (non-manifold) just fails to
                    // pair — the flood treats it as a wall, which is the
                    // conservative reading.
                    pending.Remove key |> ignore
                | _ -> pending[key] <- struct (f, e)
    {   Canonical = canonical
        CanonicalCount = next
        CopyStart = start
        CopyItems = items
        Neighbor = neighbor }

// ------------------------------------------------------------------- flatten

type Params =
    {   /// Full-snap zone: |distance| <= this is treated as pure noise and
        /// snapped all the way. Default to ~3x the plane-fit RMS of the picks
        /// — the scan's own noise floor, measured, not guessed.
        FloorMm : float
        /// Capture ceiling (the k). Beyond this nothing moves, and the snap
        /// strength has already feathered to zero on the way there.
        CeilingMm : float
        /// A face joins the flood only if its normal is within this many
        /// degrees of the plane normal. Real scan flats jitter by ~8 degrees;
        /// 30 accepts them with margin and still rejects any genuine chamfer.
        NormalGateDegrees : float }

let defaults = { FloorMm = 0.075; CeilingMm = 0.3; NormalGateDegrees = 30.0 }

type Preview =
    {   /// The refined plane (unit normal, oriented outward like the picks).
        PlaneNormal : Vec3
        PlaneOrigin : Vec3
        /// Faces the flood captured — the green highlight.
        CapturedFaces : int[]
        /// Faces in pockets completely surrounded by captured faces — the
        /// yellow highlight. Possibly a genuine feature, possibly a dent the
        /// gates rejected; either way the one thing they should never be is
        /// invisible.
        EnclaveFaces : int[]
        EnclaveCount : int
        /// Distinct connected islands the seeds landed in (a serration flat
        /// per pick, say).
        Islands : int
        /// vertex index -> new position, one entry per soup copy, so applying
        /// is a plain scatter.
        Moves : struct (int * Vec3)[]
        MovedVertexCount : int
        RmsBeforeMm : float
        RmsAfterMm : float
        MaxMoveMm : float }

/// Compute what a flatten WOULD do. Pure; nothing is modified.
///
///   seedFaces  — the triangles the picks hit (each seeds its own island)
///   planePts   — the picked points (model space), seeding the plane fit
///   normalHint — sum of pick-surface normals; orients the plane
let preview (m : Mesh) (adj : Adjacency) (seedFaces : int[]) (planePts : Vec3[])
            (normalHint : Vec3) (prm : Params) : Preview =
    if planePts.Length < 3 then invalidArg "planePts" "need at least 3 points to define the plane"
    let ceiling = max prm.CeilingMm 1e-6
    // floor = ceiling degenerates to a hard threshold snap — allowed (the
    // tests use it as the "what the naive version would have done" baseline).
    let floor = Math.Clamp(prm.FloorMm, 0.0, ceiling)
    let cosGate = cos (prm.NormalGateDegrees * Math.PI / 180.0)

    let orient (n : Vec3) = if Vec3.dot n normalHint < 0.0 then -n else n
    let fit0 = Geometry.fitPlane planePts
    let mutable planeN = orient fit0.Normal
    let mutable planeO = fit0.Centroid

    // Position of a canonical id (all copies are identical by construction).
    let inline posOf (cid : int) = m.Vertices[adj.CopyItems[adj.CopyStart[cid]]]

    // Signed distance per canonical id, memoized. Rebuilt after the refit.
    let dist = Array.create adj.CanonicalCount nan
    let inline distOf cid =
        if Double.IsNaN dist[cid] then
            dist[cid] <- Vec3.dot (posOf cid - planeO) planeN
        dist[cid]

    /// Snap weight from the vertex's own distance — the feathering that makes
    /// curves stay smooth AND box edges stay sharp (see module doc).
    let weight (ad : float) =
        if ad <= floor then 1.0
        elif ad >= ceiling then 0.0
        else
            let t = (ad - floor) / (ceiling - floor)
            1.0 - t * t * (3.0 - 2.0 * t)

    // ---- flood fill, face by face
    let nf = m.TriangleCount
    let captured = Array.zeroCreate<bool> nf
    let admissible f =
        let n = Mesh.faceNormal m f
        // abs(): winding is NOT trustworthy on scans — the synthetic's box
        // walls are wound inward, and real scans mix orientations freely —
        // so the gate is orientation-agnostic, same as the renderer's
        // headlight shading. The cost: the backside of a plate THINNER than
        // the ceiling would be captured along with its front. No part this
        // tool exists for has sub-0.3 mm walls.
        if abs (Vec3.dot n planeN) < cosGate then false
        else
            let mutable near = infinity
            for e in 0 .. 2 do
                near <- min near (abs (distOf adj.Canonical[m.Indices[f * 3 + e]]))
            near <= ceiling
    let queue = Queue<int>()
    let mutable islands = 0
    for seed in seedFaces do
        if seed >= 0 && seed < nf && not captured[seed] then
            // The seed joins unconditionally: the user clicked ON this face,
            // and a flood that dies at its own seed is indistinguishable from
            // a broken button.
            islands <- islands + 1
            captured[seed] <- true
            queue.Enqueue seed
            while queue.Count > 0 do
                let f = queue.Dequeue()
                for e in 0 .. 2 do
                    let nb = adj.Neighbor[f * 3 + e]
                    if nb >= 0 && not captured[nb] && admissible nb then
                        captured[nb] <- true
                        queue.Enqueue nb

    // ---- canonical verts of the captured region
    let inRegion = Array.zeroCreate<bool> adj.CanonicalCount
    let regionVerts = ResizeArray<int>()
    for f in 0 .. nf - 1 do
        if captured[f] then
            for e in 0 .. 2 do
                let c = adj.Canonical[m.Indices[f * 3 + e]]
                if not inRegion[c] then
                    inRegion[c] <- true
                    regionVerts.Add c

    // ---- refit the plane to the whole region (IRLS, 2 rounds).
    // The picks seeded it from 3-8 points; the flat itself has thousands.
    // Weights reuse the snap kernel, so an outlier vert influences the plane
    // exactly as much as it would be snapped — a vert we would not touch
    // does not get a vote.
    for _round in 1 .. 2 do
        let mutable wsum = 0.0
        let mutable mean = Vec3.zero
        for c in regionVerts do
            let w = weight (abs (distOf c))
            if w > 0.0 then
                wsum <- wsum + w
                mean <- mean + posOf c * w
        if wsum > 1e-9 then
            let mean = mean / wsum
            let mutable xx, xy, xz, yy, yz, zz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
            for c in regionVerts do
                let w = weight (abs (distOf c))
                if w > 0.0 then
                    let d = posOf c - mean
                    xx <- xx + w * d.X * d.X
                    xy <- xy + w * d.X * d.Y
                    xz <- xz + w * d.X * d.Z
                    yy <- yy + w * d.Y * d.Y
                    yz <- yz + w * d.Y * d.Z
                    zz <- zz + w * d.Z * d.Z
            let cov =
                Mat3.ofRows (Vec3.create (xx / wsum) (xy / wsum) (xz / wsum))
                            (Vec3.create (xy / wsum) (yy / wsum) (yz / wsum))
                            (Vec3.create (xz / wsum) (yz / wsum) (zz / wsum))
            let eigen = Geometry.eigenSymmetric3 cov
            planeN <- orient (snd eigen[0])
            planeO <- mean
            Array.fill dist 0 dist.Length nan       // distances are stale

    // ---- moves and statistics
    let moves = ResizeArray<struct (int * Vec3)>()
    let mutable movedCanon = 0
    let mutable maxMove = 0.0
    let mutable sumSqBefore = 0.0
    let mutable sumSqAfter = 0.0
    let mutable measured = 0
    for c in regionVerts do
        let d = distOf c
        let ad = abs d
        if ad <= ceiling then
            let w = weight ad
            sumSqBefore <- sumSqBefore + d * d
            let residual = d * (1.0 - w)
            sumSqAfter <- sumSqAfter + residual * residual
            measured <- measured + 1
            if w > 1e-9 then
                movedCanon <- movedCanon + 1
                maxMove <- max maxMove (w * ad)
                let np = posOf c - planeN * (w * d)
                for k in adj.CopyStart[c] .. adj.CopyStart[c + 1] - 1 do
                    moves.Add(struct (adj.CopyItems[k], np))

    // ---- enclaves: connected components of NON-captured faces.
    // On a closed surface there is no "outside" — every complement component
    // is bounded by the region — so the rule is: the largest component (by
    // area) that touches the region is the rest of the model; every other
    // touching component is an enclave. Components that never touch the
    // region (disconnected debris shells) are nobody's business here.
    let enclaveFaces = ResizeArray<int>()
    let mutable enclaveCount = 0
    if regionVerts.Count > 0 then
        let visited = Array.zeroCreate<bool> nf
        let comps = ResizeArray<int[] * float * bool>()   // faces, area, touches
        let stack = Stack<int>()
        for f0 in 0 .. nf - 1 do
            if not captured[f0] && not visited[f0] then
                let faces = ResizeArray<int>()
                let mutable area = 0.0
                let mutable touches = false
                visited[f0] <- true
                stack.Push f0
                while stack.Count > 0 do
                    let f = stack.Pop()
                    faces.Add f
                    area <- area + 0.5 * Vec3.length (Mesh.crossArea m f)
                    for e in 0 .. 2 do
                        let nb = adj.Neighbor[f * 3 + e]
                        if nb >= 0 then
                            if captured[nb] then touches <- true
                            elif not visited[nb] then
                                visited[nb] <- true
                                stack.Push nb
                comps.Add(faces.ToArray(), area, touches)
        let touching = comps |> Seq.filter (fun (_, _, t) -> t) |> Seq.toArray
        if touching.Length > 1 then
            let largest = touching |> Array.maxBy (fun (_, a, _) -> a)
            for (faces, _, _) as comp in touching do
                if not (obj.ReferenceEquals(comp, largest)) then
                    enclaveCount <- enclaveCount + 1
                    enclaveFaces.AddRange faces

    let capturedList = ResizeArray<int>()
    for f in 0 .. nf - 1 do
        if captured[f] then capturedList.Add f

    {   PlaneNormal = planeN
        PlaneOrigin = planeO
        CapturedFaces = capturedList.ToArray()
        EnclaveFaces = enclaveFaces.ToArray()
        EnclaveCount = enclaveCount
        Islands = islands
        Moves = moves.ToArray()
        MovedVertexCount = movedCanon
        RmsBeforeMm = if measured = 0 then 0.0 else sqrt (sumSqBefore / float measured)
        RmsAfterMm = if measured = 0 then 0.0 else sqrt (sumSqAfter / float measured)
        MaxMoveMm = maxMove }

/// Commit a preview: a new mesh with the moves applied, plus the inverse
/// (index, old position) list that undoes it. Region-sized, not mesh-sized,
/// so an undo stack of these stays cheap.
let apply (m : Mesh) (pv : Preview) : Mesh * (int * Vec3)[] =
    let undo = Array.zeroCreate pv.Moves.Length
    let verts = Array.copy m.Vertices
    for i in 0 .. pv.Moves.Length - 1 do
        let struct (idx, np) = pv.Moves[i]
        undo[i] <- (idx, verts[idx])
        verts[idx] <- np
    { m with Vertices = verts }, undo
