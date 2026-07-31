/// Fitting primitives: eigen-decomposition, plane fit, 2D line fit, PCA.
///
/// Ported from f2s's `core.fit_plane` / `core.auto_orient`, which lean on
/// numpy's SVD. There is no SVD here and none is needed: every fit in this
/// tool is the eigen-decomposition of a small symmetric covariance matrix, and
/// cyclic Jacobi does that in fifty lines with no dependency.
module MeshOrient.Core.Geometry

open System

/// Eigen-decomposition of a symmetric 3x3 matrix by cyclic Jacobi rotations.
///
/// Returns eigenvalues ASCENDING with their unit eigenvectors alongside, so
/// `fst result[0]` is the smallest variance (a plane's normal) and
/// `fst result[2]` the largest (a point cloud's long axis).
let eigenSymmetric3 (m : Mat3) : (float * Vec3)[] =
    let a = Array2D.zeroCreate<float> 3 3
    a[0, 0] <- m.R0.X; a[0, 1] <- m.R0.Y; a[0, 2] <- m.R0.Z
    a[1, 0] <- m.R1.X; a[1, 1] <- m.R1.Y; a[1, 2] <- m.R1.Z
    a[2, 0] <- m.R2.X; a[2, 1] <- m.R2.Y; a[2, 2] <- m.R2.Z
    let v = Array2D.init 3 3 (fun i j -> if i = j then 1.0 else 0.0)

    // Fifty sweeps is far more than the three or four cyclic Jacobi needs for
    // a 3x3; the loop exits as soon as the off-diagonal is negligible.
    let mutable sweep = 0
    let mutable converged = false
    while sweep < 50 && not converged do
        let off = abs a[0, 1] + abs a[0, 2] + abs a[1, 2]
        if off < 1e-300 then converged <- true
        else
            for struct (p, q) in [| struct (0, 1); struct (0, 2); struct (1, 2) |] do
                if abs a[p, q] > 1e-300 then
                    let theta = (a[q, q] - a[p, p]) / (2.0 * a[p, q])
                    let t =
                        let s = if theta >= 0.0 then 1.0 else -1.0
                        s / (abs theta + sqrt (theta * theta + 1.0))
                    let c = 1.0 / sqrt (t * t + 1.0)
                    let s = t * c
                    for k in 0 .. 2 do
                        let akp = a[k, p]
                        let akq = a[k, q]
                        a[k, p] <- c * akp - s * akq
                        a[k, q] <- s * akp + c * akq
                    for k in 0 .. 2 do
                        let apk = a[p, k]
                        let aqk = a[q, k]
                        a[p, k] <- c * apk - s * aqk
                        a[q, k] <- s * apk + c * aqk
                    for k in 0 .. 2 do
                        let vkp = v[k, p]
                        let vkq = v[k, q]
                        v[k, p] <- c * vkp - s * vkq
                        v[k, q] <- s * vkp + c * vkq
            sweep <- sweep + 1

    let pairs =
        [| for j in 0 .. 2 ->
            a[j, j], Vec3.normalize { X = v[0, j]; Y = v[1, j]; Z = v[2, j] } |]
    Array.sortInPlaceBy fst pairs
    pairs

/// Covariance of a set of points about their own mean, with optional weights.
let private covariance (points : Vec3[]) (weights : float[] option) =
    let w i = match weights with Some ws -> ws[i] | None -> 1.0
    let mutable wsum = 0.0
    let mutable mean = Vec3.zero
    for i in 0 .. points.Length - 1 do
        let wi = w i
        wsum <- wsum + wi
        mean <- mean + points[i] * wi
    if wsum <= 0.0 then Vec3.zero, Mat3.identity
    else
        let mean = mean / wsum
        let mutable xx, xy, xz, yy, yz, zz = 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        for i in 0 .. points.Length - 1 do
            let d = points[i] - mean
            let wi = w i
            xx <- xx + wi * d.X * d.X
            xy <- xy + wi * d.X * d.Y
            xz <- xz + wi * d.X * d.Z
            yy <- yy + wi * d.Y * d.Y
            yz <- yz + wi * d.Y * d.Z
            zz <- zz + wi * d.Z * d.Z
        let c =
            {   R0 = { X = xx / wsum; Y = xy / wsum; Z = xz / wsum }
                R1 = { X = xy / wsum; Y = yy / wsum; Z = yz / wsum }
                R2 = { X = xz / wsum; Y = yz / wsum; Z = zz / wsum } }
        mean, c

/// A least-squares plane through 3 or more points.
///
/// Returns the unit normal, the centroid, and the RMS distance of the points
/// from the fitted plane. That last number is the one that matters in use: it
/// says how coplanar the picks actually were, i.e. whether they were a fair
/// sample of one flat face or whether one of them missed.
type PlaneFit = { Normal : Vec3; Centroid : Vec3; Rms : float }

let fitPlane (points : Vec3[]) : PlaneFit =
    if points.Length < 3 then
        invalidArg "points" $"need at least 3 points to fit a plane, got {points.Length}"
    let centroid, cov = covariance points None
    let eigen = eigenSymmetric3 cov
    let normal = snd eigen[0]                        // smallest variance
    let mutable acc = 0.0
    for p in points do
        let d = Vec3.dot (p - centroid) normal
        acc <- acc + d * d
    { Normal = normal; Centroid = centroid; Rms = sqrt (acc / float points.Length) }

/// A least-squares (total, not vertical) line through 2 or more 2D points.
///
/// Total least squares, not y-on-x regression: the picks are points on a
/// surface, so error is perpendicular to the line, and a near-vertical face
/// must not blow the fit up.
type LineFit2 = { DirX : float; DirY : float; CentroidX : float; CentroidY : float; Rms : float }

let fitLine2 (points : (float * float)[]) : LineFit2 =
    if points.Length < 2 then
        invalidArg "points" $"need at least 2 points to fit a line, got {points.Length}"
    let n = float points.Length
    let mx = points |> Array.sumBy fst |> fun s -> s / n
    let my = points |> Array.sumBy snd |> fun s -> s / n
    let mutable sxx, sxy, syy = 0.0, 0.0, 0.0
    for (x, y) in points do
        let dx, dy = x - mx, y - my
        sxx <- sxx + dx * dx
        sxy <- sxy + dx * dy
        syy <- syy + dy * dy
    // Largest eigenvector of the symmetric 2x2 [[sxx, sxy], [sxy, syy]],
    // closed form. Choose whichever of the two candidate eigenvectors has the
    // larger norm: the other degenerates when the line runs along an axis.
    let tr = sxx + syy
    let det = sxx * syy - sxy * sxy
    let disc = sqrt (max 0.0 (tr * tr / 4.0 - det))
    let lam = tr / 2.0 + disc
    let c1 = (sxy, lam - sxx)
    let c2 = (lam - syy, sxy)
    let norm (a, b) = sqrt (a * a + b * b)
    let dx, dy =
        if norm c1 >= norm c2 then c1 else c2
        |> fun (a, b) ->
            let l = norm (a, b)
            if l < 1e-300 then 1.0, 0.0 else a / l, b / l
    // Perpendicular residual: the component of each offset across the line.
    let mutable acc = 0.0
    for (x, y) in points do
        let perp = -(x - mx) * dy + (y - my) * dx
        acc <- acc + perp * perp
    { DirX = dx; DirY = dy; CentroidX = mx; CentroidY = my; Rms = sqrt (acc / n) }

/// PCA orientation guess: longest principal axis -> X, thinnest -> Y,
/// middle -> Z. Same convention as f2s's `core.auto_orient`; 90-degree
/// ambiguities are expected and the UI's rotate/flip buttons resolve them.
///
/// The covariance comes from AREA-WEIGHTED TRIANGLE CENTROIDS, not from the
/// raw vertex array as f2s uses. Vertex PCA measures the tessellation, not the
/// shape: a finely remeshed patch contributes hundreds of times more points
/// than a flat wall of the same size and drags the axes toward itself.
/// Weighting each triangle's centroid by its area measures the surface, which
/// is what "which way is this thing pointing" actually means. On an evenly
/// tessellated scan the two agree closely (the tests pin that), and where they
/// disagree this one is right.
let autoOrient (m : Mesh) : Mat3 =
    let n = m.TriangleCount
    if n = 0 then Mat3.identity
    else
        let centroids = Array.zeroCreate<Vec3> n
        let areas = Array.zeroCreate<float> n
        System.Threading.Tasks.Parallel.For(0, n, fun t ->
            let struct (a, b, c) = Mesh.triangle m t
            centroids[t] <- (a + b + c) / 3.0
            areas[t] <- 0.5 * Vec3.length (Vec3.cross (b - a) (c - a))) |> ignore
        let _, cov = covariance centroids (Some areas)
        let e = eigenSymmetric3 cov
        let eSmall, eMid, eLarge = snd e[0], snd e[1], snd e[2]
        let r = Mat3.ofRows eLarge eSmall eMid
        // Keep it a rotation, not a reflection — a mirrored scan would flip
        // the triangle winding and export inside-out.
        if Mat3.det r < 0.0 then { r with R1 = -r.R1 } else r
