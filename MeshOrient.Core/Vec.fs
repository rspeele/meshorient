namespace MeshOrient.Core

open System
open System.Runtime.CompilerServices

/// A 3-vector in double precision.
///
/// Deliberately NOT System.Numerics.Vector3: that is float32, and this type
/// carries the *measurement* side of the tool. Accumulating a covariance over
/// a few million triangle centroids in float32 loses the very fractions of a
/// degree the alignment steps exist to remove. Rendering converts down to float32
/// at the last moment; nothing upstream of the GPU does.
[<Struct; IsReadOnly>]
type Vec3 =
    { X : float; Y : float; Z : float }

    static member inline (+) (a : Vec3, b : Vec3) = { X = a.X + b.X; Y = a.Y + b.Y; Z = a.Z + b.Z }
    static member inline (-) (a : Vec3, b : Vec3) = { X = a.X - b.X; Y = a.Y - b.Y; Z = a.Z - b.Z }
    static member inline (*) (a : Vec3, s : float) = { X = a.X * s; Y = a.Y * s; Z = a.Z * s }
    static member inline (*) (s : float, a : Vec3) = { X = a.X * s; Y = a.Y * s; Z = a.Z * s }
    static member inline (/) (a : Vec3, s : float) = { X = a.X / s; Y = a.Y / s; Z = a.Z / s }
    static member inline (~-) (a : Vec3) = { X = -a.X; Y = -a.Y; Z = -a.Z }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module Vec3 =

    let zero = { X = 0.0; Y = 0.0; Z = 0.0 }
    let unitX = { X = 1.0; Y = 0.0; Z = 0.0 }
    let unitY = { X = 0.0; Y = 1.0; Z = 0.0 }
    let unitZ = { X = 0.0; Y = 0.0; Z = 1.0 }

    let create x y z = { X = x; Y = y; Z = z }

    let inline dot (a : Vec3) (b : Vec3) = a.X * b.X + a.Y * b.Y + a.Z * b.Z

    let inline cross (a : Vec3) (b : Vec3) =
        {   X = a.Y * b.Z - a.Z * b.Y
            Y = a.Z * b.X - a.X * b.Z
            Z = a.X * b.Y - a.Y * b.X }

    let inline lengthSq (a : Vec3) = dot a a
    let inline length (a : Vec3) = sqrt (dot a a)

    /// Unit vector, or `zero` for a vector too short to have a direction.
    /// Callers that care about the degenerate case must check for themselves —
    /// silently returning zero is safer here than a NaN that propagates into a
    /// rotation matrix and quietly corrupts a scan.
    let normalize (a : Vec3) =
        let n = length a
        if n < 1e-300 then zero else a / n

    /// Component `i` (0 = X, 1 = Y, 2 = Z). Lets the alignment moves take an
    /// axis index instead of hard-coding one.
    let item (i : int) (a : Vec3) =
        match i with
        | 0 -> a.X
        | 1 -> a.Y
        | 2 -> a.Z
        | _ -> invalidArg "i" $"axis index must be 0, 1 or 2, not {i}"

    /// The unit vector along axis `i`.
    let axis (i : int) =
        match i with
        | 0 -> unitX
        | 1 -> unitY
        | 2 -> unitZ
        | _ -> invalidArg "i" $"axis index must be 0, 1 or 2, not {i}"

    let minOf (a : Vec3) (b : Vec3) =
        { X = min a.X b.X; Y = min a.Y b.Y; Z = min a.Z b.Z }

    let maxOf (a : Vec3) (b : Vec3) =
        { X = max a.X b.X; Y = max a.Y b.Y; Z = max a.Z b.Z }


/// A 3x3 matrix stored as three ROW vectors.
///
/// Applied to column vectors: `apply m v = m * v`, so `mul r2 r1` applies
/// r1 first.
[<Struct; IsReadOnly>]
type Mat3 =
    { R0 : Vec3; R1 : Vec3; R2 : Vec3 }

[<CompilationRepresentation(CompilationRepresentationFlags.ModuleSuffix)>]
module Mat3 =

    let identity =
        { R0 = Vec3.unitX; R1 = Vec3.unitY; R2 = Vec3.unitZ }

    /// Build from three rows.
    let ofRows r0 r1 r2 = { R0 = r0; R1 = r1; R2 = r2 }

    /// Build from three columns — the natural way to assemble a rotation from
    /// a set of basis vectors you want mapped ONTO the world axes.
    let ofCols (c0 : Vec3) (c1 : Vec3) (c2 : Vec3) =
        {   R0 = { X = c0.X; Y = c1.X; Z = c2.X }
            R1 = { X = c0.Y; Y = c1.Y; Z = c2.Y }
            R2 = { X = c0.Z; Y = c1.Z; Z = c2.Z } }

    let inline apply (m : Mat3) (v : Vec3) =
        {   X = Vec3.dot m.R0 v
            Y = Vec3.dot m.R1 v
            Z = Vec3.dot m.R2 v }

    /// `mul a b` is the matrix product a·b, i.e. "apply b, then a".
    let mul (a : Mat3) (b : Mat3) =
        let row (r : Vec3) = r.X * b.R0 + r.Y * b.R1 + r.Z * b.R2
        { R0 = row a.R0; R1 = row a.R1; R2 = row a.R2 }

    let transpose (m : Mat3) =
        {   R0 = { X = m.R0.X; Y = m.R1.X; Z = m.R2.X }
            R1 = { X = m.R0.Y; Y = m.R1.Y; Z = m.R2.Y }
            R2 = { X = m.R0.Z; Y = m.R1.Z; Z = m.R2.Z } }

    let det (m : Mat3) = Vec3.dot m.R0 (Vec3.cross m.R1 m.R2)

    /// Rotation about X by `radians`.
    let rotX (radians : float) =
        let c, s = cos radians, sin radians
        {   R0 = { X = 1.0; Y = 0.0; Z = 0.0 }
            R1 = { X = 0.0; Y = c;   Z = -s }
            R2 = { X = 0.0; Y = s;   Z = c } }

    let rotY (radians : float) =
        let c, s = cos radians, sin radians
        {   R0 = { X = c;   Y = 0.0; Z = s }
            R1 = { X = 0.0; Y = 1.0; Z = 0.0 }
            R2 = { X = -s;  Y = 0.0; Z = c } }

    let rotZ (radians : float) =
        let c, s = cos radians, sin radians
        {   R0 = { X = c;   Y = -s;  Z = 0.0 }
            R1 = { X = s;   Y = c;   Z = 0.0 }
            R2 = { X = 0.0; Y = 0.0; Z = 1.0 } }

    let rotDegrees (axis : int) (degrees : float) =
        let r = degrees * Math.PI / 180.0
        match axis with
        | 0 -> rotX r
        | 1 -> rotY r
        | 2 -> rotZ r
        | _ -> invalidArg "axis" $"axis index must be 0, 1 or 2, not {axis}"

    /// Rotation of `angle` radians about the unit vector `k` (Rodrigues):
    /// `I + sin(t) K + (1 - cos t) K^2`.
    let aboutAxis (k : Vec3) (angle : float) =
        let k = Vec3.normalize k
        if Vec3.lengthSq k < 0.5 then identity      // no axis => no rotation
        else
            let c, s = cos angle, sin angle
            let t = 1.0 - c
            {   R0 = { X = t * k.X * k.X + c;         Y = t * k.X * k.Y - s * k.Z; Z = t * k.X * k.Z + s * k.Y }
                R1 = { X = t * k.X * k.Y + s * k.Z;   Y = t * k.Y * k.Y + c;       Z = t * k.Y * k.Z - s * k.X }
                R2 = { X = t * k.X * k.Z - s * k.Y;   Y = t * k.Y * k.Z + s * k.X; Z = t * k.Z * k.Z + c } }
