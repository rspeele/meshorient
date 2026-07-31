/// The alignment stages, as pure functions over picked points.
///
/// Three stages, applied in order, each narrowing the remaining freedom:
///
///   1. Coarse   — `Geometry.autoOrient` (PCA) plus the UI's 90-degree
///                 rotate / flip / re-centre buttons.
///   2. Square   — pick 3+ points on a face that really is flat; the smallest
///                 rotation that squares that face to an axis (Y by default).
///   3. Straighten — pick 2+ points on a top or bottom reference; a rotation
///                 ABOUT the axis stage 2 just locked, which is the only
///                 rotation that leaves stage 2 intact.
///
/// Stage 3's constraint is the whole reason it works. Once a side face is
/// square to Y, rotating about X or Z tilts it straight back out of square —
/// rotating about Y is the sole remaining degree of freedom, and it happens to
/// be exactly the one that swings the bore up and down in the side view. So
/// "get the muzzle pointing down X without disturbing the levelling" and
/// "rotate about Y" are the same instruction.
module MeshOrient.Core.Orient

open System

let private toDegrees r = r * 180.0 / Math.PI

/// The offset that puts the bounding-box centre at the origin.
/// f2s's `core.center_verts`, as a translation rather than a new array.
let recentreOffset (m : Mesh) : Vec3 =
    let b = Mesh.bounds m
    if b.IsEmpty then Vec3.zero else -(Bounds.centre b)

// ------------------------------------------------------------ stage 2: square

type SquareResult =
    {   /// Apply this to the CURRENT model to square the picked face.
        Rotation : Mat3
        /// How far out of square the picked face was.
        TiltDegrees : float
        /// RMS distance of the picks from their own best-fit plane. This is
        /// your scan's flatness plus your aim — a large number means one pick
        /// was not on the flat, and the fit is not to be trusted.
        RmsMm : float }

/// Smallest rotation squaring the plane through `picks` to world `axis`
/// (0 = X, 1 = Y, 2 = Z; Y is the frame's flat side).
///
/// Rodrigues about `normal x target`, so it removes tilt without spinning the
/// model about the target axis — whatever stage 1 established stays put, and
/// stage 3 is left with a clean single degree of freedom. Straight port of
/// f2s's `core.level_rotation`.
let squareToAxis (axis : int) (picks : Vec3[]) : SquareResult =
    let fit = Geometry.fitPlane picks
    let target = Vec3.axis axis
    let n = if Vec3.dot fit.Normal target < 0.0 then -fit.Normal else fit.Normal
    let v = Vec3.cross n target
    let s = Vec3.length v
    if s < 1e-12 then
        // Already square (or exactly inverted, which the flip above ruled out).
        { Rotation = Mat3.identity; TiltDegrees = 0.0; RmsMm = fit.Rms }
    else
        let angle = atan2 s (Vec3.dot n target)
        { Rotation = Mat3.aboutAxis v angle
          TiltDegrees = toDegrees angle
          RmsMm = fit.Rms }

/// The two visible components of a tilt, in degrees, for the ortho-view
/// titles: rotation about X leans the model in the back (Y-Z) view, rotation
/// about Z leans it in the top (X-Y) view. Same decomposition f2s draws in
/// `App._draw_level_aux`. Only meaningful for a Y-axis square.
let tiltComponents (picks : Vec3[]) : float * float =
    let fit = Geometry.fitPlane picks
    let n = if fit.Normal.Y < 0.0 then -fit.Normal else fit.Normal
    toDegrees (atan2 n.Z n.Y), toDegrees (atan2 n.X n.Y)

// -------------------------------------------------------- stage 3: straighten

type StraightenResult =
    {   Rotation : Mat3
        /// How far the reference face was off horizontal, in the view that
        /// rotation about the locked axis spins.
        AngleDegrees : float
        /// RMS distance of the picks from the fitted line, measured in the
        /// projected plane. The straightening analogue of `SquareResult.RmsMm`.
        RmsMm : float }

/// Rotate ABOUT `lockedAxis` so the face through `picks` comes level.
///
/// `lockedAxis` is whatever stage 2 squared to (Y for a pistol frame). The
/// picks are projected onto the plane perpendicular to it — for Y that is the
/// X-Z side view — and a total-least-squares line is fitted through them. Two
/// picks give an exact line; more average out the error in each pick, which is
/// the point of picking several.
///
/// Dropping the locked-axis coordinate is not a loss: once that axis is
/// square, spread along it carries no information about this rotation. Points
/// across the width of a slide top collapse onto the same place on the line
/// and contribute nothing, which is correct.
let straightenAbout (lockedAxis : int) (picks : Vec3[]) : StraightenResult =
    if picks.Length < 2 then
        invalidArg "picks" $"need at least 2 points to straighten, got {picks.Length}"
    // Rotation about k carries axis a toward axis b, where (a, b, k) is cyclic.
    // For k = Y that is (Z, X, Y): Ry swings Z toward X.
    let a = (lockedAxis + 1) % 3
    let b = (lockedAxis + 2) % 3
    let fit = picks |> Array.map (fun p -> Vec3.item b p, Vec3.item a p) |> Geometry.fitLine2
    // A fitted direction has an arbitrary sign. Take the one pointing along +b
    // so we apply the SMALL correction rather than spinning the model
    // end-for-end — stage 1 already got it the right way round.
    let db, da = if fit.DirX < 0.0 then -fit.DirX, -fit.DirY else fit.DirX, fit.DirY
    let angle = atan2 da db
    { Rotation = Mat3.rotDegrees lockedAxis (toDegrees angle)
      AngleDegrees = toDegrees angle
      RmsMm = fit.Rms }
