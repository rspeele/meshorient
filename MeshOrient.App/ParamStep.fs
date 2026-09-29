/// Arrow-key stepping for the millimetre parameter boxes.
///
/// Linear above the transition, geometric below it: ±0.1 mm per press down to
/// 0.1, then halving/doubling. A flat ±0.1 step is right for capture-ceiling
/// values (0.3-ish) but useless for a noise floor of 0.07 — one press hits
/// zero — while a purely geometric step crawls at large values. The switch
/// happens when the DESTINATION would land under the transition, so the
/// worked ladder is  0.21 -> 0.11 -> 0.055 -> 0.0275  and exactly back up.
///
/// Up and down are exact inverses at EVERY value, not just on that ladder
/// (0.16 -> 0.08 -> 0.16 works too). The tests pin the invariant, and run the
/// worked ladder through the format+parse round trip, because the textbox is
/// the source of truth: each press parses the text, steps, and reformats.
module MeshOrient.App.ParamStep

open System
open System.Globalization

let transition = 0.1

let up (v : float) : float =
    if v <= 0.0 then transition          // typed-zero escape hatch
    elif v < transition then v * 2.0
    else v + transition

let down (v : float) : float =
    if v - transition >= transition then v - transition
    else v / 2.0

/// Invariant-culture, round-trip-stable formatting. Four significant figures
/// carries the halving ladder (0.0275, 0.01375, ...) without the float dust
/// a raw ToString would print (0.21 - 0.1 is 0.10999999999999999).
let format (v : float) : string = sprintf "%.4g" v

let parse (text : string) : float option =
    match Double.TryParse(text, NumberStyles.Float, CultureInfo.InvariantCulture) with
    | true, v when not (Double.IsNaN v || Double.IsInfinity v) -> Some(max 0.0 v)
    | _ -> None

/// One arrow press against the box's current text. None if the text is not a
/// number — the press does nothing rather than guessing.
let stepText (isUp : bool) (text : string) : string option =
    parse text |> Option.map (fun v -> format (if isUp then up v else down v))
