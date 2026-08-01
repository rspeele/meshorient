namespace MeshOrient.Core.Tests

open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.App

[<TestClass>]
type ParamStepTests () =

    /// The user's worked example, run through the SAME format+parse pipeline
    /// the textbox uses — the box is the source of truth, so float dust like
    /// 0.10999999999999999 must wash out in formatting or the ladder drifts.
    [<TestMethod>]
    member _.Worked_example_down_and_back_up () =
        let step isUp t = (ParamStep.stepText isUp t).Value
        let mutable t = "0.21"
        let downs = [ for _ in 1 .. 3 -> t <- step false t; t ]
        Assert.AreEqual<string list>([ "0.11"; "0.055"; "0.0275" ], downs)
        let ups = [ for _ in 1 .. 3 -> t <- step true t; t ]
        Assert.AreEqual<string list>([ "0.055"; "0.11"; "0.21" ], ups)

    /// Up and down are exact inverses at EVERY value, not just on the worked
    /// ladder. This is the property that makes arrow-tweaking feel safe: you
    /// can always get back to where you were.
    [<TestMethod>]
    member _.Up_and_down_are_inverses_everywhere () =
        let values =
            [ 0.0125; 0.02; 0.0275; 0.05; 0.055; 0.08; 0.09; 0.1; 0.11
              0.15; 0.16; 0.2; 0.21; 0.3; 0.55; 1.0; 2.5 ]
        for v in values do
            Assert.AreEqual(v, ParamStep.up (ParamStep.down v), 1e-12, $"up(down {v})")
            Assert.AreEqual(v, ParamStep.down (ParamStep.up v), 1e-12, $"down(up {v})")

    [<TestMethod>]
    member _.Transition_is_where_the_destination_would_go_under () =
        // 0.21 - 0.1 = 0.11 lands ABOVE the transition: linear.
        Assert.AreEqual(0.11, ParamStep.down 0.21, 1e-12)
        // 0.11 - 0.1 = 0.01 would land under it: halve instead.
        Assert.AreEqual(0.055, ParamStep.down 0.11, 1e-12)
        // And exactly at the boundary: 0.2 -> 0.1 linear, 0.1 -> 0.05 halves.
        Assert.AreEqual(0.1, ParamStep.down 0.2, 1e-12)
        Assert.AreEqual(0.05, ParamStep.down 0.1, 1e-12)

    [<TestMethod>]
    member _.Typed_zero_can_escape_upward () =
        // Halving can never reach zero, but a user can type it; up must not
        // be stuck at 0 * 2 = 0.
        Assert.AreEqual(ParamStep.transition, ParamStep.up 0.0, 1e-12)

    [<TestMethod>]
    member _.Junk_text_steps_nowhere () =
        Assert.IsTrue((ParamStep.stepText true "not a number").IsNone)
        Assert.IsTrue((ParamStep.stepText false "").IsNone)

    [<TestMethod>]
    member _.Negative_input_clamps_to_zero () =
        Assert.AreEqual(Some 0.0, ParamStep.parse "-5")
