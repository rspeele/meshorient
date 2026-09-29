namespace MeshOrient.Core.Tests

open System
open System.Numerics
open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.App.Camera

[<TestClass>]
type CameraTests () =

    /// Where a pan moves the orbit target, in the view space the drag
    /// started in.
    let panInView (orbit : Orbit) (dx : float32) (dy : float32) =
        let view, _ = orbitMatrices orbit 1.0f
        let panned = pan dx dy 500.0f orbit
        Vector3.Transform(panned.Target, view) - Vector3.Transform(orbit.Target, view)

    /// Panning drags the model with the cursor at EVERY pitch, including the
    /// numpad-7 and numpad-9 presets that sit exactly on the poles.
    [<TestMethod>]
    member _.Pan_follows_the_cursor_at_every_pitch_including_the_poles () =
        let pitches = [ -MathF.PI / 2.0f; -1.2f; -0.3f; 0.0f; 0.3f; 1.2f; MathF.PI / 2.0f ]
        let reference = panInView { defaultOrbit with Pitch = 0.3f } 10.0f 0.0f
        for pitch in pitches do
            for yaw in [ -MathF.PI / 2.0f; 0.4f; 2.0f ] do
                let orbit = { defaultOrbit with Yaw = yaw; Pitch = pitch; Distance = 100.0f }
                let right = panInView orbit 10.0f 0.0f
                let down = panInView orbit 0.0f 10.0f
                printfn "pitch %.3f yaw %.2f: drag right -> %A, drag down -> %A" pitch yaw right down
                // Drag right: the target moves LEFT in view, i.e. the model
                // follows the cursor right. Drag down: target moves up.
                Assert.IsTrue(right.X < 0.0f && abs right.Y < 1e-4f,
                              $"drag right at pitch {pitch}, yaw {yaw}: {right}")
                Assert.IsTrue(down.Y > 0.0f && abs down.X < 1e-4f,
                              $"drag down at pitch {pitch}, yaw {yaw}: {down}")
                let expected = reference.X * orbit.Distance / defaultOrbit.Distance
                Assert.AreEqual(expected, right.X, abs expected * 1e-3f, "same speed at every pitch")
