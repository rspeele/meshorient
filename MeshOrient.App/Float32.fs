/// Conversions between Core's double-precision `Vec3` and System.Numerics'
/// float32 `Vector3`. Rendering is the only place float32 is wanted, so these
/// run on the way to the GPU and on the way back from a cursor ray.
module MeshOrient.App.Float32

open System.Numerics
open MeshOrient.Core

let ofVec3 (v : Vec3) = Vector3(float32 v.X, float32 v.Y, float32 v.Z)

let toVec3 (v : Vector3) = Vec3.create (float v.X) (float v.Y) (float v.Z)
