/// GL resources and drawing: shaders, buffers, primitives, one draw per panel.
module MeshOrient.App.Scene

open System
open System.Numerics
open System.Runtime.InteropServices
open Silk.NET.OpenGL
open MeshOrient.Core

// ------------------------------------------------------------ vertex formats

[<Struct; StructLayout(LayoutKind.Sequential)>]
type LitVertex =
    val Px : float32
    val Py : float32
    val Pz : float32
    val Nx : float32
    val Ny : float32
    val Nz : float32
    new(p : Vector3, n : Vector3) =
        { Px = p.X; Py = p.Y; Pz = p.Z; Nx = n.X; Ny = n.Y; Nz = n.Z }

[<Struct; StructLayout(LayoutKind.Sequential)>]
type LineVertex =
    val Px : float32
    val Py : float32
    val Pz : float32
    val R : float32
    val G : float32
    val B : float32
    new(p : Vector3, c : Vector3) =
        { Px = p.X; Py = p.Y; Pz = p.Z; R = c.X; G = c.Y; B = c.Z }

let private litSize = Marshal.SizeOf<LitVertex>()
let private lineSize = Marshal.SizeOf<LineVertex>()

// ------------------------------------------------------------------ shaders

// GLSL ES 3.00, matching the CNCFlow renderer: it compiles on every backend
// Avalonia might pick (ANGLE/D3D11 on Windows, Metal on macOS, Mesa on Linux)
// and desktop GL drivers accept it via GL_ARB_ES3_compatibility.

let private litVs = """#version 300 es
precision highp float;
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aNormal;
uniform mat4 uMVP;
uniform mat4 uMV;
out vec3 vNormalView;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vNormalView = mat3(uMV) * aNormal;
}
"""

let private litFs = """#version 300 es
precision highp float;
in vec3 vNormalView;
uniform vec3 uColor;
out vec4 fragColor;
void main() {
    // Headlight: the light sits at the camera, so shading is just how far the
    // surface has turned away from us. abs() rather than max(0) because scan
    // meshes have inconsistent winding and a back-facing triangle should read
    // as surface, not as a black hole.
    vec3 n = normalize(vNormalView);
    float lambert = abs(n.z);
    // A little extra contrast at grazing angles so edges and steps read.
    float shade = 0.22 + 0.78 * pow(lambert, 0.8);
    fragColor = vec4(uColor * shade, 1.0);
}
"""

let private lineVs = """#version 300 es
precision highp float;
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor;
uniform mat4 uMVP;
out vec3 vColor;
void main() {
    gl_Position = uMVP * vec4(aPos, 1.0);
    vColor = aColor;
}
"""

let private lineFs = """#version 300 es
precision highp float;
in vec3 vColor;
out vec4 fragColor;
void main() { fragColor = vec4(vColor, 1.0); }
"""

let private compileShader (gl : GL) (ty : ShaderType) (src : string) =
    let h = gl.CreateShader ty
    gl.ShaderSource(h, src)
    gl.CompileShader h
    let mutable ok = 0
    gl.GetShader(h, ShaderParameterName.CompileStatus, &ok)
    if ok = 0 then failwithf "%A shader failed to compile: %s" ty (gl.GetShaderInfoLog h)
    h

let private compileProgram (gl : GL) (vsSrc : string) (fsSrc : string) =
    let vs = compileShader gl ShaderType.VertexShader vsSrc
    let fs = compileShader gl ShaderType.FragmentShader fsSrc
    let p = gl.CreateProgram()
    gl.AttachShader(p, vs)
    gl.AttachShader(p, fs)
    gl.LinkProgram p
    let mutable ok = 0
    gl.GetProgram(p, ProgramPropertyARB.LinkStatus, &ok)
    if ok = 0 then failwithf "Program link failed: %s" (gl.GetProgramInfoLog p)
    gl.DetachShader(p, vs); gl.DetachShader(p, fs)
    gl.DeleteShader vs; gl.DeleteShader fs
    p

type Programs =
    {   Lit : uint32
        LitMvp : int
        LitMv : int
        LitColor : int
        Line : uint32
        LineMvp : int }

let initPrograms (gl : GL) =
    let lit = compileProgram gl litVs litFs
    let line = compileProgram gl lineVs lineFs
    {   Lit = lit
        LitMvp = gl.GetUniformLocation(lit, "uMVP")
        LitMv = gl.GetUniformLocation(lit, "uMV")
        LitColor = gl.GetUniformLocation(lit, "uColor")
        Line = line
        LineMvp = gl.GetUniformLocation(line, "uMVP") }

let disposePrograms (gl : GL) (p : Programs) =
    gl.DeleteProgram p.Lit
    gl.DeleteProgram p.Line

// ------------------------------------------------------------------ buffers

type GpuBuffer =
    {   Vao : uint32
        Vbo : uint32
        VertexCount : int }

let empty = { Vao = 0u; Vbo = 0u; VertexCount = 0 }

let private makeBuffer (gl : GL) (bytes : int) (data : 'T[]) =
    if data.Length = 0 then empty
    else
        let vao = gl.GenVertexArray()
        gl.BindVertexArray vao
        let vbo = gl.GenBuffer()
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, vbo)
        gl.BufferData(BufferTargetARB.ArrayBuffer, ReadOnlySpan<'T>(data), BufferUsageARB.StaticDraw)
        let stride = uint32 bytes
        gl.VertexAttribPointer(0u, 3, VertexAttribPointerType.Float, false, stride, 0)
        gl.EnableVertexAttribArray 0u
        gl.VertexAttribPointer(1u, 3, VertexAttribPointerType.Float, false, stride, 12)
        gl.EnableVertexAttribArray 1u
        gl.BindBuffer(BufferTargetARB.ArrayBuffer, 0u)
        gl.BindVertexArray 0u
        { Vao = vao; Vbo = vbo; VertexCount = data.Length }

let makeLitBuffer (gl : GL) (verts : LitVertex[]) = makeBuffer gl litSize verts
let makeLineBuffer (gl : GL) (verts : LineVertex[]) = makeBuffer gl lineSize verts

let disposeBuffer (gl : GL) (b : GpuBuffer) =
    if b.VertexCount > 0 then
        gl.DeleteBuffer b.Vbo
        gl.DeleteVertexArray b.Vao

/// Flat-shaded triangle soup for the whole mesh, in MODEL space.
///
/// Uploaded once per file load and never again: re-orienting the scan changes
/// a uniform matrix, not 50 MB of vertex data. That is what keeps the rotate
/// buttons instant on a 760k-triangle scan.
let meshVertices (m : Mesh) : LitVertex[] =
    let n = m.TriangleCount
    let out = Array.zeroCreate<LitVertex> (n * 3)
    System.Threading.Tasks.Parallel.For(0, n, fun t ->
        let struct (a, b, c) = Mesh.triangle m t
        let nr = Mesh.faceNormal m t
        let v3 (p : Vec3) = Vector3(float32 p.X, float32 p.Y, float32 p.Z)
        let nv = v3 nr
        out[t * 3] <- LitVertex(v3 a, nv)
        out[t * 3 + 1] <- LitVertex(v3 b, nv)
        out[t * 3 + 2] <- LitVertex(v3 c, nv)) |> ignore
    out

/// A unit-radius UV sphere for pick markers. Coarse on purpose — it is a 12
/// pixel dot on screen and nobody is inspecting its silhouette.
let sphereVertices () : LitVertex[] =
    let slices, stacks = 16, 10
    let verts = ResizeArray<LitVertex>()
    let at (i : int) (j : int) =
        let phi = MathF.PI * float32 j / float32 stacks
        let theta = 2.0f * MathF.PI * float32 i / float32 slices
        let n = Vector3(MathF.Sin phi * MathF.Cos theta, MathF.Sin phi * MathF.Sin theta, MathF.Cos phi)
        LitVertex(n, n)
    for j in 0 .. stacks - 1 do
        for i in 0 .. slices - 1 do
            let a, b, c, d = at i j, at (i + 1) j, at (i + 1) (j + 1), at i (j + 1)
            verts.Add a; verts.Add b; verts.Add c
            verts.Add a; verts.Add c; verts.Add d
    verts.ToArray()

// ------------------------------------------------------------------ drawing

let private setMatrix (gl : GL) (loc : int) (m : Matrix4x4) =
    // System.Numerics is row-major with row-vector convention; GLSL is
    // column-major with column vectors. The two layouts are byte-identical for
    // the same transform, so transpose stays false and the multiply order
    // (model * view * proj) reads left to right.
    let mutable mm = m
    gl.UniformMatrix4(loc, 1u, false, &mm.M11)

let drawLit (gl : GL) (p : Programs) (buf : GpuBuffer) (model : Matrix4x4)
            (view : Matrix4x4) (proj : Matrix4x4) (colour : Vector3) =
    if buf.VertexCount > 0 then
        gl.UseProgram p.Lit
        setMatrix gl p.LitMvp (model * view * proj)
        setMatrix gl p.LitMv (model * view)
        gl.Uniform3(p.LitColor, colour.X, colour.Y, colour.Z)
        gl.BindVertexArray buf.Vao
        gl.DrawArrays(PrimitiveType.Triangles, 0, uint32 buf.VertexCount)
        gl.BindVertexArray 0u

let drawLines (gl : GL) (p : Programs) (buf : GpuBuffer) (mvp : Matrix4x4) =
    if buf.VertexCount > 0 then
        gl.UseProgram p.Line
        setMatrix gl p.LineMvp mvp
        gl.BindVertexArray buf.Vao
        gl.DrawArrays(PrimitiveType.Lines, 0, uint32 buf.VertexCount)
        gl.BindVertexArray 0u
