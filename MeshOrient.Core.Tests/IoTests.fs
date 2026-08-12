namespace MeshOrient.Core.Tests

open System
open System.Buffers.Binary
open System.IO
open System.Text
open Microsoft.VisualStudio.TestTools.UnitTesting
open MeshOrient.Core

/// A 2 mm cube, written out by hand in each supported format. Hand-authoring
/// the bytes is the point: it exercises the parsers against files this code
/// did not produce, which a write-then-read round trip cannot do.
module private Cube =

    let corners =
        [|  0.0, 0.0, 0.0;  2.0, 0.0, 0.0;  2.0, 2.0, 0.0;  0.0, 2.0, 0.0
            0.0, 0.0, 2.0;  2.0, 0.0, 2.0;  2.0, 2.0, 2.0;  0.0, 2.0, 2.0 |]

    let tris =
        [|  0, 2, 1;  0, 3, 2                    // bottom
            4, 5, 6;  4, 6, 7                    // top
            0, 1, 5;  0, 5, 4                    // y = 0
            1, 2, 6;  1, 6, 5                    // x = 2
            2, 3, 7;  2, 7, 6                    // y = 2
            3, 0, 4;  3, 4, 7 |]                 // x = 0

    let obj () =
        let sb = StringBuilder()
        sb.AppendLine "# a 2mm cube" |> ignore
        for (x, y, z) in corners do
            sb.AppendLine $"v %g{x} %g{y} %g{z}" |> ignore
        for (a, b, c) in tris do
            // OBJ is 1-based, and the loader must cope with the v/vt/vn form.
            sb.AppendLine $"f %d{a + 1}//1 %d{b + 1}//1 %d{c + 1}//1" |> ignore
        sb.ToString()

    let plyAscii () =
        let sb = StringBuilder()
        sb.AppendLine "ply" |> ignore
        sb.AppendLine "format ascii 1.0" |> ignore
        sb.AppendLine $"element vertex %d{corners.Length}" |> ignore
        sb.AppendLine "property float x" |> ignore
        sb.AppendLine "property float y" |> ignore
        sb.AppendLine "property float z" |> ignore
        sb.AppendLine $"element face %d{tris.Length}" |> ignore
        sb.AppendLine "property list uchar int vertex_indices" |> ignore
        sb.AppendLine "end_header" |> ignore
        for (x, y, z) in corners do
            sb.AppendLine $"%g{x} %g{y} %g{z}" |> ignore
        for (a, b, c) in tris do
            sb.AppendLine $"3 %d{a} %d{b} %d{c}" |> ignore
        sb.ToString()

    /// Binary little-endian, with a `confidence` property after z so the
    /// reader has to honour the declared layout rather than assume x/y/z are
    /// the only three floats in the record.
    let plyBinaryLe () =
        let header =
            String.concat "\n"
                [   "ply"; "format binary_little_endian 1.0"
                    $"element vertex %d{corners.Length}"
                    "property float x"; "property float y"; "property float z"
                    "property float confidence"
                    $"element face %d{tris.Length}"
                    "property list uchar int vertex_indices"
                    "end_header"; "" ]
        use ms = new MemoryStream()
        let bytes = Encoding.ASCII.GetBytes header
        ms.Write(bytes, 0, bytes.Length)
        let f32 (v : float) =
            let b = Array.zeroCreate<byte> 4
            BinaryPrimitives.WriteSingleLittleEndian(Span b, float32 v)
            ms.Write(b, 0, 4)
        let i32 (v : int) =
            let b = Array.zeroCreate<byte> 4
            BinaryPrimitives.WriteInt32LittleEndian(Span b, v)
            ms.Write(b, 0, 4)
        for (x, y, z) in corners do
            f32 x; f32 y; f32 z; f32 1.0
        for (a, b, c) in tris do
            ms.WriteByte 3uy
            i32 a; i32 b; i32 c
        ms.ToArray()

    let stlAscii () =
        let sb = StringBuilder()
        sb.AppendLine "solid cube" |> ignore
        for (a, b, c) in tris do
            sb.AppendLine "  facet normal 0 0 0" |> ignore
            sb.AppendLine "    outer loop" |> ignore
            for i in [| a; b; c |] do
                let (x, y, z) = corners[i]
                sb.AppendLine $"      vertex %g{x} %g{y} %g{z}" |> ignore
            sb.AppendLine "    endloop" |> ignore
            sb.AppendLine "  endfacet" |> ignore
        sb.AppendLine "endsolid cube" |> ignore
        sb.ToString()


[<TestClass>]
type IoTests () =

    let tempFile (ext : string) =
        Path.Combine(Path.GetTempPath(), $"meshorient_test_{Guid.NewGuid():N}{ext}")

    let assertIsTheCube (label : string) (m : Mesh) =
        Assert.AreEqual(12, m.TriangleCount, $"{label}: triangle count")
        let b = Mesh.bounds m
        Assert.AreEqual(0.0, b.Min.X, 1e-6, $"{label}: min X")
        Assert.AreEqual(0.0, b.Min.Y, 1e-6, $"{label}: min Y")
        Assert.AreEqual(0.0, b.Min.Z, 1e-6, $"{label}: min Z")
        Assert.AreEqual(2.0, b.Max.X, 1e-6, $"{label}: max X")
        Assert.AreEqual(2.0, b.Max.Y, 1e-6, $"{label}: max Y")
        Assert.AreEqual(2.0, b.Max.Z, 1e-6, $"{label}: max Z")

    let withFile (ext : string) (write : string -> unit) (check : string -> unit) =
        let p = tempFile ext
        try
            write p
            check p
        finally
            if File.Exists p then File.Delete p

    [<TestMethod>]
    member _.Reads_ascii_stl () =
        withFile ".stl" (fun p -> File.WriteAllText(p, Cube.stlAscii ()))
                        (fun p -> assertIsTheCube "ascii STL" (MeshIO.load p))

    [<TestMethod>]
    member _.Reads_obj () =
        withFile ".obj" (fun p -> File.WriteAllText(p, Cube.obj ()))
                        (fun p -> assertIsTheCube "OBJ" (MeshIO.load p))

    [<TestMethod>]
    member _.Reads_ascii_ply () =
        withFile ".ply" (fun p -> File.WriteAllText(p, Cube.plyAscii ()))
                        (fun p -> assertIsTheCube "ascii PLY" (MeshIO.load p))

    [<TestMethod>]
    member _.Reads_binary_ply_with_extra_property () =
        withFile ".ply" (fun p -> File.WriteAllBytes(p, Cube.plyBinaryLe ()))
                        (fun p -> assertIsTheCube "binary PLY" (MeshIO.load p))

    /// A binary STL whose 80-byte header starts with "solid" — a real thing
    /// plenty of exporters emit. Sniffing the prefix instead of checking the
    /// length reads it as ASCII, finds no `vertex` tokens, and hands back an
    /// empty mesh. The size check has to come first.
    [<TestMethod>]
    member _.Binary_stl_whose_header_says_solid_is_still_binary () =
        withFile ".stl" (fun p ->
            let m = Mesh.create
                        (Cube.tris |> Array.collect (fun (a, b, c) ->
                            [| a; b; c |] |> Array.map (fun i ->
                                let (x, y, z) = Cube.corners[i]
                                Vec3.create x y z)))
                        (Array.init 36 id) p
            MeshIO.saveStlBinary p m
            // Overwrite the header text with something that starts "solid".
            let bytes = File.ReadAllBytes p
            Encoding.ASCII.GetBytes("solid exported by something").CopyTo(bytes, 0)
            File.WriteAllBytes(p, bytes))
            (fun p -> assertIsTheCube "binary STL claiming to be solid" (MeshIO.load p))

    [<TestMethod>]
    member _.Stl_write_round_trips_the_synthetic_scan () =
        let src = Fixtures.synthetic.Value
        withFile ".stl" (fun p -> MeshIO.saveStlBinary p src) (fun p ->
            let back = MeshIO.load p
            Assert.AreEqual(src.TriangleCount, back.TriangleCount, "triangle count")
            let a, b = Mesh.bounds src, Mesh.bounds back
            // float32 in the file is the only loss; on a ~200 mm part that is
            // about 1e-5 mm, well inside any tolerance that matters.
            for (name, x, y) in [ "min X", a.Min.X, b.Min.X; "min Y", a.Min.Y, b.Min.Y
                                  "min Z", a.Min.Z, b.Min.Z; "max X", a.Max.X, b.Max.X
                                  "max Y", a.Max.Y, b.Max.Y; "max Z", a.Max.Z, b.Max.Z ] do
                Assert.AreEqual(x, y, 1e-4, name))

    [<TestMethod>]
    member _.Synthetic_scan_has_its_documented_dimensions () =
        let m = Fixtures.synthetic.Value
        let b = Mesh.bounds m
        printfn "synthetic: %d triangles, bbox %A .. %A" m.TriangleCount b.Min b.Max
        Assert.IsTrue(m.TriangleCount > 100_000, $"expected a dense scan, got {m.TriangleCount}")
        // From make_synthetic.py, plus 0.05 mm of vertex jitter.
        Assert.AreEqual(-85.0, b.Min.X, 0.3, "min X")
        Assert.AreEqual(85.0, b.Max.X, 0.3, "max X")
        Assert.AreEqual(-14.5, b.Min.Y, 0.3, "min Y (left round boss)")
        Assert.AreEqual(14.0, b.Max.Y, 0.3, "max Y (right trigger-bar boss)")
        Assert.AreEqual(-60.0, b.Min.Z, 0.3, "min Z")
        Assert.AreEqual(50.0, b.Max.Z, 0.3, "max Z")

    [<TestMethod>]
    member _.Oriented_path_sits_beside_the_source () =
        let p = MeshIO.orientedPathFor (Path.Combine("C:", "scans", "glock.ply"))
        Assert.AreEqual(Path.Combine("C:", "scans", "glock_oriented.stl"), p)

    [<TestMethod>]
    member _.Unsupported_extension_is_an_error () =
        let threw = try MeshIO.load "nope.3mf" |> ignore; false with _ -> true
        Assert.IsTrue(threw, "an unknown extension must fail loudly, not return an empty mesh")
