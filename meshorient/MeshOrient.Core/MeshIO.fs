/// Mesh file I/O: STL (binary + ASCII), OBJ and PLY in; binary STL out.
///
/// Self-contained on purpose, the way f2s's `meshio_lite.py` is. A scan loader
/// and an STL writer are a few hundred lines; a mesh library is a dependency
/// that has to be kept current forever. Nothing here needs a mesh kernel.
module MeshOrient.Core.MeshIO

open System
open System.Buffers.Binary
open System.Globalization
open System.IO

// ------------------------------------------------------------------- STL in

/// True when the byte stream matches a binary STL exactly: an 84-byte preamble
/// plus 50 bytes per declared triangle.
///
/// This is checked BEFORE the "solid" prefix, not after. Plenty of binary
/// writers put the word "solid" in the 80-byte header, so prefix-sniffing
/// alone reads a binary file as ASCII, finds no `vertex` tokens, and hands
/// back an empty mesh instead of an error.
let private looksBinaryStl (bytes : byte[]) =
    bytes.Length >= 84
    && int64 bytes.Length = 84L + 50L * int64 (BinaryPrimitives.ReadUInt32LittleEndian(ReadOnlySpan(bytes, 80, 4)))

let private parseBinaryStl (path : string) (bytes : byte[]) =
    let n = int (BinaryPrimitives.ReadUInt32LittleEndian(ReadOnlySpan(bytes, 80, 4)))
    let verts = Array.zeroCreate<Vec3> (n * 3)
    let inline readVec (off : int) =
        {   X = float (BinaryPrimitives.ReadSingleLittleEndian(ReadOnlySpan(bytes, off, 4)))
            Y = float (BinaryPrimitives.ReadSingleLittleEndian(ReadOnlySpan(bytes, off + 4, 4)))
            Z = float (BinaryPrimitives.ReadSingleLittleEndian(ReadOnlySpan(bytes, off + 8, 4))) }
    for i in 0 .. n - 1 do
        // + 12 skips the stored face normal: it is routinely absent, zero, or
        // simply wrong, and we always recompute from the winding anyway.
        let off = 84 + i * 50 + 12
        verts[i * 3] <- readVec off
        verts[i * 3 + 1] <- readVec (off + 12)
        verts[i * 3 + 2] <- readVec (off + 24)
    Mesh.create verts (Array.init (n * 3) id) path

let private parseAsciiStl (path : string) (text : string) =
    let tokens = text.Split([| ' '; '\t'; '\r'; '\n' |], StringSplitOptions.RemoveEmptyEntries)
    let ci = CultureInfo.InvariantCulture
    let verts = ResizeArray<Vec3>()
    let mutable i = 0
    while i < tokens.Length do
        if String.Equals(tokens[i], "vertex", StringComparison.OrdinalIgnoreCase) && i + 3 < tokens.Length then
            verts.Add { X = Double.Parse(tokens[i + 1], ci)
                        Y = Double.Parse(tokens[i + 2], ci)
                        Z = Double.Parse(tokens[i + 3], ci) }
            i <- i + 4
        else i <- i + 1
    if verts.Count % 3 <> 0 then
        failwithf "ASCII STL has %d vertices, which is not a whole number of triangles" verts.Count
    let v = verts.ToArray()
    Mesh.create v (Array.init v.Length id) path

let private loadStl (path : string) =
    let bytes = File.ReadAllBytes path
    if looksBinaryStl bytes then parseBinaryStl path bytes
    else parseAsciiStl path (Text.Encoding.ASCII.GetString bytes)

// ------------------------------------------------------------------- OBJ in

let private loadObj (path : string) =
    let ci = CultureInfo.InvariantCulture
    let verts = ResizeArray<Vec3>()
    let idx = ResizeArray<int>()
    for line in File.ReadLines path do
        let line = line.AsSpan().Trim()
        if line.StartsWith "v " then
            let p = line.ToString().Split([| ' '; '\t' |], StringSplitOptions.RemoveEmptyEntries)
            verts.Add { X = Double.Parse(p[1], ci); Y = Double.Parse(p[2], ci); Z = Double.Parse(p[3], ci) }
        elif line.StartsWith "f " then
            let p = line.ToString().Split([| ' '; '\t' |], StringSplitOptions.RemoveEmptyEntries)
            // "f 1/2/3" — only the position index matters here. OBJ indices are
            // 1-based, and negative ones count back from the current end.
            let corner (tok : string) =
                let i = Int32.Parse(tok.Split('/')[0], ci)
                if i > 0 then i - 1 else verts.Count + i
            let c = Array.init (p.Length - 1) (fun k -> corner p[k + 1])
            for k in 1 .. c.Length - 2 do          // fan-triangulate n-gons
                idx.Add c[0]
                idx.Add c[k]
                idx.Add c[k + 1]
    Mesh.create (verts.ToArray()) (idx.ToArray()) path

// ------------------------------------------------------------------- PLY in

type private PlyScalar =
    | I8 | U8 | I16 | U16 | I32 | U32 | F32 | F64

    member t.Size =
        match t with
        | I8 | U8 -> 1
        | I16 | U16 -> 2
        | I32 | U32 | F32 -> 4
        | F64 -> 8

type private PlyProp =
    | Scalar of name : string * ty : PlyScalar
    | List of name : string * countTy : PlyScalar * itemTy : PlyScalar

let private plyScalar =
    dict [
        "char", I8; "int8", I8; "uchar", U8; "uint8", U8
        "short", I16; "int16", I16; "ushort", U16; "uint16", U16
        "int", I32; "int32", I32; "uint", U32; "uint32", U32
        "float", F32; "float32", F32; "double", F64; "float64", F64 ]

let private plyType (s : string) =
    match plyScalar.TryGetValue s with
    | true, t -> t
    | _ -> failwithf "Unsupported PLY property type '%s'" s

/// Read one scalar from `bytes` at `off`, returning it as a float (positions)
/// and an int (indices) both — PLY stores either as any of these types.
let private readPlyScalar (bytes : byte[]) (off : int) (ty : PlyScalar) (bigEndian : bool) : float =
    let span = ReadOnlySpan(bytes, off, ty.Size)
    if bigEndian then
        match ty with
        | I8 -> float (sbyte span[0])
        | U8 -> float span[0]
        | I16 -> float (BinaryPrimitives.ReadInt16BigEndian span)
        | U16 -> float (BinaryPrimitives.ReadUInt16BigEndian span)
        | I32 -> float (BinaryPrimitives.ReadInt32BigEndian span)
        | U32 -> float (BinaryPrimitives.ReadUInt32BigEndian span)
        | F32 -> float (BinaryPrimitives.ReadSingleBigEndian span)
        | F64 -> BinaryPrimitives.ReadDoubleBigEndian span
    else
        match ty with
        | I8 -> float (sbyte span[0])
        | U8 -> float span[0]
        | I16 -> float (BinaryPrimitives.ReadInt16LittleEndian span)
        | U16 -> float (BinaryPrimitives.ReadUInt16LittleEndian span)
        | I32 -> float (BinaryPrimitives.ReadInt32LittleEndian span)
        | U32 -> float (BinaryPrimitives.ReadUInt32LittleEndian span)
        | F32 -> float (BinaryPrimitives.ReadSingleLittleEndian span)
        | F64 -> BinaryPrimitives.ReadDoubleLittleEndian span

let private loadPly (path : string) =
    let bytes = File.ReadAllBytes path

    // ---- header. Read it a byte at a time: the body may be binary, so we
    // cannot decode the whole file as text first.
    let mutable pos = 0
    let readLine () =
        let start = pos
        while pos < bytes.Length && bytes[pos] <> 10uy do pos <- pos + 1
        let stop = if pos > start && bytes[pos - 1] = 13uy then pos - 1 else pos
        pos <- pos + 1
        Text.Encoding.ASCII.GetString(bytes, start, stop - start)

    if readLine().Trim() <> "ply" then failwith "Not a PLY file (no 'ply' magic)"
    let mutable format = ""
    let elements = ResizeArray<string * int * ResizeArray<PlyProp>>()
    let mutable go = true
    while go do
        if pos >= bytes.Length then failwith "Unexpected end of PLY header"
        let tok = readLine().Split([| ' '; '\t' |], StringSplitOptions.RemoveEmptyEntries)
        if tok.Length > 0 then
            match tok[0] with
            | "format" -> format <- tok[1]
            | "element" -> elements.Add(tok[1], Int32.Parse tok[2], ResizeArray())
            | "property" ->
                let _, _, props = elements[elements.Count - 1]
                if tok[1] = "list" then props.Add(List(tok[4], plyType tok[2], plyType tok[3]))
                else props.Add(Scalar(tok[2], plyType tok[1]))
            | "end_header" -> go <- false
            | _ -> ()

    let verts = ResizeArray<Vec3>()
    let idx = ResizeArray<int>()

    let addFace (c : int[]) =
        for k in 1 .. c.Length - 2 do
            idx.Add c[0]
            idx.Add c[k]
            idx.Add c[k + 1]

    match format with
    | "ascii" ->
        let ci = CultureInfo.InvariantCulture
        for name, count, props in elements do
            let names = props |> Seq.map (function Scalar(n, _) -> n | List(n, _, _) -> n) |> Seq.toArray
            let ix = Array.IndexOf(names, "x")
            let iy = Array.IndexOf(names, "y")
            let iz = Array.IndexOf(names, "z")
            for _ in 1 .. count do
                let f = readLine().Split([| ' '; '\t' |], StringSplitOptions.RemoveEmptyEntries)
                if name = "vertex" then
                    verts.Add { X = Double.Parse(f[ix], ci); Y = Double.Parse(f[iy], ci); Z = Double.Parse(f[iz], ci) }
                elif name = "face" then
                    let n = Int32.Parse(f[0], ci)
                    addFace (Array.init n (fun k -> Int32.Parse(f[k + 1], ci)))
    | "binary_little_endian" | "binary_big_endian" ->
        let big = format = "binary_big_endian"
        for name, count, props in elements do
            if name = "vertex" then
                if props |> Seq.exists (function List _ -> true | _ -> false) then
                    failwith "PLY vertex elements with list properties are not supported"
                // Precompute each property's offset within one fixed-size record.
                let mutable stride = 0
                let offsets = ResizeArray<string * int * PlyScalar>()
                for p in props do
                    match p with
                    | Scalar(n, t) -> offsets.Add(n, stride, t); stride <- stride + t.Size
                    | List _ -> ()
                let find n =
                    match offsets |> Seq.tryFind (fun (nm, _, _) -> nm = n) with
                    | Some(_, o, t) -> o, t
                    | None -> failwithf "PLY vertex element has no '%s' property" n
                let ox, tx = find "x"
                let oy, ty = find "y"
                let oz, tz = find "z"
                for _ in 1 .. count do
                    verts.Add { X = readPlyScalar bytes (pos + ox) tx big
                                Y = readPlyScalar bytes (pos + oy) ty big
                                Z = readPlyScalar bytes (pos + oz) tz big }
                    pos <- pos + stride
            elif name = "face" then
                for _ in 1 .. count do
                    for p in props do
                        match p with
                        | Scalar(_, t) -> pos <- pos + t.Size
                        | List(_, ct, it) ->
                            let n = int (readPlyScalar bytes pos ct big)
                            pos <- pos + ct.Size
                            let c = Array.init n (fun k -> int (readPlyScalar bytes (pos + k * it.Size) it big))
                            pos <- pos + n * it.Size
                            addFace c
            else
                // Skip a whole element we do not care about (edge, material...).
                for _ in 1 .. count do
                    for p in props do
                        match p with
                        | Scalar(_, t) -> pos <- pos + t.Size
                        | List(_, ct, it) ->
                            let n = int (readPlyScalar bytes pos ct big)
                            pos <- pos + ct.Size + n * it.Size
    | f -> failwithf "Unsupported PLY format '%s'" f

    Mesh.create (verts.ToArray()) (idx.ToArray()) path

// ---------------------------------------------------------------- load / save

/// Load a mesh, dispatching on the file extension.
let load (path : string) : Mesh =
    let m =
        match Path.GetExtension(path).ToLowerInvariant() with
        | ".stl" -> loadStl path
        | ".obj" -> loadObj path
        | ".ply" -> loadPly path
        | ext -> failwithf "Unsupported mesh format '%s' (use STL, OBJ or PLY)" ext
    if m.TriangleCount = 0 then
        failwithf "%s contains no triangles" (Path.GetFileName path)
    m

/// Write a binary STL.
///
/// The point of this tool's output: the same triangles the scan came in with,
/// rigidly transformed. Face normals are recomputed from the winding rather
/// than carried across, because the input's stored normals are untrustworthy
/// and would be stale after a rotation regardless.
let saveStlBinary (path : string) (m : Mesh) =
    let n = m.TriangleCount
    let buf = Array.zeroCreate<byte> (84 + 50 * n)
    Text.Encoding.ASCII.GetBytes("meshorient").CopyTo(buf, 0)
    BinaryPrimitives.WriteUInt32LittleEndian(Span(buf, 80, 4), uint32 n)
    let inline put (off : int) (v : Vec3) =
        BinaryPrimitives.WriteSingleLittleEndian(Span(buf, off, 4), float32 v.X)
        BinaryPrimitives.WriteSingleLittleEndian(Span(buf, off + 4, 4), float32 v.Y)
        BinaryPrimitives.WriteSingleLittleEndian(Span(buf, off + 8, 4), float32 v.Z)
    System.Threading.Tasks.Parallel.For(0, n, fun t ->
        let struct (a, b, c) = Mesh.triangle m t
        let off = 84 + t * 50
        put off (Vec3.normalize (Vec3.cross (b - a) (c - a)))
        put (off + 12) a
        put (off + 24) b
        put (off + 36) c) |> ignore
    File.WriteAllBytes(path, buf)

let private suffixedPathFor (suffix : string) (sourcePath : string) =
    let dir = Path.GetDirectoryName sourcePath
    let file = Path.GetFileNameWithoutExtension sourcePath + suffix + ".stl"
    if String.IsNullOrEmpty dir then file else Path.Combine(dir, file)

/// `<name>_oriented.stl` beside the source file — the same naming f2s uses,
/// so the two tools' outputs drop into a workflow interchangeably. Always
/// the PURE rigid transform of the scan, no flattens.
let orientedPathFor = suffixedPathFor "_oriented"

/// `<name>_cleaned.stl` — the oriented scan WITH flatten mutations baked in.
/// Written alongside `_oriented` so the untouched geometry is never the
/// price of the cleanup.
let cleanedPathFor = suffixedPathFor "_cleaned"
