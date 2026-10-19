# dem2mesh: GeoTIFF to printable solid and Blender scene

Status: design for review. Written from the brainstorming conversation of 2026-10-19.

## 1. Intent

A second command in this repository, `dem2mesh`, turns an elevation GeoTIFF
into two things from one run:

- an STL file that is a finished solid, ready to slice and print: the terrain
  on top, vertical walls around, a flat base underneath;
- a Blender file that opens ready to render: the terrain surface, a camera
  framed on it, a sun, and a plain material.

The first things the owner wants to make are a printed model of Table
Mountain and a Blender scene of Cape Town's topography. The tool is generic,
though: point it at any single-band elevation GeoTIFF, optionally crop it, and
choose the output grid.

Two kinds of input exist today and both must work:

- files written by `dl_elevation` (this repository): float32, WGS84 degrees
  (EPSG:4326), one tiepoint at the north-west corner, `RasterPixelIsArea`,
  NaN for pixels never fetched, -32768 (`GDAL_NODATA`) for the server's
  no-data (sea);
- the City of Cape Town's 5 m model `CCT_GLM2019_5m_DEM_1D_v1.tif` and the
  ten pieces split from it: float32, Transverse Mercator Lo19 in metres
  (EPSG:20482, Hartebeesthoek94, central meridian 19 E, scale 1, no false
  origin), `RasterPixelIsPoint`, `GDAL_NODATA` 0, heights -37.9 to 1590.1 m.

## 2. Goals, non-goals, anti-goals

Goals:

- One command, one run, both outputs, from the same resampled grid, so the
  print and the renders match.
- Optional crop by north, east, south and west in WGS84 degrees; the whole
  file by default.
- Output grid either full resolution (one vertex per source pixel) or a size
  the user gives, with smooth resampling (the owner asked for quadratic
  interpolation) so a coarse model has rounded ridges, not stair steps.
- A watertight solid by construction, base at sea level by default, lowered
  automatically when ground lies below sea level, overridable.
- Vertical exaggeration as a parameter, default 1 (true shape).
- Blender scene that renders on first open with a camera and a sun.

Non-goals (for now):

- Draping aerial photos or any texture over the terrain. The scene leaves a
  hook for it (section 7.3). The City's server will serve its orthophotos
  (January 2026 at 5 cm per pixel, among others) as JPEG or PNG for any
  bounding box, up to 4000 x 4000 pixels per request, so a later feature is
  one image request per window.
- Reading anything but single-band float32 (or other numeric) elevation
  GeoTIFFs in a geographic or Transverse Mercator projection. Other
  projections are refused with a message pointing at GDAL's `gdalwarp`.
- Multi-band or RGB files, the old RGB16 downloads, point clouds, contours.
- Editing or repairing meshes, hollowing, text labels, magnets, split prints.
- Downloading data. `dl_elevation` and the City's files are the sources.

Anti-goals (what would make it a failure even if it runs):

- A print that is not watertight or has inverted faces, so the slicer refuses
  it or prints it hollow.
- A Blender file that opens to a grey blob or an empty viewport.
- Silent damage to the data: heights shifted, the window off by a pixel, east
  and west mirrored, a model that is a mirror image of the real place.
- A run that "works" and produces a 25 GB STL nobody can open.

## 3. The command line

Executable `dem2mesh`, built beside `dl_elevation`, parsed with CLI11 in the
same style as `dl_elevation`'s `Cli.cpp` (strict numbers, `error:` prefix,
help text with ranges, usage errors exit 1).

| Flag | Required | Default | Meaning |
|---|---|---|---|
| `--input PATH`, `-i` | yes | | the GeoTIFF |
| `--output STEM`, `-o` | yes | | output files are `STEM.stl`, `STEM.ply`, `STEM.blend` |
| `--north`, `--south`, `--east`, `--west` | all four or none | whole file | crop window, WGS84 decimal degrees |
| `--grid W[xH]` | no | full resolution | output grid in vertices across (and down); without `xH`, the height follows the window's aspect ratio in metres so cells are square |
| `--z-scale F` | no | 1 | vertical exaggeration; heights and base are multiplied by F |
| `--base M` | no | sea level, 0 | base plane height in metres before exaggeration; lowered automatically when the terrain goes below it (section 5.3) |
| `--scale F` | no | 1 | model units per metre in the STL and PLY (0.02 makes a 10 km window 200 mm wide) |
| `--no-blend` | no | off | write the STL and PLY only; do not run Blender |
| `--blender PATH` | no | see 7.1 | the Blender executable |
| `--overwrite` | no | off | replace existing output files; otherwise existing outputs are an error before anything is read |
| `--dry-run` | no | off | print the plan (section 3.1) and write nothing |
| `--verbose` | no | off | print per-stage timings and the Blender log |

Validation, in order: the input exists and is readable; `--output` names a
file stem in an existing writable directory; the four crop flags come
together, each finite, north > south, east > west, within ±90 and ±180;
`--grid` width and height at least 2; `--z-scale` and `--scale` positive and
finite; `--base` finite. The window must overlap the file by at least 2 x 2
pixels. The output grid must not exceed 25 million vertices (an STL of about
2.5 GB); above 4 million the tool prints a warning with the expected file
sizes and continues. Full resolution on the City's whole file is 254 million
pixels and is refused with the message to use `--grid` or a crop.

### 3.1 The dry run

`--dry-run` prints: the input's projection (geographic or Transverse Mercator
with its parameters), pixel size, raster type (point or area), no-data value,
the pixel window chosen for the crop and its extent in degrees and metres,
the output grid and cell size in metres, the height range in the window, the
base plane chosen and why, the number of triangles and the sizes of the STL
and PLY, and which Blender would be run.

## 4. Reading the GeoTIFF

A new reader, not `DemRaster::load`, which refuses files that `dl_elevation`
did not write. The reader accepts:

- one sample per pixel, sample format float32 (also int16, int32, uint16,
  float64 converted to float on read, since public DEMs come that way);
- strips or tiles, any libtiff-supported compression;
- `ModelPixelScale` plus one `ModelTiepoint` (any pixel, not only 0,0) or a
  `ModelTransformation` without rotation; rotated or sheared files are
  refused;
- `GTRasterTypeGeoKey` point or area; absent means area;
- `GTModelTypeGeoKey` geographic (units degrees) or projected with
  `ProjCoordTransGeoKey = CT_TransverseMercator` and the natural origin
  longitude and latitude, scale factor, false easting and northing, and the
  ellipsoid from the GeoKeys (semi-major axis and inverse flattening, or
  `GeogEllipsoidGeoKey` GRS 1980 or WGS 84). A projected file that gives only
  an EPSG code and no parameters is refused with the code in the message,
  except codes the reader knows: EPSG:20482 (Lo19) and the other South
  African Lo grids EPSG:2046 to 2055 (Lo15 to Lo33, Hartebeesthoek94), and
  UTM zones 32601 to 32660 and 32701 to 32760 (WGS 84). This is a small
  table; no PROJ dependency, because the Homebrew PROJ on the owner's machine
  cannot find EPSG:20482 in its database and GDAL there is broken;
- no-data from `GDAL_NODATA` (tag 42113) when present; NaN always counts as
  no data too.

Heights are metres. Vertical datum is ignored (the two known inputs are mean
sea level based; a note in `--dry-run` says "heights taken as metres above
sea level").

Pixel geometry. For `RasterPixelIsPoint` a sample sits exactly at tiepoint +
index x scale. For `RasterPixelIsArea` it sits at the pixel centre, half a
pixel in. The mesh vertex for a source pixel is placed at the sample
position, so area files are not shifted by half a pixel (the known
half-pixel display offset in `dl_elevation`'s files is corrected here, not
reproduced).

## 5. The pipeline

### 5.1 Crop window to pixel window

Geographic file: the crop box maps directly to fractional pixel coordinates;
the pixel window is the smallest range of whole pixels containing it.

Transverse Mercator file: project the crop box's four corners and the
midpoints of its four edges with the forward Transverse Mercator formulas
(Krüger series or the USGS form, accurate to a millimetre within a few
degrees of the central meridian), take the bounding rectangle of those eight
points in projected metres, then the pixel window as above. The mesh covers
the pixel window, which is slightly larger than the requested box; the dry
run and the normal run print the window actually used, in degrees.

Without a crop, the window is the whole file.

### 5.2 Metres

The mesh is in metres with the window's south-west corner at the origin,
x east, y north, z up (Blender's convention; the STL's too, by common use).

Projected file: x and y are the projected coordinates minus the window's
south-west corner. Geographic file: degrees are converted with a local scale
at the window's middle latitude, metres per degree of latitude
111,132.954 - 559.822 cos 2φ + 1.175 cos 4φ and of longitude 111,412.84 cos φ
- 93.5 cos 3φ, which is within 0.1 percent over a 10 km window. No proper
projection is needed for a relief model.

### 5.3 Heights, no data, the base plane

No-data and NaN pixels are set to sea level, 0 m, before resampling, so
coasts come out as flat water and gaps in a `dl_elevation` file become flat
patches; the run reports how many pixels were filled.

The base plane: the owner's choice is sea level by default, overridable with
`--base`. To keep the solid closed, the plane is lowered to the lowest
height in the window when that is below the requested level, and the run
says so. To avoid zero-height walls where terrain meets the plane, the plane
sits a margin below that level: 1 percent of the window's height range, at
least 1 m. After exaggeration, every z (heights and base) is multiplied by
`--z-scale`; after that, everything is multiplied by `--scale`.

Heights are clamped to the base plane from below (a pixel below the plane
cannot happen after the lowering, but a resampled value can undershoot a
little at a cliff; clamping keeps the solid valid).

### 5.4 Resampling

The owner asked for quadratic interpolation. The decision: a separable
quadratic B-spline kernel (three taps), which is C1-smooth and never
overshoots. Two regimes:

- Full resolution or an output grid finer than the source: the kernel runs at
  the source spacing and interpolates between source samples.
- Coarser output grid: the kernel is stretched to the output spacing, so each
  output vertex averages the source samples under its footprint with
  quadratic weights. That antialiases (no stair steps or moiré) and smooths
  in one pass.

Properties the tests pin: a constant surface is reproduced exactly; a plane
is reproduced exactly (within float rounding); the output has no values
outside the source's range; resampling a grid to its own size reproduces it
closely (a B-spline approximates, it does not interpolate: a sharp summit is
lowered a little, more the coarser the grid; the dry run prints the summit
height before and after so the owner can judge, and the builder may add a
`--sampling point` option later if that matters for prints).

Vertex positions: output vertex (i, j) of a W x H grid sits at
x = i x (window width) / (W - 1), y = j x (window height) / (H - 1) in
metres from the south-west corner, so the mesh spans the window exactly.

### 5.5 The solid

Top surface: the W x H vertices, 2 (W - 1)(H - 1) triangles, consistent
winding with normals up. Walls: for each of the 2 (W - 1) + 2 (H - 1) boundary
edges, two triangles down to the base plane, normals outward. Base: two
triangles spanning the rectangle at the base plane, normal down. Shared
vertices along every seam, so the mesh is closed and manifold by
construction. Triangle count 2 (W - 1)(H - 1) + 4 (W + H - 2) + 2.

STL: binary, 80-byte header naming the tool and the window, little-endian,
per-triangle unit normals computed from the vertices, attribute bytes zero.
Size 84 + 50 x triangles.

PLY: binary little-endian, the top surface only, shared vertices (x, y, z as
float32) and triangle faces, the hand-off to Blender and usable in any 3D
tool. Written whether or not Blender runs.

The STL and PLY are written to `STEM.stl.tmp` and `STEM.ply.tmp` and renamed
into place when complete, as `DemRaster::save` does.

## 6. Verification

After writing, the tool checks its own STL: triangle count equals the
formula; every edge is shared by exactly two triangles with opposite
direction (a hash of directed edges); no triangle has zero area; the signed
volume is positive. A failure is a bug and exits 1 with the counts. Blender's
check (section 7.2) is a second opinion, not the only one.

## 7. Blender

### 7.1 Running Blender

No library outside Blender writes a correct `.blend`, so the tool runs the
user's Blender in the background:

    blender --background --factory-startup --python <script> -- <args>

The script is `src/mesh/blender_scene.py` in the repository, embedded into
the executable at build time (CMake reads the file into a generated header),
written to a temporary file at run time, and deleted afterwards. The Blender
executable is `--blender` if given, else `$BLENDER`, else
`/Applications/Blender.app/Contents/MacOS/Blender`, else `blender` on `PATH`.
Target: Blender 4.2 LTS or later; the owner has 4.5.12 LTS (Intel build,
Python 3.11). The script checks `bpy.app.version` and exits with a message
on anything older.

If Blender cannot be found or fails, the STL and PLY stay, the Blender error
is printed, and the exit code is 1 (the owner asked for the `.blend`; a
missing one is a failure). `--no-blend` skips Blender and exits 0.

### 7.2 What the script does

Arguments: the PLY path, the STL path, the `.blend` path, the window size in
model units, the height range, a name for the object.

1. Import the PLY (`bpy.ops.wm.ply_import`), name the object after the
   output stem, shade smooth, origin at the mesh's centre of its base.
2. Material: one matte material, a warm grey with roughness 0.9, so relief
   reads under the sun. A height-based colour ramp is a nice touch the builder
   may add if it costs little.
3. Sun: a Sun light, strength 3, elevation 40 degrees, from the north-west
   (light from the top-left of the frame reads as relief to most eyes).
4. Camera: perspective, 50 mm, placed to the south-east of the terrain at
   an elevation of 35 degrees above the terrain's centre, far enough that the
   whole window fits with a 5 percent margin, looking at the centre. Set as
   the scene camera. Clip end large enough for the model.
5. World: light grey background, strength 1.
6. Render: EEVEE (whatever identifier the running Blender reports for it),
   1920 x 1080, 64 samples; Cycles is one click away.
7. Scene unit scale: metres when `--scale` is 1, otherwise unit scale set so
   that one model unit is 1/`--scale` metres, and the unit system shows
   millimetres for print-sized models.
8. Save with `bpy.ops.wm.save_as_mainfile(filepath, compress=True)`.
9. STL check: import the STL into a scratch scene, run `mesh.validate()`,
   count non-manifold edges and inverted faces with bmesh, print one line
   `stl check: N triangles, M non-manifold edges, K inverted faces`, and
   exit non-zero if M or K is not 0.

The script prints one line per step; `--verbose` shows them.

### 7.3 Hook for textures

The material has an Image Texture node left unconnected, and the mesh has a
planar UV map (u = x / width, v = y / height). Draping an aerial photo later
means fetching one image for the window and connecting it; no mesh change.

## 8. Where it lives in the repository

- `src/mesh/`: a static library `demmesh` depending on `demdl` (for
  `BoundingBox` and the libtiff setup): `GeoTiffGrid` (the reader, section 4),
  `TransverseMercator` (forward projection and the EPSG table),
  `LocalMetres` (section 5.2), `Resampler` (section 5.4), `SolidMesh`
  (section 5.5, including the self-check of section 6), `StlWriter`,
  `PlyWriter`, `BlenderRunner` (section 7.1), `blender_scene.py`.
- `src/app/`: `MeshCli.cpp` (CLI11, same conventions as `Cli.cpp`) and
  `dem2mesh_main.cpp`.
- CMake: `demmesh`, `demmesh_cli` (object library), `dem2mesh`; the tests
  target gains the new test files; the Python script is embedded with
  `file(READ ...)` into a generated header.
- Tests (Catch2, same tags style): the reader on files written in the tests
  (a geographic one via `DemRaster::save`, a Transverse Mercator one written
  with libtiff in the test, point and area variants, the no-data tag,
  strips and tiles, an int16 file); Transverse Mercator against known points
  (lon 18.44055175, lat -34.2111625310268 is x -51556.570, y -3787226.019 in
  Lo19; the City's file corner -64012.5, -3705051.806 is pixel 0,0); the crop
  window mapping; the resampler properties of section 5.4; the solid's
  invariants of section 6 on several grids including 2 x 2; byte-level STL
  and PLY layout; the CLI (same style as `test_Cli.cpp`); an end-to-end run
  of the built `dem2mesh` on a small synthetic file with `--no-blend`,
  reading the STL back and checking volume and extent; and, when Blender is
  found on the machine, one end-to-end run that produces a `.blend` and
  checks the STL-check line (skipped with a message otherwise, so CI without
  Blender still passes).
- Docs: CLAUDE.md gains a section for `dem2mesh`; this design is the spec.

## 9. Constraints

- C++20, AppleClang 15, CMake 3.27+, the repository's conventions (strict
  CLI numbers, `error:` prefixes, no `git add -A`, Catch2 names not starting
  with `-`).
- No new third-party dependency beyond what the repository has (libtiff,
  libgeotiff, CLI11, Catch2). No GDAL, no PROJ.
- Never contact the network.
- Memory: the window's pixels as float32 plus the output grid; the City's
  whole file is 1 GB, so a run on it without a crop must read strip by strip
  and resample on the fly rather than hold the file twice. The builder
  decides the streaming scheme; the dry run must not read the pixels.

## 10. Decisions and who made them

The owner decided: generic input; optional crop, whole file by default; grid
size as the main control with full resolution as the alternative; quadratic
interpolation; two outputs from one run; the STL is a finished solid with
walls and base; base at sea level unless overridden; a vertical exaggeration
parameter; a second command in this repository; STL written in C++ directly
with Blender only checking it; Blender scene for camera-and-sun renders, no
textures yet.

My calls (change any of them): mesh in metres with the south-west corner at
the origin and `--scale` for print sizing; the base plane lowered
automatically with a small margin; no-data filled at sea level; a quadratic
B-spline (approximating) rather than an interpolating quadratic; the pixel
window slightly larger than the crop box; the 4 million warning and 25
million refusal; PLY as the hand-off format; the embedded script and the
Blender search order; a missing Blender is exit 1 unless `--no-blend`; the
scene's light, camera and material choices; the small EPSG table instead of
PROJ; refusing projections other than Transverse Mercator; local metres for
geographic files; correcting the half-pixel offset of area files.

## 11. Left to the builder

- The exact quadratic B-spline formulation and the streaming scheme for
  large files.
- The Transverse Mercator series used, provided the tests' points pass to a
  millimetre.
- How the window's degrees are reported when the file is projected.
- Whether the height-based colour ramp is worth adding.
- File layout within `src/mesh/` beyond the units named.
