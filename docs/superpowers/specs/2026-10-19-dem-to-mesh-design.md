# dem2mesh: GeoTIFF to printable solid and Blender scene

Status: approved by the owner on 2026-10-19 after a builder check; the plan comes next.
Written from the brainstorming conversation of 2026-10-19.

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

- A cliff or a pit where the data merely ends: no-data must never read as
  terrain.

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
| `--plinth U` | no | 2 | minimum thickness of the solid under its lowest terrain point, in model units after scaling (section 5.3) |
| `--no-blend` | no | off | write the STL and PLY only; do not run Blender |
| `--blender PATH` | no | see 7.1 | the Blender executable |
| `--overwrite` | no | off | replace existing output files; otherwise existing outputs are an error before anything is read |
| `--dry-run` | no | off | print the plan (section 3.1) and write nothing |
| `--verbose` | no | off | print per-stage timings and the Blender log |

Validation, in order: the input exists and is readable; `--output` names a
file stem in an existing writable directory; the four crop flags come
together, each finite, north > south, east > west, within ±90 and ±180;
`--grid` width and height at least 2; `--z-scale` and `--scale` positive and
finite; `--base` finite; `--plinth` zero or positive and finite. The window must overlap the file by at least 2 x 2
pixels. The output grid must not exceed 25 million vertices (an STL of about
2.5 GB); above 4 million the tool prints a warning with the expected file
sizes and continues. Full resolution on the City's whole file is 254 million
pixels and is refused with the message to use `--grid` or a crop.

### 3.1 The dry run

`--dry-run` prints: the input's projection (geographic or Transverse Mercator
with its parameters), pixel size, raster type (point or area), no-data value,
the pixel window chosen for the crop and its extent in degrees and metres,
the output grid and cell size in metres, the height range in the window, the
base plane chosen and why, the number of no-data pixels and how they are
filled, the number of triangles and the sizes of the STL and PLY, and which
Blender would be run. It reads the window's pixels to do so (a whole-file dry
run on the City's model reads 1 GB, a few seconds from an SSD) and writes
nothing.

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
  except the UTM zones 32601 to 32660 and 32701 to 32760 (WGS 84, central
  meridian from the zone, scale 0.9996, false easting 500 km, false northing
  10,000 km in the south), which a small table covers. The City's file names
  code 20482 but also carries the full Transverse Mercator parameters, and
  the reader uses the parameters, so no table entry is needed for it. The
  EPSG South African grids 2046 to 2055 (Lo15 to Lo33) are south-oriented
  (westing and southing) and are refused by code, because reading one as
  east/north would rotate the model; a file with explicit
  `CT_TransvMercator_SouthOriented` parameters is refused too. No PROJ
  dependency: the Homebrew PROJ on the owner's machine cannot resolve these
  codes and GDAL there is broken;
- no-data from `GDAL_NODATA` (tag 42113) when present; NaN always counts as
  no data too.

Heights are metres. Vertical datum is ignored (the two known inputs are mean
sea level based; a note in `--dry-run` says "heights taken as metres above
sea level").

Pixel geometry. For `RasterPixelIsPoint` a sample sits exactly at tiepoint +
index x scale. For `RasterPixelIsArea` it sits at the pixel centre, half a
pixel in. The mesh vertex for a source pixel is placed at the sample
position. One producer is special-cased: a file whose `ImageDescription`
starts with `dl_elevation;` declares `RasterPixelIsArea` but its samples sit
at the pixel's north-west corner (the WMS server samples that corner, as the
recorded `Point:` lines in `tests/test_WmsQuery.cpp` show), so the reader
places them there. Applying the area rule to those files would reproduce the
half-pixel shift that GIS tools already show for them. A follow-up outside
this project: make `dl_elevation` write `RasterPixelIsPoint`, which the
earlier design's log left open; `DemRaster::load` does not check the raster
type, so existing files keep resuming.

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

No-data and NaN pixels are filled from the nearest pixel that has a height
(the owner's choice, made when the builder check showed that in the City's
file no-data also covers land outside the municipal boundary, up to 1,559 m
at the Hottentots Holland edge). Sea therefore comes out flat at about coast
level, land beyond the boundary continues as a plateau at the edge height
instead of dropping to a cliff, and a gap in an unfinished `dl_elevation`
file takes its surroundings' height rather than becoming a pit. Nearest means
nearest by distance in pixels, found with one breadth-first pass outward from
every valid pixel (a multi-source distance transform), so the fill is
deterministic and costs one pass over the window. A window with no valid
pixel at all is an error. The run reports how many pixels were filled and
the height range they were filled with. Filling happens before resampling.

The base plane: the owner's choice is sea level by default, overridable with
`--base`. To keep the solid closed, the plane is lowered to the lowest
height in the window when that is below the requested level, and the run
says so. To avoid zero-height walls where terrain meets the plane, the plane
sits a margin below that level: 1 percent of the window's height range, at
least 1 m. After exaggeration, every z (heights and base) is multiplied by
`--z-scale`; after that, everything is multiplied by `--scale`.

A plinth keeps the print from having a paper-thin floor. In a window that
reaches the sea the margin alone puts the base about 11 m below sea level for
a Table Mountain window, which is 0.2 mm at `--scale 0.02`: the sea and the
low coast would print one layer thick. So the base plane is pushed down,
after scaling, until the solid is at least `--plinth` model units thick under
its lowest terrain point; the default is 2 (2 mm when the slicer reads the
STL as millimetres, 2 m at `--scale 1`). The dry run reports the plinth's
effect. The PLY and the Blender scene hold the surface only and are not
affected.

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
winding with normals up. Base: two triangles over four bottom corner vertices
at the base plane, normal down. Walls: each side is one flat polygon, its top
vertices over the bottom corners under its two ends, cut into triangles with
no new vertex and normals outward, so that its bottom edge is a single edge
shared with the base (two triangles per boundary edge would leave T-junctions
along the base, which the edge check of section 6 refuses). Walking the
boundary counter-clockwise seen from above, the cut is a fan from the bottom
corner under the side's first top vertex over the side's top vertices, plus
one closing triangle to the next bottom corner, wherever that fan faces
outward all along. Where the terrain's edge hides a valley from that corner,
the fan would fold over itself and face inward, so the side is cut as a
monotone polygon instead: the fan reaches as far as the corner sees,
triangles between top vertices bridge each hidden valley, and a fan from the
next bottom corner closes the rest. Shared vertices along every seam, so the
mesh is closed and manifold by construction. Triangle count
2 (W - 1)(H - 1) + 2 (W + H) + 2 (12 for a 2 x 2 grid, a box).

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
direction (each directed edge, a 64-bit key in one sorted list, found once
and its reverse once, about 24 bytes per triangle); no triangle has zero
area; the signed volume is positive. The edge check runs up to 10,000,000
triangles; above that only the count, zero-area and volume checks run, and
the output says so. A failure is a bug and exits 1 with the counts. Blender's
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
  window mapping; the no-data fill (a sea strip beside a 0 m coast fills at
  0, a blank strip beside a 200 m plateau fills at 200, an inland hole takes
  the height of its nearest neighbour, a fully blank window is an error); the
  resampler properties of section 5.4; the solid's
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
  decides the streaming scheme. The dry run reads the window the same way,
because the height range, the base plane and the summit check need the
pixels; it writes nothing and never holds more than the real run would.

## 10. Decisions and who made them

The owner decided: generic input; optional crop, whole file by default; grid
size as the main control with full resolution as the alternative; quadratic
interpolation; two outputs from one run; the STL is a finished solid with
walls and base; base at sea level unless overridden; a vertical exaggeration
parameter; a second command in this repository; STL written in C++ directly
with Blender only checking it; Blender scene for camera-and-sun renders, no
textures yet.

The owner also chose, after the builder check: no-data filled from the
nearest real height.

My calls (change any of them): mesh in metres with the south-west corner at
the origin and `--scale` for print sizing; the base plane lowered
automatically with a small margin; a quadratic
B-spline (approximating) rather than an interpolating quadratic; the pixel
window slightly larger than the crop box; the 4 million warning and 25
million refusal; the 2-unit plinth; placing `dl_elevation` samples at pixel
corners; the dry run reading the window; refusing the south-oriented grids
and keeping only UTM in the code table; PLY as the hand-off format; the embedded script and the
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
