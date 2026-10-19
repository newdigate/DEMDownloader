# dem2mesh Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** A second command, `dem2mesh`, that turns an elevation GeoTIFF (optionally cropped, optionally resampled to a chosen grid) into a watertight STL solid for printing and a Blender scene ready to render.

**Architecture:** A static library `demmesh` in `src/mesh/` holds the pure parts in small units: a general GeoTIFF reader, Transverse Mercator and local-metres geometry, the crop window, the nearest-height fill, the quadratic B-spline resampler, the base-plane rule, the solid mesh with its self-check, and the STL and PLY writers. A `BlenderRunner` runs the user's Blender in the background with an embedded Python script that builds the scene from the PLY and checks the STL. `src/app/` adds a CLI11 parser and a thin `main` that strings the units together; the dry run uses the same path and writes nothing.

**Tech Stack:** C++20 on AppleClang 15, CMake 3.27+ with Ninja, libtiff and libgeotiff (already linked), CLI11 v2.7.2 and Catch2 v3.16.0 (already fetched), Blender 4.2+ at run time (the owner has 4.5.12 LTS, Intel build). No GDAL, no PROJ.

**Spec:** `docs/superpowers/specs/2026-10-19-dem-to-mesh-design.md`. Read it once before starting; each task names the sections it implements.

## Global Constraints

- C++20, `-Wall -Wextra` clean on AppleClang 15; no `std::format`, no `std::jthread`, no floating-point `std::from_chars`.
- No new third-party dependency: libtiff, libgeotiff, CLI11, Catch2 only. No GDAL, no PROJ, no network.
- CLI conventions of `src/app/Cli.cpp`: strict numeric values (the shared checks of Task 1), `error: ` prefix on every usage error, help text stating ranges, the program name `dem2mesh`, usage errors exit 1.
- Exit codes: 0 success, 1 any error (usage, file, mesh self-check, Blender missing or failed); reuse `demdl::cli::kExitComplete` and `kExitError` from `src/app/ExitCodes.h`.
- Catch2 test names never begin with `-` (ctest passes them to Catch2 as arguments). Tests touch no network.
- Mesh coordinates: metres, window south-west corner at the origin, x east, y north, z up; all three then multiplied by `--scale`.
- Memory: the pixel window is held once as float32 (1 GB for the City's whole file) plus the output grid; nothing else of that size.
- Output grid: refused above 25,000,000 vertices; a warning with expected file sizes above 4,000,000.
- Commits: one per task (or per step marked Commit), staged by name, never `git add -A`; message ends with the attribution line of the model doing the work (`Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>` as this plan is written).
- Build directory `build/` (`cmake --build build`; `ctest --test-dir build --output-on-failure`; one unit by tag, `./build/demdl_tests "[tm]"`).
- Never run `dl_elevation` against the real server during this work; never open the untracked `.tif` files in the repository root for writing. The City's file `~/Downloads/CCT_GLM2019_5m_DEM_1D_v1.tif` and its tiles may be read for manual checks only.

## Deviations from the spec (deliberate, small)

1. **Base and walls (spec 5.5).** The spec's base of two triangles under walls of two triangles per boundary edge leaves T-junctions along the base's edges, which fails the spec's own edge check (section 6). Each wall is instead the polygon bounded by that side's top vertices and its two bottom corners, triangulated as a monotone polygon (which equals a fan from the start corner wherever that fan is valid; a plain fan folds where a tall top vertex near its apex hides a lower one), and the base is two triangles over the four bottom corners. Triangle count becomes `2 (W - 1)(H - 1) + 2 (W + H) + 2` (12 for a 2 x 2 grid, a box). Task 7 updates the spec's formula and wording.
2. **Edge check at scale (spec 6).** The directed-edge check needs about 24 bytes per triangle; it runs in full up to 10,000,000 triangles and above that only the count, zero-area and signed-volume checks run, which the output says.
3. **Window extent.** "The mesh spans the window exactly" means from the first sample to the last: width `(cols - 1) x dx`, so at full resolution vertices coincide with samples.
4. **Reporting a projected file's window in degrees** needs an inverse projection; instead the run reports the window in projected metres and pixel indices, and echoes the crop box when one was given.

## Review Focus

Inputs the spec implies but no planned test would otherwise exercise; each has a test added to its owning task:

1. A GeoTIFF whose tiepoint is not at pixel (0, 0) (common in files cut from larger ones): the reader must still place samples correctly (Task 3).
2. A crop box entirely outside the file, or overlapping by a single row or column: a clear error, not a 1-vertex mesh or a crash (Task 4).
3. Output files already present without `--overwrite`, or a stem in a directory that does not exist: refused before the input is read, with the path in the message (Task 10).
4. A `--blender` path that does not exist: refused before anything is read, exit 1, nothing written. A `--blender` that exits non-zero: STL and PLY kept, Blender's output shown, exit 1 (Task 11). (Amended at the final review of 2026-10-19: a `--blender` that names no executable file is refused before anything is read; only a Blender that fails, or the automatic search finding none, keeps the STL and PLY.)
5. A rotated or sheared `ModelTransformation` GeoTIFF: refused with a message naming `gdalwarp`, never a silently skewed model (Task 3).

---

## File structure

| File | Responsibility |
|---|---|
| `src/app/CliChecks.h` | the strict-value checks shared by both CLIs (moved out of `Cli.cpp`) |
| `src/mesh/TransverseMercator.h/.cpp` | forward Transverse Mercator, UTM table |
| `src/mesh/GeoTiffGrid.h/.cpp` | inspect a GeoTIFF's geometry, read a pixel window as float32 with no-data as NaN |
| `src/mesh/CropWindow.h/.cpp` | crop box (degrees) to pixel window; window extent in metres (local metres for geographic files) |
| `src/mesh/NoDataFill.h/.cpp` | nearest-valid-height fill (breadth-first) |
| `src/mesh/Resampler.h/.cpp` | quadratic B-spline resampling |
| `src/mesh/BasePlane.h/.cpp` | the base-plane rule (sea level, lowering, margin, plinth) |
| `src/mesh/SolidMesh.h/.cpp` | surface and solid meshes, the self-check |
| `src/mesh/AtomicFileWriter.h/.cpp` | little-endian binary output through `<path>.tmp` and a rename after a checked close, for both writers |
| `src/mesh/StlWriter.h/.cpp`, `src/mesh/PlyWriter.h/.cpp` | binary writers, `.tmp` + rename |
| `src/mesh/BlenderRunner.h/.cpp`, `src/mesh/blender_scene.py`, `src/mesh/BlenderScenePy.h.in` | find and run Blender with the embedded script |
| `src/app/MeshCli.h/.cpp`, `src/app/dem2mesh_main.cpp` | argument parsing, the pipeline, the dry run |
| `tests/GeoTiffFixtures.h` | writes small geographic and Transverse Mercator GeoTIFFs for tests |
| `tests/SpawnedProcess.h` | the child-process helper extracted from `test_EndToEnd.cpp` |
| `tests/test_CliChecks.cpp`, `test_TransverseMercator.cpp`, `test_GeoTiffGrid.cpp`, `test_CropWindow.cpp`, `test_NoDataFill.cpp`, `test_Resampler.cpp`, `test_BasePlane.cpp`, `test_SolidMesh.cpp`, `test_MeshWriters.cpp`, `test_MeshCli.cpp`, `test_BlenderRunner.cpp`, `test_Dem2MeshEndToEnd.cpp` | one file per unit |

Namespace `demdl::mesh` for everything under `src/mesh/`; `demdl::cli` for `src/app/`.

---

### Task 1: Shared CLI value checks

Spec 3 (same conventions as `dl_elevation`). A pure move so `MeshCli.cpp` can reuse the checks.

**Files:**
- Create: `src/app/CliChecks.h`
- Modify: `src/app/Cli.cpp` (remove the anonymous-namespace copies, include the header)
- Test: `tests/test_CliChecks.cpp`; CMake test list

**Interfaces:**
- Produces, in `namespace demdl::cli`, all `inline` and `[[nodiscard]]`: `bool isPlainDecimal(const std::string&)`, `bool isPlainInteger(const std::string&)`, `std::string quoted(const std::string&)`, `std::string checkDecimal(const std::string&)`, `std::string checkWholeNumber(const std::string&)`, `std::string checkNotFlagLike(const std::string&)`, with the bodies and messages exactly as in `Cli.cpp` today.

- [ ] **Step 1: Write the failing test `tests/test_CliChecks.cpp`**, tag `[clichecks]`: `isPlainDecimal` accepts `12`, `12.`, `.5`, `-34.1845418`, `1e-3`, `+.5`; rejects ``, `1,5`, `0x1p3`, `nan`, `1e`, `12abc`. `isPlainInteger` accepts `0`, `7`, `-5`, `2048`; rejects `-0`, `0100`, `0x10`, `1_000`, `true`, ``. `quoted("a\rb")` is `'a\\rb'` (literal backslash r), `quoted("\xE2\x88\x92")` is `'\\xE2\\x88\\x92'`. `checkWholeNumber("0100")` ends with `, not '0100'`; `checkNotFlagLike("--x")` is `looks like a flag; is the value missing?`.
- [ ] **Step 2: Add the test to `demdl_tests` in `CMakeLists.txt`; build; expect** `fatal error: 'app/CliChecks.h' file not found`.
- [ ] **Step 3: Create `src/app/CliChecks.h`** with the six functions moved verbatim from `Cli.cpp`; delete them from `Cli.cpp` and include the header there. No behaviour change.
- [ ] **Step 4: Build; run** `./build/demdl_tests "[clichecks],[cli]"`; expect all passed, and `ctest --test-dir build` unchanged apart from the new cases.
- [ ] **Step 5: Commit** `refactor: shared CLI value checks in CliChecks.h`.

---

### Task 2: Transverse Mercator

Spec 4 (projected files), 5.1.

**Files:**
- Create: `src/mesh/TransverseMercator.h`, `src/mesh/TransverseMercator.cpp`
- Modify: `CMakeLists.txt` (new `add_library(demmesh STATIC ...)` linking `demdl` PUBLIC, `-Wall -Wextra`; `demdl_tests` links `demmesh`)
- Test: `tests/test_TransverseMercator.cpp`, tag `[tm]`

**Interfaces:**
- Produces: `struct TransverseMercatorParams { double originLongitudeDeg = 0, originLatitudeDeg = 0, scale = 1, falseEasting = 0, falseNorthing = 0, semiMajorAxis = 6378137.0, inverseFlattening = 298.257222101; }` (defaults are GRS 80); `struct Projected { double x, y; }`; `[[nodiscard]] Projected forwardTransverseMercator(const TransverseMercatorParams&, double lonDeg, double latDeg) noexcept`; `[[nodiscard]] std::optional<TransverseMercatorParams> utmFromEpsg(int code)` (32601..32660 north, 32701..32760 south: central meridian `-183 + 6 x zone`, scale 0.9996, false easting 500000, false northing 0 or 10000000, WGS 84 ellipsoid 6378137 / 298.257223563; anything else nullopt).

- [ ] **Step 1: Write the failing test.** Lo19 (defaults, origin longitude 19): `(18.44055175, -34.2111625310268)` gives `x -51556.5699`, `y -3787226.0192`; `(19.0, -34.0)` gives `x 0.0`, `y -3763661.4421`; `(18.3, -34.36)` gives `x -64395.8507`, `y -3803816.6955`; all within 0.002 m. UTM: `utmFromEpsg(32734)` has origin longitude 21, false northing 10000000; forward of `(18.4241, -33.9249)` gives `261881.5985, 6243182.3545` within 0.002 m; `utmFromEpsg(32634)` has false northing 0; `utmFromEpsg(2048)`, `utmFromEpsg(20482)`, `utmFromEpsg(32600)` and `utmFromEpsg(32761)` are nullopt. (Values from PROJ 9.4 `cs2cs`.)
- [ ] **Step 2: Add the library and test to CMake; build; expect** the missing-header error.
- [ ] **Step 3: Implement** the forward projection with the Krüger series (fourth order in n is enough for a millimetre within 3 degrees of the central meridian) or the USGS/Snyder series; angles in degrees at the interface, radians inside.
- [ ] **Step 4: Run** `./build/demdl_tests "[tm]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: Transverse Mercator forward projection and UTM table`.

---

### Task 3: GeoTIFF reader

Spec 4 (all of it), Review Focus 1 and 5.

**Files:**
- Create: `src/mesh/GeoTiffGrid.h`, `src/mesh/GeoTiffGrid.cpp`, `tests/GeoTiffFixtures.h`, `src/demdl/TiffSetup.h`, `src/demdl/TiffSetup.cpp`
- Modify: `CMakeLists.txt` (sources, test), `src/demdl/DemRaster.cpp` (uses `TiffSetup`)
- Test: `tests/test_GeoTiffGrid.cpp`, tag `[geotiff]`

**Interfaces:**
- Consumes: `TransverseMercatorParams`, `utmFromEpsg` (Task 2); `demdl::DemRaster::save` for the geographic fixture.
- Produces:
  - `enum class ModelKind { Geographic, TransverseMercator };`
  - `struct GridGeometry { ModelKind kind; TransverseMercatorParams tm; int width, height; double sampleX0, sampleY0; double dx, dy; bool pixelIsPoint; bool dlElevation; std::optional<double> noData; std::string description; }` where `sampleX0, sampleY0` is the position of sample (0, 0) and `dx, dy` are positive steps (x grows with column, y falls with row); member functions `[[nodiscard]] double sampleX(int col) const` (`sampleX0 + col * dx`) and `sampleY(int row) const` (`sampleY0 - row * dy`).
  - `struct PixelWindow { int col0, row0, cols, rows; }`.
  - `[[nodiscard]] GridGeometry inspectGeoTiff(const std::filesystem::path&)`: throws `std::runtime_error` with user-facing messages (file names in them): cannot open; not single-band; sample format not float32, float64, int16, int32 or uint16; rotated or sheared `ModelTransformation` ("…is rotated or sheared; reproject it first, for example with gdalwarp"); no tiepoint and no transformation; no pixel scale; model type neither geographic nor projected; projected but not Transverse Mercator ("…uses a projection this tool does not read (GeoKey ProjCoordTrans N); reproject it first…"); `CT_TransvMercator_SouthOriented` and EPSG 2046..2055 ("…is south-oriented…"); projected with only an EPSG code that `utmFromEpsg` does not know ("…names EPSG:N without parameters…").
  - `[[nodiscard]] std::vector<float> readWindow(const std::filesystem::path&, const GridGeometry&, const PixelWindow&)`: row-major, `cols x rows`, converted to float, every no-data value and every NaN as NaN; strips and tiles; throws on read errors.
- Geometry rules (spec 4): sample (0, 0) is the tiepoint's model coordinate adjusted for the tiepoint's raster position (a tiepoint at raster `(i, j)` means sample (0, 0) is `i` pixels west and `j` pixels north of it); for `RasterPixelIsArea` files add half a pixel east and south, except when `description` starts with `dl_elevation;` (samples at the north-west corners); `RasterPixelIsPoint` adds nothing. No-data from tag 42113 parsed with `strtod`. Geographic ellipsoid from `GeogSemiMajorAxisGeoKey`/`GeogInvFlatteningGeoKey`, else `GeogEllipsoidGeoKey` 7019 (GRS 80) or 7030 (WGS 84), else the GRS 80 defaults. Transverse Mercator parameters from GeoKeys 3080, 3081, 3092, 3082, 3083 when `ProjCoordTransGeoKey` is 1; when the file gives only `ProjectedCSTypeGeoKey`, from `utmFromEpsg`.
- `tests/GeoTiffFixtures.h` produces: `writeGeographicFixture(dir, width, height, box, heights)` via `DemRaster` (so the description says `dl_elevation;`); `writeTmFixture(dir, width, height, params, tiepointRaster{i,j}, tiepointModel{x,y}, dx, dy, pixelIsPoint, noData, heights, Layout{Strips|Tiles}, SampleKind{Float32|Int16})` writing the GeoKeys with libgeotiff (`GTIFKeySet`) and the tags with libtiff (tiles via `TIFFWriteTile`); `writeRotatedFixture(dir)` with a `ModelTransformation` whose off-diagonal terms are non-zero; `writeSouthOrientedFixture(dir)` with `ProjectedCSTypeGeoKey` 2048 and no parameters.

- [ ] **Step 1: Write the failing test.** Cases: a geographic 4 x 3 dl_elevation file at box north -34.18, south -34.22, east 18.47, west 18.40 gives `kind Geographic`, `pixelIsPoint false`, `dlElevation true`, `sampleX0 18.40`, `sampleY0 -34.18` (corners, not centres), `dx 0.0175`, `dy 0.04/3`, `noData -32768`; a TM point fixture with the City's numbers (tiepoint raster (0,0) at `-64012.5, -3705051.806`, dx = dy = 5, no-data 0, params origin longitude 19) gives `kind TransverseMercator`, `tm.originLongitudeDeg 19`, `sampleX0 -64012.5`; a TM area fixture with the tiepoint at raster (2, 1) and model `(1000, 2000)`, dx 10, dy 20, gives `sampleX0 1000 - 2*10 + 5 = 985`, `sampleY0 2000 + 1*20 - 10 = 2010`; `readWindow` of the full TM strips fixture returns the written heights with the no-data pixel as NaN; the same for the tiled variant and for the int16 variant (values converted); `readWindow` of `{col0 1, row0 1, cols 2, rows 2}` returns the right 2 x 2 block; the rotated fixture makes `inspectGeoTiff` throw a message containing `gdalwarp`; the south-oriented fixture throws a message containing `south-oriented`; a float64 fixture reads; a UTM fixture with only `ProjectedCSTypeGeoKey 32734` gives `tm.falseNorthing 10000000`.
- [ ] **Step 2: Build; expect** the missing-header error.
- [ ] **Step 3: Implement** `GeoTiffGrid.cpp` with libtiff/libgeotiff (reuse the tag-registration approach of `DemRaster.cpp`: call a shared `ensureTagsRegistered` by moving that function and the error capture into a new `src/demdl/TiffSetup.h/.cpp` exported from `demdl`, with `DemRaster.cpp` using it; the GDAL no-data tag must be registered before the file is opened).
- [ ] **Step 4: Run** `./build/demdl_tests "[geotiff],[demraster]"`; expect all passed.
- [ ] **Step 5: Manual check (read-only):** a scratch call of `inspectGeoTiff` on `~/Downloads/CCT_GLM2019_5m_DEM_1D_v1.tif` prints TransverseMercator, 12912 x 19696, sampleX0 -64012.5, sampleY0 -3705051.806, point, no-data 0. Record the output in the commit message.
- [ ] **Step 6: Commit** `feat: general GeoTIFF reader for geographic and Transverse Mercator elevation files`.

---

### Task 4: Crop window and metres

Spec 5.1, 5.2, Review Focus 2.

**Files:**
- Create: `src/mesh/CropWindow.h`, `src/mesh/CropWindow.cpp`
- Test: `tests/test_CropWindow.cpp`, tag `[crop]`

**Interfaces:**
- Consumes: `GridGeometry`, `PixelWindow` (Task 3), `forwardTransverseMercator` (Task 2), `demdl::BoundingBox`.
- Produces:
  - `[[nodiscard]] PixelWindow pixelWindowFor(const GridGeometry&, const std::optional<BoundingBox>& crop)`: nullopt gives the whole file. Geographic: fractional index `f = (lon - sampleX0) / dx`, clamped as a double to `[-1, width]` first, then `col0 = max(0, floor(f_west + 1e-6))`, `colLast = min(width - 1, ceil(f_east - 1e-6))` (the 1e-6-pixel snap keeps an edge that lands on a sample in double arithmetic from spilling one pixel), rows likewise with `g = (sampleY0 - lat) / dy` (north gives `row0`); TM: project the box's four corners and four edge midpoints, take their bounding rectangle, then the same. Throws `std::runtime_error("the crop window overlaps the file by fewer than 2 x 2 samples")` when `colLast - col0 < 1` or `rowLast - row0 < 1` (including no overlap at all; clamp first, then test).
  - `struct MetresPerDegree { double perDegreeLatitude, perDegreeLongitude; }`; `[[nodiscard]] MetresPerDegree metresPerDegree(double latDeg) noexcept` with the spec's formulas.
  - `struct WindowExtent { double widthM, heightM, cellDxM, cellDyM; }`; `[[nodiscard]] WindowExtent extentInMetres(const GridGeometry&, const PixelWindow&)`: TM: `cellDxM = dx`, `cellDyM = dy`; geographic: `dx`/`dy` times the metres per degree at the window's middle latitude; `widthM = (cols - 1) * cellDxM`, `heightM = (rows - 1) * cellDyM`.
  - `[[nodiscard]] std::string describeWindow(const GridGeometry&, const PixelWindow&)`: one line for the dry run: pixels `cols x rows at (col0, row0)`, then for geographic files the sample extent in degrees, for TM files in projected metres.

- [ ] **Step 1: Write the failing test.** `metresPerDegree(-34.2)` is `110926.0` latitude and `92167.8` longitude, each within 0.5 m. Geographic 100 x 80 grid, point samples from `sampleX0 18.0`, `dx 0.01`, `sampleY0 -33.0`, `dy 0.01`: crop north -33.105, south -33.395, east 18.505, west 18.205 gives `col0 20, cols 32` (20..51), `row0 10, rows 31` (10..40); no crop gives the whole grid; a crop west of the file (east 17.0) throws the fewer-than-2x2 message; a crop covering one column (west 18.2, east 18.2) throws it too; a crop beyond the east edge is clamped to `width - 1`. TM: the City geometry with crop north -34.1845418, south -34.2163447, east 18.4625244, west 18.4185791 gives a window containing pixel `(2491, 16435)` (the recorded land point) and `cols` between 800 and 830 (about 4.0 km at 5 m). `extentInMetres` of the TM window is `(cols - 1) * 5` wide; of the geographic 32 x 31 window `widthM = 31 * 0.01 * perDegreeLongitude(-33.25)` within 0.01 m.
- [ ] **Step 2: Build; expect** the missing-header error.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `./build/demdl_tests "[crop]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: crop box to pixel window; window extent in metres`.

---

### Task 5: Nearest-height fill

Spec 5.3 (the owner's choice).

**Files:**
- Create: `src/mesh/NoDataFill.h`, `src/mesh/NoDataFill.cpp`
- Test: `tests/test_NoDataFill.cpp`, tag `[fill]`

**Interfaces:**
- Produces: `struct FillReport { std::uint64_t filled = 0; float lowestFill = 0, highestFill = 0; }`; `[[nodiscard]] FillReport fillNearest(std::vector<float>& values, int cols, int rows)`: every NaN takes the height of the nearest non-NaN pixel in 4-connected steps (multi-source breadth-first search: the first frontier is every valid pixel with a NaN neighbour; each wave copies its own value into NaN 4-neighbours, in place, so a filled pixel carries its origin's height); ties resolved by scan order within a wave (deterministic). Throws `std::runtime_error("the window has no elevation value at all")` when everything is NaN. Memory: the vector plus two frontier index vectors (`std::uint32_t`).

- [ ] **Step 1: Write the failing test.** A 6 x 1 row `[NaN, NaN, 0, 5, NaN, 200]` becomes `[0, 0, 0, 5, 5, 200]` (pixel 4 is one step from both 5 and 200; scan order within the wave fills it from the lower index), `filled 3`, `lowestFill 0`, `highestFill 5`. A 5 x 5 grid with a valid ring (value 7) and a NaN centre 3 x 3 fills all 9 with 7. A coast strip: left half 0, right half NaN on 4 x 2 fills with 0. A boundary plateau: left half 200, right half NaN fills with 200. An all-NaN 3 x 3 throws. A grid with no NaN returns `filled 0` and is unchanged. Determinism: the same input twice gives identical output.
- [ ] **Step 2: Build; expect** the missing-header error.
- [ ] **Step 3: Implement.**
- [ ] **Step 4: Run** `./build/demdl_tests "[fill]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: nearest-height fill for no-data pixels`.

---

### Task 6: Quadratic resampler

Spec 5.4.

**Files:**
- Create: `src/mesh/Resampler.h`, `src/mesh/Resampler.cpp`
- Test: `tests/test_Resampler.cpp`, tag `[resample]`

**Interfaces:**
- Produces: `[[nodiscard]] double quadraticBSpline(double t) noexcept` (`3/4 - t^2` for `|t| < 1/2`, `(3/2 - |t|)^2 / 2` for `1/2 <= |t| < 3/2`, else 0); `[[nodiscard]] std::vector<float> resampleQuadratic(const std::vector<float>& src, int srcCols, int srcRows, int outCols, int outRows)`.
- Definition (pin exactly): output column `i` of `outCols` maps to source coordinate `u_i = i * (srcCols - 1) / (outCols - 1)` (so the first and last outputs sit on the first and last samples; `outCols == 1` is refused, callers guarantee at least 2); stretch `s = max(1, (srcCols - 1) / (outCols - 1))`; weight of source column `k` is `quadraticBSpline((k - u_i) / s)` for every `k` with `|k - u_i| < 1.5 s`; taps that fall outside `[0, srcCols - 1]` use odd reflection about the edge sample (`v(-k) = 2 v(0) - v(k)`, and the same past the last sample), so a plane stays a plane at the border; weights are normalised to sum 1. Separable: horizontal pass over every source row into `outCols`, then vertical pass into `outRows` with the same rule. Input must contain no NaN (assert in debug builds).

- [ ] **Step 1: Write the failing test.** `quadraticBSpline` at 0, 0.5, 1, 1.5 is 0.75, 0.5, 0.125, 0; symmetric; the three integer taps sum to 1. Constant 7.5 on 10 x 10 to 4 x 3 is all 7.5 within 1e-6. Plane `z = 2 x + 3 y` (x, y source indices) on 20 x 15 to 7 x 5 equals `2 u_i + 3 v_j` within 0.05 at every output vertex, including the corners (a half-integer stretch leaves the sampled kernel's first moment inexact by about a part in a thousand); the same to 20 x 15 (same size) within 1e-4; and to 41 x 29 (upsampling) within 0.01. Range: a random 30 x 30 grid in [0, 1000] to 9 x 7 has no output outside [min, max] of the source. Smoothing: a single 100 m spike in a 9 x 9 field of 0 resampled to 9 x 9 gives a centre below 100 and above 50 (0.75^2 x 100 = 56.25 within 0.01) with the 8 neighbours above 0. Down-sampling a 2048 x 2 grid of alternating 0/100 columns to 64 x 2 gives outputs 1 to 62 within 10 of 50 (antialiasing), while outputs 0 and 63 equal the end samples 0 and 100 (odd reflection pins the outermost outputs to the outermost samples).
- [ ] **Step 2: Build; expect** the missing-header error.
- [ ] **Step 3: Implement** with precomputed per-output tap tables for each axis.
- [ ] **Step 4: Run** `./build/demdl_tests "[resample]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: separable quadratic B-spline resampler with antialiasing`.

---

### Task 7: Base plane and solid mesh

Spec 5.3 (base plane, plinth, clamp), 5.5, 6, deviations 1 and 2.

**Files:**
- Create: `src/mesh/BasePlane.h`, `src/mesh/BasePlane.cpp`, `src/mesh/SolidMesh.h`, `src/mesh/SolidMesh.cpp`
- Modify: spec section 5.5 and 6 (deviations 1 and 2: the fan walls, the triangle formula, the 10,000,000 limit)
- Test: `tests/test_BasePlane.cpp` tag `[baseplane]`, `tests/test_SolidMesh.cpp` tag `[solid]`

**Interfaces:**
- Produces (BasePlane): `struct BasePlane { double levelM; double planeM; double planeZ; bool loweredToLowest; bool plinthApplied; }`; `[[nodiscard]] BasePlane chooseBasePlane(double lowestM, double highestM, std::optional<double> requestedM, double zScale, double scale, double plinth) noexcept` with: `levelM = requested or 0`; if `levelM > lowestM` then `levelM = lowestM`, `loweredToLowest`; `marginM = max(1.0, 0.01 * (highestM - lowestM))`; `planeM = levelM - marginM`; `planeZ = planeM * zScale * scale`; if `lowestM * zScale * scale - planeZ < plinth` then `planeZ = lowestM * zScale * scale - plinth`, `plinthApplied` (then `planeM = planeZ / (zScale * scale)`).
- Produces (SolidMesh): `struct Vec3 { float x, y, z; }`; `struct Triangle { std::uint32_t a, b, c; }`; `struct Mesh { std::vector<Vec3> vertices; std::vector<Triangle> triangles; }`; `[[nodiscard]] Mesh surfaceMesh(const std::vector<float>& heights, int cols, int rows, double widthM, double heightM, double zScale, double scale, double levelM)`: vertex `(i, j)` (row 0 north) at `x = i * widthM / (cols - 1) * scale`, `y = (rows - 1 - j) * heightM / (rows - 1) * scale`, `z = max(h, levelM) * zScale * scale`; two triangles per cell, counter-clockwise seen from above (normal +z); `[[nodiscard]] Mesh solidMesh(const Mesh& surface, int cols, int rows, float planeZ)`: the surface plus four bottom corner vertices at `planeZ`, each wall the monotone polygon of that side's top vertices and its two bottom corners triangulated without folds (walking the boundary counter-clockwise seen from above), normals outward; base two triangles over the four bottom corners, normal down; triangle count `2 (cols - 1)(rows - 1) + 2 (cols + rows) + 2`; `[[nodiscard]] std::uint64_t solidTriangleCount(int cols, int rows) noexcept`; `struct MeshCheck { std::uint64_t triangles = 0, nonManifoldEdges = 0, zeroAreaTriangles = 0; double signedVolume = 0; bool edgesChecked = true; [[nodiscard]] bool ok() const; }` (`ok` is no non-manifold edges when checked, no zero-area triangles, positive volume); `[[nodiscard]] MeshCheck checkSolid(const Mesh&)`: directed-edge check (each directed edge once and its reverse once, via a sorted vector of 64-bit keys) when `triangles <= 10,000,000`, else `edgesChecked = false`; zero-area by the cross product in double; signed volume by the divergence theorem in double.

- [ ] **Step 1: Write the failing BasePlane test.** Table Mountain: `chooseBasePlane(0, 1086, nullopt, 1, 0.02, 2)` gives `levelM 0`, `planeM -10.86` before the plinth, `plinthApplied true`, `planeZ -2.0` within 1e-9, `loweredToLowest false`. City whole file: `(-37.9, 1590.1, nullopt, 1, 1, 2)` gives `loweredToLowest true`, `levelM -37.9`, `planeM -37.9 - 16.28 = -54.18` within 0.01, `plinthApplied false`, `planeZ == planeM`. Request above the terrain: `(50, 300, 100, 1, 1, 2)` lowers to 50. Request below: `(0, 300, -100, 1, 1, 2)` keeps `levelM -100`, margin 3, `planeM -103`. Flat window: `(10, 10, nullopt, 1, 1, 2)` gives margin 1, `planeM 9` before the plinth, then `planeZ 8` by the plinth. `zScale 3, scale 0.02` scales `planeZ` by 0.06.
- [ ] **Step 2: Write the failing SolidMesh test.** 2 x 2 grid, heights all 10, width 100, height 50, no scaling, level 0: surface has 4 vertices at `(0,50,10) (100,50,10) (0,0,10) (100,0,10)` (row 0 north), 2 triangles with +z normals; `solidMesh(..., planeZ -5)` has 8 vertices and `solidTriangleCount(2,2) == 12` triangles; `checkSolid` reports 0 non-manifold edges, 0 zero-area triangles, signed volume `100 * 50 * 15 = 75000` within 1e-3. 5 x 4 grid with heights `10 + i + 2 j`: triangle count `2*4*3 + 2*9 + 2 = 44`; `checkSolid` ok; volume positive. A deliberately broken mesh (one triangle's winding reversed) has non-manifold edges > 0 and `ok() false`. `checkSolid` on a mesh with a zero-area triangle reports it. Clamp: a height below `levelM` yields `z == levelM * zScale * scale`. Mirror check: on a 3 x 2 grid with height 100 at `(col 2, row 0)` (north-east) and 0 elsewhere, the vertex with the largest x and largest y has z 100.
- [ ] **Step 3: Build; expect** the missing-header errors.
- [ ] **Step 4: Implement** `BasePlane.cpp` and `SolidMesh.cpp`.
- [ ] **Step 5: Run** `./build/demdl_tests "[baseplane],[solid]"`; expect all passed.
- [ ] **Step 6: Update the spec** (sections 5.5 and 6) to deviations 1 and 2 in one short paragraph each.
- [ ] **Step 7: Commit** `feat: base-plane rule and watertight solid mesh with self-check`.

---

### Task 8: STL and PLY writers

Spec 5.5 (file layouts), 7.3 (UVs in the PLY).

**Files:**
- Create: `src/mesh/StlWriter.h/.cpp`, `src/mesh/PlyWriter.h/.cpp`
- Test: `tests/test_MeshWriters.cpp`, tag `[writers]`

**Interfaces:**
- Consumes: `Mesh`, `Vec3`, `Triangle` (Task 7).
- Produces: `void writeBinaryStl(const Mesh&, const std::filesystem::path&, std::string_view header)`: 80-byte header (`header` truncated or zero-padded), `uint32` triangle count, per triangle the unit normal from the vertex cross product (zero vector for a degenerate triangle) then the three vertices, `uint16` 0; little-endian; written to `path + ".tmp"` and renamed when complete (a failed write removes the `.tmp`); throws `std::runtime_error` with the path and the OS reason. `void writeBinaryPly(const Mesh& surface, const std::filesystem::path&, double widthUnits, double heightUnits)`: header exactly `ply`, `format binary_little_endian 1.0`, `comment dem2mesh <version>`, `element vertex N`, `property float x`, `property float y`, `property float z`, `property float s`, `property float t`, `element face M`, `property list uchar uint vertex_indices`, `end_header`, each line ending in `\n`; `s = x / widthUnits`, `t = y / heightUnits`; faces as `uchar 3` plus three `uint32`; same `.tmp` and rename rule. `[[nodiscard]] Mesh readBinaryStl(const std::filesystem::path&)` for tests and the end-to-end check: identical positions are merged into one vertex, so `checkSolid` can run on the result.

- [ ] **Step 1: Write the failing test.** STL of the 2 x 2 solid from Task 7's test: file size `84 + 50 * 12`; bytes 80..83 hold 12 little-endian; the first triangle's normal is a unit vector; `readBinaryStl` returns 8 vertices and 12 triangles whose `checkSolid` is ok with volume 75000. A header longer than 80 bytes is truncated; shorter is zero-padded. Writing into a missing directory throws with the path in the message and leaves no `.tmp`. PLY of the 2 x 2 surface: the header lines exactly as listed; vertex 1 (north-east) has `s 1, t 1`; vertex 2 (south-west) has `s 0, t 0`; file size equals header bytes + `4 * 20` + `2 * 13`.
- [ ] **Step 2: Build; expect** the missing-header error.
- [ ] **Step 3: Implement** both writers with `std::ofstream` in binary mode and explicit little-endian byte writes (`std::bit_cast` to `uint32_t`, then four bytes), so the layout does not depend on the host.
- [ ] **Step 4: Run** `./build/demdl_tests "[writers]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: binary STL and PLY writers`.

---

### Task 9: Command-line parsing

Spec 3 (the table and validation), same conventions as `Cli.cpp`.

**Files:**
- Create: `src/app/MeshCli.h`, `src/app/MeshCli.cpp`
- Modify: `CMakeLists.txt` (`add_library(demmesh_cli OBJECT src/app/MeshCli.cpp)` linking `demmesh` PUBLIC and `CLI11::CLI11` PRIVATE; `demdl_tests` links it)
- Test: `tests/test_MeshCli.cpp`, tag `[meshcli]`

**Interfaces:**
- Consumes: `CliChecks.h` (Task 1), `demdl::cli::ParseOutcome` (`Cli.h`), `demdl::BoundingBox`.
- Produces: `struct GridSize { int cols = 0; int rows = 0; }` (`rows 0` means "from the aspect ratio"); `struct MeshOptions { std::filesystem::path input; std::filesystem::path outputStem; std::optional<BoundingBox> crop; std::optional<GridSize> grid; double zScale = 1; std::optional<double> base; double scale = 1; double plinth = 2; bool noBlend = false; std::optional<std::filesystem::path> blender; bool overwrite = false; bool dryRun = false; bool verbose = false; }`; `[[nodiscard]] ParseOutcome parseMeshArgs(int argc, const char* const* argv, MeshOptions&, std::ostream& out, std::ostream& err)` with the flags of the spec's table (`-i,--input` and `-o,--output` required; `--grid` as text `W` or `WxH`, lowercase x, each part a plain whole number; the four crop flags via `CLI::Option::needs` so all four come together); `[[nodiscard]] std::string validateMeshOptions(const MeshOptions&)` returning the first problem in this order, each phrased for the user: crop edges finite and in range with north > south and east > west; grid parts at least 2; `--z-scale` and `--scale` positive and finite; `--base` finite; `--plinth` zero or positive and finite. (Existence of the input and the output directory are checked by `main`, Task 10, so this function stays pure.) Help text: `--north` etc. "crop window edge, WGS84 decimal degrees (all four or none)"; `--grid` "output grid in vertices, W or WxH (at least 2; without xH the height follows the window's aspect)"; `--z-scale` "vertical exaggeration (default 1)"; `--base` "base plane height in metres before exaggeration (default sea level, 0; lowered to the lowest point when needed)"; `--scale` "model units per metre in the STL and PLY (0.02 makes a 10 km window 200 mm wide)"; `--plinth` "minimum thickness under the lowest point, model units after scaling (default 2)"; `--no-blend`, `--blender PATH`, `--overwrite`, `--dry-run`, `--verbose` as in the spec.

- [ ] **Step 1: Write the failing test** (helper like `test_Cli.cpp`'s `parse`, program name `dem2mesh`): the example line `-i in.tif -o out --north -33.9 --south -34.0 --east 18.5 --west 18.3 --grid 400x300 --z-scale 2 --base -5 --scale 0.02 --plinth 3 --no-blend --blender /x/blender --overwrite --dry-run --verbose` fills every field; `--grid 400` gives `rows 0`; `--grid 400x` and `--grid 0400x300` and `--grid 400X300` are usage errors naming `--grid`; three of the four crop flags is a usage error naming the missing one; `--help` lists `--plinth` and exits 0 with `dem2mesh [OPTIONS]`; `--version` prints the project version; `--z-scale 0` parses but `validateMeshOptions` says `z-scale must be positive`; a crop with north < south says `north must be greater than south`; `--grid 1x5` parses and validate says `grid must be at least 2 x 2`; numeric strictness: `--scale 0,02` and `--north ""` are usage errors with the shared messages.
- [ ] **Step 2: Build; expect** the missing-header error.
- [ ] **Step 3: Implement** `MeshCli.cpp` mirroring `Cli.cpp`'s structure.
- [ ] **Step 4: Run** `./build/demdl_tests "[meshcli]"`; expect all passed.
- [ ] **Step 5: Commit** `feat: dem2mesh command-line parsing`.

---

### Task 10: The executable and its dry run

Spec 3.1, 5 (the pipeline order), 6, 9 (memory), Review Focus 3. Blender comes in Task 11; until then `main` behaves as if `--no-blend` were always given and prints `blender: skipped (not built yet)`.

**Files:**
- Create: `src/app/dem2mesh_main.cpp`, `tests/SpawnedProcess.h`, `tests/test_Dem2MeshEndToEnd.cpp`
- Modify: `CMakeLists.txt` (`add_executable(dem2mesh src/app/dem2mesh_main.cpp)` linking `demmesh demmesh_cli`; `add_dependencies(demdl_tests dem2mesh)`; `target_compile_definitions(demdl_tests PRIVATE DEM2MESH_EXE="$<TARGET_FILE:dem2mesh>")`), `tests/test_EndToEnd.cpp` (include the extracted header)
- Test: `tests/test_Dem2MeshEndToEnd.cpp`, tag `[mesh-e2e]`

**Interfaces:**
- Consumes: everything from Tasks 3 to 9.
- Produces: the `dem2mesh` executable. `tests/SpawnedProcess.h`: the `Finished`, `Child` (constructor takes the executable path as its first argument), `readFile`, `writeFile`, `fileHolds`, `describeStatus` and `occurrences` helpers moved verbatim from `test_EndToEnd.cpp`, which then includes the header (its `Child` calls pass `kExecutable`); behaviour unchanged.
- `main` order (spec 9 style, every failure `error: <message>` on stderr, exit 1): parse; validate; input exists and is a regular file; output directory exists and is writable (`access(W_OK | X_OK)`); without `--overwrite` none of `STEM.stl`, `STEM.ply`, `STEM.blend` exists ("…exists; use --overwrite"); `inspectGeoTiff`; `pixelWindowFor`; `extentInMetres`; output grid (`--grid` or full resolution; `rows 0` gives `rows = max(2, round((cols - 1) * heightM / widthM) + 1)`); refuse above 25,000,000 vertices ("the output grid is N vertices; the limit is 25000000: use --grid or crop"), warn above 4,000,000 with the STL and PLY sizes; `readWindow`; `fillNearest`; lowest and highest; `resampleQuadratic` (always, also at full resolution: the 3-tap kernel is the spec's smoothing); summit before and after (max of window, max of grid); `chooseBasePlane`; `surfaceMesh`; `solidMesh`; `checkSolid` (failure is `error: mesh self-check failed: …`, exit 1, nothing written); write PLY then STL; Blender (Task 11); print a summary. The dry run stops after `checkSolid` and prints the plan of spec 3.1 instead of writing; it starts with the line `dry run`.
- Summary lines (stdout, one each): `input: <path>, <kind>, <width> x <height> samples at <dx> x <dy>`, `window: <describeWindow>`, `grid: <cols> x <rows> vertices, cell <cellDx> x <cellDy> m`, `heights: <lowest> to <highest> m (<filled> no-data pixels filled with <lowestFill> to <highestFill> m)`, `summit: <before> m in the window, <after> m in the mesh`, `base: <planeM> m (<reason>)` with reason among `sea level minus a <margin> m margin`, `lowered to the lowest point, <lowest> m, minus …`, `… then pushed down for a <plinth>-unit plinth`, `solid: <triangles> triangles, STL <bytes> bytes, PLY <bytes> bytes`, `wrote: <stl>`, `wrote: <ply>`, `blender: …`.

- [ ] **Step 1: Extract `tests/SpawnedProcess.h`** from `test_EndToEnd.cpp` (move, do not rewrite); run `./build/demdl_tests "[e2e]"`; expect all passed as before.
- [ ] **Step 2: Write the failing end-to-end test.** A geographic 8 x 6 dl_elevation fixture (box north -34.18, south -34.22, east 18.47, west 18.40) with heights `10 + 5 i + 3 j` and `kNoData` at `(7, 0)` and `(7, 1)`. Cases, all with `--no-blend`:
  - `--dry-run`: exit 0, stdout starts with `dry run`, contains `grid: 8 x 6 vertices` and `2 no-data pixels filled`, no files created.
  - full resolution: exit 0; `STEM.stl` and `STEM.ply` exist and no `.tmp`; `readBinaryStl` gives `solidTriangleCount(8, 6) == 2*7*5 + 2*14 + 2 = 100` triangles and `checkSolid` ok; the STL's bounding box is `[0, widthM] x [0, heightM]` within 0.01, with `widthM` and `heightM` taken from `extentInMetres` for the same geometry and window (the test calls it rather than repeating the formula); max z is 60 within 0.01 (the heights are a plane, which the resampler reproduces, and the two filled pixels lie in the north-east rows, away from the south-east maximum at `(7, 5)`); min z equals the base plane reported in stdout; stdout has `wrote:` twice and `blender: skipped`.
  - `--grid 4`: exit 0, stdout `grid: 4 x 3 vertices` (rows from the aspect), 2*3*2 + 2*7 + 2 = 28 triangles.
  - `--grid 4x4 --scale 0.01 --z-scale 2`: the STL's x extent is `0.01 * widthM` within 1e-4 and z extent at least `2 * 0.01 * (60 - base)`.
  - crop `--north -34.19 --south -34.21 --east 18.45 --west 18.42`: exit 0 and stdout's `window:` line shows fewer than 8 columns; crop `--north -30 --south -31 --east 18.45 --west 18.42` (outside): exit 1, stderr `error: the crop window overlaps the file by fewer than 2 x 2 samples`.
  - existing output: a second run without `--overwrite` exits 1 with `exists; use --overwrite` and leaves both files byte-identical; with `--overwrite` exits 0.
  - stem in a missing directory: exit 1, stderr contains the directory and `cannot write`.
  - `--grid 6000x5000` (30,000,000 vertices): exit 1 with the limit message, nothing written.
  - the TM point fixture from `GeoTiffFixtures.h` (12 x 10, dx 5) at full resolution: exit 0, `window:` in metres, triangle count `solidTriangleCount(12, 10)`.
- [ ] **Step 3: Build; expect** `Cannot find source file: src/app/dem2mesh_main.cpp`.
- [ ] **Step 4: Implement `dem2mesh_main.cpp`** in the order above; keep it a sequence of named steps calling the library; `--verbose` adds a timing line per stage.
- [ ] **Step 5: Run** `./build/demdl_tests "[mesh-e2e]"` and the full ctest; expect all passed. Run `[mesh-e2e]` ten times.
- [ ] **Step 6: Manual check (read-only input, output into a scratch folder):** `dem2mesh -i ~/Downloads/CCT_GLM2019_5m_DEM_1D_v1_tiles/CCT_GLM2019_5m_DEM_1D_v1_r3_c1.tif -o <scratch>/r3c1 --grid 400 --scale 0.02 --no-blend` exits 0 in under a minute; record the summary in the commit message; then the same with `--north -33.9459 --south -33.9800 --east 18.4300 --west 18.3900` (Table Mountain) `--grid 500` and open the STL in a viewer if one is at hand (not required).
- [ ] **Step 7: Commit** `feat: dem2mesh executable with dry run and end-to-end tests`.

---

### Task 11: Blender scene and STL check

Spec 7 (all of it), Review Focus 4.

**Files:**
- Create: `src/mesh/blender_scene.py`, `src/mesh/BlenderScenePy.h.in`, `src/mesh/BlenderRunner.h`, `src/mesh/BlenderRunner.cpp`
- Modify: `CMakeLists.txt` (embed the script: `file(READ src/mesh/blender_scene.py BLENDER_SCENE_PY)`, `configure_file(src/mesh/BlenderScenePy.h.in ${CMAKE_BINARY_DIR}/generated/mesh/BlenderScenePy.h @ONLY)`, `set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/mesh/blender_scene.py)`, `target_include_directories(demmesh PRIVATE ${CMAKE_BINARY_DIR}/generated)`), `src/app/dem2mesh_main.cpp` (call the runner), `tests/test_Dem2MeshEndToEnd.cpp` (the Blender case)
- Test: `tests/test_BlenderRunner.cpp` tag `[blender]`; the end-to-end case

**Interfaces:**
- Produces: `[[nodiscard]] std::optional<std::filesystem::path> findBlender(const std::optional<std::filesystem::path>& flag)`: `flag` if given (returned as is; existence is checked by the run), else `$BLENDER` when set and non-empty, else `/Applications/Blender.app/Contents/MacOS/Blender` when it exists, else the first executable `blender` on `PATH`, else nullopt. `struct BlenderJob { std::filesystem::path ply, stl, blend; std::string name; double widthUnits, heightUnits, minZ, maxZ, scale; }`; `struct BlenderResult { int exitCode; std::string output; }`; `[[nodiscard]] BlenderResult runBlender(const std::filesystem::path& blender, const BlenderJob&)`: writes the embedded script (`kBlenderScenePy` from the generated header) to a temporary file, runs `<blender> --background --factory-startup --python <script> -- --ply … --stl … --blend … --name … --width … --height … --zmin … --zmax … --scale …` with `posix_spawn`, stdout and stderr captured into `output`, deletes the script, returns the exit code (127 and a message in `output` when the executable cannot be started).
- The script (`blender_scene.py`, Python 3.11, Blender 4.2+): refuses `bpy.app.version < (4, 2, 0)` with a message; parses its arguments after `--` with `argparse`; the nine steps of spec 7.2 with these pins: `bpy.ops.wm.ply_import(filepath, forward_axis='Y', up_axis='Z')`; the object named `name`; `shade_smooth`; material `Terrain`, Principled BSDF base colour `(0.55, 0.50, 0.45, 1)`, roughness 0.9, plus an unconnected `ShaderNodeTexImage` named `AerialPhoto` wired to nothing (the UV map comes from the PLY's `s`,`t`); a Sun `strength 3`, rotation so it shines from the north-west at 40 degrees elevation; a camera, 50 mm, south-east of the centre at 35 degrees elevation, distance so the whole window fits with a 5 percent margin (compute from the window's diagonal and the camera's field of view), `track` by pointing it at the centre (set the rotation directly, no constraint); world colour `(0.8, 0.8, 0.8)` strength 1; render engine `BLENDER_EEVEE_NEXT` if that identifier exists in `bpy.types.RenderSettings.bl_rna.properties['engine'].enum_items`, else `BLENDER_EEVEE`; 1920 x 1080; `eevee.taa_render_samples 64`; `scene.unit_settings.system 'METRIC'`; when `scale < 0.1` (a print-sized model) `scale_length 0.001` and `length_unit 'MILLIMETERS'`, so one model unit shows as 1 mm, the size the slicer prints; when `scale == 1`, `scale_length 1.0` and `'METERS'`; otherwise `scale_length 1 / scale` and `'METERS'` (the terrain's real size); the world is 0.8 grey for camera rays and 0.25 grey for lighting (a Light Path "Is Camera Ray" mix), so the sun's shadows read; `bpy.ops.wm.save_as_mainfile(filepath=blend, compress=True)`; then the STL check: new empty scene, `bpy.ops.wm.stl_import(filepath=stl, forward_axis='Y', up_axis='Z')`, a bmesh from the mesh, `non_manifold = number of edges with len(e.link_faces) != 2`, `inverted = number of faces whose normal points inward` (test with `bmesh.ops.recalc_face_normals` on a copy and count faces that flipped), print `stl check: <triangles> triangles, <non_manifold> non-manifold edges, <inverted> inverted faces`, exit 1 when either count is not zero. One `print` per step prefixed `blender:`.
- `main`: after the writers, unless `--no-blend`: `findBlender`; none gives `error: Blender not found: pass --blender, set BLENDER, or install it at /Applications/Blender.app` and exit 1 (STL and PLY stay); otherwise `runBlender`; non-zero exit prints `error: Blender failed (exit N)` plus its output and exits 1; success prints `wrote: <blend>` and the `stl check:` line; `--verbose` prints the whole output.

- [ ] **Step 1: Write the failing runner test** (no real Blender): a fake `blender` shell script in a `TempDir` that writes its arguments to a file and exits with the code in `$FAKE_EXIT`: `runBlender` passes `--background --factory-startup --python <existing file> --` followed by the nine job arguments in order, returns the script's exit code, captures its stdout and stderr, and removes the temporary script afterwards; a non-existent executable gives exit 127 and a message naming it. `findBlender`: the flag wins; with `BLENDER` set to a path, that path; with `BLENDER` unset and `PATH` pointing only at a `TempDir` holding an executable `blender`, that file; with an empty `PATH` and no `/Applications/Blender.app` check possible, nullopt (skip that assertion when the app exists on this machine; test the order with the fake instead). The embedded script text contains `stl check:` and `ply_import`.
- [ ] **Step 2: Add the end-to-end case:** when `findBlender(nullopt)` finds one, run the 8 x 6 fixture without `--no-blend` and `--grid 4x4`: exit 0, `STEM.blend` exists and starts with the bytes `BLENDER` or with the Zstandard magic `28 b5 2f fd` (a compressed save), stdout contains `stl check: 28 triangles, 0 non-manifold edges, 0 inverted faces` and `wrote: <STEM>.blend`; with `--blender /nonexistent/blender`: exit 1 before anything is read, stderr `error: --blender /nonexistent/blender: does not exist`, nothing written (amended at the final review of 2026-10-19: a `--blender` that names no executable file is refused before anything is read; only a Blender that fails, or the automatic search finding none, keeps the STL and PLY). When no Blender is found, `SKIP("Blender not installed")`.
- [ ] **Step 3: Build; expect** the missing-header errors.
- [ ] **Step 4: Implement** the script, the embedding, the runner and the `main` changes.
- [ ] **Step 5: Run** `./build/demdl_tests "[blender],[mesh-e2e]"` and the full ctest; expect all passed (the Blender case takes a few seconds; Blender prints nothing to the terminal since its output is captured).
- [ ] **Step 6: Manual check:** the Table Mountain run of Task 10 Step 6 without `--no-blend`; open `<scratch>/tm.blend` in Blender, press F12: a lit relief seen from the south-east. Note what you saw in the commit message.
- [ ] **Step 7: Commit** `feat: Blender scene and STL check through the user's Blender`.

---

### Task 12: Documentation

Spec 8 (CLAUDE.md).

**Files:**
- Modify: `CLAUDE.md` (a `dem2mesh` section: what it does, the two example commands from Tasks 10 and 11, the Blender requirement and search order, `--dry-run` first, the no-data fill rule, the plinth, the 25 million limit, where the script lives, that `[mesh-e2e]` skips Blender when absent) and the spec's status line ("implemented").

- [ ] **Step 1: Write the section**, then run every command it quotes (dry runs and scratch outputs only).
- [ ] **Step 2: Commit** `docs: CLAUDE.md section for dem2mesh`.

---

## Self-review notes

- Spec coverage: 3 (Tasks 9, 10), 3.1 (10), 4 (3), 5.1–5.2 (4), 5.3 (5, 7), 5.4 (6), 5.5 (7, 8), 6 (7), 7 (11), 8 (2–12 as laid out), 9 (constraints, 10), 10–11 (nothing to build).
- The spec's `--grid` height-from-aspect rule and the "at least 2" rule are pinned in Tasks 9 and 10; the warning and refusal thresholds in Task 10.
- Type consistency: `PixelWindow` and `GridGeometry` (Task 3) are consumed unchanged by Tasks 4 and 10; `Mesh`/`Vec3`/`Triangle` (Task 7) by Tasks 8, 10, 11; `MeshOptions` (Task 9) by Task 10; `BlenderJob` (Task 11) only inside Task 11.

## Follow-ups deferred at the final review (2026-10-19)

- GeoTIFF reader: a Transverse Mercator file with a UTM EPSG code but no `ProjScaleAtNatOriginGeoKey` is read at scale 1, as GDAL reads it; it moves where a crop lands, not the mesh.
- GeoTIFF reader: GDAL's scale and offset metadata (tag 42112) is ignored, so an integer DEM that relies on it would show absurd heights on the `heights:` line; a refusal is about ten lines.
- Fill: nearest is 4-connected with scan-order ties, which extrudes plateaus along rows and columns beyond a diagonal boundary; if a render of the whole City looks wrong there, a Euclidean distance transform is the replacement.
- Crop window: the Transverse Mercator perimeter sampling does not add the central-meridian crossing; the error is below about 1 m.
- Resampler tests: the plane tolerance of 0.05 could be 0.005; a border-spike range test would guard the kernel.
- Writers and SolidMesh: the corner check appears three times and the cross product twice, pinned by tests; single-source them when next touched. The `FileSizeLimit` test helper is copied.
- `dem2mesh_main.cpp`: the summary texts live in main's anonymous namespace and are tested only by spawning; move them into a `MeshReport` unit in `demmesh_cli` when the summary next changes.
- Memory: the window is 1 GB for the City's whole file but resident memory is 2 GB because libtiff maps the file; opening with mode "rm" would make them match. The PLY could be written from the solid's surface prefix so that surface and solid are never held together.
- Tests: the two `[slow]` cases take about 32 s of the suite's 67 s; whether the suite is safe under `ctest -j` is untested.
- Owner decision pending: render the whole City file before printing it (61 percent of its pixels are filled).
