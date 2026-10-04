# dl_elevation: class refactor and command-line interface

Date: 2026-10-04
Status: approved design, awaiting implementation plan

## 1. Summary

`dl_elevation` scrapes a Digital Elevation Model from the City of Cape Town's
public WMS server one pixel at a time and accumulates the heights into a TIFF.
Today it is a single 450-line `main.cpp` with every parameter hardcoded, global
state throughout, an always-on OpenGL preview window, and no tests.

This design restructures it into a small static library of single-purpose
units, a thin command-line executable, an optional preview component, and a
Catch2 test suite. Every hardcoded constant becomes a flag. The output becomes
a georeferenced single-band float32 GeoTIFF that is self-describing enough to
resume safely. Several known defects are fixed along the way.

## 2. Goals and non-goals

Goals, all confirmed with the owner:

1. Reuse for other areas: point the tool at any bounding box, grid and output
   file without editing source.
2. Maintainability and tests: units with clear boundaries, testable without
   network or a display.
3. Unattended operation: headless by default, clean shutdown on signals,
   meaningful exit codes, buildable on a Linux server without OpenGL.
4. Groundwork: proper GeoTIFF tags now; room for other layers or hosts later.

Non-goals for this iteration:

- A metres-per-pixel flag (needs geodesic maths; nothing needs it yet).
- Output compression.
- Configuration files, multiple elevation sources, pluggable writers.
- Bulk fetch via WMS GetMap or WCS (not investigated).
- Windows support.
- Resuming files produced by the old RGB16 format.

## 3. Decisions made during design

| Topic | Decision |
|---|---|
| Output format | Single-band float32 GeoTIFF in metres. Old RGB16 files are not resumable and are left untouched. |
| TLS | Verify certificates by default using the system trust store. `--ca-bundle` overrides, `--insecure` disables. |
| Response parsing | Parse the whole number after `BAND 0:`. A 200 body is NoData only when it lacks the marker and is plain (not blank, no `ServiceException`, not markup); anything else without a finite in-range number is a parse error. Parse errors and transport failures are retried, then left for the next run. |
| Concurrency | `--threads` default 8 (was 50). Exponential backoff between retries. `--max-retries` default 8 (was 500). |
| Shutdown | SIGINT/SIGTERM stop queueing, drain in-flight requests, save, exit. |
| Preview | Off by default; `--preview` opens the window. |
| Structure | Static library + thin CLI + optional preview + tests. No source or sink abstractions beyond a one-method fetch interface for testing. |
| Dependencies | FetchContent pinned to release tags. CLI11 for parsing, Catch2 v3 for tests. ctpl kept for the pool. |

## 4. Architecture

### 4.1 CMake targets

| Target | Kind | Depends on | Notes |
|---|---|---|---|
| `demdl` | static library | `httplib::httplib`, `TIFF::TIFF`, libgeotiff, ctpl header | All logic. No OpenGL. |
| `dl_elevation` | executable | `demdl`, `demdl_cli`, optionally `demdl_preview` | Parses flags, wires components, maps results to exit codes. |
| `demdl_preview` | static library, optional | `demdl`, glfw, OpenGL framework (macOS) or GLEW + GL (Linux) | Built when `DL_ELEVATION_PREVIEW` is ON. Default ON on Apple, OFF elsewhere. |
| `demdl_tests` | executable | `demdl`, `Catch2::Catch2WithMain`, `demdl_cli` | Built when `DL_ELEVATION_TESTS` is ON (default ON). Registered with `ctest`. |

`demdl_cli` is an OBJECT library holding `src/app/Cli.cpp` and linking
`CLI11::CLI11`, so the parse function is compiled once and shared by the
executable and the tests.

GLM and assimp are removed. The quad shader needs only `glUniform1i`, so a
trimmed `Shader.h` without GLM moves into the preview component.

### 4.2 Source layout

```
src/demdl/
  BoundingBox.h/.cpp     validated WGS84 box, pixel scale
  Config.h               plain struct of every runtime option + validate()
  WmsQuery.h/.cpp        pure: build GetFeatureInfo URL, parse response body
  RetryPolicy.h/.cpp     pure: backoff schedule
  PixelFetcher.h         one-method interface used by Downloader
  ElevationClient.h/.cpp PixelFetcher over httplib with retries and TLS
  DemRaster.h/.cpp       float32 grid, GeoTIFF load/save, resume checks
  ProgressSink.h         event interface
  ConsoleReporter.h/.cpp headless sink
  Downloader.h/.cpp      row loop, pool, checkpoints, stop handling
src/preview/
  PreviewWindow.h/.cpp   GLFW/OpenGL sink
  Shader.h               trimmed from include/shader.h
src/app/
  Cli.h/.cpp             parseArgs(argc, argv) -> Config, testable
  main.cpp               signal handling, wiring, exit codes
tests/
  test_BoundingBox.cpp, test_WmsQuery.cpp, test_RetryPolicy.cpp,
  test_DemRaster.cpp, test_Downloader.cpp, test_ElevationClient.cpp,
  test_Cli.cpp, FakeFetcher.h, RecordingSink.h
```

Deleted: root `main.cpp`, `opengl.cmake.in`, every file in `include/`
(the shader is moved, the rest is unused LearnOpenGL boilerplate).
`ca-bundle.crt` stays in the repository as an optional `--ca-bundle` fallback
and is no longer referenced by default.

### 4.3 Units

**BoundingBox** holds `north`, `south`, `east`, `west` in decimal degrees.
`validate()` requires north > south, east > west, latitudes in [-90, 90],
longitudes in [-180, 180]. All four values must also be finite, and that check
is reported first. `pixelScale(width, height)` returns
`((east - west) / width, (north - south) / height)`. `approxEquals(other, tol)`
compares each edge with an absolute tolerance. This replaces the current code,
in which the variable named `north` holds the smaller latitude.

**Config** is a struct mirroring the flags in section 5 plus `validate()`,
which checks the box, positive grid size, `threads` between 1 and 64,
`maxRetries` between 0 and 100, `maxConsecutiveFailures >= 1`, that the output
path names a file, and that the WMS URL parses as `scheme://host[:port]/path`.
The scheme is `http` or `https`, the host is a hostname or a bracketed IPv6
literal, the port is 1 to 65535, and userinfo, queries, fragments and
whitespace are refused. No parsing library appears in the library.

**WmsQuery** has two free functions.
`std::string featureInfoUrl(const Config&, int x, int y)` returns the request
target (path plus query) for pixel column `x`, row `y`.
`ParseResult parseElevation(std::string_view body)` returns one of
`Value(double)`, `NoData`, or `ParseError`. A `BAND 0:` marker wins over
everything else in the body. The numeric prefix of the marker's line is parsed
and trailing text is ignored, so `BAND 0: 12.5 m` is 12.5; a number on the next
line does not count. The result is a `Value` only when the number is finite and
within float range, because the raster stores float32; anything else (no
number, `nan`, `inf`, `1e300`) is a `ParseError`. Without the marker the body
is `NoData` only when it is not blank (empty or only whitespace), does not
contain `ServiceException` and does not start with markup (`<` as the first
non-whitespace character). Anything else is a `ParseError` and is retried,
because NoData is stored as -32768 and never re-requested; WMS servers often
report a fault, such as a wrong layer name, as HTTP 200 with an XML exception
report or an HTML page. Both are pure.

**RetryPolicy** is `{maxRetries, baseDelay = 500 ms, maxDelay = 30 s,
jitter = 0.2}` with `delayFor(attempt, unitRandom)`: `min(base * 2^attempt,
max)` scaled by `1 + jitter * (2 * unitRandom - 1)`. The cap applies before
jitter, so the longest sleep is 36 s with the defaults. Taking the random value
as a parameter keeps the schedule testable.

**PixelFetcher** is the seam for tests:

```cpp
struct FetchResult {
  enum class Kind { Value, NoData, Failed } kind;
  float value = 0;        // valid when kind == Value
  int attempts = 0;
  std::string error;      // last error when kind == Failed
};
struct PixelFetcher {
  virtual ~PixelFetcher() = default;
  virtual FetchResult fetch(int x, int y, const std::atomic<bool>& stop) = 0;
};
```

**ElevationClient** implements `PixelFetcher` over one `httplib::Client`
constructed from the WMS origin, with keep-alive on, connection timeout 10 s,
read timeout 30 s, write timeout 10 s, certificate verification enabled unless
`insecure`, and `set_ca_cert_path` when `caBundle` is set. One instance per
worker thread. `fetch` loops over attempts: HTTP 200 with a finite in-range
band value is `Value`; HTTP 200 without the band marker is `NoData` only when
the body is not blank, does not contain `ServiceException` and does not start
with markup; any other status, a transport error, or `ParseError` (every other
200 body) is a failure that sleeps `RetryPolicy::delayFor` in 100 ms slices
while checking `stop`, then retries. Exhausting retries returns `Failed`. The
first `ParseError` body of the run is logged once to stderr, guarded by a
`std::once_flag` shared by all workers.

**DemRaster** holds `width`, `height`, `BoundingBox`, `RasterMetadata{layer,
wmsUrl}` and `std::vector<float>` in row-major order, row 0 at the north edge.
API: `create(width, height, box, metadata)` filled with NaN; `load(path)`
(throws on missing, unreadable or non-matching format, and rejects a file
whose box, reconstructed from the tiepoint and pixel scale, fails
validation); `save(path)` atomic; `float at(x, y) const`;
`void set(x, y, float)`; `std::vector<int> todoInRow(row)` listing NaN
columns; `rowsWithWork()`; `countNaN()`;
`bool matches(const Config&, std::string& why)`; constant `kNoData =
-32768.0f`.

**Downloader** runs the row loop described in section 7. It is constructed
with a `DemRaster&`, an output path, a fetcher factory
`std::function<std::unique_ptr<PixelFetcher>(int workerId)>`, a
`ProgressSink&`, a `std::atomic<bool>& stop`, the thread count and the
consecutive-failure limit. `run()` returns a `RunSummary`.

**ProgressSink** is the event interface in section 8. **ConsoleReporter** and
**PreviewWindow** implement it.

## 5. Command-line interface

Parsed with CLI11 in `src/app/Cli.cpp`. `--help` and `--version` come from
CLI11; the version string is the CMake project version, starting at 0.2.0.

| Flag | Required | Default | Meaning |
|---|---|---|---|
| `--north`, `--south`, `--east`, `--west` | yes | | WGS84 decimal degrees |
| `--width`, `--height` | yes | | grid size in pixels |
| `--output PATH`, `-o` | yes | | GeoTIFF path, relative to the current directory |
| `--wms-url URL` | no | `https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets` | scheme, host and path of the WMS endpoint |
| `--layer NAME` | no | `Elevation_CCT_GLM2019_5m_DEM` | WMS layer, used for both `LAYERS` and `QUERY_LAYERS` |
| `--threads N` | no | 8 | worker threads (1..64), one keep-alive connection each |
| `--max-retries N` | no | 8 | retries per pixel after the first attempt (0..100) |
| `--max-consecutive-failures N` | no | 100 | pixels failing in a row before the run stops |
| `--ca-bundle PATH` | no | unset | PEM bundle for certificate verification |
| `--insecure` | no | off | disable certificate verification |
| `--preview` | no | off | open the OpenGL window; error if built without preview support |
| `--overwrite` | no | off | ignore an existing output and start fresh |
| `--dry-run` | no | off | validate, report the plan, make no requests, write nothing |
| `--verbose` | no | off | log every pixel in addition to every row |

Example, today's Simon's Town run with the latitude names corrected:

```bash
dl_elevation --north -34.1845418 --south -34.2163447 --west 18.4185791 --east 18.4625244 --width 2048 --height 1792 --output simonstown.tif --preview
```

`--dry-run` prints: the resolved Config, whether the output exists and will be
resumed, rows remaining, pixels remaining, and the URL for the first pending
pixel. An existing output that does not match is still exit 1 with the
mismatch listing, dry run or not.

## 6. Output file format

Single-band GeoTIFF written with libtiff and libgeotiff.

TIFF fields: `SAMPLEFORMAT = IEEEFP`, `BITSPERSAMPLE = 32`,
`SAMPLESPERPIXEL = 1`, `PHOTOMETRIC = MINISBLACK`, `PLANARCONFIG = CONTIG`,
`COMPRESSION = NONE`, `ORIENTATION = TOPLEFT`, default strip size.

GeoTIFF keys: `GTModelTypeGeoKey = ModelTypeGeographic`,
`GTRasterTypeGeoKey = RasterPixelIsArea`, `GeographicTypeGeoKey = GCS_WGS_84`
(4326). `ModelTiepointTag = {0, 0, 0, west, north, 0}`.
`ModelPixelScaleTag = {(east - west) / width, (north - south) / height, 0}`.

Other tags: `GDAL_NODATA = "-32768"`,
`IMAGEDESCRIPTION = "dl_elevation;layer=<layer>;wms=<url>"` (fields separated
by `;`; `wms` is always last and extends to the end of the string so a URL
may contain any character), `SOFTWARE = "dl_elevation <version>"`. The
program registers the `GDAL_NODATA` tag (42113) with libtiff itself, because
libtiff does not know it by default.

Pixel semantics:

| Value | Meaning | On resume |
|---|---|---|
| finite float | elevation in metres as the server returned it, negatives kept | kept |
| `-32768` | server answered 200 with no `BAND 0:` and a plain body (not blank, no `ServiceException`, not markup): no elevation here; any other body without a finite in-range value is a parse error, retried | kept, never re-requested |
| NaN | never fetched, or failed after all retries | requested |

Row 0 is the north edge and corresponds to `J = 0` in the WMS request; column
0 is the west edge and `I = 0`. The tiepoint and pixel scale encode the same
box that is sent in every request, so the file and the requests agree exactly.

Reading back: `west = tiepoint[3]`, `north = tiepoint[4]`,
`east = west + scale[0] * width`, `south = north - scale[1] * height`.

## 7. Resume, checkpointing and the row loop

On start, when the output exists and `--overwrite` is not given:

1. `DemRaster::load` reads it. A file that is not single-band float32 fails
   with "not a dl_elevation float32 GeoTIFF; files from the previous RGB16
   format cannot be resumed".
2. `matches(config)` compares width, height, each box edge within 1e-9
   degrees, and the layer from `IMAGEDESCRIPTION`. Any mismatch is an error
   that lists every differing field and suggests `--overwrite` or a different
   `--output`. A differing WMS URL is a warning only.

When the output does not exist, or `--overwrite` is given, a new all-NaN
raster is created and written before the first request so the path is
validated early and the file can be opened while it fills in.

The work list is every NaN pixel, grouped by row, rows ascending, columns
ascending. There is no cursor in the metadata; the raster is the state. This
one rule covers a partial row after a stop, pixels that exhausted retries last
time, and a rerun on a complete file, which makes no requests.

Row loop in `Downloader::run`:

```
emit onRunStarted(plan)
for row in rowsWithWork():
    if stop: break
    todo = todoInRow(row)
    emit onPixelState(x, row, Queued) for each x in todo
    futures = []
    for x in todo:
        if stop: break            # stop queueing immediately
        futures += pool.push(job(x, row))
    wait for every future
    raster.save(output)           # atomic: write <output>.tmp, rename
    emit onRowSaved(report)
emit onRunFinished(summary)
```

`job(x, row)`: emit `InFlight`; `result = fetcher(workerId).fetch(x, row,
stop)`; write `at(x, row)` as the value, `kNoData`, or leave NaN; update the
consecutive-failure counter (reset on `Value` or `NoData`, increment on
`Failed`; reaching the limit sets `stop` with reason `ConsecutiveFailures`);
emit the final state.

Each job writes a distinct cell and the save runs only after that row's
futures have returned, so the raster needs no lock.

Atomic save: write to `<output>.tmp` in the same directory, close, then
`std::filesystem::rename` over the output. `DemRaster::save` throws on
failure; `Downloader::run` catches it, records reason `WriteError` with the
message, stops, and returns the summary normally. The previous checkpoint is
intact.

`RunSummary` carries `pixelsFetched`, `noData`, `failed`, `rowsSaved`,
`remainingNaN`, `elapsed` and `stopReason` in
`{None, Signal, WindowClosed, ConsecutiveFailures, WriteError}`.

## 8. Progress events and sinks

```cpp
enum class PixelState { Queued, InFlight, Value, NoData, Failed };
struct RunPlan   { uint64_t pixelsTotal, pixelsTodo; int rowsTodo; };
struct RowReport { int row, ok, noData, failed; std::chrono::duration<double> elapsed;
                   uint64_t pixelsDoneTotal; };
struct ProgressSink {
  virtual void onRunStarted(const RunPlan&) = 0;
  virtual void onPixelState(int x, int y, PixelState, float value) = 0;
  virtual void onRowSaved(const RowReport&) = 0;
  virtual void onRunFinished(const RunSummary&) = 0;
};
```

Pixel events arrive from worker threads; row and run events from the
downloader thread. Sinks guard their own state.

**ConsoleReporter** writes to stdout. Per saved row:

```
row 812/1792  done 1247296/3670016 (34.0%)  row: 2048 ok 0 nodata 3 failed 41.2s  49.7 px/s  eta 13h32m
```

Rate and ETA use the pixels completed over the last ten saved rows, or all
rows if fewer. With `--verbose`, each pixel also logs
`x=12 y=812 123.4 m (1 attempt)`, `x=12 y=812 nodata`, or
`x=12 y=812 failed: <error>` under a mutex. On finish:

```
done: 3669980 values, 12034 nodata, 3 failed, 1792 rows saved, 14h02m elapsed
rerun with the same arguments to retry the 3 failed pixels
```

A stop prints its reason first, for example
`stopping: interrupted; waiting for 8 in-flight requests`.

**PreviewWindow** implements the sink over GLFW 3.3 core profile in a 640x480
resizable window titled `dl_elevation 34.0%`. It owns a per-pixel state byte
array and a float value array under one mutex, updated by events. The main
thread polls GLFW events, and every 500 ms rebuilds an RGB8 texture from those
arrays and draws it on a fullscreen quad. Colours: to-do (NaN) dark grey
(40, 40, 40); NoData blue (0, 0, 255); Queued red; InFlight green; Failed
magenta (255, 0, 255); values use the existing `rainbow()` mapping over the
minimum and maximum values seen so far, so the palette stays consistent as
new maxima arrive instead of being pinned to 700 m. The fragment shader
samples the texture unchanged, which removes the broken `maxValue` uniform.
Closing the window sets `stop` with reason `WindowClosed`. The window closes
when the run finishes.

## 9. Threads, signals and `main`

`main` flow:

1. `parseArgs` into `Config`; `validate()`. Errors exit 1.
2. Load or create the `DemRaster` per section 7. Errors exit 1.
3. `--dry-run`: print the plan, exit 0.
4. Write the initial file if newly created.
5. Install handlers: SIGINT and SIGTERM set the shared `std::atomic<bool>
   stop` and record reason `Signal`; a second signal calls `_Exit(130)`.
   SIGPIPE is set to `SIG_IGN`.
6. Choose the sink: `PreviewWindow` if `--preview`, else `ConsoleReporter`.
7. Start the `Downloader` on its own thread.
8. Headless: join the downloader. Preview: run the GLFW loop on the main
   thread until the run finishes or the window closes, then join.
9. Print the summary and exit with the code from section 10.

The thread pool is ctpl with `--threads` workers. The fetcher factory creates
one `ElevationClient` per worker id so each thread reuses its own connection.

## 10. Errors and exit codes

| Code | Meaning |
|---|---|
| 0 | complete: `remainingNaN == 0` |
| 1 | usage or configuration error; nothing requested |
| 2 | stopped early (signal, window closed, consecutive failures, write error); checkpoint intact; rerun resumes |
| 3 | ran to the end but `failed > 0`; rerun resumes |
| 130 | second interrupt, immediate exit |

Mapping: `remainingNaN == 0` gives 0; otherwise `stopReason != None` gives 2;
otherwise 3. Library code throws `std::runtime_error` for unrecoverable
conditions before the run starts; `main` catches at the top level, prints the
message to stderr and exits 1. During the run the only expected failure is a
checkpoint write, which surfaces as `stopReason == WriteError` and exits 2.
Jobs never throw across the future boundary.

## 11. Tests

Catch2 v3 via FetchContent, discovered with `catch_discover_tests`, run with
`ctest --test-dir <build>`.

- **BoundingBox**: accepts a valid box; rejects north <= south, east <= west,
  out-of-range values; pixel scale for the example box; `approxEquals`
  tolerance.
- **WmsQuery**: golden URL for the example Config at pixel (0, 0) equals
  `/erdas-iws/ogc/wms/GeoSpatial%20Datasets?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetFeatureInfo&BBOX=-34.216344700%2C18.418579100%2C-34.184541800%2C18.462524400&CRS=EPSG%3A4326&WIDTH=2048&HEIGHT=1792&LAYERS=Elevation_CCT_GLM2019_5m_DEM&STYLES=&FORMAT=image%2Fpng&QUERY_LAYERS=Elevation_CCT_GLM2019_5m_DEM&INFO_FORMAT=text%2Fplain&I=0&J=0`
  (coordinates fixed at nine decimals, order south, west, north, east as WMS
  1.3.0 requires for EPSG:4326); a layer name with spaces is percent-encoded,
  and so are non-ASCII bytes (`\xC3\xA9` gives `%C3%A9`) and `%` itself
  (`%25`); `featureInfoUrl` throws `std::invalid_argument` when the WMS URL
  does not parse; parsing of `BAND 0: 123.45`, `BAND 0: -1.5`,
  `BAND 0: 1086.25` and `BAND 0: 12.5 m` (12.5, trailing text is ignored);
  `BAND 0: abc`, `BAND 0:`, `BAND 0:` followed by a newline and a number,
  `BAND 0: nan`, `BAND 0: inf`, `BAND 0: -inf`, `BAND 0: 1e300`,
  `BAND 0: -1e300` and `BAND 0: 3.5e38` (all ParseError), while
  `BAND 0: 3.4028234663852886e38`, exactly `FLT_MAX`, is a Value; a body
  without the marker is NoData when it is plain, such as
  `GetFeatureInfo results:` or `GetFeatureInfo results:\nno bands\n`, but an
  empty body, a whitespace-only body, an HTML page, an XML
  `ows:ExceptionReport` after leading spaces and a `ServiceException` body are
  ParseError, and the band marker wins over all of them when it is present; a
  default-constructed `ParseResult` is a ParseError.
- **RetryPolicy**: delays for attempts 0 to 7 with `unitRandom = 0.5` are
  0.5, 1, 2, 4, 8, 16, 30, 30 seconds; jitter bounds at 0 and 1.
- **DemRaster**: create, set a value, NoData and NaN, save to a temp
  directory, load, compare dimensions, box, values and NaN positions; the
  GeoTIFF keys and `GDAL_NODATA` read back; `matches` accepts the same Config
  and rejects a shifted box, a different size and a different layer with the
  expected text in `why`; a hand-written RGB16 TIFF is rejected with the
  legacy message; after `save` no `.tmp` file remains; `todoInRow` and
  `rowsWithWork` on a half-filled raster.
- **Downloader**: with `FakeFetcher` (value is a function of x and y;
  configurable NoData and failure sets; optional latch to block mid-row) and
  `RecordingSink`: only NaN pixels are fetched; rows without work emit no
  events; one save per worked row, verified by reloading the file; setting
  `stop` while a row is blocked saves the partial row and processes no further
  rows; the consecutive-failure limit stops the run with that reason; summary
  counts match.
- **ElevationClient**: an in-process `httplib::Server` on a loopback port
  serves canned bodies: a value, a NoData body, a 500 followed by a 200 to
  prove a retry, and a connection close to prove failure after retries, with
  a `RetryPolicy` of tiny delays. Plain HTTP; TLS is not tested.
- **Cli**: the example argv yields the expected Config; omitting `--north`
  fails; `--insecure` and `--ca-bundle` set their fields; `--threads 0`
  fails validation.

`PreviewWindow` and the real server are not tested.

## 12. Build changes

- `cmake_minimum_required(VERSION 3.27)`, `project(DownloadElevation VERSION
  0.2.0 LANGUAGES CXX)`, C++20, `CMAKE_EXPORT_COMPILE_COMMANDS ON`,
  `CMAKE_FIND_FRAMEWORK LAST` retained.
- Options `DL_ELEVATION_PREVIEW` (default `APPLE`) and `DL_ELEVATION_TESTS`
  (default ON).
- FetchContent pins: cpp-httplib `v0.46.1` (matches the Homebrew package in
  use; `FIND_PACKAGE_ARGS 0.46.1 NAMES httplib` so a system copy is preferred
  only when it is 0.46.x, otherwise the pinned copy is downloaded),
  ctpl commit `437e135dbd94eb65b45533d9ce8ee28b5bd37b6d`
  (its two tags point at this commit), CLI11 `v2.7.2`, Catch2 `v3.16.0`.
- libtiff via `find_package(TIFF REQUIRED)`; libgeotiff via `find_path` and
  `find_library` as today, now actually used.
- Preview: glfw via pkg-config; on Apple link the OpenGL, Cocoa, IOKit and
  CoreVideo frameworks; on Linux link GLEW and GL. No GLM, assimp, FreeType,
  CoreAudio or AudioUnit.
- `-Wall -Wextra` on `demdl`, `dl_elevation` and `demdl_preview`.
- `.gitignore` adds `build*/`, `*.tif`, `*.tif.tmp`.
- CLAUDE.md is rewritten for the new layout, commands, flags and test run.

Toolchain requirement: AppleClang 15 on macOS, GCC 12 or Clang 15 on Linux.
`std::jthread` and `std::format` are unavailable on AppleClang 15 and are not
used.

## 13. Behaviour changes relative to today

- Output is float32 metres with NoData `-32768`; no 700 m scale factor;
  negative heights preserved.
- Old RGB16 files are not resumable.
- Paths are relative to the current directory, not `../` from the build dir.
- Preview is off unless `--preview`.
- 8 threads and 8 retries by default instead of 50 and 500; failures back off.
- Certificates are verified.
- A missing band is NoData, not 0 m, but only for a body that is not blank,
  has no `ServiceException` and does not start with markup; anything else
  without a finite in-range band value is retried, and a failed pixel stays
  NaN for the next run.
- Resume is per pixel, not "the row after the last non-zero sample".
- Closing the window or pressing Ctrl-C saves and exits cleanly.
- Each worker reuses one connection instead of a new TLS handshake per pixel.

## 14. Future work (not in this iteration)

`--resolution` in metres per pixel, `--compress`, a `--retry-missing` mode
that re-requests NoData pixels, and investigating whether the server offers a
bulk raster endpoint.
