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
path names a file, that the layer is not empty and contains no `;` (the field
separator of the output's `IMAGEDESCRIPTION`), and that the WMS URL parses as
`scheme://host[:port]/path`. The scheme is `http` or `https`, the host is a
hostname or a bracketed IPv6 literal, the port is 1 to 65535, and userinfo,
queries, fragments and whitespace are refused. No parsing library appears in
the library.

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

`fetch` must not throw, because it runs inside a pool job: every failure is
reported in the returned `FetchResult`. Once `stop` is set an implementation
starts no further attempt and does not keep sleeping between attempts (a
request already on the wire may run to its timeout), and `value` is finite
whenever `kind == Value`.

**ElevationClient** implements `PixelFetcher` over one `httplib::Client`
constructed from the WMS origin, with keep-alive on, connection timeout 10 s,
read timeout 30 s, write timeout 10 s, an overall cap of 60 s per request
(`set_max_timeout`, so a server trickling its answer cannot hold a worker
indefinitely), the User-Agent `dl_elevation/<version>`, certificate
verification enabled unless `insecure`, and `set_ca_cert_path` when `caBundle`
is set. A stop cannot interrupt a request already on the wire. Once connected,
a request ends within the 60 s overall cap, which counts from the start of the
request; resolving, connecting (per address) and each TLS handshake wait are
bounded separately by the 10 s connection timeout. The client relies on the
process ignoring SIGPIPE (section 9), because httplib's client does not
suppress it per socket. The constructor throws `std::invalid_argument` when
the WMS URL does not parse or when the new client reports `is_valid()` false
(httplib could not set it up, for example without a TLS context), because its
methods could then dereference a null implementation. One instance per worker
thread. `fetch` loops over attempts: HTTP 200 with a finite in-range band
value is `Value`; HTTP 200 without the band marker is `NoData` only when the
body is not blank, does not contain `ServiceException` and does not start with
markup; any other status, a transport error, or `ParseError` (every other 200
body) is a failure that sleeps `RetryPolicy::delayFor` in 100 ms slices while
checking `stop`, then retries. Certificate errors are the exception: a CA
bundle that cannot be loaded, a chain that does not verify and a certificate
that does not cover the host (httplib's `SSLLoadingCerts`,
`SSLServerVerification` and `SSLServerHostnameVerification`) return `Failed`
at once with the transport error, because only a configuration change cures
them, and retrying would also hide the cause: httplib loads the bundle once
per client, so a retry after a failed load reports the verification failure
that follows instead. For the same reason a failed load is sticky: the client
keeps the first `SSLLoadingCerts` error and answers every later fetch with it,
as `Failed` with one attempt, without sending a request, so the stop message's
last error still names the real cause. Exhausting retries returns `Failed`.
Building the request target, the request and the parse run inside a `try`, so
an exception becomes that attempt's error (`exception: <what>` or `unknown
exception`) and is retried like any other failure; only an allocation failure
while building the error text could still escape, which is why `fetch` is not
`noexcept`.

Two warnings go to stderr, never stdout, so they cannot corrupt the progress
lines. Each is written once per process, across all workers, under its own
`std::once_flag`, and writing one never throws. The first failed attempt of
the run prints `warning: first failed request: <error> (retrying)` when
`fetch` will retry it, or `warning: first failed request: <error> (not
retried)` when it will not (a certificate error, retries exhausted, or a stop
requested), so a wrong path, a bad CA bundle or an unverifiable chain shows
within seconds even without `--verbose`, and a transient failure does not look
alarming. The first `ParseError` body prints the line built by the pure
function `describeUnparseableBody`, which is exposed for tests: a blank body
is reported by its byte count (`was N bytes of whitespace`), any other body by
its first 200 bytes with every byte outside printable ASCII replaced by `.`,
so the warning stays on one line and no control character, raw or
UTF-8-encoded, reaches the terminal.

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
`ProgressSink&`, a `StopToken&`, the thread count and the
consecutive-failure limit. `run()` returns a `RunSummary`. A pool job records
an exception from the fetch or the sink's `InFlight` callback as a `Failed`
pixel, and jobs are `noexcept`, so no exception crosses the future boundary.

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

`save` writes `<output>.tmp`, flushes and syncs it to disk, and only then
renames it over the output. libtiff's diagnostics are not printed: errors
are collected by a libtiff error handler and appended to the message of the
`load` or `save` error they explain, followed for a failed write by the
operating system's reason in parentheses, and warnings are discarded.

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
   format cannot be resumed". A file whose `IMAGEDESCRIPTION` does not start
   with `dl_elevation;` was not written by this tool and is refused, before
   anything is allocated, with "was not written by dl_elevation; choose
   another --output or use --overwrite". `load` also rejects a width or
   height above `INT_MAX` before allocating.
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
stop)`; call `set(x, row, ...)` with the value or `kNoData`, or leave NaN;
update the consecutive-failure counter (reset on `Value` or `NoData`,
increment on `Failed`; reaching the limit records that failure's error, unless
one is recorded already, and requests a stop with reason
`ConsecutiveFailures`); emit the final state. The `InFlight` callback and the
fetch run inside a catch-all that turns an exception into a `Failed` result
with the error `exception: <what()>` (or `unknown exception` for a type not
derived from `std::exception`), which is then counted and reported exactly
like a failed fetch. The job is `noexcept`, so an exception anywhere else in
it ends the process instead of reaching the pool.

Each job writes a distinct cell and the save runs only after that row's
futures have returned, so the raster needs no lock.

Atomic save: write to `<output>.tmp` in the same directory, flush and sync
it, close, then `std::filesystem::rename` over the output. `DemRaster::save`
throws on failure; `Downloader::run` catches it, records reason `WriteError`
with the message, stops, and returns the summary normally. If a stop was
already requested with another reason, that reason stands and the summary's
stop message gains `; checkpoint write failed: <message>`. The previous
checkpoint is intact.

`RunSummary` carries `pixelsFetched`, `noData`, `failed`, `rowsSaved`,
`remainingNaN`, `elapsed`, `stopReason` in
`{None, Signal, WindowClosed, ConsecutiveFailures, WriteError}`,
`stopMessage` and `checkpointWriteFailed` (section 8).

## 8. Progress events and sinks

```cpp
enum class PixelState { Queued, InFlight, Value, NoData, Failed };
struct RunPlan   { uint64_t pixelsTotal, pixelsTodo; int rowsTodo; };
struct RowReport { int row, ok, noData, failed; std::chrono::duration<double> elapsed;
                   uint64_t pixelsDoneTotal; };
struct RunSummary { uint64_t pixelsFetched, noData, failed, remainingNaN; int rowsSaved;
                    std::chrono::duration<double> elapsed; StopReason stopReason;
                    std::string stopMessage; bool checkpointWriteFailed; };
struct ProgressSink {
  virtual void onRunStarted(const RunPlan&) = 0;
  virtual void onPixelState(int x, int y, PixelState, float value) = 0;
  virtual void onRowSaved(const RowReport&) = 0;
  virtual void onRunFinished(const RunSummary&) = 0;
};
```

`remainingNaN` counts the NaN pixels in the last checkpoint written
successfully, not those in memory, so a failed save can never make an
incomplete file look complete. `checkpointWriteFailed` is set whenever a save
threw. The stop message then reads `checkpoint write failed: <reason>`, or,
when another stop reason came first and was kept, that reason's message
followed by `; checkpoint write failed: <reason>`. A stop for the
consecutive-failure limit reads `server unreachable: <limit> consecutive
failures; last error: <error>`, where `<error>` is the error of the failure
that reached the limit, for example `server unreachable: 100 consecutive
failures; last error: transport: Could not establish connection`.

Pixel events may arrive concurrently from worker threads and from the
downloader thread (`Queued` comes from the downloader); row and run events come
from the downloader thread. Sinks guard their own state. Callbacks must not
throw, because pixel events are delivered inside pool jobs.

**ConsoleReporter** writes to stdout. It opens with
`starting: 3670016 pixels in 1792 rows remaining of 3670016 pixels in 1792 rows`
and then prints, per saved row:

```
row 812/1792  done 1247296/3670016 (34.0%)  row: 2048 ok 0 nodata 3 failed 41.2s  49.8 px/s  eta 13h31m
```

The rate divides the pixels attempted (ok, nodata and failed) in the last ten
saved rows, or all rows if fewer, by those rows' elapsed time. The ETA divides
the pixels the run has not yet attempted (the plan's to-do count less the
pixels attempted in its saved rows) by that rate, so a pixel that failed,
which the run does not retry, stops counting once attempted; the ETA reads
`--` while the rate is zero. The `done` count and percentage describe the
file: its non-NaN pixels after the save. Durations read `41.2s` under a
minute, `12m05s` under an hour and `13h31m` beyond, with the unit chosen after
rounding, so 59.95 s reads `1m00s`; NaN and negative durations read `0.0s`,
and anything above a billion hours reads `1000000000h00m`. With `--verbose`,
each resolved pixel also logs
`x=12 y=812 123.4 m (1 attempt)`, `x=12 y=812 nodata`, or
`x=12 y=812 failed after 9 attempts: <error>` (`after 1 attempt` for a single
one, and plain `x=12 y=812 failed: <error>` when no attempt was counted, as
for an exception the Downloader caught); `Queued` and `InFlight` log nothing.
Every line is written whole under the reporter's mutex, so concurrent pixel
lines never interleave. On finish:

```
done: 3669980 values, 12034 nodata, 3 failed, 1792 rows saved, 14h02m elapsed
rerun with the same arguments to retry the 3 failed pixels
```

A stopped run adds `stopped: <stop message>` after the `done:` line. The last
line says how to complete the file, in one of three forms:

```
rerun with the same arguments to retry the 3 failed pixels
rerun with the same arguments to resume the remaining 500 pixels
rerun with the same arguments to resume the remaining 500 pixels (2 failed this run)
```

The `retry` form appears when the only NaN pixels left in the file are this
run's failures, the `resume` forms when more are left, with the failure count
when some pixels failed; there is no such line when no pixel is NaN. A count
of one reads `pixel`: `retry the 1 failed pixel`,
`resume the remaining 1 pixel`. `writeSummary` writes these finish lines;
`onRunFinished` calls it, and `main` calls it after a `--preview` run, whose
sink prints nothing. The starting line, each row line and the finish lines
are flushed as they are written, so that with stdout redirected to a file
each row reaches the log when it is saved; the frequent verbose pixel lines
are not flushed. The line that announces a stop as it happens, for example
`stopping: interrupted; waiting for 8 in-flight requests`, is printed by
`main` when it sees the stop token raised, not by the reporter: `main` polls
the token every 100 ms, because nothing may print from inside a signal
handler.

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
Closing the window requests a stop with reason `WindowClosed`. The window
closes when the run finishes.

## 9. Threads, signals and `main`

`main` flow:

1. `parseArgs` into `Config`; `validate()`. Errors exit 1.
2. Load or create the `DemRaster` per section 7. Errors exit 1.
3. `--dry-run`: print the plan, exit 0.
4. Write the initial file if newly created.
5. Install handlers: SIGINT and SIGTERM call `request(StopReason::Signal)` on
   the shared `StopToken`; a second signal calls `_Exit(130)`. SIGPIPE is set
   to `SIG_IGN`.
6. Choose the sink: `PreviewWindow` if `--preview`, else `ConsoleReporter`.
7. Start the `Downloader` on its own thread.
8. Headless: join the downloader. Preview: run the GLFW loop on the main
   thread until the run finishes or the window closes, then join.
9. Print the summary and exit with the code from section 10.

`StopToken` is the shared stop flag. `request(reason)` raises the flag and
records the reason only if none is recorded yet, so the first reason wins;
`reason` must not be `None`, and `request` touches only lock-free atomics, so a
signal handler may call it. `requested()` says whether anyone has asked to
stop, `reason()` returns the first reason given (`None` until then), and
`flag()` returns the underlying `const std::atomic<bool>&`, which is what the
`Downloader` passes to `PixelFetcher::fetch`. The flag itself is private, so
nothing can raise it without giving a reason.

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
checkpoint write. It always sets `checkpointWriteFailed`, even when another
stop reason came first and stays the recorded one, and otherwise surfaces as
`stopReason == WriteError`. Either way the run exits 2, because
`remainingNaN` describes the last checkpoint written, where the failed row's
pixels are still NaN. Jobs never throw across the future boundary.

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
  directory, load, compare dimensions, box, values and NaN positions; a
  2048 x 8 raster spanning several strips round-trips every pixel and still
  matches its Config; the GeoTIFF keys, `GDAL_NODATA` and `SOFTWARE` read
  back; `matches` accepts the same Config and rejects a shifted box, a
  different size and a different layer with the expected text in `why`,
  reports a NaN edge as a mismatch and shows an empty file layer as
  `(none)`; a hand-written RGB16 TIFF is rejected with the legacy message; a
  hand-written float32 GeoTIFF whose tiepoint has a NaN west edge is rejected
  as an invalid bounding box, and one without the dl_elevation description
  as not written by dl_elevation; a file claiming a width above `INT_MAX` is
  rejected before anything is allocated; a missing file's error carries
  libtiff's reason; after `save` no `.tmp` file remains; a `save` whose
  directory write fails (a file-size limit one byte short of a complete file)
  throws with the operating system's reason, leaves no `.tmp` and keeps the
  previous file loadable with its old values, and so does a `save` whose
  strip write fails part way (a limit of half the file) or whose temporary
  file cannot be created (a file-size limit of zero); `todoInRow` and
  `rowsWithWork` on a half-filled raster.
- **Downloader**: with `FakeFetcher` (value is a function of x and y;
  configurable NoData and failure sets; optional latch to block mid-row) and
  `RecordingSink`: only NaN pixels are fetched; rows without work emit no
  events; one save per worked row, verified by reloading the file; eight
  workers on a 64 x 16 grid of mixed values, NoData and failures leave a file
  that matches every cell and the summary counts; requesting a stop while a
  row is blocked saves the partial row and processes no further rows, and the
  two columns fetched are the first two submitted; the consecutive-failure
  limit stops the run with that reason and a message naming the error of the
  failure that reached it, an intervening success resets the streak, and the
  streak carries across rows; an exception thrown inside a
  job is recorded as a failed pixel and does not escape `run()`; a failed
  checkpoint stops the run before the next row, sets `checkpointWriteFailed`
  and leaves `remainingNaN` counting the file rather than memory, and one that
  fails after a stop request keeps the stop's reason and still reports the
  write failure; an exception from the fetcher factory leaves `run()`
  unchanged before any callback, and a null fetcher is refused the same way;
  summary counts match.
- **ElevationClient**: an in-process `httplib::Server` on a loopback port
  serves canned bodies, with a `RetryPolicy` of tiny delays: a value, a plain
  NoData body, a 500 followed by a 200 to prove a retry, a 404 that never
  recovers (three requests), and a 200 `ServiceExceptionReport` that is
  retried and fails as `unparseable body` after three requests; a refused
  connection fails after the retries with a transport error, and a stop
  request cuts the 10 s backoff after a 404 short (one attempt, `stopped
  during backoff after: http 404`). Three fetches through one client arrive
  from a single remote port, proving keep-alive reuse; the server receives the
  decoded `BBOX`, `LAYERS`, `I` and `J` for pixel (2, 0) of the example box on
  a 4 x 1 grid; a layer named `Elevation Layer+1&2%` arrives intact in both
  `LAYERS` and `QUERY_LAYERS`, which locks httplib's re-encoding of the query
  across upgrades; and every request carries exactly one User-Agent,
  `dl_elevation/<version>`. Only decoded parameters are compared, never the
  raw request target, because httplib re-encodes the query (`%2C` goes out as
  `,`). TLS is tested on loopback against an `httplib::SSLServer` whose
  self-signed EC P-256 certificate for `IP:127.0.0.1` is generated at test
  time through the OpenSSL API, so no key is committed: with the defaults,
  verification rejects it after one attempt (`SSLServerVerification`) and the
  handler is never reached; `insecure` accepts it; the certificate written to
  a temporary PEM file and passed as the CA bundle makes it trusted; and a CA
  bundle path that does not exist fails after one attempt (`SSLLoadingCerts`),
  again without reaching the handler, and keeps failing with that same error,
  one attempt each, on the next fetch; with the bundle trusted but the URL
  naming `localhost`, which the certificate's only subjectAltName
  `IP:127.0.0.1` does not cover, the host name check fails after one attempt
  (`SSLServerHostnameVerification`) before any request reaches the handler.
  `describeUnparseableBody` is tested directly: an empty and a whitespace-only
  body give their byte counts, a 300-byte body shows exactly 200 bytes, and
  ESC, BEL, DEL and the UTF-8 bytes `C2 9B` become `.` on a single line.
- **ConsoleReporter**, writing to a `std::ostringstream`: `formatDuration`
  gives `41.2s`, `12m05s`, `13h31m`, `14h02m` and, truncating the minutes,
  `1h59m` for 7199; it chooses the unit after rounding, so 59.94 gives
  `59.9s`, 59.95 and 60 give `1m00s`, 3599.4 gives `59m59s` and 3599.6 gives
  `1h00m`; -1 and NaN give `0.0s`, and 1e300 and infinity give
  `1000000000h00m`. A resumed run's starting line reads exactly
  `starting: 2422720 pixels in 1100 rows remaining of 3670016 pixels in 1792 rows`.
  Row 811 of 1792, saved with 2048 ok, 0 nodata and 3 failed in 41.2 s, with
  1247296 of 3670016 pixels done and 2422720 of the run's pixels unattempted,
  prints exactly
  `row 812/1792  done 1247296/3670016 (34.0%)  row: 2048 ok 0 nodata 3 failed 41.2s  49.8 px/s  eta 13h31m`;
  a row saved with no elapsed time prints `0.0 px/s  eta --`; of eleven rows
  whose first took 1e6 s, the tenth line still prints `0.0 px/s`, and the
  eleventh, whose window has dropped that row, prints exactly
  `row 11/20  done 1060/2000 (53.0%)  row: 90 ok 6 nodata 4 failed 2.0s  50.0 px/s  eta 18.0s`;
  a row that attempts the run's last 100 pixels, 10 of them failing, prints
  `10.0 px/s  eta 0.0s`; and after a second `onRunStarted` the rate and the
  ETA come from the new run's row alone (`50.0 px/s  eta 34.0s`). The summary
  prints the totals line and the retry hint and no `stopped:` line, and for a
  single failure the hint reads `retry the 1 failed pixel`; a stopped run with
  nothing failed and nothing left prints exactly its `done:` line and then
  `stopped: interrupted`; a stopped run with 500 pixels left ends
  `resume the remaining 500 pixels`, followed by ` (2 failed this run)` after
  two failures, and one with a single pixel left ends
  `resume the remaining 1 pixel`; `writeSummary` writes exactly what
  `onRunFinished` prints. Verbose lines are checked for
  values after one attempt and after two, NoData, failures after 9 attempts
  and after 1, and failures with attempts 0 and -1, which print
  `failed: <error>` without a count; `Queued` and `InFlight` print nothing,
  and without verbose nothing prints at all. Through a `std::stringbuf`
  subclass that counts `sync()` calls, the starting line, a row line and the
  finish lines each flush everything written so far, while five pixel events
  in verbose mode flush nothing. Four threads posting 1000 verbose events each
  produce exactly the 4000 expected lines, none interleaved.
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
