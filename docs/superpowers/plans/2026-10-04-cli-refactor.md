# dl_elevation CLI Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn the single-file `dl_elevation` scraper into a tested static library, a thin CLI11 executable with every former constant as a flag, an optional OpenGL preview, and a float32 GeoTIFF output that resumes safely.

**Architecture:** A static library `demdl` holds the pure units (BoundingBox, Config, WmsQuery, RetryPolicy), the GeoTIFF raster (DemRaster), the HTTP fetcher (ElevationClient behind a one-method PixelFetcher interface) and the row-loop orchestrator (Downloader), which reports through a ProgressSink. `dl_elevation` parses flags into a Config and wires the pieces. `demdl_preview` is a GLFW sink built only when `DL_ELEVATION_PREVIEW` is on. `demdl_tests` is Catch2 and touches neither the network nor a display, apart from an in-process HTTP server on loopback.

**Tech Stack:** C++20 on AppleClang 15 (no `std::format`, `std::jthread`, or floating-point `std::from_chars`); CMake 3.27+ with Ninja; FetchContent-pinned cpp-httplib v0.46.1, ctpl 437e135, CLI11 v2.7.2, Catch2 v3.16.0; Homebrew libtiff and libgeotiff; GLFW 3.3 for the preview.

**Spec:** `docs/superpowers/specs/2026-10-04-cli-refactor-design.md`. Read it once before starting. Each task names the spec sections it implements.

---

## Deviations from the spec (deliberate, small)

1. `ProgressSink::onPixelState` takes a `PixelEvent` struct carrying `attempts` and `error` as well as x, y, state and value, because the verbose console line prints attempts and the failed line prints the error.
2. `RunPlan` gains `rowsTotal`; `RowReport` gains `rowsTotal` and `pixelsTotal`, so the console can print `row 812/1792` and a percentage without extra state.
3. The shared stop flag is wrapped in a `StopToken` (an atomic bool plus an atomic `StopReason`) so the signal handler, the preview window and the downloader can all record why the run stopped. `PixelFetcher::fetch` still receives a bare `const std::atomic<bool>&`.
4. `Config` gets a `.cpp` for `validate()` and `parseWmsUrl()`, and `WmsQuery` exposes `formatCoordinate()` so the URL, the mismatch report and the dry run print degrees identically.
5. The `stopping: ...` line is printed by `main`, which polls the token every 100 ms, because nothing may print from inside a signal handler.
6. httplib timeouts use the `(seconds, microseconds)` overloads, which exist in every version in play; `set_max_timeout`, which has no such overload, takes milliseconds.
7. The value palette in the preview spans red to light blue rather than the full rainbow, so the minimum value is not pure blue (NoData) and the magenta end (Failed) is never used for a value.
8. The raster's metadata struct is `RasterMetadata`, and reads and writes go through `at() const` and `set()` rather than a `float&` accessor, so every concurrent write passes through one function.

## Working conventions

- Catch2 test names must not begin with `-`: ctest passes the name to the binary as an argument, and Catch2 reads it as an option, so ctest reports such a case as passed without running it (a `--help` or `-h` prefix prints Catch2's usage and exits 0) or it fails with an option error such as 'Unrecognised token'.
- Work on a branch: `git switch -c refactor/cli` before Task 1, or in a worktree per `superpowers:using-git-worktrees`.
- Build directory is `build/`. Configure once with `cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build`. Rebuild with `cmake --build build`. Run every test with `ctest --test-dir build --output-on-failure`. Run one unit's cases by tag, for example `./build/demdl_tests "[wmsquery]"`.
- Headers start with `#pragma once`. Everything lives in `namespace demdl`; test helpers in `tests/` are in the global namespace.
- Include library headers as `"demdl/Name.h"`; the `src` directory is on the include path.
- Never run `dl_elevation` without `--dry-run` against the default WMS URL during this plan. It makes real requests to a public city server. The only live-request step is the optional manual check at the end of Task 13, and it needs the owner's go-ahead first.
- Commit after every task with the message given. Every commit message ends with the trailer `Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>`.
- The first configure after Task 1 fetches four dependencies and compiles Catch2; allow a few minutes.

## File structure

| Path | Responsibility | Task |
|---|---|---|
| `CMakeLists.txt` | targets, options, pinned dependencies | 1 to 5, 7 to 13 |
| `.gitignore` | build directories and TIFF outputs | 1 |
| `src/demdl/BoundingBox.h/.cpp` | validated WGS84 box, pixel scale, tolerant equality | 1 |
| `src/demdl/Config.h/.cpp` | runtime options struct, `validate()`, `parseWmsUrl()` | 2 |
| `src/demdl/WmsQuery.h/.cpp` | `percentEncode`, `formatCoordinate`, `featureInfoUrl`, `parseElevation` | 3 |
| `src/demdl/RetryPolicy.h/.cpp` | backoff schedule | 4 |
| `src/demdl/DemRaster.h/.cpp` | float32 grid, GeoTIFF I/O, resume checks | 5, 6 |
| `src/demdl/PixelFetcher.h` | `FetchResult`, fetch interface | 7 |
| `src/demdl/ProgressSink.h` | events, `RunSummary`, `StopReason`, sink interface | 7 |
| `src/demdl/StopToken.h` | shared stop flag and reason | 7 |
| `src/demdl/Downloader.h/.cpp` | row loop, pool, checkpoints, stop handling | 8 |
| `src/demdl/ElevationClient.h/.cpp` | httplib fetcher with retries and TLS | 9 |
| `src/demdl/ConsoleReporter.h/.cpp` | headless sink, `formatDuration` | 10 |
| `src/app/Cli.h/.cpp` | `parseArgs` with CLI11 | 11 |
| `src/app/main.cpp` | wiring, signals, dry run, exit codes | 12 |
| `src/preview/Shader.h` | trimmed GL shader helper | 13 |
| `src/preview/PreviewWindow.h/.cpp` | GLFW sink | 13 |
| `tests/ExampleConfig.h` | the Simon's Town Config used by several tests | 2 |
| `tests/TempDir.h` | RAII temporary directory | 6 |
| `tests/FakeFetcher.h`, `tests/RecordingSink.h` | test doubles | 7 |
| `tests/test_*.cpp` | one per unit | the unit's task |
| `CLAUDE.md` | rewritten for the new layout | 14 |
| deleted: `main.cpp`, `include/`, `opengl.cmake.in` | | 14 |

---

### Task 1: CMake skeleton, BoundingBox, first test

Implements spec sections 4.1, 4.3 (BoundingBox) and 12. After this task the repository configures, builds a library and runs one Catch2 test. The old root `main.cpp` is no longer built; it is deleted in Task 14.

**Files:**
- Modify: `CMakeLists.txt` (full replacement)
- Modify: `.gitignore` (full replacement)
- Create: `src/demdl/BoundingBox.h`
- Create: `src/demdl/BoundingBox.cpp`
- Create: `tests/test_BoundingBox.cpp`

- [ ] **Step 1: Replace `CMakeLists.txt` with the new skeleton**

```cmake
cmake_minimum_required(VERSION 3.27)
project(DownloadElevation VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# macOS CMake searches frameworks before /usr/local by default and globs every
# *.framework/Headers, so /Library/Frameworks/Mono.framework/Headers (libtiff 4.0.9)
# would win over Homebrew's tiff.h. Search frameworks last.
set(CMAKE_FIND_FRAMEWORK LAST)

if(APPLE)
  set(_dl_preview_default ON)
else()
  set(_dl_preview_default OFF)
endif()
option(DL_ELEVATION_PREVIEW "Build the OpenGL preview window" ${_dl_preview_default})
option(DL_ELEVATION_TESTS "Build the Catch2 test suite" ON)

include(FetchContent)
# cpp-httplib is pinned to the Homebrew version in use so both dependency paths compile identical
# code. FIND_PACKAGE_ARGS prefers the system package when it is 0.46.x, otherwise the pinned copy is downloaded.
FetchContent_Declare(httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG        v0.46.1
  FIND_PACKAGE_ARGS 0.46.1 NAMES httplib)
# ctpl's last commit (2015-06-12); its two upstream tags both point here, so pin the hash.
FetchContent_Declare(ctpl
  GIT_REPOSITORY https://github.com/vit-vit/ctpl.git
  GIT_TAG        437e135dbd94eb65b45533d9ce8ee28b5bd37b6d)
FetchContent_Declare(CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        v2.7.2)
FetchContent_Declare(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.16.0)
# For the downloaded cpp-httplib only: match the Homebrew package's feature set (OpenSSL and
# zlib, no Brotli or zstd) and fail at configure time instead of silently losing HTTPS.
set(HTTPLIB_REQUIRE_OPENSSL ON)
set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF)
set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF)
set(HTTPLIB_INSTALL OFF)
FetchContent_MakeAvailable(httplib ctpl CLI11)
if(DL_ELEVATION_TESTS)
  FetchContent_MakeAvailable(Catch2)
endif()

# libtiff via CMake's bundled FindTIFF. libgeotiff ships no CMake package, so locate it directly.
find_package(TIFF REQUIRED)
find_path(GEOTIFF_INCLUDE_DIR geotiff.h PATH_SUFFIXES geotiff libgeotiff REQUIRED)
find_library(GEOTIFF_LIBRARY geotiff REQUIRED)

add_library(demdl STATIC
  src/demdl/BoundingBox.cpp)
target_include_directories(demdl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${GEOTIFF_INCLUDE_DIR})
target_include_directories(demdl SYSTEM PRIVATE ${ctpl_SOURCE_DIR})
# httplib::httplib exists on both paths: the Homebrew package only exports the
# namespaced name, and the FetchContent build adds it as an alias. The bare
# "httplib" name is not a target when the system package is found, so linking it
# silently drops CPPHTTPLIB_OPENSSL_SUPPORT and SSLClient disappears.
target_link_libraries(demdl PUBLIC httplib::httplib TIFF::TIFF ${GEOTIFF_LIBRARY})
target_compile_options(demdl PRIVATE -Wall -Wextra)
target_compile_definitions(demdl PUBLIC DL_ELEVATION_VERSION="${PROJECT_VERSION}")

if(DL_ELEVATION_TESTS)
  enable_testing()
  list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
  include(Catch)
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp)
  target_link_libraries(demdl_tests PRIVATE demdl Catch2::Catch2WithMain)
  target_compile_options(demdl_tests PRIVATE -Wall -Wextra)
  catch_discover_tests(demdl_tests)
endif()
```

- [ ] **Step 2: Replace `.gitignore`**

```
cmake-build-debug*
build*/
*.tif
*.tif.tmp
```

- [ ] **Step 3: Write the failing test `tests/test_BoundingBox.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <limits>
#include <utility>

#include "demdl/BoundingBox.h"

using Catch::Matchers::WithinAbs;
using demdl::BoundingBox;

namespace {
BoundingBox simonsTown() { return {-34.1845418, -34.2163447, 18.4625244, 18.4185791}; }
}  // namespace

TEST_CASE("a valid box passes validation", "[boundingbox]") {
  CHECK(simonsTown().validate().empty());
}

TEST_CASE("north must exceed south", "[boundingbox]") {
  BoundingBox b = simonsTown();
  std::swap(b.north, b.south);
  CHECK(b.validate() == "north must be greater than south");
}

TEST_CASE("east must exceed west", "[boundingbox]") {
  BoundingBox b = simonsTown();
  std::swap(b.east, b.west);
  CHECK(b.validate() == "east must be greater than west");
}

TEST_CASE("out-of-range coordinates are rejected", "[boundingbox]") {
  BoundingBox lat = simonsTown();
  lat.north = 91.0;
  CHECK(lat.validate() == "latitudes must be within [-90, 90]");
  BoundingBox lon = simonsTown();
  lon.west = -181.0;
  CHECK(lon.validate() == "longitudes must be within [-180, 180]");
}

TEST_CASE("pixel scale divides the box by the grid", "[boundingbox]") {
  const auto s = simonsTown().pixelScale(2048, 1792);
  CHECK_THAT(s.lonPerPixel, WithinAbs((18.4625244 - 18.4185791) / 2048.0, 1e-15));
  CHECK_THAT(s.latPerPixel, WithinAbs((-34.1845418 - -34.2163447) / 1792.0, 1e-15));
}

TEST_CASE("approxEquals honours the tolerance", "[boundingbox]") {
  const BoundingBox a = simonsTown();
  BoundingBox b = a;
  b.north += 5e-10;
  CHECK(a.approxEquals(b, 1e-9));
  b.north += 1e-8;
  CHECK_FALSE(a.approxEquals(b, 1e-9));
}

TEST_CASE("non-finite values are rejected", "[boundingbox]") {
  BoundingBox nanNorth = simonsTown();
  nanNorth.north = std::numeric_limits<double>::quiet_NaN();
  CHECK(nanNorth.validate() == "bounding box values must be finite numbers");
  BoundingBox infEast = simonsTown();
  infEast.east = std::numeric_limits<double>::infinity();
  CHECK(infEast.validate() == "bounding box values must be finite numbers");
}

TEST_CASE("equal edges are rejected and range limits are inclusive", "[boundingbox]") {
  const BoundingBox flatLat{1.0, 1.0, 2.0, 1.0};  // north == south
  CHECK(flatLat.validate() == "north must be greater than south");
  const BoundingBox flatLon{2.0, 1.0, 1.0, 1.0};  // east == west
  CHECK(flatLon.validate() == "east must be greater than west");
  const BoundingBox world{90.0, -90.0, 180.0, -180.0};
  CHECK(world.validate().empty());
  BoundingBox farSouth = simonsTown();
  farSouth.south = -90.5;
  CHECK(farSouth.validate() == "latitudes must be within [-90, 90]");
  BoundingBox farEast = simonsTown();
  farEast.east = 180.5;
  CHECK(farEast.validate() == "longitudes must be within [-180, 180]");
}

TEST_CASE("approxEquals checks every edge", "[boundingbox]") {
  // The comparison is inclusive, so zero tolerance still accepts an identical box.
  CHECK(simonsTown().approxEquals(simonsTown(), 0.0));
  const BoundingBox a = simonsTown();
  const auto edges = {&BoundingBox::north, &BoundingBox::south, &BoundingBox::east,
                      &BoundingBox::west};
  for (double BoundingBox::*edge : edges) {
    BoundingBox b = a;
    b.*edge += 5e-10;
    CHECK(a.approxEquals(b, 1e-9));
    b.*edge += 1e-8;
    CHECK_FALSE(a.approxEquals(b, 1e-9));
    // A NaN edge never matches, even at a huge tolerance and in either argument order.
    BoundingBox withNan = a;
    withNan.*edge = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(a.approxEquals(withNan, 1.0));
    CHECK_FALSE(withNan.approxEquals(a, 1.0));
  }
}
```

- [ ] **Step 4: Configure to verify it fails**

Run: `cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build`
Expected: the dependencies download, then configure stops with `Cannot find source file: src/demdl/BoundingBox.cpp` (or `.../tests/test_BoundingBox.cpp` is found but the library source is missing). Either way, configure fails.

- [ ] **Step 5: Write `src/demdl/BoundingBox.h`**

```cpp
#pragma once

#include <string>

namespace demdl {

struct PixelScale {
  double lonPerPixel = 0.0;
  double latPerPixel = 0.0;
};

// A WGS84 box in decimal degrees. north > south and east > west when valid.
struct BoundingBox {
  double north = 0.0;
  double south = 0.0;
  double east = 0.0;
  double west = 0.0;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  [[nodiscard]] std::string validate() const;
  // Degrees per pixel for a grid of the given size. width and height must be positive; callers
  // validate them (Config::validate, DemRaster::create). Not checked here.
  PixelScale pixelScale(int width, int height) const;
  // Absolute, inclusive, per-edge tolerance in degrees. A NaN edge never compares equal.
  bool approxEquals(const BoundingBox& other, double tolerance) const;
};

}  // namespace demdl
```

- [ ] **Step 6: Write `src/demdl/BoundingBox.cpp`**

```cpp
#include "demdl/BoundingBox.h"

#include <cmath>

namespace demdl {

std::string BoundingBox::validate() const {
  if (!(std::isfinite(north) && std::isfinite(south) && std::isfinite(east) && std::isfinite(west))) {
    return "bounding box values must be finite numbers";
  }
  if (north < -90.0 || north > 90.0 || south < -90.0 || south > 90.0) {
    return "latitudes must be within [-90, 90]";
  }
  if (east < -180.0 || east > 180.0 || west < -180.0 || west > 180.0) {
    return "longitudes must be within [-180, 180]";
  }
  if (!(north > south)) return "north must be greater than south";
  if (!(east > west)) return "east must be greater than west";
  return {};
}

PixelScale BoundingBox::pixelScale(int width, int height) const {
  return {(east - west) / width, (north - south) / height};
}

bool BoundingBox::approxEquals(const BoundingBox& other, double tolerance) const {
  return std::fabs(north - other.north) <= tolerance && std::fabs(south - other.south) <= tolerance &&
         std::fabs(east - other.east) <= tolerance && std::fabs(west - other.west) <= tolerance;
}

}  // namespace demdl
```

- [ ] **Step 7: Configure, build, run the test**

Run: `cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build && cmake --build build && ./build/demdl_tests "[boundingbox]"`
Expected: build succeeds (Catch2 compiles on the first build), then `All tests passed (N assertions in 9 test cases)`.

Run: `ctest --test-dir build --output-on-failure`
Expected: `100% tests passed out of 9`.

- [ ] **Step 8: Commit**

```bash
git add CMakeLists.txt .gitignore src/demdl/BoundingBox.h src/demdl/BoundingBox.cpp tests/test_BoundingBox.cpp
git commit -m "build: new CMake skeleton with demdl library, Catch2 tests and BoundingBox" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 2: Config and WMS URL parsing

Implements spec section 4.3 (Config) and the default values in section 5.

**Files:**
- Create: `src/demdl/Config.h`
- Create: `src/demdl/Config.cpp`
- Create: `tests/ExampleConfig.h`
- Create: `tests/test_Config.cpp`
- Modify: `CMakeLists.txt` (source lists)

- [ ] **Step 1: Write `tests/ExampleConfig.h`**

```cpp
#pragma once

#include "demdl/Config.h"

// Today's Simon's Town run with the latitude names corrected (the old code
// stored the smaller latitude in a variable called "north").
inline demdl::Config exampleConfig() {
  demdl::Config c;
  c.box = {-34.1845418, -34.2163447, 18.4625244, 18.4185791};
  c.width = 2048;
  c.height = 1792;
  c.output = "simonstown.tif";
  return c;
}
```

- [ ] **Step 2: Write the failing test `tests/test_Config.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <utility>

#include "ExampleConfig.h"
#include "demdl/Config.h"

using demdl::Config;
using demdl::parseWmsUrl;

TEST_CASE("the default WMS URL splits into origin and path", "[config]") {
  const auto ep = parseWmsUrl(Config{}.wmsUrl);
  REQUIRE(ep.has_value());
  CHECK(ep->origin == "https://cityimg.capetown.gov.za");
  CHECK(ep->path == "/erdas-iws/ogc/wms/GeoSpatial%20Datasets");
}

TEST_CASE("a URL with a port keeps the port in the origin", "[config]") {
  const auto ep = parseWmsUrl("http://127.0.0.1:8080/wms");
  REQUIRE(ep.has_value());
  CHECK(ep->origin == "http://127.0.0.1:8080");
  CHECK(ep->path == "/wms");
  // Both ends of the port range, and '-', '_' and '.' inside a host name, are accepted.
  CHECK(parseWmsUrl("http://a-b.c_d:1/x").has_value());
  CHECK(parseWmsUrl("https://h:65535").has_value());
}

TEST_CASE("bracketed IPv6 hosts are accepted", "[config]") {
  const auto withPort = parseWmsUrl("https://[::1]:8443/wms");
  REQUIRE(withPort.has_value());
  CHECK(withPort->origin == "https://[::1]:8443");
  CHECK(withPort->path == "/wms");
  const auto withoutPort = parseWmsUrl("http://[2001:db8::1]/wms");
  REQUIRE(withoutPort.has_value());
  CHECK(withoutPort->origin == "http://[2001:db8::1]");
}

TEST_CASE("a URL without a path gets the root path", "[config]") {
  const auto ep = parseWmsUrl("https://host.test");
  REQUIRE(ep.has_value());
  CHECK(ep->origin == "https://host.test");
  CHECK(ep->path == "/");
}

TEST_CASE("unsupported or malformed URLs are rejected", "[config]") {
  CHECK_FALSE(parseWmsUrl("ftp://host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https:///wms").has_value());
  CHECK_FALSE(parseWmsUrl("host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://:8080/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test:/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test?x=1").has_value());
  CHECK_FALSE(parseWmsUrl("HTTPS://host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test:abc/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test::8080/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://user:pw@host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test:65536/wms").has_value());
  CHECK_FALSE(parseWmsUrl("http://127.0.0.1:0/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test/wms?x=1").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test/wms#frag").has_value());
  CHECK_FALSE(parseWmsUrl("https://[::1/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test:8080:9090/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://fe80::1/wms").has_value());
  // Whitespace and control characters are refused in the path as well as in the host.
  CHECK_FALSE(parseWmsUrl("https://host.test/a b").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test/wms\r\nX: 1").has_value());
}

TEST_CASE("the example configuration is valid", "[config]") {
  CHECK(exampleConfig().validate().empty());
}

TEST_CASE("validation reports the first problem", "[config]") {
  Config threads = exampleConfig();
  threads.threads = 0;
  CHECK(threads.validate() == "threads must be at least 1");

  Config grid = exampleConfig();
  grid.width = 0;
  CHECK(grid.validate() == "width and height must be positive");

  Config url = exampleConfig();
  url.wmsUrl = "not a url";
  CHECK(url.validate() == "wms-url must look like scheme://host[:port]/path with scheme http or https");

  Config retries = exampleConfig();
  retries.maxRetries = -1;
  CHECK(retries.validate() == "max-retries must be at least 0");

  Config manyRetries = exampleConfig();
  manyRetries.maxRetries = 101;
  CHECK(manyRetries.validate() == "max-retries must be at most 100");
  manyRetries.maxRetries = 100;
  CHECK(manyRetries.validate().empty());

  Config failures = exampleConfig();
  failures.maxConsecutiveFailures = 0;
  CHECK(failures.validate() == "max-consecutive-failures must be at least 1");

  Config output = exampleConfig();
  output.output.clear();
  CHECK(output.validate() == "output path must not be empty");

  Config swapped = exampleConfig();
  std::swap(swapped.box.north, swapped.box.south);
  CHECK(swapped.validate() == "north must be greater than south");

  Config layer = exampleConfig();
  layer.layer.clear();
  CHECK(layer.validate() == "layer must not be empty");
  layer.layer = "ws:dem;v2";
  CHECK(layer.validate() == "layer must not contain ';'");

  Config height = exampleConfig();
  height.height = -1;
  CHECK(height.validate() == "width and height must be positive");

  Config manyThreads = exampleConfig();
  manyThreads.threads = 65;
  CHECK(manyThreads.validate() == "threads must be at most 64");
  manyThreads.threads = 64;
  CHECK(manyThreads.validate().empty());

  Config directory = exampleConfig();
  directory.output = "out/";
  CHECK(directory.validate() == "output must name a file");

  // Two faults at once: the grid is checked before the thread count, which pins the check order.
  Config both = exampleConfig();
  both.width = 0;
  both.threads = 0;
  CHECK(both.validate() == "width and height must be positive");
}
```

- [ ] **Step 3: Add the files to CMake**

Replace the `add_library(demdl STATIC` block's source list and the `add_executable(demdl_tests` list:

```cmake
add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp)
```

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp)
```

- [ ] **Step 4: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/demdl/Config.cpp` (CMake re-runs automatically because CMakeLists.txt changed).

- [ ] **Step 5: Write `src/demdl/Config.h`**

```cpp
#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "demdl/BoundingBox.h"

namespace demdl {

struct WmsEndpoint {
  std::string origin;  // "scheme://host[:port]", what httplib::Client is constructed with
  std::string path;    // whatever followed the authority, as given; "/" when the URL had none
};

// Splits "scheme://host[:port][/path]". The scheme is http or https (case-sensitive), the host is a
// hostname (A-Za-z0-9._-) or a bracketed IPv6 literal, and the port, if given, is 1 to 65535.
// Userinfo, queries, fragments, whitespace and control characters are refused, and so is anything
// else that does not fit: the result is then nullopt.
[[nodiscard]] std::optional<WmsEndpoint> parseWmsUrl(const std::string& url);

// Every runtime option, mirroring the command-line flags one to one.
// No command-line parsing lives here.
struct Config {
  BoundingBox box;
  int width = 0;
  int height = 0;
  std::filesystem::path output;  // relative to the current directory; must name a file
  std::string wmsUrl = "https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets";
  std::string layer = "Elevation_CCT_GLM2019_5m_DEM";  // no ';': it separates the GeoTIFF description
  int threads = 8;     // capped at 64 to go easy on a public server
  int maxRetries = 8;  // retries after the first attempt, at most 100; 0 means a single attempt
  int maxConsecutiveFailures = 100;
  std::string caBundle;  // empty means the system trust store; no effect when insecure is set
  bool insecure = false;
  bool preview = false;
  bool overwrite = false;
  bool dryRun = false;
  bool verbose = false;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  [[nodiscard]] std::string validate() const;
};

}  // namespace demdl
```

- [ ] **Step 6: Write `src/demdl/Config.cpp`**

```cpp
#include "demdl/Config.h"

#include <cstddef>

namespace demdl {

namespace {

// The character tests are ASCII only, so the result does not depend on the locale.
bool isDigit(char c) { return c >= '0' && c <= '9'; }

bool isHexDigit(char c) { return isDigit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }

// One or more characters from A-Za-z0-9._-
bool isHostname(const std::string& host) {
  if (host.empty()) return false;
  for (const char c : host) {
    const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
    if (!letter && !isDigit(c) && c != '.' && c != '_' && c != '-') return false;
  }
  return true;
}

// "[", one or more hex digits or colons, "]".
bool isIpv6Literal(const std::string& host) {
  if (host.size() < 3 || host.front() != '[' || host.back() != ']') return false;
  for (std::size_t i = 1; i + 1 < host.size(); ++i) {
    if (!isHexDigit(host[i]) && host[i] != ':') return false;
  }
  return true;
}

// 1 to 5 decimal digits with a value from 1 to 65535.
bool isPort(const std::string& text) {
  if (text.empty() || text.size() > 5) return false;
  int value = 0;
  for (const char c : text) {
    if (!isDigit(c)) return false;
    value = value * 10 + (c - '0');
  }
  return value >= 1 && value <= 65535;
}

// "host" or "host:port". The port follows the first ':' that is not inside the brackets of an
// IPv6 literal.
bool isAuthority(const std::string& authority) {
  std::size_t hostEnd = 0;
  if (!authority.empty() && authority.front() == '[') {
    const auto close = authority.find(']');
    if (close == std::string::npos) return false;
    hostEnd = close + 1;
  } else {
    hostEnd = authority.find(':');
    if (hostEnd == std::string::npos) hostEnd = authority.size();
  }
  const std::string host = authority.substr(0, hostEnd);
  if (!isIpv6Literal(host) && !isHostname(host)) return false;
  if (hostEnd == authority.size()) return true;
  return authority[hostEnd] == ':' && isPort(authority.substr(hostEnd + 1));
}

}  // namespace

std::optional<WmsEndpoint> parseWmsUrl(const std::string& url) {
  // Grammar: "http" or "https", "://", a host with an optional ":port", then nothing or a path
  // starting with "/". The client appends its own query to the path, so a query or fragment in the
  // URL is refused, as are userinfo, whitespace and control characters.
  for (const char c : url) {
    const auto byte = static_cast<unsigned char>(c);
    if (byte <= 0x20 || byte == 0x7F || c == '?' || c == '#') return std::nullopt;
  }
  const auto schemeEnd = url.find("://");
  if (schemeEnd == std::string::npos) return std::nullopt;
  const std::string scheme = url.substr(0, schemeEnd);
  if (scheme != "http" && scheme != "https") return std::nullopt;
  const std::string rest = url.substr(schemeEnd + 3);
  const auto slash = rest.find('/');
  const std::string authority = rest.substr(0, slash);
  if (!isAuthority(authority)) return std::nullopt;
  WmsEndpoint ep;
  ep.origin = scheme + "://" + authority;
  ep.path = slash == std::string::npos ? std::string("/") : rest.substr(slash);
  return ep;
}

std::string Config::validate() const {
  if (const std::string err = box.validate(); !err.empty()) return err;
  if (width <= 0 || height <= 0) return "width and height must be positive";
  if (output.empty()) return "output path must not be empty";
  if (!output.has_filename()) return "output must name a file";
  if (!parseWmsUrl(wmsUrl)) {
    return "wms-url must look like scheme://host[:port]/path with scheme http or https";
  }
  if (layer.empty()) return "layer must not be empty";
  if (layer.find(';') != std::string::npos) return "layer must not contain ';'";
  if (threads < 1) return "threads must be at least 1";
  if (threads > 64) return "threads must be at most 64";
  if (maxRetries < 0) return "max-retries must be at least 0";
  if (maxRetries > 100) return "max-retries must be at most 100";
  if (maxConsecutiveFailures < 1) return "max-consecutive-failures must be at least 1";
  return {};
}

}  // namespace demdl
```

- [ ] **Step 7: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[config]"`
Expected: `All tests passed (N assertions in 7 test cases)`.

- [ ] **Step 8: Commit**

```bash
git add CMakeLists.txt src/demdl/Config.h src/demdl/Config.cpp tests/ExampleConfig.h tests/test_Config.cpp
git commit -m "feat: Config struct with validation and WMS URL parsing" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 3: WmsQuery, URL building and response parsing

Implements spec section 4.3 (WmsQuery) and the golden URL in section 11.

**Files:**
- Create: `src/demdl/WmsQuery.h`
- Create: `src/demdl/WmsQuery.cpp`
- Create: `tests/test_WmsQuery.cpp`
- Modify: `CMakeLists.txt` (source lists)

- [ ] **Step 1: Write the failing test `tests/test_WmsQuery.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <limits>
#include <stdexcept>
#include <string>

#include "ExampleConfig.h"
#include "demdl/WmsQuery.h"

using Catch::Matchers::WithinAbs;
using demdl::ParseResult;
using demdl::featureInfoUrl;
using demdl::formatCoordinate;
using demdl::parseElevation;
using demdl::percentEncode;

TEST_CASE("the golden GetFeatureInfo URL for the example configuration", "[wmsquery]") {
  const std::string expected =
      "/erdas-iws/ogc/wms/GeoSpatial%20Datasets?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetFeatureInfo"
      "&BBOX=-34.216344700%2C18.418579100%2C-34.184541800%2C18.462524400"
      "&CRS=EPSG%3A4326&WIDTH=2048&HEIGHT=1792"
      "&LAYERS=Elevation_CCT_GLM2019_5m_DEM&STYLES=&FORMAT=image%2Fpng"
      "&QUERY_LAYERS=Elevation_CCT_GLM2019_5m_DEM&INFO_FORMAT=text%2Fplain&I=0&J=0";
  CHECK(featureInfoUrl(exampleConfig(), 0, 0) == expected);
}

TEST_CASE("I and J carry the pixel column and row", "[wmsquery]") {
  const std::string url = featureInfoUrl(exampleConfig(), 5, 7);
  CHECK(url.find("&I=5&J=7") != std::string::npos);
}

TEST_CASE("the layer name is percent-encoded in both LAYERS and QUERY_LAYERS", "[wmsquery]") {
  demdl::Config cfg = exampleConfig();
  cfg.layer = "My Layer/1";
  const std::string url = featureInfoUrl(cfg, 0, 0);
  CHECK(url.find("&LAYERS=My%20Layer%2F1&") != std::string::npos);
  CHECK(url.find("&QUERY_LAYERS=My%20Layer%2F1&") != std::string::npos);
}

TEST_CASE("featureInfoUrl throws when the WMS URL does not parse", "[wmsquery]") {
  demdl::Config cfg = exampleConfig();
  cfg.wmsUrl = "not a url";
  CHECK_THROWS_AS(featureInfoUrl(cfg, 0, 0), std::invalid_argument);
}

TEST_CASE("percentEncode keeps unreserved characters and encodes everything else", "[wmsquery]") {
  CHECK(percentEncode("Abc-1.2_x~") == "Abc-1.2_x~");
  CHECK(percentEncode("a b:c/d") == "a%20b%3Ac%2Fd");
  CHECK(percentEncode("\xC3\xA9") == "%C3%A9");
  CHECK(percentEncode("%") == "%25");
}

TEST_CASE("formatCoordinate uses nine fixed decimals", "[wmsquery]") {
  CHECK(formatCoordinate(-34.2163447) == "-34.216344700");
  CHECK(formatCoordinate(18.0) == "18.000000000");
}

TEST_CASE("parseElevation reads the whole number after the band marker", "[wmsquery]") {
  const auto a = parseElevation("GetFeatureInfo results:\nBAND 0: 123.45\n");
  REQUIRE(a.kind == ParseResult::Kind::Value);
  CHECK_THAT(a.value, WithinAbs(123.45, 1e-12));

  const auto b = parseElevation("BAND 0: -1.5");
  REQUIRE(b.kind == ParseResult::Kind::Value);
  CHECK_THAT(b.value, WithinAbs(-1.5, 1e-12));

  const auto c = parseElevation("BAND 0: 1086.25\nBAND 1: 0\n");
  REQUIRE(c.kind == ParseResult::Kind::Value);
  CHECK_THAT(c.value, WithinAbs(1086.25, 1e-12));
}

TEST_CASE("the value is the numeric prefix of the marker's line", "[wmsquery]") {
  // Text after the number is ignored.
  const auto withUnit = parseElevation("BAND 0: 12.5 m");
  REQUIRE(withUnit.kind == ParseResult::Kind::Value);
  CHECK_THAT(withUnit.value, WithinAbs(12.5, 1e-12));

  // The number must be on the marker's own line.
  CHECK(parseElevation("BAND 0:\n5").kind == ParseResult::Kind::ParseError);
}

TEST_CASE("a plain body without the band marker is NoData", "[wmsquery]") {
  CHECK(parseElevation("GetFeatureInfo results:\n").kind == ParseResult::Kind::NoData);
  CHECK(parseElevation("GetFeatureInfo results:\nno bands\n").kind == ParseResult::Kind::NoData);
}

TEST_CASE("an empty, blank or markup body is a parse error, not NoData", "[wmsquery]") {
  CHECK(parseElevation("").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("   \n").kind == ParseResult::Kind::ParseError);

  const std::string html = "<html><body>Service temporarily unavailable</body></html>";
  CHECK(parseElevation(html).kind == ParseResult::Kind::ParseError);

  const std::string owsReport = "  <?xml version=\"1.0\"?><ows:ExceptionReport/>";
  CHECK(parseElevation(owsReport).kind == ParseResult::Kind::ParseError);

  // The band marker still wins over markup.
  const auto wrapped = parseElevation("<pre>BAND 0: 42.5</pre>");
  REQUIRE(wrapped.kind == ParseResult::Kind::Value);
  CHECK_THAT(wrapped.value, WithinAbs(42.5, 1e-12));
}

TEST_CASE("a WMS service exception is a parse error, not NoData", "[wmsquery]") {
  const std::string report =
      R"(<?xml version="1.0"?><ServiceExceptionReport>)"
      R"(<ServiceException code="LayerNotDefined">no such layer</ServiceException>)"
      R"(</ServiceExceptionReport>)";
  CHECK(parseElevation(report).kind == ParseResult::Kind::ParseError);

  // The band marker wins when it is present, even if the body also mentions ServiceException.
  const auto both = parseElevation("ServiceException\nBAND 0: 42.5\n");
  REQUIRE(both.kind == ParseResult::Kind::Value);
  CHECK_THAT(both.value, WithinAbs(42.5, 1e-12));
}

TEST_CASE("a band marker without a usable number is a parse error", "[wmsquery]") {
  CHECK(parseElevation("BAND 0: abc").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0:").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: nan").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: inf").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: -inf").kind == ParseResult::Kind::ParseError);
}

TEST_CASE("a band value outside float range is a parse error", "[wmsquery]") {
  CHECK(parseElevation("BAND 0: 1e300").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: -1e300").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: 3.5e38").kind == ParseResult::Kind::ParseError);

  // FLT_MAX itself is still a value, and it survives the cast to float.
  const auto limit = parseElevation("BAND 0: 3.4028234663852886e38");
  REQUIRE(limit.kind == ParseResult::Kind::Value);
  CHECK(static_cast<float>(limit.value) == std::numeric_limits<float>::max());
}

TEST_CASE("a default-constructed ParseResult is a parse error, never a 0 m value", "[wmsquery]") {
  CHECK(ParseResult{}.kind == ParseResult::Kind::ParseError);
}
```

- [ ] **Step 2: Add the files to CMake**

```cmake
add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp
  src/demdl/WmsQuery.cpp)
```

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp)
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/demdl/WmsQuery.cpp`.

- [ ] **Step 4: Write `src/demdl/WmsQuery.h`**

```cpp
#pragma once

#include <string>
#include <string_view>

#include "demdl/Config.h"

namespace demdl {

struct ParseResult {
  enum class Kind { Value, NoData, ParseError };
  Kind kind = Kind::ParseError;  // so a value-initialised result is never a 0 m Value
  double value = 0.0;            // valid when kind == Value
};

// RFC 3986 percent-encoding: unreserved characters pass through, everything else becomes %XX.
std::string percentEncode(std::string_view s);

// Degrees with nine fixed decimals (about 0.1 mm), used in URLs and user-facing reports alike.
// Relies on the C locale for the decimal point; nothing in this program calls setlocale.
std::string formatCoordinate(double degrees);

// Request target (path plus query) of the WMS 1.3.0 GetFeatureInfo request for pixel column x, row y.
// The box is sent as south,west,north,east, the axis order WMS 1.3.0 requires for EPSG:4326.
// cfg.layer is taken unencoded and percent-encoded here, so a name that is already encoded would
// be double-encoded. Requires 0 <= x < cfg.width and 0 <= y < cfg.height; this is not checked,
// because the Downloader supplies raster indices and pool jobs must not throw.
// Throws std::invalid_argument when cfg.wmsUrl does not parse.
std::string featureInfoUrl(const Config& cfg, int x, int y);

// Classifies the text/plain body of a GetFeatureInfo response.
// "BAND 0:" anywhere in the body is a band, whatever else the body says. The numeric prefix of the
// rest of that line is the value: trailing text such as " m" is ignored, and the number must be on
// the marker's own line. It is a Value only when finite and within float range (the raster stores
// float32, so a Value always fits in a float); no number, "nan", "inf" and 1e300 are ParseError.
// Without the marker the body is NoData only when it is not blank (empty or only whitespace), does
// not contain "ServiceException" and does not start with markup ('<' as the first non-whitespace
// character). NoData is stored as -32768 and never re-requested, so any other body, such as an
// HTML error page or an OWS exception report, is ParseError and the caller retries it instead.
// Relies on the C locale for the decimal point; nothing in this program calls setlocale.
ParseResult parseElevation(std::string_view body);

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/WmsQuery.cpp`**

```cpp
#include "demdl/WmsQuery.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace demdl {

namespace {

// The character tests are ASCII only, so the result does not depend on the locale: bytes at or
// above 0x80 are never unreserved and are always percent-encoded.
bool isUnreserved(unsigned char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
         c == '.' || c == '_' || c == '~';
}

}  // namespace

std::string percentEncode(std::string_view s) {
  static constexpr char hex[] = "0123456789ABCDEF";
  std::string out;
  out.reserve(s.size());
  for (const unsigned char c : s) {
    if (isUnreserved(c)) {
      out.push_back(static_cast<char>(c));
    } else {
      out.push_back('%');
      out.push_back(hex[c >> 4]);
      out.push_back(hex[c & 0x0F]);
    }
  }
  return out;
}

std::string formatCoordinate(double degrees) {
  // snprintf, because std::to_chars for double is unavailable on this libc++ at the default
  // deployment target.
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.9f", degrees);
  return buf;
}

std::string featureInfoUrl(const Config& cfg, int x, int y) {
  const auto ep = parseWmsUrl(cfg.wmsUrl);
  if (!ep) throw std::invalid_argument("invalid WMS URL: " + cfg.wmsUrl);
  const std::string layer = percentEncode(cfg.layer);
  std::string url = ep->path;
  url += "?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetFeatureInfo&BBOX=";
  url += formatCoordinate(cfg.box.south) + "%2C" + formatCoordinate(cfg.box.west) + "%2C" +
         formatCoordinate(cfg.box.north) + "%2C" + formatCoordinate(cfg.box.east);
  url += "&CRS=EPSG%3A4326&WIDTH=" + std::to_string(cfg.width) + "&HEIGHT=" + std::to_string(cfg.height);
  url += "&LAYERS=" + layer + "&STYLES=&FORMAT=image%2Fpng&QUERY_LAYERS=" + layer;
  url += "&INFO_FORMAT=text%2Fplain&I=" + std::to_string(x) + "&J=" + std::to_string(y);
  return url;
}

ParseResult parseElevation(std::string_view body) {
  static constexpr std::string_view marker = "BAND 0:";
  const auto pos = body.find(marker);
  if (pos == std::string_view::npos) {
    // NoData is stored as -32768 and never re-requested, so a body that is not plainly "no band
    // here" must be retried instead: an empty or blank one, markup (an HTML error page or an OWS
    // exception report; text/plain was requested, so markup is never a valid answer) and anything
    // that mentions a ServiceException (WMS servers report faults such as an unknown layer so).
    const auto first = body.find_first_not_of(" \t\n\v\f\r");
    if (first == std::string_view::npos || body[first] == '<' ||
        body.find("ServiceException") != std::string_view::npos) {
      return {ParseResult::Kind::ParseError, 0.0};
    }
    return {ParseResult::Kind::NoData, 0.0};
  }
  const std::string_view rest = body.substr(pos + marker.size());
  const auto lineEnd = rest.find_first_of("\r\n");
  // strtod needs a null-terminated string; libc++ on AppleClang 15 has no floating-point from_chars.
  const std::string token(rest.substr(0, lineEnd));
  const char* begin = token.c_str();
  char* end = nullptr;
  const double value = std::strtod(begin, &end);
  // strtod also accepts "nan" and "inf". Neither is an elevation, and a NaN would read as an
  // unfetched pixel and be requested again on every run. The raster stores float32, so values
  // above FLT_MAX are refused too. A handful just above would still round to FLT_MAX; that does
  // not matter, because no elevation is anywhere near 1e38.
  if (end == begin || !std::isfinite(value) || std::fabs(value) > std::numeric_limits<float>::max()) {
    return {ParseResult::Kind::ParseError, 0.0};
  }
  return {ParseResult::Kind::Value, value};
}

}  // namespace demdl
```

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[wmsquery]"`
Expected: `All tests passed (N assertions in 14 test cases)`.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/demdl/WmsQuery.h src/demdl/WmsQuery.cpp tests/test_WmsQuery.cpp
git commit -m "feat: WmsQuery builds GetFeatureInfo URLs and parses band values" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 4: RetryPolicy

Implements spec section 4.3 (RetryPolicy) and the schedule test in section 11.

**Files:**
- Create: `src/demdl/RetryPolicy.h`
- Create: `src/demdl/RetryPolicy.cpp`
- Create: `tests/test_RetryPolicy.cpp`
- Modify: `CMakeLists.txt` (source lists)

- [ ] **Step 1: Write the failing test `tests/test_RetryPolicy.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <limits>

#include "demdl/RetryPolicy.h"

using namespace std::chrono_literals;
using demdl::RetryPolicy;

TEST_CASE("the schedule doubles from the base and caps at the maximum", "[retrypolicy]") {
  const RetryPolicy p;  // defaults: 500 ms base, 30 s cap, 20 % jitter
  const std::chrono::milliseconds expected[] = {500ms, 1000ms, 2000ms, 4000ms, 8000ms, 16000ms, 30000ms, 30000ms};
  for (int attempt = 0; attempt < 8; ++attempt) {
    CAPTURE(attempt);
    CHECK(p.delayFor(attempt, 0.5) == expected[attempt]);
  }
}

TEST_CASE("jitter scales the delay between 1 - jitter and 1 + jitter", "[retrypolicy]") {
  const RetryPolicy p;
  CHECK(p.delayFor(0, 0.0) == 400ms);
  CHECK(p.delayFor(0, 1.0) == 600ms);
}

TEST_CASE("a custom policy is honoured", "[retrypolicy]") {
  RetryPolicy p;
  p.baseDelay = 1ms;
  p.maxDelay = 3ms;
  p.jitter = 0.0;
  CHECK(p.delayFor(0, 0.9) == 1ms);
  CHECK(p.delayFor(1, 0.9) == 2ms);
  CHECK(p.delayFor(2, 0.9) == 3ms);
  CHECK(p.delayFor(40, 0.9) == 3ms);
}

TEST_CASE("jitter applies after the cap, so the longest sleep is maxDelay * (1 + jitter)", "[retrypolicy]") {
  const RetryPolicy p;
  CHECK(p.delayFor(7, 0.0) == 24000ms);
  CHECK(p.delayFor(7, 1.0) == 36000ms);
}

TEST_CASE("an attempt beyond the range of double still yields the cap", "[retrypolicy]") {
  const RetryPolicy p;
  CHECK(p.delayFor(1024, 0.5) == 30000ms);
  CHECK(p.delayFor(std::numeric_limits<int>::max(), 0.5) == 30000ms);
}
```

- [ ] **Step 2: Add the files to CMake**

```cmake
add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp
  src/demdl/WmsQuery.cpp
  src/demdl/RetryPolicy.cpp)
```

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp)
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/demdl/RetryPolicy.cpp`.

- [ ] **Step 4: Write `src/demdl/RetryPolicy.h`**

```cpp
#pragma once

#include <chrono>

namespace demdl {

// Exponential backoff: min(base * 2^attempt, max), scaled by 1 + jitter * (2 * unitRandom - 1).
struct RetryPolicy {
  int maxRetries = 8;  // retries after the first attempt
  std::chrono::milliseconds baseDelay{500};
  // The cap is applied before jitter, so the longest sleep is maxDelay * (1 + jitter).
  std::chrono::milliseconds maxDelay{30000};
  double jitter = 0.2;

  // Delay before retry number `attempt` (0-based). Requires attempt >= 0, baseDelay > 0 and
  // 0 <= jitter <= 1; these are not checked. The caller supplies unitRandom in [0, 1], which keeps
  // the schedule testable. The interval is closed on purpose: std::uniform_real_distribution can
  // return exactly 1.0 on this standard library.
  [[nodiscard]] std::chrono::milliseconds delayFor(int attempt, double unitRandom) const;
};

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/RetryPolicy.cpp`**

```cpp
#include "demdl/RetryPolicy.h"

#include <algorithm>
#include <cmath>

namespace demdl {

std::chrono::milliseconds RetryPolicy::delayFor(int attempt, double unitRandom) const {
  const double capped = std::min(static_cast<double>(baseDelay.count()) * std::ldexp(1.0, attempt),
                                 static_cast<double>(maxDelay.count()));
  const double scaled = capped * (1.0 + jitter * (2.0 * unitRandom - 1.0));
  return std::chrono::milliseconds(std::llround(scaled));
}

}  // namespace demdl
```

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[retrypolicy]"`
Expected: `All tests passed (N assertions in 5 test cases)`.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/demdl/RetryPolicy.h src/demdl/RetryPolicy.cpp tests/test_RetryPolicy.cpp
git commit -m "feat: RetryPolicy with capped exponential backoff and jitter" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 5: DemRaster in memory

Implements the in-memory half of spec section 4.3 (DemRaster) and the `IMAGEDESCRIPTION` format from section 6. GeoTIFF I/O follows in Task 6.

**Files:**
- Create: `src/demdl/DemRaster.h`
- Create: `src/demdl/DemRaster.cpp`
- Create: `tests/test_DemRaster.cpp`
- Modify: `CMakeLists.txt` (source lists)

- [ ] **Step 1: Write the failing test `tests/test_DemRaster.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

#include "demdl/DemRaster.h"

using demdl::BoundingBox;
using demdl::DemRaster;
using demdl::RasterMetadata;

namespace {
const BoundingBox kBox{-34.0, -34.3, 18.5, 18.1};
}  // namespace

TEST_CASE("a new raster is entirely NaN", "[demraster]") {
  const auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  CHECK(r.width() == 4);
  CHECK(r.height() == 3);
  CHECK(r.pixelCount() == 12);
  CHECK(r.countNaN() == 12);
  CHECK(std::isnan(r.at(3, 2)));
  CHECK(r.box().approxEquals(kBox, 0.0));
  CHECK(r.metadata().layer == "LayerA");
  CHECK(r.metadata().wmsUrl == "https://example.test/wms");
}

TEST_CASE("set and at address pixels by column and row", "[demraster]") {
  auto r = DemRaster::create(4, 3, kBox, {});
  r.set(1, 2, 12.5f);
  CHECK(r.at(1, 2) == 12.5f);
  CHECK(std::isnan(r.at(2, 1)));
  CHECK(r.countNaN() == 11);
}

TEST_CASE("todoInRow and rowsWithWork describe the remaining NaN pixels", "[demraster]") {
  auto r = DemRaster::create(4, 3, kBox, {});
  for (int x = 0; x < 4; ++x) r.set(x, 1, 1.0f);  // row 1 complete
  r.set(0, 2, DemRaster::kNoData);                 // row 2 partially done
  r.set(2, 2, 5.0f);
  CHECK(r.todoInRow(0) == std::vector<int>{0, 1, 2, 3});
  CHECK(r.todoInRow(1).empty());
  CHECK(r.todoInRow(2) == std::vector<int>{1, 3});
  CHECK(r.rowsWithWork() == std::vector<int>{0, 2});
}

TEST_CASE("create rejects non-positive dimensions", "[demraster]") {
  CHECK_THROWS_AS(DemRaster::create(0, 3, kBox, {}), std::invalid_argument);
  CHECK_THROWS_AS(DemRaster::create(3, -1, kBox, {}), std::invalid_argument);
}

TEST_CASE("the image description round-trips and keeps the URL intact", "[demraster]") {
  const RasterMetadata meta{"Elevation_CCT_GLM2019_5m_DEM", "https://host.test/a;wms=b?x=1"};
  const std::string text = demdl::formatImageDescription(meta);
  CHECK(text == "dl_elevation;layer=Elevation_CCT_GLM2019_5m_DEM;wms=https://host.test/a;wms=b?x=1");
  const RasterMetadata back = demdl::parseImageDescription(text);
  CHECK(back.layer == meta.layer);
  CHECK(back.wmsUrl == meta.wmsUrl);
  CHECK(demdl::parseImageDescription("unrelated text").layer.empty());
  CHECK(demdl::parseImageDescription("unrelated text").wmsUrl.empty());
}
```

- [ ] **Step 2: Add the files to CMake**

```cmake
add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp
  src/demdl/WmsQuery.cpp
  src/demdl/RetryPolicy.cpp
  src/demdl/DemRaster.cpp)
```

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp
    tests/test_DemRaster.cpp)
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/demdl/DemRaster.cpp`.

- [ ] **Step 4: Write `src/demdl/DemRaster.h`**

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "demdl/BoundingBox.h"

namespace demdl {

struct RasterMetadata {
  std::string layer;
  std::string wmsUrl;
};

// "dl_elevation;layer=<layer>;wms=<url>". Fields are separated by ';'. wms is always last and
// extends to the end of the string so a URL may contain any character.
[[nodiscard]] std::string formatImageDescription(const RasterMetadata& meta);
[[nodiscard]] RasterMetadata parseImageDescription(std::string_view description);

// A float32 elevation grid in metres. Row 0 is the north edge, column 0 the west edge.
// NaN means "no value yet"; kNoData means the server has no elevation there.
class DemRaster {
 public:
  static constexpr float kNoData = -32768.0f;

  // Every pixel starts as NaN. Throws std::invalid_argument for a non-positive size.
  [[nodiscard]] static DemRaster create(int width, int height, const BoundingBox& box, RasterMetadata meta);

  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] const BoundingBox& box() const { return box_; }
  [[nodiscard]] const RasterMetadata& metadata() const { return meta_; }

  [[nodiscard]] float at(int x, int y) const { return data_[index(x, y)]; }
  void set(int x, int y, float value) { data_[index(x, y)] = value; }

  [[nodiscard]] std::vector<int> todoInRow(int row) const;  // columns still NaN, ascending
  [[nodiscard]] std::vector<int> rowsWithWork() const;      // rows with at least one NaN, ascending
  [[nodiscard]] std::uint64_t countNaN() const;
  [[nodiscard]] std::uint64_t pixelCount() const { return data_.size(); }

 private:
  DemRaster() = default;
  std::size_t index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  }

  int width_ = 0;
  int height_ = 0;
  BoundingBox box_;
  RasterMetadata meta_;
  std::vector<float> data_;
};

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/DemRaster.cpp`**

```cpp
#include "demdl/DemRaster.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace demdl {

std::string formatImageDescription(const RasterMetadata& meta) {
  return "dl_elevation;layer=" + meta.layer + ";wms=" + meta.wmsUrl;
}

RasterMetadata parseImageDescription(std::string_view description) {
  RasterMetadata meta;
  std::string_view head = description;
  const auto wmsPos = description.find(";wms=");
  if (wmsPos != std::string_view::npos) {
    meta.wmsUrl = std::string(description.substr(wmsPos + 5));
    head = description.substr(0, wmsPos);
  }
  const auto layerPos = head.find("layer=");
  if (layerPos != std::string_view::npos) {
    const std::string_view value = head.substr(layerPos + 6);
    meta.layer = std::string(value.substr(0, value.find(';')));
  }
  return meta;
}

DemRaster DemRaster::create(int width, int height, const BoundingBox& box, RasterMetadata meta) {
  if (width <= 0 || height <= 0) throw std::invalid_argument("raster dimensions must be positive");
  DemRaster r;
  r.width_ = width;
  r.height_ = height;
  r.box_ = box;
  r.meta_ = std::move(meta);
  r.data_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                 std::numeric_limits<float>::quiet_NaN());
  return r;
}

std::vector<int> DemRaster::todoInRow(int row) const {
  std::vector<int> columns;
  for (int x = 0; x < width_; ++x) {
    if (std::isnan(at(x, row))) columns.push_back(x);
  }
  return columns;
}

std::vector<int> DemRaster::rowsWithWork() const {
  std::vector<int> rows;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (std::isnan(at(x, y))) {
        rows.push_back(y);
        break;
      }
    }
  }
  return rows;
}

std::uint64_t DemRaster::countNaN() const {
  std::uint64_t n = 0;
  for (const float v : data_) {
    if (std::isnan(v)) ++n;
  }
  return n;
}

}  // namespace demdl
```

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[demraster]"`
Expected: `All tests passed (N assertions in 5 test cases)`.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/demdl/DemRaster.h src/demdl/DemRaster.cpp tests/test_DemRaster.cpp
git commit -m "feat: DemRaster float32 grid with NaN to-do tracking and metadata string" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 6: DemRaster GeoTIFF save, load and resume checks

Implements spec sections 6 and 7 (file format, atomic save, resume verification) and the DemRaster tests in section 11.

**Files:**
- Modify: `src/demdl/DemRaster.h` (full replacement)
- Modify: `src/demdl/DemRaster.cpp` (full replacement)
- Create: `tests/TempDir.h`
- Modify: `tests/test_DemRaster.cpp` (append tests)

- [ ] **Step 1: Write `tests/TempDir.h`**

```cpp
#pragma once

#include <filesystem>
#include <random>
#include <string>

// A fresh directory under the system temp dir, removed with everything in it on destruction.
struct TempDir {
  std::filesystem::path path;

  TempDir() {
    std::random_device rd;
    path = std::filesystem::temp_directory_path() / ("demdl-test-" + std::to_string(rd()));
    std::filesystem::create_directories(path);
  }
  ~TempDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
};
```

- [ ] **Step 2: Append the failing tests to `tests/test_DemRaster.cpp`**

Replace the includes and using-declarations at the top of the file (everything above `namespace {`) with:

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <signal.h>
#include <sys/resource.h>

#include <cmath>
#include <cstdint>
#include <filesystem>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "ExampleConfig.h"
#include "TempDir.h"
#include "demdl/Config.h"
#include "demdl/DemRaster.h"

using Catch::Matchers::ContainsSubstring;
using demdl::BoundingBox;
using demdl::Config;
using demdl::DemRaster;
using demdl::RasterMetadata;
```

Append these helpers and test cases at the end of the file:

```cpp
namespace {

// Writes a 4x2 single-band float32 GeoTIFF straight through libtiff, as another program might, with
// the given west edge and north -34. description may be null, which leaves the tag out.
void writeFloatGeoTiff(const std::filesystem::path& file, double west, const char* description) {
  TIFF* tif = XTIFFOpen(file.string().c_str(), "w");
  REQUIRE(tif != nullptr);
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, 4u);
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, 2u);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 32);
  TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 2u);
  if (description != nullptr) TIFFSetField(tif, TIFFTAG_IMAGEDESCRIPTION, description);
  const double tiepoints[6] = {0.0, 0.0, 0.0, west, -34.0, 0.0};
  const double scale[3] = {0.1, 0.1, 0.0};
  REQUIRE(TIFFSetField(tif, TIFFTAG_GEOTIEPOINTS, 6, tiepoints) == 1);
  REQUIRE(TIFFSetField(tif, TIFFTAG_GEOPIXELSCALE, 3, scale) == 1);
  std::vector<float> row(4, 0.0f);
  for (uint32_t y = 0; y < 2; ++y) REQUIRE(TIFFWriteScanline(tif, row.data(), y, 0) >= 0);
  XTIFFClose(tif);
}

// Lowers this process's file-size limit, with SIGXFSZ ignored so that a write past the limit fails
// with EFBIG instead of killing the process. Restores both on destruction, however the scope ends.
class FileSizeLimit {
 public:
  explicit FileSizeLimit(std::uintmax_t bytes) {
    struct sigaction ignoreAction {};
    ignoreAction.sa_handler = SIG_IGN;
    sigemptyset(&ignoreAction.sa_mask);
    ignored_ = sigaction(SIGXFSZ, &ignoreAction, &previousAction_) == 0;
    if (ignored_ && getrlimit(RLIMIT_FSIZE, &previousLimit_) == 0) {
      rlimit lowered = previousLimit_;
      lowered.rlim_cur = static_cast<rlim_t>(bytes);
      lowered_ = setrlimit(RLIMIT_FSIZE, &lowered) == 0;
    }
  }
  ~FileSizeLimit() {
    if (lowered_) setrlimit(RLIMIT_FSIZE, &previousLimit_);
    if (ignored_) sigaction(SIGXFSZ, &previousAction_, nullptr);
  }
  FileSizeLimit(const FileSizeLimit&) = delete;
  FileSizeLimit& operator=(const FileSizeLimit&) = delete;

  bool active() const { return lowered_; }

 private:
  struct sigaction previousAction_ {};
  rlimit previousLimit_{};
  bool ignored_ = false;
  bool lowered_ = false;
};

struct LimitedSave {
  bool limited = false;  // the limit was in force during the save
  std::string error;     // empty when the save succeeded
};

// Saves r to file under a file-size limit. Nothing is reported while the limit is in force, so no
// test output can be cut short by it.
LimitedSave saveUnderLimit(const DemRaster& r, const std::filesystem::path& file, std::uintmax_t bytes) {
  LimitedSave result;
  const FileSizeLimit limit(bytes);
  result.limited = limit.active();
  try {
    r.save(file);
  } catch (const std::runtime_error& e) {
    result.error = e.what();
  }
  return result;
}

}  // namespace

TEST_CASE("save and load round-trip values, NaN and NoData", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  r.set(1, 1, 12.5f);
  r.set(2, 0, DemRaster::kNoData);
  r.set(0, 2, -3.25f);

  r.save(file);

  CHECK(std::filesystem::exists(file));
  CHECK_FALSE(std::filesystem::exists(dir.path / "dem.tif.tmp"));
  const auto back = DemRaster::load(file);
  CHECK(back.width() == 4);
  CHECK(back.height() == 3);
  CHECK(back.box().approxEquals(kBox, 1e-9));
  CHECK(back.at(1, 1) == 12.5f);
  CHECK(back.at(2, 0) == DemRaster::kNoData);
  CHECK(back.at(0, 2) == -3.25f);
  CHECK(std::isnan(back.at(0, 0)));
  CHECK(back.countNaN() == 9);
  CHECK(back.metadata().layer == "LayerA");
  CHECK(back.metadata().wmsUrl == "https://example.test/wms");
}

TEST_CASE("a multi-strip raster round-trips every pixel", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  Config cfg = exampleConfig();
  cfg.height = 8;
  auto r = DemRaster::create(cfg.width, cfg.height, cfg.box, {cfg.layer, cfg.wmsUrl});
  for (int y = 0; y < cfg.height; ++y) {
    for (int x = 0; x < cfg.width; ++x) {
      if ((x + y) % 2 == 0) r.set(x, y, x * 0.5f + y);
    }
  }
  r.set(1, 0, DemRaster::kNoData);
  r.set(1000, 3, DemRaster::kNoData);
  r.set(2047, 7, DemRaster::kNoData);
  r.save(file);

  TIFF* tif = XTIFFOpen(file.string().c_str(), "r");
  REQUIRE(tif != nullptr);
  CHECK(TIFFNumberOfStrips(tif) > 1);
  XTIFFClose(tif);

  const auto back = DemRaster::load(file);
  REQUIRE(back.width() == cfg.width);
  REQUIRE(back.height() == cfg.height);
  int mismatches = 0;
  for (int y = 0; y < cfg.height; ++y) {
    for (int x = 0; x < cfg.width; ++x) {
      const float want = r.at(x, y);
      const float got = back.at(x, y);
      if (std::isnan(want) ? !std::isnan(got) : got != want) ++mismatches;
    }
  }
  CHECK(mismatches == 0);
  CHECK(back.countNaN() == r.countNaN());
  std::string why;
  CHECK(back.matches(cfg, why));
  CHECK(why.empty());
}

TEST_CASE("the file carries GeoTIFF keys, the tiepoint and the NoData tag", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  DemRaster::create(4, 3, kBox, {}).save(file);

  TIFF* tif = XTIFFOpen(file.string().c_str(), "r");
  REQUIRE(tif != nullptr);
  GTIF* gtif = GTIFNew(tif);
  REQUIRE(gtif != nullptr);

  unsigned short v = 0;
  CHECK(GTIFKeyGetSHORT(gtif, GTModelTypeGeoKey, &v, 0, 1) == 1);
  CHECK(v == ModelTypeGeographic);
  CHECK(GTIFKeyGetSHORT(gtif, GTRasterTypeGeoKey, &v, 0, 1) == 1);
  CHECK(v == RasterPixelIsArea);
  CHECK(GTIFKeyGetSHORT(gtif, GeographicTypeGeoKey, &v, 0, 1) == 1);
  CHECK(v == 4326);

  uint16_t count = 0;
  double* tie = nullptr;
  REQUIRE(TIFFGetField(tif, TIFFTAG_GEOTIEPOINTS, &count, &tie) == 1);
  CHECK(count == 6);
  CHECK(tie[3] == 18.1);
  CHECK(tie[4] == -34.0);
  double* scale = nullptr;
  REQUIRE(TIFFGetField(tif, TIFFTAG_GEOPIXELSCALE, &count, &scale) == 1);
  CHECK(count == 3);
  CHECK(scale[0] == (18.5 - 18.1) / 4.0);
  CHECK(scale[1] == (-34.0 - -34.3) / 3.0);

  char* nodata = nullptr;
  REQUIRE(TIFFGetField(tif, TIFFTAG_GDAL_NODATA, &nodata) == 1);
  CHECK(std::string(nodata) == "-32768");
  uint16_t sampleFormat = 0;
  REQUIRE(TIFFGetField(tif, TIFFTAG_SAMPLEFORMAT, &sampleFormat) == 1);
  CHECK(sampleFormat == SAMPLEFORMAT_IEEEFP);
  char* software = nullptr;
  REQUIRE(TIFFGetField(tif, TIFFTAG_SOFTWARE, &software) == 1);
  CHECK(std::string(software) == std::string("dl_elevation ") + DL_ELEVATION_VERSION);

  GTIFFree(gtif);
  XTIFFClose(tif);
}

TEST_CASE("matches accepts the same configuration and explains mismatches", "[demraster]") {
  const Config cfg = exampleConfig();
  const auto r = DemRaster::create(cfg.width, cfg.height, cfg.box, {cfg.layer, cfg.wmsUrl});
  std::string why;
  CHECK(r.matches(cfg, why));
  CHECK(why.empty());

  Config shifted = cfg;
  shifted.box.north += 0.01;
  CHECK_FALSE(r.matches(shifted, why));
  CHECK_THAT(why, ContainsSubstring("north: file -34.184541800, requested -34.174541800"));

  Config other = cfg;
  other.width = 100;
  other.layer = "Other";
  CHECK_FALSE(r.matches(other, why));
  CHECK_THAT(why, ContainsSubstring("width: file 2048, requested 100"));
  CHECK_THAT(why, ContainsSubstring("layer: file Elevation_CCT_GLM2019_5m_DEM, requested Other"));

  Config mirror = cfg;
  mirror.wmsUrl = "https://mirror.test/wms";
  CHECK(r.matches(mirror, why));
  CHECK_FALSE(r.wmsUrlWarning(mirror).empty());
  CHECK(r.wmsUrlWarning(cfg).empty());

  BoundingBox nanBox = cfg.box;
  nanBox.west = std::numeric_limits<double>::quiet_NaN();
  const auto nanEdge = DemRaster::create(cfg.width, cfg.height, nanBox, {cfg.layer, cfg.wmsUrl});
  CHECK_FALSE(nanEdge.matches(cfg, why));
  CHECK_THAT(why, ContainsSubstring("west: file nan"));

  const auto noLayer = DemRaster::create(cfg.width, cfg.height, cfg.box, {"", cfg.wmsUrl});
  CHECK_FALSE(noLayer.matches(cfg, why));
  CHECK_THAT(why, ContainsSubstring("layer: file (none), requested Elevation_CCT_GLM2019_5m_DEM"));
}

TEST_CASE("legacy RGB16 files are rejected with a clear message", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "legacy.tif";
  {
    TIFF* tif = TIFFOpen(file.string().c_str(), "w");
    REQUIRE(tif != nullptr);
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, 4u);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, 2u);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 3);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 16);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
    TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 2u);
    std::vector<uint16_t> row(4 * 3, 0);
    for (uint32_t y = 0; y < 2; ++y) REQUIRE(TIFFWriteScanline(tif, row.data(), y, 0) >= 0);
    TIFFClose(tif);
  }
  CHECK_THROWS_WITH(DemRaster::load(file), ContainsSubstring("RGB16"));
}

TEST_CASE("a file with an unusable tiepoint is rejected", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "nan-tiepoint.tif";
  writeFloatGeoTiff(file, std::numeric_limits<double>::quiet_NaN(), "dl_elevation;layer=;wms=");
  CHECK_THROWS_WITH(DemRaster::load(file), ContainsSubstring("invalid bounding box"));
}

TEST_CASE("a GeoTIFF that dl_elevation did not write is rejected", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "foreign.tif";
  writeFloatGeoTiff(file, 18.1, nullptr);
  CHECK_THROWS_WITH(DemRaster::load(file), ContainsSubstring("not written by dl_elevation"));
}

TEST_CASE("oversized dimensions are rejected before allocation", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "oversized.tif";
  {
    // libtiff will not read back a directory without strip offsets, so one four-byte placeholder
    // strip stands in for the 8 GiB that a single row of this width would take.
    TIFF* tif = XTIFFOpen(file.string().c_str(), "w");
    REQUIRE(tif != nullptr);
    TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, 2147483649u);
    TIFFSetField(tif, TIFFTAG_IMAGELENGTH, 1u);
    TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 32);
    TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 1u);
    TIFFSetField(tif, TIFFTAG_IMAGEDESCRIPTION, "dl_elevation;layer=;wms=");
    float placeholder = 0.0f;
    REQUIRE(TIFFWriteRawStrip(tif, 0, &placeholder, 4) == 4);
    XTIFFClose(tif);
  }
  CHECK_THROWS_WITH(DemRaster::load(file), ContainsSubstring("is too large"));
}

TEST_CASE("loading a missing file reports the path and libtiff's reason", "[demraster]") {
  TempDir dir;
  const auto absent = dir.path / "absent.tif";
  CHECK_THROWS_WITH(DemRaster::load(absent), ContainsSubstring("absent.tif"));
  CHECK_THROWS_WITH(DemRaster::load(absent), ContainsSubstring("No such file or directory"));
}

TEST_CASE("a failed directory write throws and keeps the previous checkpoint", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  r.set(1, 1, 12.5f);
  r.save(file);

  // libtiff writes the directory after all strip data, so a limit one byte short of the complete file
  // admits the header and every strip but not the whole directory. The size is taken from a
  // successful save of exactly the content the failing save writes.
  r.set(1, 1, 99.0f);
  const auto probe = dir.path / "probe.tif";
  r.save(probe);
  const std::uintmax_t complete = std::filesystem::file_size(probe);

  const LimitedSave attempt = saveUnderLimit(r, file, complete - 1);
  REQUIRE(attempt.limited);
  CHECK_THAT(attempt.error, ContainsSubstring("cannot finish writing"));
  CHECK_THAT(attempt.error, ContainsSubstring("(File too large)"));
  CHECK_FALSE(std::filesystem::exists(dir.path / "dem.tif.tmp"));
  const auto back = DemRaster::load(file);
  CHECK(back.at(1, 1) == 12.5f);
}

TEST_CASE("a failed strip write throws and keeps the previous checkpoint", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  auto r = DemRaster::create(2048, 8, kBox, {"LayerA", "https://example.test/wms"});
  r.set(5, 5, 12.5f);
  r.save(file);
  const std::uintmax_t complete = std::filesystem::file_size(file);

  // Half the file ends inside the strip data, so a row write fails before the directory is reached.
  r.set(5, 5, 99.0f);
  const LimitedSave attempt = saveUnderLimit(r, file, complete / 2);
  REQUIRE(attempt.limited);
  CHECK_THAT(attempt.error, ContainsSubstring("write failed on row"));
  CHECK_THAT(attempt.error, ContainsSubstring("(File too large)"));
  CHECK_FALSE(std::filesystem::exists(dir.path / "dem.tif.tmp"));
  const auto back = DemRaster::load(file);
  CHECK(back.at(5, 5) == 12.5f);
}

TEST_CASE("a checkpoint that cannot be created leaves no temp file behind", "[demraster]") {
  TempDir dir;
  const auto file = dir.path / "dem.tif";
  auto r = DemRaster::create(4, 3, kBox, {"LayerA", "https://example.test/wms"});
  r.set(1, 1, 12.5f);
  r.save(file);

  // With no room at all, libtiff creates and truncates the temp file, then fails to write its header.
  r.set(1, 1, 99.0f);
  const LimitedSave attempt = saveUnderLimit(r, file, 0);
  REQUIRE(attempt.limited);
  CHECK_THAT(attempt.error, ContainsSubstring("cannot create"));
  CHECK_THAT(attempt.error, ContainsSubstring("(File too large)"));
  CHECK_FALSE(std::filesystem::exists(dir.path / "dem.tif.tmp"));
  const auto back = DemRaster::load(file);
  CHECK(back.at(1, 1) == 12.5f);
}
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with errors such as `no member named 'save' in 'demdl::DemRaster'` and `no member named 'load'`.

- [ ] **Step 4: Replace `src/demdl/DemRaster.h`**

```cpp
#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "demdl/BoundingBox.h"
#include "demdl/Config.h"

// Fast-math lets the compiler fold std::isnan to false, and then no pixel would ever be fetched.
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "NaN marks pixels still to fetch; do not build with -ffast-math or -ffinite-math-only"
#endif

namespace demdl {

struct RasterMetadata {
  std::string layer;
  std::string wmsUrl;
};

// "dl_elevation;layer=<layer>;wms=<url>". Fields are separated by ';'. wms is always last and
// extends to the end of the string so a URL may contain any character.
[[nodiscard]] std::string formatImageDescription(const RasterMetadata& meta);
[[nodiscard]] RasterMetadata parseImageDescription(std::string_view description);

// A float32 elevation grid in metres. Row 0 is the north edge, column 0 the west edge.
// NaN means "no value yet"; kNoData means the server has no elevation there.
// Threading: set may run concurrently on distinct cells, and nothing else may overlap a set.
// The first load or save sets libtiff up for the whole process: it registers the GDAL_NODATA tag,
// installs an error handler that collects libtiff's errors for the exception messages instead of
// printing them, and silences libtiff's warnings.
class DemRaster {
 public:
  static constexpr float kNoData = -32768.0f;
  static constexpr double kBoxTolerance = 1e-9;  // degrees, for resume checks

  // Every pixel starts as NaN. Throws std::invalid_argument for a non-positive size.
  [[nodiscard]] static DemRaster create(int width, int height, const BoundingBox& box, RasterMetadata meta);

  // Reads a file written by save(). Throws std::runtime_error with a user-facing message when the
  // file cannot be opened, is not a single-band float32 GeoTIFF (including the previous RGB16
  // format), was not written by dl_elevation, is too large for int indices, lacks the georeferencing
  // tags, or describes a box that fails BoundingBox::validate(). libtiff's explanation, when it gave
  // one, is appended.
  [[nodiscard]] static DemRaster load(const std::filesystem::path& path);

  // Writes <path>.tmp beside the output, syncs it to disk, then renames it over <path>, so a crash
  // mid-write leaves the previous file intact. The rename replaces a symlink at <path> with a regular
  // file, and a single file bind-mounted at <path> cannot be replaced. Throws std::runtime_error on
  // any failure, with libtiff's explanation and, for a failed write, the operating system's reason
  // appended.
  void save(const std::filesystem::path& path) const;

  // True when this raster can be resumed for cfg. Otherwise `why` lists every differing field,
  // one per line, each indented by two spaces and ending in a newline.
  [[nodiscard]] bool matches(const Config& cfg, std::string& why) const;

  // Non-empty when the file was made from a different WMS URL; a warning, not an error.
  [[nodiscard]] std::string wmsUrlWarning(const Config& cfg) const;

  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] const BoundingBox& box() const { return box_; }
  [[nodiscard]] const RasterMetadata& metadata() const { return meta_; }

  // at and set require 0 <= x < width() and 0 <= y < height(). Not checked (debug builds assert
  // it), because the Downloader supplies indices from todoInRow and pool jobs must not throw.
  [[nodiscard]] float at(int x, int y) const { return data_[index(x, y)]; }
  void set(int x, int y, float value) { data_[index(x, y)] = value; }

  // Columns still NaN, ascending. Requires 0 <= row < height(); not checked, as for at and set.
  [[nodiscard]] std::vector<int> todoInRow(int row) const;
  [[nodiscard]] std::vector<int> rowsWithWork() const;  // rows with at least one NaN, ascending
  [[nodiscard]] std::uint64_t countNaN() const;
  [[nodiscard]] std::uint64_t pixelCount() const { return data_.size(); }

 private:
  DemRaster() = default;
  std::size_t index(int x, int y) const {
    assert(x >= 0 && x < width_ && y >= 0 && y < height_);
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  }

  int width_ = 0;
  int height_ = 0;
  BoundingBox box_;
  RasterMetadata meta_;
  std::vector<float> data_;
};

}  // namespace demdl
```

- [ ] **Step 5: Replace `src/demdl/DemRaster.cpp`**

```cpp
#include "demdl/DemRaster.h"

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <unistd.h>
#include <xtiffio.h>

#include <cerrno>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "demdl/WmsQuery.h"

namespace demdl {

namespace {

struct TiffCloser {
  void operator()(TIFF* t) const {
    if (t != nullptr) XTIFFClose(t);
  }
};
using TiffHandle = std::unique_ptr<TIFF, TiffCloser>;

struct GtifCloser {
  void operator()(GTIF* g) const {
    if (g != nullptr) GTIFFree(g);
  }
};
using GtifHandle = std::unique_ptr<GTIF, GtifCloser>;

// Removes a temporary file when it goes out of scope, unless it was marked complete.
class TempFileGuard {
 public:
  explicit TempFileGuard(std::filesystem::path path) : path_(std::move(path)) {}
  ~TempFileGuard() {
    if (complete_) return;
    std::error_code ignored;
    std::filesystem::remove(path_, ignored);
  }
  TempFileGuard(const TempFileGuard&) = delete;
  TempFileGuard& operator=(const TempFileGuard&) = delete;

  void markComplete() { complete_ = true; }

 private:
  std::filesystem::path path_;
  bool complete_ = false;
};

// The libtiff errors raised on this thread since takeTiffError() last ran, joined by "; ".
thread_local std::string lastTiffError;

// Installed in place of libtiff's default handler, which prints to stderr, so that the text reaches
// the user in an exception message instead of interrupting the progress output. errno is preserved
// for the caller, which reports the operating system's reason for a failed write.
void captureTiffError(const char* module, const char* fmt, va_list ap) {
  const int savedErrno = errno;
  char text[1024];
  std::vsnprintf(text, sizeof text, fmt, ap);
  try {
    if (!lastTiffError.empty()) lastTiffError += "; ";
    if (module != nullptr) lastTiffError += std::string(module) + ": ";
    lastTiffError += text;
  } catch (...) {
    // Out of memory: the text is lost, but the caller still reports the failure itself.
  }
  errno = savedErrno;
}

// Returns the libtiff errors collected on this thread and forgets them.
std::string takeTiffError() {
  std::string text;
  text.swap(lastTiffError);
  return text;
}

// Throws what, then ": <libtiff text>" when libtiff reported errors since the last take, then
// " (<reason>)" when osError, an errno value, is not zero.
[[noreturn]] void fail(const std::string& what, int osError = 0) {
  std::string message = what;
  if (const std::string text = takeTiffError(); !text.empty()) message += ": " + text;
  if (osError != 0) message += " (" + std::generic_category().message(osError) + ")";
  throw std::runtime_error(message);
}

// libtiff does not know the GDAL_NODATA tag: TIFFSetField refuses it and a reader sees an anonymous
// field. This tag extender registers it on every TIFF handle opened after ensureTagsRegistered(),
// then calls the extender installed before it, libgeotiff's, which registers the GeoTIFF tags.
TIFFExtendProc g_parentExtender = nullptr;

void registerGdalTags(TIFF* tif) {
  static const TIFFFieldInfo fields[] = {
      {TIFFTAG_GDAL_NODATA, -1, -1, TIFF_ASCII, FIELD_CUSTOM, 1, 0, const_cast<char*>("GDALNoDataValue")},
  };
  TIFFMergeFieldInfo(tif, fields, 1);
  if (g_parentExtender != nullptr) g_parentExtender(tif);
}

void ensureTagsRegistered() {
  static std::once_flag once;
  std::call_once(once, [] {
    // libgeotiff's first-time setup is not thread-safe, so it is forced to run here, once. The
    // extender order is therefore fixed: ours runs first, then libgeotiff's.
    XTIFFInitialize();
    g_parentExtender = TIFFSetTagExtender(registerGdalTags);
    TIFFSetErrorHandler(captureTiffError);
    // Our own files raise no warnings; another program's file would otherwise print them during load.
    TIFFSetWarningHandler(nullptr);
  });
}

}  // namespace

std::string formatImageDescription(const RasterMetadata& meta) {
  return "dl_elevation;layer=" + meta.layer + ";wms=" + meta.wmsUrl;
}

RasterMetadata parseImageDescription(std::string_view description) {
  RasterMetadata meta;
  std::string_view head = description;
  const auto wmsPos = description.find(";wms=");
  if (wmsPos != std::string_view::npos) {
    meta.wmsUrl = std::string(description.substr(wmsPos + 5));
    head = description.substr(0, wmsPos);
  }
  const auto layerPos = head.find("layer=");
  if (layerPos != std::string_view::npos) {
    const std::string_view value = head.substr(layerPos + 6);
    meta.layer = std::string(value.substr(0, value.find(';')));
  }
  return meta;
}

DemRaster DemRaster::create(int width, int height, const BoundingBox& box, RasterMetadata meta) {
  if (width <= 0 || height <= 0) throw std::invalid_argument("raster dimensions must be positive");
  DemRaster r;
  r.width_ = width;
  r.height_ = height;
  r.box_ = box;
  r.meta_ = std::move(meta);
  r.data_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                 std::numeric_limits<float>::quiet_NaN());
  return r;
}

DemRaster DemRaster::load(const std::filesystem::path& path) {
  ensureTagsRegistered();
  takeTiffError();  // forget whatever earlier calls on this thread left behind
  TiffHandle tif(XTIFFOpen(path.string().c_str(), "r"));
  if (!tif) fail("cannot open " + path.string());
  TIFF* t = tif.get();

  uint32_t width = 0;
  uint32_t height = 0;
  uint16_t samplesPerPixel = 0;
  uint16_t bitsPerSample = 0;
  uint16_t sampleFormat = 0;
  TIFFGetField(t, TIFFTAG_IMAGEWIDTH, &width);
  TIFFGetField(t, TIFFTAG_IMAGELENGTH, &height);
  TIFFGetFieldDefaulted(t, TIFFTAG_SAMPLESPERPIXEL, &samplesPerPixel);
  TIFFGetFieldDefaulted(t, TIFFTAG_BITSPERSAMPLE, &bitsPerSample);
  TIFFGetFieldDefaulted(t, TIFFTAG_SAMPLEFORMAT, &sampleFormat);
  if (samplesPerPixel != 1 || bitsPerSample != 32 || sampleFormat != SAMPLEFORMAT_IEEEFP) {
    fail(path.string() +
         " is not a dl_elevation float32 GeoTIFF; files from the previous RGB16 "
         "format cannot be resumed");
  }
  // Checked before anything is allocated, so that another program's file is never resumed.
  char* description = nullptr;
  if (!TIFFGetField(t, TIFFTAG_IMAGEDESCRIPTION, &description) || description == nullptr ||
      !std::string_view(description).starts_with("dl_elevation;")) {
    fail(path.string() + " was not written by dl_elevation; choose another --output or use "
                         "--overwrite");
  }
  RasterMetadata meta = parseImageDescription(description);
  constexpr auto kMaxSide = static_cast<uint32_t>(std::numeric_limits<int>::max());
  if (width > kMaxSide || height > kMaxSide) {
    fail(path.string() + " is too large: " + std::to_string(width) + " x " + std::to_string(height) +
         " pixels");
  }
  if (width == 0 || height == 0) fail(path.string() + " has an empty image");

  uint16_t tieCount = 0;
  double* tiepoints = nullptr;
  if (!TIFFGetField(t, TIFFTAG_GEOTIEPOINTS, &tieCount, &tiepoints) || tieCount < 6 || tiepoints[0] != 0.0 ||
      tiepoints[1] != 0.0) {
    fail(path.string() + " has no usable GeoTIFF tiepoint");
  }
  uint16_t scaleCount = 0;
  double* scale = nullptr;
  if (!TIFFGetField(t, TIFFTAG_GEOPIXELSCALE, &scaleCount, &scale) || scaleCount < 2) {
    fail(path.string() + " has no GeoTIFF pixel scale");
  }
  BoundingBox box;
  box.west = tiepoints[3];
  box.north = tiepoints[4];
  box.east = box.west + scale[0] * width;
  box.south = box.north - scale[1] * height;
  if (const std::string problem = box.validate(); !problem.empty()) {
    fail(path.string() + " has an invalid bounding box: " + problem);
  }

  DemRaster r = create(static_cast<int>(width), static_cast<int>(height), box, std::move(meta));
  for (uint32_t y = 0; y < height; ++y) {
    if (TIFFReadScanline(t, &r.data_[r.index(0, static_cast<int>(y))], y, 0) < 0) {
      fail("read failed on row " + std::to_string(y) + " of " + path.string());
    }
  }
  return r;
}

void DemRaster::save(const std::filesystem::path& path) const {
  ensureTagsRegistered();
  takeTiffError();  // forget whatever earlier calls on this thread left behind
  const std::filesystem::path tmp = path.string() + ".tmp";
  // Declared before the TIFF handle, so that on failure the file is removed after the handle closes.
  TempFileGuard tmpGuard(tmp);
  {
    // errno is cleared just before each call that can fail in the operating system and read just
    // after a failure, so that, for example, a full disk is named as the cause.
    const std::string tmpName = tmp.string();
    errno = 0;
    TiffHandle tif(XTIFFOpen(tmpName.c_str(), "w"));
    const int openError = errno;
    if (!tif) fail("cannot create " + tmpName, openError);
    TIFF* t = tif.get();
    // Checked so that a tag libtiff does not know fails the save instead of being left out.
    const auto setField = [&](const char* name, uint32_t tag, auto... values) {
      if (TIFFSetField(t, tag, values...) != 1) {
        fail(std::string("cannot set tag ") + name + " on " + tmp.string());
      }
    };
    setField("IMAGEWIDTH", TIFFTAG_IMAGEWIDTH, static_cast<uint32_t>(width_));
    setField("IMAGELENGTH", TIFFTAG_IMAGELENGTH, static_cast<uint32_t>(height_));
    setField("SAMPLESPERPIXEL", TIFFTAG_SAMPLESPERPIXEL, 1);
    setField("BITSPERSAMPLE", TIFFTAG_BITSPERSAMPLE, 32);
    setField("SAMPLEFORMAT", TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    setField("PHOTOMETRIC", TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    setField("PLANARCONFIG", TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    setField("COMPRESSION", TIFFTAG_COMPRESSION, COMPRESSION_NONE);
    setField("ORIENTATION", TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    setField("ROWSPERSTRIP", TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(t, 0));

    const PixelScale scale = box_.pixelScale(width_, height_);
    const double tiepoints[6] = {0.0, 0.0, 0.0, box_.west, box_.north, 0.0};
    const double pixelScale[3] = {scale.lonPerPixel, scale.latPerPixel, 0.0};
    setField("GEOTIEPOINTS", TIFFTAG_GEOTIEPOINTS, 6, tiepoints);
    setField("GEOPIXELSCALE", TIFFTAG_GEOPIXELSCALE, 3, pixelScale);
    static_assert(DemRaster::kNoData == -32768.0f);  // the GDAL_NODATA text below spells out kNoData
    setField("GDAL_NODATA", TIFFTAG_GDAL_NODATA, "-32768");
    const std::string description = formatImageDescription(meta_);
    setField("IMAGEDESCRIPTION", TIFFTAG_IMAGEDESCRIPTION, description.c_str());
    const std::string software = std::string("dl_elevation ") + DL_ELEVATION_VERSION;
    setField("SOFTWARE", TIFFTAG_SOFTWARE, software.c_str());

    {
      GtifHandle gtif(GTIFNew(t));
      if (!gtif) fail("cannot attach GeoTIFF keys to " + tmp.string());
      const auto setKey = [&](const char* name, geokey_t key, int value) {
        if (GTIFKeySet(gtif.get(), key, TYPE_SHORT, 1, value) != 1) {
          fail(std::string("cannot set GeoTIFF key ") + name + " on " + tmp.string());
        }
      };
      setKey("GTModelTypeGeoKey", GTModelTypeGeoKey, ModelTypeGeographic);
      setKey("GTRasterTypeGeoKey", GTRasterTypeGeoKey, RasterPixelIsArea);
      setKey("GeographicTypeGeoKey", GeographicTypeGeoKey, GCS_WGS_84);
      if (GTIFWriteKeys(gtif.get()) != 1) fail("cannot write the GeoTIFF keys to " + tmp.string());
    }

    std::vector<float> row(static_cast<std::size_t>(width_));
    for (int y = 0; y < height_; ++y) {
      std::memcpy(row.data(), &data_[index(0, y)], row.size() * sizeof(float));
      errno = 0;
      if (TIFFWriteScanline(t, row.data(), static_cast<uint32_t>(y), 0) < 0) {
        const int writeError = errno;
        fail("write failed on row " + std::to_string(y) + " of " + tmp.string(), writeError);
      }
    }
    // XTIFFClose swallows errors from writing the last strip and the directory. Flushing first
    // surfaces them, so a failed write never reaches the rename and the previous checkpoint survives.
    errno = 0;
    if (TIFFFlush(t) != 1) {
      const int flushError = errno;
      fail("cannot finish writing " + tmp.string(), flushError);
    }
    // The data must be on disk before the rename makes it the checkpoint.
    if (fsync(TIFFFileno(t)) != 0) {
      const int syncError = errno;
      fail("cannot finish writing " + tmp.string(), syncError);
    }
  }
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) {
    throw std::runtime_error("cannot rename " + tmp.string() + " to " + path.string() + ": " +
                             ec.message());
  }
  tmpGuard.markComplete();
}

bool DemRaster::matches(const Config& cfg, std::string& why) const {
  why.clear();
  if (width_ != cfg.width) {
    why += "  width: file " + std::to_string(width_) + ", requested " + std::to_string(cfg.width) + "\n";
  }
  if (height_ != cfg.height) {
    why += "  height: file " + std::to_string(height_) + ", requested " + std::to_string(cfg.height) + "\n";
  }
  const auto edge = [&why](const char* name, double file, double requested) {
    // Negated so that a NaN edge, for which every comparison is false, is reported as a mismatch.
    if (!(std::fabs(file - requested) <= kBoxTolerance)) {
      why += std::string("  ") + name + ": file " + formatCoordinate(file) + ", requested " +
             formatCoordinate(requested) + "\n";
    }
  };
  edge("north", box_.north, cfg.box.north);
  edge("south", box_.south, cfg.box.south);
  edge("east", box_.east, cfg.box.east);
  edge("west", box_.west, cfg.box.west);
  if (meta_.layer != cfg.layer) {
    const std::string fileLayer = meta_.layer.empty() ? "(none)" : meta_.layer;
    why += "  layer: file " + fileLayer + ", requested " + cfg.layer + "\n";
  }
  return why.empty();
}

std::string DemRaster::wmsUrlWarning(const Config& cfg) const {
  if (meta_.wmsUrl.empty() || meta_.wmsUrl == cfg.wmsUrl) return {};
  return "output was created from " + meta_.wmsUrl + " but this run uses " + cfg.wmsUrl;
}

std::vector<int> DemRaster::todoInRow(int row) const {
  std::vector<int> columns;
  for (int x = 0; x < width_; ++x) {
    if (std::isnan(at(x, row))) columns.push_back(x);
  }
  return columns;
}

std::vector<int> DemRaster::rowsWithWork() const {
  std::vector<int> rows;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (std::isnan(at(x, y))) {
        rows.push_back(y);
        break;
      }
    }
  }
  return rows;
}

std::uint64_t DemRaster::countNaN() const {
  std::uint64_t n = 0;
  for (const float v : data_) {
    if (std::isnan(v)) ++n;
  }
  return n;
}

}  // namespace demdl
```

Notes for the engineer: `XTIFFOpen` registers the GeoTIFF tags with libtiff, which is why the geo tags are set and read through plain `TIFFSetField`/`TIFFGetField`; the tag count for them is a `uint16_t` and the data a `double*`. The `GTIFKeySet` value arguments are plain `int`s through varargs, so the enum constants are passed as written.

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[demraster]"`
Expected: `All tests passed (N assertions in 17 test cases)`.

- [ ] **Step 7: Commit**

```bash
git add src/demdl/DemRaster.h src/demdl/DemRaster.cpp tests/TempDir.h tests/test_DemRaster.cpp
git commit -m "feat: DemRaster saves and loads single-band float32 GeoTIFF with resume checks" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 7: Interfaces, stop token and test doubles

Implements spec sections 4.3 (PixelFetcher), 8 (events and sink) and the stop flag from section 9, plus the doubles the Downloader tests need. Deviations 1 to 3 from the header apply here.

**Files:**
- Create: `src/demdl/PixelFetcher.h`
- Create: `src/demdl/ProgressSink.h`
- Create: `src/demdl/StopToken.h`
- Create: `tests/FakeFetcher.h`
- Create: `tests/RecordingSink.h`
- Create: `tests/test_StopToken.cpp`
- Modify: `CMakeLists.txt` (test source list)

- [ ] **Step 1: Write the failing test `tests/test_StopToken.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "FakeFetcher.h"
#include "RecordingSink.h"
#include "demdl/StopToken.h"

using demdl::StopReason;
using demdl::StopToken;

TEST_CASE("a stop token keeps the first reason", "[stoptoken]") {
  StopToken t;
  CHECK_FALSE(t.requested());
  CHECK(t.reason() == StopReason::None);
  t.request(StopReason::Signal);
  CHECK(t.requested());
  CHECK(t.reason() == StopReason::Signal);
  t.request(StopReason::WindowClosed);
  CHECK(t.requested());
  CHECK(t.reason() == StopReason::Signal);
}

TEST_CASE("the flag handed to fetchers follows the token", "[stoptoken]") {
  StopToken t;
  const std::atomic<bool>& flag = t.flag();
  CHECK_FALSE(flag.load());
  CHECK(flag.load() == t.requested());
  t.request(StopReason::WriteError);
  CHECK(flag.load());
  CHECK(flag.load() == t.requested());
}

TEST_CASE("describe names every reason", "[stoptoken]") {
  CHECK(std::string(demdl::describe(StopReason::None)) == "finished");
  CHECK(std::string(demdl::describe(StopReason::Signal)) == "interrupted");
  CHECK(std::string(demdl::describe(StopReason::WindowClosed)) == "preview window closed");
  CHECK(std::string(demdl::describe(StopReason::ConsecutiveFailures)) == "too many consecutive failures");
  CHECK(std::string(demdl::describe(StopReason::WriteError)) == "checkpoint write failed");
}

TEST_CASE("describe falls back for a value outside the enum", "[stoptoken]") {
  CHECK(std::string(demdl::describe(static_cast<StopReason>(99))) == "unknown");
}

TEST_CASE("the test doubles compile and behave", "[doubles]") {
  FakeFetcher fetcher;
  std::atomic<int> calls{0};
  fetcher.calls = &calls;
  fetcher.behaviour = [](int x, int) { return valueResult(static_cast<float>(x)); };
  std::atomic<bool> stop{false};
  const demdl::FetchResult r = fetcher.fetch(3, 0, stop);
  CHECK(r.kind == demdl::FetchResult::Kind::Value);
  CHECK(r.value == 3.0f);
  CHECK(calls.load() == 1);

  RecordingSink sink;
  CHECK_FALSE(sink.plan.has_value());
  sink.onRunStarted({100, 40, 2, 5});
  REQUIRE(sink.plan.has_value());
  CHECK(sink.plan->pixelsTodo == 40);
  CHECK(sink.plan->rowsTotal == 5);
  sink.onPixelState({1, 2, demdl::PixelState::Failed, 0.0f, 2, "boom"});
  sink.onPixelState({1, 3, demdl::PixelState::Value, 4.5f, 1, {}});
  CHECK(sink.count(demdl::PixelState::Failed) == 1);
  CHECK(sink.count(demdl::PixelState::Value) == 1);
  CHECK(sink.pixels.size() == 2);
  const RecordingSink& view = sink;
  CHECK(view.count(demdl::PixelState::Failed) == 1);
}

TEST_CASE("a gate holds callers until released and reports arrivals", "[doubles]") {
  Gate gate;
  CHECK_FALSE(gate.awaitArrivals(1, std::chrono::milliseconds(10)));  // nobody has come yet

  std::atomic<int> calls{0};
  FakeFetcher first;
  FakeFetcher second;
  for (FakeFetcher* fetcher : {&first, &second}) {
    fetcher->gate = &gate;
    fetcher->calls = &calls;
    fetcher->behaviour = [](int, int) { return noDataResult(); };
  }
  std::atomic<bool> stop{false};
  demdl::FetchResult firstResult;
  demdl::FetchResult secondResult;
  std::thread a([&] { firstResult = first.fetch(1, 0, stop); });
  std::thread b([&] { secondResult = second.fetch(2, 0, stop); });

  const bool bothArrived = gate.awaitArrivals(2, std::chrono::seconds(5));
  const bool atLeastOne = gate.awaitArrivals(1, std::chrono::milliseconds(10));
  const bool thirdArrived = gate.awaitArrivals(3, std::chrono::milliseconds(10));
  const int callsWhileGated = calls.load();
  gate.release();
  a.join();
  b.join();

  CHECK(bothArrived);
  CHECK(atLeastOne);
  CHECK_FALSE(thirdArrived);
  CHECK(callsWhileGated == 0);
  CHECK(calls.load() == 2);
  CHECK(firstResult.kind == demdl::FetchResult::Kind::NoData);
  CHECK(secondResult.kind == demdl::FetchResult::Kind::NoData);
}

TEST_CASE("a fake fetcher counts the calls that see the stop flag set", "[doubles]") {
  StopToken token;
  std::atomic<bool> otherFlag{false};
  std::atomic<int> sawStop{0};
  FakeFetcher fetcher;
  fetcher.sawStop = &sawStop;
  fetcher.behaviour = [](int, int) { return noDataResult(); };

  CHECK(fetcher.fetch(0, 0, token.flag()).kind == demdl::FetchResult::Kind::NoData);
  CHECK(sawStop.load() == 0);
  token.request(StopReason::Signal);
  CHECK(fetcher.fetch(1, 0, token.flag()).kind == demdl::FetchResult::Kind::NoData);
  CHECK(sawStop.load() == 1);
  // A fetcher handed some other flag does not see the stop, which is how a test notices a
  // Downloader that passes the wrong one.
  CHECK(fetcher.fetch(2, 0, otherFlag).kind == demdl::FetchResult::Kind::NoData);
  CHECK(sawStop.load() == 1);
}

TEST_CASE("a fake fetcher reads the stop flag after the gate opens", "[doubles]") {
  StopToken token;
  Gate gate;
  std::atomic<int> sawStop{0};
  FakeFetcher fetcher;
  fetcher.gate = &gate;
  fetcher.sawStop = &sawStop;
  fetcher.behaviour = [](int, int) { return noDataResult(); };

  demdl::FetchResult result;
  std::thread worker([&] { result = fetcher.fetch(0, 0, token.flag()); });
  const bool arrived = gate.awaitArrivals(1, std::chrono::seconds(5));
  token.request(StopReason::WindowClosed);  // raised while the fetcher is held at the gate
  gate.release();
  worker.join();

  CHECK(arrived);
  CHECK(sawStop.load() == 1);
  CHECK(result.kind == demdl::FetchResult::Kind::NoData);
}
```

- [ ] **Step 2: Add the test to CMake**

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp
    tests/test_DemRaster.cpp
    tests/test_StopToken.cpp)
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `'FakeFetcher.h' file not found`.

- [ ] **Step 4: Write `src/demdl/PixelFetcher.h`**

```cpp
#pragma once

#include <atomic>
#include <string>

namespace demdl {

struct FetchResult {
  enum class Kind { Value, NoData, Failed };
  Kind kind = Kind::Failed;
  float value = 0.0f;   // valid when kind == Value
  int attempts = 0;     // attempts made, including the first
  std::string error{};  // last error when kind == Failed
};

// Fetches one pixel. Implementations must not throw: fetch runs inside a pool job, so every
// failure is reported through the result. They may block for retries, but once `stop` is set they
// must neither start another attempt nor keep sleeping (a request already on the wire may run to
// its timeout). `value` is finite when `kind == Value`. One instance serves one worker thread;
// instances need not be thread-safe.
class PixelFetcher {
 public:
  virtual ~PixelFetcher() = default;
  [[nodiscard]] virtual FetchResult fetch(int x, int y, const std::atomic<bool>& stop) = 0;
};

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/ProgressSink.h`**

```cpp
#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace demdl {

enum class PixelState { Queued, InFlight, Value, NoData, Failed };

enum class StopReason { None, Signal, WindowClosed, ConsecutiveFailures, WriteError };

[[nodiscard]] constexpr const char* describe(StopReason reason) noexcept {
  switch (reason) {
    case StopReason::None: return "finished";
    case StopReason::Signal: return "interrupted";
    case StopReason::WindowClosed: return "preview window closed";
    case StopReason::ConsecutiveFailures: return "too many consecutive failures";
    case StopReason::WriteError: return "checkpoint write failed";
  }
  return "unknown";
}

struct RunPlan {
  std::uint64_t pixelsTotal = 0;
  std::uint64_t pixelsTodo = 0;
  int rowsTodo = 0;
  int rowsTotal = 0;
};

struct PixelEvent {
  int x = 0;
  int y = 0;
  PixelState state = PixelState::Queued;
  float value = 0.0f;   // valid when state == Value
  int attempts = 0;
  std::string error{};  // set when state == Failed
};

struct RowReport {
  int row = 0;  // 0-based
  int rowsTotal = 0;
  int ok = 0;
  int noData = 0;
  int failed = 0;
  std::chrono::duration<double> elapsed{0.0};
  std::uint64_t pixelsDoneTotal = 0;  // non-NaN pixels in the whole raster after this save
  std::uint64_t pixelsTotal = 0;
};

struct RunSummary {
  std::uint64_t pixelsFetched = 0;
  std::uint64_t noData = 0;
  std::uint64_t failed = 0;
  std::uint64_t remainingNaN = 0;  // NaN pixels in the last successfully written file
  int rowsSaved = 0;
  std::chrono::duration<double> elapsed{0.0};
  StopReason stopReason = StopReason::None;
  std::string stopMessage{};
  bool checkpointWriteFailed = false;  // a save threw, even if stopReason names an earlier reason
};

// Callbacks must not throw: pixel events are delivered inside pool jobs. Pixel events may arrive
// concurrently from worker threads and from the downloader thread (Queued comes from the
// downloader); row and run events come from the downloader thread. Implementations guard their
// own state.
class ProgressSink {
 public:
  virtual ~ProgressSink() = default;
  virtual void onRunStarted(const RunPlan& plan) = 0;
  virtual void onPixelState(const PixelEvent& event) = 0;
  virtual void onRowSaved(const RowReport& report) = 0;
  virtual void onRunFinished(const RunSummary& summary) = 0;
};

}  // namespace demdl
```

- [ ] **Step 6: Write `src/demdl/StopToken.h`**

```cpp
#pragma once

#include <atomic>
#include <cassert>

#include "demdl/ProgressSink.h"

namespace demdl {

// Shared "please stop" flag plus the first reason given. request() raises the flag and keeps the
// first reason: later reasons are ignored. Both members are lock-free atomics (see the
// static_asserts) and request() touches nothing else, so a signal handler may call it.
class StopToken {
 public:
  // Requires reason != StopReason::None (asserted in debug builds); None would raise the flag
  // without saying why.
  void request(StopReason reason) noexcept {
    assert(reason != StopReason::None);
    StopReason expected = StopReason::None;
    // Reason first, then flag; both seq_cst, so a reader that observes the flag observes a reason.
    reason_.compare_exchange_strong(expected, reason);
    stop_.store(true);
  }
  [[nodiscard]] bool requested() const noexcept { return stop_.load(); }
  [[nodiscard]] StopReason reason() const noexcept { return reason_.load(); }
  // The underlying flag, read-only, for handing to PixelFetcher::fetch.
  [[nodiscard]] const std::atomic<bool>& flag() const noexcept { return stop_; }

 private:
  std::atomic<bool> stop_{false};
  std::atomic<StopReason> reason_{StopReason::None};
  static_assert(std::atomic<bool>::is_always_lock_free);
  static_assert(std::atomic<StopReason>::is_always_lock_free);
};

}  // namespace demdl
```

- [ ] **Step 7: Write `tests/FakeFetcher.h`**

```cpp
#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include "demdl/PixelFetcher.h"

// Blocks every caller of wait() until release() is called, and counts how many have arrived;
// awaitArrivals() lets a test wait for that count without spinning.
struct Gate {
  std::mutex m;
  std::condition_variable cv;
  bool open = false;
  std::atomic<int> waiting{0};

  void wait() {
    std::unique_lock<std::mutex> lock(m);
    ++waiting;  // under the lock, so awaitArrivals() cannot miss the notification below
    cv.notify_all();
    cv.wait(lock, [this] { return open; });
  }
  void release() {
    {
      std::lock_guard<std::mutex> lock(m);
      open = true;
    }
    cv.notify_all();
  }
  // True once at least n callers have reached wait(), false if the timeout passes first.
  [[nodiscard]] bool awaitArrivals(int n, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(m);
    return cv.wait_for(lock, timeout, [this, n] { return waiting.load() >= n; });
  }
};

// Answers fetches from a lambda; optionally blocks on a Gate, counts calls, and counts the calls
// that find `stop` set once the gate has opened.
struct FakeFetcher : demdl::PixelFetcher {
  std::function<demdl::FetchResult(int x, int y)> behaviour;
  std::atomic<int>* calls = nullptr;
  std::atomic<int>* sawStop = nullptr;
  Gate* gate = nullptr;

  [[nodiscard]] demdl::FetchResult fetch(int x, int y, const std::atomic<bool>& stop) override {
    if (gate != nullptr) gate->wait();
    if (sawStop != nullptr && stop.load()) ++*sawStop;
    if (calls != nullptr) ++*calls;
    return behaviour(x, y);
  }
};

[[nodiscard]] inline demdl::FetchResult valueResult(float value) {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::Value;
  r.value = value;
  r.attempts = 1;
  return r;
}

[[nodiscard]] inline demdl::FetchResult noDataResult() {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::NoData;
  r.attempts = 1;
  return r;
}

[[nodiscard]] inline demdl::FetchResult failedResult(std::string error) {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::Failed;
  r.attempts = 3;
  r.error = std::move(error);
  return r;
}
```

- [ ] **Step 8: Write `tests/RecordingSink.h`**

```cpp
#pragma once

#include <algorithm>
#include <mutex>
#include <optional>
#include <vector>

#include "demdl/ProgressSink.h"

// Records every event. Reading the vectors is safe once the run has returned or been joined.
struct RecordingSink : demdl::ProgressSink {
  mutable std::mutex m;
  std::optional<demdl::RunPlan> plan;  // empty until onRunStarted
  std::vector<demdl::PixelEvent> pixels;
  std::vector<demdl::RowReport> rows;
  std::optional<demdl::RunSummary> summary;

  void onRunStarted(const demdl::RunPlan& p) override {
    std::lock_guard<std::mutex> lock(m);
    plan = p;
  }
  void onPixelState(const demdl::PixelEvent& e) override {
    std::lock_guard<std::mutex> lock(m);
    pixels.push_back(e);
  }
  void onRowSaved(const demdl::RowReport& r) override {
    std::lock_guard<std::mutex> lock(m);
    rows.push_back(r);
  }
  void onRunFinished(const demdl::RunSummary& s) override {
    std::lock_guard<std::mutex> lock(m);
    summary = s;
  }

  [[nodiscard]] int count(demdl::PixelState state) const {
    std::lock_guard<std::mutex> lock(m);
    return static_cast<int>(std::count_if(pixels.begin(), pixels.end(),
                                          [state](const demdl::PixelEvent& e) { return e.state == state; }));
  }
};
```

- [ ] **Step 9: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[stoptoken],[doubles]"`
Expected: `All tests passed (N assertions in 8 test cases)`.

- [ ] **Step 10: Commit**

```bash
git add CMakeLists.txt src/demdl/PixelFetcher.h src/demdl/ProgressSink.h src/demdl/StopToken.h tests/FakeFetcher.h tests/RecordingSink.h tests/test_StopToken.cpp
git commit -m "feat: PixelFetcher and ProgressSink interfaces, StopToken, test doubles" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 8: Downloader

Implements spec section 7 (row loop, checkpoints, stop handling) and the Downloader tests in section 11.

**Files:**
- Create: `src/demdl/Downloader.h`
- Create: `src/demdl/Downloader.cpp`
- Create: `tests/test_Downloader.cpp`
- Modify: `CMakeLists.txt` (source lists)

- [ ] **Step 1: Write the failing test `tests/test_Downloader.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include "FakeFetcher.h"
#include "RecordingSink.h"
#include "TempDir.h"
#include "demdl/Downloader.h"

using namespace std::chrono_literals;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::Message;
using Catch::Matchers::StartsWith;
using demdl::BoundingBox;
using demdl::DemRaster;
using demdl::Downloader;
using demdl::DownloaderOptions;
using demdl::FetchResult;
using demdl::FetcherFactory;
using demdl::PixelState;
using demdl::StopReason;
using demdl::StopToken;

namespace {

const BoundingBox kBox{-34.0, -34.3, 18.5, 18.1};

float expectedValue(int x, int y) { return static_cast<float>(x * 10 + y); }

// A type of its own, so a test can tell that an exception arrived unchanged.
struct FactoryError : std::runtime_error {
  using std::runtime_error::runtime_error;
};

struct Fixture {
  TempDir dir;
  std::filesystem::path output = dir.path / "out.tif";
  RecordingSink sink;
  StopToken stop;
  std::atomic<int> calls{0};
  std::atomic<int> sawStop{0};
  Gate gate;

  FetcherFactory factory(std::function<FetchResult(int, int)> behaviour, bool blocking = false) {
    return [this, behaviour, blocking](int) -> std::unique_ptr<demdl::PixelFetcher> {
      auto f = std::make_unique<FakeFetcher>();
      f->behaviour = behaviour;
      f->calls = &calls;
      f->sawStop = &sawStop;
      f->gate = blocking ? &gate : nullptr;
      return f;
    };
  }
};

}  // namespace

TEST_CASE("only NaN pixels are fetched and finished rows are skipped", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(4, 3, kBox, {});
  for (int x = 0; x < 4; ++x) raster.set(x, 1, 99.0f);  // row 1 complete
  raster.set(0, 2, 7.0f);                               // row 2 half done
  raster.set(1, 2, DemRaster::kNoData);
  Downloader d(raster, f.output, f.factory([](int x, int y) { return valueResult(expectedValue(x, y)); }),
               f.sink, f.stop, DownloaderOptions{2, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.pixelsFetched == 6);
  CHECK(summary.noData == 0);
  CHECK(summary.failed == 0);
  CHECK(summary.rowsSaved == 2);
  CHECK(summary.remainingNaN == 0);
  CHECK(summary.stopReason == StopReason::None);
  CHECK(summary.stopMessage == "finished");
  CHECK(f.calls.load() == 6);
  REQUIRE(f.sink.plan.has_value());
  CHECK(f.sink.plan->pixelsTotal == 12);
  CHECK(f.sink.plan->pixelsTodo == 6);
  CHECK(f.sink.plan->rowsTodo == 2);
  CHECK(f.sink.plan->rowsTotal == 3);
  REQUIRE(f.sink.rows.size() == 2);
  CHECK(f.sink.rows[0].row == 0);
  CHECK(f.sink.rows[0].ok == 4);
  CHECK(f.sink.rows[0].pixelsDoneTotal == 10);
  CHECK(f.sink.rows[1].row == 2);
  CHECK(f.sink.rows[1].pixelsDoneTotal == 12);
  CHECK(f.sink.count(PixelState::Queued) == 6);
  CHECK(f.sink.count(PixelState::InFlight) == 6);
  CHECK(f.sink.count(PixelState::Value) == 6);
  CHECK(raster.at(3, 0) == expectedValue(3, 0));
  CHECK(raster.at(0, 2) == 7.0f);
  CHECK(raster.at(2, 2) == expectedValue(2, 2));

  const auto onDisk = DemRaster::load(f.output);
  CHECK(onDisk.countNaN() == 0);
  CHECK(onDisk.at(3, 2) == expectedValue(3, 2));
}

TEST_CASE("NoData and failures are recorded and counted", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(4, 1, kBox, {});
  Downloader d(raster, f.output, f.factory([](int x, int y) {
                 if (x == 0) return noDataResult();
                 if (x == 1) return failedResult("http 500");
                 return valueResult(expectedValue(x, y));
               }),
               f.sink, f.stop, DownloaderOptions{1, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.pixelsFetched == 2);
  CHECK(summary.noData == 1);
  CHECK(summary.failed == 1);
  CHECK(summary.remainingNaN == 1);
  CHECK(summary.stopReason == StopReason::None);
  CHECK(raster.at(0, 0) == DemRaster::kNoData);
  CHECK(std::isnan(raster.at(1, 0)));
  CHECK(f.sink.count(PixelState::Failed) == 1);
  CHECK(f.sink.count(PixelState::NoData) == 1);
  CHECK(f.sink.count(PixelState::Value) == 2);
  REQUIRE(f.sink.rows.size() == 1);
  CHECK(f.sink.rows[0].ok == 2);
  CHECK(f.sink.rows[0].noData == 1);
  CHECK(f.sink.rows[0].failed == 1);
  CHECK(f.sink.rows[0].pixelsDoneTotal == 3);
}

TEST_CASE("eight workers fill a mixed grid that matches the file and the summary", "[downloader]") {
  constexpr int kWidth = 64;
  constexpr int kHeight = 16;
  // Each pixel's outcome follows from its position.
  const auto outcome = [](int x, int y) {
    if ((x + 2 * y) % 7 == 0) return FetchResult::Kind::NoData;
    if ((3 * x + y) % 11 == 0) return FetchResult::Kind::Failed;
    return FetchResult::Kind::Value;
  };
  Fixture f;
  auto raster = DemRaster::create(kWidth, kHeight, kBox, {});
  Downloader d(raster, f.output, f.factory([outcome](int x, int y) {
                 switch (outcome(x, y)) {
                   case FetchResult::Kind::NoData: return noDataResult();
                   case FetchResult::Kind::Failed: return failedResult("http 503");
                   case FetchResult::Kind::Value: break;
                 }
                 return valueResult(expectedValue(x, y));
               }),
               f.sink, f.stop, DownloaderOptions{8, 100});

  const demdl::RunSummary summary = d.run();

  const auto onDisk = DemRaster::load(f.output);
  std::uint64_t values = 0;
  std::uint64_t noData = 0;
  std::uint64_t failures = 0;
  int mismatches = 0;
  for (int y = 0; y < kHeight; ++y) {
    for (int x = 0; x < kWidth; ++x) {
      const float cell = onDisk.at(x, y);
      switch (outcome(x, y)) {
        case FetchResult::Kind::Value:
          ++values;
          if (cell != expectedValue(x, y)) ++mismatches;
          break;
        case FetchResult::Kind::NoData:
          ++noData;
          if (cell != DemRaster::kNoData) ++mismatches;
          break;
        case FetchResult::Kind::Failed:
          ++failures;
          if (!std::isnan(cell)) ++mismatches;
          break;
      }
    }
  }
  CHECK(mismatches == 0);
  CHECK(f.calls.load() == kWidth * kHeight);
  CHECK(summary.stopReason == StopReason::None);
  CHECK(summary.rowsSaved == kHeight);
  CHECK(summary.pixelsFetched == values);
  CHECK(summary.noData == noData);
  CHECK(summary.failed == failures);
  CHECK(summary.remainingNaN == failures);
}

TEST_CASE("a stop request saves the partial row and goes no further", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(4, 2, kBox, {});
  Downloader d(raster, f.output,
               f.factory([](int x, int y) { return valueResult(expectedValue(x, y)); }, /*blocking=*/true),
               f.sink, f.stop, DownloaderOptions{2, 100});

  std::thread runner([&d] { (void)d.run(); });
  // Opens the gate and joins the runner however the test exits, so a failed REQUIRE cannot destroy
  // a joinable std::thread, which would call std::terminate.
  struct ReleaseAndJoin {
    Gate& gate;
    std::thread& runner;
    ~ReleaseAndJoin() {
      gate.release();
      if (runner.joinable()) runner.join();
    }
  } guard{f.gate, runner};
  REQUIRE(f.gate.awaitArrivals(2, 5s));
  CHECK(d.inFlight() == 2);

  f.stop.request(StopReason::Signal);
  f.gate.release();
  runner.join();

  CHECK(f.sawStop.load() == 2);
  REQUIRE(f.sink.summary.has_value());
  CHECK(f.sink.summary->stopReason == StopReason::Signal);
  CHECK(f.sink.summary->stopMessage == "interrupted");
  CHECK(f.sink.summary->pixelsFetched == 2);
  CHECK(f.sink.summary->rowsSaved == 1);
  CHECK(f.sink.summary->remainingNaN == 6);
  CHECK(f.calls.load() == 2);
  CHECK(d.inFlight() == 0);
  REQUIRE(f.sink.rows.size() == 1);
  CHECK(f.sink.rows[0].row == 0);
  CHECK(f.sink.rows[0].ok == 2);

  const auto onDisk = DemRaster::load(f.output);
  CHECK(onDisk.countNaN() == 6);
  CHECK(onDisk.todoInRow(0).size() == 2);
  CHECK(onDisk.todoInRow(1).size() == 4);
  // The pool takes jobs in submission order, so the two fetches were columns 0 and 1.
  CHECK(onDisk.at(0, 0) == expectedValue(0, 0));
  CHECK(onDisk.at(1, 0) == expectedValue(1, 0));
  CHECK(std::isnan(onDisk.at(2, 0)));
  CHECK(std::isnan(onDisk.at(3, 0)));
}

TEST_CASE("too many consecutive failures stop the run", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(10, 1, kBox, {});
  // One worker takes the columns in order, and only the third failure, which reaches the limit,
  // says "connection refused": the stop message must name that one.
  Downloader d(raster, f.output, f.factory([](int x, int) {
                 return failedResult(x == 2 ? "connection refused" : "http 503");
               }),
               f.sink, f.stop, DownloaderOptions{1, 3});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::ConsecutiveFailures);
  CHECK(summary.stopMessage == "server unreachable: 3 consecutive failures; last error: connection refused");
  CHECK(summary.failed == 3);
  CHECK(f.calls.load() == 3);
  CHECK(summary.remainingNaN == 10);
  CHECK(summary.rowsSaved == 1);
  CHECK(std::filesystem::exists(f.output));
}

TEST_CASE("an intervening success resets the failure streak", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(6, 1, kBox, {});
  // One worker takes the columns in order: failure, failure, value, failure, failure, failure.
  Downloader d(raster, f.output, f.factory([](int x, int y) {
                 if (x == 2) return valueResult(expectedValue(x, y));
                 return failedResult("http 503");
               }),
               f.sink, f.stop, DownloaderOptions{1, 3});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::ConsecutiveFailures);
  CHECK(f.calls.load() == 6);  // without the reset the limit would trip on the fourth call
  CHECK(summary.failed == 5);
  CHECK(summary.pixelsFetched == 1);
}

TEST_CASE("the failure streak carries across rows", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(2, 3, kBox, {});
  Downloader d(raster, f.output, f.factory([](int, int) { return failedResult("connection refused"); }),
               f.sink, f.stop, DownloaderOptions{1, 3});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::ConsecutiveFailures);
  CHECK(f.calls.load() == 3);  // both pixels of row 0, then the first of row 1
  CHECK(summary.failed == 3);
  CHECK(summary.rowsSaved == 2);
}

TEST_CASE("a complete raster makes no requests and writes nothing", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(3, 2, kBox, {});
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 3; ++x) raster.set(x, y, 1.0f);
  }
  Downloader d(raster, f.output, f.factory([](int, int) { return valueResult(0.0f); }), f.sink, f.stop,
               DownloaderOptions{2, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(f.calls.load() == 0);
  CHECK(summary.rowsSaved == 0);
  CHECK(summary.remainingNaN == 0);
  REQUIRE(f.sink.plan.has_value());
  CHECK(f.sink.plan->pixelsTodo == 0);
  CHECK(f.sink.plan->rowsTodo == 0);
  REQUIRE(f.sink.summary.has_value());
  CHECK_FALSE(std::filesystem::exists(f.output));
}

TEST_CASE("an exception inside a job is recorded as a failure, not propagated", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(3, 1, kBox, {});
  Downloader d(raster, f.output, f.factory([](int x, int y) {
                 if (x == 1) throw std::runtime_error("boom");
                 return valueResult(expectedValue(x, y));
               }),
               f.sink, f.stop, DownloaderOptions{1, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.failed == 1);
  CHECK(summary.pixelsFetched == 2);
  CHECK(summary.remainingNaN == 1);
  CHECK(d.inFlight() == 0);
  const auto failure = std::find_if(f.sink.pixels.begin(), f.sink.pixels.end(),
                                    [](const demdl::PixelEvent& e) { return e.state == PixelState::Failed; });
  REQUIRE(failure != f.sink.pixels.end());
  CHECK(failure->x == 1);
  CHECK_THAT(failure->error, ContainsSubstring("boom"));
}

TEST_CASE("a write failure after a stop request is still reported", "[downloader]") {
  Fixture f;
  REQUIRE(std::filesystem::create_directory(f.output));  // the checkpoint cannot replace a directory
  auto raster = DemRaster::create(2, 1, kBox, {});
  // A stop requested before run() would end the run before any save, so the first fetch requests it.
  // The second pixel is then skipped, and the row's save fails after the stop.
  Downloader d(raster, f.output, f.factory([&stop = f.stop](int x, int y) {
                 stop.request(StopReason::Signal);
                 return valueResult(expectedValue(x, y));
               }),
               f.sink, f.stop, DownloaderOptions{1, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::Signal);
  CHECK(summary.checkpointWriteFailed);
  CHECK_THAT(summary.stopMessage, StartsWith("interrupted; checkpoint write failed: "));
  CHECK(summary.rowsSaved == 0);
}

TEST_CASE("a failed checkpoint is reported and nothing after it runs", "[downloader]") {
  Fixture f;
  REQUIRE(std::filesystem::create_directory(f.output));  // the checkpoint cannot replace a directory
  auto raster = DemRaster::create(3, 2, kBox, {});
  Downloader d(raster, f.output, f.factory([](int x, int y) { return valueResult(expectedValue(x, y)); }),
               f.sink, f.stop, DownloaderOptions{1, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::WriteError);
  CHECK(summary.checkpointWriteFailed);
  CHECK_THAT(summary.stopMessage, StartsWith("checkpoint write failed: "));
  CHECK_THAT(summary.stopMessage, ContainsSubstring("cannot rename"));
  CHECK(f.calls.load() == 3);  // row 1 is never fetched
  CHECK(f.sink.rows.empty());
  CHECK(summary.rowsSaved == 0);
  CHECK(summary.remainingNaN == 6);  // row 0's values never reached the file
}

TEST_CASE("an exception from the fetcher factory escapes run() unchanged", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(2, 1, kBox, {});
  Downloader d(raster, f.output,
               [](int) -> std::unique_ptr<demdl::PixelFetcher> { throw FactoryError("cannot create a client"); },
               f.sink, f.stop, DownloaderOptions{2, 100});

  CHECK_THROWS_AS(d.run(), FactoryError);
  CHECK_FALSE(f.sink.plan.has_value());     // the fetchers are made before onRunStarted
  CHECK_FALSE(f.sink.summary.has_value());  // and onRunFinished never comes
}

TEST_CASE("a null fetcher from the factory is refused before the run starts", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(2, 1, kBox, {});
  Downloader d(raster, f.output, [](int) { return std::unique_ptr<demdl::PixelFetcher>(); }, f.sink, f.stop,
               DownloaderOptions{2, 100});

  CHECK_THROWS_MATCHES(d.run(), std::invalid_argument, Message("fetcher factory returned null"));
  CHECK_FALSE(f.sink.plan.has_value());
}
```

- [ ] **Step 2: Add the files to CMake**

```cmake
add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp
  src/demdl/WmsQuery.cpp
  src/demdl/RetryPolicy.cpp
  src/demdl/DemRaster.cpp
  src/demdl/Downloader.cpp)
```

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp
    tests/test_DemRaster.cpp
    tests/test_StopToken.cpp
    tests/test_Downloader.cpp)
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/demdl/Downloader.cpp`.

- [ ] **Step 4: Write `src/demdl/Downloader.h`**

```cpp
#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include "demdl/DemRaster.h"
#include "demdl/PixelFetcher.h"
#include "demdl/ProgressSink.h"
#include "demdl/StopToken.h"

namespace demdl {

// Creates the fetcher for one worker thread. run() calls it once per worker, before onRunStarted.
using FetcherFactory = std::function<std::unique_ptr<PixelFetcher>(int workerId)>;

struct DownloaderOptions {
  int threads = 8;
  int maxConsecutiveFailures = 100;
};

// Walks the rows that still contain NaN pixels, fetches those pixels through a thread pool,
// saves the raster after each row and reports progress.
//
// Call run() exactly once. It blocks until the run ends, so when a preview window is active it must
// not run on the main thread, which belongs to the GLFW event loop; main calls it under std::async.
// Pixel callbacks reach the sink from the pool threads and from the thread calling run() (Queued);
// row and run callbacks come only from the thread calling run().
//
// Errors: run() makes every fetcher before its first callback, so an exception from the factory, or
// std::invalid_argument for a null fetcher, leaves run() before the sink hears anything. Inside a pool
// job an exception from the InFlight callback or the fetch is recorded as a Failed pixel. Jobs are
// noexcept, so any other exception in one (a sink callback reporting the result, or building the
// error text under memory pressure) calls std::terminate instead of unwinding past the pool.
class Downloader {
 public:
  // Throws std::invalid_argument when threads or maxConsecutiveFailures is below 1.
  Downloader(DemRaster& raster, std::filesystem::path output, FetcherFactory factory, ProgressSink& sink,
             StopToken& stop, DownloaderOptions options);

  [[nodiscard]] RunSummary run();

  // Pixels currently inside a fetch call. For the "stopping" message.
  [[nodiscard]] int inFlight() const { return inFlight_.load(); }

 private:
  // Counters the pool jobs update concurrently: RowTally is reset before each row, RunTally spans
  // the run.
  struct RowTally {
    std::atomic<int> ok{0};
    std::atomic<int> noData{0};
    std::atomic<int> failed{0};
    void reset() noexcept {
      ok = 0;
      noData = 0;
      failed = 0;
    }
  };
  struct RunTally {
    std::atomic<std::uint64_t> fetched{0};
    std::atomic<std::uint64_t> noData{0};
    std::atomic<std::uint64_t> failed{0};
    std::atomic<int> consecutiveFailures{0};  // reset by every Value or NoData
    std::mutex limitMutex;
    std::optional<std::string> limitError;  // guarded by limitMutex; the failure that reached the limit
  };

  // One pool job: unless a stop came first, fetches pixel (x, row), stores the result, updates the
  // tallies and reports the pixel's final state.
  void fetchPixel(PixelFetcher& fetcher, int x, int row, RowTally& rowTally, RunTally& runTally) noexcept;

  DemRaster& raster_;
  std::filesystem::path output_;
  FetcherFactory factory_;
  ProgressSink& sink_;
  StopToken& stop_;
  DownloaderOptions options_;
  std::atomic<int> inFlight_{0};
};

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/Downloader.cpp`**

```cpp
#include "demdl/Downloader.h"

#include <ctpl_stl.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace demdl {

namespace {

// Counts one fetch in Downloader::inFlight() for as long as it lives.
class InFlightGuard {
 public:
  explicit InFlightGuard(std::atomic<int>& count) noexcept : count_(count) { ++count_; }
  ~InFlightGuard() { --count_; }
  InFlightGuard(const InFlightGuard&) = delete;
  InFlightGuard& operator=(const InFlightGuard&) = delete;

 private:
  std::atomic<int>& count_;
};

}  // namespace

Downloader::Downloader(DemRaster& raster, std::filesystem::path output, FetcherFactory factory,
                       ProgressSink& sink, StopToken& stop, DownloaderOptions options)
    : raster_(raster),
      output_(std::move(output)),
      factory_(std::move(factory)),
      sink_(sink),
      stop_(stop),
      options_(options) {
  if (options_.threads < 1) throw std::invalid_argument("threads must be at least 1");
  if (options_.maxConsecutiveFailures < 1) throw std::invalid_argument("maxConsecutiveFailures must be at least 1");
}

RunSummary Downloader::run() {
  using Clock = std::chrono::steady_clock;
  const auto runStart = Clock::now();

  // Made before the first callback, so a factory that fails leaves the sink untouched.
  std::vector<std::unique_ptr<PixelFetcher>> fetchers;
  fetchers.reserve(static_cast<std::size_t>(options_.threads));
  for (int i = 0; i < options_.threads; ++i) {
    fetchers.push_back(factory_(i));
    if (!fetchers.back()) throw std::invalid_argument("fetcher factory returned null");
  }

  const std::vector<int> rows = raster_.rowsWithWork();
  RunPlan plan;
  plan.pixelsTotal = raster_.pixelCount();
  plan.pixelsTodo = raster_.countNaN();
  plan.rowsTodo = static_cast<int>(rows.size());
  plan.rowsTotal = raster_.height();
  sink_.onRunStarted(plan);

  RunTally runTally;
  RowTally rowTally;
  std::uint64_t doneTotal = plan.pixelsTotal - plan.pixelsTodo;  // non-NaN pixels in the last checkpoint
  int rowsSaved = 0;
  bool checkpointWriteFailed = false;
  std::string writeError;
  // Declared after everything its jobs use, so its destructor, which waits for them, runs first.
  ctpl::thread_pool pool(options_.threads);

  for (const int row : rows) {
    if (stop_.requested()) break;
    const auto rowStart = Clock::now();
    const std::vector<int> todo = raster_.todoInRow(row);
    for (const int x : todo) sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::Queued});

    rowTally.reset();
    std::vector<std::future<void>> futures;
    futures.reserve(todo.size());
    try {
      for (const int x : todo) {
        if (stop_.requested()) break;  // stop queueing immediately
        futures.push_back(pool.push([this, &fetchers, &rowTally, &runTally, x, row](int workerId) noexcept {
          fetchPixel(*fetchers[static_cast<std::size_t>(workerId)], x, row, rowTally, runTally);
        }));
      }
    } catch (...) {
      pool.stop(false);  // discard the queued jobs and wait for the running ones before unwinding
      throw;
    }
    for (auto& f : futures) f.get();

    try {
      raster_.save(output_);
    } catch (const std::exception& e) {
      checkpointWriteFailed = true;
      writeError = e.what();
      stop_.request(StopReason::WriteError);
      break;
    }
    ++rowsSaved;
    doneTotal += static_cast<std::uint64_t>(rowTally.ok.load() + rowTally.noData.load());
    RowReport report;
    report.row = row;
    report.rowsTotal = raster_.height();
    report.ok = rowTally.ok.load();
    report.noData = rowTally.noData.load();
    report.failed = rowTally.failed.load();
    report.elapsed = Clock::now() - rowStart;
    report.pixelsDoneTotal = doneTotal;
    report.pixelsTotal = plan.pixelsTotal;
    sink_.onRowSaved(report);
  }

  RunSummary summary;
  summary.pixelsFetched = runTally.fetched.load();
  summary.noData = runTally.noData.load();
  summary.failed = runTally.failed.load();
  summary.remainingNaN = plan.pixelsTotal - doneTotal;  // the last checkpoint's count, not memory's
  summary.rowsSaved = rowsSaved;
  summary.elapsed = Clock::now() - runStart;
  summary.stopReason = stop_.reason();
  summary.checkpointWriteFailed = checkpointWriteFailed;
  if (summary.stopReason == StopReason::ConsecutiveFailures) {
    summary.stopMessage =
        "server unreachable: " + std::to_string(options_.maxConsecutiveFailures) + " consecutive failures";
    const std::lock_guard<std::mutex> lock(runTally.limitMutex);
    if (runTally.limitError) summary.stopMessage += "; last error: " + *runTally.limitError;
  } else {
    summary.stopMessage = describe(summary.stopReason);
  }
  if (checkpointWriteFailed) {
    const std::string failure = std::string(describe(StopReason::WriteError)) + ": " + writeError;
    if (summary.stopReason == StopReason::WriteError) {
      summary.stopMessage = failure;
    } else {
      summary.stopMessage += "; " + failure;  // an earlier reason was kept, but the failure still shows
    }
  }
  sink_.onRunFinished(summary);
  return summary;
}

void Downloader::fetchPixel(PixelFetcher& fetcher, int x, int row, RowTally& rowTally, RunTally& runTally) noexcept {
  if (stop_.requested()) return;  // queued before the stop, never started
  FetchResult result;
  {
    const InFlightGuard guard(inFlight_);
    // Fetchers and sinks must not throw. Should the InFlight callback or the fetch do so anyway, the
    // pixel is recorded as Failed; an exception anywhere else ends the process, as this is noexcept.
    try {
      sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::InFlight});
      result = fetcher.fetch(x, row, stop_.flag());
    } catch (const std::exception& e) {
      result = FetchResult{.kind = FetchResult::Kind::Failed, .error = std::string("exception: ") + e.what()};
    } catch (...) {
      result = FetchResult{.kind = FetchResult::Kind::Failed, .error = "unknown exception"};
    }
  }
  switch (result.kind) {
    case FetchResult::Kind::Value:
      raster_.set(x, row, result.value);
      runTally.consecutiveFailures.store(0);
      ++rowTally.ok;
      ++runTally.fetched;
      sink_.onPixelState(PixelEvent{
          .x = x, .y = row, .state = PixelState::Value, .value = result.value, .attempts = result.attempts});
      break;
    case FetchResult::Kind::NoData:
      raster_.set(x, row, DemRaster::kNoData);
      runTally.consecutiveFailures.store(0);
      ++rowTally.noData;
      ++runTally.noData;
      sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::NoData, .attempts = result.attempts});
      break;
    case FetchResult::Kind::Failed: {
      ++rowTally.failed;
      ++runTally.failed;
      const int streak = ++runTally.consecutiveFailures;
      sink_.onPixelState(PixelEvent{
          .x = x, .y = row, .state = PixelState::Failed, .attempts = result.attempts, .error = result.error});
      if (streak >= options_.maxConsecutiveFailures) {
        {
          const std::lock_guard<std::mutex> lock(runTally.limitMutex);
          if (!runTally.limitError) runTally.limitError = std::move(result.error);  // the first to reach it
        }
        stop_.request(StopReason::ConsecutiveFailures);
      }
      break;
    }
  }
}

}  // namespace demdl
```

Notes for the engineer: every job writes a distinct raster cell and `save` runs only after `f.get()` has returned for the whole row, so the raster needs no lock. The stop test is deterministic because both workers are parked in the gate when the stop is requested: the row's other columns are either skipped before submission or return at the first line of `fetchPixel`, so exactly two fetches happen whatever the timing. Its column assertions (columns 0 and 1 hold values, 2 and 3 are NaN) rely on `ctpl::thread_pool` taking queued jobs in FIFO order, so the two parked jobs are the first two submitted. The pool is declared after everything its jobs use, so its destructor, which waits for them, runs first, and `f.get()` has nothing to rethrow: `fetchPixel` is `noexcept` and turns an exception from the InFlight callback or the fetch into a Failed result.

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[downloader]"`
Expected: `All tests passed (N assertions in 13 test cases)`. The stop test takes well under a second.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/demdl/Downloader.h src/demdl/Downloader.cpp tests/test_Downloader.cpp
git commit -m "feat: Downloader row loop with per-row checkpoints, stop handling and failure limit" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 9: ElevationClient over httplib

Implements spec section 4.3 (ElevationClient) and the in-process server test in section 11. Deviation 6 applies.

**Files:**
- Create: `src/demdl/ElevationClient.h`
- Create: `src/demdl/ElevationClient.cpp`
- Create: `tests/test_ElevationClient.cpp`
- Modify: `CMakeLists.txt` (source lists)

- [ ] **Step 1: Write the failing test `tests/test_ElevationClient.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>

#include <httplib.h>
#include <openssl/evp.h>
#include <openssl/pem.h>
#include <openssl/ssl.h>
#include <openssl/x509.h>
#include <openssl/x509v3.h>

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ExampleConfig.h"
#include "TempDir.h"
#include "demdl/ElevationClient.h"

using namespace std::chrono_literals;
using demdl::Config;
using demdl::ElevationClient;
using demdl::FetchResult;
using demdl::RetryPolicy;
using demdl::describeUnparseableBody;

namespace {

RetryPolicy fastPolicy() {
  RetryPolicy p;
  p.maxRetries = 2;
  p.baseDelay = 1ms;
  p.maxDelay = 2ms;
  p.jitter = 0.0;
  return p;
}

// The example configuration's box and layer on a 4 x 1 grid, requested from wmsUrl.
Config configFor(const std::string& wmsUrl) {
  Config c = exampleConfig();
  c.width = 4;
  c.height = 1;
  c.output = "unused.tif";
  c.wmsUrl = wmsUrl;
  return c;
}

// A self-signed EC P-256 certificate for 127.0.0.1, made at test time so that no key is committed.
// It is valid from an hour ago to a day ahead and carries subjectAltName IP:127.0.0.1 and
// basicConstraints CA:TRUE, so that as a CA bundle it is its own trust anchor.
struct LoopbackCertificate {
  std::unique_ptr<EVP_PKEY, decltype(&EVP_PKEY_free)> key{EVP_PKEY_Q_keygen(nullptr, nullptr, "EC", "P-256"),
                                                          &EVP_PKEY_free};
  std::unique_ptr<X509, decltype(&X509_free)> cert{X509_new(), &X509_free};

  LoopbackCertificate() {
    REQUIRE(key != nullptr);
    REQUIRE(cert != nullptr);
    X509* const x = cert.get();
    X509_NAME* const name = X509_get_subject_name(x);
    const auto* const commonName = reinterpret_cast<const unsigned char*>("127.0.0.1");
    REQUIRE(X509_set_version(x, X509_VERSION_3) == 1);
    REQUIRE(ASN1_INTEGER_set(X509_get_serialNumber(x), 1) == 1);
    REQUIRE(X509_gmtime_adj(X509_getm_notBefore(x), -60 * 60) != nullptr);
    REQUIRE(X509_gmtime_adj(X509_getm_notAfter(x), 24 * 60 * 60) != nullptr);
    REQUIRE(X509_set_pubkey(x, key.get()) == 1);
    REQUIRE(X509_NAME_add_entry_by_txt(name, "CN", MBSTRING_ASC, commonName, -1, -1, 0) == 1);
    REQUIRE(X509_set_issuer_name(x, name) == 1);
    X509V3_CTX ctx{};
    X509V3_set_ctx(&ctx, x, x, nullptr, nullptr, 0);
    for (const auto& [nid, value] : {std::pair{NID_subject_alt_name, "IP:127.0.0.1"},
                                     std::pair{NID_basic_constraints, "critical,CA:TRUE"}}) {
      X509_EXTENSION* const extension = X509V3_EXT_conf_nid(nullptr, &ctx, nid, value);
      REQUIRE(extension != nullptr);
      const int added = X509_add_ext(x, extension, -1);
      X509_EXTENSION_free(extension);
      REQUIRE(added == 1);
    }
    REQUIRE(X509_sign(x, key.get(), EVP_sha256()) > 0);
  }

  // Writes the certificate as PEM, for use as a CA bundle.
  void writePem(const std::filesystem::path& path) const {
    std::FILE* const file = std::fopen(path.c_str(), "w");
    REQUIRE(file != nullptr);
    const int written = PEM_write_X509(file, cert.get());
    REQUIRE(std::fclose(file) == 0);
    REQUIRE(written == 1);
  }
};

// An httplib::SSLServer presenting the certificate. httplib 0.46 has no constructor that takes an
// X509* and an EVP_PKEY*, so the context callback installs them; SSL_CTX_use_certificate and
// SSL_CTX_use_PrivateKey take references of their own.
std::unique_ptr<httplib::Server> tlsServer(const LoopbackCertificate& certificate) {
  return std::make_unique<httplib::SSLServer>([&certificate](httplib::tls::ctx_t ctx) {
    auto* const ssl = static_cast<SSL_CTX*>(ctx);
    return SSL_CTX_use_certificate(ssl, certificate.cert.get()) == 1 &&
           SSL_CTX_use_PrivateKey(ssl, certificate.key.get()) == 1;
  });
}

// The pixel column I of a request, or -1 unless it is a single digit from 0 to 7.
int columnOf(const httplib::Request& req) {
  const std::string i = req.get_param_value("I");
  return i.size() == 1 && i[0] >= '0' && i[0] <= '7' ? i[0] - '0' : -1;
}

constexpr const char* kServiceExceptionReport =
    R"(<?xml version="1.0"?><ServiceExceptionReport>)"
    R"(<ServiceException code="LayerNotDefined">no such layer</ServiceException>)"
    R"(</ServiceExceptionReport>)";

// What the server decoded from one request.
struct SeenRequest {
  std::map<std::string, std::string> params;  // BBOX, LAYERS, QUERY_LAYERS, I and J
  std::vector<std::string> userAgents;         // every User-Agent header value
};

// Serves canned GetFeatureInfo bodies on loopback, over plain HTTP or, given a certificate, over TLS.
// The pixel column I selects the behaviour: 0 a band value, 1 a plain body without a band, 2 a 500
// and then a band value, 4 a 200 service exception report, any other column a 404. Hits are counted
// per column; the remote port of every request and what the latest one carried are recorded.
// httplib re-encodes the query on the wire (%2C goes out as ','), so tests compare the decoded
// parameters and never the raw request target.
struct CannedServer {
  std::unique_ptr<httplib::Server> server;
  std::string scheme;
  std::thread thread;
  std::array<std::atomic<int>, 8> hits{};  // indexed by column
  std::mutex mutex;
  std::set<int> remotePorts;  // guarded by mutex; one port per connection
  SeenRequest latest;         // guarded by mutex
  int port = 0;

  CannedServer() : CannedServer(std::make_unique<httplib::Server>(), "http") {}
  explicit CannedServer(const LoopbackCertificate& certificate)
      : CannedServer(tlsServer(certificate), "https") {}
  CannedServer(std::unique_ptr<httplib::Server> s, std::string urlScheme)
      : server(std::move(s)), scheme(std::move(urlScheme)) {
    REQUIRE(server->is_valid());
    server->Get("/wms", [this](const httplib::Request& req, httplib::Response& res) { respond(req, res); });
    port = server->bind_to_any_port("127.0.0.1");
    REQUIRE(port > 0);
    thread = std::thread([this] { server->listen_after_bind(); });
    server->wait_until_ready();
  }
  ~CannedServer() {
    server->stop();
    thread.join();
  }

  void respond(const httplib::Request& req, httplib::Response& res) {
    {
      const std::lock_guard<std::mutex> lock(mutex);
      remotePorts.insert(req.remote_port);
      latest = SeenRequest{};
      for (const char* key : {"BBOX", "LAYERS", "QUERY_LAYERS", "I", "J"}) {
        latest.params[key] = req.get_param_value(key);
      }
      for (std::size_t i = 0; i < req.get_header_value_count("User-Agent"); ++i) {
        latest.userAgents.push_back(req.get_header_value("User-Agent", "", i));
      }
    }
    const int column = columnOf(req);
    const int hit = column < 0 ? 0 : ++hits[static_cast<std::size_t>(column)];  // 1 on the first
    if (column == 0) {
      res.set_content("GetFeatureInfo results:\nBAND 0: 12.5\n", "text/plain");
    } else if (column == 1) {
      res.set_content("GetFeatureInfo results:\n", "text/plain");
    } else if (column == 2) {
      if (hit == 1) {
        res.status = 500;
      } else {
        res.set_content("BAND 0: 7\n", "text/plain");
      }
    } else if (column == 4) {
      res.set_content(kServiceExceptionReport, "application/vnd.ogc.se_xml");
    } else {
      res.status = 404;
    }
  }

  std::string url() const { return scheme + "://127.0.0.1:" + std::to_string(port) + "/wms"; }
  std::size_t distinctRemotePorts() {
    const std::lock_guard<std::mutex> lock(mutex);
    return remotePorts.size();
  }
  SeenRequest latestRequest() {
    const std::lock_guard<std::mutex> lock(mutex);
    return latest;
  }
};

}  // namespace

TEST_CASE("a band value is returned on the first attempt", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 12.5f);
  CHECK(r.attempts == 1);
}

TEST_CASE("a plain 200 without a band is NoData", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(1, 0, stop);
  CHECK(r.kind == FetchResult::Kind::NoData);
  CHECK(r.attempts == 1);
}

TEST_CASE("a 500 is retried and the second attempt succeeds", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(2, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 7.0f);
  CHECK(r.attempts == 2);
  CHECK(srv.hits[2].load() == 2);
}

TEST_CASE("a status that never recovers fails after the retries", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(3, 0, stop);  // 404 every time
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "http 404");
  CHECK(srv.hits[3].load() == 3);
}

TEST_CASE("a 200 service exception is retried and then fails", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(4, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "unparseable body");
  CHECK(srv.hits[4].load() == 3);
}

TEST_CASE("a refused connection fails after the retries with a transport error", "[elevationclient]") {
  // On macOS unprivileged processes can bind low ports; the assumption is only that nothing listens
  // on port 1 on a developer machine, so the connection is refused at once.
  ElevationClient client(configFor("http://127.0.0.1:1/wms"), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error.rfind("transport: ", 0) == 0);
}

TEST_CASE("a stop request cuts the backoff short", "[elevationclient]") {
  CannedServer srv;
  RetryPolicy slow;
  slow.maxRetries = 5;
  slow.baseDelay = 10s;
  slow.maxDelay = 10s;
  slow.jitter = 0.0;
  ElevationClient client(configFor(srv.url()), slow);
  std::atomic<bool> stop{false};
  std::thread stopper([&stop] {
    std::this_thread::sleep_for(150ms);
    stop = true;
  });
  const auto start = std::chrono::steady_clock::now();
  const FetchResult r = client.fetch(3, 0, stop);  // a 404, then a 10 s backoff
  stopper.join();
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "stopped during backoff after: http 404");
  CHECK(std::chrono::steady_clock::now() - start < 5s);
}

TEST_CASE("keep-alive reuses one connection for successive fetches", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  for (int i = 0; i < 3; ++i) {
    const FetchResult r = client.fetch(0, 0, stop);
    CHECK(r.kind == FetchResult::Kind::Value);
  }
  // Every new connection arrives from a new ephemeral port.
  CHECK(srv.distinctRemotePorts() == 1);
}

TEST_CASE("the server receives the decoded BBOX, LAYERS, I and J", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(2, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  // South, west, north, east of the example box with nine decimals, as the server decodes them.
  const SeenRequest seen = srv.latestRequest();
  CHECK(seen.params.at("BBOX") == "-34.216344700,18.418579100,-34.184541800,18.462524400");
  CHECK(seen.params.at("LAYERS") == "Elevation_CCT_GLM2019_5m_DEM");
  CHECK(seen.params.at("I") == "2");
  CHECK(seen.params.at("J") == "0");
}

TEST_CASE("a layer name with special characters arrives decoded", "[elevationclient]") {
  CannedServer srv;
  Config cfg = configFor(srv.url());
  cfg.layer = "Elevation Layer+1&2%";
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  // Locks in how httplib re-encodes the query: the name must survive it in both parameters.
  const SeenRequest seen = srv.latestRequest();
  CHECK(seen.params.at("LAYERS") == "Elevation Layer+1&2%");
  CHECK(seen.params.at("QUERY_LAYERS") == "Elevation Layer+1&2%");
}

TEST_CASE("requests identify themselves as dl_elevation", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(srv.latestRequest().userAgents == std::vector<std::string>{"dl_elevation/" DL_ELEVATION_VERSION});
}

TEST_CASE("certificate verification rejects an untrusted loopback certificate without retrying",
          "[elevationclient]") {
  const LoopbackCertificate certificate;
  CannedServer srv(certificate);
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLServerVerification));
  CHECK(srv.hits[0].load() == 0);  // the handshake fails before any request is sent
}

TEST_CASE("insecure accepts an untrusted certificate", "[elevationclient]") {
  const LoopbackCertificate certificate;
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.insecure = true;
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 12.5f);
}

TEST_CASE("a CA bundle trusts the loopback certificate", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  const std::filesystem::path bundle = dir.path / "bundle.pem";
  certificate.writePem(bundle);
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.caBundle = bundle.string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Value);
  CHECK(r.value == 12.5f);
}

TEST_CASE("a missing CA bundle fails without retrying", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.caBundle = (dir.path / "missing.pem").string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLLoadingCerts));
  CHECK(srv.hits[0].load() == 0);  // loading the bundle fails before any request is sent
}

TEST_CASE("a CA bundle that failed to load keeps failing with the same error", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  CannedServer srv(certificate);
  Config cfg = configFor(srv.url());
  cfg.caBundle = (dir.path / "missing.pem").string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  // httplib loads the bundle once per client, so a second request would fail verification instead.
  for (int fetch = 0; fetch < 2; ++fetch) {
    const FetchResult r = client.fetch(0, 0, stop);
    CHECK(r.kind == FetchResult::Kind::Failed);
    CHECK(r.attempts == 1);
    CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLLoadingCerts));
  }
  CHECK(srv.hits[0].load() == 0);
}

TEST_CASE("a certificate that does not cover the host name fails without retrying", "[elevationclient]") {
  const LoopbackCertificate certificate;
  const TempDir dir;
  const std::filesystem::path bundle = dir.path / "bundle.pem";
  certificate.writePem(bundle);
  CannedServer srv(certificate);
  // The certificate's only subjectAltName is IP:127.0.0.1, which the name localhost does not match.
  Config cfg = configFor("https://localhost:" + std::to_string(srv.port) + "/wms");
  cfg.caBundle = bundle.string();
  ElevationClient client(cfg, fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 1);
  CHECK(r.error == "transport: " + httplib::to_string(httplib::Error::SSLServerHostnameVerification));
  CHECK(srv.hits[0].load() == 0);  // the host name is checked before any request is sent
}

TEST_CASE("a blank unparseable body is described by its size", "[elevationclient]") {
  CHECK(describeUnparseableBody("") == "warning: first unparseable response body was 0 bytes of whitespace");
  CHECK(describeUnparseableBody(" \t\r\n") ==
        "warning: first unparseable response body was 4 bytes of whitespace");
}

TEST_CASE("an unparseable body is shown up to 200 bytes", "[elevationclient]") {
  const std::string body = "<" + std::string(299, 'x');
  const std::string line = describeUnparseableBody(body);
  const std::string prefix = "warning: first unparseable response body (300 bytes, up to 200 shown): ";
  REQUIRE(line.rfind(prefix, 0) == 0);
  CHECK(line.substr(prefix.size()) == body.substr(0, 200));
}

TEST_CASE("an unparseable body is shown as printable ASCII on one line", "[elevationclient]") {
  // ESC, BEL, DEL, the UTF-8 encoding of the C1 control CSI (0xC2 0x9B) and a newline.
  const std::string body = std::string("<p>\x1b[31mred\a\x7f") + "\xC2\x9B" + "31m</p>\n";
  const std::string line = describeUnparseableBody(body);
  CHECK(line ==
        "warning: first unparseable response body (23 bytes, up to 200 shown): <p>.[31mred....31m</p>.");
  CHECK(line.find('\n') == std::string::npos);
}
```

- [ ] **Step 2: Add the files to CMake**

```cmake
add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp
  src/demdl/WmsQuery.cpp
  src/demdl/RetryPolicy.cpp
  src/demdl/DemRaster.cpp
  src/demdl/Downloader.cpp
  src/demdl/ElevationClient.cpp)
```

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp
    tests/test_DemRaster.cpp
    tests/test_StopToken.cpp
    tests/test_Downloader.cpp
    tests/test_ElevationClient.cpp)
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/demdl/ElevationClient.cpp`.

- [ ] **Step 4: Write `src/demdl/ElevationClient.h`**

```cpp
#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <string_view>

#include "demdl/Config.h"
#include "demdl/PixelFetcher.h"
#include "demdl/RetryPolicy.h"

namespace httplib {
class Client;
}

namespace demdl {

// Fetches one pixel's elevation over HTTP or HTTPS with keep-alive, retries and backoff.
// One instance per worker thread; instances are not thread-safe.
//
// Requests identify themselves as "dl_elevation/<version>". Timeouts: 10 s to connect, 10 s per
// write, 30 s per read and 60 s overall per request, so a server trickling its answer cannot hold a
// worker indefinitely. A stop cannot interrupt a request already on the wire. Once connected, a
// request ends within the 60 s overall cap, which counts from the start of the request; resolving,
// connecting (per address) and each TLS handshake wait are bounded separately by the 10 s connection
// timeout. The client relies on the process ignoring SIGPIPE, as main arranges: httplib's client
// sets neither SO_NOSIGPIPE nor MSG_NOSIGNAL. Two warnings go to stderr, each once per process across
// all instances: the error of the first failed attempt, marked "(retrying)" or "(not retried)", and
// the first unparseable body.
class ElevationClient : public PixelFetcher {
 public:
  // Uses cfg.wmsUrl, layer, box, width, height, insecure and caBundle. The box and the grid must
  // describe the raster being filled, which main's matches() check guarantees. cfg.maxRetries is not
  // read: the policy carries it, and main maps one onto the other.
  // Throws std::invalid_argument when cfg.wmsUrl does not parse, or when httplib cannot create a
  // client for its origin (is_valid() is false, for example without a TLS context).
  ElevationClient(const Config& cfg, RetryPolicy policy);
  ~ElevationClient() override;

  // Failed attempts are retried per the policy, except certificate errors (a CA bundle that cannot
  // be loaded, a chain that does not verify, a certificate that does not cover the host): those fail
  // at once, because only a configuration change cures them. A CA bundle that failed to load stays
  // failed: httplib loads it once per client, so every later fetch returns the same error, with
  // attempts == 1, without sending a request. An exception during an attempt becomes that attempt's
  // error ("exception: ..." or "unknown exception") and is retried like any other failure. Not
  // noexcept: building error text allocates.
  [[nodiscard]] FetchResult fetch(int x, int y, const std::atomic<bool>& stop) override;

 private:
  Config cfg_;
  RetryPolicy policy_;
  std::unique_ptr<httplib::Client> client_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
  std::optional<std::string> caBundleFailure_;  // the error of a failed CA-bundle load, once one failed
};

// The warning line for the first unparseable response body, without a trailing newline. A blank body
// is reported by its size, any other by its first 200 bytes with every byte outside printable ASCII
// shown as '.', so that the warning stays on one line and cannot drive the terminal. Exposed for
// tests.
[[nodiscard]] std::string describeUnparseableBody(std::string_view body);

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/ElevationClient.cpp`**

```cpp
#include "demdl/ElevationClient.h"

#include <httplib.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "demdl/WmsQuery.h"

namespace demdl {

namespace {

// Transport errors that no retry can cure, only a configuration change: the CA bundle cannot be
// loaded, the chain does not verify, or the certificate does not cover the host. Retrying would
// also hide the cause: httplib loads the bundle once per client, so a retry after a failed load
// reports the verification failure that follows instead.
bool needsConfigurationChange(httplib::Error error) {
  return error == httplib::Error::SSLLoadingCerts || error == httplib::Error::SSLServerVerification ||
         error == httplib::Error::SSLServerHostnameVerification;
}

// Sleeps in 100 ms slices; returns false as soon as stop is set.
bool sleepUnlessStopped(std::chrono::milliseconds delay, const std::atomic<bool>& stop) {
  std::chrono::milliseconds remaining = delay;
  while (remaining.count() > 0) {
    if (stop.load()) return false;
    const std::chrono::milliseconds slice = std::min(remaining, std::chrono::milliseconds(100));
    std::this_thread::sleep_for(slice);
    remaining -= slice;
  }
  return !stop.load();
}

// Writes the error of the first failed attempt in the process, once across all instances, so that a
// wrong path, a bad CA bundle or an unverifiable chain shows within seconds even without --verbose.
// The suffix says whether fetch retries, so that a transient failure does not look alarming.
void logFirstFailureOnce(const std::string& error, bool retrying) noexcept {
  static std::once_flag flag;
  try {
    // stderr, so it cannot corrupt the progress lines on stdout.
    std::call_once(flag, [&error, retrying] {
      const char* const suffix = retrying ? " (retrying)" : " (not retried)";
      std::cerr << "warning: first failed request: " + error + suffix + "\n";
    });
  } catch (...) {
    // A warning must never become a failure of its own.
  }
}

// Writes the first unparseable body of the process, once across all instances.
void logUnparseableBodyOnce(std::string_view body) noexcept {
  static std::once_flag flag;
  try {
    // stderr, so it cannot corrupt the progress lines on stdout.
    std::call_once(flag, [body] { std::cerr << describeUnparseableBody(body) + "\n"; });
  } catch (...) {
    // A warning must never become a failure of its own.
  }
}

}  // namespace

std::string describeUnparseableBody(std::string_view body) {
  const std::string prefix = "warning: first unparseable response body ";
  if (body.find_first_not_of(" \t\n\v\f\r") == std::string_view::npos) {
    // Blank text would print as nothing, so give the size instead.
    return prefix + "was " + std::to_string(body.size()) + " bytes of whitespace";
  }
  // Every byte outside printable ASCII becomes '.', so the warning stays on one line and only
  // printable ASCII reaches the terminal: neither control characters nor UTF-8-encoded C1 controls.
  std::string shown(body.substr(0, 200));
  for (char& c : shown) {
    const auto byte = static_cast<unsigned char>(c);
    if (byte < 0x20 || byte > 0x7E) c = '.';
  }
  return prefix + "(" + std::to_string(body.size()) + " bytes, up to 200 shown): " + shown;
}

ElevationClient::ElevationClient(const Config& cfg, RetryPolicy policy)
    : cfg_(cfg), policy_(policy), rng_(std::random_device{}()) {
  const auto endpoint = parseWmsUrl(cfg_.wmsUrl);
  if (!endpoint) throw std::invalid_argument("invalid WMS URL: " + cfg_.wmsUrl);
  client_ = std::make_unique<httplib::Client>(endpoint->origin);
  // httplib reports a client it could not set up through is_valid(); the calls below could then
  // dereference a null implementation.
  if (!client_->is_valid()) {
    throw std::invalid_argument("cannot create an HTTP client for origin: " + endpoint->origin);
  }
  client_->set_default_headers({{"User-Agent", "dl_elevation/" DL_ELEVATION_VERSION}});
  client_->set_keep_alive(true);
  client_->set_connection_timeout(10, 0);
  client_->set_read_timeout(30, 0);
  client_->set_write_timeout(10, 0);
  client_->set_max_timeout(60 * 1000);  // milliseconds, counted from the start of each request
  client_->enable_server_certificate_verification(!cfg_.insecure);
  if (!cfg_.caBundle.empty()) client_->set_ca_cert_path(cfg_.caBundle);
}

ElevationClient::~ElevationClient() = default;

FetchResult ElevationClient::fetch(int x, int y, const std::atomic<bool>& stop) {
  FetchResult result;
  if (caBundleFailure_) {
    // httplib loads the CA bundle once per client, so a new request would fail verification instead
    // and hide the cause. Report the load failure again without sending anything.
    result.kind = FetchResult::Kind::Failed;
    result.attempts = 1;
    result.error = *caBundleFailure_;
    return result;
  }
  for (int attempt = 0;; ++attempt) {
    result.attempts = attempt + 1;
    std::string error;
    bool retryable = true;
    // fetch must not throw (see PixelFetcher), so building the target, the request and the parse
    // run inside the try, and an exception is this attempt's error.
    try {
      const httplib::Result response = client_->Get(featureInfoUrl(cfg_, x, y));
      if (!response) {
        error = "transport: " + httplib::to_string(response.error());
        retryable = !needsConfigurationChange(response.error());
        if (response.error() == httplib::Error::SSLLoadingCerts) caBundleFailure_ = error;
      } else if (response->status != 200) {
        error = "http " + std::to_string(response->status);
      } else {
        const ParseResult parsed = parseElevation(response->body);
        if (parsed.kind == ParseResult::Kind::Value) {
          result.kind = FetchResult::Kind::Value;
          result.value = static_cast<float>(parsed.value);
          return result;
        }
        if (parsed.kind == ParseResult::Kind::NoData) {
          result.kind = FetchResult::Kind::NoData;
          return result;
        }
        error = "unparseable body";
        logUnparseableBodyOnce(response->body);
      }
    } catch (const std::exception& e) {
      error = std::string("exception: ") + e.what();
    } catch (...) {
      error = "unknown exception";
    }
    const bool retrying = retryable && attempt < policy_.maxRetries && !stop.load();
    logFirstFailureOnce(error, retrying);
    if (!retrying) {
      result.kind = FetchResult::Kind::Failed;
      result.error = std::move(error);
      return result;
    }
    const std::chrono::milliseconds delay = policy_.delayFor(attempt, unit_(rng_));
    if (!sleepUnlessStopped(delay, stop)) {
      result.kind = FetchResult::Kind::Failed;
      result.error = "stopped during backoff after: " + error;
      return result;
    }
  }
}

}  // namespace demdl
```

Notes for the engineer: `httplib::Client` built from an `https://` origin is the TLS client; the OpenSSL define comes from the `httplib::httplib` target, which is why Task 1 links that name. The certificate and CA calls are no-ops for plain `http://`; the TLS cases serve a certificate the test generates through the OpenSSL API and install it with `SSLServer`'s context callback, because httplib 0.46 has no constructor taking an `X509*` and an `EVP_PKEY*`. `httplib::Result` is move-only; initialising a `const` local from the call is fine under guaranteed copy elision.

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[elevationclient]"`
Expected: `All tests passed (N assertions in 20 test cases)`. The stop test takes about 0.2 s; the refused-connection tests are instant.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/demdl/ElevationClient.h src/demdl/ElevationClient.cpp tests/test_ElevationClient.cpp
git commit -m "feat: ElevationClient with keep-alive, TLS verification, retries and backoff" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 10: ConsoleReporter

Implements the headless sink from spec section 8.

**Files:**
- Create: `src/demdl/ConsoleReporter.h`
- Create: `src/demdl/ConsoleReporter.cpp`
- Create: `tests/test_ConsoleReporter.cpp`
- Modify: `CMakeLists.txt` (source lists)

- [ ] **Step 1: Write the failing test `tests/test_ConsoleReporter.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "demdl/ConsoleReporter.h"

using Catch::Matchers::ContainsSubstring;
using demdl::ConsoleReporter;
using demdl::PixelEvent;
using demdl::PixelState;
using demdl::RowReport;
using demdl::RunPlan;
using demdl::RunSummary;
using demdl::StopReason;
using demdl::formatDuration;
using demdl::writeSummary;

namespace {

// The lines of `text`, without their newlines.
std::vector<std::string> linesOf(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

// A string buffer that counts sync() calls, which is where std::ostream::flush() ends up, and
// remembers how many characters had been written at the last one.
class SyncCountingBuf : public std::stringbuf {
 public:
  int syncs = 0;
  std::size_t syncedSize = 0;

 protected:
  int sync() override {
    ++syncs;
    syncedSize = str().size();
    return std::stringbuf::sync();
  }
};

}  // namespace

TEST_CASE("formatDuration picks units by magnitude", "[consolereporter]") {
  CHECK(formatDuration(41.2) == "41.2s");
  CHECK(formatDuration(725.0) == "12m05s");
  CHECK(formatDuration(48667.0) == "13h31m");
  CHECK(formatDuration(50520.0) == "14h02m");
  CHECK(formatDuration(7199.0) == "1h59m");  // minutes are truncated, not rounded
}

TEST_CASE("formatDuration rounds before it picks the unit and tames odd inputs", "[consolereporter]") {
  CHECK(formatDuration(59.94) == "59.9s");
  CHECK(formatDuration(59.95) == "1m00s");  // "%.1f" alone would print 60.0
  CHECK(formatDuration(60.0) == "1m00s");
  CHECK(formatDuration(3599.4) == "59m59s");
  CHECK(formatDuration(3599.6) == "1h00m");  // not 60m00s
  CHECK(formatDuration(-1.0) == "0.0s");
  CHECK(formatDuration(std::nan("")) == "0.0s");
  CHECK(formatDuration(1e300) == "1000000000h00m");  // capped at a billion hours
  CHECK(formatDuration(std::numeric_limits<double>::infinity()) == "1000000000h00m");
}

TEST_CASE("the starting line gives what is left and the whole", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRunStarted(RunPlan{3670016, 2422720, 1100, 1792});
  CHECK(out.str() == "starting: 2422720 pixels in 1100 rows remaining of 3670016 pixels in 1792 rows\n");
}

TEST_CASE("a saved row prints progress, rate and ETA", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  // The run set out to fetch 2424771 pixels; this row attempts 2051 of them, which leaves 2422720.
  reporter.onRunStarted(RunPlan{3670016, 2424771, 1184, 1792});
  reporter.onRowSaved(RowReport{811, 1792, 2048, 0, 3, std::chrono::duration<double>(41.2), 1247296, 3670016});
  const std::string text = out.str();
  CHECK_THAT(text,
             ContainsSubstring("starting: 2424771 pixels in 1184 rows remaining of 3670016 pixels in 1792 rows\n"));
  CHECK_THAT(text, ContainsSubstring("row 812/1792  done 1247296/3670016 (34.0%)  row: 2048 ok 0 nodata 3 failed "
                                     "41.2s  49.8 px/s  eta 13h31m"));
}

TEST_CASE("a row without elapsed time prints no ETA", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRowSaved(RowReport{0, 4, 10, 0, 0, std::chrono::duration<double>(0.0), 10, 40});
  CHECK(out.str() == "row 1/4  done 10/40 (25.0%)  row: 10 ok 0 nodata 0 failed 0.0s  0.0 px/s  eta --\n");
}

TEST_CASE("rate and ETA cover the last ten saved rows", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRunStarted(RunPlan{2000, 2000, 20, 20});
  // Row 0 attempts 100 pixels in an absurd 1e6 s. Rows 1 to 10 attempt 100 each (90 ok, 6 nodata,
  // 4 failed) in 2 s, so each of them adds 96 to the 2000 pixels' done count.
  reporter.onRowSaved(RowReport{0, 20, 100, 0, 0, std::chrono::duration<double>(1e6), 100, 2000});
  for (int row = 1; row <= 10; ++row) {
    reporter.onRowSaved(RowReport{row, 20, 90, 6, 4, std::chrono::duration<double>(2.0),
                                  100 + 96 * static_cast<std::uint64_t>(row), 2000});
  }
  const std::vector<std::string> lines = linesOf(out.str());
  REQUIRE(lines.size() == 12);  // the starting line, then eleven row lines
  // The tenth row line still counts row 0: 1000 pixels in 1000018 s.
  CHECK_THAT(lines[10], ContainsSubstring("  0.0 px/s  "));
  // The eleventh drops it: 1000 pixels in 20 s is 50 px/s, and 900 of the 2000 pixels are unattempted.
  CHECK(lines[11] == "row 11/20  done 1060/2000 (53.0%)  row: 90 ok 6 nodata 4 failed 2.0s  50.0 px/s  eta 18.0s");
}

TEST_CASE("the ETA leaves out pixels that failed this run", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRunStarted(RunPlan{100, 100, 1, 1});
  // Every pixel has been attempted. The 10 failures stay NaN, but this run does not retry them.
  reporter.onRowSaved(RowReport{0, 1, 90, 0, 10, std::chrono::duration<double>(10.0), 90, 100});
  const std::vector<std::string> lines = linesOf(out.str());
  REQUIRE(lines.size() == 2);
  CHECK(lines[1] == "row 1/1  done 90/100 (90.0%)  row: 90 ok 0 nodata 10 failed 10.0s  10.0 px/s  eta 0.0s");
}

TEST_CASE("a new run starts a new window and a new count", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRunStarted(RunPlan{2000, 2000, 20, 20});
  reporter.onRowSaved(RowReport{0, 20, 100, 0, 0, std::chrono::duration<double>(100.0), 100, 2000});
  reporter.onRowSaved(RowReport{1, 20, 100, 0, 0, std::chrono::duration<double>(100.0), 200, 2000});
  reporter.onRunStarted(RunPlan{2000, 1800, 18, 20});
  reporter.onRowSaved(RowReport{2, 20, 100, 0, 0, std::chrono::duration<double>(2.0), 300, 2000});
  const std::vector<std::string> lines = linesOf(out.str());
  REQUIRE(lines.size() == 5);
  // Only the second run's row counts: 100 pixels in 2 s, with 1700 of its 1800 pixels left. Keeping
  // the first run's rows would give 1.5 px/s, and keeping its attempts an ETA of 30.0s.
  CHECK(lines[4] == "row 3/20  done 300/2000 (15.0%)  row: 100 ok 0 nodata 0 failed 2.0s  50.0 px/s  eta 34.0s");
}

TEST_CASE("the summary lists totals and the retry hint", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  RunSummary s;
  s.pixelsFetched = 3669980;
  s.noData = 12034;
  s.failed = 3;
  s.remainingNaN = 3;
  s.rowsSaved = 1792;
  s.elapsed = std::chrono::duration<double>(50520.0);
  reporter.onRunFinished(s);
  CHECK_THAT(out.str(),
             ContainsSubstring("done: 3669980 values, 12034 nodata, 3 failed, 1792 rows saved, 14h02m elapsed"));
  CHECK_THAT(out.str(), ContainsSubstring("rerun with the same arguments to retry the 3 failed pixels"));
  CHECK_THAT(out.str(), !ContainsSubstring("stopped:"));

  s.failed = 1;
  s.remainingNaN = 1;
  std::ostringstream one;
  ConsoleReporter oneFailure(one, false);
  oneFailure.onRunFinished(s);
  CHECK_THAT(one.str(), ContainsSubstring("rerun with the same arguments to retry the 1 failed pixel\n"));
}

TEST_CASE("a stopped run names the reason", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  RunSummary s;
  s.stopReason = StopReason::Signal;
  s.stopMessage = "interrupted";
  reporter.onRunFinished(s);
  // The totals come first; with nothing failed and nothing left there is no hint.
  CHECK(out.str() == "done: 0 values, 0 nodata, 0 failed, 0 rows saved, 0.0s elapsed\nstopped: interrupted\n");
}

TEST_CASE("a run that stops early says how to resume", "[consolereporter]") {
  RunSummary s;
  s.pixelsFetched = 1490;
  s.noData = 8;
  s.remainingNaN = 500;
  s.rowsSaved = 3;
  s.elapsed = std::chrono::duration<double>(30.0);
  s.stopReason = StopReason::Signal;
  s.stopMessage = "interrupted";

  std::ostringstream noFailures;
  writeSummary(noFailures, s);
  CHECK(noFailures.str() ==
        "done: 1490 values, 8 nodata, 0 failed, 3 rows saved, 30.0s elapsed\n"
        "stopped: interrupted\n"
        "rerun with the same arguments to resume the remaining 500 pixels\n");

  s.failed = 2;
  std::ostringstream twoFailures;
  writeSummary(twoFailures, s);
  CHECK(twoFailures.str() ==
        "done: 1490 values, 8 nodata, 2 failed, 3 rows saved, 30.0s elapsed\n"
        "stopped: interrupted\n"
        "rerun with the same arguments to resume the remaining 500 pixels (2 failed this run)\n");

  s.failed = 0;
  s.remainingNaN = 1;
  std::ostringstream onePixel;
  writeSummary(onePixel, s);
  CHECK_THAT(onePixel.str(), ContainsSubstring("rerun with the same arguments to resume the remaining 1 pixel\n"));
}

TEST_CASE("writeSummary writes the lines onRunFinished prints", "[consolereporter]") {
  RunSummary s;
  s.pixelsFetched = 1490;
  s.noData = 8;
  s.failed = 2;
  s.remainingNaN = 500;
  s.rowsSaved = 3;
  s.elapsed = std::chrono::duration<double>(30.0);
  s.stopReason = StopReason::ConsecutiveFailures;
  s.stopMessage = "server unreachable: 100 consecutive failures; last error: http 503";
  std::ostringstream direct;
  writeSummary(direct, s);
  std::ostringstream viaSink;
  ConsoleReporter reporter(viaSink, false);
  reporter.onRunFinished(s);
  CHECK(linesOf(direct.str()).size() == 3);
  CHECK(direct.str() == viaSink.str());
}

TEST_CASE("verbose mode prints one line per resolved pixel", "[consolereporter]") {
  std::ostringstream quiet;
  ConsoleReporter silent(quiet, false);
  silent.onPixelState(PixelEvent{12, 812, PixelState::Value, 123.4f, 1, {}});
  CHECK(quiet.str().empty());

  std::ostringstream out;
  ConsoleReporter verbose(out, true);
  verbose.onPixelState(PixelEvent{12, 812, PixelState::Value, 123.4f, 1, {}});
  verbose.onPixelState(PixelEvent{13, 812, PixelState::NoData, 0.0f, 1, {}});
  verbose.onPixelState(PixelEvent{14, 812, PixelState::Failed, 0.0f, 9, "http 500"});
  verbose.onPixelState(PixelEvent{15, 812, PixelState::Queued, 0.0f, 0, {}});
  verbose.onPixelState(PixelEvent{15, 812, PixelState::InFlight, 0.0f, 0, {}});
  verbose.onPixelState(PixelEvent{15, 812, PixelState::Failed, 0.0f, 1, "transport: SSL server verification failed"});
  verbose.onPixelState(PixelEvent{16, 812, PixelState::Value, 5.0f, 2, {}});
  // No attempt counted, as for an exception the Downloader caught: the line carries no count.
  verbose.onPixelState(PixelEvent{17, 812, PixelState::Failed, 0.0f, 0, "unknown exception"});
  verbose.onPixelState(PixelEvent{18, 812, PixelState::Failed, 0.0f, -1, "exception: boom"});
  CHECK(out.str() ==
        "x=12 y=812 123.4 m (1 attempt)\n"
        "x=13 y=812 nodata\n"
        "x=14 y=812 failed after 9 attempts: http 500\n"
        "x=15 y=812 failed after 1 attempt: transport: SSL server verification failed\n"
        "x=16 y=812 5.0 m (2 attempts)\n"
        "x=17 y=812 failed: unknown exception\n"
        "x=18 y=812 failed: exception: boom\n");
}

TEST_CASE("the starting, row and finish lines are flushed, pixel lines are not", "[consolereporter]") {
  SyncCountingBuf buf;
  std::ostream out(&buf);
  ConsoleReporter reporter(out, true);

  reporter.onRunStarted(RunPlan{40, 40, 4, 4});
  CHECK(buf.syncs >= 1);
  CHECK(buf.syncedSize == buf.str().size());

  const int beforePixels = buf.syncs;
  reporter.onPixelState(PixelEvent{0, 0, PixelState::Queued, 0.0f, 0, {}});
  reporter.onPixelState(PixelEvent{0, 0, PixelState::InFlight, 0.0f, 0, {}});
  reporter.onPixelState(PixelEvent{0, 0, PixelState::Value, 12.5f, 1, {}});
  reporter.onPixelState(PixelEvent{1, 0, PixelState::NoData, 0.0f, 1, {}});
  reporter.onPixelState(PixelEvent{2, 0, PixelState::Failed, 0.0f, 3, "http 500"});
  CHECK_THAT(buf.str(), ContainsSubstring("x=2 y=0 failed after 3 attempts: http 500\n"));
  CHECK(buf.syncs == beforePixels);

  reporter.onRowSaved(RowReport{0, 4, 1, 1, 1, std::chrono::duration<double>(1.0), 2, 40});
  CHECK(buf.syncs > beforePixels);
  CHECK(buf.syncedSize == buf.str().size());

  const int beforeFinish = buf.syncs;
  RunSummary s;
  s.failed = 1;
  s.stopReason = StopReason::Signal;
  s.stopMessage = "interrupted";
  reporter.onRunFinished(s);
  CHECK(buf.syncs > beforeFinish);
  CHECK(buf.syncedSize == buf.str().size());
}

TEST_CASE("concurrent pixel events print whole lines", "[consolereporter]") {
  constexpr int kThreads = 4;
  constexpr int kEvents = 1000;
  std::ostringstream out;
  ConsoleReporter reporter(out, true);
  std::atomic<bool> go{false};
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&reporter, &go, t] {
      while (!go.load()) std::this_thread::yield();
      for (int i = 0; i < kEvents; ++i) {
        if (i % 3 == 0) reporter.onPixelState(PixelEvent{i, t, PixelState::Value, static_cast<float>(i) + 0.5f, 1, {}});
        if (i % 3 == 1) reporter.onPixelState(PixelEvent{i, t, PixelState::NoData, 0.0f, 1, {}});
        if (i % 3 == 2) reporter.onPixelState(PixelEvent{i, t, PixelState::Failed, 0.0f, 3, "http 503"});
      }
    });
  }
  go = true;
  for (std::thread& thread : threads) thread.join();

  std::vector<std::string> expected;
  for (int t = 0; t < kThreads; ++t) {
    for (int i = 0; i < kEvents; ++i) {
      const std::string pixel = "x=" + std::to_string(i) + " y=" + std::to_string(t);
      if (i % 3 == 0) expected.push_back(pixel + " " + std::to_string(i) + ".5 m (1 attempt)");
      if (i % 3 == 1) expected.push_back(pixel + " nodata");
      if (i % 3 == 2) expected.push_back(pixel + " failed after 3 attempts: http 503");
    }
  }
  std::vector<std::string> lines = linesOf(out.str());
  CHECK(lines.size() == static_cast<std::size_t>(kThreads * kEvents));
  std::sort(lines.begin(), lines.end());
  std::sort(expected.begin(), expected.end());
  CHECK(lines == expected);
}
```

- [ ] **Step 2: Add the files to CMake**

```cmake
add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp
  src/demdl/WmsQuery.cpp
  src/demdl/RetryPolicy.cpp
  src/demdl/DemRaster.cpp
  src/demdl/Downloader.cpp
  src/demdl/ElevationClient.cpp
  src/demdl/ConsoleReporter.cpp)
```

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp
    tests/test_DemRaster.cpp
    tests/test_StopToken.cpp
    tests/test_Downloader.cpp
    tests/test_ElevationClient.cpp
    tests/test_ConsoleReporter.cpp)
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/demdl/ConsoleReporter.cpp`.

- [ ] **Step 4: Write `src/demdl/ConsoleReporter.h`**

```cpp
#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <mutex>
#include <string>

#include "demdl/ProgressSink.h"

namespace demdl {

// "41.2s" under a minute, "12m05s" under an hour, otherwise "13h31m" (minutes truncated). The unit
// is chosen after rounding, so 59.95 gives "1m00s" and 3599.6 gives "1h00m". NaN and negative
// inputs give "0.0s", and inputs above a billion hours give "1000000000h00m".
[[nodiscard]] std::string formatDuration(double seconds);

// Writes the finish lines of a run: the totals, "stopped: <message>" if it stopped, and, while the
// file still has NaN pixels, how to finish the job (retry this run's failures, or resume the rest).
// Does not flush. ConsoleReporter::onRunFinished prints these lines; main calls this itself after a
// --preview run, whose sink prints nothing.
void writeSummary(std::ostream& out, const RunSummary& summary);

// Headless progress: one line per saved row, optional per-pixel lines, a final summary.
// Each callback writes its lines whole under one mutex, so concurrent pixel events never interleave.
// The starting, row and finish lines are flushed as they are written, so a redirected stdout logs
// each row when it is saved; the frequent verbose pixel lines are left to the stream's buffering.
// Callbacks do not throw in practice: lines are formatted into fixed buffers and short strings.
// They are not noexcept only because std::mutex::lock and std::string construction may formally
// throw. The stream must keep exceptions disabled; once it fails (closed stdout, EPIPE), later
// lines are silently dropped.
class ConsoleReporter : public ProgressSink {
 public:
  // `out` must outlive the reporter. `verbose` adds a line per resolved pixel.
  ConsoleReporter(std::ostream& out, bool verbose);

  void onRunStarted(const RunPlan& plan) override;
  void onPixelState(const PixelEvent& event) override;
  void onRowSaved(const RowReport& report) override;
  void onRunFinished(const RunSummary& summary) override;

 private:
  struct RowSample {
    double pixels = 0.0;   // ok + noData + failed
    double seconds = 0.0;  // the row's elapsed time
  };

  std::ostream& out_;
  const bool verbose_;
  std::mutex mutex_;
  // The last ten saved rows, for the rate: a ring that never allocates. Unused slots are zero.
  std::array<RowSample, 10> window_{};
  std::size_t next_ = 0;  // the slot the next saved row overwrites
  // For the ETA: the pixels the run set out to fetch and how many it has attempted. A run does not
  // retry its own failures, so a pixel that failed no longer counts as work left.
  std::uint64_t pixelsTodo_ = 0;
  std::uint64_t attempted_ = 0;
};

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/ConsoleReporter.cpp`**

```cpp
#include "demdl/ConsoleReporter.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ostream>

namespace demdl {

std::string formatDuration(double seconds) {
  // NaN fails the comparison as well, so it joins the negatives at zero. The cap keeps llround in range.
  constexpr double kMaxSeconds = 3600.0 * 1e9;  // a billion hours
  if (!(seconds > 0.0)) seconds = 0.0;
  if (seconds > kMaxSeconds) seconds = kMaxSeconds;
  char buf[48];
  // Round first and choose the unit from the rounded value, which rules out "60.0s" and "60m00s".
  const long long tenths = std::llround(seconds * 10.0);
  if (tenths < 600) {
    std::snprintf(buf, sizeof buf, "%lld.%llds", tenths / 10, tenths % 10);
    return buf;
  }
  const long long total = std::llround(seconds);
  if (total < 3600) {
    std::snprintf(buf, sizeof buf, "%lldm%02llds", total / 60, total % 60);
    return buf;
  }
  std::snprintf(buf, sizeof buf, "%lldh%02lldm", total / 3600, (total % 3600) / 60);
  return buf;
}

void writeSummary(std::ostream& out, const RunSummary& s) {
  out << "done: " << s.pixelsFetched << " values, " << s.noData << " nodata, " << s.failed << " failed, "
      << s.rowsSaved << " rows saved, " << formatDuration(s.elapsed.count()) << " elapsed\n";
  if (s.stopReason != StopReason::None) out << "stopped: " << s.stopMessage << "\n";
  if (s.remainingNaN == 0) return;
  out << "rerun with the same arguments to ";
  if (s.failed > 0 && s.remainingNaN == s.failed) {
    out << "retry the " << s.failed << (s.failed == 1 ? " failed pixel" : " failed pixels");
  } else {
    out << "resume the remaining " << s.remainingNaN << (s.remainingNaN == 1 ? " pixel" : " pixels");
    if (s.failed > 0) out << " (" << s.failed << " failed this run)";
  }
  out << "\n";
}

ConsoleReporter::ConsoleReporter(std::ostream& out, bool verbose) : out_(out), verbose_(verbose) {}

void ConsoleReporter::onRunStarted(const RunPlan& plan) {
  std::lock_guard<std::mutex> lock(mutex_);
  window_.fill(RowSample{});
  next_ = 0;
  pixelsTodo_ = plan.pixelsTodo;
  attempted_ = 0;
  out_ << "starting: " << plan.pixelsTodo << " pixels in " << plan.rowsTodo << " rows remaining of "
       << plan.pixelsTotal << " pixels in " << plan.rowsTotal << " rows\n" << std::flush;
}

void ConsoleReporter::onPixelState(const PixelEvent& e) {
  // Queued and InFlight print nothing, so they return before taking the lock.
  if (!verbose_ || e.state == PixelState::Queued || e.state == PixelState::InFlight) return;
  std::lock_guard<std::mutex> lock(mutex_);
  switch (e.state) {
    case PixelState::Value: {
      char value[48];  // room for -FLT_MAX at one decimal
      std::snprintf(value, sizeof value, "%.1f", static_cast<double>(e.value));
      out_ << "x=" << e.x << " y=" << e.y << " " << value << " m (" << e.attempts
           << (e.attempts == 1 ? " attempt)" : " attempts)") << "\n";
      break;
    }
    case PixelState::NoData:
      out_ << "x=" << e.x << " y=" << e.y << " nodata\n";
      break;
    case PixelState::Failed:
      // An exception the Downloader caught leaves attempts at 0, so there is no count to print.
      out_ << "x=" << e.x << " y=" << e.y << " failed";
      if (e.attempts >= 1) out_ << " after " << e.attempts << (e.attempts == 1 ? " attempt" : " attempts");
      out_ << ": " << e.error << "\n";
      break;
    case PixelState::Queued:
    case PixelState::InFlight:
      break;
  }
}

void ConsoleReporter::onRowSaved(const RowReport& r) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::uint64_t rowAttempted =
      static_cast<std::uint64_t>(r.ok) + static_cast<std::uint64_t>(r.noData) + static_cast<std::uint64_t>(r.failed);
  attempted_ += rowAttempted;
  window_[next_] = RowSample{static_cast<double>(rowAttempted), r.elapsed.count()};
  next_ = (next_ + 1) % window_.size();
  double windowPixels = 0.0;
  double windowSeconds = 0.0;
  for (const RowSample& w : window_) {
    windowPixels += w.pixels;
    windowSeconds += w.seconds;
  }
  const double rate = windowSeconds > 0.0 ? windowPixels / windowSeconds : 0.0;
  // "done" and the percentage describe the file; the ETA counts only pixels the run has yet to attempt.
  const double percent =
      r.pixelsTotal > 0 ? 100.0 * static_cast<double>(r.pixelsDoneTotal) / static_cast<double>(r.pixelsTotal) : 100.0;
  const std::uint64_t remaining = pixelsTodo_ > attempted_ ? pixelsTodo_ - attempted_ : 0;
  const std::string eta = rate > 0.0 ? formatDuration(static_cast<double>(remaining) / rate) : std::string("--");
  const std::string rowTime = formatDuration(r.elapsed.count());
  char line[256];
  std::snprintf(line, sizeof line,
                "row %d/%d  done %llu/%llu (%.1f%%)  row: %d ok %d nodata %d failed %s  %.1f px/s  eta %s", r.row + 1,
                r.rowsTotal, static_cast<unsigned long long>(r.pixelsDoneTotal),
                static_cast<unsigned long long>(r.pixelsTotal), percent, r.ok, r.noData, r.failed, rowTime.c_str(),
                rate, eta.c_str());
  out_ << line << "\n" << std::flush;
}

void ConsoleReporter::onRunFinished(const RunSummary& s) {
  std::lock_guard<std::mutex> lock(mutex_);
  writeSummary(out_, s);
  out_.flush();
}

}  // namespace demdl
```

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[consolereporter]"`
Expected: `All tests passed (N assertions in 15 test cases)`. The four-thread case takes about 0.05 s.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/demdl/ConsoleReporter.h src/demdl/ConsoleReporter.cpp tests/test_ConsoleReporter.cpp
git commit -m "feat: ConsoleReporter with per-row progress lines, ETA and summary" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 11: Command-line parsing

Implements spec section 5 with CLI11 and the Cli tests in section 11.

**Files:**
- Create: `src/app/Cli.h`
- Create: `src/app/Cli.cpp`
- Create: `tests/test_Cli.cpp`
- Modify: `CMakeLists.txt` (new `demdl_cli` object library, test sources and link)

- [ ] **Step 1: Write the failing test `tests/test_Cli.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "TempDir.h"
#include "app/Cli.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;
using demdl::Config;
using demdl::cli::ParseOutcome;
using demdl::cli::parseArgs;

namespace {

// argv[0] as the shell passes it when the program is run from the repository root.
const char* const kArgv0 = "./build/dl_elevation";

struct Parsed {
  Config config;
  ParseOutcome outcome;
  std::string out;
  std::string err;
};

Parsed parse(std::vector<const char*> argv) {
  argv.insert(argv.begin(), kArgv0);
  Parsed p;
  std::ostringstream out;
  std::ostringstream err;
  p.outcome = parseArgs(static_cast<int>(argv.size()), argv.data(), p.config, out, err);
  p.out = out.str();
  p.err = err.str();
  return p;
}

const std::vector<const char*> kExample = {"--north", "-34.1845418", "--south",  "-34.2163447", "--west",
                                            "18.4185791", "--east", "18.4625244", "--width", "2048",
                                            "--height", "1792", "--output", "simonstown.tif"};

// kExample with `flag` given `value`: the example's own value is replaced, or the pair is appended.
std::vector<const char*> exampleWith(const char* flag, const char* value) {
  std::vector<const char*> argv = kExample;
  const auto it =
      std::find_if(argv.begin(), argv.end(), [flag](const char* arg) { return std::string_view(arg) == flag; });
  if (it == argv.end()) {
    argv.push_back(flag);
    argv.push_back(value);
  } else {
    *std::next(it) = value;
  }
  return argv;
}

// Creates an empty file and returns its path. The CLI checks only that a CA bundle exists.
std::string createFile(const std::filesystem::path& path) {
  std::ofstream file(path);
  REQUIRE(file.is_open());
  return path.string();
}

}  // namespace

// No test name here starts with '-': the test binary would parse such a name as an option when ctest runs it.
TEST_CASE("the example command line fills the Config", "[cli]") {
  std::vector<const char*> argv = kExample;
  argv.push_back("--preview");
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK_THAT(p.config.box.north, WithinAbs(-34.1845418, 1e-12));
  CHECK_THAT(p.config.box.south, WithinAbs(-34.2163447, 1e-12));
  CHECK_THAT(p.config.box.east, WithinAbs(18.4625244, 1e-12));
  CHECK_THAT(p.config.box.west, WithinAbs(18.4185791, 1e-12));
  CHECK(p.config.width == 2048);
  CHECK(p.config.height == 1792);
  CHECK(p.config.output == "simonstown.tif");
  CHECK(p.config.preview);
  CHECK(p.config.threads == 8);
  CHECK(p.config.maxRetries == 8);
  CHECK(p.config.maxConsecutiveFailures == 100);
  CHECK(p.config.layer == "Elevation_CCT_GLM2019_5m_DEM");
  CHECK(p.config.wmsUrl == "https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets");
  CHECK_FALSE(p.config.insecure);
  CHECK_FALSE(p.config.dryRun);
  CHECK(p.config.validate().empty());
}

TEST_CASE("the -o short form sets the output", "[cli]") {
  std::vector<const char*> argv = kExample;
  const auto it = std::find(argv.begin(), argv.end(), std::string("--output"));
  REQUIRE(it != argv.end());
  *it = "-o";
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.output == "simonstown.tif");
}

TEST_CASE("a missing required flag is a usage error", "[cli]") {
  const std::string flag =
      GENERATE(as<std::string>{}, "--north", "--south", "--west", "--east", "--width", "--height", "--output");
  std::vector<const char*> argv = kExample;
  const auto it = std::find(argv.begin(), argv.end(), flag);
  REQUIRE(it != argv.end());
  argv.erase(it, it + 2);  // the flag and its value
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, StartsWith("error: "));
  CHECK_THAT(p.err, ContainsSubstring(flag + " is required"));
}

TEST_CASE("the --help flag prints usage and exits successfully", "[cli]") {
  const Parsed p = parse({"--help"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("dl_elevation [OPTIONS]"));
  CHECK_THAT(p.out, !ContainsSubstring(kArgv0));  // the program's name, not the path it was run as
  CHECK_THAT(p.out, ContainsSubstring("--wms-url"));
  CHECK_THAT(p.out, ContainsSubstring("--max-consecutive-failures"));
  CHECK_THAT(p.out, ContainsSubstring("(1..64)"));
  CHECK_THAT(p.out, ContainsSubstring("(0..100)"));
}

TEST_CASE("help wins over an invalid value", "[cli]") {
  std::vector<const char*> argv = exampleWith("--north", "1,5");
  argv.push_back("--help");
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("dl_elevation [OPTIONS]"));
  CHECK(p.err.empty());
}

TEST_CASE("the --version flag prints the project version", "[cli]") {
  const Parsed p = parse({"--version"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring(DL_ELEVATION_VERSION));
}

TEST_CASE("network, TLS and mode flags set their fields", "[cli]") {
  std::vector<const char*> argv = kExample;
  for (const char* a : {"--threads", "3", "--max-retries", "0", "--max-consecutive-failures", "5", "--insecure",
                        "--verbose", "--dry-run", "--overwrite", "--layer", "Other", "--wms-url",
                        "http://127.0.0.1:8080/wms"}) {
    argv.push_back(a);
  }
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.threads == 3);
  CHECK(p.config.maxRetries == 0);
  CHECK(p.config.maxConsecutiveFailures == 5);
  CHECK(p.config.insecure);
  CHECK(p.config.caBundle.empty());
  CHECK(p.config.verbose);
  CHECK(p.config.dryRun);
  CHECK(p.config.overwrite);
  CHECK(p.config.layer == "Other");
  CHECK(p.config.wmsUrl == "http://127.0.0.1:8080/wms");
}

TEST_CASE("the --ca-bundle flag takes an existing file", "[cli]") {
  const TempDir dir;
  const std::string bundle = createFile(dir.path / "roots.pem");
  std::vector<const char*> argv = kExample;
  argv.push_back("--ca-bundle");
  argv.push_back(bundle.c_str());
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.caBundle == bundle);
  CHECK_FALSE(p.config.insecure);
}

TEST_CASE("a missing CA bundle is a usage error", "[cli]") {
  const TempDir dir;
  const std::string missing = (dir.path / "missing.pem").string();
  std::vector<const char*> argv = kExample;
  argv.push_back("--ca-bundle");
  argv.push_back(missing.c_str());
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--ca-bundle"));
  CHECK_THAT(p.err, ContainsSubstring(missing));
}

TEST_CASE("the --insecure and --ca-bundle flags together are a usage error", "[cli]") {
  const TempDir dir;
  const std::string bundle = createFile(dir.path / "roots.pem");
  std::vector<const char*> argv = kExample;
  for (const char* a : {"--insecure", "--ca-bundle", bundle.c_str()}) {
    argv.push_back(a);
  }
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--ca-bundle excludes --insecure"));
}

TEST_CASE("a value that does not convert is a usage error", "[cli]") {
  // A plain whole number passes the CLI's check, but this one does not fit an int, so CLI11's conversion fails.
  const Parsed p = parse(exampleWith("--threads", "99999999999"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("Could not convert: --threads"));
}

TEST_CASE("integer flags take only plain whole numbers", "[cli]") {
  // CLI11 alone would read these as 64, 16, 1000, 2048, 1 and 0.
  const auto [flag, value] = GENERATE(table<const char*, const char*>({{"--width", "0100"},
                                                                       {"--width", "0x10"},
                                                                       {"--width", "1_000"},
                                                                       {"--width", "20,48"},
                                                                       {"--width", "true"},
                                                                       {"--max-retries", ""}}));
  const Parsed p = parse(exampleWith(flag, value));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(std::string(flag) +
                                      ": must be a whole number without leading zeros, separators or prefixes"));
  CHECK_THAT(p.err, ContainsSubstring(std::string(", not '") + value + "'"));
}

TEST_CASE("each integer flag runs the whole-number check", "[cli]") {
  const std::string flag =
      GENERATE(as<std::string>{}, "--width", "--height", "--threads", "--max-retries", "--max-consecutive-failures");
  const Parsed p = parse(exampleWith(flag.c_str(), "0100"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err,
             ContainsSubstring(flag + ": must be a whole number without leading zeros, separators or prefixes"));
}

TEST_CASE("coordinates take only plain decimals", "[cli]") {
  // CLI11 alone would read "" as 0, "1,5" as 15 and "-34,18" as -3418, and accept a hex float, nan and inf.
  const std::string value = GENERATE(as<std::string>{}, "", "1,5", "-34,18", "0x1p3", "nan", "inf");
  const Parsed p = parse(exampleWith("--north", value.c_str()));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(
                        "--north: must be a decimal number such as -34.1845418 (use '.' as the decimal separator)"));
  CHECK_THAT(p.err, ContainsSubstring(", not '" + value + "'"));
}

TEST_CASE("each coordinate flag runs the decimal check", "[cli]") {
  const std::string flag = GENERATE(as<std::string>{}, "--north", "--south", "--east", "--west");
  const Parsed p = parse(exampleWith(flag.c_str(), "1,5"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(
                        flag + ": must be a decimal number such as -34.1845418 (use '.' as the decimal separator)"));
}

TEST_CASE("coordinates may have an exponent or a bare leading or trailing point", "[cli]") {
  const auto [value, expected] =
      GENERATE(table<const char*, double>({{"1e-3", 0.001}, {".5", 0.5}, {"-34.", -34.0}}));
  const Parsed p = parse(exampleWith("--north", value));
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK_THAT(p.config.box.north, WithinAbs(expected, 1e-12));
}

TEST_CASE("the echoed value is quoted, with bytes outside printable ASCII escaped", "[cli]") {
  const auto [flag, value, shown] = GENERATE(table<const char*, const char*, const char*>({
      {"--width", "2048\r", "2048\\r"},  // a carriage return left by a Windows file
      {"--north", "-34.18 ", "-34.18 "},   // a trailing space stays visible inside the quotes
      {"--north", "\xE2\x88\x92" "34.18", "\\xE2\\x88\\x9234.18"},  // a Unicode minus sign
      {"--width", "1\t2", "1\\t2"},
      {"--width", "1\n2", "1\\n2"},
      {"--width", "1\\2", "1\\\\2"},
      {"--width", "1'2", "1\\'2"}}));
  const Parsed p = parse(exampleWith(flag, value));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(std::string(", not '") + shown + "'\n"));  // the end of the message
  CHECK(p.err.find('\r') == std::string::npos);
}

TEST_CASE("a value that looks like a flag is a usage error", "[cli]") {
  // Unchecked, "--output --dry-run" would take "--dry-run" as the path and start a real run.
  const auto [flag, value] =
      GENERATE(table<const char*, const char*>({{"--output", "--dry-run"}, {"--layer", "--verbose"}}));
  const Parsed p = parse(exampleWith(flag, value));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(std::string(flag) + ": looks like a flag; is the value missing?"));
}

TEST_CASE("each string option refuses a value that looks like a flag", "[cli]") {
  // For --ca-bundle this also shows that the check runs before the existing-file check.
  const std::string flag = GENERATE(as<std::string>{}, "--output", "--layer", "--wms-url", "--ca-bundle");
  const Parsed p = parse(exampleWith(flag.c_str(), "--x"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(flag + ": looks like a flag; is the value missing?"));
}

TEST_CASE("an unknown flag or a positional argument is a usage error", "[cli]") {
  const std::string extra = GENERATE(as<std::string>{}, "--bogus", "extra");
  std::vector<const char*> argv = kExample;
  argv.push_back(extra.c_str());
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("not expected: " + extra));
}

TEST_CASE("a value option given twice is a usage error", "[cli]") {
  std::vector<const char*> argv = kExample;
  for (const char* a : {"--threads", "3", "--threads", "4"}) {
    argv.push_back(a);
  }
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--threads: At most 1 required"));
}

TEST_CASE("values that parse but are invalid are caught by validate", "[cli]") {
  std::vector<const char*> argv = kExample;
  argv.push_back("--threads");
  argv.push_back("0");
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.validate() == "threads must be at least 1");
}
```

- [ ] **Step 2: Add the object library and the test to CMake**

Insert this block after the `target_compile_definitions(demdl ...)` line and before `if(DL_ELEVATION_TESTS)`:

```cmake
# Cli.cpp is compiled once and shared by the executable and the tests; only it includes CLI11.
add_library(demdl_cli OBJECT src/app/Cli.cpp)
target_link_libraries(demdl_cli PUBLIC demdl PRIVATE CLI11::CLI11)
target_compile_options(demdl_cli PRIVATE -Wall -Wextra)
```

Then update the test target:

```cmake
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp
    tests/test_DemRaster.cpp
    tests/test_StopToken.cpp
    tests/test_Downloader.cpp
    tests/test_ElevationClient.cpp
    tests/test_ConsoleReporter.cpp
    tests/test_Cli.cpp)
  target_link_libraries(demdl_tests PRIVATE demdl demdl_cli Catch2::Catch2WithMain)
```

Insert right after `cmake_minimum_required`:

```cmake
# CMake 3.29+ de-duplicates static libraries on the link line (CMP0156); without it, naming demdl
# directly and again through demdl_cli's PUBLIC link makes the Xcode 15 linker warn about a duplicate.
if(POLICY CMP0156)
  cmake_policy(SET CMP0156 NEW)
endif()
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/app/Cli.cpp`.

- [ ] **Step 4: Write `src/app/Cli.h`**

```cpp
#pragma once

#include <iosfwd>

#include "demdl/Config.h"

namespace demdl::cli {

struct [[nodiscard]] ParseOutcome {
  bool exitNow = false;  // true after --help, --version or a usage error
  int exitCode = 0;      // 0 for help and version, 1 for a usage error
};

// Fills `config` from argv with CLI11. Help, version and error text go to the given streams.
// Does not call Config::validate(); the caller does. After a usage error `config` may be partially
// filled and must not be used.
[[nodiscard]] ParseOutcome parseArgs(int argc, const char* const* argv, Config& config, std::ostream& out,
                                     std::ostream& err);

}  // namespace demdl::cli
```

- [ ] **Step 5: Write `src/app/Cli.cpp`**

```cpp
#include "app/Cli.h"

#include <CLI/CLI.hpp>

#include <cstddef>
#include <ostream>
#include <string>

namespace demdl::cli {

namespace {

// The character tests are ASCII only, so the result does not depend on the locale.
bool isDigit(char c) { return c >= '0' && c <= '9'; }

// Moves i past a run of digits and returns how many there were.
std::size_t skipDigits(const std::string& text, std::size_t& i) {
  const std::size_t start = i;
  while (i < text.size() && isDigit(text[i])) ++i;
  return i - start;
}

// An optional sign, digits with an optional fraction or a fraction alone, then an optional exponent:
// 12, 12., .5, -34.1845418, 1e-3.
bool isPlainDecimal(const std::string& text) {
  std::size_t i = 0;
  if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
  std::size_t digits = skipDigits(text, i);
  if (i < text.size() && text[i] == '.') {
    ++i;
    digits += skipDigits(text, i);
  }
  if (digits == 0) return false;
  if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
    ++i;
    if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
    if (skipDigits(text, i) == 0) return false;
  }
  return i == text.size();
}

// An optional '-', then digits without a leading zero; "0" itself is allowed.
bool isPlainInteger(const std::string& text) {
  if (text == "0") return true;
  const std::size_t first = text.starts_with('-') ? 1 : 0;
  if (first == text.size() || text[first] == '0') return false;
  for (std::size_t i = first; i < text.size(); ++i) {
    if (!isDigit(text[i])) return false;
  }
  return true;
}

// `text` in single quotes, readable whatever it holds: \r, \n and \t become those escapes, a backslash
// becomes \\, a single quote \', and any other byte outside printable ASCII (0x20..0x7E) \xHH.
std::string quoted(const std::string& text) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  std::string out = "'";
  for (const char c : text) {
    const auto byte = static_cast<unsigned char>(c);
    switch (c) {
      case '\r': out += "\\r"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      case '\\': out += "\\\\"; break;
      case '\'': out += "\\'"; break;
      default:
        if (byte >= 0x20 && byte <= 0x7E) {
          out += c;
        } else {
          out += "\\x";
          out += kHex[byte >> 4];
          out += kHex[byte & 0x0F];
        }
    }
  }
  return out + "'";
}

// The checks below return an empty string for an acceptable value, otherwise the problem. The numeric ones end
// with the value quoted, so that stray characters such as a trailing space or a carriage return show.

std::string checkDecimal(const std::string& text) {
  if (isPlainDecimal(text)) return {};
  return "must be a decimal number such as -34.1845418 (use '.' as the decimal separator), not " + quoted(text);
}

std::string checkWholeNumber(const std::string& text) {
  if (isPlainInteger(text)) return {};
  return "must be a whole number without leading zeros, separators or prefixes, not " + quoted(text);
}

std::string checkNotFlagLike(const std::string& text) {
  if (!text.starts_with("--")) return {};
  return "looks like a flag; is the value missing?";
}

}  // namespace

ParseOutcome parseArgs(int argc, const char* const* argv, Config& config, std::ostream& out, std::ostream& err) {
  CLI::App app{"Download a digital elevation model from a WMS server one pixel at a time into a GeoTIFF",
               "dl_elevation"};
  app.set_version_flag("--version", std::string(DL_ELEVATION_VERSION));
  app.failure_message(
      [](const CLI::App* self, const CLI::Error& e) { return "error: " + CLI::FailureMessage::simple(self, e); });

  // CLI11 2.7.2 runs these checks on the raw text before converting it. Without them it would read "" as 0,
  // drop digit separators ('_', '\'' and the locale's thousands separator, ',' by default, so "1,5" is 15),
  // take 0x, 0b, 0o, leading-zero octal and "true" as integers, and take hex floats, nan and inf as decimals.
  const CLI::Validator decimal(checkDecimal, "");
  const CLI::Validator wholeNumber(checkWholeNumber, "");
  // A forgotten value would otherwise swallow the next flag: "--output --dry-run" would write to "--dry-run".
  const CLI::Validator notFlagLike(checkNotFlagLike, "");

  // Parsed into a local so that config.output stays untouched when parsing fails.
  std::string output;
  app.add_option("--north", config.box.north, "northern edge, WGS84 decimal degrees")->required()->check(decimal);
  app.add_option("--south", config.box.south, "southern edge, WGS84 decimal degrees")->required()->check(decimal);
  app.add_option("--east", config.box.east, "eastern edge, WGS84 decimal degrees")->required()->check(decimal);
  app.add_option("--west", config.box.west, "western edge, WGS84 decimal degrees")->required()->check(decimal);
  app.add_option("--width", config.width, "grid width in pixels")->required()->check(wholeNumber);
  app.add_option("--height", config.height, "grid height in pixels")->required()->check(wholeNumber);
  app.add_option("-o,--output", output, "GeoTIFF to create or resume")->required()->check(notFlagLike);
  app.add_option("--wms-url", config.wmsUrl, "WMS endpoint: scheme, host and path")
      ->capture_default_str()
      ->check(notFlagLike);
  app.add_option("--layer", config.layer, "WMS layer name, taken unencoded; used for LAYERS and QUERY_LAYERS")
      ->capture_default_str()
      ->check(notFlagLike);
  // The ranges below are stated for the user; Config::validate alone enforces them.
  app.add_option("--threads", config.threads, "worker threads (1..64), one keep-alive connection each")
      ->capture_default_str()
      ->check(wholeNumber);
  app.add_option("--max-retries", config.maxRetries, "retries per pixel after the first attempt (0..100)")
      ->capture_default_str()
      ->check(wholeNumber);
  app.add_option("--max-consecutive-failures", config.maxConsecutiveFailures,
                 "stop after this many pixels fail in a row (at least 1)")
      ->capture_default_str()
      ->check(wholeNumber);
  CLI::Option* const caBundleOption =
      app.add_option("--ca-bundle", config.caBundle,
                     "PEM bundle for certificate verification (file must exist; excludes --insecure)")
          ->check(notFlagLike)
          ->check(CLI::ExistingFile);
  app.add_flag("--insecure", config.insecure, "disable certificate verification (excludes --ca-bundle)")
      ->excludes(caBundleOption);
  app.add_flag("--preview", config.preview, "open the OpenGL preview window (fails if built without preview support)");
  app.add_flag("--overwrite", config.overwrite, "ignore an existing output and start fresh");
  app.add_flag("--dry-run", config.dryRun, "validate and report the plan; make no requests and write nothing");
  app.add_flag("--verbose", config.verbose, "log every pixel, not just every row");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    const int code = app.exit(e, out, err);
    return {true, code == 0 ? 0 : 1};
  }
  config.output = output;
  return {false, 0};
}

}  // namespace demdl::cli
```

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[cli]"`
Expected: `All tests passed (N assertions in 22 test cases)`.

Run: `ctest --test-dir build --output-on-failure`
Expected: `100% tests passed` across every tag so far.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/app/Cli.h src/app/Cli.cpp tests/test_Cli.cpp
git commit -m "feat: CLI11 argument parsing into Config" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 12: main and the executable

Implements spec sections 9 and 10 (wiring, signals, dry run, exit codes). The preview hooks are included behind `#ifdef DL_ELEVATION_HAVE_PREVIEW` now so Task 13 only adds a target. Deviation 5 applies.

**Files:**
- Create: `src/app/main.cpp`
- Modify: `CMakeLists.txt` (new executable target)

- [ ] **Step 1: Add the executable to CMake**

Insert after the `demdl_cli` block and before `if(DL_ELEVATION_TESTS)`:

```cmake
add_executable(dl_elevation src/app/main.cpp)
target_link_libraries(dl_elevation PRIVATE demdl demdl_cli)
target_compile_options(dl_elevation PRIVATE -Wall -Wextra)
```

- [ ] **Step 2: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/app/main.cpp`.

- [ ] **Step 3: Write `src/app/main.cpp`**

```cpp
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "app/Cli.h"
#include "demdl/Config.h"
#include "demdl/ConsoleReporter.h"
#include "demdl/DemRaster.h"
#include "demdl/Downloader.h"
#include "demdl/ElevationClient.h"
#include "demdl/ProgressSink.h"
#include "demdl/RetryPolicy.h"
#include "demdl/StopToken.h"
#include "demdl/WmsQuery.h"
#ifdef DL_ELEVATION_HAVE_PREVIEW
#include "preview/PreviewWindow.h"
#endif

namespace {

demdl::StopToken* g_stop = nullptr;
std::atomic<int> g_signalCount{0};

// Only lock-free atomics are touched here; nothing is printed from a signal handler.
void onSignal(int) {
  if (g_signalCount.fetch_add(1) >= 1) std::_Exit(130);
  if (g_stop != nullptr) g_stop->request(demdl::StopReason::Signal);
}

void printPlan(const demdl::Config& cfg, const demdl::DemRaster& raster, bool created) {
  using demdl::formatCoordinate;
  std::cout << "dry run\n"
            << "  box: north " << formatCoordinate(cfg.box.north) << ", south " << formatCoordinate(cfg.box.south)
            << ", east " << formatCoordinate(cfg.box.east) << ", west " << formatCoordinate(cfg.box.west) << "\n"
            << "  grid: " << cfg.width << " x " << cfg.height << "\n"
            << "  output: " << cfg.output.string() << (created ? " (will be created)" : " (exists, will be resumed)")
            << "\n"
            << "  wms: " << cfg.wmsUrl << "\n"
            << "  layer: " << cfg.layer << "\n"
            << "  threads: " << cfg.threads << ", max retries: " << cfg.maxRetries
            << ", max consecutive failures: " << cfg.maxConsecutiveFailures << "\n";
  const std::vector<int> rows = raster.rowsWithWork();
  std::cout << "  rows remaining: " << rows.size() << " of " << raster.height() << "\n"
            << "  pixels remaining: " << raster.countNaN() << " of " << raster.pixelCount() << "\n";
  if (!rows.empty()) {
    const int row = rows.front();
    const int x = raster.todoInRow(row).front();
    std::cout << "  first request: " << demdl::parseWmsUrl(cfg.wmsUrl)->origin << demdl::featureInfoUrl(cfg, x, row)
              << "\n";
  }
}

int exitCodeFor(const demdl::RunSummary& s) {
  if (s.remainingNaN == 0) return 0;
  if (s.stopReason != demdl::StopReason::None) return 2;
  return 3;
}

}  // namespace

int main(int argc, char** argv) {
  using namespace std::chrono_literals;
  using namespace demdl;

  Config cfg;
  const cli::ParseOutcome parsed = cli::parseArgs(argc, argv, cfg, std::cout, std::cerr);
  if (parsed.exitNow) return parsed.exitCode;
  if (const std::string err = cfg.validate(); !err.empty()) {
    std::cerr << "error: " << err << "\n";
    return 1;
  }
#ifndef DL_ELEVATION_HAVE_PREVIEW
  if (cfg.preview) {
    std::cerr << "error: this build has no preview support; configure with -DDL_ELEVATION_PREVIEW=ON\n";
    return 1;
  }
#endif

  std::optional<DemRaster> raster;
  bool created = false;
  try {
    if (std::filesystem::exists(cfg.output) && !cfg.overwrite) {
      raster = DemRaster::load(cfg.output);
      std::string why;
      if (!raster->matches(cfg, why)) {
        std::cerr << "error: " << cfg.output.string() << " was created for different parameters:\n"
                  << why << "use --overwrite to start fresh or choose another --output\n";
        return 1;
      }
      if (const std::string warning = raster->wmsUrlWarning(cfg); !warning.empty()) {
        std::cerr << "warning: " << warning << "\n";
      }
    } else {
      raster = DemRaster::create(cfg.width, cfg.height, cfg.box, RasterMetadata{cfg.layer, cfg.wmsUrl});
      created = true;
    }
    if (cfg.dryRun) {
      printPlan(cfg, *raster, created);
      return 0;
    }
    if (created) raster->save(cfg.output);  // validates the path early; the file is openable while it fills
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return 1;
  }

  StopToken stop;
  g_stop = &stop;
  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  std::signal(SIGPIPE, SIG_IGN);

  RetryPolicy policy;
  policy.maxRetries = cfg.maxRetries;
  const FetcherFactory factory = [&cfg, policy](int) -> std::unique_ptr<PixelFetcher> {
    return std::make_unique<ElevationClient>(cfg, policy);
  };

  std::unique_ptr<ProgressSink> sink;
#ifdef DL_ELEVATION_HAVE_PREVIEW
  PreviewWindow* preview = nullptr;
  if (cfg.preview) {
    try {
      auto window = std::make_unique<PreviewWindow>(*raster, stop);
      preview = window.get();
      sink = std::move(window);
    } catch (const std::exception& e) {
      std::cerr << "error: " << e.what() << "\n";
      return 1;
    }
  }
#endif
  if (!sink) sink = std::make_unique<ConsoleReporter>(std::cout, cfg.verbose);

  Downloader downloader(*raster, cfg.output, factory, *sink, stop,
                        DownloaderOptions{cfg.threads, cfg.maxConsecutiveFailures});
  std::future<RunSummary> result = std::async(std::launch::async, [&downloader] { return downloader.run(); });

  bool announced = false;
  while (result.wait_for(100ms) != std::future_status::ready) {
#ifdef DL_ELEVATION_HAVE_PREVIEW
    if (preview != nullptr) preview->pump();
#endif
    if (stop.stop.load() && !announced) {
      announced = true;
      std::cout << "stopping: " << describe(stop.reason()) << "; waiting for " << downloader.inFlight()
                << " in-flight requests\n";
    }
  }
  const RunSummary summary = result.get();
  g_stop = nullptr;
  return exitCodeFor(summary);
}
```

- [ ] **Step 4: Build**

Run: `cmake --build build`
Expected: builds `dl_elevation` with no errors.

- [ ] **Step 5: Smoke-test the dry run (no network)**

Run:

```bash
./build/dl_elevation --north -34.1845418 --south -34.2163447 --west 18.4185791 --east 18.4625244 --width 16 --height 8 --output "${TMPDIR:-/tmp}/demdl-smoke.tif" --dry-run; echo "exit $?"
```

Expected output, with your temp directory in place of `/tmp`:

```
dry run
  box: north -34.184541800, south -34.216344700, east 18.462524400, west 18.418579100
  grid: 16 x 8
  output: /tmp/demdl-smoke.tif (will be created)
  wms: https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets
  layer: Elevation_CCT_GLM2019_5m_DEM
  threads: 8, max retries: 8, max consecutive failures: 100
  rows remaining: 8 of 8
  pixels remaining: 128 of 128
  first request: https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetFeatureInfo&BBOX=-34.216344700%2C18.418579100%2C-34.184541800%2C18.462524400&CRS=EPSG%3A4326&WIDTH=16&HEIGHT=8&LAYERS=Elevation_CCT_GLM2019_5m_DEM&STYLES=&FORMAT=image%2Fpng&QUERY_LAYERS=Elevation_CCT_GLM2019_5m_DEM&INFO_FORMAT=text%2Fplain&I=0&J=0
exit 0
```

Run: `ls "${TMPDIR:-/tmp}/demdl-smoke.tif"`
Expected: `No such file or directory`. A dry run writes nothing.

- [ ] **Step 6: Smoke-test the error paths**

Run: `./build/dl_elevation --north -34.18 --south -34.21 --west 18.41 --east 18.46 --width 16 --height 8 --output x.tif --threads 0; echo "exit $?"`
Expected: `error: threads must be at least 1` on stderr, then `exit 1`.

Run: `./build/dl_elevation --help >/dev/null; echo "exit $?"`
Expected: `exit 0`.

Run: `./build/dl_elevation --north -34.18 --south -34.21 --west 18.41 --east 18.46 --width 16 --height 8 --output x.tif --preview --dry-run; echo "exit $?"`
Expected on a build without the preview target (which is every build until Task 13): `error: this build has no preview support; configure with -DDL_ELEVATION_PREVIEW=ON` and `exit 1`.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/app/main.cpp
git commit -m "feat: dl_elevation executable with dry run, signal handling and exit codes" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 13: Preview window

Implements the `PreviewWindow` sink from spec section 8 and the `DL_ELEVATION_PREVIEW` option from section 12. Deviation 7 applies. There is no automated test; the compile and the `--preview --dry-run` probe are the checks, and a live run is optional and gated on the owner.

**Files:**
- Create: `src/preview/Shader.h`
- Create: `src/preview/PreviewWindow.h`
- Create: `src/preview/PreviewWindow.cpp`
- Modify: `CMakeLists.txt` (preview target and define)

- [ ] **Step 1: Add the preview target to CMake**

Insert after the `dl_elevation` block and before `if(DL_ELEVATION_TESTS)`:

```cmake
if(DL_ELEVATION_PREVIEW)
  find_package(PkgConfig REQUIRED)
  pkg_search_module(glfw REQUIRED IMPORTED_TARGET glfw3)
  add_library(demdl_preview STATIC src/preview/PreviewWindow.cpp)
  target_link_libraries(demdl_preview PUBLIC demdl PkgConfig::glfw)
  target_compile_definitions(demdl_preview PRIVATE GL_SILENCE_DEPRECATION)
  target_compile_options(demdl_preview PRIVATE -Wall -Wextra)
  if(APPLE)
    target_link_libraries(demdl_preview PUBLIC
      "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
  else()
    find_package(OpenGL REQUIRED)
    pkg_search_module(glew REQUIRED IMPORTED_TARGET glew)
    target_link_libraries(demdl_preview PUBLIC OpenGL::GL PkgConfig::glew)
  endif()
  target_link_libraries(dl_elevation PRIVATE demdl_preview)
  target_compile_definitions(dl_elevation PRIVATE DL_ELEVATION_HAVE_PREVIEW)
endif()
```

- [ ] **Step 2: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/preview/PreviewWindow.cpp`.

- [ ] **Step 3: Write `src/preview/Shader.h`**

```cpp
#pragma once

// The including translation unit must include the OpenGL headers before this file.

#include <iostream>
#include <string>

namespace demdl {

// Compiles and links a vertex/fragment pair from source strings. Trimmed from the LearnOpenGL
// helper: no file loading, no GLM, only the one uniform setter the preview needs.
class Shader {
 public:
  Shader(const std::string& vertexSource, const std::string& fragmentSource) {
    const unsigned int vertex = compile(GL_VERTEX_SHADER, vertexSource, "vertex");
    const unsigned int fragment = compile(GL_FRAGMENT_SHADER, fragmentSource, "fragment");
    id_ = glCreateProgram();
    glAttachShader(id_, vertex);
    glAttachShader(id_, fragment);
    glLinkProgram(id_);
    int success = 0;
    glGetProgramiv(id_, GL_LINK_STATUS, &success);
    if (!success) {
      char log[1024];
      glGetProgramInfoLog(id_, sizeof log, nullptr, log);
      std::cerr << "shader link error: " << log << "\n";
    }
    glDeleteShader(vertex);
    glDeleteShader(fragment);
  }
  ~Shader() {
    if (id_ != 0) glDeleteProgram(id_);
  }
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  void use() const { glUseProgram(id_); }
  void setInt(const char* name, int value) const { glUniform1i(glGetUniformLocation(id_, name), value); }

 private:
  static unsigned int compile(unsigned int type, const std::string& source, const char* label) {
    const unsigned int shader = glCreateShader(type);
    const char* text = source.c_str();
    glShaderSource(shader, 1, &text, nullptr);
    glCompileShader(shader);
    int success = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &success);
    if (!success) {
      char log[1024];
      glGetShaderInfoLog(shader, sizeof log, nullptr, log);
      std::cerr << label << " shader compile error: " << log << "\n";
    }
    return shader;
  }

  unsigned int id_ = 0;
};

}  // namespace demdl
```

- [ ] **Step 4: Write `src/preview/PreviewWindow.h`**

```cpp
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "demdl/DemRaster.h"
#include "demdl/ProgressSink.h"
#include "demdl/StopToken.h"

struct GLFWwindow;

namespace demdl {

class Shader;

// A GLFW window showing the raster as a status map. Construct it and call pump() on the main
// thread; the sink methods may be called from any thread.
class PreviewWindow : public ProgressSink {
 public:
  // Throws std::runtime_error when GLFW cannot initialise or open a window.
  PreviewWindow(const DemRaster& raster, StopToken& stop);
  ~PreviewWindow() override;
  PreviewWindow(const PreviewWindow&) = delete;
  PreviewWindow& operator=(const PreviewWindow&) = delete;

  // Polls events, redraws every 500 ms, requests StopReason::WindowClosed when the window closes.
  void pump();

  void onRunStarted(const RunPlan& plan) override;
  void onPixelState(const PixelEvent& event) override;
  void onRowSaved(const RowReport& report) override;
  void onRunFinished(const RunSummary& summary) override;

 private:
  enum Cell : std::uint8_t { kTodo = 0, kNoData = 1, kQueued = 2, kInFlight = 3, kValue = 4, kFailed = 5 };

  void initGl();
  void redraw();
  std::size_t index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  }

  int width_;
  int height_;
  StopToken& stop_;

  std::mutex mutex_;
  std::vector<std::uint8_t> cells_;
  std::vector<float> values_;
  float minValue_;
  float maxValue_;
  double percent_ = 0.0;

  GLFWwindow* window_ = nullptr;
  unsigned int texture_ = 0;
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int ebo_ = 0;
  std::unique_ptr<Shader> shader_;
  std::vector<unsigned char> rgb_;
  std::chrono::steady_clock::time_point lastRedraw_;
};

}  // namespace demdl
```

- [ ] **Step 5: Write `src/preview/PreviewWindow.cpp`**

```cpp
#include "preview/PreviewWindow.h"

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#else
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>

#include "preview/Shader.h"

namespace demdl {

namespace {

const char* const kVertexShader = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;
out vec2 TexCoord;
void main() {
  gl_Position = vec4(aPos, 1.0);
  TexCoord = aTexCoord;
}
)glsl";

const char* const kFragmentShader = R"glsl(
#version 330 core
out vec4 FragColor;
in vec2 TexCoord;
uniform sampler2D texture1;
void main() {
  FragColor = texture(texture1, TexCoord);
}
)glsl";

// Maps t in [0, 1] (low to high) onto light blue, cyan, green, yellow, red. Stops short of pure
// blue (NoData) and never reaches magenta (Failed).
void valueColour(float t, unsigned char& r, unsigned char& g, unsigned char& b) {
  const double a = (1.0 - std::clamp(static_cast<double>(t), 0.0, 1.0)) * 3.5;
  const int x = static_cast<int>(std::floor(a));
  const auto y = static_cast<unsigned char>(std::floor(255.0 * (a - x)));
  switch (x) {
    case 0: r = 255; g = y; b = 0; break;
    case 1: r = static_cast<unsigned char>(255 - y); g = 255; b = 0; break;
    case 2: r = 0; g = 255; b = y; break;
    default: r = 0; g = static_cast<unsigned char>(255 - y); b = 255; break;
  }
}

void onFramebufferResized(GLFWwindow*, int width, int height) { glViewport(0, 0, width, height); }

}  // namespace

PreviewWindow::PreviewWindow(const DemRaster& raster, StopToken& stop)
    : width_(raster.width()),
      height_(raster.height()),
      stop_(stop),
      cells_(raster.pixelCount(), kTodo),
      values_(raster.pixelCount(), 0.0f),
      minValue_(std::numeric_limits<float>::infinity()),
      maxValue_(-std::numeric_limits<float>::infinity()),
      rgb_(raster.pixelCount() * 3, 0) {
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const float v = raster.at(x, y);
      const std::size_t i = index(x, y);
      if (std::isnan(v)) continue;
      if (v == DemRaster::kNoData) {
        cells_[i] = kNoData;
        continue;
      }
      cells_[i] = kValue;
      values_[i] = v;
      minValue_ = std::min(minValue_, v);
      maxValue_ = std::max(maxValue_, v);
    }
  }
  const std::uint64_t done = raster.pixelCount() - raster.countNaN();
  percent_ = 100.0 * static_cast<double>(done) / static_cast<double>(raster.pixelCount());
  initGl();
  lastRedraw_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);  // draw on the first pump
}

PreviewWindow::~PreviewWindow() {
  if (window_ != nullptr) {
    glfwMakeContextCurrent(window_);
    shader_.reset();
    glDeleteTextures(1, &texture_);
    glDeleteBuffers(1, &ebo_);
    glDeleteBuffers(1, &vbo_);
    glDeleteVertexArrays(1, &vao_);
    glfwDestroyWindow(window_);
  }
  glfwTerminate();
}

void PreviewWindow::initGl() {
  if (!glfwInit()) throw std::runtime_error("GLFW initialisation failed");
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  window_ = glfwCreateWindow(640, 480, "dl_elevation", nullptr, nullptr);
  if (window_ == nullptr) {
    glfwTerminate();
    throw std::runtime_error("GLFW could not create a window; is a display available?");
  }
  glfwMakeContextCurrent(window_);
  glfwSetFramebufferSizeCallback(window_, onFramebufferResized);
#ifndef __APPLE__
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) throw std::runtime_error("GLEW initialisation failed");
#endif

  shader_ = std::make_unique<Shader>(kVertexShader, kFragmentShader);

  const float vertices[] = {
      //  x      y     z     u     v
      1.0f,  1.0f, 0.0f, 1.0f, 0.0f,  // top right
      1.0f,  -1.0f, 0.0f, 1.0f, 1.0f,  // bottom right
      -1.0f, -1.0f, 0.0f, 0.0f, 1.0f,  // bottom left
      -1.0f, 1.0f,  0.0f, 0.0f, 0.0f,  // top left
  };
  const unsigned int indices[] = {0, 1, 3, 1, 2, 3};
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof vertices, vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof indices, indices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  // Rows are width*3 bytes, which need not be a multiple of the default 4-byte alignment.
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

void PreviewWindow::pump() {
  glfwPollEvents();
  if (glfwWindowShouldClose(window_)) stop_.request(StopReason::WindowClosed);
  const auto now = std::chrono::steady_clock::now();
  if (now - lastRedraw_ >= std::chrono::milliseconds(500)) {
    redraw();
    lastRedraw_ = now;
  }
}

void PreviewWindow::redraw() {
  char title[64];
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const float lo = minValue_;
    const float span = maxValue_ > lo ? maxValue_ - lo : 1.0f;
    for (std::size_t i = 0; i < cells_.size(); ++i) {
      unsigned char r = 0;
      unsigned char g = 0;
      unsigned char b = 0;
      switch (cells_[i]) {
        case kTodo: r = g = b = 40; break;
        case kNoData: b = 255; break;
        case kQueued: r = 255; break;
        case kInFlight: g = 255; break;
        case kFailed: r = 255; b = 255; break;
        case kValue: valueColour((values_[i] - lo) / span, r, g, b); break;
      }
      rgb_[3 * i] = r;
      rgb_[3 * i + 1] = g;
      rgb_[3 * i + 2] = b;
    }
    std::snprintf(title, sizeof title, "dl_elevation %.1f%%", percent_);
  }
  glfwSetWindowTitle(window_, title);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_.data());
  shader_->use();
  shader_->setInt("texture1", 0);
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
  glfwSwapBuffers(window_);
}

void PreviewWindow::onRunStarted(const RunPlan&) {}

void PreviewWindow::onPixelState(const PixelEvent& e) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t i = index(e.x, e.y);
  switch (e.state) {
    case PixelState::Queued: cells_[i] = kQueued; break;
    case PixelState::InFlight: cells_[i] = kInFlight; break;
    case PixelState::NoData: cells_[i] = kNoData; break;
    case PixelState::Failed: cells_[i] = kFailed; break;
    case PixelState::Value:
      cells_[i] = kValue;
      values_[i] = e.value;
      minValue_ = std::min(minValue_, e.value);
      maxValue_ = std::max(maxValue_, e.value);
      break;
  }
}

void PreviewWindow::onRowSaved(const RowReport& r) {
  std::lock_guard<std::mutex> lock(mutex_);
  percent_ = r.pixelsTotal > 0 ? 100.0 * static_cast<double>(r.pixelsDoneTotal) / static_cast<double>(r.pixelsTotal)
                               : 100.0;
}

void PreviewWindow::onRunFinished(const RunSummary&) {}

}  // namespace demdl
```

- [ ] **Step 6: Build and probe**

Run: `cmake --build build`
Expected: builds `demdl_preview` and relinks `dl_elevation` with no errors.

Run: `./build/dl_elevation --north -34.18 --south -34.21 --west 18.41 --east 18.46 --width 16 --height 8 --output x.tif --preview --dry-run; echo "exit $?"`
Expected: the dry-run plan and `exit 0`. The "no preview support" error from Task 12 is gone, which proves the define reached `main.cpp`. No window opens because a dry run returns before the sink is created.

Run: `ctest --test-dir build --output-on-failure`
Expected: `100% tests passed`.

- [ ] **Step 7: Optional live check, only with the owner's explicit go-ahead**

This makes 128 real requests to the Cape Town server. Do not run it on your own initiative.

```bash
./build/dl_elevation --north -34.1845418 --south -34.2163447 --west 18.4185791 --east 18.4625244 --width 16 --height 8 --output "${TMPDIR:-/tmp}/demdl-preview.tif" --preview --threads 2
```

Expected: a 640x480 window titled `dl_elevation 0.0%`; pixels start dark grey, turn red when queued, green in flight, then a red-to-light-blue value colour or blue for NoData, rows filling top to bottom; the title percentage rises after each row; the process exits 0 when the last row is saved and the window closes. Closing the window mid-run prints `stopping: preview window closed; waiting for N in-flight requests`, saves, exits 2, and running the same command again resumes from the remaining NaN pixels.

- [ ] **Step 8: Commit**

```bash
git add CMakeLists.txt src/preview/Shader.h src/preview/PreviewWindow.h src/preview/PreviewWindow.cpp
git commit -m "feat: optional GLFW preview window behind DL_ELEVATION_PREVIEW" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 14: Remove the old sources and rewrite CLAUDE.md

Implements the deletions in spec section 4.2 and the documentation item in section 12.

**Files:**
- Delete: `main.cpp`, `opengl.cmake.in`, `include/` (nineteen headers)
- Modify: `CLAUDE.md` (full replacement)

- [ ] **Step 1: Remove the files**

```bash
git rm -q -r include opengl.cmake.in main.cpp
```

Run: `git status --short | head`
Expected: `D` entries for `main.cpp`, `opengl.cmake.in` and every file under `include/`, nothing else.

- [ ] **Step 2: Confirm nothing referenced them**

Run: `grep -rn "opengl.cmake.in\|include/shader\|include/" CMakeLists.txt src tests || echo "no references"`
Expected: `no references`.

Run: `cmake --build build && ctest --test-dir build --output-on-failure`
Expected: builds and `100% tests passed`.

- [ ] **Step 3: Replace `CLAUDE.md`**

```markdown
# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`dl_elevation` scrapes a Digital Elevation Model (DEM) from a WMS server one pixel at a time, one `GetFeatureInfo` request per pixel, into a single-band float32 GeoTIFF, and resumes safely across runs. The default source is the City of Cape Town's public server. macOS is the primary platform; the headless build also targets Linux.

Design and plan: `docs/superpowers/specs/2026-10-04-cli-refactor-design.md` and `docs/superpowers/plans/2026-10-04-cli-refactor.md`.

## Build, test, run

CMake 3.27+ with Ninja. The first configure fetches cpp-httplib, ctpl, CLI11 and Catch2 and compiles Catch2, so allow a few minutes.

```bash
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build
```

```bash
cmake --build build
```

```bash
ctest --test-dir build --output-on-failure
```

One unit's tests by Catch2 tag, for example `./build/demdl_tests "[downloader]"`. One case by name, for example `./build/demdl_tests "a stop request saves the partial row and goes no further"`.

Options: `-DDL_ELEVATION_PREVIEW=OFF` drops the GLFW window and every OpenGL dependency (default ON on macOS, OFF elsewhere). `-DDL_ELEVATION_TESTS=OFF` skips Catch2.

Homebrew packages: `pkg-config libtiff libgeotiff openssl@3`, plus `glfw` for the preview (Linux also needs `glew`). Homebrew's `cpp-httplib` is used when installed; otherwise FetchContent downloads the pinned version.

Running the tool makes real requests to a public server. Check a command line with `--dry-run`, and never start a live download just to see whether the build works:

```bash
./build/dl_elevation --north -34.1845418 --south -34.2163447 --west 18.4185791 --east 18.4625244 --width 2048 --height 1792 --output simonstown.tif --dry-run
```

`./build/dl_elevation --help` lists every flag. Headless is the default; `--preview` opens the window.

## Layout

- `src/demdl/` is the static library holding all logic: `BoundingBox`; `Config` (every option plus `validate()`); `WmsQuery` (URL building and body parsing, pure functions); `RetryPolicy`; `DemRaster` (float32 grid, GeoTIFF I/O, resume checks); `PixelFetcher` (one-method interface); `ElevationClient` (httplib); `Downloader` (row loop, pool, checkpoints, stop handling); `ProgressSink` with `ConsoleReporter`; `StopToken`.
- `src/app/` is the executable: `Cli.cpp` turns argv into a `Config` with CLI11; `main.cpp` wires the components, handles signals and maps results to exit codes.
- `src/preview/` is the optional GLFW/OpenGL sink.
- `tests/` has one Catch2 file per unit plus the doubles `FakeFetcher.h` and `RecordingSink.h`, `TempDir.h` and `ExampleConfig.h`. Nothing touches the network except an in-process `httplib::Server` on loopback.

## Behaviours to keep in mind

- **The raster is the resume state.** NaN means not fetched, or failed last time and due for retry. `-32768`, the declared NoData, means the server has no elevation there and is final. On start an existing output must match width, height, bounding box (within 1e-9 degrees) and layer, otherwise exit 1 with the differences; `--overwrite` starts fresh. Files from the old RGB16 format are rejected.
- **Checkpoints are atomic.** After every row with work the whole raster is written to `<output>.tmp` and renamed over the output.
- **Stopping is one flag.** SIGINT, SIGTERM, closing the preview, a checkpoint write failure, or `--max-consecutive-failures` pixels failing in a row all set the `StopToken`. Queued pixels are abandoned, in-flight ones finish, the partial row is saved. A second Ctrl-C exits immediately.
- **Exit codes:** 0 complete, 1 usage or configuration error, 2 stopped early and resumable, 3 finished with failed pixels and resumable, 130 second interrupt.
- **Threading:** worker threads write distinct raster cells and own one `ElevationClient` each; sinks guard their own state; nothing prints from a signal handler.
- The old single-file `main.cpp` and the `include/` LearnOpenGL boilerplate are gone. Do not resurrect them.

## CMake notes

- Link `httplib::httplib`, never bare `httplib`. Only the namespaced target exists when Homebrew's package is found, and the bare name silently drops `CPPHTTPLIB_OPENSSL_SUPPORT`.
- `CMAKE_FIND_FRAMEWORK` is `LAST` because `/Library/Frameworks/Mono.framework/Headers` ships a libtiff 4.0.9 `tiff.h` that otherwise wins over Homebrew's.
- Apple clang searches `/usr/local/include` ahead of every `-isystem` directory, so a Homebrew header shadows a same-named header from a FetchContent target. Harmless today, worth knowing when bumping versions.
- Dependencies are pinned: cpp-httplib v0.46.1 (matches the Homebrew package), ctpl 437e135, CLI11 v2.7.2, Catch2 v3.16.0. Bump cpp-httplib together with the Homebrew package.

## Repository hygiene

`*.tif` and `*.tif.tmp` are ignored. The 20 to 40 MB files in the repository root are download outputs, not sources. Never stage with `git add -A`.
```

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "chore: remove the single-file program and LearnOpenGL boilerplate; rewrite CLAUDE.md" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

### Task 15: Final verification

Proves the result on every configuration the spec promises. Nothing is committed unless a fix is needed.

- [ ] **Step 1: Clean build on the default dependency path**

```bash
rm -rf build && cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build && cmake --build build && ctest --test-dir build --output-on-failure
```

Expected: configure with no warnings, build with no errors, `100% tests passed`.

- [ ] **Step 2: Clean build with cpp-httplib forced through FetchContent**

```bash
rm -rf build-fetch && cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -DFETCHCONTENT_TRY_FIND_PACKAGE_MODE=NEVER -S . -B build-fetch && cmake --build build-fetch && ctest --test-dir build-fetch --output-on-failure
```

Expected: same result. This is the path a machine without Homebrew's `cpp-httplib` takes.

- [ ] **Step 3: Headless build without any OpenGL dependency**

```bash
rm -rf build-headless && cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -DDL_ELEVATION_PREVIEW=OFF -S . -B build-headless && cmake --build build-headless
```

Expected: configures and builds without touching pkg-config for glfw. Then:

Run: `./build-headless/dl_elevation --north -34.18 --south -34.21 --west 18.41 --east 18.46 --width 16 --height 8 --output x.tif --preview --dry-run; echo "exit $?"`
Expected: `error: this build has no preview support; configure with -DDL_ELEVATION_PREVIEW=ON` and `exit 1`.

- [ ] **Step 4: Dry run and version on the default build**

Run: `./build/dl_elevation --version`
Expected: `0.2.0`.

Run: `./build/dl_elevation --north -34.1845418 --south -34.2163447 --west 18.4185791 --east 18.4625244 --width 16 --height 8 --output "${TMPDIR:-/tmp}/demdl-smoke.tif" --dry-run; echo "exit $?"`
Expected: the plan from Task 12 Step 5 and `exit 0`.

- [ ] **Step 5: Repository state**

Run: `git status --short`
Expected: only the pre-existing untracked `.idea/`, `.vscode/` and the root `.tif` files, which `.gitignore` now hides, so ideally only `.idea/` and `.vscode/`. No modified tracked files.

Run: `rm -rf build-fetch build-headless`
Expected: the scratch build directories are gone; `build/` stays.

- [ ] **Step 6: Report**

Summarise for the owner: commit list since `refactor/cli` branched, test case count from `ctest`, the three configurations verified, and that no live request was made unless Task 13 Step 7 was explicitly approved and run.


---

## Execution log (controller notes)

Executed with superpowers:subagent-driven-development on branch `refactor/cli`. Each task: implementer, spec-compliance review, code-quality review; every accepted fix is mirrored into this plan's code blocks and into the spec, so the plan blocks stay byte-identical to the files.

| Task | Status | Commits |
|---|---|---|
| 1 CMake skeleton, BoundingBox | complete, approved | `fdf24b4`, `13e69a2`, `3fd84d0` |
| 2 Config, WMS URL parsing | complete, approved | `2b83855`, `31db417`, `b49b7e5`, plus `179fb60` (retries cap) |
| 3 WmsQuery | complete, approved | `207ee12`, `2b5a53d`, `c2a2284`, `acd3ab3` |
| 4 RetryPolicy | complete, approved | `d856fc0`, `179fb60` |
| 5 DemRaster in memory | complete, approved | `fa5b5ad` |
| 6 DemRaster GeoTIFF I/O | complete, approved | `6aa85d5`, `99318f4`, `b671d28`, `e482239`, `eb31e35` |
| 7 Interfaces, StopToken, doubles | complete, approved | `8e1fd6c`, `6070aa2` |
| 8 Downloader | complete, approved | `6ed7659`, `e0a441b`, `830662e` |
| 9 ElevationClient | complete, approved | `a1781fc`, `fd34feb`, `23bc748` |
| 10 ConsoleReporter | complete, approved | `6848fbf`, `7a215d6`, `b9c52c4`, `306a2fe` |

Decisions taken during execution (all reflected in the spec): WMS URL grammar restricted to `http(s)://host[:port][/path]` with a hostname or bracketed IPv6 literal, port 1..65535, no userinfo, query, fragment, whitespace or control bytes; `--threads` 1..64; `--max-retries` 0..100; `--output` must name a file; a 200 body without the band marker is NoData only when it is non-empty, contains no `ServiceException` and does not start with markup, otherwise it is a parse error and retried; band values must be finite and within float range; the float-range guard stays at `> FLT_MAX`; `GDAL_NODATA` is registered with libtiff by the program; `save` fails (and keeps the previous checkpoint) when any tag write, key write, flush or temp-file creation fails; `load` rejects a reconstructed box that fails validation; certificate errors (`SSLLoadingCerts`, `SSLServerVerification`, `SSLServerHostnameVerification`) are not retried, and a CA bundle that failed to load stays failed for that worker; the first failed request and the first unparseable body are each reported once per process on stderr, the former with `(retrying)` or `(not retried)`; the consecutive-failure stop message names the last error; requests carry `User-Agent: dl_elevation/<version>` and a 60 s overall cap; TLS is tested on loopback with a certificate generated at test time. The reporter flushes after the starting, row and finish lines; its ETA counts only pixels not yet attempted this run (failed pixels are never retried within a run); `formatDuration` rounds to tenths before choosing the unit and treats NaN or negative input as 0; a stopped run gets a resume hint (`resume the remaining N pixels`, with `(M failed this run)` when some failed), a completed run with failures keeps the retry hint; `writeSummary(std::ostream&, const RunSummary&)` prints the finish lines for sinks that do not (the preview window).

**Open items by task**

- **Task 7**: done. Deferred by decision: a `Stopped` fetch kind and a `Cancelled` pixel state so work cut short by a stop is not counted as failed (accounting only; data and exit codes are unaffected).
- **Task 8**: done. `RunSummary` gained `checkpointWriteFailed`, and `remainingNaN` counts NaN pixels as of the last successful save (or the start of the run if none succeeded), so a failed final checkpoint can never exit 0. Deferred: a `Stopped` fetch kind; a test that a sink throwing on the InFlight callback is recorded as one Failed pixel; the `pool.stop(false)` rethrow path is untestable without allocation injection.
- **Task 9**: done. Not taken, by decision: a response payload cap (httplib's client has no simple API for it); a truly `noexcept` `fetch` with literal fallbacks for the error text (the Downloader catches the allocation-failure case, as `Downloader.h` documents); making 4xx statuses other than 408 and 429 non-retryable (owner decision; a wrong `--layer` or path currently costs about 20 minutes before the 100-failure stop, visible at once through the first-failure warning). Untested by design: the first-failure warning's text and the 60 s cap (once-per-process state and a trickling server); the `is_valid()` constructor path cannot be reached in this build.
- **Task 10**: done. Deferred by decision: plural count words stay plural at 1 except the rerun hints (`1 values`, `1 rows saved`); the percentage rounds to one decimal, so a run with a few failed pixels ends on `(100.0%)`; `formatDuration` rounds exact ties half away from zero while `%.1f` rounds half to even, never visible for the same quantity.
- **Task 11**: `->check(CLI::ExistingFile)` on `--ca-bundle`; make `--insecure` and `--ca-bundle` mutually exclusive; help text to state the 1..64 thread range, the 0..100 retry range, and that `--layer` is taken unencoded; `Config::validate` should refuse `;` in the layer name (the description format cannot carry it); note that CLI11 parses integers with base 0, so `--width 0100` is 64.
- **Task 12**: build the `stopping: <reason>; waiting for N in-flight requests` line as one string and write it to `std::cerr` (the reporter owns stdout and holds its own mutex, so a direct `std::cout` write can land inside a row or pixel line); call `std::cerr.tie(nullptr)` once at startup so a stderr write cannot flush a half-written stdout line; never call `sync_with_stdio(false)`; after `result.get()` on a `--preview` run call `demdl::writeSummary(std::cout, summary)` and flush, because `PreviewWindow::onRunFinished` prints nothing (spec 9 step 9 wants the summary); `stop.requested()` instead of `stop.stop.load()`; wrap `result.get()` in try/catch and exit 1 (a factory failure for a later worker id, a throwing run-thread callback or an allocation failure can still escape `run()`); the exit mapping must return 2 when `summary.checkpointWriteFailed` is set regardless of `remainingNaN`; consider a namespace-scope `constinit demdl::StopToken` instead of the raw `g_stop` pointer; print the "use --overwrite or choose another --output" hint for the RGB16 load failure as well; save once right after a successful `matches` on resume so an unwritable directory or full disk is found before the first row; construct one fetcher inside `main`'s guarded block before starting the run so constructor errors do not escape through `result.get()`.
- **Task 13**: pixels left Queued after a stop stay red until the window closes; acceptable while the window closes with the run. `PreviewWindow::onRunFinished` stays empty; main prints the summary through `writeSummary` (see Task 12).
- **Task 13, Step 7**: the client now fails fast on certificate errors, so if Homebrew OpenSSL does not trust the real server's chain the first live run prints `warning: first failed request: transport: SSL server verification failed (not retried)` and stops after 100 pixels; the remedy is `--ca-bundle` with the server's chain (the old `ca-bundle.crt` in the repo root may serve), never `--insecure` for a long run.
- **Task 13, before Step 7 or any live request** (needs the owner's approval, two requests): capture one land and one sea GetFeatureInfo body from the real server, for example the golden URL with `I=2040&J=8` and `I=1024&J=1500`, add them as recorded fixtures to `test_WmsQuery.cpp`, and settle the residual parse edges against them: a UTF-8 BOM before markup is currently NoData, a plain-text error line such as `Error: layer not found` is NoData, and `BAND 0: 12,5` parses as 12. The old `simonstown.tif` has 21.8 percent zero pixels in coherent sea regions, so sea responses must be known before the first long run.
- **Task 14**: spec section 8's listing still shows `RunPlan`, `RowReport` and `onPixelState` in their pre-deviation shapes and should be regenerated from the headers; refine the `remainingNaN` comment in `ProgressSink.h` and spec 8 to say "as of the last successful save, or at the start of the run if none succeeded"; complete the "Errors" paragraph in `Downloader.h` for exceptions on the calling thread after `onRunStarted`; state in CLAUDE.md that with the default policy and 8 workers an unreachable server or a wrong path takes about 20 minutes to trip the 100-failure limit, that certificate errors fail at once, and that the two once-per-process warnings go to stderr; the open-failure message now repeats the OS reason ("No such file or directory (No such file or directory)"): append the errno text only when libtiff's message does not already contain it; loop the BoundingBox finite-value test over all four edges; `.` and `..` output names should fail `output must name a file`; clearer `wms-url` error message naming what is refused; spec section 5 rows for `--output` and `--wms-url` to mention the new limits; spec 4.3 to mention the inclusive tolerance and `pixelScale` preconditions; the `BoundingBox.h` header comment could name libc++ and LWG 2524 for the closed `unitRandom` interval; consider a sanity cap on width x height in `Config::validate` (classic TIFF tops out near 4 GiB).
