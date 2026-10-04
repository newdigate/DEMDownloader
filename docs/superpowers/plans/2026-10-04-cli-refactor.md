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
6. httplib timeouts use the `(seconds, microseconds)` overloads, which exist in every version in play.
7. The value palette in the preview spans red to light blue rather than the full rainbow, so the minimum value is not pure blue (NoData) and the magenta end (Failed) is never used for a value.

## Working conventions

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
| `CMakeLists.txt` | targets, options, pinned dependencies | 1, 5, 6, 8, 9, 10, 11, 12, 13 |
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
# cpp-httplib is pinned to the Homebrew version in use so both dependency paths
# compile identical code. FIND_PACKAGE_ARGS prefers the system package when present.
FetchContent_Declare(httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG        v0.46.1
  FIND_PACKAGE_ARGS NAMES httplib)
# ctpl has no tags; this is its last commit (2015-06-12).
FetchContent_Declare(ctpl
  GIT_REPOSITORY https://github.com/vit-vit/ctpl.git
  GIT_TAG        437e135dbd94eb65b45533d9ce8ee28b5bd37b6d)
FetchContent_Declare(CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        v2.7.2)
FetchContent_Declare(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.16.0)
FetchContent_MakeAvailable(httplib ctpl CLI11)
if(DL_ELEVATION_TESTS)
  FetchContent_MakeAvailable(Catch2)
endif()

# libtiff via CMake's bundled FindTIFF. libgeotiff ships no CMake package, so locate it directly.
find_package(TIFF REQUIRED)
find_path(GEOTIFF_INCLUDE_DIR geotiff.h REQUIRED)
find_library(GEOTIFF_LIBRARY geotiff REQUIRED)

add_library(demdl STATIC
  src/demdl/BoundingBox.cpp)
target_include_directories(demdl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${GEOTIFF_INCLUDE_DIR})
target_include_directories(demdl SYSTEM PUBLIC ${ctpl_SOURCE_DIR})
# httplib::httplib exists on both paths: the Homebrew package only exports the
# namespaced name, and the FetchContent build adds it as an alias. The bare
# "httplib" name is not a target when the system package is found, so linking it
# silently drops CPPHTTPLIB_OPENSSL_SUPPORT and SSLClient disappears.
target_link_libraries(demdl PUBLIC httplib::httplib TIFF::TIFF ${GEOTIFF_LIBRARY})
target_compile_options(demdl PRIVATE -Wall -Wextra)
target_compile_definitions(demdl PUBLIC DL_ELEVATION_VERSION="${PROJECT_VERSION}")

if(DL_ELEVATION_TESTS)
  include(CTest)
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
/output/new2.tif
*.tif
*.tif.tmp
```

- [ ] **Step 3: Write the failing test `tests/test_BoundingBox.cpp`**

```cpp
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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
  double lonPerPixel;
  double latPerPixel;
};

// A WGS84 box in decimal degrees. north > south and east > west when valid.
struct BoundingBox {
  double north = 0.0;
  double south = 0.0;
  double east = 0.0;
  double west = 0.0;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  std::string validate() const;
  PixelScale pixelScale(int width, int height) const;
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

bool BoundingBox::approxEquals(const BoundingBox& o, double tolerance) const {
  return std::fabs(north - o.north) <= tolerance && std::fabs(south - o.south) <= tolerance &&
         std::fabs(east - o.east) <= tolerance && std::fabs(west - o.west) <= tolerance;
}

}  // namespace demdl
```

- [ ] **Step 7: Configure, build, run the test**

Run: `cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build && cmake --build build && ./build/demdl_tests "[boundingbox]"`
Expected: build succeeds (Catch2 compiles on the first build), then `All tests passed (N assertions in 6 test cases)`.

Run: `ctest --test-dir build --output-on-failure`
Expected: `100% tests passed, 0 tests failed out of 6`.

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
}

TEST_CASE("a URL without a path gets the root path", "[config]") {
  const auto ep = parseWmsUrl("https://host.test");
  REQUIRE(ep.has_value());
  CHECK(ep->path == "/");
}

TEST_CASE("unsupported or malformed URLs are rejected", "[config]") {
  CHECK_FALSE(parseWmsUrl("ftp://host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https:///wms").has_value());
  CHECK_FALSE(parseWmsUrl("host.test/wms").has_value());
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

  Config failures = exampleConfig();
  failures.maxConsecutiveFailures = 0;
  CHECK(failures.validate() == "max-consecutive-failures must be at least 1");

  Config output = exampleConfig();
  output.output.clear();
  CHECK(output.validate() == "output path must not be empty");
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
  std::string origin;  // "https://host[:port]", what httplib::Client is constructed with
  std::string path;    // "/erdas-iws/ogc/wms/GeoSpatial%20Datasets", kept as given
};

// Splits "scheme://host[:port]/path". nullopt unless the scheme is http or https and a host is present.
std::optional<WmsEndpoint> parseWmsUrl(const std::string& url);

// Every runtime option. Mirrors the command-line flags one to one; no parsing lives here.
struct Config {
  BoundingBox box;
  int width = 0;
  int height = 0;
  std::filesystem::path output;
  std::string wmsUrl = "https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets";
  std::string layer = "Elevation_CCT_GLM2019_5m_DEM";
  int threads = 8;
  int maxRetries = 8;
  int maxConsecutiveFailures = 100;
  std::string caBundle;
  bool insecure = false;
  bool preview = false;
  bool overwrite = false;
  bool dryRun = false;
  bool verbose = false;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  std::string validate() const;
};

}  // namespace demdl
```

- [ ] **Step 6: Write `src/demdl/Config.cpp`**

```cpp
#include "demdl/Config.h"

namespace demdl {

std::optional<WmsEndpoint> parseWmsUrl(const std::string& url) {
  const auto schemeEnd = url.find("://");
  if (schemeEnd == std::string::npos) return std::nullopt;
  const std::string scheme = url.substr(0, schemeEnd);
  if (scheme != "http" && scheme != "https") return std::nullopt;
  const std::string rest = url.substr(schemeEnd + 3);
  const auto slash = rest.find('/');
  const std::string hostPort = rest.substr(0, slash);
  if (hostPort.empty()) return std::nullopt;
  WmsEndpoint ep;
  ep.origin = scheme + "://" + hostPort;
  ep.path = slash == std::string::npos ? std::string("/") : rest.substr(slash);
  return ep;
}

std::string Config::validate() const {
  if (const std::string err = box.validate(); !err.empty()) return err;
  if (width <= 0 || height <= 0) return "width and height must be positive";
  if (output.empty()) return "output path must not be empty";
  if (!parseWmsUrl(wmsUrl)) {
    return "wms-url must look like scheme://host[:port]/path with scheme http or https";
  }
  if (layer.empty()) return "layer must not be empty";
  if (threads < 1) return "threads must be at least 1";
  if (maxRetries < 0) return "max-retries must be at least 0";
  if (maxConsecutiveFailures < 1) return "max-consecutive-failures must be at least 1";
  return {};
}

}  // namespace demdl
```

- [ ] **Step 7: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[config]"`
Expected: `All tests passed (N assertions in 6 test cases)`.

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

TEST_CASE("percentEncode keeps unreserved characters", "[wmsquery]") {
  CHECK(percentEncode("Abc-1.2_x~") == "Abc-1.2_x~");
  CHECK(percentEncode("a b:c/d") == "a%20b%3Ac%2Fd");
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

TEST_CASE("a body without the band marker is NoData", "[wmsquery]") {
  CHECK(parseElevation("GetFeatureInfo results:\n").kind == ParseResult::Kind::NoData);
  CHECK(parseElevation("").kind == ParseResult::Kind::NoData);
}

TEST_CASE("a band marker without a number is a parse error", "[wmsquery]") {
  CHECK(parseElevation("BAND 0: abc").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0:").kind == ParseResult::Kind::ParseError);
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
  Kind kind;
  double value = 0.0;  // valid when kind == Value
};

// RFC 3986 percent-encoding: unreserved characters pass through, everything else becomes %XX.
std::string percentEncode(std::string_view s);

// Degrees with nine fixed decimals (about 0.1 mm), used in URLs and user-facing reports alike.
std::string formatCoordinate(double degrees);

// Request target (path plus query) of the WMS 1.3.0 GetFeatureInfo request for pixel column x, row y.
// The box is sent as south,west,north,east, the axis order WMS 1.3.0 requires for EPSG:4326.
// Throws std::invalid_argument when cfg.wmsUrl does not parse.
std::string featureInfoUrl(const Config& cfg, int x, int y);

// "BAND 0: <number>" anywhere in the body is a value; no marker is NoData; a marker without
// a parseable number is ParseError.
ParseResult parseElevation(std::string_view body);

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/WmsQuery.cpp`**

```cpp
#include "demdl/WmsQuery.h"

#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>

namespace demdl {

namespace {

bool isUnreserved(unsigned char c) {
  return std::isalnum(c) != 0 || c == '-' || c == '.' || c == '_' || c == '~';
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
  if (pos == std::string_view::npos) return {ParseResult::Kind::NoData, 0.0};
  const std::string_view rest = body.substr(pos + marker.size());
  const auto lineEnd = rest.find_first_of("\r\n");
  // strtod needs a null-terminated string; libc++ on AppleClang 15 has no floating-point from_chars.
  const std::string token(rest.substr(0, lineEnd));
  const char* begin = token.c_str();
  char* end = nullptr;
  const double value = std::strtod(begin, &end);
  if (end == begin) return {ParseResult::Kind::ParseError, 0.0};
  return {ParseResult::Kind::Value, value};
}

}  // namespace demdl
```

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[wmsquery]"`
Expected: `All tests passed (N assertions in 9 test cases)`.

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
  std::chrono::milliseconds maxDelay{30000};
  double jitter = 0.2;

  // Delay before retry number `attempt` (0-based). unitRandom is in [0, 1]; passing it in keeps the schedule testable.
  std::chrono::milliseconds delayFor(int attempt, double unitRandom) const;
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
Expected: `All tests passed (N assertions in 3 test cases)`.

- [ ] **Step 7: Commit**

```bash
git add CMakeLists.txt src/demdl/RetryPolicy.h src/demdl/RetryPolicy.cpp tests/test_RetryPolicy.cpp
git commit -m "feat: RetryPolicy with capped exponential backoff and jitter" -m "Co-Authored-By: Claude Fable 5.1 <noreply@anthropic.com>"
```

---

## STATUS: PARTIAL PLAN. Tasks 1 to 4 above are complete and executable. Tasks 5 to 15 below are an outline only and must be expanded to full steps (test code, implementation code, commands, commits) before execution.

The session that wrote this ran out of usage budget after Task 4. Expand each outline into the same bite-sized TDD format as Tasks 1 to 4. Interface names below are the ones Tasks 1 to 4 and the spec already use; keep them.

- **Task 5, DemRaster in memory.** `src/demdl/DemRaster.h/.cpp`, `tests/test_DemRaster.cpp`. `RasterMetadata{layer, wmsUrl}`; `static DemRaster create(width, height, box, meta)` filled with NaN; `at(x,y)`, `set(x,y,v)`, `todoInRow(row)`, `rowsWithWork()`, `countNaN()`, `pixelCount()`, `width()`, `height()`, `box()`, `metadata()`; constant `kNoData = -32768.0f`; free functions `formatImageDescription(meta)` producing `dl_elevation;layer=<layer>;wms=<url>` and `parseImageDescription(string_view)` where `wms=` is last and runs to the end.
- **Task 6, DemRaster GeoTIFF I/O.** Add `save(path)` (write `<path>.tmp` via `XTIFFOpen`, fields per spec section 6, `GTIFKeySet` for GTModelTypeGeoKey=ModelTypeGeographic, GTRasterTypeGeoKey=RasterPixelIsArea, GeographicTypeGeoKey=GCS_WGS_84, `TIFFTAG_GEOTIEPOINTS` count 6, `TIFFTAG_GEOPIXELSCALE` count 3, `TIFFTAG_GDAL_NODATA` "-32768", then `std::filesystem::rename`), `static load(path)` (reject anything not 1 sample, 32 bits, SAMPLEFORMAT_IEEEFP with the legacy RGB16 message; box from tiepoint[3], tiepoint[4] and the scale), `bool matches(const Config&, std::string& why)` with 1e-9 degree tolerance using `formatCoordinate`, `std::string wmsUrlWarning(const Config&)`. `tests/TempDir.h` RAII temp directory. Tests: round trip, keys read back with `GTIFKeyGetSHORT`, matches and mismatch text, hand-written RGB16 file rejected, no `.tmp` left. Set `glPixelStorei` is irrelevant here; note `TIFFGetField` for the geo tags takes `uint16_t count, double* data`.
- **Task 7, interfaces and doubles.** `PixelFetcher.h` (`FetchResult{kind Value|NoData|Failed, value, attempts, error}`, `virtual FetchResult fetch(int x, int y, const std::atomic<bool>& stop)`); `ProgressSink.h` (`PixelState{Queued, InFlight, Value, NoData, Failed}`, `StopReason{None, Signal, WindowClosed, ConsecutiveFailures, WriteError}` with `describe()`, `RunPlan{pixelsTotal, pixelsTodo, rowsTodo, rowsTotal}`, `PixelEvent{x, y, state, value, attempts, error}`, `RowReport{row, rowsTotal, ok, noData, failed, elapsed, pixelsDoneTotal, pixelsTotal}`, `RunSummary{pixelsFetched, noData, failed, remainingNaN, rowsSaved, elapsed, stopReason, stopMessage}`, four virtual methods); `StopToken.h` (`std::atomic<bool> stop`, `request(StopReason)` keeping the first reason via compare_exchange, `reason()`); `tests/FakeFetcher.h` (behaviour lambda, optional `Gate` that blocks until released and counts waiters, call counter), `tests/RecordingSink.h`; `tests/test_StopToken.cpp`.
- **Task 8, Downloader.** `Downloader(DemRaster&, output path, FetcherFactory, ProgressSink&, StopToken&, DownloaderOptions{threads, maxConsecutiveFailures})`, `RunSummary run()`, `int inFlight()`. Row loop per spec section 7 with ctpl; a job returns immediately when stop is already set; one fetcher per worker id; consecutive-failure counter resets on Value or NoData; `save` exceptions become `StopReason::WriteError`. Tests with FakeFetcher: only NaN pixels fetched and finished rows skipped; NoData and Failed counted; stop mid-row via Gate saves the partial row and goes no further; consecutive-failure limit stops with that reason; complete raster makes no requests.
- **Task 9, ElevationClient.** Implements PixelFetcher over one `httplib::Client(origin)` with keep-alive, `set_connection_timeout(10, 0)`, `set_read_timeout(30, 0)`, `set_write_timeout(10, 0)`, verification unless `insecure`, `set_ca_cert_path` when given; retry loop per spec with sleep sliced in 100 ms checking stop; first unparseable body logged once through a shared `std::once_flag`. Test with an in-process `httplib::Server` bound with `bind_to_any_port("127.0.0.1")`, `listen_after_bind()` in a thread, `wait_until_ready()`: value, NoData body, 500 then 200, and a refused port for failure after retries, using a tiny RetryPolicy.
- **Task 10, ConsoleReporter.** `ConsoleReporter(std::ostream&, bool verbose)`, free `formatDuration(double seconds)` ("41.2s", "12m05s", "13h31m"). Row line `row 812/1792  done 1247296/3670016 (34.0%)  row: 2048 ok 0 nodata 3 failed 41.2s  49.8 px/s  eta 13h31m` with rate over the last ten rows; verbose pixel lines; finish summary and the retry hint. Test with a stringstream.
- **Task 11, Cli.** `src/app/Cli.h/.cpp`: `cli::parseArgs(int argc, const char* const* argv, Config&, std::ostream& out, std::ostream& err) -> ParseOutcome{exitNow, exitCode}` using CLI11 (flags per spec section 5, `set_version_flag("--version", DL_ELEVATION_VERSION)`, parse `--output` into a std::string then assign to the path). CMake: `add_library(demdl_cli OBJECT src/app/Cli.cpp)` linking `demdl` and `CLI11::CLI11` PUBLIC; tests link it. Tests: example argv, missing `--north`, `--help`, `--threads 0` fails validate, `--insecure` and `--ca-bundle`.
- **Task 12, main and the executable.** `src/app/main.cpp` per spec section 9: parse, validate, load-or-create with mismatch error, `--dry-run` plan print (box, grid, output state, rows and pixels remaining, first request URL), initial save, SIGINT/SIGTERM handler calling `StopToken::request(Signal)` and `_Exit(130)` on the second signal, SIGPIPE ignored, `std::async` for the downloader, 100 ms poll loop that prints `stopping: <describe(reason)>; waiting for N in-flight requests` once, exit code mapping (0 complete, 2 stopped, 3 failures). Include the `#ifdef DL_ELEVATION_HAVE_PREVIEW` blocks now so Task 13 does not edit this file. CMake: `add_executable(dl_elevation src/app/main.cpp)` linking `demdl demdl_cli`. Smoke: `--dry-run` with a 16x8 grid exits 0 and creates no file; `--threads 0` exits 1; `--help` exits 0.
- **Task 13, PreviewWindow.** `src/preview/Shader.h` (constructor from two source strings, `use()`, `setInt()`, no GLM), `src/preview/PreviewWindow.h/.cpp` (`PreviewWindow(const DemRaster&, StopToken&)`, `pump()` called from the main loop, sink methods updating a state byte array and value array under a mutex; redraw every 500 ms recolouring from the current min and max; `glPixelStorei(GL_UNPACK_ALIGNMENT, 1)` because width*3 need not be a multiple of 4; colours per spec section 8 with the value palette from red to light blue; `glfwWindowShouldClose` requests `WindowClosed`; on Linux call `glewInit()` after context creation). CMake block guarded by `DL_ELEVATION_PREVIEW` with `pkg_search_module(glfw REQUIRED IMPORTED_TARGET glfw3)`, Apple frameworks OpenGL/Cocoa/IOKit/CoreVideo, else OpenGL::GL and GLEW; link into `dl_elevation` and define `DL_ELEVATION_HAVE_PREVIEW`. Optional manual check only with the owner's approval because it makes live requests.
- **Task 14, cleanup and docs.** `git rm -r include opengl.cmake.in main.cpp`; rewrite `CLAUDE.md` for the new layout, build and test commands, flags, resume rules, exit codes, and the three deliberate CMake oddities (httplib::httplib, CMAKE_FIND_FRAMEWORK LAST, pinned versions; the GLM find_path bullet goes away with GLM).
- **Task 15, final verification.** Clean configure and build on both dependency paths (default and `-DFETCHCONTENT_TRY_FIND_PACKAGE_MODE=NEVER`), `ctest`, a `-DDL_ELEVATION_PREVIEW=OFF` build to prove the headless build needs no GL packages, the dry-run smoke, and a clean `git status`.
