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
  const RasterMetadata meta{"Elevation_CCT_GLM2019_5m_DEM", "https://host.test/a;b=c?x=1"};
  const std::string text = demdl::formatImageDescription(meta);
  CHECK(text == "dl_elevation;layer=Elevation_CCT_GLM2019_5m_DEM;wms=https://host.test/a;b=c?x=1");
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
std::string formatImageDescription(const RasterMetadata& meta);
RasterMetadata parseImageDescription(std::string_view description);

// A float32 elevation grid in metres. Row 0 is the north edge, column 0 the west edge.
// NaN means "no value yet"; kNoData means the server has no elevation there.
class DemRaster {
 public:
  static constexpr float kNoData = -32768.0f;

  // Every pixel starts as NaN. Throws std::invalid_argument for a non-positive size.
  static DemRaster create(int width, int height, const BoundingBox& box, RasterMetadata meta);

  int width() const { return width_; }
  int height() const { return height_; }
  const BoundingBox& box() const { return box_; }
  const RasterMetadata& metadata() const { return meta_; }

  float at(int x, int y) const { return data_[index(x, y)]; }
  void set(int x, int y, float value) { data_[index(x, y)] = value; }

  std::vector<int> todoInRow(int row) const;  // columns still NaN, ascending
  std::vector<int> rowsWithWork() const;      // rows with at least one NaN, ascending
  std::uint64_t countNaN() const;
  std::uint64_t pixelCount() const { return data_.size(); }

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

Add these includes at the top of the file, after the existing ones:

```cpp
#include <catch2/matchers/catch_matchers_string.hpp>

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <cstdint>
#include <filesystem>

#include "ExampleConfig.h"
#include "TempDir.h"
#include "demdl/Config.h"

using Catch::Matchers::ContainsSubstring;
using demdl::Config;
```

Append these test cases at the end of the file:

```cpp
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

TEST_CASE("loading a missing file reports the path", "[demraster]") {
  TempDir dir;
  CHECK_THROWS_WITH(DemRaster::load(dir.path / "absent.tif"), ContainsSubstring("absent.tif"));
}
```

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with errors such as `no member named 'save' in 'demdl::DemRaster'` and `no member named 'load'`.

- [ ] **Step 4: Replace `src/demdl/DemRaster.h`**

```cpp
#pragma once

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "demdl/BoundingBox.h"
#include "demdl/Config.h"

namespace demdl {

struct RasterMetadata {
  std::string layer;
  std::string wmsUrl;
};

// "dl_elevation;layer=<layer>;wms=<url>". Fields are separated by ';'. wms is always last and
// extends to the end of the string so a URL may contain any character.
std::string formatImageDescription(const RasterMetadata& meta);
RasterMetadata parseImageDescription(std::string_view description);

// A float32 elevation grid in metres. Row 0 is the north edge, column 0 the west edge.
// NaN means "no value yet"; kNoData means the server has no elevation there.
class DemRaster {
 public:
  static constexpr float kNoData = -32768.0f;
  static constexpr double kBoxTolerance = 1e-9;  // degrees, for resume checks

  // Every pixel starts as NaN. Throws std::invalid_argument for a non-positive size.
  static DemRaster create(int width, int height, const BoundingBox& box, RasterMetadata meta);

  // Reads a file written by save(). Throws std::runtime_error with a user-facing message when the
  // file cannot be opened, is not a single-band float32 GeoTIFF (including the previous RGB16
  // format), or lacks the georeferencing tags.
  static DemRaster load(const std::filesystem::path& path);

  // Writes <path>.tmp beside the output, then renames it over <path>, so a crash mid-write leaves
  // the previous file intact. Throws std::runtime_error on any failure.
  void save(const std::filesystem::path& path) const;

  // True when this raster can be resumed for cfg. Otherwise `why` lists every differing field,
  // one per line, each indented by two spaces and ending in a newline.
  bool matches(const Config& cfg, std::string& why) const;

  // Non-empty when the file was made from a different WMS URL; a warning, not an error.
  std::string wmsUrlWarning(const Config& cfg) const;

  int width() const { return width_; }
  int height() const { return height_; }
  const BoundingBox& box() const { return box_; }
  const RasterMetadata& metadata() const { return meta_; }

  float at(int x, int y) const { return data_[index(x, y)]; }
  void set(int x, int y, float value) { data_[index(x, y)] = value; }

  std::vector<int> todoInRow(int row) const;  // columns still NaN, ascending
  std::vector<int> rowsWithWork() const;      // rows with at least one NaN, ascending
  std::uint64_t countNaN() const;
  std::uint64_t pixelCount() const { return data_.size(); }

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

- [ ] **Step 5: Replace `src/demdl/DemRaster.cpp`**

```cpp
#include "demdl/DemRaster.h"

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
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
  TiffHandle tif(XTIFFOpen(path.string().c_str(), "r"));
  if (!tif) throw std::runtime_error("cannot open " + path.string());
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
    throw std::runtime_error(path.string() +
                             " is not a dl_elevation float32 GeoTIFF; files from the previous RGB16 "
                             "format cannot be resumed");
  }
  if (width == 0 || height == 0) throw std::runtime_error(path.string() + " has an empty image");

  uint16_t tieCount = 0;
  double* tiepoints = nullptr;
  if (!TIFFGetField(t, TIFFTAG_GEOTIEPOINTS, &tieCount, &tiepoints) || tieCount < 6 || tiepoints[0] != 0.0 ||
      tiepoints[1] != 0.0) {
    throw std::runtime_error(path.string() + " has no usable GeoTIFF tiepoint");
  }
  uint16_t scaleCount = 0;
  double* scale = nullptr;
  if (!TIFFGetField(t, TIFFTAG_GEOPIXELSCALE, &scaleCount, &scale) || scaleCount < 2) {
    throw std::runtime_error(path.string() + " has no GeoTIFF pixel scale");
  }
  BoundingBox box;
  box.west = tiepoints[3];
  box.north = tiepoints[4];
  box.east = box.west + scale[0] * width;
  box.south = box.north - scale[1] * height;

  RasterMetadata meta;
  char* description = nullptr;
  if (TIFFGetField(t, TIFFTAG_IMAGEDESCRIPTION, &description) && description != nullptr) {
    meta = parseImageDescription(description);
  }

  DemRaster r = create(static_cast<int>(width), static_cast<int>(height), box, std::move(meta));
  for (uint32_t y = 0; y < height; ++y) {
    if (TIFFReadScanline(t, &r.data_[r.index(0, static_cast<int>(y))], y, 0) < 0) {
      throw std::runtime_error("read failed on row " + std::to_string(y) + " of " + path.string());
    }
  }
  return r;
}

void DemRaster::save(const std::filesystem::path& path) const {
  const std::filesystem::path tmp = path.string() + ".tmp";
  const auto fail = [&tmp](const std::string& what) {
    std::error_code ignored;
    std::filesystem::remove(tmp, ignored);
    throw std::runtime_error(what);
  };
  {
    TiffHandle tif(XTIFFOpen(tmp.string().c_str(), "w"));
    if (!tif) throw std::runtime_error("cannot create " + tmp.string());
    TIFF* t = tif.get();
    TIFFSetField(t, TIFFTAG_IMAGEWIDTH, static_cast<uint32_t>(width_));
    TIFFSetField(t, TIFFTAG_IMAGELENGTH, static_cast<uint32_t>(height_));
    TIFFSetField(t, TIFFTAG_SAMPLESPERPIXEL, 1);
    TIFFSetField(t, TIFFTAG_BITSPERSAMPLE, 32);
    TIFFSetField(t, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    TIFFSetField(t, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    TIFFSetField(t, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    TIFFSetField(t, TIFFTAG_COMPRESSION, COMPRESSION_NONE);
    TIFFSetField(t, TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    TIFFSetField(t, TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(t, 0));

    const PixelScale scale = box_.pixelScale(width_, height_);
    const double tiepoints[6] = {0.0, 0.0, 0.0, box_.west, box_.north, 0.0};
    const double pixelScale[3] = {scale.lonPerPixel, scale.latPerPixel, 0.0};
    TIFFSetField(t, TIFFTAG_GEOTIEPOINTS, 6, tiepoints);
    TIFFSetField(t, TIFFTAG_GEOPIXELSCALE, 3, pixelScale);
    TIFFSetField(t, TIFFTAG_GDAL_NODATA, "-32768");
    const std::string description = formatImageDescription(meta_);
    TIFFSetField(t, TIFFTAG_IMAGEDESCRIPTION, description.c_str());
    const std::string software = std::string("dl_elevation ") + DL_ELEVATION_VERSION;
    TIFFSetField(t, TIFFTAG_SOFTWARE, software.c_str());

    {
      GtifHandle gtif(GTIFNew(t));
      if (!gtif) {
        tif.reset();
        fail("cannot attach GeoTIFF keys to " + tmp.string());
      }
      GTIFKeySet(gtif.get(), GTModelTypeGeoKey, TYPE_SHORT, 1, ModelTypeGeographic);
      GTIFKeySet(gtif.get(), GTRasterTypeGeoKey, TYPE_SHORT, 1, RasterPixelIsArea);
      GTIFKeySet(gtif.get(), GeographicTypeGeoKey, TYPE_SHORT, 1, GCS_WGS_84);
      GTIFWriteKeys(gtif.get());
    }

    std::vector<float> row(static_cast<std::size_t>(width_));
    for (int y = 0; y < height_; ++y) {
      std::memcpy(row.data(), &data_[index(0, y)], row.size() * sizeof(float));
      if (TIFFWriteScanline(t, row.data(), static_cast<uint32_t>(y), 0) < 0) {
        tif.reset();
        fail("write failed on row " + std::to_string(y) + " of " + tmp.string());
      }
    }
  }  // XTIFFClose writes the directory
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) fail("cannot rename " + tmp.string() + " to " + path.string() + ": " + ec.message());
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
    if (std::fabs(file - requested) > kBoxTolerance) {
      why += std::string("  ") + name + ": file " + formatCoordinate(file) + ", requested " +
             formatCoordinate(requested) + "\n";
    }
  };
  edge("north", box_.north, cfg.box.north);
  edge("south", box_.south, cfg.box.south);
  edge("east", box_.east, cfg.box.east);
  edge("west", box_.west, cfg.box.west);
  if (meta_.layer != cfg.layer) {
    why += "  layer: file " + meta_.layer + ", requested " + cfg.layer + "\n";
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

Notes for the engineer: `XTIFFOpen` registers the GeoTIFF tags with libtiff, which is why the geo tags are set and read through plain `TIFFSetField`/`TIFFGetField`; the tag count for them is a `uint16_t` and the data a `double*`. `TIFFTAG_GDAL_NODATA` is a built-in ASCII tag in libtiff 4.x. The `GTIFKeySet` value arguments are plain `int`s through varargs, so the enum constants are passed as written.

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[demraster]"`
Expected: `All tests passed (N assertions in 10 test cases)`.

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
#include <string>

#include "FakeFetcher.h"
#include "RecordingSink.h"
#include "demdl/StopToken.h"

using demdl::StopReason;
using demdl::StopToken;

TEST_CASE("a stop token keeps the first reason", "[stoptoken]") {
  StopToken t;
  CHECK_FALSE(t.stop.load());
  CHECK(t.reason() == StopReason::None);
  t.request(StopReason::Signal);
  CHECK(t.stop.load());
  CHECK(t.reason() == StopReason::Signal);
  t.request(StopReason::WindowClosed);
  CHECK(t.reason() == StopReason::Signal);
}

TEST_CASE("describe names every reason", "[stoptoken]") {
  CHECK(std::string(demdl::describe(StopReason::None)) == "finished");
  CHECK(std::string(demdl::describe(StopReason::Signal)) == "interrupted");
  CHECK(std::string(demdl::describe(StopReason::WindowClosed)) == "preview window closed");
  CHECK(std::string(demdl::describe(StopReason::ConsecutiveFailures)) == "too many consecutive failures");
  CHECK(std::string(demdl::describe(StopReason::WriteError)) == "checkpoint write failed");
}

TEST_CASE("the test doubles compile and behave", "[stoptoken]") {
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
  sink.onPixelState({1, 2, demdl::PixelState::Failed, 0.0f, 2, "boom"});
  sink.onPixelState({1, 3, demdl::PixelState::Value, 4.5f, 1, {}});
  CHECK(sink.count(demdl::PixelState::Failed) == 1);
  CHECK(sink.count(demdl::PixelState::Value) == 1);
  CHECK(sink.pixels.size() == 2);
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
  float value = 0.0f;  // valid when kind == Value
  int attempts = 0;    // attempts made, including the first
  std::string error;   // last error when kind == Failed
};

// Fetches one pixel. Implementations may block for retries but must return promptly once `stop`
// is set. One instance serves one worker thread; instances need not be thread-safe.
class PixelFetcher {
 public:
  virtual ~PixelFetcher() = default;
  virtual FetchResult fetch(int x, int y, const std::atomic<bool>& stop) = 0;
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

inline const char* describe(StopReason reason) {
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
  float value = 0.0f;  // valid when state == Value
  int attempts = 0;
  std::string error;   // set when state == Failed
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
  std::uint64_t remainingNaN = 0;
  int rowsSaved = 0;
  std::chrono::duration<double> elapsed{0.0};
  StopReason stopReason = StopReason::None;
  std::string stopMessage;
};

// Pixel events arrive from worker threads; row and run events from the downloader thread.
// Implementations guard their own state.
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

#include "demdl/ProgressSink.h"

namespace demdl {

// Shared "please stop" flag plus the first reason given. request() touches only lock-free
// atomics, so a signal handler may call it.
class StopToken {
 public:
  std::atomic<bool> stop{false};

  void request(StopReason reason) noexcept {
    StopReason expected = StopReason::None;
    reason_.compare_exchange_strong(expected, reason);
    stop.store(true);
  }
  StopReason reason() const noexcept { return reason_.load(); }

 private:
  std::atomic<StopReason> reason_{StopReason::None};
};

}  // namespace demdl
```

- [ ] **Step 7: Write `tests/FakeFetcher.h`**

```cpp
#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include "demdl/PixelFetcher.h"

// Blocks every caller of wait() until release() is called, and counts how many have arrived.
struct Gate {
  std::mutex m;
  std::condition_variable cv;
  bool open = false;
  std::atomic<int> waiting{0};

  void wait() {
    ++waiting;
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [this] { return open; });
  }
  void release() {
    {
      std::lock_guard<std::mutex> lock(m);
      open = true;
    }
    cv.notify_all();
  }
};

// Answers fetches from a lambda; optionally blocks on a Gate and counts calls.
struct FakeFetcher : demdl::PixelFetcher {
  std::function<demdl::FetchResult(int x, int y)> behaviour;
  std::atomic<int>* calls = nullptr;
  Gate* gate = nullptr;

  demdl::FetchResult fetch(int x, int y, const std::atomic<bool>&) override {
    if (gate != nullptr) gate->wait();
    if (calls != nullptr) ++*calls;
    return behaviour(x, y);
  }
};

inline demdl::FetchResult valueResult(float value) {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::Value;
  r.value = value;
  r.attempts = 1;
  return r;
}

inline demdl::FetchResult noDataResult() {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::NoData;
  r.attempts = 1;
  return r;
}

inline demdl::FetchResult failedResult(std::string error) {
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
  std::mutex m;
  demdl::RunPlan plan;
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

  int count(demdl::PixelState state) {
    std::lock_guard<std::mutex> lock(m);
    return static_cast<int>(std::count_if(pixels.begin(), pixels.end(),
                                          [state](const demdl::PixelEvent& e) { return e.state == state; }));
  }
};
```

- [ ] **Step 9: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[stoptoken]"`
Expected: `All tests passed (N assertions in 3 test cases)`.

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

#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <thread>

#include "FakeFetcher.h"
#include "RecordingSink.h"
#include "TempDir.h"
#include "demdl/Downloader.h"

using namespace std::chrono_literals;
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

struct Fixture {
  TempDir dir;
  std::filesystem::path output = dir.path / "out.tif";
  RecordingSink sink;
  StopToken stop;
  std::atomic<int> calls{0};
  Gate gate;

  FetcherFactory factory(std::function<FetchResult(int, int)> behaviour, bool blocking = false) {
    return [this, behaviour, blocking](int) -> std::unique_ptr<demdl::PixelFetcher> {
      auto f = std::make_unique<FakeFetcher>();
      f->behaviour = behaviour;
      f->calls = &calls;
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
  CHECK(f.sink.plan.pixelsTotal == 12);
  CHECK(f.sink.plan.pixelsTodo == 6);
  CHECK(f.sink.plan.rowsTodo == 2);
  CHECK(f.sink.plan.rowsTotal == 3);
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

TEST_CASE("a stop request saves the partial row and goes no further", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(4, 2, kBox, {});
  Downloader d(raster, f.output,
               f.factory([](int x, int y) { return valueResult(expectedValue(x, y)); }, /*blocking=*/true),
               f.sink, f.stop, DownloaderOptions{2, 100});

  std::thread runner([&d] { d.run(); });
  const auto deadline = std::chrono::steady_clock::now() + 5s;
  while (f.gate.waiting.load() < 2 && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::sleep_for(1ms);
  }
  REQUIRE(f.gate.waiting.load() == 2);
  CHECK(d.inFlight() == 2);

  f.stop.request(StopReason::Signal);
  f.gate.release();
  runner.join();

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
}

TEST_CASE("too many consecutive failures stop the run", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(10, 1, kBox, {});
  Downloader d(raster, f.output, f.factory([](int, int) { return failedResult("connection refused"); }),
               f.sink, f.stop, DownloaderOptions{1, 3});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::ConsecutiveFailures);
  CHECK(summary.stopMessage == "server unreachable: 3 consecutive failures");
  CHECK(summary.failed == 3);
  CHECK(f.calls.load() == 3);
  CHECK(summary.remainingNaN == 10);
  CHECK(summary.rowsSaved == 1);
  CHECK(std::filesystem::exists(f.output));
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
  CHECK(f.sink.plan.pixelsTodo == 0);
  CHECK(f.sink.plan.rowsTodo == 0);
  REQUIRE(f.sink.summary.has_value());
  CHECK_FALSE(std::filesystem::exists(f.output));
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
#include <filesystem>
#include <functional>
#include <memory>

#include "demdl/DemRaster.h"
#include "demdl/PixelFetcher.h"
#include "demdl/ProgressSink.h"
#include "demdl/StopToken.h"

namespace demdl {

// Creates the fetcher for one worker thread. Called once per worker before the first row.
using FetcherFactory = std::function<std::unique_ptr<PixelFetcher>(int workerId)>;

struct DownloaderOptions {
  int threads = 8;
  int maxConsecutiveFailures = 100;
};

// Walks the rows that still contain NaN pixels, fetches those pixels through a thread pool,
// saves the raster after each row and reports progress. One run() per instance.
class Downloader {
 public:
  Downloader(DemRaster& raster, std::filesystem::path output, FetcherFactory factory, ProgressSink& sink,
             StopToken& stop, DownloaderOptions options);

  RunSummary run();

  // Pixels currently inside a fetch call. For the "stopping" message.
  int inFlight() const { return inFlight_.load(); }

 private:
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
#include <exception>
#include <future>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace demdl {

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

  const std::vector<int> rows = raster_.rowsWithWork();
  RunPlan plan;
  plan.pixelsTotal = raster_.pixelCount();
  plan.pixelsTodo = raster_.countNaN();
  plan.rowsTodo = static_cast<int>(rows.size());
  plan.rowsTotal = raster_.height();
  sink_.onRunStarted(plan);

  ctpl::thread_pool pool(options_.threads);
  std::vector<std::unique_ptr<PixelFetcher>> fetchers;
  fetchers.reserve(static_cast<std::size_t>(options_.threads));
  for (int i = 0; i < options_.threads; ++i) fetchers.push_back(factory_(i));

  std::atomic<int> consecutiveFailures{0};
  std::atomic<std::uint64_t> fetched{0};
  std::atomic<std::uint64_t> noData{0};
  std::atomic<std::uint64_t> failed{0};
  std::uint64_t doneTotal = plan.pixelsTotal - plan.pixelsTodo;
  int rowsSaved = 0;
  std::string writeError;

  for (const int row : rows) {
    if (stop_.stop.load()) break;
    const auto rowStart = Clock::now();
    const std::vector<int> todo = raster_.todoInRow(row);
    for (const int x : todo) sink_.onPixelState({x, row, PixelState::Queued, 0.0f, 0, {}});

    std::atomic<int> rowOk{0};
    std::atomic<int> rowNoData{0};
    std::atomic<int> rowFailed{0};
    std::vector<std::future<void>> futures;
    futures.reserve(todo.size());
    for (const int x : todo) {
      if (stop_.stop.load()) break;  // stop queueing immediately
      futures.push_back(pool.push([&, x, row](int workerId) {
        if (stop_.stop.load()) return;  // queued before the stop, never started
        ++inFlight_;
        sink_.onPixelState({x, row, PixelState::InFlight, 0.0f, 0, {}});
        const FetchResult result = fetchers[static_cast<std::size_t>(workerId)]->fetch(x, row, stop_.stop);
        --inFlight_;
        switch (result.kind) {
          case FetchResult::Kind::Value:
            raster_.set(x, row, result.value);
            consecutiveFailures.store(0);
            ++rowOk;
            ++fetched;
            sink_.onPixelState({x, row, PixelState::Value, result.value, result.attempts, {}});
            break;
          case FetchResult::Kind::NoData:
            raster_.set(x, row, DemRaster::kNoData);
            consecutiveFailures.store(0);
            ++rowNoData;
            ++noData;
            sink_.onPixelState({x, row, PixelState::NoData, 0.0f, result.attempts, {}});
            break;
          case FetchResult::Kind::Failed: {
            ++rowFailed;
            ++failed;
            const int streak = ++consecutiveFailures;
            sink_.onPixelState({x, row, PixelState::Failed, 0.0f, result.attempts, result.error});
            if (streak >= options_.maxConsecutiveFailures) stop_.request(StopReason::ConsecutiveFailures);
            break;
          }
        }
      }));
    }
    for (auto& f : futures) f.get();

    try {
      raster_.save(output_);
    } catch (const std::exception& e) {
      writeError = e.what();
      stop_.request(StopReason::WriteError);
      break;
    }
    ++rowsSaved;
    doneTotal += static_cast<std::uint64_t>(rowOk.load() + rowNoData.load());
    RowReport report;
    report.row = row;
    report.rowsTotal = raster_.height();
    report.ok = rowOk.load();
    report.noData = rowNoData.load();
    report.failed = rowFailed.load();
    report.elapsed = Clock::now() - rowStart;
    report.pixelsDoneTotal = doneTotal;
    report.pixelsTotal = plan.pixelsTotal;
    sink_.onRowSaved(report);
  }

  RunSummary summary;
  summary.pixelsFetched = fetched.load();
  summary.noData = noData.load();
  summary.failed = failed.load();
  summary.remainingNaN = raster_.countNaN();
  summary.rowsSaved = rowsSaved;
  summary.elapsed = Clock::now() - runStart;
  summary.stopReason = stop_.reason();
  switch (summary.stopReason) {
    case StopReason::ConsecutiveFailures:
      summary.stopMessage =
          "server unreachable: " + std::to_string(options_.maxConsecutiveFailures) + " consecutive failures";
      break;
    case StopReason::WriteError:
      summary.stopMessage = writeError;
      break;
    default:
      summary.stopMessage = describe(summary.stopReason);
      break;
  }
  sink_.onRunFinished(summary);
  return summary;
}

}  // namespace demdl
```

Notes for the engineer: every job writes a distinct raster cell and `save` runs only after `f.get()` has returned for the whole row, so the raster needs no lock. `ctpl::thread_pool` runs queued jobs in FIFO order, which is what makes the stop test deterministic: with two workers blocked on columns 0 and 1, columns 2 and 3 are still queued when the stop arrives and return at their first line. The pool's destructor waits for its queue to drain, which is empty by then.

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[downloader]"`
Expected: `All tests passed (N assertions in 5 test cases)`. The stop test takes well under a second.

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

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "demdl/ElevationClient.h"

using namespace std::chrono_literals;
using demdl::Config;
using demdl::ElevationClient;
using demdl::FetchResult;
using demdl::RetryPolicy;

namespace {

RetryPolicy fastPolicy() {
  RetryPolicy p;
  p.maxRetries = 2;
  p.baseDelay = 1ms;
  p.maxDelay = 2ms;
  p.jitter = 0.0;
  return p;
}

Config configFor(const std::string& wmsUrl) {
  Config c;
  c.box = {-34.0, -34.3, 18.5, 18.1};
  c.width = 4;
  c.height = 1;
  c.output = "unused.tif";
  c.wmsUrl = wmsUrl;
  return c;
}

// Serves canned GetFeatureInfo bodies on loopback. The pixel column I selects the behaviour.
struct CannedServer {
  httplib::Server server;
  std::thread thread;
  std::atomic<int> hitsForColumn2{0};
  int port = 0;

  CannedServer() {
    server.Get("/wms", [this](const httplib::Request& req, httplib::Response& res) {
      const std::string column = req.get_param_value("I");
      if (column == "0") {
        res.set_content("GetFeatureInfo results:\nBAND 0: 12.5\n", "text/plain");
      } else if (column == "1") {
        res.set_content("GetFeatureInfo results:\n", "text/plain");
      } else if (column == "2") {
        if (hitsForColumn2++ == 0) {
          res.status = 500;
        } else {
          res.set_content("BAND 0: 7\n", "text/plain");
        }
      } else {
        res.status = 404;
      }
    });
    port = server.bind_to_any_port("127.0.0.1");
    thread = std::thread([this] { server.listen_after_bind(); });
    server.wait_until_ready();
  }
  ~CannedServer() {
    server.stop();
    thread.join();
  }
  std::string url() const { return "http://127.0.0.1:" + std::to_string(port) + "/wms"; }
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

TEST_CASE("a 200 without a band is NoData", "[elevationclient]") {
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
  CHECK(srv.hitsForColumn2.load() == 2);
}

TEST_CASE("a status that never recovers fails after the retries", "[elevationclient]") {
  CannedServer srv;
  ElevationClient client(configFor(srv.url()), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(3, 0, stop);  // 404 every time
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error == "http 404");
}

TEST_CASE("a refused connection fails after the retries with a transport error", "[elevationclient]") {
  // Port 1 is privileged and never has a listener on a developer machine.
  ElevationClient client(configFor("http://127.0.0.1:1/wms"), fastPolicy());
  std::atomic<bool> stop{false};
  const FetchResult r = client.fetch(0, 0, stop);
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.attempts == 3);
  CHECK(r.error.rfind("transport: ", 0) == 0);
}

TEST_CASE("a stop request cuts the backoff short", "[elevationclient]") {
  RetryPolicy slow;
  slow.maxRetries = 5;
  slow.baseDelay = 10s;
  slow.maxDelay = 10s;
  slow.jitter = 0.0;
  ElevationClient client(configFor("http://127.0.0.1:1/wms"), slow);
  std::atomic<bool> stop{false};
  std::thread stopper([&stop] {
    std::this_thread::sleep_for(150ms);
    stop = true;
  });
  const auto start = std::chrono::steady_clock::now();
  const FetchResult r = client.fetch(0, 0, stop);
  stopper.join();
  CHECK(r.kind == FetchResult::Kind::Failed);
  CHECK(r.error.rfind("stopped during backoff", 0) == 0);
  CHECK(std::chrono::steady_clock::now() - start < 5s);
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
#include <chrono>
#include <memory>
#include <random>
#include <string>

#include "demdl/Config.h"
#include "demdl/PixelFetcher.h"
#include "demdl/RetryPolicy.h"

namespace httplib {
class Client;
}

namespace demdl {

// Fetches one pixel's elevation over HTTP or HTTPS with keep-alive, retries and backoff.
// One instance per worker thread; instances are not thread-safe.
class ElevationClient : public PixelFetcher {
 public:
  // Throws std::invalid_argument when cfg.wmsUrl does not parse.
  ElevationClient(const Config& cfg, RetryPolicy policy);
  ~ElevationClient() override;

  FetchResult fetch(int x, int y, const std::atomic<bool>& stop) override;

 private:
  // Sleeps in 100 ms slices; returns false as soon as stop is set.
  static bool sleepUnlessStopped(std::chrono::milliseconds delay, const std::atomic<bool>& stop);
  // Writes the first unparseable body of the process to stderr, once, across all instances.
  static void logUnparseableBodyOnce(const std::string& body);

  Config cfg_;
  RetryPolicy policy_;
  std::unique_ptr<httplib::Client> client_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
};

}  // namespace demdl
```

- [ ] **Step 5: Write `src/demdl/ElevationClient.cpp`**

```cpp
#include "demdl/ElevationClient.h"

#include <httplib.h>

#include <algorithm>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "demdl/WmsQuery.h"

namespace demdl {

ElevationClient::ElevationClient(const Config& cfg, RetryPolicy policy)
    : cfg_(cfg), policy_(policy), rng_(std::random_device{}()) {
  const auto endpoint = parseWmsUrl(cfg_.wmsUrl);
  if (!endpoint) throw std::invalid_argument("invalid WMS URL: " + cfg_.wmsUrl);
  client_ = std::make_unique<httplib::Client>(endpoint->origin);
  client_->set_keep_alive(true);
  client_->set_connection_timeout(10, 0);
  client_->set_read_timeout(30, 0);
  client_->set_write_timeout(10, 0);
  client_->enable_server_certificate_verification(!cfg_.insecure);
  if (!cfg_.caBundle.empty()) client_->set_ca_cert_path(cfg_.caBundle);
}

ElevationClient::~ElevationClient() = default;

FetchResult ElevationClient::fetch(int x, int y, const std::atomic<bool>& stop) {
  const std::string target = featureInfoUrl(cfg_, x, y);
  FetchResult result;
  for (int attempt = 0;; ++attempt) {
    result.attempts = attempt + 1;
    std::string error;
    const httplib::Result response = client_->Get(target);
    if (!response) {
      error = "transport: " + httplib::to_string(response.error());
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
    if (attempt >= policy_.maxRetries || stop.load()) {
      result.kind = FetchResult::Kind::Failed;
      result.error = error;
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

bool ElevationClient::sleepUnlessStopped(std::chrono::milliseconds delay, const std::atomic<bool>& stop) {
  std::chrono::milliseconds remaining = delay;
  while (remaining.count() > 0) {
    if (stop.load()) return false;
    const std::chrono::milliseconds slice = std::min(remaining, std::chrono::milliseconds(100));
    std::this_thread::sleep_for(slice);
    remaining -= slice;
  }
  return !stop.load();
}

void ElevationClient::logUnparseableBodyOnce(const std::string& body) {
  static std::once_flag flag;
  std::call_once(flag, [&body] {
    std::cerr << "warning: first unparseable response body (truncated): " << body.substr(0, 200) << "\n";
  });
}

}  // namespace demdl
```

Notes for the engineer: `httplib::Client` built from an `https://` origin is the TLS client; the OpenSSL define comes from the `httplib::httplib` target, which is why Task 1 links that name. The certificate and CA calls are no-ops for plain `http://`, which the tests use. `httplib::Result` is move-only; initialising a `const` local from the call is fine under guaranteed copy elision.

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[elevationclient]"`
Expected: `All tests passed (N assertions in 6 test cases)`. The stop test takes about 0.2 s; the refused-connection tests are instant.

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

#include <chrono>
#include <sstream>
#include <string>

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

TEST_CASE("formatDuration picks units by magnitude", "[consolereporter]") {
  CHECK(formatDuration(41.2) == "41.2s");
  CHECK(formatDuration(725.0) == "12m05s");
  CHECK(formatDuration(48667.0) == "13h31m");
  CHECK(formatDuration(50520.0) == "14h02m");
}

TEST_CASE("a saved row prints progress, rate and ETA", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRunStarted(RunPlan{3670016, 3670016, 1792, 1792});
  reporter.onRowSaved(RowReport{811, 1792, 2048, 0, 3, std::chrono::duration<double>(41.2), 1247296, 3670016});
  const std::string text = out.str();
  CHECK_THAT(text, ContainsSubstring("starting: 3670016 pixels in 1792 rows"));
  CHECK_THAT(text, ContainsSubstring("row 812/1792  done 1247296/3670016 (34.0%)  row: 2048 ok 0 nodata 3 failed 41.2s  "
                                     "49.8 px/s  eta 13h31m"));
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
  CHECK_THAT(out.str(), ContainsSubstring("done: 3669980 values, 12034 nodata, 3 failed, 1792 rows saved, 14h02m elapsed"));
  CHECK_THAT(out.str(), ContainsSubstring("rerun with the same arguments to retry the 3 failed pixels"));
  CHECK_THAT(out.str(), !ContainsSubstring("stopped:"));
}

TEST_CASE("a stopped run names the reason", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  RunSummary s;
  s.stopReason = StopReason::Signal;
  s.stopMessage = "interrupted";
  reporter.onRunFinished(s);
  CHECK_THAT(out.str(), ContainsSubstring("stopped: interrupted"));
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
  CHECK(out.str() == "x=12 y=812 123.4 m (1 attempt)\nx=13 y=812 nodata\nx=14 y=812 failed: http 500\n");
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

#include <deque>
#include <iosfwd>
#include <mutex>
#include <string>

#include "demdl/ProgressSink.h"

namespace demdl {

// "41.2s" under a minute, "12m05s" under an hour, otherwise "13h31m".
std::string formatDuration(double seconds);

// Headless progress: one line per saved row, optional per-pixel lines, a final summary.
class ConsoleReporter : public ProgressSink {
 public:
  ConsoleReporter(std::ostream& out, bool verbose);

  void onRunStarted(const RunPlan& plan) override;
  void onPixelState(const PixelEvent& event) override;
  void onRowSaved(const RowReport& report) override;
  void onRunFinished(const RunSummary& summary) override;

 private:
  std::ostream& out_;
  bool verbose_;
  std::mutex mutex_;
  std::deque<RowReport> window_;  // the last ten rows, for rate and ETA
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
  char buf[48];
  if (seconds < 60.0) {
    std::snprintf(buf, sizeof buf, "%.1fs", seconds);
    return buf;
  }
  const long total = std::lround(seconds);
  if (total < 3600) {
    std::snprintf(buf, sizeof buf, "%ldm%02lds", total / 60, total % 60);
    return buf;
  }
  std::snprintf(buf, sizeof buf, "%ldh%02ldm", total / 3600, (total % 3600) / 60);
  return buf;
}

ConsoleReporter::ConsoleReporter(std::ostream& out, bool verbose) : out_(out), verbose_(verbose) {}

void ConsoleReporter::onRunStarted(const RunPlan& plan) {
  std::lock_guard<std::mutex> lock(mutex_);
  window_.clear();
  out_ << "starting: " << plan.pixelsTodo << " pixels in " << plan.rowsTodo << " rows remaining of "
       << plan.pixelsTotal << " pixels in " << plan.rowsTotal << " rows\n";
}

void ConsoleReporter::onPixelState(const PixelEvent& e) {
  if (!verbose_) return;
  std::lock_guard<std::mutex> lock(mutex_);
  switch (e.state) {
    case PixelState::Value: {
      char value[32];
      std::snprintf(value, sizeof value, "%.1f", static_cast<double>(e.value));
      out_ << "x=" << e.x << " y=" << e.y << " " << value << " m (" << e.attempts
           << (e.attempts == 1 ? " attempt)" : " attempts)") << "\n";
      break;
    }
    case PixelState::NoData:
      out_ << "x=" << e.x << " y=" << e.y << " nodata\n";
      break;
    case PixelState::Failed:
      out_ << "x=" << e.x << " y=" << e.y << " failed: " << e.error << "\n";
      break;
    case PixelState::Queued:
    case PixelState::InFlight:
      break;
  }
}

void ConsoleReporter::onRowSaved(const RowReport& r) {
  std::lock_guard<std::mutex> lock(mutex_);
  window_.push_back(r);
  if (window_.size() > 10) window_.pop_front();
  double attempted = 0.0;
  double seconds = 0.0;
  for (const RowReport& w : window_) {
    attempted += static_cast<double>(w.ok + w.noData + w.failed);
    seconds += w.elapsed.count();
  }
  const double rate = seconds > 0.0 ? attempted / seconds : 0.0;
  const double percent =
      r.pixelsTotal > 0 ? 100.0 * static_cast<double>(r.pixelsDoneTotal) / static_cast<double>(r.pixelsTotal) : 100.0;
  const std::uint64_t remaining = r.pixelsTotal > r.pixelsDoneTotal ? r.pixelsTotal - r.pixelsDoneTotal : 0;
  const std::string eta = rate > 0.0 ? formatDuration(static_cast<double>(remaining) / rate) : std::string("--");
  const std::string rowTime = formatDuration(r.elapsed.count());
  char line[256];
  std::snprintf(line, sizeof line,
                "row %d/%d  done %llu/%llu (%.1f%%)  row: %d ok %d nodata %d failed %s  %.1f px/s  eta %s", r.row + 1,
                r.rowsTotal, static_cast<unsigned long long>(r.pixelsDoneTotal),
                static_cast<unsigned long long>(r.pixelsTotal), percent, r.ok, r.noData, r.failed, rowTime.c_str(),
                rate, eta.c_str());
  out_ << line << "\n";
}

void ConsoleReporter::onRunFinished(const RunSummary& s) {
  std::lock_guard<std::mutex> lock(mutex_);
  out_ << "done: " << s.pixelsFetched << " values, " << s.noData << " nodata, " << s.failed << " failed, "
       << s.rowsSaved << " rows saved, " << formatDuration(s.elapsed.count()) << " elapsed\n";
  if (s.stopReason != StopReason::None) out_ << "stopped: " << s.stopMessage << "\n";
  if (s.failed > 0) out_ << "rerun with the same arguments to retry the " << s.failed << " failed pixels\n";
}

}  // namespace demdl
```

- [ ] **Step 6: Build and run the tests**

Run: `cmake --build build && ./build/demdl_tests "[consolereporter]"`
Expected: `All tests passed (N assertions in 5 test cases)`.

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
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <sstream>
#include <string>
#include <vector>

#include "app/Cli.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using demdl::Config;
using demdl::cli::ParseOutcome;
using demdl::cli::parseArgs;

namespace {

struct Parsed {
  Config config;
  ParseOutcome outcome;
  std::string out;
  std::string err;
};

Parsed parse(std::vector<const char*> argv) {
  argv.insert(argv.begin(), "dl_elevation");
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

}  // namespace

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

TEST_CASE("a missing required flag is a usage error", "[cli]") {
  const std::vector<const char*> argv(kExample.begin() + 2, kExample.end());  // drop --north and its value
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--north"));
}

TEST_CASE("--help prints usage and exits successfully", "[cli]") {
  const Parsed p = parse({"--help"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("--wms-url"));
  CHECK_THAT(p.out, ContainsSubstring("--max-consecutive-failures"));
}

TEST_CASE("--version prints the project version", "[cli]") {
  const Parsed p = parse({"--version"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring(DL_ELEVATION_VERSION));
}

TEST_CASE("network, TLS and mode flags set their fields", "[cli]") {
  std::vector<const char*> argv = kExample;
  for (const char* a : {"--threads", "3", "--max-retries", "0", "--max-consecutive-failures", "5", "--insecure",
                        "--ca-bundle", "roots.pem", "--verbose", "--dry-run", "--overwrite", "--layer", "Other",
                        "--wms-url", "http://127.0.0.1:8080/wms"}) {
    argv.push_back(a);
  }
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.threads == 3);
  CHECK(p.config.maxRetries == 0);
  CHECK(p.config.maxConsecutiveFailures == 5);
  CHECK(p.config.insecure);
  CHECK(p.config.caBundle == "roots.pem");
  CHECK(p.config.verbose);
  CHECK(p.config.dryRun);
  CHECK(p.config.overwrite);
  CHECK(p.config.layer == "Other");
  CHECK(p.config.wmsUrl == "http://127.0.0.1:8080/wms");
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
# Cli.cpp is compiled once and shared by the executable and the tests.
add_library(demdl_cli OBJECT src/app/Cli.cpp)
target_link_libraries(demdl_cli PUBLIC demdl CLI11::CLI11)
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

- [ ] **Step 3: Build to verify it fails**

Run: `cmake --build build`
Expected: FAIL with `Cannot find source file: src/app/Cli.cpp`.

- [ ] **Step 4: Write `src/app/Cli.h`**

```cpp
#pragma once

#include <iosfwd>

#include "demdl/Config.h"

namespace demdl::cli {

struct ParseOutcome {
  bool exitNow = false;  // true after --help, --version or a usage error
  int exitCode = 0;      // 0 for help and version, 1 for a usage error
};

// Fills `config` from argv with CLI11. Help, version and error text go to the given streams.
// Does not call Config::validate(); the caller does.
ParseOutcome parseArgs(int argc, const char* const* argv, Config& config, std::ostream& out, std::ostream& err);

}  // namespace demdl::cli
```

- [ ] **Step 5: Write `src/app/Cli.cpp`**

```cpp
#include "app/Cli.h"

#include <CLI/CLI.hpp>

#include <ostream>
#include <string>

namespace demdl::cli {

ParseOutcome parseArgs(int argc, const char* const* argv, Config& config, std::ostream& out, std::ostream& err) {
  CLI::App app{"Download a digital elevation model from a WMS server one pixel at a time into a GeoTIFF"};
  app.set_version_flag("--version", std::string(DL_ELEVATION_VERSION));

  std::string output;
  app.add_option("--north", config.box.north, "northern edge, WGS84 decimal degrees")->required();
  app.add_option("--south", config.box.south, "southern edge, WGS84 decimal degrees")->required();
  app.add_option("--east", config.box.east, "eastern edge, WGS84 decimal degrees")->required();
  app.add_option("--west", config.box.west, "western edge, WGS84 decimal degrees")->required();
  app.add_option("--width", config.width, "grid width in pixels")->required();
  app.add_option("--height", config.height, "grid height in pixels")->required();
  app.add_option("-o,--output", output, "GeoTIFF to create or resume")->required();
  app.add_option("--wms-url", config.wmsUrl, "WMS endpoint: scheme, host and path")->capture_default_str();
  app.add_option("--layer", config.layer, "WMS layer name")->capture_default_str();
  app.add_option("--threads", config.threads, "worker threads, one keep-alive connection each")
      ->capture_default_str();
  app.add_option("--max-retries", config.maxRetries, "retries per pixel after the first attempt")
      ->capture_default_str();
  app.add_option("--max-consecutive-failures", config.maxConsecutiveFailures,
                 "stop after this many pixels fail in a row")
      ->capture_default_str();
  app.add_option("--ca-bundle", config.caBundle, "PEM bundle for certificate verification");
  app.add_flag("--insecure", config.insecure, "disable certificate verification");
  app.add_flag("--preview", config.preview, "open the OpenGL preview window");
  app.add_flag("--overwrite", config.overwrite, "ignore an existing output and start fresh");
  app.add_flag("--dry-run", config.dryRun, "validate and report the plan without making requests");
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
Expected: `All tests passed (N assertions in 6 test cases)`.

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
