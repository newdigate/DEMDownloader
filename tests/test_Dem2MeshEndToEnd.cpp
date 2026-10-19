#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <optional>
#include <string>
#include <string_view>
#include <system_error>
#include <vector>

#include "FakeBlender.h"
#include "GeoTiffFixtures.h"
#include "SpawnedProcess.h"
#include "TempDir.h"
#include "demdl/BoundingBox.h"
#include "demdl/DemRaster.h"
#include "mesh/BlenderRunner.h"
#include "mesh/CropWindow.h"
#include "mesh/GeoTiffGrid.h"
#include "mesh/PlyWriter.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"
#include "mesh/TransverseMercator.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::EndsWith;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;
using demdl::mesh::checkSolid;
using demdl::mesh::extentInMetres;
using demdl::mesh::findBlender;
using demdl::mesh::GridGeometry;
using demdl::mesh::inspectGeoTiff;
using demdl::mesh::Mesh;
using demdl::mesh::pixelWindowFor;
using demdl::mesh::plyFileBytes;
using demdl::mesh::readBinaryStl;
using demdl::mesh::solidTriangleCount;
using demdl::mesh::stlFileBytes;
using demdl::mesh::WindowExtent;

namespace {

using Path = std::filesystem::path;

// The dem2mesh built by this CMake tree. These tests give it files in their own TempDir only.
const char* const kExecutable = DEM2MESH_EXE;

// The design's Simon's Town box, which dl_elevation would download as an 8 x 6 grid of pixels 0.00875 degrees across
// and 0.04 / 6 down, sampled at their north-west corners.
const demdl::BoundingBox kBox{-34.18, -34.22, 18.47, 18.40};
constexpr int kCols = 8;
constexpr int kRows = 6;

// 10 + 5 i + 3 j at column i and row j, row 0 north: a plane rising to the east and the south, from 10 m at the
// north-west corner to its summit of 60 m at the south-east one, except for dl_elevation's no-data at (7, 0) and
// (7, 1), in the north-east corner and away from the summit. The fill gives those the heights of their nearest pixels:
// 40 from (6, 0), and 43 from (6, 1), the first in scan order of its two nearest.
std::vector<double> fixtureHeights() {
  std::vector<double> heights;
  for (int j = 0; j < kRows; ++j) {
    for (int i = 0; i < kCols; ++i) heights.push_back(10.0 + 5.0 * i + 3.0 * j);
  }
  heights[7] = demdl::DemRaster::kNoData;
  heights[kCols + 7] = demdl::DemRaster::kNoData;
  return heights;
}

// A run with the real Blender: its start-up alone takes six seconds on the owner's machine, and more under a parallel
// ctest, which kDeadline is too short for.
constexpr std::chrono::seconds kBlenderDeadline{120};

// dem2mesh started with args and run to its end, its stdout and stderr captured in files in dir.
Finished runDem2Mesh(const std::vector<std::string>& args, const Path& dir,
                     std::chrono::seconds limit = kDeadline) {
  Child child(kExecutable, args, dir);
  return child.wait(limit);
}

// The 8 x 6 fixture in a TempDir, and an empty directory out/ for the outputs, so that "nothing written" is "out/ is
// still empty".
struct Workspace {
  TempDir dir;
  Path input = writeGeographicFixture(dir.path, kCols, kRows, kBox, fixtureHeights());
  Path outDir = dir.path / "out";
  Path stem = outDir / "model";

  Workspace() { REQUIRE(std::filesystem::create_directory(outDir)); }

  [[nodiscard]] Path stl() const { return stem.string() + ".stl"; }
  [[nodiscard]] Path ply() const { return stem.string() + ".ply"; }
  [[nodiscard]] Path blend() const { return stem.string() + ".blend"; }

  // dem2mesh on the fixture with the outputs at stem and without Blender, then `extra`.
  [[nodiscard]] Finished run(const std::vector<std::string>& extra) const {
    std::vector<std::string> args{"-i", input.string(), "-o", stem.string(), "--no-blend"};
    args.insert(args.end(), extra.begin(), extra.end());
    return runDem2Mesh(args, dir.path);
  }

  // The same with Blender, as a run goes without --no-blend.
  [[nodiscard]] Finished runWithBlender(const std::vector<std::string>& extra) const {
    std::vector<std::string> args{"-i", input.string(), "-o", stem.string()};
    args.insert(args.end(), extra.begin(), extra.end());
    return runDem2Mesh(args, dir.path, kBlenderDeadline);
  }
};

// The names of the entries in dir, sorted.
std::vector<std::string> filesIn(const Path& dir) {
  std::vector<std::string> names;
  for (const auto& entry : std::filesystem::directory_iterator(dir)) names.push_back(entry.path().filename().string());
  std::sort(names.begin(), names.end());
  return names;
}

// The window of the whole file in metres, as dem2mesh works it out.
WindowExtent wholeFileExtent(const Path& input) {
  const GridGeometry geometry = inspectGeoTiff(input);
  return extentInMetres(geometry, pixelWindowFor(geometry, std::nullopt));
}

struct Bounds {
  double minX = std::numeric_limits<double>::infinity(), maxX = -std::numeric_limits<double>::infinity();
  double minY = std::numeric_limits<double>::infinity(), maxY = -std::numeric_limits<double>::infinity();
  double minZ = std::numeric_limits<double>::infinity(), maxZ = -std::numeric_limits<double>::infinity();
};

Bounds boundsOf(const Mesh& mesh) {
  Bounds b;
  for (const auto& v : mesh.vertices) {
    b.minX = std::min(b.minX, double{v.x});
    b.maxX = std::max(b.maxX, double{v.x});
    b.minY = std::min(b.minY, double{v.y});
    b.maxY = std::max(b.maxY, double{v.y});
    b.minZ = std::min(b.minZ, double{v.z});
    b.maxZ = std::max(b.maxZ, double{v.z});
  }
  return b;
}

// The terrain's height at the corner (x, y) of the solid: the higher of the two vertices there, a wall's top and the
// bottom corner under it. NaN when the solid has no vertex there.
double topAt(const Mesh& mesh, double x, double y) {
  double top = std::numeric_limits<double>::quiet_NaN();
  for (const auto& v : mesh.vertices) {
    if (std::abs(v.x - x) < 0.01 && std::abs(v.y - y) < 0.01 && !(v.z <= top)) top = v.z;
  }
  return top;
}

// What follows `label` on the first line of `text` that starts with it, indentation aside, such as "-1 m (...)" for
// "base: " in "base: -1 m (...)". Fails the test when no line does.
std::string lineAfter(const std::string& text, std::string_view label) {
  for (std::size_t start = 0; start < text.size();) {
    const std::size_t newline = text.find('\n', start);
    const std::size_t end = newline == std::string::npos ? text.size() : newline;
    const std::string_view line(text.data() + start, end - start);
    const std::size_t first = line.find_first_not_of(' ');
    if (first != std::string_view::npos && line.substr(first).starts_with(label)) {
      return std::string(line.substr(first + label.size()));
    }
    start = end + 1;
  }
  FAIL("no line starts with '" << label << "'");
  return {};
}

// The base plane in metres from the line "base: <metres> m (<why>)".
double reportedBase(const std::string& out) {
  const std::string rest = lineAfter(out, "base: ");
  char* end = nullptr;
  const double base = std::strtod(rest.c_str(), &end);
  REQUIRE(end != rest.c_str());
  REQUIRE(std::string_view(end).starts_with(" m ("));
  return base;
}

// The window's width in pixels from the line "window: <cols> x <rows> pixels at ...".
int reportedWindowCols(const std::string& out) {
  const std::string rest = lineAfter(out, "window: ");
  int cols = -1;
  const auto [end, error] = std::from_chars(rest.data(), rest.data() + rest.size(), cols);
  REQUIRE(error == std::errc{});
  REQUIRE(std::string_view(end, static_cast<std::size_t>(rest.data() + rest.size() - end)).starts_with(" x "));
  return cols;
}

}  // namespace

TEST_CASE("a dry run reads the window, prints the plan from its first line on and writes nothing", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--dry-run"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK_THAT(result.out, StartsWith("dry run\n"));
  CHECK_THAT(result.out, ContainsSubstring("grid: 8 x 6 vertices"));
  CHECK_THAT(result.out, ContainsSubstring("2 no-data pixels filled"));
  CHECK_THAT(result.out, ContainsSubstring("metres above sea level"));
  // What a run would write, without writing it: 8 x 6 vertices, 70 surface triangles and 100 in the solid.
  CHECK_THAT(result.out, ContainsSubstring("solid: 100 triangles, STL " + std::to_string(stlFileBytes(100)) +
                                           " bytes, PLY " + std::to_string(plyFileBytes(48, 70)) + " bytes"));
  CHECK_THAT(result.out, ContainsSubstring("blender: skipped"));
  CHECK(std::filesystem::is_empty(w.outDir));
}

TEST_CASE("a full-resolution run writes the closed solid over the window and the surface beside it", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK(std::filesystem::exists(w.stl()));
  CHECK(std::filesystem::exists(w.ply()));
  CHECK_FALSE(std::filesystem::exists(w.stl().string() + ".tmp"));
  CHECK_FALSE(std::filesystem::exists(w.ply().string() + ".tmp"));

  const Mesh solid = readBinaryStl(w.stl());
  CHECK(solidTriangleCount(kCols, kRows) == 2 * 7 * 5 + 2 * 14 + 2);
  CHECK(solid.triangles.size() == solidTriangleCount(kCols, kRows));
  CHECK(checkSolid(solid).ok());

  // The mesh spans the window from its first sample to its last, south-west corner at the origin. The heights are a
  // plane, which the resampler reproduces, and its corners exactly, so the summit is the south-east corner's 60 m.
  const WindowExtent extent = wholeFileExtent(w.input);
  const Bounds b = boundsOf(solid);
  CHECK_THAT(b.minX, WithinAbs(0.0, 0.01));
  CHECK_THAT(b.maxX, WithinAbs(extent.widthM, 0.01));
  CHECK_THAT(b.minY, WithinAbs(0.0, 0.01));
  CHECK_THAT(b.maxY, WithinAbs(extent.heightM, 0.01));
  CHECK_THAT(b.maxZ, WithinAbs(60.0, 0.01));
  CHECK_THAT(b.minZ, WithinAbs(reportedBase(result.out), 0.01));
  // Each corner where it belongs, not mirrored: north-west 10 m, south-west 10 + 3 x 5 = 25 m, south-east 60 m, and
  // north-east the 40 m the fill gave it.
  CHECK_THAT(topAt(solid, 0.0, extent.heightM), WithinAbs(10.0, 0.01));
  CHECK_THAT(topAt(solid, 0.0, 0.0), WithinAbs(25.0, 0.01));
  CHECK_THAT(topAt(solid, extent.widthM, 0.0), WithinAbs(60.0, 0.01));
  CHECK_THAT(topAt(solid, extent.widthM, extent.heightM), WithinAbs(40.0, 0.01));

  CHECK_THAT(result.out, ContainsSubstring("input: " + w.input.string() + ", geographic, 8 x 6 samples at "));
  CHECK(result.out.find("crop: ") == std::string::npos);  // none was asked for
  CHECK_THAT(result.out, ContainsSubstring("\nheights: 10 to 60 m (2 no-data pixels filled with 40 to 43 m)\n"));
  CHECK_THAT(result.out, ContainsSubstring("\nsummit: 60 m in the window, 60 m in the mesh\n"));
  CHECK_THAT(result.out, ContainsSubstring("\nbase: -1 m (sea level minus a 1 m margin)\n"));
  // The sizes the summary gives are those of the files written.
  CHECK_THAT(result.out, ContainsSubstring("\nsolid: 100 triangles, STL " +
                                           std::to_string(std::filesystem::file_size(w.stl())) + " bytes, PLY " +
                                           std::to_string(std::filesystem::file_size(w.ply())) + " bytes\n"));
  CHECK(occurrences(result.out, "wrote: ") == 2);
  CHECK_THAT(result.out, ContainsSubstring("\nwrote: " + w.stl().string() + "\nwrote: " + w.ply().string() + "\n"));
  CHECK_THAT(result.out, ContainsSubstring("blender: skipped"));

  // Some readers take a file whose header begins "solid" for an ASCII STL, whatever follows.
  const std::string header = readFile(w.stl()).substr(0, 80);
  CHECK_FALSE(header.starts_with("solid"));
  CHECK_THAT(header, StartsWith("dem2mesh "));
  CHECK(std::filesystem::file_size(w.ply()) == plyFileBytes(48, 70));
}

TEST_CASE("a grid of 4 takes its height from the window's aspect", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--grid", "4"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  // The window is about 5646 m by 3698 m, so 3 cells across take round(3 x 3698 / 5646) = 2 down.
  CHECK_THAT(result.out, ContainsSubstring("grid: 4 x 3 vertices"));
  const Mesh solid = readBinaryStl(w.stl());
  CHECK(solid.triangles.size() == 2 * 3 * 2 + 2 * 7 + 2);
  CHECK(checkSolid(solid).ok());
}

TEST_CASE("scale and z-scale size the model, and the plinth keeps a scaled-down one thick enough", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--grid", "4x4", "--scale", "0.01", "--z-scale", "2"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  const WindowExtent extent = wholeFileExtent(w.input);
  const Bounds b = boundsOf(readBinaryStl(w.stl()));
  CHECK_THAT(b.maxX - b.minX, WithinAbs(0.01 * extent.widthM, 1e-4));
  CHECK_THAT(b.maxY - b.minY, WithinAbs(0.01 * extent.heightM, 1e-4));
  const double base = reportedBase(result.out);
  CHECK(b.maxZ - b.minZ >= 2 * 0.01 * (60 - base) - 1e-4);
  // Scaled, the 1 m margin puts the base 0.02 units under the lowest point, at 10 m x 2 x 0.01 = 0.2 units: 0.22 units
  // of solid. So the base goes down to leave the 2-unit plinth, to -1.8 units, which is -90 m before scaling.
  CHECK_THAT(result.out, ContainsSubstring(", then pushed down for a 2-unit plinth)"));
  CHECK_THAT(b.minZ, WithinAbs(10 * 2 * 0.01 - 2, 1e-4));
  CHECK_THAT(base, WithinAbs(-90.0, 0.01));
}

TEST_CASE("a crop narrows the window, and one beside the file is refused", "[mesh-e2e]") {
  const Workspace w;
  SECTION("inside the file") {
    const Finished result = w.run({"--north", "-34.19", "--south", "-34.21", "--east", "18.45", "--west", "18.42"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    REQUIRE(result.exitCode() == 0);
    const int cols = reportedWindowCols(result.out);
    CHECK(cols >= 2);
    CHECK(cols < kCols);
    // Columns 2 to 6 and rows 1 to 5 hold the box. Neither no-data pixel is among them, so nothing was filled.
    CHECK_THAT(result.out, ContainsSubstring("window: 5 x 5 pixels at (2, 1), "));
    // The crop asked for, echoed in degrees between the input and the window, as the dry run echoes it.
    CHECK_THAT(result.out, ContainsSubstring(" degrees\ncrop: north -34.19, south -34.21, east 18.45, west 18.42 "
                                             "degrees\nwindow: 5 x 5 pixels at (2, 1), "));
    CHECK_THAT(result.out, StartsWith("input: "));
    CHECK(result.out.find("filled") == std::string::npos);
    CHECK(readBinaryStl(w.stl()).triangles.size() == solidTriangleCount(5, 5));
  }
  SECTION("north of the file") {
    const Finished result = w.run({"--north", "-30", "--south", "-31", "--east", "18.45", "--west", "18.42"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK_THAT(result.err,
               ContainsSubstring("error: the crop window overlaps the file by fewer than 2 x 2 samples\n"));
    CHECK(std::filesystem::is_empty(w.outDir));
  }
}

TEST_CASE("existing outputs are refused without --overwrite and replaced with it", "[mesh-e2e]") {
  const Workspace w;
  SECTION("an STL and a PLY from an earlier run") {
    REQUIRE(w.run({}).exitCode() == 0);
    const std::string stl = readFile(w.stl());
    const std::string ply = readFile(w.ply());
    {
      const Finished refused = w.run({});
      INFO("stdout:\n" << refused.out << "stderr:\n" << refused.err);
      CHECK(refused.exitCode() == 1);
      CHECK_THAT(refused.err, ContainsSubstring("error: " + w.stl().string() + " exists; use --overwrite"));
      CHECK(fileHolds(w.stl(), stl));
      CHECK(fileHolds(w.ply(), ply));
    }
    const Finished replaced = w.run({"--overwrite", "--grid", "4"});
    INFO("stdout:\n" << replaced.out << "stderr:\n" << replaced.err);
    CHECK(replaced.exitCode() == 0);
    CHECK(readBinaryStl(w.stl()).triangles.size() == solidTriangleCount(4, 3));  // the new grid's
    CHECK_FALSE(fileHolds(w.ply(), ply));
  }
  SECTION("a Blender file alone") {
    const Path blend = w.stem.string() + ".blend";
    writeFile(blend, "not a Blender file");
    const Finished refused = w.run({});
    INFO("stdout:\n" << refused.out << "stderr:\n" << refused.err);
    CHECK(refused.exitCode() == 1);
    CHECK_THAT(refused.err, ContainsSubstring("error: " + blend.string() + " exists; use --overwrite"));
    CHECK_FALSE(std::filesystem::exists(w.stl()));
    CHECK_FALSE(std::filesystem::exists(w.ply()));
  }
}

TEST_CASE("a stem in a missing directory is refused with the directory named, and nothing is created", "[mesh-e2e]") {
  const Workspace w;
  const Path missing = w.dir.path / "nodir";
  const Finished result =
      runDem2Mesh({"-i", w.input.string(), "-o", (missing / "model").string(), "--no-blend"}, w.dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK_THAT(result.err, ContainsSubstring("cannot write"));
  CHECK_THAT(result.err, ContainsSubstring(missing.string()));
  CHECK_FALSE(std::filesystem::exists(missing));
}

TEST_CASE("a grid of more than 25,000,000 vertices is refused and nothing is written", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--grid", "6000x5000"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK_THAT(result.err, ContainsSubstring("error: the output grid is 30000000 vertices; the limit is 25000000: use "
                                           "--grid or crop\n"));
  CHECK(std::filesystem::is_empty(w.outDir));
}

TEST_CASE("a Transverse Mercator point file meshes at full resolution, its window in metres", "[mesh-e2e]") {
  const TempDir dir;
  demdl::mesh::TransverseMercatorParams lo19;
  lo19.originLongitudeDeg = 19.0;
  std::vector<double> heights;
  for (int j = 0; j < 10; ++j) {
    for (int i = 0; i < 12; ++i) heights.push_back(100.0 + 2.0 * i + j);
  }
  // The City of Cape Town's corner and 5 m spacing, as in test_GeoTiffGrid.cpp; its no-data, 0, is no height here.
  const Path input = writeTmFixture(dir.path, 12, 10, lo19, {0.0, 0.0}, {-64012.5, -3705051.806}, 5.0, 5.0, true, 0.0,
                                    heights, Layout::Strips, SampleKind::Float32);
  const Path stem = dir.path / "tm";
  const Finished result = runDem2Mesh({"-i", input.string(), "-o", stem.string(), "--no-blend"}, dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK_THAT(result.out,
             ContainsSubstring("input: " + input.string() + ", Transverse Mercator, 12 x 10 samples at 5 x 5 m\n"));
  CHECK_THAT(result.out, ContainsSubstring("window: 12 x 10 pixels at (0, 0), samples from easting -64012.50 to "
                                           "-63957.50 m and northing -3705051.81 to -3705096.81 m\n"));
  const Mesh solid = readBinaryStl(stem.string() + ".stl");
  CHECK(solid.triangles.size() == solidTriangleCount(12, 10));
  CHECK(checkSolid(solid).ok());
  const Bounds b = boundsOf(solid);
  CHECK_THAT(b.maxX - b.minX, WithinAbs(11 * 5.0, 1e-4));
  CHECK_THAT(b.maxY - b.minY, WithinAbs(9 * 5.0, 1e-4));
  CHECK_THAT(b.maxZ, WithinAbs(100.0 + 2 * 11 + 9, 1e-3));
}

TEST_CASE("a bad option or input is refused with exit 1 and nothing is written", "[mesh-e2e]") {
  const Workspace w;
  const auto runOn = [&w](const Path& input) {
    return runDem2Mesh({"-i", input.string(), "-o", w.stem.string(), "--no-blend"}, w.dir.path);
  };
  SECTION("an option that does not validate") {
    const Finished result = w.run({"--z-scale", "0"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK_THAT(result.err, ContainsSubstring("error: z-scale must be positive\n"));
  }
  SECTION("an input that does not exist") {
    const Path missing = w.dir.path / "none.tif";
    const Finished result = runOn(missing);
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK_THAT(result.err,
               ContainsSubstring("error: cannot read " + missing.string() + ": No such file or directory\n"));
  }
  SECTION("an input that is a directory") {
    const Finished result = runOn(w.outDir);
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK_THAT(result.err, ContainsSubstring("error: " + w.outDir.string() + " is not a regular file\n"));
  }
  SECTION("an input that is not a GeoTIFF, which the library refuses") {
    const Path junk = w.dir.path / "junk.tif";
    writeFile(junk, "these bytes are not a TIFF\n");
    const Finished result = runOn(junk);
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK_THAT(result.err, StartsWith("error: "));
    CHECK_THAT(result.err, ContainsSubstring(junk.string()));
  }
  CHECK(std::filesystem::is_empty(w.outDir));
}

TEST_CASE("verbose adds a timing line for each stage", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--verbose"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  for (const char* stage : {"read", "fill", "resample", "surface", "solid", "check", "write PLY", "write STL"}) {
    CAPTURE(stage);
    CHECK_THAT(result.out, ContainsSubstring(std::string("time: ") + stage + " "));
  }
  const Finished quiet = w.run({"--overwrite"});
  CHECK(quiet.exitCode() == 0);
  CHECK(quiet.out.find("time: ") == std::string::npos);
}

TEST_CASE("with Blender installed, a run saves the scene and Blender's check of the STL passes", "[mesh-e2e]") {
  const std::optional<Path> blender = findBlender(std::nullopt);
  if (!blender) SKIP("Blender not installed");
  const Workspace w;
  // A print-sized model. The same without Blender, to show that Blender leaves the STL and the PLY as dem2mesh wrote
  // them.
  const std::vector<std::string> model{"--grid", "4x4", "--scale", "0.01"};
  std::vector<std::string> plain{"-i", w.input.string(), "-o", (w.dir.path / "reference").string(), "--no-blend"};
  plain.insert(plain.end(), model.begin(), model.end());
  REQUIRE(runDem2Mesh(plain, w.dir.path).exitCode() == 0);

  std::vector<std::string> args = model;
  args.push_back("--verbose");
  const Finished result = w.runWithBlender(args);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  // Blender 3.0 and later compress a .blend with Zstandard; an uncompressed one begins "BLENDER".
  const std::string scene = readFile(w.blend());
  CHECK((scene.starts_with("BLENDER") || scene.starts_with("\x28\xb5\x2f\xfd")));
  CHECK_THAT(result.out, ContainsSubstring("\nwrote: " + w.stl().string() + "\nwrote: " + w.ply().string() +
                                           "\nblender: " + blender->string() + "\n"));
  // Blender's log, shown with --verbose: a scene of a print-sized model shows a unit as the millimetre it prints as.
  CHECK_THAT(result.out,
             ContainsSubstring("\nblender: units: METRIC, scale_length 0.001, length_unit MILLIMETERS\n"));
  // A 4 x 4 grid's solid: 2 x 3 x 3 surface triangles, 2 x 8 in the walls and 2 in the base.
  CHECK(solidTriangleCount(4, 4) == 36);
  CHECK_THAT(result.out, EndsWith("\nwrote: " + w.blend().string() +
                                  "\nstl check: 36 triangles, 0 non-manifold edges, 0 inverted faces\n"));
  CHECK(result.err.empty());
  CHECK(fileHolds(w.stl(), readFile((w.dir.path / "reference.stl").string())));
  CHECK(fileHolds(w.ply(), readFile((w.dir.path / "reference.ply").string())));
  // Nothing else: no temporary file of Blender's, and no .blend1 (BlenderRunner's test saves over a scene).
  CHECK(filesIn(w.outDir) == std::vector<std::string>{"model.blend", "model.ply", "model.stl"});
}

TEST_CASE("a --blender or BLENDER that names no executable file is refused before anything is read", "[mesh-e2e]") {
  const Workspace w;
  // Refused in the round of checks before the input is opened: nothing is written, and a dry run prints nothing, as
  // its "dry run" line comes after those checks.
  const auto checkRefused = [&w](const Finished& result, const std::string& problem) {
    CHECK(result.exitCode() == 1);
    CHECK(result.err == "error: " + problem + "\n");
    CHECK(result.out.empty());
    CHECK(std::filesystem::is_empty(w.outDir));
  };
  SECTION("a path that names nothing") {
    const Finished result = w.runWithBlender({"--blender", "/nonexistent/blender"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "--blender /nonexistent/blender: does not exist");
  }
  SECTION("the same in a dry run") {
    const Finished result = w.runWithBlender({"--dry-run", "--blender", "/nonexistent/blender"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "--blender /nonexistent/blender: does not exist");
  }
  SECTION("BLENDER naming nothing") {
    const ScopedEnv blender("BLENDER", "/nonexistent/blender");
    const Finished result = w.runWithBlender({});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "BLENDER=/nonexistent/blender: does not exist");
  }
  SECTION("a directory") {
    const Path folder = w.dir.path / "bin";
    REQUIRE(std::filesystem::create_directory(folder));
    const Finished result = w.runWithBlender({"--blender", folder.string()});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "--blender " + folder.string() + ": is not a regular file");
  }
  SECTION("the app bundle, which holds the executable") {
    const Path app = w.dir.path / "Blender.app";
    REQUIRE(std::filesystem::create_directory(app));
    const Finished result = w.runWithBlender({"--blender", app.string()});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "--blender " + app.string() + ": is the app bundle; its executable is " + app.string() +
                             "/Contents/MacOS/Blender");
  }
  SECTION("not with --no-blend, which starts no Blender") {
    const Finished result = w.run({"--blender", "/nonexistent/blender"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 0);
    CHECK_THAT(result.out, EndsWith("\nblender: skipped (--no-blend)\n"));
  }
}

TEST_CASE("a Blender that fails leaves the STL and PLY, shows its output and exits 1", "[mesh-e2e]") {
  const Workspace w;
  const TempDir bin;
  const FakeBlender fake = writeFakeBlender(bin.path);
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  // What each of these runs leaves: the summary down to the Blender it tried, the STL and the PLY, and no scene.
  const auto checkKept = [&w](const Finished& result, const std::string& blenderShown) {
    CHECK(result.exitCode() == 1);
    CHECK_THAT(result.out, EndsWith("\nwrote: " + w.stl().string() + "\nwrote: " + w.ply().string() +
                                    "\nblender: " + blenderShown + "\n"));
    CHECK(readBinaryStl(w.stl()).triangles.size() == solidTriangleCount(kCols, kRows));
    CHECK(std::filesystem::exists(w.ply()));
    CHECK_FALSE(std::filesystem::exists(w.blend()));
  };
  SECTION("a Blender that exits 3, whose output is shown") {
    const ScopedEnv code("FAKE_EXIT", "3");
    const Finished result = w.runWithBlender({"--blender", fake.executable.string()});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkKept(result, fake.executable.string());
    CHECK(result.err == "error: Blender failed (exit 3)\nfake blender: to stdout\nfake blender: to stderr\n");
  }
  SECTION("a Blender that exits 0 without checking the STL") {
    const Finished result = w.runWithBlender({"--blender", fake.executable.string()});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkKept(result, fake.executable.string());
    CHECK(result.err == "error: Blender exited 0 but did not report the STL check\nfake blender: to stdout\n"
                        "fake blender: to stderr\n");
  }
  SECTION("a Blender that exits 0 and reports the check without saving the scene") {
    const ScopedEnv check("FAKE_STL_CHECK", "stl check: 100 triangles, 0 non-manifold edges, 0 inverted faces");
    const ScopedEnv noBlend("FAKE_NO_BLEND", "1");
    const Finished result = w.runWithBlender({"--blender", fake.executable.string()});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkKept(result, fake.executable.string());
    CHECK(result.err == "error: Blender exited 0 but did not write " + w.blend().string() +
                            "\nfake blender: to stdout\nfake blender: to stderr\n"
                            "stl check: 100 triangles, 0 non-manifold edges, 0 inverted faces\n");
  }
  SECTION("an old scene, which goes before Blender starts, so that a Blender that fails leaves none") {
    writeFile(w.blend(), "an old scene");
    const ScopedEnv code("FAKE_EXIT", "1");
    const Finished result = w.runWithBlender({"--blender", fake.executable.string(), "--overwrite"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkKept(result, fake.executable.string());
    CHECK(result.err == "error: Blender failed (exit 1)\nfake blender: to stdout\nfake blender: to stderr\n");
  }
  SECTION("an old scene that cannot be removed, which fails the run before Blender starts") {
    // A directory with something in it, which a removal leaves where it is.
    REQUIRE(std::filesystem::create_directory(w.blend()));
    writeFile(w.blend() / "inside", "");
    const Finished result = w.runWithBlender({"--blender", fake.executable.string(), "--overwrite"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK(result.err == "error: cannot remove " + w.blend().string() + ": Directory not empty\n");
    CHECK_THAT(result.out, EndsWith("\nwrote: " + w.ply().string() + "\nblender: " + fake.executable.string() + "\n"));
    CHECK_FALSE(std::filesystem::exists(fake.arguments));  // never started
  }
}

TEST_CASE("dem2mesh reports the scene and the check Blender made, and shows Blender's log with --verbose",
          "[mesh-e2e]") {
  const Workspace w;
  const TempDir bin;
  const FakeBlender fake = writeFakeBlender(bin.path);
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  const ScopedEnv check("FAKE_STL_CHECK", "stl check: 100 triangles, 0 non-manifold edges, 0 inverted faces");
  const Finished result = w.runWithBlender({"--blender", fake.executable.string()});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK_THAT(result.out, EndsWith("\nwrote: " + w.ply().string() + "\nblender: " + fake.executable.string() +
                                  "\nwrote: " + w.blend().string() +
                                  "\nstl check: 100 triangles, 0 non-manifold edges, 0 inverted faces\n"));
  CHECK(occurrences(result.out, "wrote: ") == 3);
  CHECK(result.out.find("fake blender:") == std::string::npos);  // the log is for --verbose
  CHECK(result.err.empty());

  const Finished verbose = w.runWithBlender({"--blender", fake.executable.string(), "--overwrite", "--verbose"});
  INFO("verbose, stdout:\n" << verbose.out << "stderr:\n" << verbose.err);
  REQUIRE(verbose.exitCode() == 0);
  CHECK_THAT(verbose.out, ContainsSubstring("\nblender: " + fake.executable.string() +
                                            "\nfake blender: to stdout\nfake blender: to stderr\nstl check: 100 "));
  CHECK_THAT(verbose.out, ContainsSubstring("\ntime: blender "));
  CHECK_THAT(verbose.out, EndsWith("\nwrote: " + w.blend().string() +
                                   "\nstl check: 100 triangles, 0 non-manifold edges, 0 inverted faces\n"));
}

TEST_CASE("dem2mesh gives Blender the outputs, the stem's name, and the model's size and heights in model units",
          "[mesh-e2e]") {
  const Workspace w;
  const TempDir bin;
  const FakeBlender fake = writeFakeBlender(bin.path);
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  const ScopedEnv check("FAKE_STL_CHECK", "stl check: 36 triangles, 0 non-manifold edges, 0 inverted faces");
  const Finished result = w.runWithBlender(
      {"--blender", fake.executable.string(), "--grid", "4x4", "--scale", "0.01", "--z-scale", "2"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  const std::vector<std::string> args = fake.recordedArguments();
  REQUIRE(args.size() == 14);
  CHECK(args[5] == "--ply=" + w.ply().string());
  CHECK(args[6] == "--stl=" + w.stl().string());
  CHECK(args[7] == "--blend=" + w.blend().string());
  CHECK(args[8] == "--name=model");
  const WindowExtent extent = wholeFileExtent(w.input);
  CHECK_THAT(numberIn(args[9], "--width"), WithinAbs(0.01 * extent.widthM, 1e-9));
  CHECK_THAT(numberIn(args[10], "--height"), WithinAbs(0.01 * extent.heightM, 1e-9));
  // The surface's lowest and highest heights, 10 m and 60 m at the plane's corners, which the resampler keeps, times
  // the z-scale and the scale. The base the plinth pushed down is not the surface's.
  CHECK_THAT(numberIn(args[11], "--zmin"), WithinAbs(10 * 2 * 0.01, 1e-5));
  CHECK_THAT(numberIn(args[12], "--zmax"), WithinAbs(60 * 2 * 0.01, 1e-5));
  CHECK(numberIn(args[13], "--scale") == 0.01);
}

TEST_CASE("a dry run names the Blender a run would start, the memory it would take, and the scene it would save",
          "[mesh-e2e]") {
  const Workspace w;
  const TempDir bin;
  const FakeBlender fake = writeFakeBlender(bin.path);
  const Finished result = w.runWithBlender({"--dry-run", "--blender", fake.executable.string()});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK_THAT(result.out, ContainsSubstring("\n  output: " + w.blend().string() + " (will be created)\n"));
  // 250 MB and 670 bytes for each of the solid's 100 triangles: 0.250067 GB.
  CHECK_THAT(result.out,
             EndsWith("\n  blender: " + fake.executable.string() + " (will be run; about 0.3 GB of memory)\n"));
  CHECK(std::filesystem::is_empty(w.outDir));
  CHECK_FALSE(std::filesystem::exists(fake.arguments));  // it was not started

  const Finished without = w.run({"--dry-run"});
  INFO("--no-blend, stdout:\n" << without.out << "stderr:\n" << without.err);
  CHECK(without.exitCode() == 0);
  CHECK(without.out.find(w.blend().string()) == std::string::npos);
  CHECK_THAT(without.out, EndsWith("\n  blender: skipped (--no-blend)\n"));
}

TEST_CASE("with no Blender to be found, a run writes the STL and PLY and fails, naming the ways to give one",
          "[mesh-e2e]") {
  const ScopedEnv blender("BLENDER", std::nullopt);
  const ScopedEnv path("PATH", "");
  if (findBlender(std::nullopt)) SKIP("the Blender app is installed, so dem2mesh finds one whatever PATH and BLENDER say");
  const Workspace w;
  const std::string notFound =
      "Blender not found: pass --blender, set BLENDER, or install it at /Applications/Blender.app";
  const Finished result = w.runWithBlender({});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK(result.err == "error: " + notFound + "\n");
  CHECK_THAT(result.out, EndsWith("\nwrote: " + w.ply().string() + "\nblender: not found\n"));
  CHECK(std::filesystem::exists(w.stl()));
  CHECK(std::filesystem::exists(w.ply()));
  CHECK_FALSE(std::filesystem::exists(w.blend()));

  const Finished plan = w.runWithBlender({"--dry-run", "--overwrite"});
  INFO("dry run, stdout:\n" << plan.out << "stderr:\n" << plan.err);
  CHECK(plan.exitCode() == 0);
  CHECK_THAT(plan.out, EndsWith("\n  blender: not found (a run would fail after writing the STL and PLY: pass "
                                "--blender, set BLENDER, or install it at /Applications/Blender.app)\n"));
}
