#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <charconv>
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

#include "GeoTiffFixtures.h"
#include "SpawnedProcess.h"
#include "TempDir.h"
#include "demdl/BoundingBox.h"
#include "demdl/DemRaster.h"
#include "mesh/CropWindow.h"
#include "mesh/GeoTiffGrid.h"
#include "mesh/PlyWriter.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"
#include "mesh/TransverseMercator.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;
using demdl::mesh::checkSolid;
using demdl::mesh::extentInMetres;
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

// dem2mesh started with args and run to its end, its stdout and stderr captured in files in dir.
Finished runDem2Mesh(const std::vector<std::string>& args, const Path& dir) {
  Child child(kExecutable, args, dir);
  return child.wait();
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

  // dem2mesh on the fixture with the outputs at stem and without Blender, then `extra`.
  [[nodiscard]] Finished run(const std::vector<std::string>& extra) const {
    std::vector<std::string> args{"-i", input.string(), "-o", stem.string(), "--no-blend"};
    args.insert(args.end(), extra.begin(), extra.end());
    return runDem2Mesh(args, dir.path);
  }
};

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
