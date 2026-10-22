#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <limits>
#include <optional>
#include <set>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "FakeBlender.h"
#include "GeoTiffFixtures.h"
#include "IslandFixture.h"
#include "SpawnedProcess.h"
#include "TempDir.h"
#include "demdl/BoundingBox.h"
#include "demdl/DemRaster.h"
#include "mesh/BasePlane.h"
#include "mesh/BlenderRunner.h"
#include "mesh/CropWindow.h"
#include "mesh/Footprint.h"
#include "mesh/FootprintSolid.h"
#include "mesh/GeoTiffGrid.h"
#include "mesh/NoDataFill.h"
#include "mesh/PlyWriter.h"
#include "mesh/Resampler.h"
#include "mesh/Simplifier.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"
#include "mesh/Tiles.h"
#include "mesh/TransverseMercator.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::EndsWith;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;
using demdl::mesh::BasePlane;
using demdl::mesh::BlockFootprint;
using demdl::mesh::blockFootprint;
using demdl::mesh::blockName;
using demdl::mesh::BlockRect;
using demdl::mesh::blockRect;
using demdl::mesh::BlockResult;
using demdl::mesh::BlockSimplifier;
using demdl::mesh::checkSolid;
using demdl::mesh::chooseBasePlane;
using demdl::mesh::cutShort;
using demdl::mesh::dataMaskOf;
using demdl::mesh::downwardFacingTriangles;
using demdl::mesh::extentInMetres;
using demdl::mesh::FillReport;
using demdl::mesh::fillNearest;
using demdl::mesh::findBlender;
using demdl::mesh::footprintCells;
using demdl::mesh::GridGeometry;
using demdl::mesh::GridPoint;
using demdl::mesh::inspectGeoTiff;
using demdl::mesh::Mesh;
using demdl::mesh::MeshCheck;
using demdl::mesh::Piece;
using demdl::mesh::PixelWindow;
using demdl::mesh::pixelWindowFor;
using demdl::mesh::plyFileBytes;
using demdl::mesh::readBinaryStl;
using demdl::mesh::readWindow;
using demdl::mesh::resampleQuadratic;
using demdl::mesh::solidTriangleCount;
using demdl::mesh::stlFileBytes;
using demdl::mesh::TileLayout;
using demdl::mesh::tileLayout;
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

// The terrain's height at (x, y) on the solid: the highest point of the solid above or below it, which is on the top
// surface. Every triangle is tried, so any triangulation will do, a grid's cells or a simplified surface's, and (x, y)
// need not be a vertex: a triangle that covers it seen from above gives the height of its plane there. The walls stand
// upright and cover nothing seen from above, and the base lies under the surface. A point outside a triangle by up to
// kSlack of it, in barycentric terms, counts as covered, so that one on an edge or a corner is found whatever the
// rounding of the vertices' floats. NaN when no triangle covers (x, y).
double topAt(const Mesh& mesh, double x, double y) {
  constexpr double kSlack = 1e-4;
  double top = std::numeric_limits<double>::quiet_NaN();
  for (const auto& t : mesh.triangles) {
    const auto& a = mesh.vertices[t.a];
    const auto& b = mesh.vertices[t.b];
    const auto& c = mesh.vertices[t.c];
    // Twice the triangle's area seen from above, signed, and the same for (x, y) with each pair of its corners: the
    // barycentric weights of a, b and c at (x, y).
    const auto twiceArea = [](double px, double py, double qx, double qy, double rx, double ry) {
      return (qx - px) * (ry - py) - (qy - py) * (rx - px);
    };
    const double area = twiceArea(a.x, a.y, b.x, b.y, c.x, c.y);
    if (!(std::abs(area) > 1e-9)) continue;  // a wall's, upright
    const double wa = twiceArea(x, y, b.x, b.y, c.x, c.y) / area;
    const double wb = twiceArea(a.x, a.y, x, y, c.x, c.y) / area;
    const double wc = 1 - wa - wb;
    if (wa < -kSlack || wb < -kSlack || wc < -kSlack) continue;
    const double z = wa * a.z + wb * b.z + wc * c.z;
    if (!(z <= top)) top = z;
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

// What follows `label` in the header of the PLY at `path`, such as N for "element vertex " in "element vertex N".
std::string plyHeaderAfter(const Path& path, std::string_view label) {
  const std::string bytes = readFile(path);
  const std::size_t end = bytes.find("end_header\n");
  REQUIRE(end != std::string::npos);
  return lineAfter(bytes.substr(0, end), label);
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
  // The memory and the simplification are planned only with --tolerance.
  CHECK(result.out.find("memory: ") == std::string::npos);
  CHECK(result.out.find("simplify: ") == std::string::npos);
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
  CHECK(result.out.find("simplify: ") == std::string::npos);  // without --tolerance the grid is written as it is

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

TEST_CASE("in Part B's modes without --tolerance a grid of more than 25,000,000 vertices is refused before reading",
          "[mesh-e2e]") {
  // --tile and --footprint data simplify at 0 without --tolerance, which keeps nearly every sample, so the cap on the
  // grid is the cap on what would be built (the plan's ruling on the cap); the refusal names the flag that lifts it.
  // Required, not checked: a grid let through would be read and simplified at 0, 30,000,000 vertices of it.
  const Workspace w;
  const std::string refusal = "error: the output grid is 30000000 vertices; without --tolerance the limit is "
                              "25000000: give --tolerance, or use --grid or crop\n";
  for (const std::vector<std::string>& mode :
       {std::vector<std::string>{"--footprint", "data", "--tile", "10"}, {"--tile", "10"}, {"--footprint", "data"}}) {
    std::vector<std::string> args = mode;
    args.insert(args.end(), {"--grid", "6000x5000"});
    const Finished run = w.run(args);
    INFO("stdout:\n" << run.out << "stderr:\n" << run.err);
    CHECK(run.exitCode() == 1);
    REQUIRE(run.err == refusal);
    CHECK(run.out.empty());
    args.push_back("--dry-run");
    const Finished dry = w.run(args);
    INFO("dry run, stdout:\n" << dry.out << "stderr:\n" << dry.err);
    CHECK(dry.exitCode() == 1);
    REQUIRE(dry.err == refusal);
    CHECK(dry.out == "dry run\n");
  }
  CHECK(std::filesystem::is_empty(w.outDir));
}

TEST_CASE("a tolerance writes a simplified solid within it", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--tolerance", "0.5", "--grid", "32x24"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  // The check of the TIN's surface, made before the solid is built on it, found no triangle facing down; a failure would
  // say "mesh self-check failed" on stderr.
  CHECK(result.out.find("self-check") == std::string::npos);
  CHECK(result.err.find("self-check") == std::string::npos);

  // The plane collapses, and the dent the fill left in the north-east corner keeps a few triangles more than the box's.
  const Mesh solid = readBinaryStl(w.stl());
  CHECK(solid.triangles.size() < solidTriangleCount(32, 24));
  CHECK(solid.triangles.size() > 12);
  CHECK(checkSolid(solid).ok());

  // The same extent and summit as the full grid's: the corners are always vertices, and the summit is one of them.
  const WindowExtent extent = wholeFileExtent(w.input);
  const Bounds b = boundsOf(solid);
  CHECK_THAT(b.minX, WithinAbs(0.0, 0.01));
  CHECK_THAT(b.maxX, WithinAbs(extent.widthM, 0.01));
  CHECK_THAT(b.minY, WithinAbs(0.0, 0.01));
  CHECK_THAT(b.maxY, WithinAbs(extent.heightM, 0.01));
  CHECK_THAT(b.maxZ, WithinAbs(60.0, 0.01));
  CHECK_THAT(b.minZ, WithinAbs(reportedBase(result.out), 0.01));

  // The grid dem2mesh simplified, rebuilt as it builds it: the window, filled, resampled to 32 x 24. At the scale of 1
  // a metre is a unit, so the solid's top is within 0.5 of every one of its 768 heights, give or take float rounding.
  const GridGeometry geometry = inspectGeoTiff(w.input);
  const PixelWindow window = pixelWindowFor(geometry, std::nullopt);
  std::vector<float> heights = readWindow(w.input, geometry, window);
  const FillReport fill = fillNearest(heights, window.cols, window.rows);
  CHECK(fill.filled == 2);
  const std::vector<float> grid = resampleQuadratic(heights, window.cols, window.rows, 32, 24);
  REQUIRE(grid.size() == 768);
  for (int j = 0; j < 24; ++j) {
    for (int i = 0; i < 32; ++i) {
      CAPTURE(i, j);
      CHECK_THAT(topAt(solid, i * extent.widthM / 31, (23 - j) * extent.heightM / 23),
                 WithinAbs(grid[static_cast<std::size_t>(j * 32 + i)], 0.5 + 0.01));
    }
  }

  // The PLY holds the simplified surface, whose counts the simplify: line gives, between the grid: and heights: lines.
  const std::string vertices = plyHeaderAfter(w.ply(), "element vertex ");
  const std::string faces = plyHeaderAfter(w.ply(), "element face ");
  CHECK(std::stoul(vertices) < 768);
  CHECK(std::stoul(faces) < 1426);
  CHECK_THAT(result.out, ContainsSubstring("\ngrid: " + lineAfter(result.out, "grid: ") +
                                           "\nsimplify: tolerance 0.5 units (0.5 m), 768 grid vertices to " + vertices +
                                           ", 1426 triangles to " + faces + "\nheights: "));
  // The sizes the summary gives are those of the files written.
  CHECK_THAT(result.out, ContainsSubstring("\nsolid: " + std::to_string(solid.triangles.size()) + " triangles, STL " +
                                           std::to_string(std::filesystem::file_size(w.stl())) + " bytes, PLY " +
                                           std::to_string(std::filesystem::file_size(w.ply())) + " bytes\n"));
}

TEST_CASE("with a tolerance the summit line gives the simplified mesh's highest point", "[mesh-e2e]") {
  // A field at 10 m with a bump of 0.4 m, which the resampler at full resolution lowers to 3/4 x 3/4 of it above the
  // field, 10.225 m, and which a tolerance of 0.5 leaves out: the mesh is the field's four corners at 10 m.
  const TempDir dir;
  std::vector<double> heights(static_cast<std::size_t>(kCols) * kRows, 10.0);
  heights[2 * kCols + 3] = 10.4;
  const Path input = writeGeographicFixture(dir.path, kCols, kRows, kBox, heights);
  const auto run = [&](const std::string& name, const std::vector<std::string>& extra) {
    std::vector<std::string> args{"-i", input.string(), "-o", (dir.path / name).string(), "--no-blend"};
    args.insert(args.end(), extra.begin(), extra.end());
    return runDem2Mesh(args, dir.path);
  };

  const Finished simplified = run("simplified", {"--tolerance", "0.5"});
  INFO("stdout:\n" << simplified.out << "stderr:\n" << simplified.err);
  REQUIRE(simplified.exitCode() == 0);
  CHECK_THAT(simplified.out,
             ContainsSubstring("\nsimplify: tolerance 0.5 units (0.5 m), 48 grid vertices to 4, 70 triangles to 2\n"));
  CHECK_THAT(simplified.out, ContainsSubstring("\nsummit: 10.4 m in the window, 10 m in the mesh\n"));
  // A plane's solid is the box: two triangles on top, two on each wall and two in the base.
  const Mesh box = readBinaryStl(dir.path / "simplified.stl");
  CHECK(box.triangles.size() == 12);
  CHECK(checkSolid(box).ok());
  CHECK_THAT(boundsOf(box).maxZ, WithinAbs(10.0, 1e-6));

  // The grid itself keeps the bump, and so does its summit line.
  const Finished full = run("full", {});
  INFO("without --tolerance, stdout:\n" << full.out << "stderr:\n" << full.err);
  REQUIRE(full.exitCode() == 0);
  CHECK_THAT(full.out, ContainsSubstring("\nsummit: 10.4 m in the window, 10.22 m in the mesh\n"));
}

TEST_CASE("tolerance is taken in model units", "[mesh-e2e]") {
  const Workspace w;
  // 0.1 units at a scale of 0.02 and an exaggeration of 2 stand for 0.1 / (2 x 0.02) = 2.5 m of terrain.
  const Finished result = w.run({"--tolerance", "0.1", "--scale", "0.02", "--z-scale", "2", "--dry-run"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK_THAT(result.out, ContainsSubstring("\n  simplify: tolerance 0.1 units (2.5 m), "));
  CHECK(std::filesystem::is_empty(w.outDir));
}

TEST_CASE("with a tolerance the grid may exceed the vertex limit", "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--grid", "5001x5001", "--tolerance", "1000", "--dry-run"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK_THAT(result.out, ContainsSubstring("\n  grid: 5001 x 5001 vertices"));
  // The window's 48 samples and the grid's 25,010,001 heights as float32: 100,040,196 bytes. Then the simplification:
  // the heights span 50 m, well within 1000, so each of the 3 x 3 chunks of up to 2048 cells is two triangles over its
  // corners, a lattice of 4 x 4 vertices. The memory line comes after the grid line and before the simplify line.
  CHECK_THAT(result.out, ContainsSubstring("\n  grid: " + lineAfter(result.out, "grid: ") +
                                           "\n  memory: window 48 samples and grid 25010001 vertices, 4 bytes each: "
                                           "about 100 MB\n  simplify: tolerance 1000 units (1000 m), 25010001 grid "
                                           "vertices to 16, 50000000 triangles to 18\n  heights: "));
  CHECK(std::filesystem::is_empty(w.outDir));
}

TEST_CASE("with a tolerance a grid of more than 2,147,483,647 vertices is refused and nothing is written",
          "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--grid", "50000x50000", "--tolerance", "1"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK(result.err == "error: the output grid is 2500000000 vertices; with --tolerance the limit is 2147483647: use "
                      "--grid or crop\n");
  CHECK(std::filesystem::is_empty(w.outDir));
}

TEST_CASE("in Part B's modes --tolerance lifts the grid's limit to 2,147,483,647 vertices, as it does without them",
          "[mesh-e2e]") {
  const Workspace w;
  const Finished result = w.run({"--tile", "10", "--tolerance", "0.5", "--grid", "50000x50000"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK(result.err == "error: the output grid is 2500000000 vertices; with --tolerance the limit is 2147483647: use "
                      "--grid or crop\n");
  CHECK(result.out.empty());

  // A grid above 25,000,000 vertices passes the grid's check with the flag and goes on to the lattice, which blocks of
  // 10 m on cells of about a metre make too large, still before any pixel is read.
  const Finished lifted =
      w.run({"--footprint", "data", "--tile", "10", "--tolerance", "0.5", "--grid", "6000x5000", "--dry-run"});
  INFO("a grid of 30,000,000 vertices with --tolerance, stdout:\n" << lifted.out << "stderr:\n" << lifted.err);
  CHECK(lifted.exitCode() == 1);
  CHECK_THAT(lifted.err, StartsWith("error: the lattice is "));
  CHECK_THAT(lifted.err, EndsWith(" blocks; the limit is 10000: use a larger --tile\n"));
  CHECK(lifted.out == "dry run\n");
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

  // With --tolerance the grid is simplified instead of being made into a surface as it stands.
  const Finished simplified = w.run({"--overwrite", "--verbose", "--tolerance", "0.5"});
  INFO("with --tolerance, stdout:\n" << simplified.out << "stderr:\n" << simplified.err);
  CHECK(simplified.exitCode() == 0);
  CHECK_THAT(simplified.out, ContainsSubstring("\ntime: simplify "));
  CHECK(simplified.out.find("time: surface ") == std::string::npos);
}

TEST_CASE("with Blender installed, a run saves the scene and Blender's check of the STL passes", "[mesh-e2e][slow]") {
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
  SECTION("an old scene that cannot be removed, which fails the run before Blender is looked for") {
    // A directory with something in it, which a removal leaves where it is.
    REQUIRE(std::filesystem::create_directory(w.blend()));
    writeFile(w.blend() / "inside", "");
    const Finished result = w.runWithBlender({"--blender", fake.executable.string(), "--overwrite"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK(result.err == "error: cannot remove " + w.blend().string() + ": Directory not empty\n");
    CHECK_THAT(result.out, EndsWith("\nwrote: " + w.ply().string() + "\n"));  // no blender: line yet
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

  // An old scene goes before Blender is looked for, so that a run that finds none leaves none beside the new STL
  // and PLY.
  writeFile(w.blend(), "an old scene");
  const Finished again = w.runWithBlender({"--overwrite"});
  INFO("over an old scene, stdout:\n" << again.out << "stderr:\n" << again.err);
  CHECK(again.exitCode() == 1);
  CHECK(again.err == "error: " + notFound + "\n");
  CHECK_THAT(again.out, EndsWith("\nwrote: " + w.ply().string() + "\nblender: not found\n"));
  CHECK_FALSE(std::filesystem::exists(w.blend()));
}

// Part B, the simplify-and-tiles design's sections 5.2 to 5.4 and 7: the island of IslandFixture.h, cut off where its
// data ends (--footprint data) and split into blocks (--tile), each written as a closed solid of its own.

namespace {

// The island's heights as a file of dl_elevation's holds them, its no-data as DemRaster::kNoData.
std::vector<double> islandFileHeights() {
  std::vector<double> heights;
  for (const float h : islandHeights()) heights.push_back(std::isnan(h) ? demdl::DemRaster::kNoData : double{h});
  return heights;
}

// value to `count` decimals, as dem2mesh prints a size or an area.
std::string decimals(double value, int count) {
  char text[64];
  std::snprintf(text, sizeof text, "%.*f", count, value);
  return text;
}

// A block's name as the summary gives it: "r1 c2" for "r1_c2".
std::string label(std::string name) {
  name[name.find('_')] = ' ';
  return name;
}

// The percentage the summary gives for part of whole: rounded to a whole number.
std::string percent(std::uint64_t part, std::uint64_t whole) {
  return std::to_string(std::llround(100.0 * static_cast<double>(part) / static_cast<double>(whole)));
}

// What a block of the island's lattice is to come to, worked out with the library as dem2mesh works it out.
struct ExpectedBlock {
  BlockRect rect{};
  std::string name;  // "r1_c2"
  bool cutShort = false;
  bool land = false;  // a piece kept, which every block of --footprint window has
  std::uint64_t cells = 0, landCells = 0, cornerContacts = 0;
  std::size_t pieces = 0;
  std::vector<std::uint64_t> dropped;  // the cells of each piece under --min-piece
  std::uint64_t solidTriangles = 0;    // its solid's, when it has land
};

// A vertex of a block's top, back in the window's frame.
struct TopVertex {
  double x = 0, y = 0;
  float z = 0;
};

// The island in a TempDir as a file of dl_elevation's, and an empty out/ for the outputs. The runs are at full
// resolution and --scale 1, so a model unit is a metre and the grid's cells are the window's.
struct Island {
  TempDir dir;
  Path input = writeGeographicFixture(dir.path, kIslandCols, kIslandRows, islandBox(), islandFileHeights());
  Path outDir = dir.path / "out";
  Path stem = outDir / "island";
  WindowExtent extent = wholeFileExtent(input);
  // The output grid's cells as dem2mesh works them out: the window's extent over the cells.
  double cellX = extent.widthM / (kIslandCols - 1);
  double cellY = extent.heightM / (kIslandRows - 1);

  Island() { REQUIRE(std::filesystem::create_directory(outDir)); }

  [[nodiscard]] Path stl() const { return stem.string() + ".stl"; }
  [[nodiscard]] Path ply() const { return stem.string() + ".ply"; }
  [[nodiscard]] Path blend() const { return stem.string() + ".blend"; }
  [[nodiscard]] Path blockStl(const ExpectedBlock& block) const { return stem.string() + "_" + block.name + ".stl"; }

  // --tile for blocks `cells` of the grid's cells wide, and the lattice it makes.
  [[nodiscard]] std::string tile(int cells) const { return decimals(cells * cellX, 6); }
  [[nodiscard]] TileLayout layout(int cells) const {
    return tileLayout(kIslandCols, kIslandRows, std::stod(tile(cells)), 1, cellX, cellY);
  }
  // The one block of the whole grid that a footprint without --tile is.
  [[nodiscard]] TileLayout wholeGrid() const { return tileLayout(kIslandCols, kIslandRows, 1e9, 1, cellX, cellY); }

  // Where sample (column, row) lies in the window's frame, the column and row fractional or not.
  [[nodiscard]] double xOf(double column) const { return column * cellX; }
  [[nodiscard]] double yOf(double row) const { return (kIslandRows - 1 - row) * cellY; }

  // A block's top read back from its STL: the vertices above the base, which is the solid's lowest z, with the
  // block's south-west corner added back to x and y, from which its own frame measures them (Part B 5.3).
  [[nodiscard]] std::vector<TopVertex> topOf(const ExpectedBlock& block) const {
    const Mesh solid = readBinaryStl(blockStl(block));
    const double base = boundsOf(solid).minZ;
    const double x0 = block.rect.firstColumn * extent.widthM / (kIslandCols - 1);
    const double y0 = (kIslandRows - 1 - block.rect.lastRow) * extent.heightM / (kIslandRows - 1);
    std::vector<TopVertex> top;
    for (const auto& v : solid.vertices) {
      if (v.z > base + 1e-3) top.push_back({v.x + x0, v.y + y0, v.z});
    }
    return top;
  }

  // dem2mesh on the island with the outputs at stem, without Blender, at --scale 1 and --tolerance 0.5, then extra.
  [[nodiscard]] Finished run(const std::vector<std::string>& extra) const { return runAt("0.5", extra); }

  // The same at the --tolerance given, or with none.
  [[nodiscard]] Finished runAt(const std::optional<std::string>& tolerance,
                               const std::vector<std::string>& extra) const {
    std::vector<std::string> args{"-i", input.string(), "-o", stem.string(), "--no-blend", "--scale", "1"};
    if (tolerance) args.insert(args.end(), {"--tolerance", *tolerance});
    args.insert(args.end(), extra.begin(), extra.end());
    return runDem2Mesh(args, dir.path);
  }

  // The same with Blender, as a run goes without --no-blend, at --scale 1 and --tolerance 0.5, then extra.
  [[nodiscard]] Finished runWithBlender(const std::vector<std::string>& extra) const {
    std::vector<std::string> args{"-i", input.string(), "-o", stem.string(), "--scale", "1", "--tolerance", "0.5"};
    args.insert(args.end(), extra.begin(), extra.end());
    return runDem2Mesh(args, dir.path, kBlenderDeadline);
  }

  // The grid dem2mesh meshes: the window read, filled from the nearest data and resampled to its own size.
  [[nodiscard]] std::vector<float> grid() const {
    const GridGeometry geometry = inspectGeoTiff(input);
    const PixelWindow window = pixelWindowFor(geometry, std::nullopt);
    std::vector<float> heights = readWindow(input, geometry, window);
    (void)fillNearest(heights, window.cols, window.rows);
    return resampleQuadratic(heights, window.cols, window.rows, window.cols, window.rows);
  }

  // The footprint's cells as dem2mesh works them out, and the corner contacts they join: the window's data as the
  // mask, which at full resolution is the grid's.
  [[nodiscard]] std::vector<std::uint8_t> cells(std::vector<GridPoint>* joined = nullptr) const {
    const GridGeometry geometry = inspectGeoTiff(input);
    const std::vector<float> heights = readWindow(input, geometry, pixelWindowFor(geometry, std::nullopt));
    return footprintCells(dataMaskOf(heights), kIslandCols, kIslandRows, joined);
  }

  // Every block of the lattice, row by row from the north-west, as dem2mesh is to make it at --tolerance toleranceM
  // (metres being units here): the footprint's pieces in the block (with --footprint data), and the solid of each block
  // with land, simplified with the library in lattice order over the grid above, its base plane chosen as dem2mesh
  // chooses it.
  [[nodiscard]] std::vector<ExpectedBlock> expected(const TileLayout& layout, bool footprint, double minPiece = 1,
                                                    double toleranceM = 0.5) const {
    const GridGeometry geometry = inspectGeoTiff(input);
    const PixelWindow window = pixelWindowFor(geometry, std::nullopt);
    std::vector<float> filled = readWindow(input, geometry, window);
    (void)fillNearest(filled, window.cols, window.rows);
    const auto [lowest, highest] = std::minmax_element(filled.begin(), filled.end());
    const BasePlane base = chooseBasePlane(*lowest, *highest, std::nullopt, 1, 1, 2);
    const std::vector<float> heights = resampleQuadratic(filled, window.cols, window.rows, window.cols, window.rows);
    std::vector<GridPoint> joined;
    const std::vector<std::uint8_t> footprintCellsOfGrid = cells(&joined);
    BlockSimplifier simplifier(heights, kIslandCols, kIslandRows, toleranceM, extent.widthM, extent.heightM, 1, 1,
                               base.levelM);
    std::vector<ExpectedBlock> blocks;
    for (int r = 0; r < layout.down; ++r) {
      for (int c = 0; c < layout.across; ++c) {
        ExpectedBlock& b = blocks.emplace_back();
        b.rect = blockRect(layout, r, c);
        b.name = blockName(layout, r, c);
        b.cutShort = cutShort(layout, r, c);
        b.cells = static_cast<std::uint64_t>(b.rect.lastColumn - b.rect.firstColumn) *
                  static_cast<std::uint64_t>(b.rect.lastRow - b.rect.firstRow);
        if (!footprint) {
          b.land = true;
          b.landCells = b.cells;
          const BlockResult result = simplifier.simplifyBlock(b.rect, nullptr);
          b.solidTriangles = solidTriangleCount(*result.rectangle, result.triangles);
          continue;
        }
        const BlockFootprint cut =
            blockFootprint(footprintCellsOfGrid, kIslandCols, kIslandRows, b.rect, cellX * cellY, minPiece, joined);
        for (const Piece& piece : cut.dropped) b.dropped.push_back(piece.cells);
        b.land = !cut.pieces.empty();
        b.landCells = cut.cells;
        b.cornerContacts = cut.cornerContacts;
        b.pieces = cut.pieces.size();
        if (b.land) {
          const BlockResult result = simplifier.simplifyBlock(b.rect, &cut);
          b.solidTriangles = solidTriangleCount(result.pieces, result.triangles);
        }
      }
    }
    return blocks;
  }
};

// The summary's line for a block written: "block r1 c2: 34% land, 282 triangles, 0.0 MB, island_r1_c2.stl".
std::string blockLine(const Island& island, const ExpectedBlock& b) {
  return "block " + label(b.name) + ": " + percent(b.landCells, b.cells) + "% land, " +
         std::to_string(b.solidTriangles) + " triangles, " +
         decimals(static_cast<double>(stlFileBytes(b.solidTriangles)) / 1e6, 1) + " MB, " +
         island.blockStl(b).filename().string();
}

// The names of blocks, as the summary lists them: "r2 c1, r2 c2".
std::string listOf(const std::vector<const ExpectedBlock*>& blocks) {
  std::string list;
  for (const ExpectedBlock* b : blocks) list += (list.empty() ? "" : ", ") + label(b->name);
  return list;
}

// The vertices of a block's top on the grid line where it meets its neighbour, within the stretches of that line where
// both blocks have land: `shared` lists the cells along the line, rows down a column or columns along a row, at which
// the cells on both sides of the line are in the footprint. The line is column `line` when `vertical`, and row `line`
// otherwise. Sorted along the line.
std::vector<TopVertex> onSharedStretches(const Island& island, const std::vector<TopVertex>& top, bool vertical,
                                         int line, const std::vector<int>& shared) {
  std::vector<std::pair<double, TopVertex>> found;
  for (const TopVertex& v : top) {
    const double across = vertical ? v.x - island.xOf(line) : v.y - island.yOf(line);
    if (std::abs(across) > 1e-3) continue;
    // Where the vertex lies along the line, in cells: a row down a column, or a column along a row.
    const double along = vertical ? (kIslandRows - 1) - v.y / island.cellY : v.x / island.cellX;
    const bool inStretch = std::any_of(shared.begin(), shared.end(), [along](int cell) {
      return along >= cell - 1e-6 && along <= cell + 1 + 1e-6;
    });
    if (inStretch) found.emplace_back(along, v);
  }
  std::sort(found.begin(), found.end(), [](const auto& a, const auto& b) { return a.first < b.first; });
  std::vector<TopVertex> sorted;
  for (const auto& [along, v] : found) sorted.push_back(v);
  return sorted;
}

// Part B 5.3's shared edges: wherever two neighbouring blocks that are written both have land along the side between
// them, their tops have the same vertices there, at positions within 1e-3 units of each other and at exactly the same
// height, so that the printed blocks meet without a step. Returns how many vertices matched, so that a check that
// found nothing to compare shows.
std::size_t matchSharedSides(const Island& island, const TileLayout& layout, const std::vector<ExpectedBlock>& blocks) {
  const std::vector<std::uint8_t> cells = island.cells();
  const auto in = [&cells](int column, int row) {
    return cells[static_cast<std::size_t>(row) * (kIslandCols - 1) + static_cast<std::size_t>(column)] != 0;
  };
  std::size_t matched = 0;
  for (std::size_t a = 0; a < blocks.size(); ++a) {
    const int r = static_cast<int>(a) / layout.across;
    const int c = static_cast<int>(a) % layout.across;
    // The neighbour to the east, then the one to the south.
    for (const bool vertical : {true, false}) {
      if (vertical ? c + 1 == layout.across : r + 1 == layout.down) continue;
      const std::size_t b = vertical ? a + 1 : a + static_cast<std::size_t>(layout.across);
      if (!blocks[a].land || !blocks[b].land) continue;
      const BlockRect& rect = blocks[a].rect;
      const int line = vertical ? rect.lastColumn : rect.lastRow;
      std::vector<int> shared;  // the cells along the line with land on both sides of it
      for (int k = vertical ? rect.firstRow : rect.firstColumn; k < (vertical ? rect.lastRow : rect.lastColumn); ++k) {
        if (vertical ? in(line - 1, k) && in(line, k) : in(k, line - 1) && in(k, line)) shared.push_back(k);
      }
      const std::vector<TopVertex> west = onSharedStretches(island, island.topOf(blocks[a]), vertical, line, shared);
      const std::vector<TopVertex> east = onSharedStretches(island, island.topOf(blocks[b]), vertical, line, shared);
      CAPTURE(blocks[a].name, blocks[b].name, shared.size());
      REQUIRE(west.size() == east.size());
      for (std::size_t k = 0; k < west.size(); ++k) {
        CAPTURE(k);
        CHECK_THAT(west[k].x, WithinAbs(east[k].x, 1e-3));
        CHECK_THAT(west[k].y, WithinAbs(east[k].y, 1e-3));
        CHECK(west[k].z == east[k].z);
      }
      matched += west.size();
    }
  }
  return matched;
}

// The grid samples that the written blocks' tops stand on, each once, every top vertex found within 1e-3 units of its
// sample.
std::set<std::pair<int, int>> samplesUnder(const Island& island, const std::vector<ExpectedBlock>& blocks) {
  std::set<std::pair<int, int>> samples;
  for (const ExpectedBlock& b : blocks) {
    if (!b.land) continue;
    for (const TopVertex& v : island.topOf(b)) {
      const int column = static_cast<int>(std::lround(v.x / island.cellX));
      const int row = static_cast<int>(std::lround((kIslandRows - 1) - v.y / island.cellY));
      CHECK_THAT(v.x, WithinAbs(island.xOf(column), 1e-3));
      CHECK_THAT(v.y, WithinAbs(island.yOf(row), 1e-3));
      samples.emplace(column, row);
    }
  }
  return samples;
}

// The island in blocks `cells` cells wide with --footprint data (Part B's section 7): only the blocks with land are
// written, each a closed solid of the triangles its outlines make; neighbours match along their shared sides; the
// summary counts and names the blocks; the PLY is the blocks' surfaces joined, each sample once.
void checkIslandTiles(int cells) {
  const Island island;
  const TileLayout layout = island.layout(cells);
  CAPTURE(cells, layout.blockCols, layout.blockRows, layout.across, layout.down);
  const std::vector<ExpectedBlock> expected = island.expected(layout, true);
  const Finished result = island.run({"--footprint", "data", "--tile", island.tile(cells)});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);

  // The files: a block's for each block with land, and the PLY; nothing for a block of sea alone.
  std::vector<std::string> files{island.ply().filename().string()};
  std::vector<const ExpectedBlock*> written, skipped;
  std::uint64_t withLand = 0, cutShortBlocks = 0, contacts = 0;
  for (const ExpectedBlock& b : expected) {
    (b.land ? written : skipped).push_back(&b);
    if (b.land) {
      files.push_back(island.blockStl(b).filename().string());
      ++withLand;
      contacts += b.cornerContacts;
    }
    if (b.cutShort) ++cutShortBlocks;
  }
  std::sort(files.begin(), files.end());
  CHECK(filesIn(island.outDir) == files);
  REQUIRE_FALSE(written.empty());
  REQUIRE_FALSE(skipped.empty());  // the island's lattices keep a block of sea and a lone sample, which makes no cell

  // Each block a closed solid facing out, read back as a slicer would read it, its triangles those its outlines make.
  for (const ExpectedBlock* b : written) {
    CAPTURE(b->name);
    const Mesh solid = readBinaryStl(island.blockStl(*b));
    const MeshCheck check = checkSolid(solid);
    CHECK(check.ok());
    CHECK(check.nonManifoldEdges == 0);
    CHECK(solid.triangles.size() == b->solidTriangles);
    // The header names the block before the grid.
    const std::string header = readFile(island.blockStl(*b)).substr(0, 80);
    CHECK_THAT(header, StartsWith("dem2mesh "));
    CHECK_THAT(header, ContainsSubstring(" block " + b->name + " grid 24x18 window 24x18+0+0 "));
  }

  // No step between neighbours, and something to compare: a run of the island's coast crosses a block side.
  CHECK(matchSharedSides(island, layout, expected) > 0);

  // The PLY is the union of the blocks' tops, each sample once: the vertices blocks share along a side are merged.
  CHECK(std::stoul(plyHeaderAfter(island.ply(), "element vertex ")) == samplesUnder(island, expected).size());

  // The summary: the lattice, a line for each block written, the blocks of sea, the contact that the join made.
  const std::string tiles = lineAfter(result.out, "tiles: ");
  CHECK_THAT(tiles, StartsWith(std::to_string(layout.across) + " x " + std::to_string(layout.down) + " blocks of " +
                               std::to_string(layout.blockCols) + " x " + std::to_string(layout.blockRows) +
                               " cells ("));
  CHECK_THAT(tiles, EndsWith("); " + std::to_string(withLand) + " with land, " +
                             std::to_string(expected.size() - withLand) + " without, " +
                             std::to_string(cutShortBlocks) + " cut short at the east or south edge"));
  // A full block's size in metres and in model units, the same at --scale 1: blockCols x cellX by blockRows x cellY.
  char* end = nullptr;
  const char* const size = tiles.c_str() + tiles.find('(') + 1;
  CHECK_THAT(std::strtod(size, &end), WithinAbs(layout.blockCols * island.cellX, 0.01));
  CHECK_THAT(std::strtod(end + 3, &end), WithinAbs(layout.blockRows * island.cellY, 0.01));
  CHECK_THAT(std::string(end), StartsWith(" m, "));
  std::string lines = "\ntiles: " + tiles + "\n";
  for (const ExpectedBlock* b : written) lines += blockLine(island, *b) + "\n";
  lines += "skipped: no land in " + listOf(skipped) + "\n";
  if (contacts > 0) lines += "corner contacts: " + std::to_string(contacts) + "\n";
  CHECK_THAT(result.out, ContainsSubstring(lines + "solid: "));
  CHECK(result.out.find("dropped pieces: ") == std::string::npos);  // the lone sample makes no cell, so no piece
  CHECK(result.out.find("not selected: ") == std::string::npos);
  CHECK(result.out.find("stale: ") == std::string::npos);
  CHECK(result.out.find("footprint: ") == std::string::npos);

  // The files written, every block's STL in lattice order and then the PLY, with their sizes in the solid line.
  std::string wrote;
  std::uintmax_t stlBytes = 0;
  for (const ExpectedBlock* b : written) {
    wrote += "\nwrote: " + island.blockStl(*b).string();
    stlBytes += std::filesystem::file_size(island.blockStl(*b));
  }
  CHECK_THAT(result.out, ContainsSubstring(wrote + "\nwrote: " + island.ply().string() + "\n"));
  CHECK(occurrences(result.out, "wrote: ") == written.size() + 1);
  std::uint64_t triangles = 0;
  for (const ExpectedBlock* b : written) triangles += b->solidTriangles;
  CHECK_THAT(result.out, ContainsSubstring("\nsolid: " + std::to_string(triangles) + " triangles, STL " +
                                           std::to_string(stlBytes) + " bytes, PLY " +
                                           std::to_string(std::filesystem::file_size(island.ply())) + " bytes\n"));
  CHECK_THAT(result.out, EndsWith("\nblender: skipped (--no-blend)\n"));
}

}  // namespace

TEST_CASE("a footprint without tiles ends at the coast", "[mesh-e2e]") {
  const Island island;
  const Finished result = island.run({"--footprint", "data"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK(filesIn(island.outDir) == std::vector<std::string>{"island.ply", "island.stl"});

  const Mesh solid = readBinaryStl(island.stl());
  const MeshCheck check = checkSolid(solid);
  CHECK(check.ok());
  CHECK(check.nonManifoldEdges == 0);
  // One solid, whose header names no block.
  CHECK_THAT(readFile(island.stl()).substr(0, 80), StartsWith("dem2mesh " DL_ELEVATION_VERSION " grid 24x18 "));
  // The top surface faces up: the triangles with no corner on the base, since a footprint's walls are two triangles
  // under each edge of its outline, each with a corner on the base, and the base has all three there.
  const double base = boundsOf(solid).minZ;
  Mesh top;
  top.vertices = solid.vertices;
  for (const auto& t : solid.triangles) {
    const auto above = [&](std::uint32_t v) { return solid.vertices[v].z > base + 1e-3; };
    if (above(t.a) && above(t.b) && above(t.c)) top.triangles.push_back(t);
  }
  CHECK_FALSE(top.triangles.empty());
  CHECK(downwardFacingTriangles(top) == 0);

  // No surface over the sea, nor over the lone sample, which makes no cell; the lake, enclosed, is filled and has one.
  CHECK(std::isnan(topAt(solid, island.xOf(1), island.yOf(1))));
  CHECK(std::isnan(topAt(solid, island.xOf(18), island.yOf(13))));
  CHECK_FALSE(std::isnan(topAt(solid, island.xOf(7.5), island.yOf(5.5))));
  // The knoll's summit, a sample inside the island, within the tolerance of the grid's height there.
  const std::vector<float> grid = island.grid();
  CHECK_THAT(topAt(solid, island.xOf(kIslandKnollColumn), island.yOf(kIslandKnollRow)),
             WithinAbs(grid[static_cast<std::size_t>(kIslandKnollRow * kIslandCols + kIslandKnollColumn)], 0.5 + 0.01));

  // One block of the whole grid: its pieces and land, the triangles they make, and the blobs' contact, joined.
  const std::vector<ExpectedBlock> expected = island.expected(island.wholeGrid(), true);
  REQUIRE(expected.size() == 1);
  const ExpectedBlock& whole = expected.front();
  CHECK(solid.triangles.size() == whole.solidTriangles);
  CHECK(whole.cornerContacts == 1);
  CHECK_THAT(result.out, ContainsSubstring("\nbase: " + lineAfter(result.out, "base: ") + "\nfootprint: " +
                                           percent(whole.landCells, whole.cells) + "% of the window, " +
                                           std::to_string(whole.pieces) + " pieces\ncorner contacts: 1\nsolid: "));
  CHECK(result.out.find("dropped pieces: ") == std::string::npos);  // the lone sample makes no cell, so no piece
  CHECK(result.out.find("tiles: ") == std::string::npos);
  CHECK(result.out.find("block ") == std::string::npos);
  CHECK_THAT(result.out, ContainsSubstring("\nwrote: " + island.stl().string() + "\nwrote: " + island.ply().string() +
                                           "\nblender: skipped (--no-blend)\n"));
  // The simplification, at the tolerance given.
  CHECK_THAT(result.out, ContainsSubstring("\nsimplify: tolerance 0.5 units (0.5 m), 432 grid vertices to "));
}

TEST_CASE("blocks without --tolerance are simplified at a tolerance of 0", "[mesh-e2e]") {
  // The plan's ruling: Part B's modes run on the block simplifier, which without --tolerance simplifies at 0, keeping
  // every sample that is off its neighbours' plane.
  const Island island;
  const TileLayout layout = island.layout(12);
  const std::vector<ExpectedBlock> expected = island.expected(layout, true, 1, 0.0);
  const Finished result = island.runAt(std::nullopt, {"--footprint", "data", "--tile", island.tile(12)});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK_THAT(result.out, ContainsSubstring("\nsimplify: tolerance 0 units (0 m), 432 grid vertices to "));
  for (const ExpectedBlock& b : expected) {
    CAPTURE(b.name);
    CHECK(std::filesystem::exists(island.blockStl(b)) == b.land);
    if (!b.land) continue;
    const Mesh solid = readBinaryStl(island.blockStl(b));
    CHECK(checkSolid(solid).ok());
    CHECK(solid.triangles.size() == b.solidTriangles);
  }
  // The same without --tile: the one block of a footprint.
  const Finished whole = island.runAt(std::nullopt, {"--footprint", "data", "--overwrite"});
  INFO("without --tile, stdout:\n" << whole.out << "stderr:\n" << whole.err);
  REQUIRE(whole.exitCode() == 0);
  CHECK_THAT(whole.out, ContainsSubstring("\nsimplify: tolerance 0 units (0 m), 432 grid vertices to "));
  CHECK(readBinaryStl(island.stl()).triangles.size() ==
        island.expected(island.wholeGrid(), true, 1, 0.0).front().solidTriangles);
}

TEST_CASE("tiles write one closed block per piece of land and match along shared edges", "[mesh-e2e]") {
  checkIslandTiles(12);
}

TEST_CASE("a cut-short lattice still closes and matches", "[mesh-e2e]") {
  // 13 cells a block over the grid's 23 x 17 cells: the east blocks are 10 cells wide and the south ones 4 tall.
  checkIslandTiles(13);
}

TEST_CASE("a speck under --min-piece is dropped and the block skipped", "[mesh-e2e]") {
  const Island island;
  const TileLayout layout = island.layout(12);
  // Between the islet's 4 cells and the 8 of the smallest piece any block keeps besides it, the tab that column 12
  // cuts off the island.
  const double minPiece = 6 * island.cellX * island.cellY;
  const std::vector<ExpectedBlock> expected = island.expected(layout, true, minPiece);
  const ExpectedBlock* islet = nullptr;
  std::vector<const ExpectedBlock*> skipped;
  for (const ExpectedBlock& b : expected) {
    if (!b.dropped.empty()) islet = &b;
    if (!b.land) skipped.push_back(&b);
  }
  REQUIRE(islet != nullptr);
  REQUIRE(islet->dropped == std::vector<std::uint64_t>{4});
  REQUIRE_FALSE(islet->land);

  const Finished result = island.run({"--footprint", "data", "--tile", island.tile(12), "--min-piece",
                                      decimals(minPiece, 3)});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK_FALSE(std::filesystem::exists(island.blockStl(*islet)));
  // The islet, by its block, in cells and square model units to one decimal, and its block among the skipped ones.
  CHECK_THAT(result.out, ContainsSubstring("\nskipped: no land in " + listOf(skipped) + "\ndropped pieces: " +
                                           label(islet->name) + " 4 cells " +
                                           decimals(4 * (island.cellX * island.cellY), 1) + " units2\n"));
  CHECK_THAT(lineAfter(result.out, "tiles: "),
             ContainsSubstring("; " + std::to_string(expected.size() - skipped.size()) + " with land, " +
                               std::to_string(skipped.size()) + " without, "));
}

TEST_CASE("without tiles a speck under --min-piece is dropped and listed without a block", "[mesh-e2e]") {
  const Island island;
  const double minPiece = 6 * island.cellX * island.cellY;  // above the islet's 4 cells, as in the case above
  const std::vector<ExpectedBlock> expected = island.expected(island.wholeGrid(), true, minPiece);
  REQUIRE(expected.size() == 1);
  REQUIRE(expected.front().dropped == std::vector<std::uint64_t>{4});
  const Finished result = island.run({"--footprint", "data", "--min-piece", decimals(minPiece, 3)});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  const ExpectedBlock& whole = expected.front();
  CHECK_THAT(result.out, ContainsSubstring("\nfootprint: " + percent(whole.landCells, whole.cells) +
                                           "% of the window, " + std::to_string(whole.pieces) +
                                           " pieces\ndropped pieces: 4 cells " +
                                           decimals(4 * (island.cellX * island.cellY), 1) + " units2\n"));
  CHECK(readBinaryStl(island.stl()).triangles.size() == whole.solidTriangles);
}

TEST_CASE("a footprint with no piece to write is refused and nothing is written", "[mesh-e2e]") {
  const Island island;
  SECTION("every piece under --min-piece") {
    const Finished result = island.run({"--footprint", "data", "--min-piece", "1000000000"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    // The island, the islet and the blobs that the join makes one.
    CHECK(result.err == "error: the footprint has nothing to write: 3 pieces were dropped under --min-piece "
                        "1000000000\n");
    CHECK(result.out.empty());
  }
  SECTION("a selection of blocks of sea alone") {
    const TileLayout layout = island.layout(12);
    const std::vector<ExpectedBlock> expected = island.expected(layout, true);
    REQUIRE_FALSE(expected.back().land);  // the south-east block, with the lone sample, which makes no cell
    REQUIRE(expected.back().dropped.empty());
    const Finished result = island.run({"--footprint", "data", "--tile", island.tile(12), "--blocks",
                                        std::to_string(layout.down) + ":" + std::to_string(layout.across)});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    CHECK(result.exitCode() == 1);
    CHECK(result.err == "error: no selected block has land to write: no cell has data at all four of its corners\n");
    CHECK(result.out.empty());
  }
  CHECK(std::filesystem::is_empty(island.outDir));
}

TEST_CASE("pieces touching at a corner are joined into one closed shell, and the contact is counted", "[mesh-e2e]") {
  const Island island;
  const TileLayout layout = island.layout(12);
  // The blobs touch at (19, 4), and footprintCells joins them there with the cell (19, 3), the north-west-most of the
  // vertex's two empty cells (IslandFixture.h). The contact is inside one block, the one this finds.
  std::vector<GridPoint> joined;
  (void)island.cells(&joined);
  REQUIRE(joined.size() == 1);
  REQUIRE(joined.front().x == 19);
  REQUIRE(joined.front().y == 4);
  const std::vector<ExpectedBlock> expected = island.expected(layout, true);
  const auto blobs = std::find_if(expected.begin(), expected.end(), [](const ExpectedBlock& b) {
    return b.rect.firstColumn < 19 && 19 < b.rect.lastColumn && b.rect.firstRow < 4 && 4 < b.rect.lastRow;
  });
  REQUIRE(blobs != expected.end());
  REQUIRE(blobs->cornerContacts == 1);

  const Finished result = island.run({"--footprint", "data", "--tile", island.tile(12)});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  // One file for the block, a closed solid, with no edge of more than two faces where the blobs met: two shells that
  // touched there would stand two walls on the line under the vertex, which the STL's merged corners make one edge of
  // four faces.
  const Mesh solid = readBinaryStl(island.blockStl(*blobs));
  const MeshCheck check = checkSolid(solid);
  CHECK(check.ok());
  CHECK(check.nonManifoldEdges == 0);
  CHECK(solid.triangles.size() == blobs->solidTriangles);
  // The cell that joins them is land, with a surface over its middle, at the block's own frame.
  const double x0 = blobs->rect.firstColumn * island.cellX;
  const double y0 = island.yOf(blobs->rect.lastRow);
  CHECK_FALSE(std::isnan(topAt(solid, island.xOf(19.5) - x0, island.yOf(3.5) - y0)));
  CHECK_THAT(result.out, ContainsSubstring("\ncorner contacts: 1\nsolid: "));
}

TEST_CASE("a file without no-data gives the whole window as one piece", "[mesh-e2e]") {
  // The plane of the 8 x 6 fixture, 10 + 5 i + 3 j, with a height at every pixel.
  const TempDir dir;
  std::vector<double> heights;
  for (int j = 0; j < kRows; ++j) {
    for (int i = 0; i < kCols; ++i) heights.push_back(10.0 + 5.0 * i + 3.0 * j);
  }
  const Path input = writeGeographicFixture(dir.path, kCols, kRows, kBox, heights);
  const auto run = [&](const std::string& name, const std::vector<std::string>& extra) {
    std::vector<std::string> args{"-i", input.string(), "-o", (dir.path / name).string(), "--no-blend", "--tolerance",
                                  "0.5"};
    args.insert(args.end(), extra.begin(), extra.end());
    return runDem2Mesh(args, dir.path);
  };
  const Finished window = run("window", {});
  INFO("--footprint window, stdout:\n" << window.out << "stderr:\n" << window.err);
  REQUIRE(window.exitCode() == 0);
  const Finished data = run("data", {"--footprint", "data"});
  INFO("--footprint data, stdout:\n" << data.out << "stderr:\n" << data.err);
  REQUIRE(data.exitCode() == 0);

  // The same surface as --footprint window's (Review Focus 4): the same simplification, and the PLY byte for byte.
  CHECK(lineAfter(data.out, "simplify: ") == lineAfter(window.out, "simplify: "));
  CHECK_THAT(data.out, ContainsSubstring("\nsimplify: tolerance 0.5 units (0.5 m), 48 grid vertices to 4, 70 triangles "
                                         "to 2\n"));
  CHECK(fileHolds(dir.path / "data.ply", readFile(dir.path / "window.ply")));
  CHECK_THAT(data.out, ContainsSubstring("\nfootprint: 100% of the window, 1 piece\nsolid: "));
  // Its solid closed: the two triangles, a wall of two triangles under each of the outline's four edges, and a base of
  // two, as the window's box has.
  const Mesh solid = readBinaryStl(dir.path / "data.stl");
  const MeshCheck check = checkSolid(solid);
  CHECK(check.ok());
  CHECK(check.nonManifoldEdges == 0);
  CHECK(solid.triangles.size() == 12);
}

TEST_CASE("a --blocks selection writes those blocks and lists the rest", "[mesh-e2e]") {
  const Island island;
  const TileLayout layout = island.layout(12);
  const std::vector<ExpectedBlock> expected = island.expected(layout, true);
  REQUIRE(layout.down == 2);
  REQUIRE(layout.across == 2);
  const Finished result = island.run({"--footprint", "data", "--tile", island.tile(12), "--blocks", "1-1:1-2"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  // The first row of blocks, both with land, and the PLY of their surfaces alone.
  REQUIRE(expected[0].land);
  REQUIRE(expected[1].land);
  CHECK(filesIn(island.outDir) == std::vector<std::string>{island.ply().filename().string(),
                                                           island.blockStl(expected[0]).filename().string(),
                                                           island.blockStl(expected[1]).filename().string()});
  // The second row is outside the selection, the block with the islet and the block of sea alike, and nothing about
  // them is skipped or dropped.
  CHECK_THAT(result.out, ContainsSubstring("\n" + blockLine(island, expected[0]) + "\n" +
                                           blockLine(island, expected[1]) + "\nnot selected: " +
                                           listOf({&expected[2], &expected[3]}) + "\n"));
  CHECK(result.out.find("skipped: ") == std::string::npos);
  CHECK(occurrences(result.out, "\nblock ") == 2);
  CHECK(occurrences(result.out, "wrote: ") == 3);
  // The lattice is counted whole whatever --blocks picks.
  const auto withLand = std::count_if(expected.begin(), expected.end(), [](const auto& b) { return b.land; });
  CHECK_THAT(lineAfter(result.out, "tiles: "),
             ContainsSubstring("; " + std::to_string(withLand) + " with land, " +
                               std::to_string(static_cast<long>(expected.size()) - withLand) + " without, "));
}

TEST_CASE("stale block files are left alone and named", "[mesh-e2e]") {
  const Island island;
  const TileLayout layout = island.layout(12);
  const std::vector<ExpectedBlock> before = island.expected(layout, true);
  const double minPiece = 6 * island.cellX * island.cellY;  // above the islet's 4 cells, as in the case above
  const std::vector<ExpectedBlock> after = island.expected(layout, true, minPiece);
  std::vector<const ExpectedBlock*> emptied;  // the blocks with land before and none after: the islet's
  for (std::size_t k = 0; k < before.size(); ++k) {
    if (before[k].land && !after[k].land) emptied.push_back(&before[k]);
  }
  REQUIRE(emptied.size() == 1);
  const Path isletStl = island.blockStl(*emptied.front());

  const std::vector<std::string> tiles{"--footprint", "data", "--tile", island.tile(12)};
  const Finished first = island.run(tiles);
  INFO("first run, stdout:\n" << first.out << "stderr:\n" << first.err);
  REQUIRE(first.exitCode() == 0);
  REQUIRE(std::filesystem::exists(isletStl));
  const std::string islet = readFile(isletStl);
  const std::vector<std::string> filesBefore = filesIn(island.outDir);

  std::vector<std::string> again = tiles;
  again.insert(again.end(), {"--min-piece", decimals(minPiece, 3)});
  SECTION("without --overwrite, any block file of the lattice is refused before anything is read") {
    const Finished refused = island.run(again);
    INFO("stdout:\n" << refused.out << "stderr:\n" << refused.err);
    CHECK(refused.exitCode() == 1);
    // The first in lattice order, whether this run would write it or not.
    CHECK(refused.err == "error: " + island.blockStl(before.front()).string() + " exists; use --overwrite\n");
    CHECK(refused.out.empty());
    CHECK(filesIn(island.outDir) == filesBefore);
  }
  SECTION("with --overwrite, the islet's old file is left as it was and named") {
    again.push_back("--overwrite");
    const Finished second = island.run(again);
    INFO("stdout:\n" << second.out << "stderr:\n" << second.err);
    REQUIRE(second.exitCode() == 0);
    CHECK(fileHolds(isletStl, islet));
    CHECK_THAT(second.out, ContainsSubstring("\nstale: " + isletStl.filename().string() + "\nsolid: "));
    CHECK(filesIn(island.outDir) == filesBefore);
    CHECK(occurrences(second.out, "wrote: ") == filesBefore.size() - 1);  // every file but the islet's
  }
}

TEST_CASE("stale: names every block file of the stem that the run leaves behind, whatever lattice made it",
          "[mesh-e2e]") {
  // The lattice of 12-cell blocks first: 2 x 2 blocks, three of them with land, whose files are r1_c1, r1_c2 and r2_c1.
  const Island island;
  const std::vector<ExpectedBlock> twelve = island.expected(island.layout(12), true);
  const Finished once = island.run({"--footprint", "data", "--tile", island.tile(12)});
  INFO("first run, stdout:\n" << once.out << "stderr:\n" << once.err);
  REQUIRE(once.exitCode() == 0);
  std::vector<std::string> blockFiles;
  for (const ExpectedBlock& b : twelve) {
    if (b.land) blockFiles.push_back(island.blockStl(b).filename().string());
  }
  REQUIRE(blockFiles == std::vector<std::string>{"island_r1_c1.stl", "island_r1_c2.stl", "island_r2_c1.stl"});
  // A block's file of the stem from a lattice padded another way, and names that only look like a block's: without a
  // number, of another stem, with another extension, and a directory.
  writeFile(island.outDir / "island_r01_c01.stl", "an old block\n");
  writeFile(island.outDir / "island_r1_c.stl", "not a block\n");
  writeFile(island.outDir / "other_r1_c1.stl", "another stem's\n");
  writeFile(island.outDir / "island_r1_c1.stl.bak", "a copy\n");
  REQUIRE(std::filesystem::create_directory(island.outDir / "island_r9_c9.stl"));
  const std::string islet = readFile(island.outDir / "island_r2_c1.stl");

  SECTION("a run in another lattice names every one of them, in name order, and leaves them as they were") {
    // Blocks of 2 cells: a lattice of 12 x 9, named r1_c01 to r9_c12, none of them a name of the first run's.
    const TileLayout two = island.layout(2);
    REQUIRE(two.across == 12);
    REQUIRE(two.down == 9);
    const std::string stale = "stale: island_r01_c01.stl, island_r1_c1.stl, island_r1_c2.stl, island_r2_c1.stl\n";
    const std::vector<std::string> args{"--footprint", "data", "--tile", island.tile(2), "--overwrite"};
    std::vector<std::string> dryArgs = args;
    dryArgs.push_back("--dry-run");
    const Finished dry = island.run(dryArgs);
    INFO("dry run, stdout:\n" << dry.out << "stderr:\n" << dry.err);
    REQUIRE(dry.exitCode() == 0);
    CHECK_THAT(dry.out, ContainsSubstring("\n  " + stale + "  solid: "));
    const Finished again = island.run(args);
    INFO("second run, stdout:\n" << again.out << "stderr:\n" << again.err);
    REQUIRE(again.exitCode() == 0);
    CHECK_THAT(again.out, ContainsSubstring("\n" + stale + "solid: "));
    CHECK(fileHolds(island.outDir / "island_r2_c1.stl", islet));
    CHECK(fileHolds(island.outDir / "island_r01_c01.stl", "an old block\n"));
  }
  SECTION("blocks outside --blocks keep their files unnamed") {
    // The first lattice again, its west column of blocks selected and the islet dropped under --min-piece: r2_c1,
    // selected and now without land, is named, and so is the file of the other padding; r1_c2, not selected, is not.
    const double minPiece = 6 * island.cellX * island.cellY;  // above the islet's 4 cells
    const Finished again = island.run({"--footprint", "data", "--tile", island.tile(12), "--blocks", "1-2:1",
                                       "--min-piece", decimals(minPiece, 3), "--overwrite"});
    INFO("second run, stdout:\n" << again.out << "stderr:\n" << again.err);
    REQUIRE(again.exitCode() == 0);
    CHECK_THAT(again.out, ContainsSubstring("\nnot selected: r1 c2, r2 c2\n"));
    CHECK_THAT(again.out, ContainsSubstring("\nstale: island_r01_c01.stl, island_r2_c1.stl\nsolid: "));
    CHECK(fileHolds(island.outDir / "island_r2_c1.stl", islet));
  }
  SECTION("a footprint without --tile names them all, its own STL being no block's") {
    const Finished again = island.run({"--footprint", "data", "--overwrite"});
    INFO("second run, stdout:\n" << again.out << "stderr:\n" << again.err);
    REQUIRE(again.exitCode() == 0);
    CHECK_THAT(again.out, ContainsSubstring("\ncorner contacts: 1\nstale: island_r01_c01.stl, island_r1_c1.stl, "
                                            "island_r1_c2.stl, island_r2_c1.stl\nsolid: "));
  }
}

TEST_CASE("the dry run plans tiles without writing", "[mesh-e2e]") {
  const Island island;
  const TileLayout layout = island.layout(12);
  const std::vector<ExpectedBlock> expected = island.expected(layout, true);
  const Finished result = island.run({"--footprint", "data", "--tile", island.tile(12), "--dry-run"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK_THAT(result.out, StartsWith("dry run\n"));
  CHECK_THAT(result.out, ContainsSubstring("\n  tiles: "));
  // The plan's lines on the blocks are the summary's, then what a run would write, every STL and then the PLY.
  std::string blocks, wouldWrite;
  for (const ExpectedBlock& b : expected) {
    if (!b.land) continue;
    blocks += "\n  " + blockLine(island, b);
    wouldWrite += "\n  would write: " + island.blockStl(b).string();
  }
  CHECK_THAT(result.out, ContainsSubstring(blocks + "\n  skipped: no land in "));
  CHECK_THAT(result.out, ContainsSubstring(wouldWrite + "\n  would write: " + island.ply().string() + "\n  output: " +
                                           island.ply().string() + " (will be created)\n"));
  CHECK(result.out.find("output: " + island.stem.string() + "_") == std::string::npos);  // none for the blocks
  CHECK(result.out.find("\nwrote: ") == std::string::npos);
  CHECK_THAT(result.out, EndsWith("\n  blender: skipped (--no-blend)\n"));
  CHECK(std::filesystem::is_empty(island.outDir));
}

TEST_CASE("window tiles are full squares", "[mesh-e2e]") {
  const Island island;
  const TileLayout layout = island.layout(12);
  const std::vector<ExpectedBlock> expected = island.expected(layout, false);
  REQUIRE(expected.size() == 4);
  const Finished result = island.run({"--footprint", "window", "--tile", island.tile(12)});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  std::vector<std::string> files{island.ply().filename().string()};
  std::string lines;
  for (const ExpectedBlock& b : expected) {
    CAPTURE(b.name);
    files.push_back(island.blockStl(b).filename().string());
    // A square of the window's own surface, with the fill where the sea was: the rectangle's walls and base.
    const Mesh solid = readBinaryStl(island.blockStl(b));
    const MeshCheck check = checkSolid(solid);
    CHECK(check.ok());
    CHECK(check.nonManifoldEdges == 0);
    CHECK(solid.triangles.size() == b.solidTriangles);
    // In the block's own frame, from its south-west corner to its north-east one.
    const auto bounds = boundsOf(solid);
    CHECK_THAT(bounds.minX, WithinAbs(0.0, 1e-3));
    CHECK_THAT(bounds.minY, WithinAbs(0.0, 1e-3));
    CHECK_THAT(bounds.maxX, WithinAbs((b.rect.lastColumn - b.rect.firstColumn) * island.cellX, 1e-3));
    CHECK_THAT(bounds.maxY, WithinAbs((b.rect.lastRow - b.rect.firstRow) * island.cellY, 1e-3));
    lines += blockLine(island, b) + "\n";
    CHECK_THAT(blockLine(island, b), ContainsSubstring(": 100% land, "));
  }
  std::sort(files.begin(), files.end());
  CHECK(filesIn(island.outDir) == files);
  CHECK_THAT(result.out, ContainsSubstring("\ntiles: " + lineAfter(result.out, "tiles: ") + "\n" + lines + "solid: "));
  const auto cutShortBlocks = std::count_if(expected.begin(), expected.end(), [](const auto& b) { return b.cutShort; });
  CHECK_THAT(lineAfter(result.out, "tiles: "),
             EndsWith("; " + std::to_string(expected.size()) + " with land, 0 without, " +
                      std::to_string(cutShortBlocks) + " cut short at the east or south edge"));
  // Squares meet along their whole sides: every vertex on a side the first block shares is in its neighbour too.
  for (const bool vertical : {true, false}) {
    const ExpectedBlock& a = expected[0];
    const ExpectedBlock& b = expected[vertical ? 1 : 2];
    const int line = vertical ? a.rect.lastColumn : a.rect.lastRow;
    std::vector<int> whole;
    for (int k = vertical ? a.rect.firstRow : a.rect.firstColumn; k < (vertical ? a.rect.lastRow : a.rect.lastColumn);
         ++k) {
      whole.push_back(k);
    }
    const std::vector<TopVertex> first = onSharedStretches(island, island.topOf(a), vertical, line, whole);
    const std::vector<TopVertex> second = onSharedStretches(island, island.topOf(b), vertical, line, whole);
    CAPTURE(vertical);
    REQUIRE(first.size() == second.size());
    CHECK(first.size() >= 2);
    for (std::size_t k = 0; k < first.size(); ++k) {
      CHECK_THAT(first[k].x, WithinAbs(second[k].x, 1e-3));
      CHECK_THAT(first[k].y, WithinAbs(second[k].y, 1e-3));
      CHECK(first[k].z == second[k].z);
    }
  }
}

TEST_CASE("a lattice or a selection that cannot be written is refused before anything is read", "[mesh-e2e]") {
  const Island island;
  const auto checkRefused = [&island](const Finished& result, const std::string& problem) {
    CHECK(result.exitCode() == 1);
    CHECK(result.err == "error: " + problem + "\n");
    CHECK(result.out.empty());
    CHECK(std::filesystem::is_empty(island.outDir));
  };
  SECTION("--blocks that is not ROWS:COLS, read before anything else") {
    const Finished result = island.run({"--tile", island.tile(12), "--blocks", "1-2"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "--blocks: must be ROWS:COLS, each part a whole number from 1 or a range such as 3-5, not "
                         "'1-2'");
  }
  SECTION("--blocks beyond the lattice, which is named") {
    const Finished result = island.run({"--tile", island.tile(12), "--blocks", "1-3:2"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "--blocks: '1-3:2' reaches beyond the lattice of 2 rows and 2 columns of blocks");
  }
  SECTION("a lattice of more than 10,000 blocks") {
    // 199 x 199 cells of blocks of one cell each, a tile of 1 m being under half of either cell.
    const Finished result = island.run({"--grid", "200x200", "--tile", "1"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "the lattice is 39601 blocks; the limit is 10000: use a larger --tile");
  }
  SECTION("a block side above 16,384 cells") {
    const Finished result = island.run({"--grid", "16386x2", "--tile", "100000000"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "a block would be 16385 cells wide, but a block side may be at most 16384 cells: use a "
                         "smaller --tile or a coarser --grid");
  }
  SECTION("a footprint without --tile above 16,384 cells a side") {
    const Finished result = island.run({"--grid", "16386x2", "--footprint", "data"});
    INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
    checkRefused(result, "the window is 16385 x 1 cells; a footprint without --tile is limited to 16384 a side: use "
                         "--tile or --grid");
  }
}

namespace {

// The blocks of the island that a run writes, which are those with land, and what Blender is to be told of them: the
// list of their STLs, one to a line in lattice order, and the one line it is to check them with, as the stand-in for
// Blender prints it, from the blocks' triangles added up.
struct WrittenBlocks {
  std::vector<const ExpectedBlock*> blocks;
  std::string list;
  std::string totals;

  WrittenBlocks(const Island& island, const std::vector<ExpectedBlock>& expected) {
    std::uint64_t triangles = 0;
    for (const ExpectedBlock& b : expected) {
      if (!b.land) continue;
      blocks.push_back(&b);
      list += island.blockStl(b).string() + "\n";
      triangles += b.solidTriangles;
    }
    totals = "stl check: " + std::to_string(blocks.size()) + " files, " + std::to_string(triangles) +
             " triangles, 0 non-manifold edges, 0 inverted faces";
  }
};

}  // namespace

TEST_CASE("a tiled run gives Blender every block's STL in one list and repeats its one line of totals",
          "[mesh-e2e]") {
  const Island island;
  const std::vector<ExpectedBlock> expected = island.expected(island.layout(12), true);
  const WrittenBlocks written(island, expected);
  REQUIRE(written.blocks.size() > 1);
  const TempDir bin;
  const TempDir scripts;  // the temporary directory while dem2mesh runs, so that the list's file can be seen to go
  const FakeBlender fake = writeFakeBlender(bin.path);
  const ScopedEnv tmpdir("TMPDIR", scripts.path.string());
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  // What the stand-in prints for one file. For a list it prints the totals of the files in it instead, as Blender does.
  const ScopedEnv check("FAKE_STL_CHECK", "stl check: 100 triangles, 0 non-manifold edges, 0 inverted faces");

  const Finished result = island.runWithBlender(
      {"--blender", fake.executable.string(), "--footprint", "data", "--tile", island.tile(12)});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK(result.err.empty());
  // The summary down to the PLY, then Blender's scene and its one line, which is the last of the output.
  CHECK_THAT(result.out, EndsWith("\nwrote: " + island.ply().string() + "\nblender: " + fake.executable.string() +
                                  "\nwrote: " + island.blend().string() + "\n" + written.totals + "\n"));
  CHECK(occurrences(result.out, "stl check: ") == 1);

  // Blender was given the PLY of the whole area and, in the place of one STL, a list.
  const std::vector<std::string> args = fake.recordedArguments();
  REQUIRE(args.size() == 14);
  CHECK(args[5] == "--ply=" + island.ply().string());
  REQUIRE(args[6].starts_with("--stl-list="));
  CHECK(args[7] == "--blend=" + island.blend().string());
  CHECK(args[8] == "--name=island");
  // The list held every block written, in lattice order, and is gone with the script.
  CHECK(fileHolds(fake.listCopy, written.list));
  CHECK(std::filesystem::is_empty(scripts.path));
  std::vector<std::string> files{island.ply().filename().string(), island.blend().filename().string()};
  for (const ExpectedBlock* b : written.blocks) files.push_back(island.blockStl(*b).filename().string());
  std::sort(files.begin(), files.end());
  CHECK(filesIn(island.outDir) == files);
}

TEST_CASE("a run with one STL gives Blender that file, tiled or not", "[mesh-e2e]") {
  const Island island;
  const TempDir bin;
  const FakeBlender fake = writeFakeBlender(bin.path);
  const ScopedEnv exit("FAKE_EXIT", std::nullopt);
  const std::string line = "stl check: 100 triangles, 0 non-manifold edges, 0 inverted faces";
  const ScopedEnv check("FAKE_STL_CHECK", line);
  Path stl;
  std::vector<std::string> options{"--blender", fake.executable.string(), "--footprint", "data"};
  SECTION("a footprint without tiles") {
    stl = island.stl();
  }
  SECTION("a lattice of a single block, which is still tiled") {
    const std::vector<ExpectedBlock> expected = island.expected(island.layout(30), true);
    REQUIRE(expected.size() == 1);
    REQUIRE(expected.front().land);
    stl = island.blockStl(expected.front());
    options.insert(options.end(), {"--tile", island.tile(30)});
  }
  const Finished result = island.runWithBlender(options);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK_THAT(result.out, EndsWith("\nwrote: " + island.blend().string() + "\n" + line + "\n"));
  const std::vector<std::string> args = fake.recordedArguments();
  REQUIRE(args.size() == 14);
  CHECK(args[6] == "--stl=" + stl.string());
  CHECK_FALSE(std::filesystem::exists(fake.listCopy));
}

TEST_CASE("with Blender installed, every block of a tiled run passes its check in one session",
          "[mesh-e2e][slow]") {
  const std::optional<Path> blender = findBlender(std::nullopt);
  if (!blender) SKIP("Blender not installed");
  const Island island;
  const std::vector<ExpectedBlock> expected = island.expected(island.layout(12), true);
  const WrittenBlocks written(island, expected);
  REQUIRE(written.blocks.size() > 1);

  const Finished result =
      island.runWithBlender({"--footprint", "data", "--tile", island.tile(12), "--verbose"});
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  REQUIRE(result.exitCode() == 0);
  CHECK(result.err.empty());
  CHECK_THAT(result.out, EndsWith("\nwrote: " + island.blend().string() + "\n" + written.totals + "\n"));
  // Blender's log, shown with --verbose: each block imported once, in its turn, with nothing for validation to mend.
  for (const ExpectedBlock* b : written.blocks) {
    CHECK_THAT(result.out, ContainsSubstring("\nblender: imported " + island.blockStl(*b).string() +
                                             " into an empty scene; mesh.validate() found nothing to mend\n"));
  }
  // The one line of the check is there twice, in the log and again at the end, and no block has a line of its own.
  CHECK(occurrences(result.out, "stl check: ") == 2);
  // The scene is saved from the whole area's PLY. Blender 3.0 and later compress a .blend with Zstandard.
  const std::string scene = readFile(island.blend());
  CHECK((scene.starts_with("BLENDER") || scene.starts_with("\x28\xb5\x2f\xfd")));
  std::vector<std::string> files{island.ply().filename().string(), island.blend().filename().string()};
  for (const ExpectedBlock* b : written.blocks) files.push_back(island.blockStl(*b).filename().string());
  std::sort(files.begin(), files.end());
  CHECK(filesIn(island.outDir) == files);
}
