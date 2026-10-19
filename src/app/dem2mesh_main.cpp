// dem2mesh: an elevation GeoTIFF to a printable STL solid and the PLY surface a Blender scene is built from (spec 3 to
// 6). main is the spec's pipeline as a sequence of named steps, each a call into the demmesh library, so that the order
// can be read here and the work is tested there. The summary goes to stdout; every failure is one line on stderr,
// "error: <message>", and exit code 1.

#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include "app/ExitCodes.h"
#include "app/MeshCli.h"
#include "demdl/BoundingBox.h"
#include "mesh/BasePlane.h"
#include "mesh/CropWindow.h"
#include "mesh/GeoTiffGrid.h"
#include "mesh/NoDataFill.h"
#include "mesh/PlyWriter.h"
#include "mesh/Resampler.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"
#include "mesh/TransverseMercator.h"

namespace {

namespace fs = std::filesystem;
using demdl::cli::MeshOptions;
using namespace demdl::mesh;

// Spec 3: an output grid of more vertices than this is refused, its STL being about 2.5 GB at the limit, and one of
// more than kWarnVertices is warned about, with the sizes of its files.
constexpr std::uint64_t kMostVertices = 25'000'000;
constexpr std::uint64_t kWarnVertices = 4'000'000;

// value with `decimals` decimals, sized by a first call so that no value, however large, is cut short.
std::string fixed(double value, int decimals) {
  const int length = std::snprintf(nullptr, 0, "%.*f", decimals, value);
  if (length < 0) return {};
  std::string text(static_cast<std::size_t>(length), '\0');
  std::snprintf(text.data(), text.size() + 1, "%.*f", decimals, value);
  return text;
}

// Metres to the centimetre, without trailing zeros: "60", "-37.9", "10.86". A centimetre is finer than any elevation
// model this reads is accurate to, and a summary of whole numbers reads more easily than one of ".00"s.
std::string metres(double value) {
  std::string text = fixed(value, 2);
  if (text.find('.') != std::string::npos) {
    while (text.back() == '0') text.pop_back();
    if (text.back() == '.') text.pop_back();
  }
  return text == "-0" ? "0" : text;
}

// A number from the file or the command line, to twelve significant digits, which shows what was given without the
// noise of its binary form: "0.00875", "5", "298.257222101".
std::string number(double value) {
  char text[32];
  std::snprintf(text, sizeof text, "%.12g", value);
  return text;
}

// Gives a vector's memory back at once: clear() and assigning {} keep the capacity.
template <typename T>
void release(std::vector<T>& values) {
  std::vector<T>().swap(values);
}

// With --verbose, one line per stage as it finishes, "time: <stage> <seconds> s", so that a slow run shows where.
class StageClock {
 public:
  explicit StageClock(bool print) : print_(print) {}

  void lap(const char* stage) {
    const auto now = std::chrono::steady_clock::now();
    if (print_) {
      std::cout << "time: " << stage << " " << fixed(std::chrono::duration<double>(now - start_).count(), 3)
                << " s\n";
    }
    start_ = now;
  }

 private:
  bool print_;
  std::chrono::steady_clock::time_point start_ = std::chrono::steady_clock::now();
};

// The files a run makes: STEM.stl and STEM.ply here, STEM.blend by Blender. The suffixes are added to the stem as
// typed, so that a stem with a dot in it keeps the dot.
struct Outputs {
  fs::path stl, ply, blend;
};

Outputs outputsFor(const fs::path& stem) {
  return {stem.string() + ".stl", stem.string() + ".ply", stem.string() + ".blend"};
}

// The input must be a file this process can read. Checked before the reader opens it, so that the message names the
// path plainly; what the file holds is for the reader to judge.
void checkInput(const fs::path& input) {
  if (input.empty()) throw std::runtime_error("the input path is empty");
  std::error_code ec;
  const fs::file_status status = fs::status(input, ec);
  if (ec) throw std::runtime_error("cannot read " + input.string() + ": " + ec.message());
  if (!fs::is_regular_file(status)) throw std::runtime_error(input.string() + " is not a regular file");
  if (access(input.c_str(), R_OK) != 0) {
    const int error = errno;
    throw std::runtime_error("cannot read " + input.string() + ": " + std::generic_category().message(error));
  }
}

// The stem's directory, "." when it names none, must exist and take new files, with the same test and message as
// dl_elevation's output: a run that reads a large window should not fail at its very end on what was known at its
// start.
void checkOutputDirectory(const fs::path& stem) {
  fs::path dir = stem.parent_path();
  if (dir.empty()) dir = ".";
  if (access(dir.c_str(), W_OK | X_OK) != 0) {
    const int error = errno;
    throw std::runtime_error("cannot write to " + dir.string() + ": " + std::generic_category().message(error));
  }
}

// Without --overwrite, any of the three outputs already there is refused before anything is read: the .blend too,
// though Blender writes it, since the run would replace it. Anything at the name counts, a dangling link included.
void checkNoOutputExists(const Outputs& outputs) {
  for (const fs::path* path : {&outputs.stl, &outputs.ply, &outputs.blend}) {
    std::error_code ignored;  // a name that cannot be looked at is left to the writers to fail on
    if (fs::exists(fs::symlink_status(*path, ignored))) {
      throw std::runtime_error(path->string() + " exists; use --overwrite");
    }
  }
}

// The output grid in vertices (spec 3).
struct OutputGrid {
  int cols = 0, rows = 0;

  [[nodiscard]] std::uint64_t vertices() const {
    return static_cast<std::uint64_t>(cols) * static_cast<std::uint64_t>(rows);
  }
  [[nodiscard]] std::uint64_t surfaceTriangles() const {
    return 2 * static_cast<std::uint64_t>(cols - 1) * static_cast<std::uint64_t>(rows - 1);
  }
};

// --grid, or one vertex per sample of the window. A height left out follows the window's aspect in metres, so that the
// cells come out square: rows = max(2, round((cols - 1) heightM / widthM) + 1). Worked out in double, because --grid
// takes any int and an extreme aspect can ask for more rows than an int holds; refused, before anything is read, above
// kMostVertices, which leaves both sides within an int.
OutputGrid outputGridFor(const std::optional<demdl::cli::GridSize>& grid, const PixelWindow& window,
                         const WindowExtent& extent) {
  double cols = window.cols;
  double rows = window.rows;
  if (grid) {
    cols = grid->cols;
    rows = grid->rows != 0 ? grid->rows : std::max(2.0, std::round((cols - 1) * extent.heightM / extent.widthM) + 1);
  }
  const double vertices = cols * rows;
  // Asked this way round so that a NaN is refused too.
  if (!(vertices <= static_cast<double>(kMostVertices))) {
    throw std::runtime_error("the output grid is " + fixed(vertices, 0) + " vertices; the limit is " +
                             std::to_string(kMostVertices) + ": use --grid or crop");
  }
  return {static_cast<int>(cols), static_cast<int>(rows)};
}

// What the run has found and made so far, for the summary and the dry run's plan.
struct Facts {
  GridGeometry geometry;
  PixelWindow window;
  WindowExtent extent;
  OutputGrid grid;
  std::uint64_t triangles = 0;  // in the solid, from the formula
  std::uint64_t stlBytes = 0, plyBytes = 0;
  FillReport fill;
  double lowest = 0, highest = 0;  // the window's heights after the fill, before resampling
  double gridSummit = 0;           // the highest height of the resampled grid
  BasePlane base;
  MeshCheck check;
};

// "4000000 vertices" and up: the sizes in megabytes, which is what a user weighs before waiting for them.
void warnOfLargeFiles(const Facts& f) {
  std::cerr << "warning: the output grid is " << f.grid.vertices() << " vertices: the STL will take "
            << fixed(static_cast<double>(f.stlBytes) / 1e6, 0) << " MB and the PLY "
            << fixed(static_cast<double>(f.plyBytes) / 1e6, 0) << " MB\n";
}

// Spec 5, steps 1 to 4, on the window: its heights, read once as float32; no-data filled from the nearest height; the
// range after the fill, which the base plane is chosen from and which comes before resampling, so that surfaceMesh's
// clamp to the level still catches a resampled value that undershoots it; the grid resampled to the output size,
// always, even at full resolution, as the kernel's smoothing is part of the design. The window goes when this returns,
// so that it is never held beside a mesh.
std::vector<float> readTerrain(const MeshOptions& options, Facts& f, StageClock& clock) {
  std::vector<float> heights = readWindow(options.input, f.geometry, f.window);
  clock.lap("read");
  f.fill = fillNearest(heights, f.window.cols, f.window.rows);
  clock.lap("fill");
  const auto [lowest, highest] = std::minmax_element(heights.begin(), heights.end());
  f.lowest = *lowest;
  f.highest = *highest;
  std::vector<float> grid = resampleQuadratic(heights, f.window.cols, f.window.rows, f.grid.cols, f.grid.rows);
  f.gridSummit = *std::max_element(grid.begin(), grid.end());
  clock.lap("resample");
  return grid;
}

// Spec 6: the solid is checked before anything is written. A failure is a bug in this program, not a fault of the
// input, so the message gives the counts for a report.
void selfCheck(const MeshCheck& check, std::uint64_t expectedTriangles) {
  if (check.ok() && check.triangles == expectedTriangles) return;
  std::string counts = std::to_string(check.triangles) + " triangles where " + std::to_string(expectedTriangles) +
                       " were expected, ";
  counts += check.edgesChecked ? std::to_string(check.nonManifoldEdges) + " non-manifold edges" : "edges not checked";
  counts += ", " + std::to_string(check.zeroAreaTriangles) + " triangles without area, signed volume " +
            number(check.signedVolume);
  throw std::runtime_error("mesh self-check failed: " + counts);
}

// The STL's 80-byte header names the tool and what was meshed (spec 5.5), the file name last since the header cuts
// what does not fit, and so never begins "solid", which marks an ASCII STL to some readers.
std::string stlHeader(const MeshOptions& options, const Facts& f) {
  return "dem2mesh " DL_ELEVATION_VERSION " grid " + std::to_string(f.grid.cols) + "x" +
         std::to_string(f.grid.rows) + " window " + std::to_string(f.window.cols) + "x" +
         std::to_string(f.window.rows) + "+" + std::to_string(f.window.col0) + "+" + std::to_string(f.window.row0) +
         " " + options.input.filename().string();
}

// The lines of the summary, which the dry run's plan shares.

std::string inputLine(const MeshOptions& options, const GridGeometry& g) {
  const bool geographic = g.kind == ModelKind::Geographic;
  return "input: " + options.input.string() + ", " + (geographic ? "geographic" : "Transverse Mercator") + ", " +
         std::to_string(g.width) + " x " + std::to_string(g.height) + " samples at " + number(g.dx) + " x " +
         number(g.dy) + (geographic ? " degrees" : " m");
}

std::string windowLine(const Facts& f) { return "window: " + describeWindow(f.geometry, f.window); }

std::string gridLine(const Facts& f) {
  return "grid: " + std::to_string(f.grid.cols) + " x " + std::to_string(f.grid.rows) + " vertices, cell " +
         metres(f.extent.widthM / (f.grid.cols - 1)) + " x " + metres(f.extent.heightM / (f.grid.rows - 1)) + " m";
}

// The fill's part only when there was something to fill.
std::string heightsLine(const Facts& f) {
  std::string line = "heights: " + metres(f.lowest) + " to " + metres(f.highest) + " m";
  if (f.fill.filled > 0) {
    line += " (" + std::to_string(f.fill.filled) + (f.fill.filled == 1 ? " no-data pixel" : " no-data pixels") +
            " filled with " + metres(f.fill.lowestFill);
    if (f.fill.highestFill != f.fill.lowestFill) line += " to " + metres(f.fill.highestFill);
    line += " m)";
  }
  return line;
}

// The B-spline lowers a sharp summit a little (spec 5.4); this shows by how much, so the owner can judge.
std::string summitLine(const Facts& f) {
  return "summit: " + metres(f.highest) + " m in the window, " + metres(f.gridSummit) + " m in the mesh";
}

std::string baseLine(const MeshOptions& options, const Facts& f) {
  const BasePlane& b = f.base;
  std::string why;
  if (b.loweredToLowest) {
    why = "lowered to the lowest point, " + metres(f.lowest) + " m, minus a " + metres(b.marginM) + " m margin";
  } else if (options.base) {
    why = "the requested " + metres(b.levelM) + " m minus a " + metres(b.marginM) + " m margin";
  } else {
    why = "sea level minus a " + metres(b.marginM) + " m margin";
  }
  if (b.plinthApplied) why += ", then pushed down for a " + number(options.plinth) + "-unit plinth";
  return "base: " + metres(b.planeM) + " m (" + why + ")";
}

std::string solidLine(const Facts& f) {
  return "solid: " + std::to_string(f.triangles) + " triangles, STL " + std::to_string(f.stlBytes) + " bytes, PLY " +
         std::to_string(f.plyBytes) + " bytes";
}

// Spec 6 asks that the output say so when the edge check was left out.
std::string edgesNotCheckedLine() {
  return "check: edges not checked above " + std::to_string(kMostTrianglesForEdgeCheck) +
         " triangles; the count, the areas and the volume were";
}

// Task 11 runs Blender at this point; until then every run goes as if --no-blend were given.
std::string blenderLine(const MeshOptions& options) {
  return options.noBlend ? "blender: skipped (--no-blend)" : "blender: skipped (not built yet)";
}

void printSummary(const MeshOptions& options, const Outputs& outputs, const Facts& f) {
  std::cout << inputLine(options, f.geometry) << "\n"
            << windowLine(f) << "\n"
            << gridLine(f) << "\n"
            << heightsLine(f) << "\n"
            << summitLine(f) << "\n"
            << baseLine(options, f) << "\n"
            << solidLine(f) << "\n";
  if (!f.check.edgesChecked) std::cout << edgesNotCheckedLine() << "\n";
  std::cout << "wrote: " << outputs.stl.string() << "\n"
            << "wrote: " << outputs.ply.string() << "\n"
            << blenderLine(options) << "\n";
}

// The plan of spec 3.1, after the "dry run" line that run() printed first: the summary's lines with what the summary
// leaves out (the projection, where the samples sit, the no-data value, the vertical datum, the crop asked for, the
// window in metres) and what would be written instead of "wrote:".
void printPlan(const MeshOptions& options, const Outputs& outputs, const Facts& f) {
  const GridGeometry& g = f.geometry;
  const auto line = [](const std::string& text) { std::cout << "  " << text << "\n"; };
  line(inputLine(options, g));
  if (g.kind == ModelKind::Geographic) {
    line("projection: geographic, longitude and latitude in degrees, turned into metres with a local scale at the "
         "window's middle latitude");
  } else {
    line("projection: Transverse Mercator, central meridian " + number(g.tm.originLongitudeDeg) +
         ", latitude of origin " + number(g.tm.originLatitudeDeg) + ", scale " + number(g.tm.scale) +
         ", false easting " + number(g.tm.falseEasting) + " m, false northing " + number(g.tm.falseNorthing) +
         " m, ellipsoid " + number(g.tm.semiMajorAxis) + " m and 1/" + number(g.tm.inverseFlattening));
  }
  if (g.pixelIsPoint) {
    line("samples: at the pixel positions (RasterPixelIsPoint)");
  } else if (g.dlElevation) {
    line("samples: at the pixels' north-west corners, where dl_elevation's server measures them (the file declares "
         "RasterPixelIsArea)");
  } else {
    line("samples: at the pixels' centres (RasterPixelIsArea)");
  }
  line("no-data: " + (g.noData ? number(*g.noData) + " and NaN" : std::string("NaN only (no GDAL_NODATA)")) +
       ", each filled with the height of the nearest pixel that has one");
  line("vertical: heights taken as metres above sea level");
  if (options.crop) {
    const demdl::BoundingBox& c = *options.crop;
    line("crop: north " + number(c.north) + ", south " + number(c.south) + ", east " + number(c.east) + ", west " +
         number(c.west) + " degrees");
  }
  line(windowLine(f));
  line("extent: " + metres(f.extent.widthM) + " x " + metres(f.extent.heightM) + " m");
  line(gridLine(f));
  line(heightsLine(f));
  line(summitLine(f));
  line(baseLine(options, f));
  line(solidLine(f));
  if (!f.check.edgesChecked) line(edgesNotCheckedLine());
  for (const fs::path* path : {&outputs.stl, &outputs.ply}) {
    std::error_code ignored;
    const bool exists = fs::exists(fs::symlink_status(*path, ignored));
    line("output: " + path->string() + (exists ? " (exists, will be overwritten)" : " (will be created)"));
  }
  line(blenderLine(options));
}

int run(int argc, char** argv) {
  MeshOptions options;
  const demdl::cli::ParseOutcome parsed = demdl::cli::parseMeshArgs(argc, argv, options, std::cout, std::cerr);
  if (parsed.exitNow) return parsed.exitCode;  // help, version, or a usage error the parser has printed
  if (const std::string problem = demdl::cli::validateMeshOptions(options); !problem.empty()) {
    throw std::runtime_error(problem);
  }
  checkInput(options.input);
  checkOutputDirectory(options.outputStem);
  const Outputs outputs = outputsFor(options.outputStem);
  if (!options.overwrite) checkNoOutputExists(outputs);
  // First, so that it stays the first line when --verbose prints the stages' times.
  if (options.dryRun) std::cout << "dry run\n";

  StageClock clock(options.verbose);
  Facts f;
  f.geometry = inspectGeoTiff(options.input);
  f.window = pixelWindowFor(f.geometry, options.crop);
  f.extent = extentInMetres(f.geometry, f.window);
  f.grid = outputGridFor(options.grid, f.window, f.extent);
  f.triangles = solidTriangleCount(f.grid.cols, f.grid.rows);
  f.stlBytes = stlFileBytes(f.triangles);
  f.plyBytes = plyFileBytes(f.grid.vertices(), f.grid.surfaceTriangles());
  if (f.grid.vertices() > kWarnVertices) warnOfLargeFiles(f);
  clock.lap("inspect");

  std::vector<float> grid = readTerrain(options, f, clock);
  f.base = chooseBasePlane(f.lowest, f.highest, options.base, options.zScale, options.scale, options.plinth);
  Mesh surface = surfaceMesh(grid, f.grid.cols, f.grid.rows, f.extent.widthM, f.extent.heightM, options.zScale,
                             options.scale, f.base.levelM);
  release(grid);
  clock.lap("surface");
  // The surface and the solid, which starts with a copy of it, are both held from here until the PLY is written: at
  // the 25,000,000-vertex limit about 1.8 GB.
  const Mesh solid = solidMesh(surface, f.grid.cols, f.grid.rows, static_cast<float>(f.base.planeZ));
  clock.lap("solid");
  f.check = checkSolid(solid);
  selfCheck(f.check, f.triangles);
  clock.lap("check");

  if (options.dryRun) {
    printPlan(options, outputs, f);
    return demdl::cli::kExitComplete;
  }
  writeBinaryPly(surface, outputs.ply, f.extent.widthM * options.scale, f.extent.heightM * options.scale);
  release(surface.vertices);
  release(surface.triangles);
  clock.lap("write PLY");
  writeBinaryStl(solid, outputs.stl, stlHeader(options, f));
  clock.lap("write STL");
  printSummary(options, outputs, f);
  return demdl::cli::kExitComplete;
}

}  // namespace

int main(int argc, char** argv) {
  // Every failure, the library's included, ends here as one line. std::exception, not only std::runtime_error: the
  // crop window throws std::invalid_argument, and an allocation std::bad_alloc.
  try {
    return run(argc, argv);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
  } catch (...) {
    std::cerr << "error: unknown exception\n";
  }
  return demdl::cli::kExitError;
}
