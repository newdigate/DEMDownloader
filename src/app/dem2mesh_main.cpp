// dem2mesh: an elevation GeoTIFF to a printable STL solid, the PLY surface, and the Blender scene built from it (spec 3
// to 7), the surface simplified to within a tolerance when asked (spec 4.1 to 4.7). A section numbered 4.1 to 4.7 is
// the simplify-and-tiles design's, the only one of the two designs with such sections; the other numbers are the
// dem-to-mesh design's. main is the spec's pipeline as a sequence of named steps, each a call into the demmesh library,
// so that the order can be read here and the work is tested there. The summary goes to stdout; every failure is one
// line on stderr, "error: <message>", followed by Blender's output when Blender failed, and exit code 1.

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
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "app/ExitCodes.h"
#include "app/MeshCli.h"
#include "demdl/BoundingBox.h"
#include "mesh/BasePlane.h"
#include "mesh/BlenderRunner.h"
#include "mesh/CropWindow.h"
#include "mesh/GeoTiffGrid.h"
#include "mesh/NoDataFill.h"
#include "mesh/PlyWriter.h"
#include "mesh/Resampler.h"
#include "mesh/Simplifier.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"
#include "mesh/TransverseMercator.h"

namespace {

namespace fs = std::filesystem;
using demdl::cli::MeshOptions;
using namespace demdl::mesh;

// Spec 3 and 4.2: a mesh of more vertices than this is refused, its STL being about 2.5 GB at the limit, and one of
// more than kWarnVertices is warned about, with the sizes of its files. Both count the mesh that is written: the output
// grid, or with --tolerance the simplified mesh, whose size is known only once the grid has been simplified.
constexpr std::uint64_t kMostVertices = 25'000'000;
constexpr std::uint64_t kWarnVertices = 4'000'000;

// With --tolerance the output grid is simplified, not written, so it may be as large as the window (spec 4.2): it is
// refused only above the most an int holds, which still leaves both its sides within an int. At that limit its heights
// are 8.6 GB of float32.
constexpr std::uint64_t kMostVerticesToSimplify = std::numeric_limits<int>::max();

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

// A Blender the user named, with --blender or BLENDER, that cannot be started is refused before anything is read, in a
// dry run too, rather than once the STL and PLY are written. The search that runs when none is named is left to find
// what it finds; a run that finds nothing still writes the STL and PLY before it fails.
void checkNamedBlender(const MeshOptions& options) {
  const std::optional<fs::path> named = namedBlender(options.blender);
  if (!named) return;
  if (const std::optional<std::string> problem = blenderStartProblem(*named)) {
    throw std::runtime_error((options.blender ? "--blender " : "BLENDER=") + named->string() + ": " + *problem);
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
// kMostVertices, or when the grid is to be simplified (simplify) above kMostVerticesToSimplify, either of which leaves
// both sides within an int.
OutputGrid outputGridFor(const std::optional<demdl::cli::GridSize>& grid, const PixelWindow& window,
                         const WindowExtent& extent, bool simplify) {
  double cols = window.cols;
  double rows = window.rows;
  if (grid) {
    cols = grid->cols;
    rows = grid->rows != 0 ? grid->rows : std::max(2.0, std::round((cols - 1) * extent.heightM / extent.widthM) + 1);
  }
  const double vertices = cols * rows;
  const std::uint64_t limit = simplify ? kMostVerticesToSimplify : kMostVertices;
  // Asked this way round so that a NaN is refused too.
  if (!(vertices <= static_cast<double>(limit))) {
    throw std::runtime_error("the output grid is " + fixed(vertices, 0) + " vertices; " +
                             (simplify ? "with --tolerance " : "") + "the limit is " + std::to_string(limit) +
                             ": use --grid or crop");
  }
  return {static_cast<int>(cols), static_cast<int>(rows)};
}

// What the run has found and made so far, for the summary and the dry run's plan.
struct Facts {
  GridGeometry geometry;
  PixelWindow window;
  WindowExtent extent;
  OutputGrid grid;
  std::uint64_t surfaceVertices = 0, surfaceTriangles = 0;  // the surface written: the grid's, or the simplified one
  std::uint64_t triangles = 0;                              // in the solid built on the surface
  std::uint64_t stlBytes = 0, plyBytes = 0;
  FillReport fill;
  double lowest = 0, highest = 0;  // the window's heights after the fill, before resampling
  double meshSummit = 0;           // the surface's highest height: the resampled grid's, or the simplified mesh's
  double surfaceLowZ = 0, surfaceHighZ = 0;  // the surface's lowest and highest z in model units, for Blender's camera
  BasePlane base;
  MeshCheck check;
};

// Blender's peak resident memory, estimated from the solid's triangle count, as "<gigabytes> GB" to one decimal: about
// 670 bytes a triangle over a floor of 250 MB, from Blender 4.5.12, which peaked at 583 MB with 511,004 triangles and
// at 1,606 MB with 2,042,004. Just above the warning's 4,000,000 vertices that is about 5.6 GB, and at the limit about
// 34 GB.
std::string blenderMemory(std::uint64_t triangles) {
  return fixed((250e6 + 670.0 * static_cast<double>(triangles)) / 1e9, 1) + " GB";
}

// What will be written: the surface of `vertices` and `triangles`, which is the PLY, and the solid of `solidTriangles`
// built on it, which is the STL.
void countOutputs(Facts& f, std::uint64_t vertices, std::uint64_t triangles, std::uint64_t solidTriangles) {
  f.surfaceVertices = vertices;
  f.surfaceTriangles = triangles;
  f.triangles = solidTriangles;
  f.stlBytes = stlFileBytes(solidTriangles);
  f.plyBytes = plyFileBytes(vertices, triangles);
}

// "4000000 vertices" and up: the sizes in megabytes, which is what a user weighs before waiting for them, and the
// memory Blender will want, which can be more than the machine has, unless --no-blend leaves Blender out. mesh names
// what has that many vertices: "the output grid", or with --tolerance "the simplified mesh".
void warnOfLargeFiles(const MeshOptions& options, std::string_view mesh, const Facts& f) {
  const std::string stl = fixed(static_cast<double>(f.stlBytes) / 1e6, 0) + " MB";
  const std::string ply = fixed(static_cast<double>(f.plyBytes) / 1e6, 0) + " MB";
  std::cerr << "warning: " << mesh << " is " << f.surfaceVertices << " vertices: the STL will take " << stl;
  if (options.noBlend) {
    std::cerr << " and the PLY " << ply << "\n";
  } else {
    std::cerr << ", the PLY " << ply << ", and Blender about " << blenderMemory(f.triangles)
              << " of memory (--no-blend skips it)\n";
  }
}

// Spec 5, steps 1 to 4, on the window: its heights, read once as float32; no-data filled from the nearest height; the
// range after the fill, which the base plane is chosen from, and which the resampled grid never leaves, so that the
// surface's clamp to the level is only a guard; the grid resampled to the output size, always, even at full
// resolution, as the kernel's smoothing is part of the design. The window goes when this returns, so that it is never
// held beside a mesh.
std::vector<float> readTerrain(const MeshOptions& options, Facts& f, StageClock& clock) {
  std::vector<float> heights = readWindow(options.input, f.geometry, f.window);
  clock.lap("read");
  f.fill = fillNearest(heights, f.window.cols, f.window.rows);
  clock.lap("fill");
  const auto [lowest, highest] = std::minmax_element(heights.begin(), heights.end());
  f.lowest = *lowest;
  f.highest = *highest;
  std::vector<float> grid = resampleQuadratic(heights, f.window.cols, f.window.rows, f.grid.cols, f.grid.rows);
  f.meshSummit = *std::max_element(grid.begin(), grid.end());  // the grid's, until it is simplified
  clock.lap("resample");
  return grid;
}

// --tolerance is in model units, after --z-scale and --scale (spec 4.1); the simplifier takes the heights' metres.
double toleranceInMetres(const MeshOptions& options) { return *options.tolerance / (options.zScale * options.scale); }

// The terrain's top surface and its outline, which the solid is built on.
struct Surface {
  Mesh mesh;
  Boundary boundary;
};

// Spec 4.2: the grid becomes the surface as it stands (stage "surface"), or with --tolerance is simplified to a TIN
// within it (stage "simplify"), and goes as soon as the surface is made, so that nothing per sample outlives this. What
// will be written is counted from what was made. Only now is a simplified mesh's size known, so with --tolerance it is
// refused here above kMostVertices and warned about above kWarnVertices, as a grid is before anything is read.
Surface makeSurface(const MeshOptions& options, Facts& f, std::vector<float> grid, StageClock& clock) {
  Surface surface;
  if (options.tolerance) {
    SimplifiedSurface tin = simplifyGrid(grid, f.grid.cols, f.grid.rows, toleranceInMetres(options), f.extent.widthM,
                                         f.extent.heightM, options.zScale, options.scale, f.base.levelM);
    // The grid's summit is a vertex only if the tolerance needs it there, so the mesh's summit is its highest vertex,
    // back in metres.
    const auto summit = std::max_element(tin.surface.vertices.begin(), tin.surface.vertices.end(),
                                         [](const Vec3& a, const Vec3& b) { return a.z < b.z; });
    f.meshSummit = static_cast<double>(summit->z) / (options.zScale * options.scale);
    surface = {std::move(tin.surface), std::move(tin.boundary)};
  } else {
    surface = {surfaceMesh(grid, f.grid.cols, f.grid.rows, f.extent.widthM, f.extent.heightM, options.zScale,
                           options.scale, f.base.levelM),
               gridBoundary(f.grid.cols, f.grid.rows)};
  }
  release(grid);
  const std::uint64_t triangles = surface.mesh.triangles.size();
  countOutputs(f, surface.mesh.vertices.size(), triangles, solidTriangleCount(surface.boundary, triangles));
  clock.lap(options.tolerance ? "simplify" : "surface");
  if (options.tolerance) {
    if (f.surfaceVertices > kMostVertices) {
      throw std::runtime_error("the simplified mesh is " + std::to_string(f.surfaceVertices) +
                               " vertices; the limit is " + std::to_string(kMostVertices) +
                               ": raise --tolerance, or use --grid or crop");
    }
    if (f.surfaceVertices > kWarnVertices) warnOfLargeFiles(options, "the simplified mesh", f);
  }
  return surface;
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

// The crop asked for, in degrees as given, which the window line cannot show for a Transverse Mercator file, its
// window being in metres.
std::string cropLine(const demdl::BoundingBox& crop) {
  return "crop: north " + number(crop.north) + ", south " + number(crop.south) + ", east " + number(crop.east) +
         ", west " + number(crop.west) + " degrees";
}

std::string windowLine(const Facts& f) { return "window: " + describeWindow(f.geometry, f.window); }

std::string gridLine(const Facts& f) {
  return "grid: " + std::to_string(f.grid.cols) + " x " + std::to_string(f.grid.rows) + " vertices, cell " +
         metres(f.extent.widthM / (f.grid.cols - 1)) + " x " + metres(f.extent.heightM / (f.grid.rows - 1)) + " m";
}

// The plan's memory with --tolerance (spec 4.2), where the grid can be far larger than the mesh written: the window and
// the grid, each held once as float32, and both at once while the one is resampled into the other; the simplified mesh
// grows with what it keeps, not with the grid. To the nearest megabyte, or once that is 1,000 or more, in gigabytes to
// one decimal.
std::string memoryLine(const Facts& f) {
  const std::uint64_t samples = static_cast<std::uint64_t>(f.window.cols) * static_cast<std::uint64_t>(f.window.rows);
  const double megabytes = 4.0 * static_cast<double>(samples + f.grid.vertices()) / 1e6;
  const std::string about = megabytes < 999.5 ? fixed(megabytes, 0) + " MB" : fixed(megabytes / 1000, 1) + " GB";
  return "memory: window " + std::to_string(samples) + " samples and grid " + std::to_string(f.grid.vertices()) +
         " vertices, 4 bytes each: about " + about;
}

// Spec 4.7: the tolerance in model units as given and in the metres of terrain they stand for, then the grid's surface
// against the simplified one.
std::string simplifyLine(const MeshOptions& options, const Facts& f) {
  return "simplify: tolerance " + number(*options.tolerance) + " units (" + metres(toleranceInMetres(options)) +
         " m), " + std::to_string(f.grid.vertices()) + " grid vertices to " + std::to_string(f.surfaceVertices) + ", " +
         std::to_string(f.grid.surfaceTriangles()) + " triangles to " + std::to_string(f.surfaceTriangles);
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

// The B-spline lowers a sharp summit a little (spec 5.4), and a simplification may lower it by up to its tolerance;
// this shows by how much, so the owner can judge.
std::string summitLine(const Facts& f) {
  return "summit: " + metres(f.highest) + " m in the window, " + metres(f.meshSummit) + " m in the mesh";
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

// What to do about a missing Blender (spec 7.1), for its error and for the dry run's plan.
constexpr std::string_view kBlenderAdvice = "pass --blender, set BLENDER, or install it at /Applications/Blender.app";

// The dry run's line for spec 3.1's "which Blender would be run" and about how much memory it will need; blender is
// what findBlender gave.
std::string plannedBlenderLine(const MeshOptions& options, const Facts& f, const std::optional<fs::path>& blender) {
  if (options.noBlend) return "blender: skipped (--no-blend)";
  if (!blender) {
    return "blender: not found (a run would fail after writing the STL and PLY: " + std::string(kBlenderAdvice) + ")";
  }
  return "blender: " + blender->string() + " (will be run; about " + blenderMemory(f.triangles) + " of memory)";
}

// The summary down to the files dem2mesh writes itself. A line about Blender follows, from run() or makeScene().
void printSummary(const MeshOptions& options, const Outputs& outputs, const Facts& f) {
  std::cout << inputLine(options, f.geometry) << "\n";
  if (options.crop) std::cout << cropLine(*options.crop) << "\n";
  std::cout << windowLine(f) << "\n" << gridLine(f) << "\n";
  if (options.tolerance) std::cout << simplifyLine(options, f) << "\n";
  std::cout << heightsLine(f) << "\n"
            << summitLine(f) << "\n"
            << baseLine(options, f) << "\n"
            << solidLine(f) << "\n";
  if (!f.check.edgesChecked) std::cout << edgesNotCheckedLine() << "\n";
  std::cout << "wrote: " << outputs.stl.string() << "\n"
            << "wrote: " << outputs.ply.string() << "\n";
}

// The plan of spec 3.1, after the "dry run" line that run() printed first: the summary's lines with what the summary
// leaves out (the projection, where the samples sit, the no-data value, the vertical datum, the window in metres) and
// what would be written instead of "wrote:".
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
  if (options.crop) line(cropLine(*options.crop));
  line(windowLine(f));
  line("extent: " + metres(f.extent.widthM) + " x " + metres(f.extent.heightM) + " m");
  line(gridLine(f));
  if (options.tolerance) {
    line(memoryLine(f));
    line(simplifyLine(options, f));
  }
  line(heightsLine(f));
  line(summitLine(f));
  line(baseLine(options, f));
  line(solidLine(f));
  if (!f.check.edgesChecked) line(edgesNotCheckedLine());
  const std::optional<fs::path> blender = options.noBlend ? std::nullopt : findBlender(options.blender);
  std::vector<const fs::path*> written{&outputs.stl, &outputs.ply};
  if (blender) written.push_back(&outputs.blend);
  for (const fs::path* path : written) {
    std::error_code ignored;
    const bool exists = fs::exists(fs::symlink_status(*path, ignored));
    line("output: " + path->string() + (exists ? " (exists, will be overwritten)" : " (will be created)"));
  }
  line(plannedBlenderLine(options, f, blender));
}

// The first line of text that begins with prefix, without its newline.
std::optional<std::string> lineStartingWith(const std::string& text, std::string_view prefix) {
  for (std::size_t start = 0; start < text.size();) {
    const std::size_t newline = text.find('\n', start);
    const std::size_t end = newline == std::string::npos ? text.size() : newline;
    const std::string_view line(text.data() + start, end - start);
    if (line.starts_with(prefix)) return std::string(line);
    start = end + 1;
  }
  return std::nullopt;
}

// text, then a newline if it does not end in one, so that what follows starts a line of its own.
void printLines(std::ostream& out, const std::string& text) {
  out << text;
  if (!text.empty() && text.back() != '\n') out << "\n";
}

// The error, then what Blender printed, which says why; exit 1.
int blenderFailed(const std::string& message, const std::string& output) {
  std::cerr << "error: " << message << "\n";
  printLines(std::cerr, output);
  return demdl::cli::kExitError;
}

// Spec 7: once the STL and PLY are written, which stay whatever happens here, Blender builds the scene from the PLY,
// saves it as the .blend and checks the STL. The owner asked for the .blend, so a Blender that is missing or fails
// fails the run. Exit 0 alone does not prove that the script ran: Blender exits 0 when it cannot read the script, or
// after an exception in it. So the run succeeds only when the output holds the script's "stl check:" line, which it
// prints last, and the .blend is there. An old .blend, there only under --overwrite, is removed first, before Blender
// is looked for, so that the .blend found afterwards is this Blender's, and a run whose Blender fails or is not found
// leaves no old scene beside the new STL and PLY.
int makeScene(const MeshOptions& options, const Outputs& outputs, const Facts& f, StageClock& clock) {
  std::error_code removal;
  fs::remove(outputs.blend, removal);  // no error when there is none
  if (removal) throw std::runtime_error("cannot remove " + outputs.blend.string() + ": " + removal.message());
  const std::optional<fs::path> blender = findBlender(options.blender);
  if (!blender) {
    std::cout << "blender: not found" << std::endl;
    throw std::runtime_error("Blender not found: " + std::string(kBlenderAdvice));
  }
  // Flushed, as Blender takes seconds, or a minute on a large mesh, and this says what the wait is for.
  std::cout << "blender: " << blender->string() << std::endl;
  const BlenderJob job{outputs.ply,
                       outputs.stl,
                       outputs.blend,
                       options.outputStem.filename().string(),
                       f.extent.widthM * options.scale,
                       f.extent.heightM * options.scale,
                       f.surfaceLowZ,
                       f.surfaceHighZ,
                       options.scale};
  const BlenderResult result = runBlender(*blender, job);
  if (result.exitCode != 0) {
    return blenderFailed("Blender failed (exit " + std::to_string(result.exitCode) + ")", result.output);
  }
  const std::optional<std::string> check = lineStartingWith(result.output, "stl check: ");
  if (!check) return blenderFailed("Blender exited 0 but did not report the STL check", result.output);
  std::error_code ignored;
  if (!fs::is_regular_file(outputs.blend, ignored)) {
    return blenderFailed("Blender exited 0 but did not write " + outputs.blend.string(), result.output);
  }
  if (options.verbose) printLines(std::cout, result.output);
  clock.lap("blender");
  std::cout << "wrote: " << outputs.blend.string() << "\n" << *check << "\n";
  return demdl::cli::kExitComplete;
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
  if (!options.noBlend) checkNamedBlender(options);
  // First, so that it stays the first line when --verbose prints the stages' times.
  if (options.dryRun) std::cout << "dry run\n";

  StageClock clock(options.verbose);
  Facts f;
  f.geometry = inspectGeoTiff(options.input);
  f.window = pixelWindowFor(f.geometry, options.crop);
  f.extent = extentInMetres(f.geometry, f.window);
  f.grid = outputGridFor(options.grid, f.window, f.extent, options.tolerance.has_value());
  // Without --tolerance the grid is what will be written, so its files are known before anything is read and a large
  // one is warned about first.
  if (!options.tolerance) {
    countOutputs(f, f.grid.vertices(), f.grid.surfaceTriangles(), solidTriangleCount(f.grid.cols, f.grid.rows));
    if (f.surfaceVertices > kWarnVertices) warnOfLargeFiles(options, "the output grid", f);
  }
  clock.lap("inspect");

  std::vector<float> grid = readTerrain(options, f, clock);
  f.base = chooseBasePlane(f.lowest, f.highest, options.base, options.zScale, options.scale, options.plinth);
  Surface surface = makeSurface(options, f, std::move(grid), clock);
  // The surface and the solid, which starts with a copy of it, are both held from here until the PLY is written: at
  // the 25,000,000-vertex limit about 1.8 GB. The solid goes once the STL is written, before Blender starts, which
  // needs memory of its own (blenderMemory).
  Mesh solid = solidFromSurface(surface.mesh, surface.boundary, static_cast<float>(f.base.planeZ));
  clock.lap("solid");
  f.check = checkSolid(solid);
  selfCheck(f.check, f.triangles);
  clock.lap("check");

  if (options.dryRun) {
    printPlan(options, outputs, f);
    return demdl::cli::kExitComplete;
  }
  const auto [low, high] = std::minmax_element(surface.mesh.vertices.begin(), surface.mesh.vertices.end(),
                                               [](const Vec3& a, const Vec3& b) { return a.z < b.z; });
  f.surfaceLowZ = low->z;
  f.surfaceHighZ = high->z;
  writeBinaryPly(surface.mesh, outputs.ply, f.extent.widthM * options.scale, f.extent.heightM * options.scale);
  release(surface.mesh.vertices);
  release(surface.mesh.triangles);
  clock.lap("write PLY");
  writeBinaryStl(solid, outputs.stl, stlHeader(options, f));
  release(solid.vertices);
  release(solid.triangles);
  clock.lap("write STL");
  printSummary(options, outputs, f);
  if (options.noBlend) {
    std::cout << "blender: skipped (--no-blend)\n";
    return demdl::cli::kExitComplete;
  }
  return makeScene(options, outputs, f, clock);
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
