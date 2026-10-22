// dem2mesh: an elevation GeoTIFF to a printable STL solid, the PLY surface, and the Blender scene built from it (spec 3
// to 7), the surface simplified to within a tolerance when asked (spec 4.1 to 4.7), and the solid cut off where the
// data ends and split into blocks when asked (Part B 5.1 to 5.5). A section numbered 4.1 to 4.7 is the
// simplify-and-tiles design's, the only one of the two designs with such sections, and one cited as "Part B 5.x" is
// that design's too; the other numbers are the dem-to-mesh design's. A deviation or a ruling is the Part B plan's
// (docs/superpowers/plans/2026-10-22-footprints-and-tiles.md). main is the spec's pipeline as a sequence of named
// steps, each a call into the demmesh library, so that the order can be read here and the work is tested there. The
// summary goes to stdout; every failure is one line on stderr, "error: <message>", followed by Blender's output when
// Blender failed, and exit code 1.

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
#include <set>
#include <stdexcept>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>
#include <vector>

#include "app/CliChecks.h"
#include "app/ExitCodes.h"
#include "app/MeshCli.h"
#include "demdl/BoundingBox.h"
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

namespace {

namespace fs = std::filesystem;
using demdl::cli::MeshOptions;
using namespace demdl::mesh;

// Spec 3 and 4.2: a mesh of more vertices than this is refused, its STL being about 2.5 GB at the limit, and one of
// more than kWarnVertices is warned about, with the sizes of its files. Both count the mesh that is written: the output
// grid, or with --tolerance, and in Part B's modes, the simplified mesh, whose size is known only once the grid has
// been simplified; the simplifier is given the limit as well, so that it stops a mesh that has passed it while making
// it (the plan's ruling on the cap). Without --tolerance the output grid is held to the limit in Part B's modes too,
// since simplified at 0 it keeps nearly every sample.
constexpr std::uint64_t kMostVertices = 25'000'000;
constexpr std::uint64_t kWarnVertices = 4'000'000;

// With --tolerance the output grid is simplified, not written, so it may be as large as the window (spec 4.2): it is
// refused only above the most an int holds, which still leaves both its sides within an int. At that limit its heights
// are 8.6 GB of float32. Only the flag lifts the grid's limit to this: Part B's modes simplify without it too, at 0,
// and keep kMostVertices on the grid (the plan's ruling on the cap).
constexpr std::uint64_t kMostVerticesToSimplify = std::numeric_limits<int>::max();

// Part B 5.3: a lattice of more blocks than this is refused before anything is read, since a --tile of a few cells
// would otherwise write millions of files. Counted in 64 bits.
constexpr std::uint64_t kMostBlocks = 10'000;

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

// Whether anything is at path, a dangling link included. A name that cannot be looked at counts as nothing, and is left
// to the writers to fail on.
bool anythingAt(const fs::path& path) {
  std::error_code ignored;
  return fs::exists(fs::symlink_status(path, ignored));
}

// An output that is there already, refused when --overwrite was not given.
void refuseExisting(const fs::path& path) {
  if (anythingAt(path)) throw std::runtime_error(path.string() + " exists; use --overwrite");
}

// Without --overwrite, any of the three outputs already there is refused before anything is read: the .blend too,
// though Blender writes it, since the run would replace it. Anything at the name counts, a dangling link included.
void checkNoOutputExists(const Outputs& outputs) {
  for (const fs::path* path : {&outputs.stl, &outputs.ply, &outputs.blend}) refuseExisting(*path);
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
// kMostVertices, or when --tolerance was given (toleranceGiven) above kMostVerticesToSimplify, either of which leaves
// both sides within an int. Part B's modes (inBlocks) simplify at 0 without the flag, which keeps nearly every sample,
// so their grid has kMostVertices too, and the refusal names the flag that lifts it (the plan's ruling on the cap); the
// message of today's path stays as it was (spec 6).
OutputGrid outputGridFor(const std::optional<demdl::cli::GridSize>& grid, const PixelWindow& window,
                         const WindowExtent& extent, bool toleranceGiven, bool inBlocks) {
  double cols = window.cols;
  double rows = window.rows;
  if (grid) {
    cols = grid->cols;
    rows = grid->rows != 0 ? grid->rows : std::max(2.0, std::round((cols - 1) * extent.heightM / extent.widthM) + 1);
  }
  const double vertices = cols * rows;
  const std::uint64_t limit = toleranceGiven ? kMostVerticesToSimplify : kMostVertices;
  // Asked this way round so that a NaN is refused too.
  if (!(vertices <= static_cast<double>(limit))) {
    const std::string size = "the output grid is " + fixed(vertices, 0) + " vertices; ";
    const std::string most = "the limit is " + std::to_string(limit) + ": ";
    if (toleranceGiven) throw std::runtime_error(size + "with --tolerance " + most + "use --grid or crop");
    if (inBlocks) {
      throw std::runtime_error(size + "without --tolerance " + most + "give --tolerance, or use --grid or crop");
    }
    throw std::runtime_error(size + most + "use --grid or crop");
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
  MeshCheck check;  // the solid's; in Part B's modes only edgesChecked, false when a block's edges went unchecked
  std::vector<fs::path> stls;           // the STLs written, in lattice order: STEM.stl alone but with --tile
  std::vector<std::string> blockLines;  // Part B 5.4's lines on the blocks, after base: (none without blocks)
};

// --footprint data: the solid stands only where the DEM has data (Part B 5.2).
bool footprintOfData(const MeshOptions& options) { return options.footprint == MeshOptions::Footprint::Data; }

// Part B's modes, which mesh the window block by block (Part B 5.3): --tile, and --footprint data, which without --tile
// is one block of the whole grid (deviation 2). Without either, the run is today's, byte for byte (spec 6).
bool meshesInBlocks(const MeshOptions& options) { return options.tile.has_value() || footprintOfData(options); }

// The blocks of a run in Part B's modes, laid out from the grid before anything is read (Part B 5.3): the lattice, the
// selection --blocks makes from it, and each block's name and STL, row by row from the north-west. With --tile a block
// writes STEM_rRR_cCC.stl; the one block of --footprint data without --tile writes STEM.stl.
struct BlockPlan {
  bool tiled = false;      // --tile
  bool footprint = false;  // --footprint data: a block holds the pieces of the data; otherwise it is a whole square
  TileLayout layout{};
  BlockSelection selection;
  std::vector<std::string> names;  // "r03_c05"
  std::vector<fs::path> stls;
};

// The footprint of --footprint data on the output grid (Part B 5.2): footprintCells' cells, a byte each, and the
// corner contacts it joined, which each block counts within itself.
struct FootprintGrid {
  std::vector<std::uint8_t> cells;
  std::vector<GridPoint> joined;
};

// A block to be written, as the simplifier left it: its place in the lattice, its part of the whole-area surface, and
// what the summary says of it.
struct SimplifiedBlock {
  std::size_t index = 0;  // in the lattice, row by row from the north-west
  BlockRect rect{};
  BlockResult result;
  std::uint64_t cells = 0;      // the block's
  std::uint64_t landCells = 0;  // its kept pieces', or every cell of a square
  std::uint64_t cornerContacts = 0;
  std::uint64_t solidTriangles = 0;  // in the solid its outline makes
};

// A piece of footprint under --min-piece in a selected block, dropped and listed (Part B 5.2).
struct DroppedPiece {
  std::size_t block = 0;
  std::uint64_t cells = 0;
};

// What the blocks come to (Part B 5.4): the whole-area surface, which is the PLY, the blocks to write in lattice order,
// and the others as the summary lists them. withLand and cutShort count the whole lattice, selected or not.
struct Blocks {
  Mesh surface;
  std::vector<SimplifiedBlock> written;
  std::vector<std::size_t> skipped;      // selected, with no piece kept
  std::vector<std::size_t> notSelected;  // outside --blocks
  std::vector<DroppedPiece> dropped;
  std::vector<std::string> stale;  // the stem's block files that this run leaves behind, by name (staleFiles)
  std::uint64_t withLand = 0, cutShort = 0;
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
// what has that many vertices: "the output grid", or with --tolerance "the simplified mesh". With --tile the STLs are
// the blocks', their sizes added up.
void warnOfLargeFiles(const MeshOptions& options, std::string_view mesh, const Facts& f) {
  const std::string stl = fixed(static_cast<double>(f.stlBytes) / 1e6, 0) + " MB";
  const std::string ply = fixed(static_cast<double>(f.plyBytes) / 1e6, 0) + " MB";
  std::cerr << "warning: " << mesh << " is " << f.surfaceVertices << " vertices: "
            << (options.tile ? "the STLs" : "the STL") << " will take " << stl;
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
//
// With footprint, for --footprint data (Part B 5.2), the window's data is taken as the mask before the fill gives every
// no-data sample a height, so that the fill copies data heights and the range is the data's (the plan's ruling on the
// base plane); the mask is carried to the grid by the nearest sample, which at full resolution is the mask as it
// stands; and the grid's cells with data at all four corners are the footprint (footprintCells). The window goes
// before the cells are made and the mask once they are, each a byte a cell, 254 MB on the City's whole file.
std::vector<float> readTerrain(const MeshOptions& options, Facts& f, FootprintGrid* footprint, StageClock& clock) {
  std::vector<float> heights = readWindow(options.input, f.geometry, f.window);
  clock.lap("read");
  std::vector<std::uint8_t> mask;
  if (footprint != nullptr) mask = dataMaskOf(heights);
  f.fill = fillNearest(heights, f.window.cols, f.window.rows);
  clock.lap("fill");
  const auto [lowest, highest] = std::minmax_element(heights.begin(), heights.end());
  f.lowest = *lowest;
  f.highest = *highest;
  std::vector<float> grid = resampleQuadratic(heights, f.window.cols, f.window.rows, f.grid.cols, f.grid.rows);
  f.meshSummit = *std::max_element(grid.begin(), grid.end());  // the grid's, until it is simplified
  clock.lap("resample");
  if (footprint != nullptr) {
    release(heights);
    if (f.grid.cols != f.window.cols || f.grid.rows != f.window.rows) {
      mask = resampleMaskNearest(mask, f.window.cols, f.window.rows, f.grid.cols, f.grid.rows);
    }
    footprint->cells = footprintCells(mask, f.grid.cols, f.grid.rows, &footprint->joined);
    clock.lap("footprint");
  }
  return grid;
}

// --tolerance is in model units, after --z-scale and --scale (spec 4.1); the simplifier takes the heights' metres.
double toleranceInMetres(const MeshOptions& options) { return *options.tolerance / (options.zScale * options.scale); }

// The terrain's top surface and its outline, which the solid is built on.
struct Surface {
  Mesh mesh;
  Boundary boundary;
};

// Spec 4.2: with --tolerance the mesh written is known only once it is simplified, so only then is it refused above
// kMostVertices, with nothing written yet, and warned about above kWarnVertices, as a grid is before anything is read.
// The simplifier, given kMostVertices, stops a mesh while making it once it has passed the limit, by up to the samples
// round a block's edge, so this exact count is what refuses one that ends just above it. In Part B's modes the mesh is
// the whole-area surface, with or without --tolerance (the plan's rulings on the tolerance and the cap).
void limitSimplifiedMesh(const MeshOptions& options, const Facts& f) {
  if (f.surfaceVertices > kMostVertices) {
    throw std::runtime_error("the simplified mesh is " + std::to_string(f.surfaceVertices) +
                             " vertices; the limit is " + std::to_string(kMostVertices) +
                             ": raise --tolerance, or use --grid or crop");
  }
  if (f.surfaceVertices > kWarnVertices) warnOfLargeFiles(options, "the simplified mesh", f);
}

// Spec 4.2: the grid becomes the surface as it stands (stage "surface"), or with --tolerance is simplified to a TIN
// within it (stage "simplify"), and goes as soon as the surface is made, so that nothing per sample outlives this. What
// will be written is counted from what was made, and with --tolerance limited, by the simplifier while it works and
// then exactly (limitSimplifiedMesh).
Surface makeSurface(const MeshOptions& options, Facts& f, std::vector<float> grid, StageClock& clock) {
  Surface surface;
  if (options.tolerance) {
    SimplifiedSurface tin = simplifyGrid(grid, f.grid.cols, f.grid.rows, toleranceInMetres(options), f.extent.widthM,
                                         f.extent.heightM, options.zScale, options.scale, f.base.levelM, kChunkCells,
                                         kMostVertices);
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
  if (options.tolerance) limitSimplifiedMesh(options, f);
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

// Spec 4.3 and 6: the surface is checked before the solid is built on it, on both paths, so that a bad one is refused
// as early as possible. checkSolid sees the edges and the total volume, and a top triangle that float rounding has
// turned over keeps both, so this counts the triangles that do not face up (downwardFacingTriangles). The regular grid
// cannot fail it, so a failure is a bug in the simplifier, and the message gives the count for a report.
void checkSurface(const Mesh& surface) {
  const std::uint64_t down = downwardFacingTriangles(surface);
  if (down == 0) return;
  throw std::runtime_error("mesh self-check failed: " + std::to_string(down) +
                           " surface triangles face down or have no area");
}

// The STL's 80-byte header names the tool and what was meshed (spec 5.5), a tile's block by its name before the grid
// (Part B 5.4), the file name last since the header cuts what does not fit, and so never begins "solid", which marks an
// ASCII STL to some readers.
std::string stlHeader(const MeshOptions& options, const Facts& f, const std::string& block = {}) {
  return "dem2mesh " DL_ELEVATION_VERSION + (block.empty() ? std::string() : " block " + block) + " grid " +
         std::to_string(f.grid.cols) + "x" + std::to_string(f.grid.rows) + " window " + std::to_string(f.window.cols) +
         "x" + std::to_string(f.window.rows) + "+" + std::to_string(f.window.col0) + "+" +
         std::to_string(f.window.row0) + " " + options.input.filename().string();
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

// The plan's memory with --tolerance (spec 4.2), and in Part B's modes, where the grid can be far larger than the mesh
// written: the window and the grid, each held once as float32, and both at once while the one is resampled into the
// other; with --footprint data the window's data mask too, a byte a sample, which readTerrain holds beside them then
// (the grid's own mask comes after the window has gone). The simplified mesh grows with what it keeps, not with the
// grid, and the footprint's cells and working state come on top, as the fill's do. To the nearest megabyte, or once
// that is 1,000 or more, in gigabytes to one decimal.
std::string memoryLine(const MeshOptions& options, const Facts& f) {
  const std::uint64_t samples = static_cast<std::uint64_t>(f.window.cols) * static_cast<std::uint64_t>(f.window.rows);
  const bool mask = footprintOfData(options);
  const double floats = 4.0 * static_cast<double>(samples + f.grid.vertices());
  const double megabytes = (floats + (mask ? static_cast<double>(samples) : 0.0)) / 1e6;
  const std::string about = megabytes < 999.5 ? fixed(megabytes, 0) + " MB" : fixed(megabytes / 1000, 1) + " GB";
  return "memory: window " + std::to_string(samples) + " samples and grid " + std::to_string(f.grid.vertices()) +
         " vertices, 4 bytes each" + (mask ? ", and the data mask, a byte a sample" : "") + ": about " + about;
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

// The lines on the blocks (Part B 5.4), after base:, with counts as plain integers and sizes in "units" (deviation 1).

// "r03 c05": a block's name as the summary gives it.
std::string blockLabel(const BlockPlan& plan, std::size_t block) {
  std::string label = plan.names[block];
  label[label.find('_')] = ' ';
  return label;
}

// The blocks' labels, a comma between each two.
std::string blockList(const BlockPlan& plan, const std::vector<std::size_t>& blocks) {
  std::string list;
  for (const std::size_t block : blocks) list += (list.empty() ? "" : ", ") + blockLabel(plan, block);
  return list;
}

// part of whole as a percentage, rounded to a whole number: "72".
std::string percentOf(std::uint64_t part, std::uint64_t whole) {
  return std::to_string(std::llround(100.0 * static_cast<double>(part) / static_cast<double>(whole)));
}

// --min-piece, or its default, in square model units (Part B 5.1).
double minPieceUnits2(const MeshOptions& options) { return options.minPiece.value_or(demdl::cli::kDefaultMinPiece); }

// One cell of the output grid in square model units, which --min-piece is given in (Part B 5.2).
double cellAreaUnits2(const MeshOptions& options, const Facts& f) {
  return f.extent.widthM / (f.grid.cols - 1) * options.scale * (f.extent.heightM / (f.grid.rows - 1) * options.scale);
}

// "tiles: 13 x 20 blocks of 1000 x 1000 cells (5000 x 5000 m, 100 x 100 units); 146 with land, 114 without, 32 cut
// short at the east or south edge": the lattice across and down; a full block in cells, in metres and in model units,
// the units to the hundredth as metres are; and what the lattice's blocks hold, whether --blocks picks them or not.
std::string tilesLine(const MeshOptions& options, const Facts& f, const BlockPlan& plan, const Blocks& blocks) {
  const TileLayout& t = plan.layout;
  const double widthM = t.blockCols * (f.extent.widthM / (f.grid.cols - 1));
  const double heightM = t.blockRows * (f.extent.heightM / (f.grid.rows - 1));
  const std::uint64_t all = static_cast<std::uint64_t>(t.across) * static_cast<std::uint64_t>(t.down);
  return "tiles: " + std::to_string(t.across) + " x " + std::to_string(t.down) + " blocks of " +
         std::to_string(t.blockCols) + " x " + std::to_string(t.blockRows) + " cells (" + metres(widthM) + " x " +
         metres(heightM) + " m, " + metres(widthM * options.scale) + " x " + metres(heightM * options.scale) +
         " units); " + std::to_string(blocks.withLand) + " with land, " + std::to_string(all - blocks.withLand) +
         " without, " + std::to_string(blocks.cutShort) + " cut short at the east or south edge";
}

// "block r03 c05: 72% land, 184220 triangles, 9.2 MB, cape-town_r03_c05.stl": the share of the block's cells that its
// kept pieces cover (all of a square's), its solid's triangles, and its file, by name, with its size to a tenth of a
// megabyte.
std::string blockLine(const BlockPlan& plan, const SimplifiedBlock& b) {
  return "block " + blockLabel(plan, b.index) + ": " + percentOf(b.landCells, b.cells) + "% land, " +
         std::to_string(b.solidTriangles) + " triangles, " +
         fixed(static_cast<double>(stlFileBytes(b.solidTriangles)) / 1e6, 1) + " MB, " +
         plan.stls[b.index].filename().string();
}

// "footprint: 72% of the window, 2 pieces": the one block of a footprint without --tile, which is the whole grid.
std::string footprintLine(const SimplifiedBlock& b) {
  const std::size_t pieces = b.result.pieces.size();
  return "footprint: " + percentOf(b.landCells, b.cells) + "% of the window, " + std::to_string(pieces) +
         (pieces == 1 ? " piece" : " pieces");
}

// "dropped pieces: r05 c02 2 cells 0.5 units2, ...": each piece under --min-piece, by its block when there are tiles,
// in cells and in square model units to one decimal.
std::string droppedLine(const MeshOptions& options, const Facts& f, const BlockPlan& plan, const Blocks& blocks) {
  const double cellArea = cellAreaUnits2(options, f);
  std::string line = "dropped pieces: ";
  for (std::size_t k = 0; k < blocks.dropped.size(); ++k) {
    const DroppedPiece& piece = blocks.dropped[k];
    if (k > 0) line += ", ";
    if (plan.tiled) line += blockLabel(plan, piece.block) + " ";
    line += std::to_string(piece.cells) + (piece.cells == 1 ? " cell " : " cells ") +
            fixed(static_cast<double>(piece.cells) * cellArea, 1) + " units2";
  }
  return line;
}

// "stale: cape-town_r01_c05.stl, ...": the block files of the stem that this run left behind, by name (staleFiles).
std::string staleLine(const Blocks& blocks) {
  std::string line = "stale: ";
  for (std::size_t k = 0; k < blocks.stale.size(); ++k) line += (k > 0 ? ", " : "") + blocks.stale[k];
  return line;
}

// All of the lines on the blocks, in the summary's order: with --tile the lattice, a line for each block written, then
// the blocks skipped for want of land and those outside --blocks; without it, the footprint; then, when there are
// any, the pieces dropped, the corner contacts joined inside the blocks written (the plan's ruling on contacts), and
// the stale files.
std::vector<std::string> blockLines(const MeshOptions& options, const Facts& f, const BlockPlan& plan,
                                    const Blocks& blocks) {
  std::vector<std::string> lines;
  if (plan.tiled) {
    lines.push_back(tilesLine(options, f, plan, blocks));
    for (const SimplifiedBlock& b : blocks.written) lines.push_back(blockLine(plan, b));
    if (!blocks.skipped.empty()) lines.push_back("skipped: no land in " + blockList(plan, blocks.skipped));
    if (!blocks.notSelected.empty()) lines.push_back("not selected: " + blockList(plan, blocks.notSelected));
  } else {
    lines.push_back(footprintLine(blocks.written.front()));
  }
  if (!blocks.dropped.empty()) lines.push_back(droppedLine(options, f, plan, blocks));
  std::uint64_t contacts = 0;
  for (const SimplifiedBlock& b : blocks.written) contacts += b.cornerContacts;
  if (contacts > 0) lines.push_back("corner contacts: " + std::to_string(contacts));
  if (!blocks.stale.empty()) lines.push_back(staleLine(blocks));
  return lines;
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
            << baseLine(options, f) << "\n";
  for (const std::string& line : f.blockLines) std::cout << line << "\n";
  std::cout << solidLine(f) << "\n";
  if (!f.check.edgesChecked) std::cout << edgesNotCheckedLine() << "\n";
  for (const fs::path& stl : f.stls) std::cout << "wrote: " << stl.string() << "\n";
  std::cout << "wrote: " << outputs.ply.string() << "\n";
}

// The plan of spec 3.1, after the "dry run" line that run() printed first: the summary's lines with what the summary
// leaves out (the projection, where the samples sit, the no-data value, the vertical datum, the window in metres) and
// what would be written instead of "wrote:". With --tile the blocks' STLs, and the PLY after them, are what a run
// "would write:", and only the PLY and the .blend are looked for on disk (Part B 5.4).
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
    line(memoryLine(options, f));
    line(simplifyLine(options, f));
  }
  line(heightsLine(f));
  line(summitLine(f));
  line(baseLine(options, f));
  for (const std::string& text : f.blockLines) line(text);
  line(solidLine(f));
  if (!f.check.edgesChecked) line(edgesNotCheckedLine());
  const std::optional<fs::path> blender = options.noBlend ? std::nullopt : findBlender(options.blender);
  std::vector<const fs::path*> written;
  if (options.tile) {
    for (const fs::path& stl : f.stls) line("would write: " + stl.string());
    line("would write: " + outputs.ply.string());
  } else {
    written.push_back(&outputs.stl);
  }
  written.push_back(&outputs.ply);
  if (blender) written.push_back(&outputs.blend);
  for (const fs::path* path : written) {
    line("output: " + path->string() + (anythingAt(*path) ? " (exists, will be overwritten)" : " (will be created)"));
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
// saves it as the .blend and checks the STL, or with --tile every block's STL, one after another in the one session
// (Part B 5.4). The owner asked for the .blend, so a Blender that is missing or fails fails the run. Exit 0 alone does
// not prove that the script ran: Blender exits 0 when it cannot read the script, or after an exception in it. So the
// run succeeds only when the output holds the script's "stl check:" line, which it prints last (for blocks, the one
// line of their totals), and the .blend is there. An old .blend, there only under --overwrite, is removed first, before
// Blender is looked for, so that the .blend found afterwards is this Blender's, and a run whose Blender fails or is not
// found leaves no old scene beside the new STL and PLY.
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
                       f.stls,
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

// The summary, then Blender's scene, which --no-blend leaves out.
int reportAndMakeScene(const MeshOptions& options, const Outputs& outputs, const Facts& f, StageClock& clock) {
  printSummary(options, outputs, f);
  if (options.noBlend) {
    std::cout << "blender: skipped (--no-blend)\n";
    return demdl::cli::kExitComplete;
  }
  return makeScene(options, outputs, f, clock);
}

// Part B: the blocks.

// --blocks, read before anything else is looked at, or every block when it was not given. parseBlocks' messages
// complete "--blocks: ", as the parser's do for the flags it checks itself.
BlockSelection blockSelection(const MeshOptions& options) {
  if (!options.blocks) return {};
  try {
    return parseBlocks(*options.blocks);
  } catch (const std::invalid_argument& e) {
    throw std::runtime_error(std::string("--blocks: ") + e.what());
  }
}

// "1 row", "2 rows".
std::string counted(int count, const std::string& thing) {
  return std::to_string(count) + " " + thing + (count == 1 ? "" : "s");
}

// Part B 5.3 and deviation 2: the lattice of --tile, each block as near SIZE square as the output grid's cells allow,
// or without --tile the one block of the whole grid, which the exact predicates bound to kMostChunkCells a side; with
// each block's name and file. Refused before anything is read when a block side would be longer than that
// (tileLayout says which), when the lattice has more than kMostBlocks blocks, and when --blocks reaches beyond it,
// which BlockSelection::selected would let pass.
BlockPlan planBlocks(const MeshOptions& options, const Outputs& outputs, const Facts& f,
                     const BlockSelection& selection) {
  BlockPlan plan;
  plan.tiled = options.tile.has_value();
  plan.footprint = footprintOfData(options);
  plan.selection = selection;
  const int cols = f.grid.cols;
  const int rows = f.grid.rows;
  if (plan.tiled) {
    plan.layout = tileLayout(cols, rows, *options.tile, options.scale, f.extent.widthM / (cols - 1),
                             f.extent.heightM / (rows - 1));
  } else if (cols - 1 > kMostChunkCells || rows - 1 > kMostChunkCells) {
    throw std::runtime_error("the window is " + std::to_string(cols - 1) + " x " + std::to_string(rows - 1) +
                             " cells; a footprint without --tile is limited to " + std::to_string(kMostChunkCells) +
                             " a side: use --tile or --grid");
  } else {
    plan.layout = {cols, rows, cols - 1, rows - 1, 1, 1};
  }
  const TileLayout& t = plan.layout;
  const std::uint64_t count = static_cast<std::uint64_t>(t.across) * static_cast<std::uint64_t>(t.down);
  if (count > kMostBlocks) {
    throw std::runtime_error("the lattice is " + std::to_string(count) + " blocks; the limit is " +
                             std::to_string(kMostBlocks) + ": use a larger --tile");
  }
  if (!selection.all && (selection.lastRow > t.down || selection.lastCol > t.across)) {
    throw std::runtime_error("--blocks: " + demdl::cli::quoted(*options.blocks) + " reaches beyond the lattice of " +
                             counted(t.down, "row") + " and " + counted(t.across, "column") + " of blocks");
  }
  for (int r = 0; r < t.down; ++r) {
    for (int c = 0; c < t.across; ++c) {
      plan.names.push_back(blockName(t, r, c));
      plan.stls.push_back(plan.tiled ? fs::path(options.outputStem.string() + "_" + plan.names.back() + ".stl")
                                     : outputs.stl);
    }
  }
  return plan;
}

// Deviation 4: without --overwrite, every file of the lattice that is there already is refused before anything is
// read, whether or not this run would write it, since which blocks hold land is known only once the window is read:
// each block's STL in lattice order, then the PLY and the .blend, as checkNoOutputExists takes them for one solid.
void checkNoLatticeFileExists(const BlockPlan& plan, const Outputs& outputs) {
  for (const fs::path& stl : plan.stls) refuseExisting(stl);
  refuseExisting(outputs.ply);
  refuseExisting(outputs.blend);
}

// Whether name is a block's file of the stem in some lattice, STEM_r<digits>_c<digits>.stl, its numbers of any length
// and padding: "cape-town_r03_c05.stl" and "cape-town_r3_c5.stl", not "cape-town_r3_c.stl".
bool isBlockFileName(std::string_view name, std::string_view stem) {
  const auto skip = [&name](std::string_view text) {
    if (!name.starts_with(text)) return false;
    name.remove_prefix(text.size());
    return true;
  };
  const auto skipDigits = [&name] {
    const std::size_t count = std::min(name.find_first_not_of("0123456789"), name.size());
    name.remove_prefix(count);
    return count > 0;
  };
  return skip(stem) && skip("_r") && skipDigits() && skip("_c") && skipDigits() && name == ".stl";
}

// Part B 5.4: the block files of the stem that this run leaves behind, by name and in name order, which for one
// lattice's files is lattice order: every regular file in the output directory named as a block's of any lattice
// (isBlockFileName) that this run neither writes (in a dry run, would write) nor leaves alone as the file of a block of
// its lattice outside --blocks. So the files of another lattice, which a change of --tile, --grid, --scale or crop
// makes, are named too, though the refusal without --overwrite looks only at this lattice's names. A directory that
// cannot be listed names none, as a name that cannot be looked at counts as nothing (anythingAt).
std::vector<std::string> staleFiles(const MeshOptions& options, const BlockPlan& plan, const Blocks& blocks) {
  std::set<std::string> accounted;
  for (const SimplifiedBlock& b : blocks.written) accounted.insert(plan.stls[b.index].filename().string());
  for (const std::size_t block : blocks.notSelected) accounted.insert(plan.stls[block].filename().string());
  fs::path dir = options.outputStem.parent_path();
  if (dir.empty()) dir = ".";
  const std::string stem = options.outputStem.filename().string();
  std::vector<std::string> stale;
  std::error_code listing;
  for (fs::directory_iterator entry(dir, listing), end; !listing && entry != end; entry.increment(listing)) {
    const std::string name = entry->path().filename().string();
    std::error_code ignored;
    if (isBlockFileName(name, stem) && !accounted.contains(name) && entry->is_regular_file(ignored)) {
      stale.push_back(name);
    }
  }
  std::sort(stale.begin(), stale.end());
  return stale;
}

// Part B 5.3, the first pass: every block of the lattice in order, its footprint cut from the grid's cells with
// --footprint data (pieces under --min-piece dropped), and each one that is selected and holds land simplified into the
// one surface of the whole area, which makes blocks that share a side share its vertices. The grid and the cells are
// only read; the lattice is counted whole, selected or not, for the tiles: line.
Blocks simplifyBlocks(const MeshOptions& options, const Facts& f, const BlockPlan& plan, const std::vector<float>& grid,
                      const FootprintGrid& footprint) {
  const int cols = f.grid.cols;
  const int rows = f.grid.rows;
  const double cellArea = cellAreaUnits2(options, f);
  BlockSimplifier simplifier(grid, cols, rows, toleranceInMetres(options), f.extent.widthM, f.extent.heightM,
                             options.zScale, options.scale, f.base.levelM, kMostVertices);
  Blocks blocks;
  const TileLayout& t = plan.layout;
  for (int r = 0; r < t.down; ++r) {
    for (int c = 0; c < t.across; ++c) {
      const std::size_t index = static_cast<std::size_t>(r) * static_cast<std::size_t>(t.across) +
                                static_cast<std::size_t>(c);
      const BlockRect rect = blockRect(t, r, c);
      std::optional<BlockFootprint> cut;
      if (plan.footprint) {
        cut = blockFootprint(footprint.cells, cols, rows, rect, cellArea, minPieceUnits2(options), footprint.joined);
      }
      const bool land = !cut || !cut->pieces.empty();
      if (land) ++blocks.withLand;
      if (cutShort(t, r, c)) ++blocks.cutShort;
      if (!plan.selection.selected(r, c)) {
        blocks.notSelected.push_back(index);
        continue;
      }
      if (cut) {
        for (const Piece& piece : cut->dropped) blocks.dropped.push_back({index, piece.cells});
      }
      if (!land) {
        blocks.skipped.push_back(index);
        continue;
      }
      SimplifiedBlock& b = blocks.written.emplace_back();
      b.index = index;
      b.rect = rect;
      b.result = simplifier.simplifyBlock(rect, cut ? &*cut : nullptr);
      b.cells = static_cast<std::uint64_t>(rect.lastColumn - rect.firstColumn) *
                static_cast<std::uint64_t>(rect.lastRow - rect.firstRow);
      b.landCells = cut ? cut->cells : b.cells;
      b.cornerContacts = cut ? cut->cornerContacts : 0;
      b.solidTriangles = b.result.rectangle ? solidTriangleCount(*b.result.rectangle, b.result.triangles)
                                            : solidTriangleCount(b.result.pieces, b.result.triangles);
    }
  }
  blocks.surface = simplifier.takeSurface();
  return blocks;
}

// Why a run in Part B's modes has no block to write: no selected block keeps a piece of footprint.
std::string nothingToWrite(const MeshOptions& options, const BlockPlan& plan, const Blocks& blocks) {
  const std::string what =
      plan.tiled ? "no selected block has land to write: " : "the footprint has nothing to write: ";
  const std::size_t dropped = blocks.dropped.size();
  if (dropped == 0) return what + "no cell has data at all four of its corners";
  return what + std::to_string(dropped) + (dropped == 1 ? " piece was" : " pieces were") +
         " dropped under --min-piece " + number(minPieceUnits2(options));
}

// What the blocks will write (Part B 5.4): the whole-area surface, which is the PLY, and each block's solid, an STL of
// its own, their triangles and sizes added up; and the surface's summit, its highest vertex back in metres, and its
// lowest and highest z for Blender's camera, as makeSurface and run() find them for one solid.
void countBlocks(const MeshOptions& options, Facts& f, const Blocks& blocks) {
  f.surfaceVertices = blocks.surface.vertices.size();
  f.surfaceTriangles = blocks.surface.triangles.size();
  f.triangles = 0;
  f.stlBytes = 0;
  for (const SimplifiedBlock& b : blocks.written) {
    f.triangles += b.solidTriangles;
    f.stlBytes += stlFileBytes(b.solidTriangles);
  }
  f.plyBytes = plyFileBytes(f.surfaceVertices, f.surfaceTriangles);
  const auto [low, high] = std::minmax_element(blocks.surface.vertices.begin(), blocks.surface.vertices.end(),
                                               [](const Vec3& a, const Vec3& b) { return a.z < b.z; });
  f.surfaceLowZ = low->z;
  f.surfaceHighZ = high->z;
  f.meshSummit = static_cast<double>(high->z) / (options.zScale * options.scale);
}

// The number in a block's own mesh of the surface's vertex n: its place in numbers, which lists the block's vertices by
// their numbers in the surface, ascending. A vertex the block does not have gets numbers.size(), which no vertex of the
// block's mesh has, so that the solid builders refuse it.
std::uint32_t localNumber(const std::vector<std::uint32_t>& numbers, std::uint32_t n) {
  const auto at = std::lower_bound(numbers.begin(), numbers.end(), n);
  return static_cast<std::uint32_t>((at != numbers.end() && *at == n ? at : numbers.end()) - numbers.begin());
}

// A block's part of the whole-area surface as a mesh of its own (Part B 5.3): its triangles, and the vertices they use,
// numbered afresh in the order of their numbers in the surface, x and y measured from the block's south-west corner,
// (x0, y0), in double and then rounded to float as the surface's are, and z as it is. numbers gets each of the block's
// vertices' number in the surface, for localNumber.
Mesh blockSurface(const Mesh& surface, const SimplifiedBlock& b, double x0, double y0,
                  std::vector<std::uint32_t>& numbers) {
  const auto first = surface.triangles.begin() + static_cast<std::ptrdiff_t>(b.result.firstTriangle);
  const auto last = first + static_cast<std::ptrdiff_t>(b.result.triangles);
  numbers.clear();
  for (auto t = first; t != last; ++t) numbers.insert(numbers.end(), {t->a, t->b, t->c});
  std::sort(numbers.begin(), numbers.end());
  numbers.erase(std::unique(numbers.begin(), numbers.end()), numbers.end());
  Mesh mesh;
  mesh.vertices.reserve(numbers.size());
  for (const std::uint32_t n : numbers) {
    const Vec3& v = surface.vertices[n];
    mesh.vertices.push_back({static_cast<float>(v.x - x0), static_cast<float>(v.y - y0), v.z});
  }
  mesh.triangles.reserve(b.result.triangles);
  for (auto t = first; t != last; ++t) {
    mesh.triangles.push_back({localNumber(numbers, t->a), localNumber(numbers, t->b), localNumber(numbers, t->c)});
  }
  return mesh;
}

// A window block's four sides, and a footprint block's outlines, in the numbers of the block's own mesh (the plan's
// ruling on the outlines' numbers).
Boundary renumbered(const Boundary& boundary, const std::vector<std::uint32_t>& numbers) {
  Boundary local = boundary;
  for (std::vector<std::uint32_t>& side : local.sides) {
    for (std::uint32_t& v : side) v = localNumber(numbers, v);
  }
  return local;
}

std::vector<PieceOutline> renumbered(const std::vector<PieceOutline>& pieces,
                                     const std::vector<std::uint32_t>& numbers) {
  std::vector<PieceOutline> local = pieces;
  for (PieceOutline& piece : local) {
    for (std::uint32_t& v : piece.vertices) v = localNumber(numbers, v);
  }
  return local;
}

// One block as a solid of its own (Part B 5.3): its part of the whole-area surface in the block's frame, checked to
// face up; the solid on it, a square's walls and base for a window block and each piece's for a footprint block,
// checked to be closed, to face out and to have the triangles its outline makes (spec 6); then, unless this is a dry
// run, its STL. numbers is room for blockSurface's.
void writeBlock(const MeshOptions& options, const BlockPlan& plan, Facts& f, const Mesh& surface,
                const SimplifiedBlock& b, std::vector<std::uint32_t>& numbers) {
  // The block's south-west corner, worked out in double as the simplifier places its vertices (Simplifier.h).
  const double x0 = b.rect.firstColumn * f.extent.widthM / (f.grid.cols - 1) * options.scale;
  const double y0 = (f.grid.rows - 1 - b.rect.lastRow) * f.extent.heightM / (f.grid.rows - 1) * options.scale;
  const Mesh top = blockSurface(surface, b, x0, y0, numbers);
  checkSurface(top);
  const auto planeZ = static_cast<float>(f.base.planeZ);
  const Mesh solid = b.result.rectangle ? solidFromSurface(top, renumbered(*b.result.rectangle, numbers), planeZ)
                                        : solidFromPieces(top, renumbered(b.result.pieces, numbers), planeZ);
  const MeshCheck check = checkSolid(solid);
  selfCheck(check, b.solidTriangles);
  if (!check.edgesChecked) f.check.edgesChecked = false;
  if (options.dryRun) return;
  writeBinaryStl(solid, plan.stls[b.index], stlHeader(options, f, plan.tiled ? plan.names[b.index] : std::string()));
}

// Part B 5.3, the second pass: the blocks to be written, in lattice order and one at a time, so that only one block's
// mesh and solid are held beside the whole-area surface. With --tile a failure names its block.
void writeBlocks(const MeshOptions& options, const BlockPlan& plan, Facts& f, const Blocks& blocks) {
  std::vector<std::uint32_t> numbers;
  for (const SimplifiedBlock& b : blocks.written) {
    if (!plan.tiled) {
      writeBlock(options, plan, f, blocks.surface, b, numbers);
      continue;
    }
    try {
      writeBlock(options, plan, f, blocks.surface, b, numbers);
    } catch (const std::exception& e) {
      throw std::runtime_error("block " + blockLabel(plan, b.index) + ": " + e.what());
    }
  }
}

// Part B 5.3 to 5.5, once the grid and its footprint are made and the base plane chosen. The blocks are simplified into
// one surface of the whole area, which is counted, limited and warned about as a simplified mesh is (spec 4.2), before
// anything is written; the grid and the cells go once it is made. Then each block's solid is built, checked and
// written; then the PLY of the whole area, which goes before Blender starts, the summary and Blender, as for one solid.
// The block files of the stem that this run does not write, from an earlier run of this lattice or another, are left
// alone and named (staleFiles). The dry run builds and checks everything, as for one solid, and writes nothing.
int meshBlocks(const MeshOptions& options, const Outputs& outputs, const BlockPlan& plan, Facts& f,
               std::vector<float> grid, FootprintGrid footprint, StageClock& clock) {
  Blocks blocks = simplifyBlocks(options, f, plan, grid, footprint);
  release(grid);
  release(footprint.cells);
  clock.lap("simplify");
  if (blocks.written.empty()) throw std::runtime_error(nothingToWrite(options, plan, blocks));
  countBlocks(options, f, blocks);
  limitSimplifiedMesh(options, f);
  writeBlocks(options, plan, f, blocks);
  clock.lap("blocks");
  blocks.stale = staleFiles(options, plan, blocks);
  f.stls.clear();
  for (const SimplifiedBlock& b : blocks.written) f.stls.push_back(plan.stls[b.index]);
  f.blockLines = blockLines(options, f, plan, blocks);
  if (options.dryRun) {
    printPlan(options, outputs, f);
    return demdl::cli::kExitComplete;
  }
  writeBinaryPly(blocks.surface, outputs.ply, f.extent.widthM * options.scale, f.extent.heightM * options.scale);
  release(blocks.surface.vertices);
  release(blocks.surface.triangles);
  clock.lap("write PLY");
  return reportAndMakeScene(options, outputs, f, clock);
}

int run(int argc, char** argv) {
  MeshOptions options;
  const demdl::cli::ParseOutcome parsed = demdl::cli::parseMeshArgs(argc, argv, options, std::cout, std::cerr);
  if (parsed.exitNow) return parsed.exitCode;  // help, version, or a usage error the parser has printed
  if (const std::string problem = demdl::cli::validateMeshOptions(options); !problem.empty()) {
    throw std::runtime_error(problem);
  }
  const BlockSelection selection = blockSelection(options);
  // Part B's modes run on the block simplifier, so without --tolerance they simplify at 0, which keeps every sample off
  // its neighbours' plane, and their memory: line and warning are the tolerance's (the plan's ruling); but their grid
  // keeps the cap it has without the flag (the plan's ruling on the cap), so whether it was given is noted first.
  const bool toleranceGiven = options.tolerance.has_value();
  if (meshesInBlocks(options) && !options.tolerance) options.tolerance = 0.0;
  checkInput(options.input);
  checkOutputDirectory(options.outputStem);
  const Outputs outputs = outputsFor(options.outputStem);
  // With --tile the files are the lattice's, which the grid decides (checkNoLatticeFileExists).
  if (!options.overwrite && !options.tile) checkNoOutputExists(outputs);
  if (!options.noBlend) checkNamedBlender(options);
  // First, so that it stays the first line when --verbose prints the stages' times.
  if (options.dryRun) std::cout << "dry run\n";

  StageClock clock(options.verbose);
  Facts f;
  f.stls = {outputs.stl};
  f.geometry = inspectGeoTiff(options.input);
  f.window = pixelWindowFor(f.geometry, options.crop);
  f.extent = extentInMetres(f.geometry, f.window);
  f.grid = outputGridFor(options.grid, f.window, f.extent, toleranceGiven, meshesInBlocks(options));
  std::optional<BlockPlan> plan;
  if (meshesInBlocks(options)) {
    plan = planBlocks(options, outputs, f, selection);
    if (!options.overwrite && plan->tiled) checkNoLatticeFileExists(*plan, outputs);
  }
  // Without --tolerance the grid is what will be written, so its files are known before anything is read and a large
  // one is warned about first. Part B's modes, which have a tolerance of 0 by now, warn after simplifying instead, on
  // the whole-area surface and the blocks that will be written (limitSimplifiedMesh).
  if (!options.tolerance) {
    countOutputs(f, f.grid.vertices(), f.grid.surfaceTriangles(), solidTriangleCount(f.grid.cols, f.grid.rows));
    if (f.surfaceVertices > kWarnVertices) warnOfLargeFiles(options, "the output grid", f);
  }
  clock.lap("inspect");

  FootprintGrid footprint;
  std::vector<float> grid = readTerrain(options, f, plan && plan->footprint ? &footprint : nullptr, clock);
  f.base = chooseBasePlane(f.lowest, f.highest, options.base, options.zScale, options.scale, options.plinth);
  if (plan) return meshBlocks(options, outputs, *plan, f, std::move(grid), std::move(footprint), clock);
  Surface surface = makeSurface(options, f, std::move(grid), clock);
  checkSurface(surface.mesh);
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
  return reportAndMakeScene(options, outputs, f, clock);
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
