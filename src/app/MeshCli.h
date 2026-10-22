#pragma once

#include <filesystem>
#include <iosfwd>
#include <optional>
#include <string>

#include "app/Cli.h"
#include "demdl/BoundingBox.h"

namespace demdl::cli {

// The output grid in vertices across (cols) and down (rows). rows 0 stands for "no height given": the height then
// follows the aspect ratio of the window in metres, so that cells come out square. An explicit height is never 0;
// parseMeshArgs refuses `--grid 400x0` rather than let it pass for 400 alone. Each part is whatever int the user typed,
// up to INT_MAX, and validateMeshOptions only sets a lower bound, so widen before multiplying them into a vertex count.
struct GridSize {
  int cols = 0;
  int rows = 0;
};

// What dem2mesh's command line asks for (spec 3), with the defaults of the spec's table. The paths are the text the
// user typed; nothing here has been looked at on disk.
struct MeshOptions {
  // Where the solid exists: over the whole window (today's solid), or only where the DEM has data, with a wall along
  // the edge of the data.
  enum class Footprint { Window, Data };

  std::filesystem::path input;
  std::filesystem::path outputStem;  // the files are <outputStem>.stl, .ply and .blend
  std::optional<BoundingBox> crop;   // all four edges given, or nullopt for the whole file
  std::optional<GridSize> grid;      // nullopt for full resolution
  double zScale = 1;
  std::optional<double> base;  // metres before exaggeration; nullopt for sea level
  double scale = 1;
  double plinth = 2;
  std::optional<double> tolerance;  // model units after --z-scale and --scale; nullopt writes the full grid
  Footprint footprint = Footprint::Window;
  std::optional<double> tile;         // block side in model units after --scale; nullopt writes one solid
  std::optional<std::string> blocks;  // the --blocks text as typed, ROWS:COLS; main reads it with parseBlocks
  double minPiece = 1;                // square model units; a piece of footprint below it is dropped and listed
  bool noBlend = false;
  std::optional<std::filesystem::path> blender;
  bool overwrite = false;
  bool dryRun = false;
  bool verbose = false;
};

// Fills `options` from argv with CLI11. Help, version and error text go to the given streams.
// Does not call validateMeshOptions(); the caller does. After a usage error `options` may be partially filled and must
// not be used.
//
// Numbers must be plain decimals and whole numbers, as in dl_elevation's parser (CliChecks.h), the four crop flags
// come together or not at all, --footprint must be window or data, and --output, --blender and --blocks must not be
// empty. The form of the --blocks text, ROWS:COLS, is for main to check with parseBlocks (mesh/Tiles.h), which is why
// the text is kept as typed. Nothing is looked up on disk: that the input exists, that the output directory can be
// written and that a Blender named can be started is for main to check. Text that CLI11 reads as infinity, such as
// 1e999, gets through here and is for validateMeshOptions to refuse.
[[nodiscard]] ParseOutcome parseMeshArgs(int argc, const char* const* argv, MeshOptions& options, std::ostream& out,
                                         std::ostream& err);

// Empty when the options can be used, otherwise the first problem found, phrased for the user. Checked in this order:
// the crop box (BoundingBox::validate: finite, in range, north above south, east above west); the grid (at least 2
// vertices each way, a height of 0 aside); z-scale and scale (finite, then positive); base (finite); plinth (finite,
// then zero or positive); tolerance, when given (finite, then positive); tile, when given (finite, then positive);
// min-piece (finite, then zero or positive); and --blocks, which needs --tile (its text is not read here). Pure: it
// never touches the file system.
[[nodiscard]] std::string validateMeshOptions(const MeshOptions& options);

}  // namespace demdl::cli
