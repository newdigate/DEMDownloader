#include "app/MeshCli.h"
#include "app/CliChecks.h"

#include <CLI/CLI.hpp>

#include <charconv>
#include <cmath>
#include <cstddef>
#include <limits>
#include <ostream>
#include <string>
#include <system_error>

namespace demdl::cli {

namespace {

// What --grid's text says: `size` is good when `problem` is empty, otherwise `problem` completes "--grid: ".
struct GridReading {
  GridSize size;
  std::string problem;
};

// `digits` has passed checkWholeNumber, so only its size can be wrong.
bool fitsInt(const std::string& digits, int& value) {
  const char* const last = digits.data() + digits.size();
  const auto [end, error] = std::from_chars(digits.data(), last, value);
  return error == std::errc{} && end == last;
}

// W or WxH: the text before the first lowercase x is the width and the text after it the height. Each part must pass
// checkWholeNumber, so an empty part, a leading zero, a sign other than '-' and the like are refused as they are for
// every other integer flag. Splitting comes first, which is why "0x10" is a width of 0 and a height of 10, not hex.
GridReading readGrid(const std::string& text) {
  const std::size_t x = text.find('x');
  const bool hasHeight = x != std::string::npos;
  const std::string widthText = text.substr(0, x);
  const std::string heightText = hasHeight ? text.substr(x + 1) : std::string();
  if (!checkWholeNumber(widthText).empty() || (hasHeight && !checkWholeNumber(heightText).empty())) {
    return {{},
            "must be W or WxH with a lowercase x, such as 400 or 400x300, each part a whole number without leading "
            "zeros, separators or prefixes, not " +
                quoted(text)};
  }
  GridSize size;
  if (!fitsInt(widthText, size.cols) || (hasHeight && !fitsInt(heightText, size.rows))) {
    return {{},
            "each part must be a whole number from " + std::to_string(std::numeric_limits<int>::min()) + " to " +
                std::to_string(std::numeric_limits<int>::max()) + ", not " + quoted(text)};
  }
  // rows 0 means that no height was given, so an explicit 0 cannot travel to validateMeshOptions as it is.
  if (hasHeight && size.rows == 0) {
    return {{},
            "the height must not be 0 in " + quoted(text) +
                " (leave out the xH to let the height follow the window's aspect)"};
  }
  return {size, {}};
}

// Without a file name at the end the files would be called .stl, .ply and .blend, which most desktops hide, and
// "." and ".." name directories. The same test as Config::validate applies to dl_elevation's --output.
std::string checkFileStem(const std::string& text) {
  const std::filesystem::path stem(text);
  if (stem.has_filename() && stem.filename() != "." && stem.filename() != "..") return {};
  return "must end in a file name (the stem of STEM.stl, STEM.ply and STEM.blend), not " + quoted(text);
}

// An empty --blender names no file, so a run could only fail on it, and only once it had read, meshed and written
// everything; it is refused here, as an empty --output is.
std::string checkBlenderName(const std::string& text) {
  if (!text.empty()) return {};
  return "must name the Blender executable, not " + quoted(text);
}

// An empty --blocks names no block. It is refused here, as an empty --blender is, so that it is a usage error whatever
// main goes on to do. Whether the text is ROWS:COLS is for main to say, with parseBlocks (mesh/Tiles.h), which puts
// the flag in front of its messages; that is why the text is kept as typed.
std::string checkBlocksText(const std::string& text) {
  if (!text.empty()) return {};
  return "must be ROWS:COLS, such as 3-5:2-4 or 7:12, not " + quoted(text);
}

}  // namespace

ParseOutcome parseMeshArgs(int argc, const char* const* argv, MeshOptions& options, std::ostream& out,
                           std::ostream& err) {
  CLI::App app{"Turn an elevation GeoTIFF into a printable STL solid, a PLY surface and a Blender scene", "dem2mesh"};
  app.set_version_flag("--version", std::string(DL_ELEVATION_VERSION));
  app.failure_message(
      [](const CLI::App* self, const CLI::Error& e) { return "error: " + CLI::FailureMessage::simple(self, e); });

  // The same strict checks as dl_elevation's parser, for the same reasons (see Cli.cpp): CLI11 would otherwise read
  // "" as 0 and "0,02" as 2, and take hex floats, nan and inf as decimals.
  const CLI::Validator decimal(checkDecimal, "");
  // A forgotten value would otherwise swallow the next flag: "-o --dry-run" would write files called --dry-run.
  const CLI::Validator notFlagLike(checkNotFlagLike, "");
  const CLI::Validator gridText([](const std::string& text) { return readGrid(text).problem; }, "");
  const CLI::Validator fileStem(checkFileStem, "");
  const CLI::Validator blenderName(checkBlenderName, "");
  const CLI::Validator blocksText(checkBlocksText, "");
  // --footprint takes window or data and nothing else; any other text is a usage error that names both. The
  // validator's own description is dropped: the option's type name, window|data, says as much in the help.
  const CLI::Validator footprintValue = CLI::IsMember({"window", "data"}).description("");

  // Parsed into locals so that the paths, the optional values, the footprint and the grid are assigned only when the
  // whole command line has parsed. Every flag sits in the order of the spec's table.
  std::string input;
  std::string outputStem;
  BoundingBox crop;
  std::string gridArg;
  double base = 0;
  double tolerance = 0;
  std::string footprint;
  double tile = 0;
  std::string blocks;
  std::string blender;
  app.add_option("-i,--input", input, "GeoTIFF elevation model to read")
      ->type_name("PATH")
      ->required()
      ->check(notFlagLike);
  app.add_option("-o,--output", outputStem, "output file stem: STEM.stl, STEM.ply and STEM.blend are written")
      ->type_name("STEM")
      ->required()
      ->check(notFlagLike)
      ->check(fileStem);
  // CLI11 checks that required options and needs are met before it converts and checks the values of ordinary options.
  // An edge is converted first, so that a lone "--north 1,5" is told what is wrong with 1,5 rather than asked for
  // --south. Help and version are processed ahead of both.
  const auto addEdge = [&app, &decimal](const char* name, double& edge, const char* description) {
    return app.add_option(name, edge, description)
        ->check(decimal)
        ->callback_priority(CLI::CallbackPriority::PreRequirementsCheck);
  };
  const char* const latitude = "crop window edge, WGS84 decimal degrees, -90 to 90 (all four or none)";
  const char* const longitude = "crop window edge, WGS84 decimal degrees, -180 to 180 (all four or none)";
  CLI::Option* const north = addEdge("--north", crop.north, latitude);
  CLI::Option* const south = addEdge("--south", crop.south, latitude);
  CLI::Option* const east = addEdge("--east", crop.east, longitude);
  CLI::Option* const west = addEdge("--west", crop.west, longitude);
  // Each edge needs the other three, so one edge alone, or three of the four, is a usage error that names a flag
  // still missing. Whether the box makes sense is validateMeshOptions' to say.
  north->needs(south, east, west);
  south->needs(north, east, west);
  east->needs(north, south, west);
  west->needs(north, south, east);
  // The bounds are stated for the user. validateMeshOptions enforces the 2, and main the 25,000,000 or, with
  // --tolerance, the 2,147,483,647, either of which needs the window's aspect when the height is left out.
  CLI::Option* const gridOption =
      app.add_option("--grid", gridArg,
                     "output grid in vertices, W or WxH, each at least 2 and at most 25,000,000 in all without "
                     "--tolerance, 2,147,483,647 with it (without xH the height follows the window's aspect)")
          ->type_name("W[xH]")
          ->check(gridText);
  // The descriptions give the defaults and the ranges, which validateMeshOptions enforces; CLI11's captured default,
  // a "[1]" after the type, would say the default twice.
  app.add_option("--z-scale", options.zScale, "vertical exaggeration, above 0 (default 1)")->check(decimal);
  CLI::Option* const baseOption =
      app.add_option("--base", base,
                     "base plane height in metres before exaggeration, any finite number (default sea level, 0; "
                     "lowered to the lowest point when needed)")
          ->check(decimal);
  app.add_option("--scale", options.scale,
                 "model units per metre in the STL and PLY, above 0 (default 1; 0.02 makes a 10 km window 200 mm "
                 "wide)")
      ->check(decimal);
  app.add_option("--plinth", options.plinth,
                 "minimum thickness under the lowest point, model units after scaling, 0 or more (default 2)")
      ->check(decimal);
  CLI::Option* const toleranceOption =
      app.add_option("--tolerance", tolerance,
                     "simplify the mesh to a TIN whose vertical error at any grid sample is at most FLOAT model units "
                     "(after --z-scale and --scale), above 0; absent, the full grid is written")
          ->check(decimal);
  // The type names are the forms of the spec's table, and the descriptions of --tile and --min-piece refer to SIZE and
  // A by them.
  CLI::Option* const footprintOption =
      app.add_option("--footprint", footprint,
                     "where the solid exists: window, the whole window as today; data, only where the DEM has data, "
                     "with a wall along the data's edge (default window)")
          ->type_name("window|data")
          ->check(footprintValue);
  CLI::Option* const tileOption =
      app.add_option("--tile", tile,
                     "split the window into square blocks of SIZE model units, each written as STEM_rRR_cCC.stl, "
                     "above 0; absent, one solid is written")
          ->type_name("SIZE")
          ->check(decimal);
  CLI::Option* const blocksOption =
      app.add_option("--blocks", blocks,
                     "write only these blocks, each part a number or a range from 1, such as 3-5:2-4 or 7:12; "
                     "the others are listed")
          ->type_name("ROWS:COLS")
          ->check(notFlagLike)
          ->check(blocksText);
  app.add_option("--min-piece", options.minPiece,
                 "drop a piece of footprint smaller than A square model units and list it, 0 or more (default 1)")
      ->type_name("A")
      ->check(decimal);
  app.add_flag("--no-blend", options.noBlend, "write the STL and PLY only; do not run Blender");
  CLI::Option* const blenderOption =
      app.add_option("--blender", blender,
                     "the Blender executable (default: $BLENDER, then /Applications/Blender.app, then blender on "
                     "PATH)")
          ->type_name("PATH")
          ->check(notFlagLike)
          ->check(blenderName);
  app.add_flag("--overwrite", options.overwrite,
               "replace existing output files (otherwise they are an error, raised before anything is read)");
  app.add_flag("--dry-run", options.dryRun, "read the input and print the plan; write nothing");
  app.add_flag("--verbose", options.verbose, "print per-stage timings and the Blender log");

  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    const int code = app.exit(e, out, err);
    return {true, code == 0 ? 0 : 1};
  }
  options.input = input;
  options.outputStem = outputStem;
  // needs() has seen to it that the other three edges were given with this one.
  if (north->count() > 0) options.crop = crop;
  if (gridOption->count() > 0) options.grid = readGrid(gridArg).size;
  if (baseOption->count() > 0) options.base = base;
  if (toleranceOption->count() > 0) options.tolerance = tolerance;
  // The check let through window and data only.
  if (footprintOption->count() > 0) {
    options.footprint = footprint == "data" ? MeshOptions::Footprint::Data : MeshOptions::Footprint::Window;
  }
  if (tileOption->count() > 0) options.tile = tile;
  if (blocksOption->count() > 0) options.blocks = blocks;
  if (blenderOption->count() > 0) options.blender = std::filesystem::path(blender);
  return {false, 0};
}

std::string validateMeshOptions(const MeshOptions& options) {
  if (options.crop) {
    if (const std::string problem = options.crop->validate(); !problem.empty()) return problem;
  }
  if (options.grid) {
    // A height of 0 asks for the aspect ratio's; only a height that was given has to reach 2.
    if (options.grid->cols < 2 || (options.grid->rows != 0 && options.grid->rows < 2)) {
      return "grid must be at least 2 x 2";
    }
  }
  // The finite tests come first so that NaN, which is not positive either, is called what it is.
  if (!std::isfinite(options.zScale)) return "z-scale must be finite";
  if (!(options.zScale > 0)) return "z-scale must be positive";
  if (!std::isfinite(options.scale)) return "scale must be finite";
  if (!(options.scale > 0)) return "scale must be positive";
  if (options.base && !std::isfinite(*options.base)) return "base must be finite";
  if (!std::isfinite(options.plinth)) return "plinth must be finite";
  if (options.plinth < 0) return "plinth must be zero or positive";
  if (options.tolerance) {
    if (!std::isfinite(*options.tolerance)) return "tolerance must be finite";
    if (!(*options.tolerance > 0)) return "tolerance must be positive";
  }
  if (options.tile) {
    if (!std::isfinite(*options.tile)) return "tile must be finite";
    if (!(*options.tile > 0)) return "tile must be positive";
  }
  if (!std::isfinite(options.minPiece)) return "min-piece must be finite";
  if (options.minPiece < 0) return "min-piece must be zero or positive";
  if (options.blocks && !options.tile) return "--blocks requires --tile";
  return {};
}

}  // namespace demdl::cli
