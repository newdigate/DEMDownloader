#include "app/Cli.h"
#include "app/CliChecks.h"

#include <CLI/CLI.hpp>

#include <ostream>
#include <string>

namespace demdl::cli {

ParseOutcome parseArgs(int argc, const char* const* argv, Config& config, std::ostream& out, std::ostream& err) {
  CLI::App app{"Download a digital elevation model from a WMS server one pixel at a time into a GeoTIFF",
               "dl_elevation"};
  app.set_version_flag("--version", std::string(DL_ELEVATION_VERSION));
  app.failure_message(
      [](const CLI::App* self, const CLI::Error& e) { return "error: " + CLI::FailureMessage::simple(self, e); });

  // CLI11 2.7.2 runs these checks on the raw text before converting it. Without them it would read "" as 0,
  // drop digit separators ('_', '\'' and the locale's thousands separator, ',' by default, so "1,5" is 15),
  // take 0x, 0b, 0o, leading-zero octal and "true" as integers, and take hex floats, nan and inf as decimals.
  const CLI::Validator decimal(checkDecimal, "");
  const CLI::Validator wholeNumber(checkWholeNumber, "");
  // A forgotten value would otherwise swallow the next flag: "--output --dry-run" would write to "--dry-run".
  const CLI::Validator notFlagLike(checkNotFlagLike, "");

  // Parsed into a local so that config.output stays untouched when parsing fails.
  std::string output;
  app.add_option("--north", config.box.north, "northern edge, WGS84 decimal degrees")->required()->check(decimal);
  app.add_option("--south", config.box.south, "southern edge, WGS84 decimal degrees")->required()->check(decimal);
  app.add_option("--east", config.box.east, "eastern edge, WGS84 decimal degrees")->required()->check(decimal);
  app.add_option("--west", config.box.west, "western edge, WGS84 decimal degrees")->required()->check(decimal);
  // The limit Config::validate applies, from the same constant.
  const std::string sizeLimit = " (width x height at most " + std::to_string(kMaxPixels) + ")";
  app.add_option("--width", config.width, "grid width in pixels" + sizeLimit)->required()->check(wholeNumber);
  app.add_option("--height", config.height, "grid height in pixels" + sizeLimit)->required()->check(wholeNumber);
  app.add_option("-o,--output", output, "GeoTIFF to create or resume (a file name, not . or ..)")
      ->required()
      ->check(notFlagLike);
  app.add_option("--wms-url", config.wmsUrl, "WMS endpoint: scheme, host and path")
      ->capture_default_str()
      ->check(notFlagLike);
  app.add_option("--layer", config.layer, "WMS layer name, taken unencoded; used for LAYERS and QUERY_LAYERS")
      ->capture_default_str()
      ->check(notFlagLike);
  // The ranges below are stated for the user; Config::validate alone enforces them.
  app.add_option("--threads", config.threads, "worker threads (1..64), one keep-alive connection each")
      ->capture_default_str()
      ->check(wholeNumber);
  app.add_option("--max-retries", config.maxRetries, "retries per pixel after the first attempt (0..100)")
      ->capture_default_str()
      ->check(wholeNumber);
  app.add_option("--max-consecutive-failures", config.maxConsecutiveFailures,
                 "stop after this many pixels fail in a row (at least 1)")
      ->capture_default_str()
      ->check(wholeNumber);
  CLI::Option* const caBundleOption =
      app.add_option("--ca-bundle", config.caBundle,
                     "PEM bundle for certificate verification (file must exist; excludes --insecure)")
          ->check(notFlagLike)
          ->check(CLI::ExistingFile);
  app.add_flag("--insecure", config.insecure, "disable certificate verification (excludes --ca-bundle)")
      ->excludes(caBundleOption);
  app.add_flag("--preview", config.preview, "open the OpenGL preview window (fails if built without preview support)");
  app.add_flag("--overwrite", config.overwrite, "ignore an existing output and start fresh");
  app.add_flag("--dry-run", config.dryRun, "validate and report the plan; make no requests and write nothing");
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
