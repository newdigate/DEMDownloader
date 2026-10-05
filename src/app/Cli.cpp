#include "app/Cli.h"

#include <CLI/CLI.hpp>

#include <cstddef>
#include <ostream>
#include <string>

namespace demdl::cli {

namespace {

// The character tests are ASCII only, so the result does not depend on the locale.
bool isDigit(char c) { return c >= '0' && c <= '9'; }

// Moves i past a run of digits and returns how many there were.
std::size_t skipDigits(const std::string& text, std::size_t& i) {
  const std::size_t start = i;
  while (i < text.size() && isDigit(text[i])) ++i;
  return i - start;
}

// An optional sign, digits with an optional fraction or a fraction alone, then an optional exponent:
// 12, 12., .5, -34.1845418, 1e-3.
bool isPlainDecimal(const std::string& text) {
  std::size_t i = 0;
  if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
  std::size_t digits = skipDigits(text, i);
  if (i < text.size() && text[i] == '.') {
    ++i;
    digits += skipDigits(text, i);
  }
  if (digits == 0) return false;
  if (i < text.size() && (text[i] == 'e' || text[i] == 'E')) {
    ++i;
    if (i < text.size() && (text[i] == '+' || text[i] == '-')) ++i;
    if (skipDigits(text, i) == 0) return false;
  }
  return i == text.size();
}

// An optional '-', then digits without a leading zero; "0" itself is allowed.
bool isPlainInteger(const std::string& text) {
  if (text == "0") return true;
  const std::size_t first = text.starts_with('-') ? 1 : 0;
  if (first == text.size() || text[first] == '0') return false;
  for (std::size_t i = first; i < text.size(); ++i) {
    if (!isDigit(text[i])) return false;
  }
  return true;
}

// `text` in single quotes, readable whatever it holds: \r, \n and \t become those escapes, a backslash
// becomes \\, a single quote \', and any other byte outside printable ASCII (0x20..0x7E) \xHH.
std::string quoted(const std::string& text) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  std::string out = "'";
  for (const char c : text) {
    const auto byte = static_cast<unsigned char>(c);
    switch (c) {
      case '\r': out += "\\r"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      case '\\': out += "\\\\"; break;
      case '\'': out += "\\'"; break;
      default:
        if (byte >= 0x20 && byte <= 0x7E) {
          out += c;
        } else {
          out += "\\x";
          out += kHex[byte >> 4];
          out += kHex[byte & 0x0F];
        }
    }
  }
  return out + "'";
}

// The checks below return an empty string for an acceptable value, otherwise the problem. The numeric ones end
// with the value quoted, so that stray characters such as a trailing space or a carriage return show.

std::string checkDecimal(const std::string& text) {
  if (isPlainDecimal(text)) return {};
  return "must be a decimal number such as -34.1845418 (use '.' as the decimal separator), not " + quoted(text);
}

std::string checkWholeNumber(const std::string& text) {
  if (isPlainInteger(text)) return {};
  return "must be a whole number without leading zeros, separators or prefixes, not " + quoted(text);
}

std::string checkNotFlagLike(const std::string& text) {
  if (!text.starts_with("--")) return {};
  return "looks like a flag; is the value missing?";
}

}  // namespace

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
  app.add_option("--width", config.width, "grid width in pixels")->required()->check(wholeNumber);
  app.add_option("--height", config.height, "grid height in pixels")->required()->check(wholeNumber);
  app.add_option("-o,--output", output, "GeoTIFF to create or resume")->required()->check(notFlagLike);
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
