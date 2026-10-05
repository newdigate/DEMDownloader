#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "TempDir.h"
#include "app/Cli.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;
using demdl::Config;
using demdl::cli::ParseOutcome;
using demdl::cli::parseArgs;

namespace {

// argv[0] as the shell passes it when the program is run from the repository root.
const char* const kArgv0 = "./build/dl_elevation";

struct Parsed {
  Config config;
  ParseOutcome outcome;
  std::string out;
  std::string err;
};

Parsed parse(std::vector<const char*> argv) {
  argv.insert(argv.begin(), kArgv0);
  Parsed p;
  std::ostringstream out;
  std::ostringstream err;
  p.outcome = parseArgs(static_cast<int>(argv.size()), argv.data(), p.config, out, err);
  p.out = out.str();
  p.err = err.str();
  return p;
}

const std::vector<const char*> kExample = {"--north", "-34.1845418", "--south",  "-34.2163447", "--west",
                                            "18.4185791", "--east", "18.4625244", "--width", "2048",
                                            "--height", "1792", "--output", "simonstown.tif"};

// kExample with `flag` given `value`: the example's own value is replaced, or the pair is appended.
std::vector<const char*> exampleWith(const char* flag, const char* value) {
  std::vector<const char*> argv = kExample;
  const auto it =
      std::find_if(argv.begin(), argv.end(), [flag](const char* arg) { return std::string_view(arg) == flag; });
  if (it == argv.end()) {
    argv.push_back(flag);
    argv.push_back(value);
  } else {
    *std::next(it) = value;
  }
  return argv;
}

// Creates an empty file and returns its path. The CLI checks only that a CA bundle exists.
std::string createFile(const std::filesystem::path& path) {
  std::ofstream file(path);
  REQUIRE(file.is_open());
  return path.string();
}

}  // namespace

// No test name here starts with '-': the test binary would parse such a name as an option when ctest runs it.
TEST_CASE("the example command line fills the Config", "[cli]") {
  std::vector<const char*> argv = kExample;
  argv.push_back("--preview");
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK_THAT(p.config.box.north, WithinAbs(-34.1845418, 1e-12));
  CHECK_THAT(p.config.box.south, WithinAbs(-34.2163447, 1e-12));
  CHECK_THAT(p.config.box.east, WithinAbs(18.4625244, 1e-12));
  CHECK_THAT(p.config.box.west, WithinAbs(18.4185791, 1e-12));
  CHECK(p.config.width == 2048);
  CHECK(p.config.height == 1792);
  CHECK(p.config.output == "simonstown.tif");
  CHECK(p.config.preview);
  CHECK(p.config.threads == 8);
  CHECK(p.config.maxRetries == 8);
  CHECK(p.config.maxConsecutiveFailures == 100);
  CHECK(p.config.layer == "Elevation_CCT_GLM2019_5m_DEM");
  CHECK(p.config.wmsUrl == "https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets");
  CHECK_FALSE(p.config.insecure);
  CHECK_FALSE(p.config.dryRun);
  CHECK(p.config.validate().empty());
}

TEST_CASE("the -o short form sets the output", "[cli]") {
  std::vector<const char*> argv = kExample;
  const auto it = std::find(argv.begin(), argv.end(), std::string("--output"));
  REQUIRE(it != argv.end());
  *it = "-o";
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.output == "simonstown.tif");
}

TEST_CASE("a missing required flag is a usage error", "[cli]") {
  const std::string flag =
      GENERATE(as<std::string>{}, "--north", "--south", "--west", "--east", "--width", "--height", "--output");
  std::vector<const char*> argv = kExample;
  const auto it = std::find(argv.begin(), argv.end(), flag);
  REQUIRE(it != argv.end());
  argv.erase(it, it + 2);  // the flag and its value
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, StartsWith("error: "));
  CHECK_THAT(p.err, ContainsSubstring(flag + " is required"));
}

TEST_CASE("the --help flag prints usage and exits successfully", "[cli]") {
  const Parsed p = parse({"--help"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("dl_elevation [OPTIONS]"));
  CHECK_THAT(p.out, !ContainsSubstring(kArgv0));  // the program's name, not the path it was run as
  CHECK_THAT(p.out, ContainsSubstring("--wms-url"));
  CHECK_THAT(p.out, ContainsSubstring("--max-consecutive-failures"));
  CHECK_THAT(p.out, ContainsSubstring("(1..64)"));
  CHECK_THAT(p.out, ContainsSubstring("(0..100)"));
  CHECK_THAT(p.out, ContainsSubstring("(width x height at most 536870912)"));
  CHECK_THAT(p.out, ContainsSubstring("(a file name, not . or ..)"));
}

TEST_CASE("help wins over an invalid value", "[cli]") {
  std::vector<const char*> argv = exampleWith("--north", "1,5");
  argv.push_back("--help");
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("dl_elevation [OPTIONS]"));
  CHECK(p.err.empty());
}

TEST_CASE("the --version flag prints the project version", "[cli]") {
  const Parsed p = parse({"--version"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring(DL_ELEVATION_VERSION));
}

TEST_CASE("network, TLS and mode flags set their fields", "[cli]") {
  std::vector<const char*> argv = kExample;
  for (const char* a : {"--threads", "3", "--max-retries", "0", "--max-consecutive-failures", "5", "--insecure",
                        "--verbose", "--dry-run", "--overwrite", "--layer", "Other", "--wms-url",
                        "http://127.0.0.1:8080/wms"}) {
    argv.push_back(a);
  }
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.threads == 3);
  CHECK(p.config.maxRetries == 0);
  CHECK(p.config.maxConsecutiveFailures == 5);
  CHECK(p.config.insecure);
  CHECK(p.config.caBundle.empty());
  CHECK(p.config.verbose);
  CHECK(p.config.dryRun);
  CHECK(p.config.overwrite);
  CHECK(p.config.layer == "Other");
  CHECK(p.config.wmsUrl == "http://127.0.0.1:8080/wms");
}

TEST_CASE("the --ca-bundle flag takes an existing file", "[cli]") {
  const TempDir dir;
  const std::string bundle = createFile(dir.path / "roots.pem");
  std::vector<const char*> argv = kExample;
  argv.push_back("--ca-bundle");
  argv.push_back(bundle.c_str());
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.caBundle == bundle);
  CHECK_FALSE(p.config.insecure);
}

TEST_CASE("a missing CA bundle is a usage error", "[cli]") {
  const TempDir dir;
  const std::string missing = (dir.path / "missing.pem").string();
  std::vector<const char*> argv = kExample;
  argv.push_back("--ca-bundle");
  argv.push_back(missing.c_str());
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--ca-bundle"));
  CHECK_THAT(p.err, ContainsSubstring(missing));
}

TEST_CASE("the --insecure and --ca-bundle flags together are a usage error", "[cli]") {
  const TempDir dir;
  const std::string bundle = createFile(dir.path / "roots.pem");
  std::vector<const char*> argv = kExample;
  for (const char* a : {"--insecure", "--ca-bundle", bundle.c_str()}) {
    argv.push_back(a);
  }
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--ca-bundle excludes --insecure"));
}

TEST_CASE("a value that does not convert is a usage error", "[cli]") {
  // A plain whole number passes the CLI's check, but this one does not fit an int, so CLI11's conversion fails.
  const Parsed p = parse(exampleWith("--threads", "99999999999"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("Could not convert: --threads"));
}

TEST_CASE("integer flags take only plain whole numbers", "[cli]") {
  // CLI11 alone would read these as 64, 16, 1000, 2048, 1 and 0.
  const auto [flag, value] = GENERATE(table<const char*, const char*>({{"--width", "0100"},
                                                                       {"--width", "0x10"},
                                                                       {"--width", "1_000"},
                                                                       {"--width", "20,48"},
                                                                       {"--width", "true"},
                                                                       {"--max-retries", ""}}));
  const Parsed p = parse(exampleWith(flag, value));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(std::string(flag) +
                                      ": must be a whole number without leading zeros, separators or prefixes"));
  CHECK_THAT(p.err, ContainsSubstring(std::string(", not '") + value + "'"));
}

TEST_CASE("each integer flag runs the whole-number check", "[cli]") {
  const std::string flag =
      GENERATE(as<std::string>{}, "--width", "--height", "--threads", "--max-retries", "--max-consecutive-failures");
  const Parsed p = parse(exampleWith(flag.c_str(), "0100"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err,
             ContainsSubstring(flag + ": must be a whole number without leading zeros, separators or prefixes"));
}

TEST_CASE("coordinates take only plain decimals", "[cli]") {
  // CLI11 alone would read "" as 0, "1,5" as 15 and "-34,18" as -3418, and accept a hex float, nan and inf.
  const std::string value = GENERATE(as<std::string>{}, "", "1,5", "-34,18", "0x1p3", "nan", "inf");
  const Parsed p = parse(exampleWith("--north", value.c_str()));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(
                        "--north: must be a decimal number such as -34.1845418 (use '.' as the decimal separator)"));
  CHECK_THAT(p.err, ContainsSubstring(", not '" + value + "'"));
}

TEST_CASE("each coordinate flag runs the decimal check", "[cli]") {
  const std::string flag = GENERATE(as<std::string>{}, "--north", "--south", "--east", "--west");
  const Parsed p = parse(exampleWith(flag.c_str(), "1,5"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(
                        flag + ": must be a decimal number such as -34.1845418 (use '.' as the decimal separator)"));
}

TEST_CASE("coordinates may have an exponent or a bare leading or trailing point", "[cli]") {
  const auto [value, expected] =
      GENERATE(table<const char*, double>({{"1e-3", 0.001}, {".5", 0.5}, {"-34.", -34.0}}));
  const Parsed p = parse(exampleWith("--north", value));
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK_THAT(p.config.box.north, WithinAbs(expected, 1e-12));
}

TEST_CASE("the echoed value is quoted, with bytes outside printable ASCII escaped", "[cli]") {
  const auto [flag, value, shown] = GENERATE(table<const char*, const char*, const char*>({
      {"--width", "2048\r", "2048\\r"},  // a carriage return left by a Windows file
      {"--north", "-34.18 ", "-34.18 "},   // a trailing space stays visible inside the quotes
      {"--north", "\xE2\x88\x92" "34.18", "\\xE2\\x88\\x9234.18"},  // a Unicode minus sign
      {"--width", "1\t2", "1\\t2"},
      {"--width", "1\n2", "1\\n2"},
      {"--width", "1\\2", "1\\\\2"},
      {"--width", "1'2", "1\\'2"}}));
  const Parsed p = parse(exampleWith(flag, value));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(std::string(", not '") + shown + "'\n"));  // the end of the message
  CHECK(p.err.find('\r') == std::string::npos);
}

TEST_CASE("a value that looks like a flag is a usage error", "[cli]") {
  // Unchecked, "--output --dry-run" would take "--dry-run" as the path and start a real run.
  const auto [flag, value] =
      GENERATE(table<const char*, const char*>({{"--output", "--dry-run"}, {"--layer", "--verbose"}}));
  const Parsed p = parse(exampleWith(flag, value));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(std::string(flag) + ": looks like a flag; is the value missing?"));
}

TEST_CASE("each string option refuses a value that looks like a flag", "[cli]") {
  // For --ca-bundle this also shows that the check runs before the existing-file check.
  const std::string flag = GENERATE(as<std::string>{}, "--output", "--layer", "--wms-url", "--ca-bundle");
  const Parsed p = parse(exampleWith(flag.c_str(), "--x"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(flag + ": looks like a flag; is the value missing?"));
}

TEST_CASE("an unknown flag or a positional argument is a usage error", "[cli]") {
  const std::string extra = GENERATE(as<std::string>{}, "--bogus", "extra");
  std::vector<const char*> argv = kExample;
  argv.push_back(extra.c_str());
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("not expected: " + extra));
}

TEST_CASE("a value option given twice is a usage error", "[cli]") {
  std::vector<const char*> argv = kExample;
  for (const char* a : {"--threads", "3", "--threads", "4"}) {
    argv.push_back(a);
  }
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--threads: At most 1 required"));
}

TEST_CASE("values that parse but are invalid are caught by validate", "[cli]") {
  std::vector<const char*> argv = kExample;
  argv.push_back("--threads");
  argv.push_back("0");
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.validate() == "threads must be at least 1");
}
