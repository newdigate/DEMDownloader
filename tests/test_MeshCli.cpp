#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cctype>
#include <limits>
#include <sstream>
#include <string>
#include <string_view>
#include <vector>

#include "app/MeshCli.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;
using demdl::BoundingBox;
using demdl::cli::GridSize;
using demdl::cli::MeshOptions;
using demdl::cli::ParseOutcome;
using demdl::cli::parseMeshArgs;
using demdl::cli::validateMeshOptions;

namespace {

// argv[0] as the shell passes it when the program is run from the repository root.
const char* const kArgv0 = "./build/dem2mesh";

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

struct Parsed {
  MeshOptions options;
  ParseOutcome outcome;
  std::string out;
  std::string err;
};

// `initial` is what the caller passes in, for the test that looks at what a failed parse leaves alone.
Parsed parse(std::vector<const char*> argv, const MeshOptions& initial = {}) {
  argv.insert(argv.begin(), kArgv0);
  Parsed p;
  p.options = initial;
  std::ostringstream out;
  std::ostringstream err;
  p.outcome = parseMeshArgs(static_cast<int>(argv.size()), argv.data(), p.options, out, err);
  p.out = out.str();
  p.err = err.str();
  return p;
}

// Every flag at once. The crop is valid and every value differs from its default.
const std::vector<const char*> kExample = {
    "-i", "in.tif",
    "-o", "out",
    "--north", "-33.9",
    "--south", "-34.0",
    "--east", "18.5",
    "--west", "18.3",
    "--grid", "400x300",
    "--z-scale", "2",
    "--base", "-5",
    "--scale", "0.02",
    "--plinth", "3",
    "--no-blend",
    "--blender", "/x/blender",
    "--overwrite",
    "--dry-run",
    "--verbose"};

// The two required flags and nothing else.
const std::vector<const char*> kRequired = {"-i", "in.tif", "-o", "out"};

// kRequired followed by `more`.
std::vector<const char*> requiredWith(std::initializer_list<const char*> more) {
  std::vector<const char*> argv = kRequired;
  argv.insert(argv.end(), more.begin(), more.end());
  return argv;
}

// kExample with the value of `flag` replaced.
std::vector<const char*> exampleWith(const char* flag, const char* value) {
  std::vector<const char*> argv = kExample;
  const auto it =
      std::find_if(argv.begin(), argv.end(), [flag](const char* arg) { return std::string_view(arg) == flag; });
  REQUIRE(it != argv.end());
  *std::next(it) = value;
  return argv;
}

// kExample without `flag` and its value.
std::vector<const char*> exampleWithout(const char* flag) {
  std::vector<const char*> argv = kExample;
  const auto it =
      std::find_if(argv.begin(), argv.end(), [flag](const char* arg) { return std::string_view(arg) == flag; });
  REQUIRE(it != argv.end());
  argv.erase(it, it + 2);  // the flag and its value
  return argv;
}

// The help text with every run of white space turned into one space, so that a description is found whole
// wherever CLI11 broke its lines.
std::string squashed(const std::string& text) {
  std::string out;
  for (const char c : text) {
    if (std::isspace(static_cast<unsigned char>(c))) {
      if (!out.empty() && out.back() != ' ') out += ' ';
    } else {
      out += c;
    }
  }
  return out;
}

// Options that validate, for the tests that spoil one field at a time.
MeshOptions validOptions() {
  MeshOptions options;
  options.input = "in.tif";
  options.outputStem = "out";
  return options;
}

}  // namespace

// No test name here starts with '-': the test binary would parse such a name as an option when ctest runs it.
TEST_CASE("the example command line fills every MeshOptions field", "[meshcli]") {
  const Parsed p = parse(kExample);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.out.empty());
  CHECK(p.err.empty());
  CHECK(p.options.input == "in.tif");
  CHECK(p.options.outputStem == "out");
  REQUIRE(p.options.crop.has_value());
  CHECK_THAT(p.options.crop->north, WithinAbs(-33.9, 1e-12));
  CHECK_THAT(p.options.crop->south, WithinAbs(-34.0, 1e-12));
  CHECK_THAT(p.options.crop->east, WithinAbs(18.5, 1e-12));
  CHECK_THAT(p.options.crop->west, WithinAbs(18.3, 1e-12));
  REQUIRE(p.options.grid.has_value());
  CHECK(p.options.grid->cols == 400);
  CHECK(p.options.grid->rows == 300);
  CHECK_THAT(p.options.zScale, WithinAbs(2.0, 1e-12));
  REQUIRE(p.options.base.has_value());
  CHECK_THAT(*p.options.base, WithinAbs(-5.0, 1e-12));
  CHECK_THAT(p.options.scale, WithinAbs(0.02, 1e-12));
  CHECK_THAT(p.options.plinth, WithinAbs(3.0, 1e-12));
  CHECK(p.options.noBlend);
  REQUIRE(p.options.blender.has_value());
  CHECK(*p.options.blender == "/x/blender");
  CHECK(p.options.overwrite);
  CHECK(p.options.dryRun);
  CHECK(p.options.verbose);
  CHECK(validateMeshOptions(p.options).empty());
}

TEST_CASE("the two required flags alone leave everything else at its default", "[meshcli]") {
  const Parsed p = parse(kRequired);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.options.input == "in.tif");
  CHECK(p.options.outputStem == "out");
  CHECK_FALSE(p.options.crop.has_value());
  CHECK_FALSE(p.options.grid.has_value());
  CHECK(p.options.zScale == 1.0);
  CHECK_FALSE(p.options.base.has_value());
  CHECK(p.options.scale == 1.0);
  CHECK(p.options.plinth == 2.0);
  CHECK_FALSE(p.options.noBlend);
  CHECK_FALSE(p.options.blender.has_value());
  CHECK_FALSE(p.options.overwrite);
  CHECK_FALSE(p.options.dryRun);
  CHECK_FALSE(p.options.verbose);
  CHECK(validateMeshOptions(p.options).empty());
}

TEST_CASE("the long forms --input and --output work as -i and -o", "[meshcli]") {
  const Parsed p = parse({"--input", "dir/in.tif", "--output", "dir/out"});
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.options.input == "dir/in.tif");
  CHECK(p.options.outputStem == "dir/out");
}

TEST_CASE("parsing and validating never look at the file system", "[meshcli]") {
  // Task 10's main checks that the input exists and that the output directory can be written.
  const Parsed p = parse({"-i", "/no/such/dir/in.tif", "-o", "/no/such/dir/out"});
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(validateMeshOptions(p.options).empty());
}

TEST_CASE("a missing required flag is a usage error", "[meshcli]") {
  const Parsed noInput = parse({"-o", "out"});
  CHECK(noInput.outcome.exitNow);
  CHECK(noInput.outcome.exitCode == 1);
  CHECK_THAT(noInput.err, StartsWith("error: "));
  CHECK_THAT(noInput.err, ContainsSubstring("--input is required"));

  const Parsed noOutput = parse({"-i", "in.tif"});
  CHECK(noOutput.outcome.exitNow);
  CHECK(noOutput.outcome.exitCode == 1);
  CHECK_THAT(noOutput.err, StartsWith("error: "));
  CHECK_THAT(noOutput.err, ContainsSubstring("--output is required"));
}

TEST_CASE("the --help flag prints usage and exits successfully", "[meshcli]") {
  const Parsed p = parse({"--help"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("dem2mesh [OPTIONS]"));
  CHECK_THAT(p.out, !ContainsSubstring(kArgv0));  // the program's name, not the path it was run as
  CHECK(p.err.empty());
}

TEST_CASE("the help lists every flag", "[meshcli]") {
  const Parsed p = parse({"--help"});
  for (const char* flag : {"--input", "--output", "--north", "--south", "--east", "--west", "--grid", "--z-scale",
                           "--base", "--scale", "--plinth", "--no-blend", "--blender", "--overwrite", "--dry-run",
                           "--verbose", "--version"}) {
    CAPTURE(flag);
    CHECK_THAT(p.out, ContainsSubstring(flag));
  }
}

TEST_CASE("the help states the ranges and defaults", "[meshcli]") {
  const std::string help = squashed(parse({"--help"}).out);
  for (const char* description : {
           "crop window edge, WGS84 decimal degrees (all four or none)",
           "output grid in vertices, W or WxH (at least 2; without xH the height follows the window's aspect)",
           "vertical exaggeration (default 1)",
           "base plane height in metres before exaggeration (default sea level, 0; lowered to the lowest point when "
           "needed)",
           "model units per metre in the STL and PLY (0.02 makes a 10 km window 200 mm wide)",
           "minimum thickness under the lowest point, model units after scaling (default 2)"}) {
    CAPTURE(description);
    CHECK_THAT(help, ContainsSubstring(description));
  }
}

TEST_CASE("the help shows the defaults CLI11 captured", "[meshcli]") {
  const std::string help = squashed(parse({"--help"}).out);
  for (const char* option : {"--z-scale FLOAT [1]", "--scale FLOAT [1]", "--plinth FLOAT [2]"}) {
    CAPTURE(option);
    CHECK_THAT(help, ContainsSubstring(option));
  }
}

TEST_CASE("help wins over an invalid value", "[meshcli]") {
  std::vector<const char*> argv = exampleWith("--scale", "0,02");
  argv.push_back("--help");
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("dem2mesh [OPTIONS]"));
  CHECK(p.err.empty());
}

TEST_CASE("the --version flag prints the project version", "[meshcli]") {
  const Parsed p = parse({"--version"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring(DL_ELEVATION_VERSION));
}

TEST_CASE("a --grid with a width alone leaves the height to the aspect ratio", "[meshcli]") {
  const Parsed p = parse(exampleWith("--grid", "400"));
  REQUIRE_FALSE(p.outcome.exitNow);
  REQUIRE(p.options.grid.has_value());
  CHECK(p.options.grid->cols == 400);
  CHECK(p.options.grid->rows == 0);
  CHECK(validateMeshOptions(p.options).empty());
}

TEST_CASE("the --grid flag takes a width and a height", "[meshcli]") {
  const auto [text, cols, rows] = GENERATE(table<const char*, int, int>(
      {{"400x300", 400, 300}, {"2x2", 2, 2}, {"300x400", 300, 400}, {"4x3", 4, 3}, {"2147483647x2", 2147483647, 2}}));
  CAPTURE(text);
  const Parsed p = parse(exampleWith("--grid", text));
  REQUIRE_FALSE(p.outcome.exitNow);
  REQUIRE(p.options.grid.has_value());
  CHECK(p.options.grid->cols == cols);
  CHECK(p.options.grid->rows == rows);
}

TEST_CASE("a malformed --grid is a usage error naming --grid", "[meshcli]") {
  // A trailing x with no height, a leading zero, an uppercase X (which is no separator), a missing width, nothing at
  // all, a third part, a plus sign, a digit separator, an exponent, a leading zero again after the x, a hex prefix with
  // letters, stray white space and full-width digits.
  const std::string text = GENERATE(as<std::string>{}, "400x", "0400x300", "400X300", "x300", "x", "", "400x300x2",
                                    "+400", "4,00", "1_000", "4e2", "400x03", "0x1f", " 400", "400 ", "400x300\r",
                                    "\xEF\xBC\x94\xEF\xBC\x90\xEF\xBC\x90");
  CAPTURE(text);
  const Parsed p = parse(exampleWith("--grid", text.c_str()));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, StartsWith("error: --grid: must be W or WxH with a lowercase x"));
  CHECK_THAT(p.err, ContainsSubstring("whole number without leading zeros, separators or prefixes"));
  CHECK(p.err.find('\r') == std::string::npos);
}

TEST_CASE("the echoed --grid text is quoted", "[meshcli]") {
  const Parsed p = parse(exampleWith("--grid", "400X300"));
  CHECK(p.outcome.exitNow);
  CHECK_THAT(p.err, ContainsSubstring(", not '400X300'\n"));
}

TEST_CASE("a --grid part that does not fit an int is a usage error naming --grid", "[meshcli]") {
  const std::string text = GENERATE(as<std::string>{}, "2147483648", "99999999999", "400x2147483648", "-2147483649");
  CAPTURE(text);
  const Parsed p = parse(exampleWith("--grid", text.c_str()));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--grid: each part must be a whole number from -2147483648 to 2147483647, not '" +
                                          text + "'"));
}

TEST_CASE("a --grid height of 0 is a usage error, since 0 rows would mean no height was given", "[meshcli]") {
  const Parsed p = parse(exampleWith("--grid", "400x0"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--grid: the height must not be 0 in '400x0'"));
  CHECK_THAT(p.err, ContainsSubstring("leave out the xH"));
}

TEST_CASE("a --grid that is too small parses and validate refuses it", "[meshcli]") {
  // "0x10" is a width of 0 and a height of 10, not a hexadecimal number: the text is split at the x first.
  const std::string text = GENERATE(as<std::string>{}, "1x5", "5x1", "1", "0", "0x10", "1x1", "-5", "400x-3");
  CAPTURE(text);
  const Parsed p = parse(exampleWith("--grid", text.c_str()));
  REQUIRE_FALSE(p.outcome.exitNow);
  REQUIRE(p.options.grid.has_value());
  CHECK(validateMeshOptions(p.options) == "grid must be at least 2 x 2");
}

TEST_CASE("a --grid of 1x5 reads as one column and five rows", "[meshcli]") {
  const Parsed p = parse(exampleWith("--grid", "1x5"));
  REQUIRE_FALSE(p.outcome.exitNow);
  REQUIRE(p.options.grid.has_value());
  CHECK(p.options.grid->cols == 1);
  CHECK(p.options.grid->rows == 5);
}

TEST_CASE("a --grid given twice is a usage error", "[meshcli]") {
  const Parsed p = parse(requiredWith({"--grid", "4", "--grid", "5"}));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--grid: At most 1 required"));
}

TEST_CASE("three of the four crop flags is a usage error naming the missing one", "[meshcli]") {
  const std::string missing = GENERATE(as<std::string>{}, "--north", "--south", "--east", "--west");
  CAPTURE(missing);
  const Parsed p = parse(exampleWithout(missing.c_str()));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, StartsWith("error: "));
  CHECK_THAT(p.err, ContainsSubstring("requires " + missing));
}

TEST_CASE("fewer than three crop flags is a usage error naming one that is missing", "[meshcli]") {
  struct Case {
    std::vector<const char*> given;
    std::vector<std::string> missing;
  };
  // CLI11 names one missing flag per error, and which of several it names is its own business. Each edge appears
  // alone, so that every flag's own requirement is exercised.
  const std::vector<Case> cases = {
      {{"--north", "-33.9"}, {"--south", "--east", "--west"}},
      {{"--south", "-34.0"}, {"--north", "--east", "--west"}},
      {{"--east", "18.5"}, {"--north", "--south", "--west"}},
      {{"--west", "18.3"}, {"--north", "--south", "--east"}},
      {{"--north", "-33.9", "--south", "-34.0"}, {"--east", "--west"}},
      {{"--east", "18.5", "--west", "18.3"}, {"--north", "--south"}},
      {{"--south", "-34.0", "--west", "18.3"}, {"--north", "--east"}},
  };
  for (const Case& c : cases) {
    std::vector<const char*> argv = kRequired;
    argv.insert(argv.end(), c.given.begin(), c.given.end());
    CAPTURE(c.given);
    const Parsed p = parse(argv);
    CHECK(p.outcome.exitNow);
    CHECK(p.outcome.exitCode == 1);
    CHECK_THAT(p.err, StartsWith("error: "));
    const bool namesOne = std::any_of(c.missing.begin(), c.missing.end(), [&p](const std::string& flag) {
      return p.err.find(" requires " + flag) != std::string::npos;
    });
    CHECK(namesOne);
  }
}

TEST_CASE("all four crop flags give a crop box, in any order", "[meshcli]") {
  const Parsed p =
      parse(requiredWith({"--west", "18.3", "--east", "18.5", "--south", "-34.0", "--north", "-33.9"}));
  REQUIRE_FALSE(p.outcome.exitNow);
  REQUIRE(p.options.crop.has_value());
  CHECK_THAT(p.options.crop->north, WithinAbs(-33.9, 1e-12));
  CHECK_THAT(p.options.crop->south, WithinAbs(-34.0, 1e-12));
  CHECK_THAT(p.options.crop->east, WithinAbs(18.5, 1e-12));
  CHECK_THAT(p.options.crop->west, WithinAbs(18.3, 1e-12));
}

TEST_CASE("a crop with north below south parses and validate refuses it", "[meshcli]") {
  const Parsed p = parse(requiredWith({"--north", "-34.0", "--south", "-33.9", "--east", "18.5", "--west", "18.3"}));
  REQUIRE_FALSE(p.outcome.exitNow);
  REQUIRE(p.options.crop.has_value());
  CHECK(validateMeshOptions(p.options) == "north must be greater than south");
}

TEST_CASE("a --z-scale of 0 parses and validate refuses it", "[meshcli]") {
  const Parsed p = parse(exampleWith("--z-scale", "0"));
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.options.zScale == 0.0);
  CHECK(validateMeshOptions(p.options) == "z-scale must be positive");
}

TEST_CASE("a non-positive scale or a negative plinth parses and validate refuses it", "[meshcli]") {
  const auto [flag, value, message] = GENERATE(table<const char*, const char*, const char*>(
      {{"--scale", "0", "scale must be positive"},
       {"--scale", "-0.02", "scale must be positive"},
       {"--z-scale", "-1", "z-scale must be positive"},
       {"--plinth", "-0.5", "plinth must be zero or positive"}}));
  CAPTURE(flag, value);
  const Parsed p = parse(exampleWith(flag, value));
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(validateMeshOptions(p.options) == message);
}

TEST_CASE("a --plinth of 0 is valid", "[meshcli]") {
  const Parsed p = parse(exampleWith("--plinth", "0"));
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.options.plinth == 0.0);
  CHECK(validateMeshOptions(p.options).empty());
}

TEST_CASE("the --base flag may be negative, zero or positive", "[meshcli]") {
  const auto [text, expected] = GENERATE(table<const char*, double>({{"-5", -5.0}, {"0", 0.0}, {"1500.5", 1500.5}}));
  CAPTURE(text);
  const Parsed p = parse(exampleWith("--base", text));
  REQUIRE_FALSE(p.outcome.exitNow);
  REQUIRE(p.options.base.has_value());
  CHECK_THAT(*p.options.base, WithinAbs(expected, 1e-12));
  CHECK(validateMeshOptions(p.options).empty());
}

TEST_CASE("decimal flags may have an exponent or a bare leading or trailing point", "[meshcli]") {
  const Parsed p =
      parse(requiredWith({"--scale", "2e-2", "--z-scale", ".5", "--plinth", "3.", "--base", "-5.", "--north", "1e1",
                          "--south", "-.5", "--east", "+18.5", "--west", "1.8e1"}));
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK_THAT(p.options.scale, WithinAbs(0.02, 1e-12));
  CHECK_THAT(p.options.zScale, WithinAbs(0.5, 1e-12));
  CHECK_THAT(p.options.plinth, WithinAbs(3.0, 1e-12));
  REQUIRE(p.options.base.has_value());
  CHECK_THAT(*p.options.base, WithinAbs(-5.0, 1e-12));
  REQUIRE(p.options.crop.has_value());
  CHECK_THAT(p.options.crop->north, WithinAbs(10.0, 1e-12));
  CHECK_THAT(p.options.crop->south, WithinAbs(-0.5, 1e-12));
  CHECK_THAT(p.options.crop->east, WithinAbs(18.5, 1e-12));
  CHECK_THAT(p.options.crop->west, WithinAbs(18.0, 1e-12));
}

TEST_CASE("decimal flags take only plain decimals", "[meshcli]") {
  // CLI11 alone would read "" as 0 and "0,02" as 2 (the comma is dropped as a digit separator), and would accept a
  // hex float, nan and inf.
  const std::string flag = GENERATE(as<std::string>{}, "--scale", "--north");
  const std::string value = GENERATE(as<std::string>{}, "", "0,02", "1,5", "0x1p3", "nan", "inf", "1e", "1 ");
  CAPTURE(flag, value);
  const Parsed p = parse(exampleWith(flag.c_str(), value.c_str()));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, StartsWith("error: "));
  CHECK_THAT(p.err, ContainsSubstring(
                        flag + ": must be a decimal number such as -34.1845418 (use '.' as the decimal separator)"));
  CHECK_THAT(p.err, ContainsSubstring(", not '" + value + "'"));
}

TEST_CASE("a bad crop value is reported before the missing edges that it needs", "[meshcli]") {
  // CLI11 would otherwise check the requirement first and answer "--north requires --south".
  const std::string flag = GENERATE(as<std::string>{}, "--north", "--south", "--east", "--west");
  const std::string value = GENERATE(as<std::string>{}, "", "1,5");
  CAPTURE(flag, value);
  const Parsed p = parse(requiredWith({flag.c_str(), value.c_str()}));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(
                        flag + ": must be a decimal number such as -34.1845418 (use '.' as the decimal separator)"));
  CHECK_THAT(p.err, ContainsSubstring(", not '" + value + "'"));
  CHECK_THAT(p.err, !ContainsSubstring("requires"));
}

TEST_CASE("help still wins over a bad crop value", "[meshcli]") {
  const Parsed p = parse(requiredWith({"--north", "1,5", "--help"}));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("dem2mesh [OPTIONS]"));
  CHECK(p.err.empty());
}

TEST_CASE("each numeric flag runs the decimal check", "[meshcli]") {
  const std::string flag = GENERATE(as<std::string>{}, "--north", "--south", "--east", "--west", "--z-scale", "--base",
                                    "--scale", "--plinth");
  CAPTURE(flag);
  const Parsed p = parse(exampleWith(flag.c_str(), "1,5"));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(
                        flag + ": must be a decimal number such as -34.1845418 (use '.' as the decimal separator)"));
}

TEST_CASE("a decimal beyond the range of a double parses as infinity and validate refuses it", "[meshcli]") {
  // Plain decimal text passes the check and CLI11's conversion returns infinity rather than failing, so the finite
  // tests in validateMeshOptions are reached from the command line.
  const auto [flag, value, message] = GENERATE(table<const char*, const char*, const char*>(
      {{"--z-scale", "1e999", "z-scale must be finite"},
       {"--scale", "-1e999", "scale must be finite"},
       {"--base", "1e999", "base must be finite"},
       {"--plinth", "1e999", "plinth must be finite"},
       {"--north", "1e999", "bounding box values must be finite numbers"}}));
  CAPTURE(flag, value);
  const Parsed p = parse(exampleWith(flag, value));
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(validateMeshOptions(p.options) == message);
}

TEST_CASE("a string option refuses a value that looks like a flag", "[meshcli]") {
  // Unchecked, "-o --dry-run" would take "--dry-run" as the stem and write files called that.
  const std::vector<const char*> argv = GENERATE(values<std::vector<const char*>>({
      {"-i", "--dry-run", "-o", "out"},
      {"--input", "--verbose", "-o", "out"},
      {"-i", "in.tif", "-o", "--dry-run"},
      {"-i", "in.tif", "--output", "--overwrite"},
      {"-i", "in.tif", "-o", "out", "--blender", "--verbose"},
  }));
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring(": looks like a flag; is the value missing?"));
}

TEST_CASE("an unknown flag or a positional argument is a usage error", "[meshcli]") {
  const std::string extra = GENERATE(as<std::string>{}, "--bogus", "extra");
  std::vector<const char*> argv = kExample;
  argv.push_back(extra.c_str());
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("not expected: " + extra));
}

TEST_CASE("a value option given twice is a usage error", "[meshcli]") {
  const Parsed p = parse(requiredWith({"--z-scale", "2", "--z-scale", "3"}));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--z-scale: At most 1 required"));
}

TEST_CASE("the --output stem must end in a file name", "[meshcli]") {
  // Without one the files would be called .stl, .ply and .blend, which most file browsers hide.
  const std::string stem = GENERATE(as<std::string>{}, "", ".", "..", "dir/", "a/..", "a/.");
  CAPTURE(stem);
  const Parsed p = parse({"-i", "in.tif", "-o", stem.c_str()});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--output: must end in a file name"));
  CHECK_THAT(p.err, ContainsSubstring(", not '" + stem + "'"));
}

TEST_CASE("an empty --blender is a usage error, as an empty --output is", "[meshcli]") {
  // It names no file at all, so a run could only fail on it, and only after reading, meshing and writing.
  const Parsed p = parse(requiredWith({"--blender", ""}));
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, StartsWith("error: --blender: must name the Blender executable, not ''\n"));
}

TEST_CASE("the --output flag takes a stem with or without a directory", "[meshcli]") {
  const std::string stem = GENERATE(as<std::string>{}, "out", "dir/out", "/abs/dir/out", "../out", "./out", "out.v2");
  CAPTURE(stem);
  const Parsed p = parse({"-i", "in.tif", "-o", stem.c_str()});
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.options.outputStem == stem);
}

TEST_CASE("a usage error leaves the paths as the caller had them", "[meshcli]") {
  MeshOptions before;
  before.input = "keep.tif";
  before.outputStem = "keep";
  const Parsed p = parse(exampleWith("--scale", "0,02"), before);
  REQUIRE(p.outcome.exitNow);
  CHECK(p.options.input == "keep.tif");
  CHECK(p.options.outputStem == "keep");
  CHECK_FALSE(p.options.blender.has_value());
}

TEST_CASE("the default options are valid", "[meshcli]") {
  CHECK(validateMeshOptions(MeshOptions{}).empty());
  CHECK(validateMeshOptions(validOptions()).empty());
}

TEST_CASE("validate refuses a crop box that BoundingBox refuses, in its words", "[meshcli]") {
  const auto [north, south, east, west, message] = GENERATE(table<double, double, double, double, const char*>({
      {-34.0, -33.9, 18.5, 18.3, "north must be greater than south"},
      {-34.0, -34.0, 18.5, 18.3, "north must be greater than south"},
      {-33.9, -34.0, 18.3, 18.5, "east must be greater than west"},
      {-33.9, -34.0, 18.5, 18.5, "east must be greater than west"},
      {91.0, -34.0, 18.5, 18.3, "latitudes must be within [-90, 90]"},
      {-33.9, -91.0, 18.5, 18.3, "latitudes must be within [-90, 90]"},
      {-33.9, -34.0, 181.0, 18.3, "longitudes must be within [-180, 180]"},
      {-33.9, -34.0, 18.5, -181.0, "longitudes must be within [-180, 180]"},
      {kNaN, -34.0, 18.5, 18.3, "bounding box values must be finite numbers"},
      {-33.9, -34.0, kInf, 18.3, "bounding box values must be finite numbers"},
  }));
  MeshOptions options = validOptions();
  options.crop = BoundingBox{north, south, east, west};
  CHECK(validateMeshOptions(options) == message);
}

TEST_CASE("validate accepts a crop box that spans the globe", "[meshcli]") {
  MeshOptions options = validOptions();
  options.crop = BoundingBox{90.0, -90.0, 180.0, -180.0};
  CHECK(validateMeshOptions(options).empty());
}

TEST_CASE("validate wants at least 2 vertices each way, a height of 0 meaning the aspect ratio", "[meshcli]") {
  const auto [cols, rows, valid] = GENERATE(table<int, int, bool>({{2, 2, true},
                                                                  {400, 300, true},
                                                                  {2, 0, true},
                                                                  {400, 0, true},
                                                                  {1, 5, false},
                                                                  {5, 1, false},
                                                                  {0, 0, false},
                                                                  {1, 0, false},
                                                                  {-4, 0, false},
                                                                  {400, -3, false}}));
  CAPTURE(cols, rows);
  MeshOptions options = validOptions();
  options.grid = GridSize{cols, rows};
  if (valid) {
    CHECK(validateMeshOptions(options).empty());
  } else {
    CHECK(validateMeshOptions(options) == "grid must be at least 2 x 2");
  }
}

TEST_CASE("validate wants a z-scale and a scale that are positive and finite", "[meshcli]") {
  const auto [value, message] = GENERATE(table<double, const char*>({{0.0, "must be positive"},
                                                                     {-1.0, "must be positive"},
                                                                     {-0.0, "must be positive"},
                                                                     {kNaN, "must be finite"},
                                                                     {kInf, "must be finite"},
                                                                     {-kInf, "must be finite"}}));
  CAPTURE(value);
  MeshOptions zScale = validOptions();
  zScale.zScale = value;
  CHECK(validateMeshOptions(zScale) == std::string("z-scale ") + message);
  MeshOptions scale = validOptions();
  scale.scale = value;
  CHECK(validateMeshOptions(scale) == std::string("scale ") + message);
}

TEST_CASE("validate accepts a small positive z-scale and scale", "[meshcli]") {
  MeshOptions options = validOptions();
  options.zScale = 1e-6;
  options.scale = 0.02;
  CHECK(validateMeshOptions(options).empty());
}

TEST_CASE("validate wants a base that is finite", "[meshcli]") {
  MeshOptions options = validOptions();
  for (const double ok : {-8000.0, -5.0, 0.0, 1590.1}) {
    options.base = ok;
    CHECK(validateMeshOptions(options).empty());
  }
  for (const double bad : {kNaN, kInf, -kInf}) {
    options.base = bad;
    CHECK(validateMeshOptions(options) == "base must be finite");
  }
}

TEST_CASE("validate wants a plinth that is zero or positive and finite", "[meshcli]") {
  MeshOptions options = validOptions();
  for (const double ok : {0.0, 0.5, 2.0, 1e6}) {
    options.plinth = ok;
    CHECK(validateMeshOptions(options).empty());
  }
  options.plinth = -0.5;
  CHECK(validateMeshOptions(options) == "plinth must be zero or positive");
  for (const double bad : {kNaN, kInf, -kInf}) {
    options.plinth = bad;
    CHECK(validateMeshOptions(options) == "plinth must be finite");
  }
}

TEST_CASE("validate returns the first problem in the documented order", "[meshcli]") {
  MeshOptions options = validOptions();
  options.crop = BoundingBox{-34.0, -33.9, 18.5, 18.3};
  options.grid = GridSize{1, 1};
  options.zScale = 0;
  options.scale = 0;
  options.base = kNaN;
  options.plinth = -1;
  CHECK(validateMeshOptions(options) == "north must be greater than south");
  options.crop.reset();
  CHECK(validateMeshOptions(options) == "grid must be at least 2 x 2");
  options.grid.reset();
  CHECK(validateMeshOptions(options) == "z-scale must be positive");
  options.zScale = 1;
  CHECK(validateMeshOptions(options) == "scale must be positive");
  options.scale = 1;
  CHECK(validateMeshOptions(options) == "base must be finite");
  options.base.reset();
  CHECK(validateMeshOptions(options) == "plinth must be zero or positive");
  options.plinth = 0;
  CHECK(validateMeshOptions(options).empty());
}
