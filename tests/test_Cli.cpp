#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "TempDir.h"
#include "app/Cli.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::WithinAbs;
using demdl::Config;
using demdl::cli::ParseOutcome;
using demdl::cli::parseArgs;

namespace {

struct Parsed {
  Config config;
  ParseOutcome outcome;
  std::string out;
  std::string err;
};

Parsed parse(std::vector<const char*> argv) {
  argv.insert(argv.begin(), "dl_elevation");
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

TEST_CASE("a missing required flag is a usage error", "[cli]") {
  const std::vector<const char*> argv(kExample.begin() + 2, kExample.end());  // drop --north and its value
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--north"));
}

TEST_CASE("the --help flag prints usage and exits successfully", "[cli]") {
  const Parsed p = parse({"--help"});
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 0);
  CHECK_THAT(p.out, ContainsSubstring("--wms-url"));
  CHECK_THAT(p.out, ContainsSubstring("--max-consecutive-failures"));
  CHECK_THAT(p.out, ContainsSubstring("(1..64)"));
  CHECK_THAT(p.out, ContainsSubstring("(0..100)"));
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
  std::vector<const char*> argv = kExample;
  argv.push_back("--threads");
  argv.push_back("abc");
  const Parsed p = parse(argv);
  CHECK(p.outcome.exitNow);
  CHECK(p.outcome.exitCode == 1);
  CHECK_THAT(p.err, ContainsSubstring("--threads"));
}

TEST_CASE("integer flags read a leading 0 as octal and 0x as hex", "[cli]") {
  // Pins CLI11's std::strtoll(..., 0) conversion so that an upgrade that changes it is noticed.
  std::vector<const char*> argv = kExample;
  REQUIRE(argv[8] == std::string("--width"));
  argv[9] = "0100";
  const Parsed octal = parse(argv);
  REQUIRE_FALSE(octal.outcome.exitNow);
  CHECK(octal.config.width == 64);
  argv[9] = "0x10";
  const Parsed hex = parse(argv);
  REQUIRE_FALSE(hex.outcome.exitNow);
  CHECK(hex.config.width == 16);
}

TEST_CASE("values that parse but are invalid are caught by validate", "[cli]") {
  std::vector<const char*> argv = kExample;
  argv.push_back("--threads");
  argv.push_back("0");
  const Parsed p = parse(argv);
  REQUIRE_FALSE(p.outcome.exitNow);
  CHECK(p.config.validate() == "threads must be at least 1");
}
