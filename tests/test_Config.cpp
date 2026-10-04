#include <catch2/catch_test_macros.hpp>

#include "ExampleConfig.h"
#include "demdl/Config.h"

using demdl::Config;
using demdl::parseWmsUrl;

TEST_CASE("the default WMS URL splits into origin and path", "[config]") {
  const auto ep = parseWmsUrl(Config{}.wmsUrl);
  REQUIRE(ep.has_value());
  CHECK(ep->origin == "https://cityimg.capetown.gov.za");
  CHECK(ep->path == "/erdas-iws/ogc/wms/GeoSpatial%20Datasets");
}

TEST_CASE("a URL with a port keeps the port in the origin", "[config]") {
  const auto ep = parseWmsUrl("http://127.0.0.1:8080/wms");
  REQUIRE(ep.has_value());
  CHECK(ep->origin == "http://127.0.0.1:8080");
  CHECK(ep->path == "/wms");
}

TEST_CASE("a URL without a path gets the root path", "[config]") {
  const auto ep = parseWmsUrl("https://host.test");
  REQUIRE(ep.has_value());
  CHECK(ep->path == "/");
}

TEST_CASE("unsupported or malformed URLs are rejected", "[config]") {
  CHECK_FALSE(parseWmsUrl("ftp://host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https:///wms").has_value());
  CHECK_FALSE(parseWmsUrl("host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://:8080/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test:/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test?x=1").has_value());
  CHECK_FALSE(parseWmsUrl("HTTPS://host.test/wms").has_value());
}

TEST_CASE("the example configuration is valid", "[config]") {
  CHECK(exampleConfig().validate().empty());
}

TEST_CASE("validation reports the first problem", "[config]") {
  Config threads = exampleConfig();
  threads.threads = 0;
  CHECK(threads.validate() == "threads must be at least 1");

  Config grid = exampleConfig();
  grid.width = 0;
  CHECK(grid.validate() == "width and height must be positive");

  Config url = exampleConfig();
  url.wmsUrl = "not a url";
  CHECK(url.validate() == "wms-url must look like scheme://host[:port]/path with scheme http or https");

  Config retries = exampleConfig();
  retries.maxRetries = -1;
  CHECK(retries.validate() == "max-retries must be at least 0");

  Config failures = exampleConfig();
  failures.maxConsecutiveFailures = 0;
  CHECK(failures.validate() == "max-consecutive-failures must be at least 1");

  Config output = exampleConfig();
  output.output.clear();
  CHECK(output.validate() == "output path must not be empty");
}
