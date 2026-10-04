#include <catch2/catch_test_macros.hpp>

#include <utility>

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
  // Both ends of the port range, and '-', '_' and '.' inside a host name, are accepted.
  CHECK(parseWmsUrl("http://a-b.c_d:1/x").has_value());
  CHECK(parseWmsUrl("https://h:65535").has_value());
}

TEST_CASE("bracketed IPv6 hosts are accepted", "[config]") {
  const auto withPort = parseWmsUrl("https://[::1]:8443/wms");
  REQUIRE(withPort.has_value());
  CHECK(withPort->origin == "https://[::1]:8443");
  CHECK(withPort->path == "/wms");
  const auto withoutPort = parseWmsUrl("http://[2001:db8::1]/wms");
  REQUIRE(withoutPort.has_value());
  CHECK(withoutPort->origin == "http://[2001:db8::1]");
}

TEST_CASE("a URL without a path gets the root path", "[config]") {
  const auto ep = parseWmsUrl("https://host.test");
  REQUIRE(ep.has_value());
  CHECK(ep->origin == "https://host.test");
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
  CHECK_FALSE(parseWmsUrl("https://host.test:abc/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test::8080/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://user:pw@host.test/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test:65536/wms").has_value());
  CHECK_FALSE(parseWmsUrl("http://127.0.0.1:0/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test/wms?x=1").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test/wms#frag").has_value());
  CHECK_FALSE(parseWmsUrl("https://[::1/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test:8080:9090/wms").has_value());
  CHECK_FALSE(parseWmsUrl("https://fe80::1/wms").has_value());
  // Whitespace and control characters are refused in the path as well as in the host.
  CHECK_FALSE(parseWmsUrl("https://host.test/a b").has_value());
  CHECK_FALSE(parseWmsUrl("https://host.test/wms\r\nX: 1").has_value());
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

  Config manyRetries = exampleConfig();
  manyRetries.maxRetries = 101;
  CHECK(manyRetries.validate() == "max-retries must be at most 100");
  manyRetries.maxRetries = 100;
  CHECK(manyRetries.validate().empty());

  Config failures = exampleConfig();
  failures.maxConsecutiveFailures = 0;
  CHECK(failures.validate() == "max-consecutive-failures must be at least 1");

  Config output = exampleConfig();
  output.output.clear();
  CHECK(output.validate() == "output path must not be empty");

  Config swapped = exampleConfig();
  std::swap(swapped.box.north, swapped.box.south);
  CHECK(swapped.validate() == "north must be greater than south");

  Config layer = exampleConfig();
  layer.layer.clear();
  CHECK(layer.validate() == "layer must not be empty");
  layer.layer = "ws:dem;v2";
  CHECK(layer.validate() == "layer must not contain ';'");

  Config height = exampleConfig();
  height.height = -1;
  CHECK(height.validate() == "width and height must be positive");

  Config manyThreads = exampleConfig();
  manyThreads.threads = 65;
  CHECK(manyThreads.validate() == "threads must be at most 64");
  manyThreads.threads = 64;
  CHECK(manyThreads.validate().empty());

  Config directory = exampleConfig();
  directory.output = "out/";
  CHECK(directory.validate() == "output must name a file");

  // Two faults at once: the grid is checked before the thread count, which pins the check order.
  Config both = exampleConfig();
  both.width = 0;
  both.threads = 0;
  CHECK(both.validate() == "width and height must be positive");
}
