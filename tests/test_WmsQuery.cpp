#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <limits>
#include <stdexcept>
#include <string>

#include "ExampleConfig.h"
#include "demdl/WmsQuery.h"

using Catch::Matchers::WithinAbs;
using demdl::ParseResult;
using demdl::featureInfoUrl;
using demdl::formatCoordinate;
using demdl::parseElevation;
using demdl::percentEncode;

TEST_CASE("the golden GetFeatureInfo URL for the example configuration", "[wmsquery]") {
  const std::string expected =
      "/erdas-iws/ogc/wms/GeoSpatial%20Datasets?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetFeatureInfo"
      "&BBOX=-34.216344700%2C18.418579100%2C-34.184541800%2C18.462524400"
      "&CRS=EPSG%3A4326&WIDTH=2048&HEIGHT=1792"
      "&LAYERS=Elevation_CCT_GLM2019_5m_DEM&STYLES=&FORMAT=image%2Fpng"
      "&QUERY_LAYERS=Elevation_CCT_GLM2019_5m_DEM&INFO_FORMAT=text%2Fplain&I=0&J=0";
  CHECK(featureInfoUrl(exampleConfig(), 0, 0) == expected);
}

TEST_CASE("I and J carry the pixel column and row", "[wmsquery]") {
  const std::string url = featureInfoUrl(exampleConfig(), 5, 7);
  CHECK(url.find("&I=5&J=7") != std::string::npos);
}

TEST_CASE("the layer name is percent-encoded in both LAYERS and QUERY_LAYERS", "[wmsquery]") {
  demdl::Config cfg = exampleConfig();
  cfg.layer = "My Layer/1";
  const std::string url = featureInfoUrl(cfg, 0, 0);
  CHECK(url.find("&LAYERS=My%20Layer%2F1&") != std::string::npos);
  CHECK(url.find("&QUERY_LAYERS=My%20Layer%2F1&") != std::string::npos);
}

TEST_CASE("featureInfoUrl throws when the WMS URL does not parse", "[wmsquery]") {
  demdl::Config cfg = exampleConfig();
  cfg.wmsUrl = "not a url";
  CHECK_THROWS_AS(featureInfoUrl(cfg, 0, 0), std::invalid_argument);
}

TEST_CASE("percentEncode keeps unreserved characters and encodes everything else", "[wmsquery]") {
  CHECK(percentEncode("Abc-1.2_x~") == "Abc-1.2_x~");
  CHECK(percentEncode("a b:c/d") == "a%20b%3Ac%2Fd");
  CHECK(percentEncode("\xC3\xA9") == "%C3%A9");
  CHECK(percentEncode("%") == "%25");
}

TEST_CASE("formatCoordinate uses nine fixed decimals", "[wmsquery]") {
  CHECK(formatCoordinate(-34.2163447) == "-34.216344700");
  CHECK(formatCoordinate(18.0) == "18.000000000");
}

TEST_CASE("parseElevation reads the whole number after the band marker", "[wmsquery]") {
  const auto a = parseElevation("GetFeatureInfo results:\nBAND 0: 123.45\n");
  REQUIRE(a.kind == ParseResult::Kind::Value);
  CHECK_THAT(a.value, WithinAbs(123.45, 1e-12));

  const auto b = parseElevation("BAND 0: -1.5");
  REQUIRE(b.kind == ParseResult::Kind::Value);
  CHECK_THAT(b.value, WithinAbs(-1.5, 1e-12));

  const auto c = parseElevation("BAND 0: 1086.25\nBAND 1: 0\n");
  REQUIRE(c.kind == ParseResult::Kind::Value);
  CHECK_THAT(c.value, WithinAbs(1086.25, 1e-12));
}

TEST_CASE("the value is the number at the start of the marker's line", "[wmsquery]") {
  // The number ends the body or the line, or whitespace follows it, and then any text may.
  for (const char* body : {"BAND 0: 12.5 m", "BAND 0: 12.5\tm", "BAND 0: 12.5\r\n", "BAND 0: 12.5"}) {
    CAPTURE(body);
    const auto parsed = parseElevation(body);
    REQUIRE(parsed.kind == ParseResult::Kind::Value);
    CHECK_THAT(parsed.value, WithinAbs(12.5, 1e-12));
  }

  // The number must be on the marker's own line.
  CHECK(parseElevation("BAND 0:\n5").kind == ParseResult::Kind::ParseError);
}

TEST_CASE("a number with other text glued to it is a parse error", "[wmsquery]") {
  // strtod would read 1, 12, 12 and 12.5 from these, and a stored value is never fetched again; a parse
  // error is retried.
  for (const char* body : {"BAND 0: 1.#QNAN", "BAND 0: 12,5", "BAND 0: 12abc", "BAND 0: 12.5e"}) {
    CAPTURE(body);
    CHECK(parseElevation(body).kind == ParseResult::Kind::ParseError);
  }
}

TEST_CASE("a plain body without the band marker is NoData", "[wmsquery]") {
  CHECK(parseElevation("GetFeatureInfo results:\n").kind == ParseResult::Kind::NoData);
  CHECK(parseElevation("GetFeatureInfo results:\nno bands\n").kind == ParseResult::Kind::NoData);
}

TEST_CASE("an empty, blank or markup body is a parse error, not NoData", "[wmsquery]") {
  CHECK(parseElevation("").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("   \n").kind == ParseResult::Kind::ParseError);

  const std::string html = "<html><body>Service temporarily unavailable</body></html>";
  CHECK(parseElevation(html).kind == ParseResult::Kind::ParseError);

  const std::string owsReport = "  <?xml version=\"1.0\"?><ows:ExceptionReport/>";
  CHECK(parseElevation(owsReport).kind == ParseResult::Kind::ParseError);

  // The band marker still wins over markup. No number can contain '<', so it may end one.
  for (const char* body : {"<pre>BAND 0: 42.5</pre>", "<pre>BAND 0: 42.5\n</pre>"}) {
    CAPTURE(body);
    const auto wrapped = parseElevation(body);
    REQUIRE(wrapped.kind == ParseResult::Kind::Value);
    CHECK_THAT(wrapped.value, WithinAbs(42.5, 1e-12));
  }
}

TEST_CASE("a WMS service exception is a parse error, not NoData", "[wmsquery]") {
  const std::string report =
      R"(<?xml version="1.0"?><ServiceExceptionReport>)"
      R"(<ServiceException code="LayerNotDefined">no such layer</ServiceException>)"
      R"(</ServiceExceptionReport>)";
  CHECK(parseElevation(report).kind == ParseResult::Kind::ParseError);

  // The band marker wins when it is present, even if the body also mentions ServiceException.
  const auto both = parseElevation("ServiceException\nBAND 0: 42.5\n");
  REQUIRE(both.kind == ParseResult::Kind::Value);
  CHECK_THAT(both.value, WithinAbs(42.5, 1e-12));
}

TEST_CASE("a band marker without a usable number is a parse error", "[wmsquery]") {
  CHECK(parseElevation("BAND 0: abc").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0:").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: nan").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: inf").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: -inf").kind == ParseResult::Kind::ParseError);
}

TEST_CASE("a band value outside float range is a parse error", "[wmsquery]") {
  CHECK(parseElevation("BAND 0: 1e300").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: -1e300").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: 3.5e38").kind == ParseResult::Kind::ParseError);

  // FLT_MAX itself is still a value, and it survives the cast to float.
  const auto limit = parseElevation("BAND 0: 3.4028234663852886e38");
  REQUIRE(limit.kind == ParseResult::Kind::Value);
  CHECK(static_cast<float>(limit.value) == std::numeric_limits<float>::max());
}

TEST_CASE("a default-constructed ParseResult is a parse error, never a 0 m value", "[wmsquery]") {
  CHECK(ParseResult{}.kind == ParseResult::Kind::ParseError);
}
