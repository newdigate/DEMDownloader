#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

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

TEST_CASE("percentEncode keeps unreserved characters", "[wmsquery]") {
  CHECK(percentEncode("Abc-1.2_x~") == "Abc-1.2_x~");
  CHECK(percentEncode("a b:c/d") == "a%20b%3Ac%2Fd");
  CHECK(percentEncode("\xC3\xA9") == "%C3%A9");
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

TEST_CASE("a body without the band marker is NoData", "[wmsquery]") {
  CHECK(parseElevation("GetFeatureInfo results:\n").kind == ParseResult::Kind::NoData);
  CHECK(parseElevation("").kind == ParseResult::Kind::NoData);
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

TEST_CASE("a band marker without a number is a parse error", "[wmsquery]") {
  CHECK(parseElevation("BAND 0: abc").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0:").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: nan").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: inf").kind == ParseResult::Kind::ParseError);
  CHECK(parseElevation("BAND 0: -inf").kind == ParseResult::Kind::ParseError);
}
