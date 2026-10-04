#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <utility>

#include "demdl/BoundingBox.h"

using Catch::Matchers::WithinAbs;
using demdl::BoundingBox;

namespace {
BoundingBox simonsTown() { return {-34.1845418, -34.2163447, 18.4625244, 18.4185791}; }
}  // namespace

TEST_CASE("a valid box passes validation", "[boundingbox]") {
  CHECK(simonsTown().validate().empty());
}

TEST_CASE("north must exceed south", "[boundingbox]") {
  BoundingBox b = simonsTown();
  std::swap(b.north, b.south);
  CHECK(b.validate() == "north must be greater than south");
}

TEST_CASE("east must exceed west", "[boundingbox]") {
  BoundingBox b = simonsTown();
  std::swap(b.east, b.west);
  CHECK(b.validate() == "east must be greater than west");
}

TEST_CASE("out-of-range coordinates are rejected", "[boundingbox]") {
  BoundingBox lat = simonsTown();
  lat.north = 91.0;
  CHECK(lat.validate() == "latitudes must be within [-90, 90]");
  BoundingBox lon = simonsTown();
  lon.west = -181.0;
  CHECK(lon.validate() == "longitudes must be within [-180, 180]");
}

TEST_CASE("pixel scale divides the box by the grid", "[boundingbox]") {
  const auto s = simonsTown().pixelScale(2048, 1792);
  CHECK_THAT(s.lonPerPixel, WithinAbs((18.4625244 - 18.4185791) / 2048.0, 1e-15));
  CHECK_THAT(s.latPerPixel, WithinAbs((-34.1845418 - -34.2163447) / 1792.0, 1e-15));
}

TEST_CASE("approxEquals honours the tolerance", "[boundingbox]") {
  const BoundingBox a = simonsTown();
  BoundingBox b = a;
  b.north += 5e-10;
  CHECK(a.approxEquals(b, 1e-9));
  b.north += 1e-8;
  CHECK_FALSE(a.approxEquals(b, 1e-9));
}
