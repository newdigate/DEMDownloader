#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <limits>
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

TEST_CASE("non-finite values are rejected", "[boundingbox]") {
  BoundingBox nanNorth = simonsTown();
  nanNorth.north = std::numeric_limits<double>::quiet_NaN();
  CHECK(nanNorth.validate() == "bounding box values must be finite numbers");
  BoundingBox infEast = simonsTown();
  infEast.east = std::numeric_limits<double>::infinity();
  CHECK(infEast.validate() == "bounding box values must be finite numbers");
}

TEST_CASE("equal edges are rejected and range limits are inclusive", "[boundingbox]") {
  const BoundingBox flatLat{1.0, 1.0, 2.0, 1.0};  // north == south
  CHECK(flatLat.validate() == "north must be greater than south");
  const BoundingBox flatLon{2.0, 1.0, 1.0, 1.0};  // east == west
  CHECK(flatLon.validate() == "east must be greater than west");
  const BoundingBox world{90.0, -90.0, 180.0, -180.0};
  CHECK(world.validate().empty());
  BoundingBox farSouth = simonsTown();
  farSouth.south = -90.5;
  CHECK(farSouth.validate() == "latitudes must be within [-90, 90]");
  BoundingBox farEast = simonsTown();
  farEast.east = 180.5;
  CHECK(farEast.validate() == "longitudes must be within [-180, 180]");
}

TEST_CASE("approxEquals checks every edge", "[boundingbox]") {
  // The comparison is inclusive, so zero tolerance still accepts an identical box.
  CHECK(simonsTown().approxEquals(simonsTown(), 0.0));
  const BoundingBox a = simonsTown();
  const auto edges = {&BoundingBox::north, &BoundingBox::south, &BoundingBox::east,
                      &BoundingBox::west};
  for (double BoundingBox::*edge : edges) {
    BoundingBox b = a;
    b.*edge += 5e-10;
    CHECK(a.approxEquals(b, 1e-9));
    b.*edge += 1e-8;
    CHECK_FALSE(a.approxEquals(b, 1e-9));
    // A NaN edge never matches, even at a huge tolerance and in either argument order.
    BoundingBox withNan = a;
    withNan.*edge = std::numeric_limits<double>::quiet_NaN();
    CHECK_FALSE(a.approxEquals(withNan, 1.0));
    CHECK_FALSE(withNan.approxEquals(a, 1.0));
  }
}
