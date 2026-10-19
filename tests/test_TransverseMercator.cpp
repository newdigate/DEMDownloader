#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <cmath>
#include <initializer_list>
#include <limits>
#include <optional>

#include "mesh/TransverseMercator.h"

using Catch::Matchers::WithinAbs;
using demdl::mesh::forwardTransverseMercator;
using demdl::mesh::Projected;
using demdl::mesh::TransverseMercatorParams;
using demdl::mesh::utmFromEpsg;

// The expected positions come from PROJ 9.4 (cs2cs with explicit +proj=tmerc and +proj=utm strings), not from this
// code, except where a test names another source. PROJ printed four decimals.

namespace {

// The design asks for a millimetre or so and leaves the series to the builder, so 2 mm is deliberately too coarse to
// pin one.
constexpr double kTolerance = 0.002;  // metres

// Lo19 as the City of Cape Town's elevation model uses it: central meridian 19 E, scale 1, no false origin, and the
// GRS 80 ellipsoid that the struct defaults to.
TransverseMercatorParams lo19() {
  TransverseMercatorParams p;
  p.originLongitudeDeg = 19.0;
  return p;
}

struct Point {
  double lon;
  double lat;
  double x;  // expected, metres
  double y;
};

void checkPositions(const TransverseMercatorParams& params, std::initializer_list<Point> points) {
  for (const Point& p : points) {
    CAPTURE(p.lon, p.lat);
    const Projected got = forwardTransverseMercator(params, p.lon, p.lat);
    CHECK_THAT(got.x, WithinAbs(p.x, kTolerance));
    CHECK_THAT(got.y, WithinAbs(p.y, kTolerance));
  }
}

}  // namespace

TEST_CASE("Lo19 positions match PROJ", "[tm]") {
  checkPositions(lo19(), {
      {18.44055175, -34.2111625310268, -51556.5699, -3787226.0192},
      {19.0, -34.0, 0.0, -3763661.4421},  // on the central meridian
      {18.3, -34.36, -64395.8507, -3803816.6955},
      // Three degrees either side, the edge of a UTM zone and where the series is least accurate. Mirror images:
      // west of the central meridian must give a negative x.
      {22.0, -34.0, 277202.1763, -3767721.8047},
      {16.0, -34.0, -277202.1763, -3767721.8047},
      // The equator, and latitudes 15 and 45 degrees south, where the series' third term (worth up to 8 mm) is at its
      // largest, so that a wrong third coefficient shows.
      {22.0, 0.0, 334112.2018, 0.0},
      {22.0, -15.0, 322780.0738, -1661178.1581},
      {22.0, -45.0, 236540.6424, -4989325.2347},
  });
}

TEST_CASE("the natural origin maps to the false origin and other points are measured from it", "[tm]") {
  TransverseMercatorParams p = lo19();
  p.originLatitudeDeg = -34.0;
  p.scale = 0.9996;
  p.falseEasting = 1000.0;
  p.falseNorthing = 2000.0;
  checkPositions(p, {
      {19.0, -34.0, 1000.0, 2000.0},
      {20.0, -33.0, 94417.7782, 112425.0035},
  });
}

TEST_CASE("another ellipsoid and an origin latitude match the Ordnance Survey's worked example", "[tm]") {
  // The British National Grid: Airy 1830, origin 49 N 2 W, scale 0.9996012717, false origin (400 km, -100 km).
  TransverseMercatorParams grid;
  grid.originLongitudeDeg = -2.0;
  grid.originLatitudeDeg = 49.0;
  grid.scale = 0.9996012717;
  grid.falseEasting = 400000.0;
  grid.falseNorthing = -100000.0;
  grid.semiMajorAxis = 6377563.396;
  grid.inverseFlattening = 299.3249646;
  // The worked example in annexe C.1 of the Ordnance Survey's "A Guide to Coordinate Systems in Great Britain" (V3.6):
  // 52 39 27.2531 N, 1 43 4.5177 E is grid position 651409.903, 313177.270. Unlike the other expected values, this
  // one does not come from PROJ (which agrees to 0.4 mm).
  checkPositions(grid, {
      {1.0 + 43.0 / 60.0 + 4.5177 / 3600.0, 52.0 + 39.0 / 60.0 + 27.2531 / 3600.0, 651409.903, 313177.270},
  });
}

TEST_CASE("the poles project to the length of a meridian quadrant", "[tm]") {
  // A crop box may reach a pole (BoundingBox allows latitude 90), where tan(lat) is huge and cos(lat) nearly zero.
  // 10001965.7292 m is a quarter of the GRS 80 meridian, and since all meridians meet there, x is the false
  // easting whatever the longitude.
  checkPositions(lo19(), {
      {19.0, 90.0, 0.0, 10001965.7292},
      {22.0, 90.0, 0.0, 10001965.7292},
      {19.0, -90.0, 0.0, -10001965.7292},
  });
}

TEST_CASE("a flattening of zero or infinity is a sphere", "[tm]") {
  // WKT writes a sphere as an inverse flattening of 0, and a file that gives a semi-minor axis equal to the semi-major
  // one leads to infinity. Either must give a position, not a division by zero.
  TransverseMercatorParams p;
  p.originLongitudeDeg = 21.0;
  p.originLatitudeDeg = -30.0;
  p.scale = 0.9996;
  p.falseEasting = 500000.0;
  p.falseNorthing = 10000000.0;
  for (const double inverseFlattening : {0.0, std::numeric_limits<double>::infinity()}) {
    CAPTURE(inverseFlattening);
    p.inverseFlattening = inverseFlattening;
    // PROJ: +a=6378137 +b=6378137
    checkPositions(p, {{22.0, -33.0, 593324.9634, 9665731.5223}});
  }
}

TEST_CASE("a coordinate that is not a finite number gives NaN rather than a position", "[tm]") {
  // The crop window projects a box's corners and takes their bounding rectangle, so a bad corner must be detectable
  // and must not pass for the origin.
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  const double coordinates[][2] = {{nan, -34.0}, {18.4, nan}, {inf, -34.0}, {18.4, inf}, {18.4, -inf}};
  for (const auto& c : coordinates) {
    CAPTURE(c[0], c[1]);
    const Projected got = forwardTransverseMercator(lo19(), c[0], c[1]);
    CHECK(std::isnan(got.x));
    CHECK(std::isnan(got.y));
  }
}

TEST_CASE("a longitude across the antimeridian projects against the nearest central meridian", "[tm]") {
  // Zone 60 is centred on 177 E and zone 1 on 177 W. A point 3.5 degrees from either is on the far side of 180,
  // where subtracting the central meridian naively leaves a difference of about 356 degrees. A series in the
  // difference itself, such as the USGS/Snyder polynomial, would give nonsense unless it wrapped the difference first.
  const std::optional<TransverseMercatorParams> zone60 = utmFromEpsg(32760);
  const std::optional<TransverseMercatorParams> zone1 = utmFromEpsg(32701);
  REQUIRE(zone60.has_value());
  REQUIRE(zone1.has_value());
  // Each place is given both ways round.
  checkPositions(*zone60, {
      {-179.5, -17.5, 871740.0289, 8061713.8041},
      {180.5, -17.5, 871740.0289, 8061713.8041},
  });
  checkPositions(*zone1, {
      {179.5, -17.5, 128259.9711, 8061713.8041},
      {-180.5, -17.5, 128259.9711, 8061713.8041},
  });
}

TEST_CASE("UTM south zone 34 has its central meridian, false northing and WGS 84 ellipsoid", "[tm]") {
  const std::optional<TransverseMercatorParams> p = utmFromEpsg(32734);
  REQUIRE(p.has_value());
  CHECK(p->originLongitudeDeg == 21.0);
  CHECK(p->originLatitudeDeg == 0.0);
  CHECK(p->scale == 0.9996);
  CHECK(p->falseEasting == 500000.0);
  CHECK(p->falseNorthing == 10000000.0);
  // WGS 84, not the GRS 80 that the struct defaults to: the inverse flattenings are 1.5e-6 apart, which moves a
  // position by about a tenth of a millimetre, too little for the positions below to tell.
  CHECK(p->semiMajorAxis == 6378137.0);
  CHECK(p->inverseFlattening == 298.257223563);

  checkPositions(*p, {{18.4241, -33.9249, 261881.5985, 6243182.3545}});
}

TEST_CASE("UTM north zones have no false northing", "[tm]") {
  const std::optional<TransverseMercatorParams> p = utmFromEpsg(32634);
  REQUIRE(p.has_value());
  CHECK(p->falseNorthing == 0.0);
  CHECK(p->originLongitudeDeg == 21.0);  // the same zone as 32734, in the other hemisphere
}

TEST_CASE("the first and last zone of each hemisphere are UTM zones", "[tm]") {
  struct Zone {
    int code;
    double centralMeridian;
    double falseNorthing;
  };
  const Zone zones[] = {
      {32601, -177.0, 0.0},
      {32660, 177.0, 0.0},
      {32701, -177.0, 10000000.0},
      {32760, 177.0, 10000000.0},
  };
  for (const Zone& z : zones) {
    CAPTURE(z.code);
    const std::optional<TransverseMercatorParams> p = utmFromEpsg(z.code);
    REQUIRE(p.has_value());
    CHECK(p->originLongitudeDeg == z.centralMeridian);
    CHECK(p->falseNorthing == z.falseNorthing);
  }
}

TEST_CASE("codes that are not UTM zones give nothing", "[tm]") {
  // 2048 is a south-oriented South African Lo grid and 20482 the City's own code for Lo19; 32600 and 32700 sit just
  // below zone 1, and 32661 and 32761 (the polar stereographic UPS grids) just above zone 60.
  const int codes[] = {2048, 20482, 32600, 32661, 32700, 32761, 0, -1,
                       std::numeric_limits<int>::min(), std::numeric_limits<int>::max()};
  for (const int code : codes) {
    CAPTURE(code);
    CHECK_FALSE(utmFromEpsg(code).has_value());
  }
}
