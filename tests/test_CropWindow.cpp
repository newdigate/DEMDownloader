#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <array>
#include <cmath>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>

#include "demdl/BoundingBox.h"
#include "mesh/CropWindow.h"
#include "mesh/GeoTiffGrid.h"
#include "mesh/TransverseMercator.h"

using Catch::Matchers::Message;
using Catch::Matchers::WithinAbs;
using demdl::BoundingBox;
using demdl::mesh::describeWindow;
using demdl::mesh::extentInMetres;
using demdl::mesh::forwardTransverseMercator;
using demdl::mesh::GridGeometry;
using demdl::mesh::metresPerDegree;
using demdl::mesh::MetresPerDegree;
using demdl::mesh::ModelKind;
using demdl::mesh::PixelWindow;
using demdl::mesh::pixelWindowFor;
using demdl::mesh::utmFromEpsg;
using demdl::mesh::WindowExtent;

// Where a window's expected numbers follow from positions that PROJ 9.4 gave (cs2cs with explicit +proj=tmerc
// strings), the comment beside it shows the working from those positions; none is read back from this code.

namespace {

constexpr const char* kTooSmall = "the crop window overlaps the file by fewer than 2 x 2 samples";

BoundingBox box(double north, double south, double east, double west) {
  BoundingBox b;
  b.north = north;
  b.south = south;
  b.east = east;
  b.west = west;
  return b;
}

// 100 x 80 point samples, 0.01 degrees apart: columns 0 to 99 are at 18.00 to 18.99 E and rows 0 to 79 at 33.00 to
// 33.79 S.
GridGeometry geographicGrid() {
  GridGeometry g;
  g.kind = ModelKind::Geographic;
  g.width = 100;
  g.height = 80;
  g.sampleX0 = 18.0;
  g.sampleY0 = -33.0;
  g.dx = 0.01;
  g.dy = 0.01;
  g.pixelIsPoint = true;
  return g;
}

// The City of Cape Town's 5 m model as the reader sees it: Lo19, which is central meridian 19 E and the GRS 80
// defaults, with point samples.
GridGeometry cityGrid() {
  GridGeometry g;
  g.kind = ModelKind::TransverseMercator;
  g.tm.originLongitudeDeg = 19.0;
  g.width = 12912;
  g.height = 19696;
  g.sampleX0 = -64012.5;
  g.sampleY0 = -3705051.806;
  g.dx = g.dy = 5.0;
  g.pixelIsPoint = true;
  return g;
}

// A Lo19 grid 250 km across, for boxes that straddle the central meridian, which the City's file (it ends 0.5 km
// east of it) cannot hold.
GridGeometry wideGrid() {
  GridGeometry g = cityGrid();
  g.width = 50000;
  g.height = 40000;
  g.sampleX0 = -125000.0;
  g.sampleY0 = -3700000.0;
  return g;
}

// WGS 84 / UTM zone 34S (EPSG 32734) at 10 m: scale 0.9996, false easting 500 km and false northing 10,000 km, with
// the Cape Peninsula 2.5 to 2.7 degrees west of the central meridian.
GridGeometry utmGrid() {
  GridGeometry g;
  g.kind = ModelKind::TransverseMercator;
  g.tm = *utmFromEpsg(32734);
  g.width = 10000;
  g.height = 10000;
  g.sampleX0 = 200000.0;
  g.sampleY0 = 6300000.0;
  g.dx = g.dy = 10.0;
  g.pixelIsPoint = true;
  return g;
}

void checkWindow(const PixelWindow& got, const PixelWindow& want) {
  CAPTURE(got.col0, got.row0, got.cols, got.rows);
  CHECK(got.col0 == want.col0);
  CHECK(got.row0 == want.row0);
  CHECK(got.cols == want.cols);
  CHECK(got.rows == want.rows);
}

void checkRefused(const GridGeometry& g, const BoundingBox& crop) {
  CAPTURE(crop.north, crop.south, crop.east, crop.west);
  CHECK_THROWS_MATCHES(pixelWindowFor(g, crop), std::runtime_error, Message(kTooSmall));
}

// What spec 5.1 asks of a Transverse Mercator crop, restated without the implementation: the window is the smallest
// range of whole samples that holds the bounding rectangle of the box's four corners and the midpoints of its four
// edges, once projected. The box must lie well inside the grid, or the window would be clipped.
void checkTightAroundProjectedPoints(const GridGeometry& g, const BoundingBox& crop) {
  const double midLon = (crop.west + crop.east) / 2;
  const double midLat = (crop.north + crop.south) / 2;
  const std::array<std::pair<double, double>, 8> points{{
      {crop.west, crop.north},
      {crop.east, crop.north},
      {crop.west, crop.south},
      {crop.east, crop.south},
      {midLon, crop.north},
      {midLon, crop.south},
      {crop.west, midLat},
      {crop.east, midLat},
  }};
  double minX = std::numeric_limits<double>::infinity(), maxX = -minX, minY = minX, maxY = -minX;
  for (const auto& [lon, lat] : points) {
    const auto p = forwardTransverseMercator(g.tm, lon, lat);
    minX = std::fmin(minX, p.x);
    maxX = std::fmax(maxX, p.x);
    minY = std::fmin(minY, p.y);
    maxY = std::fmax(maxY, p.y);
  }
  const PixelWindow w = pixelWindowFor(g, crop);
  const int lastCol = w.col0 + w.cols - 1;
  const int lastRow = w.row0 + w.rows - 1;
  CAPTURE(minX, maxX, minY, maxY, w.col0, w.row0, w.cols, w.rows);
  // It holds all eight points,
  CHECK(g.sampleX(w.col0) <= minX);
  CHECK(g.sampleX(lastCol) >= maxX);
  CHECK(g.sampleY(w.row0) >= maxY);
  CHECK(g.sampleY(lastRow) <= minY);
  // and it is the smallest that does: one column or row less on any side would leave a point outside.
  CHECK(g.sampleX(w.col0 + 1) > minX);
  CHECK(g.sampleX(lastCol - 1) < maxX);
  CHECK(g.sampleY(w.row0 + 1) < maxY);
  CHECK(g.sampleY(lastRow - 1) > minY);
}

}  // namespace

// ---- metres per degree ----

TEST_CASE("metres per degree at 34.2 degrees south", "[crop]") {
  const MetresPerDegree m = metresPerDegree(-34.2);
  CHECK_THAT(m.perDegreeLatitude, WithinAbs(110926.0, 0.5));
  // The brief quotes 92168.8 here, a metre too much for the 0.5 m it asks for: the spec's formula gives 92167.79 and
  // the ellipsoid itself, N cos(phi) pi / 180 with N = a / sqrt(1 - e^2 sin^2(phi)), gives 92167.71.
  CHECK_THAT(m.perDegreeLongitude, WithinAbs(92167.8, 0.5));
}

TEST_CASE("metres per degree follow the WGS 84 ellipsoid within a quarter of a metre up to 60 degrees", "[crop]") {
  // One degree of meridian, M(phi) pi / 180, and of parallel, N(phi) cos(phi) pi / 180, on a = 6378137 and
  // 1/f = 298.257223563, worked out separately from the spec's formulas that the code uses.
  struct Row {
    double lat;
    double perLatitude;
    double perLongitude;
  };
  for (const Row& row : {Row{0.0, 110574.28, 111319.49}, Row{30.0, 110852.44, 96486.28},
                         Row{45.0, 111131.78, 78846.84}, Row{60.0, 111412.29, 55800.00}}) {
    for (const double sign : {1.0, -1.0}) {
      CAPTURE(sign * row.lat);
      const MetresPerDegree m = metresPerDegree(sign * row.lat);
      CHECK_THAT(m.perDegreeLatitude, WithinAbs(row.perLatitude, 0.25));
      CHECK_THAT(m.perDegreeLongitude, WithinAbs(row.perLongitude, 0.25));
    }
  }
}

// ---- geographic files ----

TEST_CASE("no crop gives the whole file", "[crop]") {
  checkWindow(pixelWindowFor(geographicGrid(), std::nullopt), {0, 0, 100, 80});
  checkWindow(pixelWindowFor(cityGrid(), std::nullopt), {0, 0, 12912, 19696});
}

TEST_CASE("a geographic crop becomes the smallest range of whole samples around it", "[crop]") {
  SECTION("edges that fall half way between samples") {
    // 18.205 and 18.505 are 20.5 and 50.5 samples in, so the columns run 20 to 51; -33.105 and -33.395 are rows 10.5
    // and 39.5, so the rows run 10 to 40.
    checkWindow(pixelWindowFor(geographicGrid(), box(-33.105, -33.395, 18.505, 18.205)), {20, 10, 32, 31});
  }
  SECTION("cells that are not square, in the northern hemisphere") {
    GridGeometry g;
    g.width = 50;
    g.height = 100;
    g.sampleX0 = 10.0;
    g.sampleY0 = 50.0;
    g.dx = 0.02;
    g.dy = 0.005;
    g.pixelIsPoint = true;
    // 10.105 and 10.505 are columns 5.25 and 25.25; 49.9025 and 49.7025 are rows 19.5 and 59.5.
    checkWindow(pixelWindowFor(g, box(49.9025, 49.7025, 10.505, 10.105)), {5, 19, 22, 42});
  }
}

TEST_CASE("a crop cut along sample lines gives exactly those samples", "[crop]") {
  // 18.2 and 0.01 are not exact in binary, and (18.2 - 18.0) / 0.01 comes out as 19.99999999999993. Floor and ceil
  // alone would start the window at column 19, a column early, though this box starts on column 20. The error runs
  // either way from sample to sample, so every position is tried.
  const GridGeometry g = geographicGrid();
  for (const int span : {1, 2, 5}) {
    for (int first = 0; first + span < g.width; ++first) {
      CAPTURE(first, span);
      const PixelWindow w = pixelWindowFor(g, box(-33.1, -33.4, g.sampleX(first + span), g.sampleX(first)));
      CHECK(w.col0 == first);
      CHECK(w.cols == span + 1);
    }
    for (int first = 0; first + span < g.height; ++first) {
      CAPTURE(first, span);
      const PixelWindow w = pixelWindowFor(g, box(g.sampleY(first), g.sampleY(first + span), 18.9, 18.1));
      CHECK(w.row0 == first);
      CHECK(w.rows == span + 1);
    }
  }
}

TEST_CASE("an edge a hundredth of a sample off a sample line is not taken for being on it", "[crop]") {
  // The tolerance that lets 19.99999999999993 count as sample 20 is far smaller than that: 18.1999 and 18.2001 are
  // 19.99 and 20.01 samples in, so the box holds sample 20 with a hundredth of a cell to spare either side and needs
  // columns 19 to 21.
  checkWindow(pixelWindowFor(geographicGrid(), box(-33.1, -33.4, 18.2001, 18.1999)), {19, 10, 3, 31});
}

TEST_CASE("cropping to the whole box of a dl_elevation file gives the whole file", "[crop]") {
  // dl_elevation's samples sit on the pixels' north-west corners, so the last column is one cell short of the east
  // edge of the box it downloaded.
  const BoundingBox download = box(-34.1845418, -34.2163447, 18.4625244, 18.4185791);
  GridGeometry g;
  g.width = 2048;
  g.height = 1792;
  g.sampleX0 = download.west;
  g.sampleY0 = download.north;
  g.dx = (download.east - download.west) / g.width;
  g.dy = (download.north - download.south) / g.height;
  checkWindow(pixelWindowFor(g, download), {0, 0, 2048, 1792});
}

TEST_CASE("a crop beyond the file's edges is clipped to them", "[crop]") {
  const GridGeometry g = geographicGrid();
  SECTION("east") {
    const PixelWindow w = pixelWindowFor(g, box(-33.105, -33.395, 19.5, 18.905));
    checkWindow(w, {90, 10, 10, 31});
    CHECK(w.col0 + w.cols - 1 == g.width - 1);
  }
  SECTION("west") {
    checkWindow(pixelWindowFor(g, box(-33.105, -33.395, 18.305, 17.0)), {0, 10, 32, 31});
  }
  SECTION("north") {
    checkWindow(pixelWindowFor(g, box(-30.0, -33.395, 18.505, 18.205)), {20, 0, 32, 41});
  }
  SECTION("south") {
    const PixelWindow w = pixelWindowFor(g, box(-33.105, -50.0, 18.505, 18.205));
    checkWindow(w, {20, 10, 32, 70});
    CHECK(w.row0 + w.rows - 1 == g.height - 1);
  }
  SECTION("all four sides") {
    checkWindow(pixelWindowFor(g, box(-30.0, -50.0, 25.0, 10.0)), {0, 0, 100, 80});
  }
  SECTION("by absurd amounts, which must not overflow the conversion to int") {
    checkWindow(pixelWindowFor(g, box(1e300, -1e300, 1e300, -1e300)), {0, 0, 100, 80});
    checkWindow(pixelWindowFor(g, box(-33.105, -33.395, 1e300, 18.905)), {90, 10, 10, 31});
  }
}

TEST_CASE("a crop that misses the file or touches one column or row of it is refused", "[crop]") {
  const GridGeometry g = geographicGrid();
  SECTION("west of the file") { checkRefused(g, box(-33.1, -33.4, 17.0, 16.0)); }
  SECTION("east of the file") { checkRefused(g, box(-33.1, -33.4, 21.0, 19.0)); }
  SECTION("north of the file") { checkRefused(g, box(-32.0, -32.5, 18.5, 18.2)); }
  SECTION("south of the file") { checkRefused(g, box(-41.0, -42.0, 18.5, 18.2)); }
  SECTION("one column wide") { checkRefused(g, box(-33.1, -33.4, 18.2, 18.2)); }
  SECTION("one row high") { checkRefused(g, box(-33.2, -33.2, 18.5, 18.2)); }
  SECTION("ending on the first column") { checkRefused(g, box(-33.1, -33.4, 18.0, 17.5)); }
  SECTION("starting on the last column") { checkRefused(g, box(-33.1, -33.4, 19.5, g.sampleX(99))); }
  SECTION("ending on the first row") { checkRefused(g, box(-32.5, -33.0, 18.5, 18.2)); }
  SECTION("starting on the last row") { checkRefused(g, box(g.sampleY(79), -35.0, 18.5, 18.2)); }
  SECTION("a line along any one column") {
    for (int col = 0; col < g.width; ++col) {
      CAPTURE(col);
      checkRefused(g, box(-33.1, -33.4, g.sampleX(col), g.sampleX(col)));
    }
  }
  SECTION("a line along any one row") {
    for (int row = 0; row < g.height; ++row) {
      CAPTURE(row);
      checkRefused(g, box(g.sampleY(row), g.sampleY(row), 18.5, 18.2));
    }
  }
  SECTION("a file one sample wide has no window of two") {
    GridGeometry thin = g;
    thin.width = 1;
    checkRefused(thin, box(-33.1, -33.4, 18.5, 17.5));
  }
}

TEST_CASE("a file with one column or one row has no window of two, with or without a crop", "[crop]") {
  const char* const message = "the file has fewer than 2 x 2 samples";
  GridGeometry oneColumn = geographicGrid();
  oneColumn.width = 1;
  GridGeometry oneRow = geographicGrid();
  oneRow.height = 1;
  CHECK_THROWS_MATCHES(pixelWindowFor(oneColumn, std::nullopt), std::runtime_error, Message(message));
  CHECK_THROWS_MATCHES(pixelWindowFor(oneRow, std::nullopt), std::runtime_error, Message(message));
  // Two columns and two rows, the smallest mesh, are the whole file.
  GridGeometry twoByTwo = geographicGrid();
  twoByTwo.width = 2;
  twoByTwo.height = 2;
  checkWindow(pixelWindowFor(twoByTwo, std::nullopt), {0, 0, 2, 2});
}

TEST_CASE("a crop that reaches two columns and two rows is enough", "[crop]") {
  const GridGeometry g = geographicGrid();
  // 18.985 is column 98.5 and -33.785 is row 78.5, so the box reaches the last two columns and rows.
  checkWindow(pixelWindowFor(g, box(-33.785, -34.0, 19.5, 18.985)), {98, 78, 2, 2});
  // A box inside one cell is held by the two columns and two rows around it.
  checkWindow(pixelWindowFor(g, box(-33.2049, -33.2051, 18.2051, 18.2049)), {20, 20, 2, 2});
}

TEST_CASE("a crop that is not a box is refused", "[crop]") {
  const double nan = std::numeric_limits<double>::quiet_NaN();
  const double inf = std::numeric_limits<double>::infinity();
  // Geographic and Transverse Mercator alike: a NaN would reach the conversion to int, and a projected box turned
  // inside out would be taken for its mirror image.
  for (const GridGeometry& g : {geographicGrid(), cityGrid()}) {
    CAPTURE(g.kind == ModelKind::Geographic);
    CHECK_THROWS_AS(pixelWindowFor(g, box(nan, -33.4, 18.5, 18.2)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.1, nan, 18.5, 18.2)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.1, -33.4, nan, 18.2)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.1, -33.4, 18.5, nan)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(inf, -33.4, 18.5, 18.2)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.1, -inf, 18.5, 18.2)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.1, -33.4, inf, 18.2)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.1, -33.4, 18.5, -inf)), std::invalid_argument);
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.4, -33.1, 18.5, 18.2)), std::invalid_argument);  // north below south
    CHECK_THROWS_AS(pixelWindowFor(g, box(-33.1, -33.4, 18.2, 18.5)), std::invalid_argument);  // east west of west
  }
}

// ---- Transverse Mercator files ----

TEST_CASE("a Transverse Mercator crop is projected before it is cut", "[crop]") {
  const GridGeometry g = cityGrid();
  const PixelWindow w = pixelWindowFor(g, box(-34.1845418, -34.2163447, 18.4625244, 18.4185791));
  // The recorded land point, (18.44055175, -34.2111625310268), is sample (2491, 16435).
  CHECK(w.col0 <= 2491);
  CHECK(2491 < w.col0 + w.cols);
  CHECK(w.row0 <= 16435);
  CHECK(16435 < w.row0 + w.rows);
  // About 4.0 km at 5 m.
  CHECK(w.cols >= 800);
  CHECK(w.cols <= 830);
  // The box's eight points lie at x from -53598.352 to -49528.601 and y from -3787812.204 to -3784262.142 (PROJ
  // 9.4). With samples 5 m apart from x = -64012.5 and y = -3705051.806, that is 2082.83 to 2896.78 columns across
  // and 15842.07 to 16552.08 rows down, so the columns run 2082 to 2897 and the rows 15842 to 16553.
  checkWindow(w, {2082, 15842, 816, 712});
}

TEST_CASE("a Transverse Mercator crop holds the midpoints of the box's edges as well as its corners", "[crop]") {
  // The box is two degrees wide across the central meridian, where the parallel at 34 S bulges 450 m north of its
  // ends: PROJ 9.4 puts the north corners at y = -3764112.305 and the north edge's midpoint, (19, -34), at
  // -3763661.442. Samples 5 m apart from y = -3700000 make the top row 12732.29 down, so 12732; the corners alone
  // would give 12822. The columns reach x = +-92386.558 (6522.69 and 43477.31 across) and the bottom row is
  // 17259.68 down, the south corners being at y = -3786298.409.
  checkWindow(pixelWindowFor(wideGrid(), box(-34.0, -34.2, 20.0, 18.0)), {6522, 12732, 36957, 4529});
}

TEST_CASE("a Transverse Mercator window is the smallest that holds the projected box", "[crop]") {
  SECTION("the City's file, Lo19") {
    checkTightAroundProjectedPoints(cityGrid(), box(-34.1845418, -34.2163447, 18.4625244, 18.4185791));
  }
  SECTION("across the central meridian") {
    checkTightAroundProjectedPoints(wideGrid(), box(-34.0, -34.2, 20.0, 18.0));
  }
  SECTION("a UTM zone, with a scale, a false easting and a false northing") {
    checkTightAroundProjectedPoints(utmGrid(), box(-33.9, -34.2, 18.5, 18.3));
  }
}

TEST_CASE("a Transverse Mercator crop outside the file is refused", "[crop]") {
  // Johannesburg is 800 km east of the City's model and 800 km north.
  checkRefused(cityGrid(), box(-26.0, -26.3, 28.2, 28.0));
  // Cape Town's own latitudes, but 65 to 83 km west of the file's west edge.
  checkRefused(cityGrid(), box(-34.0, -34.2, 17.6, 17.4));
}

TEST_CASE("a crop that projects to enormous coordinates is clipped, not converted", "[crop]") {
  // 90 degrees from the central meridian on the equator the projection is singular. The box has its east edge at
  // 109 E, and the midpoint of that edge, (109, 0), comes out at x = 1e127 m; its corners, 0.01 degrees off the
  // equator, at x = 2e27 m and y = +-7e15 m. All finite, all far beyond int. Converted to int before the clamp, the
  // east edge became INT_MIN on x86-64 and a window that reaches the file's east side was refused.
  GridGeometry g;
  g.kind = ModelKind::TransverseMercator;
  g.tm.originLongitudeDeg = 19.0;
  g.width = 1000;
  g.height = 1000;
  g.sampleX0 = -2500.0;
  g.sampleY0 = 1500.0;
  g.dx = g.dy = 5.0;
  g.pixelIsPoint = true;
  // The west edge, 0.01 degrees west of the central meridian, is 1113.2 m west of it: column 277.36.
  checkWindow(pixelWindowFor(g, box(0.01, -0.01, 109.0, 18.99)), {277, 0, 723, 1000});
}

// ---- extent in metres ----

TEST_CASE("the extent of a Transverse Mercator window is its sample count less one times the cell", "[crop]") {
  SECTION("square cells") {
    const GridGeometry g = cityGrid();
    const PixelWindow w = pixelWindowFor(g, box(-34.1845418, -34.2163447, 18.4625244, 18.4185791));
    const WindowExtent e = extentInMetres(g, w);
    CHECK(e.cellDxM == 5.0);
    CHECK(e.cellDyM == 5.0);
    CHECK_THAT(e.widthM, WithinAbs((w.cols - 1) * 5.0, 1e-9));
    CHECK_THAT(e.heightM, WithinAbs((w.rows - 1) * 5.0, 1e-9));
    CHECK_THAT(e.widthM, WithinAbs(4075.0, 1e-9));  // 815 cells
    CHECK_THAT(e.heightM, WithinAbs(3555.0, 1e-9));  // 711 cells
  }
  SECTION("cells that are not square") {
    GridGeometry g = cityGrid();
    g.dy = 2.0;
    const WindowExtent e = extentInMetres(g, {10, 20, 30, 40});
    CHECK(e.cellDxM == 5.0);
    CHECK(e.cellDyM == 2.0);
    CHECK_THAT(e.widthM, WithinAbs(29 * 5.0, 1e-9));
    CHECK_THAT(e.heightM, WithinAbs(39 * 2.0, 1e-9));
  }
}

TEST_CASE("the extent of a geographic window uses the metres per degree at its middle latitude", "[crop]") {
  const GridGeometry g = geographicGrid();
  SECTION("the 32 x 31 window of the crop") {
    const PixelWindow w = pixelWindowFor(g, box(-33.105, -33.395, 18.505, 18.205));
    const WindowExtent e = extentInMetres(g, w);
    // Rows 10 to 40 run from 33.10 S to 33.40 S, so the middle is 33.25 S.
    const MetresPerDegree m = metresPerDegree(-33.25);
    CHECK_THAT(e.cellDxM, WithinAbs(0.01 * m.perDegreeLongitude, 1e-6));
    CHECK_THAT(e.cellDyM, WithinAbs(0.01 * m.perDegreeLatitude, 1e-6));
    CHECK_THAT(e.widthM, WithinAbs(31 * 0.01 * m.perDegreeLongitude, 0.01));
    CHECK_THAT(e.heightM, WithinAbs(30 * 0.01 * m.perDegreeLatitude, 0.01));
    // The same, from the spec's formulas worked out separately: 93188.85 and 110908.92 m per degree.
    CHECK_THAT(e.widthM, WithinAbs(28888.54, 0.01));
    CHECK_THAT(e.heightM, WithinAbs(33272.68, 0.01));
  }
  SECTION("each window has its own middle latitude") {
    // Rows 0 and 1 lie at 33.00 S and 33.01 S, rows 78 and 79 at 33.78 S and 33.79 S.
    const WindowExtent north = extentInMetres(g, {0, 0, 2, 2});
    const WindowExtent south = extentInMetres(g, {0, 78, 2, 2});
    CHECK_THAT(north.cellDxM, WithinAbs(0.01 * metresPerDegree(-33.005).perDegreeLongitude, 1e-6));
    CHECK_THAT(south.cellDxM, WithinAbs(0.01 * metresPerDegree(-33.785).perDegreeLongitude, 1e-6));
    CHECK(south.cellDxM < north.cellDxM - 5.0);  // a cell is about 8 m narrower 0.78 degrees closer to the pole
  }
  SECTION("cells that are not square") {
    GridGeometry wide = g;
    wide.dx = 0.02;
    wide.dy = 0.005;
    // Rows 0 to 19 run from 33.000 S to 33.095 S, so the middle is 33.0475 S.
    const WindowExtent e = extentInMetres(wide, {0, 0, 10, 20});
    const MetresPerDegree m = metresPerDegree(-33.0475);
    CHECK_THAT(e.cellDxM, WithinAbs(0.02 * m.perDegreeLongitude, 1e-6));
    CHECK_THAT(e.cellDyM, WithinAbs(0.005 * m.perDegreeLatitude, 1e-6));
    CHECK_THAT(e.widthM, WithinAbs(9 * 0.02 * m.perDegreeLongitude, 1e-6));
    CHECK_THAT(e.heightM, WithinAbs(19 * 0.005 * m.perDegreeLatitude, 1e-6));
  }
}

// ---- the dry-run line ----

TEST_CASE("describeWindow gives the pixels and the extent of the samples on one line", "[crop]") {
  SECTION("geographic: in degrees") {
    const std::string text = describeWindow(geographicGrid(), {20, 10, 32, 31});
    CHECK(text ==
          "32 x 31 pixels at (20, 10), samples from longitude 18.2000000 to 18.5100000 and latitude -33.1000000 to "
          "-33.4000000 degrees");
  }
  SECTION("Transverse Mercator: in projected metres") {
    // Columns 2082 to 2897 and rows 15842 to 16553 of the City's model.
    const std::string text = describeWindow(cityGrid(), {2082, 15842, 816, 712});
    CHECK(text ==
          "816 x 712 pixels at (2082, 15842), samples from easting -53602.50 to -49527.50 m and northing "
          "-3784261.81 to -3787816.81 m");
  }
}
