#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <array>
#include <climits>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "GeoTiffFixtures.h"
#include "TempDir.h"
#include "demdl/BoundingBox.h"
#include "mesh/GeoTiffGrid.h"
#include "mesh/TransverseMercator.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using Catch::Matchers::WithinAbs;
using demdl::BoundingBox;
using demdl::mesh::GridGeometry;
using demdl::mesh::inspectGeoTiff;
using demdl::mesh::ModelKind;
using demdl::mesh::PixelWindow;
using demdl::mesh::readWindow;
using demdl::mesh::TransverseMercatorParams;

namespace {

using Path = std::filesystem::path;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

// The Simon's Town box of the design, as dl_elevation would download it.
const BoundingBox kBox{-34.18, -34.22, 18.47, 18.40};

// Lo19 as the City of Cape Town's model states it: central meridian 19 E and otherwise the struct's defaults.
TransverseMercatorParams lo19() {
  TransverseMercatorParams p;
  p.originLongitudeDeg = 19.0;
  return p;
}

// A 4 x 3 field, row 0 north. 0 is the no-data value of the fixtures that declare one; the NaN is a sample nobody
// measured.
const std::vector<double> kHeights = {
    12.5,   0.0,  -3.25, 100.75,  //
    1590.5, 7.0,  8.0,   9.0,     //
    -37.5,  10.0, 11.0,  kNaN,
};

// kHeights as readWindow returns them from a file whose no-data is 0.
const std::vector<double> kRead = {
    12.5,   kNaN, -3.25, 100.75,  //
    1590.5, 7.0,  8.0,   9.0,     //
    -37.5,  10.0, 11.0,  kNaN,
};

const PixelWindow kWhole{0, 0, 4, 3};

// The City's georeferencing in miniature: its corner as the tiepoint, 5 m pixels, no-data 0.
void cityTags(TIFF* t) {
  setTiepoint(t, {0.0, 0.0}, {-64012.5, -3705051.806});
  setPixelScale(t, 5.0, 5.0);
  setNoData(t, 0.0);
}

void cityKeys(GTIF* g) { setTmKeys(g, lo19(), true); }

// The City's model in miniature: point samples of Lo19, 5 m apart from the City's corner, no-data 0.
Path writeCityFixture(const Path& dir, Layout layout = Layout::Strips, SampleKind kind = SampleKind::Float32,
                      const std::vector<double>& heights = kHeights) {
  return writeTmFixture(dir, 4, 3, lo19(), {0.0, 0.0}, {-64012.5, -3705051.806}, 5.0, 5.0, true, 0.0, heights,
                        layout, kind);
}

// got against want sample by sample, NaN matching NaN.
void checkHeights(const std::vector<float>& got, const std::vector<double>& want) {
  CHECK(got.size() == want.size());
  if (got.size() != want.size()) return;
  for (std::size_t i = 0; i < want.size(); ++i) {
    CAPTURE(i);
    if (std::isnan(want[i])) {
      CHECK(std::isnan(got[i]));
    } else {
      CHECK(got[i] == static_cast<float>(want[i]));
    }
  }
}

// A 4 x 2 TIFF of zeros with no georeferencing.
void writePlainTiff(const Path& file, int samplesPerPixel, int bitsPerSample, int sampleFormat) {
  TIFF* tif = XTIFFOpen(file.string().c_str(), "w");
  REQUIRE(tif != nullptr);
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, 4u);
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, 2u);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, samplesPerPixel);
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, bitsPerSample);
  TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, sampleFormat);
  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, samplesPerPixel == 3 ? PHOTOMETRIC_RGB : PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tif, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 2u);
  std::vector<unsigned char> row(static_cast<std::size_t>(4 * samplesPerPixel * bitsPerSample / 8), 0);
  for (uint32_t y = 0; y < 2; ++y) REQUIRE(TIFFWriteScanline(tif, row.data(), y, 0) == 1);
  XTIFFClose(tif);
}

}  // namespace

TEST_CASE("a dl_elevation file is geographic, with its samples on the pixel corners", "[geotiff]") {
  TempDir dir;
  const GridGeometry g = inspectGeoTiff(writeGeographicFixture(dir.path, 4, 3, kBox, std::vector<double>(12, 100.0)));
  CHECK(g.kind == ModelKind::Geographic);
  CHECK(g.width == 4);
  CHECK(g.height == 3);
  CHECK_FALSE(g.pixelIsPoint);
  CHECK(g.dlElevation);
  CHECK_THAT(g.description, StartsWith("dl_elevation;"));
  // The file says RasterPixelIsArea, whose rule would put sample (0, 0) half a pixel in, at 18.40875, -34.18667; the
  // server sampled the corner.
  CHECK_THAT(g.sampleX0, WithinAbs(18.40, 1e-12));
  CHECK_THAT(g.sampleY0, WithinAbs(-34.18, 1e-12));
  CHECK_THAT(g.dx, WithinAbs(0.0175, 1e-12));
  CHECK_THAT(g.dy, WithinAbs(0.04 / 3.0, 1e-12));
  REQUIRE(g.noData.has_value());
  CHECK(*g.noData == -32768.0);
}

TEST_CASE("a dl_elevation file's unfetched pixels and its no-data read as NaN", "[geotiff]") {
  TempDir dir;
  const Path file = writeGeographicFixture(dir.path, 4, 3, kBox,
                                           {
                                               kNaN, -32768.0, 5.5, 100.0,  //
                                               1.0,  2.0,      3.0, 4.0,    //
                                               5.0,  6.0,      7.0, -32768.0,
                                           });
  checkHeights(readWindow(file, inspectGeoTiff(file), kWhole), {
                                                                   kNaN, kNaN, 5.5, 100.0,  //
                                                                   1.0,  2.0,  3.0, 4.0,    //
                                                                   5.0,  6.0,  7.0, kNaN,
                                                               });
}

TEST_CASE("a point file like the City's has sample (0, 0) on its tiepoint", "[geotiff]") {
  TempDir dir;
  const GridGeometry g = inspectGeoTiff(writeCityFixture(dir.path));
  CHECK(g.kind == ModelKind::TransverseMercator);
  CHECK(g.tm.originLongitudeDeg == 19.0);
  CHECK(g.width == 4);
  CHECK(g.height == 3);
  CHECK(g.pixelIsPoint);
  CHECK_FALSE(g.dlElevation);
  CHECK(g.description.empty());
  CHECK(g.sampleX0 == -64012.5);
  CHECK(g.sampleY0 == -3705051.806);
  CHECK(g.dx == 5.0);
  CHECK(g.dy == 5.0);
  // Columns run east and rows south.
  CHECK(g.sampleX(3) == -63997.5);
  CHECK_THAT(g.sampleY(2), WithinAbs(-3705061.806, 1e-6));
  REQUIRE(g.noData.has_value());
  CHECK(*g.noData == 0.0);
}

TEST_CASE("every Transverse Mercator parameter comes from its own GeoKey", "[geotiff]") {
  // No two alike and none the struct's default, so a key read into the wrong field, or not read at all, shows. The
  // ellipsoid is Clarke 1880, given by its axes beside the code for GRS 1980, as the City's file gives both.
  TransverseMercatorParams p;
  p.originLongitudeDeg = 21.0;
  p.originLatitudeDeg = -30.0;
  p.scale = 0.9996;
  p.falseEasting = 500000.0;
  p.falseNorthing = 10000000.0;
  p.semiMajorAxis = 6378249.145;
  p.inverseFlattening = 293.465;
  TempDir dir;
  const GridGeometry g = inspectGeoTiff(writeTmFixture(dir.path, 4, 3, p, {0.0, 0.0}, {1000.0, 2000.0}, 5.0, 5.0,
                                                       true, std::nullopt, kHeights, Layout::Strips,
                                                       SampleKind::Float32));
  CHECK(g.tm.originLongitudeDeg == 21.0);
  CHECK(g.tm.originLatitudeDeg == -30.0);
  CHECK(g.tm.scale == 0.9996);
  CHECK(g.tm.falseEasting == 500000.0);
  CHECK(g.tm.falseNorthing == 10000000.0);
  CHECK(g.tm.semiMajorAxis == 6378249.145);
  CHECK(g.tm.inverseFlattening == 293.465);
}

TEST_CASE("a tiepoint away from raster (0, 0) moves sample (0, 0) by whole pixels", "[geotiff]") {
  // Raster (2, 1) is tied to model (1000, 2000) with pixels 10 wide and 20 tall, so raster (0, 0), the north-west
  // corner of pixel (0, 0), is two pixels west and one north of it, at (980, 2020).
  TempDir dir;
  const auto inspectTied = [&](bool pixelIsPoint) {
    return inspectGeoTiff(writeTmFixture(dir.path, 4, 3, lo19(), {2.0, 1.0}, {1000.0, 2000.0}, 10.0, 20.0,
                                         pixelIsPoint, std::nullopt, kHeights, Layout::Strips, SampleKind::Float32));
  };
  SECTION("an area sample sits at the pixel's centre, half a pixel east and south") {
    const GridGeometry g = inspectTied(false);
    CHECK_FALSE(g.pixelIsPoint);
    CHECK(g.sampleX0 == 985.0);
    CHECK(g.sampleY0 == 2010.0);
    CHECK(g.sampleX(3) == 1015.0);
    CHECK(g.sampleY(2) == 1970.0);
  }
  SECTION("a point sample sits on its raster position") {
    const GridGeometry g = inspectTied(true);
    CHECK(g.pixelIsPoint);
    CHECK(g.sampleX0 == 980.0);
    CHECK(g.sampleY0 == 2020.0);
  }
}

TEST_CASE("an area file that dl_elevation did not write has its samples at the pixel centres", "[geotiff]") {
  // Geographic and with no GTRasterTypeGeoKey at all, which means area.
  TempDir dir;
  const Path file = dir.path / "area.tif";
  writeFixtureFile(
      file, {4, 3, kHeights},
      [](TIFF* t) {
        setTiepoint(t, {0.0, 0.0}, {18.0, -34.0});
        setPixelScale(t, 0.25, 0.5);
      },
      setGeographicKeys);
  const GridGeometry g = inspectGeoTiff(file);
  CHECK(g.kind == ModelKind::Geographic);
  CHECK_FALSE(g.pixelIsPoint);
  CHECK_FALSE(g.dlElevation);
  CHECK(g.sampleX0 == 18.125);
  CHECK(g.sampleY0 == -34.25);
  CHECK_FALSE(g.noData.has_value());
}

TEST_CASE("a ModelTransformation without rotation places samples as a tiepoint and a pixel scale would", "[geotiff]") {
  // x = 10 i + 1000 and y = -20 j + 2000: raster (0, 0) at (1000, 2000), pixels 10 east by 20 south.
  TempDir dir;
  const Path file = dir.path / "matrix.tif";
  writeFixtureFile(
      file, {4, 3, kHeights},
      [](TIFF* t) {
        setTransformation(t, {10.0, 0.0, 0.0, 1000.0, 0.0, -20.0, 0.0, 2000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
      },
      [](GTIF* g) { setTmKeys(g, lo19(), false); });
  const GridGeometry g = inspectGeoTiff(file);
  CHECK(g.dx == 10.0);
  CHECK(g.dy == 20.0);
  CHECK(g.sampleX0 == 1005.0);
  CHECK(g.sampleY0 == 1990.0);
}

TEST_CASE("readWindow returns every sample, with no-data and NaN as NaN, in each layout and sample type",
          "[geotiff]") {
  struct Case {
    const char* label;
    Layout layout;
    SampleKind kind;
    std::vector<double> heights;
    std::vector<double> want;
  };
  const std::vector<double> int16s = {12, 0, -3, 100, 1590, 7, 8, 9, -38, 10, 11, 12};
  const std::vector<double> int16sRead = {12, kNaN, -3, 100, 1590, 7, 8, 9, -38, 10, 11, 12};
  const Case cases[] = {
      {"float32 strips", Layout::Strips, SampleKind::Float32, kHeights, kRead},
      {"float32 tiles", Layout::Tiles, SampleKind::Float32, kHeights, kRead},
      {"float64 strips", Layout::Strips, SampleKind::Float64, kHeights, kRead},
      {"int16 strips", Layout::Strips, SampleKind::Int16, int16s, int16sRead},
      {"int16 tiles", Layout::Tiles, SampleKind::Int16, int16s, int16sRead},
      // 65000 would read as -536 taken as signed, and 100000 does not fit 16 bits.
      {"uint16 strips", Layout::Strips, SampleKind::UInt16, {12, 0, 3, 100, 1590, 7, 8, 9, 38, 10, 11, 65000},
       {12, kNaN, 3, 100, 1590, 7, 8, 9, 38, 10, 11, 65000}},
      {"int32 strips", Layout::Strips, SampleKind::Int32, {12, 0, -3, 100, 1590, 7, 8, 9, -38, 10, 11, 100000},
       {12, kNaN, -3, 100, 1590, 7, 8, 9, -38, 10, 11, 100000}},
  };
  for (const Case& c : cases) {
    CAPTURE(c.label);
    TempDir dir;
    const Path file = writeCityFixture(dir.path, c.layout, c.kind, c.heights);
    checkHeights(readWindow(file, inspectGeoTiff(file), kWhole), c.want);
  }
}

TEST_CASE("readWindow returns the block it is asked for, starting inside a strip or a tile", "[geotiff]") {
  // Strips hold two rows, so row 1 is the second row of the first strip, and LZW strips decode only from their start.
  struct Case {
    const char* label;
    Layout layout;
    std::uint16_t compression;
  };
  const Case cases[] = {{"strips", Layout::Strips, COMPRESSION_NONE},
                        {"tiles", Layout::Tiles, COMPRESSION_NONE},
                        {"LZW strips", Layout::Strips, COMPRESSION_LZW},
                        {"LZW tiles", Layout::Tiles, COMPRESSION_LZW}};
  for (const Case& c : cases) {
    CAPTURE(c.label);
    TempDir dir;
    const Path file = dir.path / "block.tif";
    writeFixtureFile(file, {4, 3, kHeights, c.layout, SampleKind::Float32, c.compression}, cityTags, cityKeys);
    const GridGeometry g = inspectGeoTiff(file);
    checkHeights(readWindow(file, g, {1, 1, 2, 2}), {7.0, 8.0, 10.0, 11.0});
    checkHeights(readWindow(file, g, {2, 1, 2, 2}), {8.0, 9.0, 11.0, kNaN});
    checkHeights(readWindow(file, g, {0, 2, 4, 1}), {-37.5, 10.0, 11.0, kNaN});
    checkHeights(readWindow(file, g, {1, 0, 1, 1}), {kNaN});
  }
}

TEST_CASE("a window across tile boundaries takes each sample from the right tile", "[geotiff]") {
  // 37 x 21 samples in 16 x 16 tiles: three tiles across, the last mostly padding, and two down. Sample (c, r) holds
  // c + 100 r.
  const int width = 37;
  const int height = 21;
  std::vector<double> heights;
  for (int r = 0; r < height; ++r) {
    for (int c = 0; c < width; ++c) heights.push_back(c + 100.0 * r);
  }
  TempDir dir;
  const Path file = writeTmFixture(dir.path, width, height, lo19(), {0.0, 0.0}, {0.0, 0.0}, 5.0, 5.0, true,
                                   std::nullopt, heights, Layout::Tiles, SampleKind::Float32);
  const PixelWindow w{14, 13, 20, 6};  // columns 14 to 33, rows 13 to 18
  const std::vector<float> got = readWindow(file, inspectGeoTiff(file), w);
  REQUIRE(got.size() == 120);
  CHECK(got[0] == 1314.0f);    // column 14, row 13
  CHECK(got[2] == 1316.0f);    // column 16, row 13: the second tile across
  CHECK(got[18] == 1332.0f);   // column 32, row 13: the third
  CHECK(got[60] == 1614.0f);   // column 14, row 16: the second tile down
  CHECK(got[119] == 1833.0f);  // column 33, row 18
  int wrong = 0;
  for (int r = 0; r < w.rows; ++r) {
    for (int c = 0; c < w.cols; ++c) {
      if (got[static_cast<std::size_t>(r * w.cols + c)] != static_cast<float>(w.col0 + c + 100 * (w.row0 + r))) ++wrong;
    }
  }
  CHECK(wrong == 0);
}

TEST_CASE("an infinite height, or a float64 one beyond float32's range, is no data", "[geotiff]") {
  // No terrain is infinitely high, and a NaN is filled from its neighbours where an infinity would wreck the mesh.
  const double inf = std::numeric_limits<double>::infinity();
  struct Case {
    const char* label;
    SampleKind kind;
    std::vector<double> heights;
  };
  const Case cases[] = {{"float32", SampleKind::Float32, {inf, -inf, 5.0, 6.0}},
                        {"float64", SampleKind::Float64, {inf, -1e300, 5.0, 6.0}}};
  for (const Case& c : cases) {
    CAPTURE(c.label);
    TempDir dir;
    const Path file = dir.path / "infinite.tif";
    writeFixtureFile(file, {2, 2, c.heights, Layout::Strips, c.kind}, cityTags, cityKeys);
    checkHeights(readWindow(file, inspectGeoTiff(file), {0, 0, 2, 2}), {kNaN, kNaN, 5.0, 6.0});
  }
}

TEST_CASE("without a GDAL_NODATA tag a zero is a height", "[geotiff]") {
  TempDir dir;
  const Path file = writeTmFixture(dir.path, 4, 3, lo19(), {0.0, 0.0}, {-64012.5, -3705051.806}, 5.0, 5.0, true,
                                   std::nullopt, kHeights, Layout::Strips, SampleKind::Float32);
  const GridGeometry g = inspectGeoTiff(file);
  CHECK_FALSE(g.noData.has_value());
  checkHeights(readWindow(file, g, {0, 0, 2, 1}), {12.5, 0.0});
}

TEST_CASE("a no-data value given with fewer digits than a float32 holds still matches its samples", "[geotiff]") {
  // Some writers give the lowest float32 as -3.40282346639e+038; read exactly, that double is no float's value.
  TempDir dir;
  const Path file = dir.path / "lowest.tif";
  writeFixtureFile(
      file, {2, 1, {static_cast<double>(std::numeric_limits<float>::lowest()), 5.0}},
      [](TIFF* t) {
        setTiepoint(t, {0.0, 0.0}, {-64012.5, -3705051.806});
        setPixelScale(t, 5.0, 5.0);
        setNoDataText(t, "-3.40282346639e+038");
      },
      cityKeys);
  checkHeights(readWindow(file, inspectGeoTiff(file), {0, 0, 2, 1}), {kNaN, 5.0});
}

TEST_CASE("the ellipsoid comes from its axes, else its code, else GRS 80, and never from a missing flattening",
          "[geotiff]") {
  TempDir dir;
  const Path file = dir.path / "ellipsoid.tif";
  // The City's keys without its ellipsoid, which each section gives its own way.
  const auto inspectWith = [&](const std::function<void(GTIF*)>& ellipsoid) {
    writeFixtureFile(file, {4, 3, kHeights}, cityTags, [&](GTIF* g) {
      setTmProjectionKeys(g, lo19(), true);
      ellipsoid(g);
    });
    return inspectGeoTiff(file).tm;
  };
  SECTION("WGS 84 by code") {
    const TransverseMercatorParams tm =
        inspectWith([](GTIF* g) { setShortKey(g, GeogEllipsoidGeoKey, Ellipse_WGS_84); });
    CHECK(tm.semiMajorAxis == 6378137.0);
    CHECK(tm.inverseFlattening == 298.257223563);
  }
  SECTION("GRS 1980 by code") {
    const TransverseMercatorParams tm =
        inspectWith([](GTIF* g) { setShortKey(g, GeogEllipsoidGeoKey, Ellipse_GRS_1980); });
    CHECK(tm.semiMajorAxis == 6378137.0);
    CHECK(tm.inverseFlattening == 298.257222101);
  }
  SECTION("nothing at all is GRS 80") {
    const TransverseMercatorParams tm = inspectWith([](GTIF*) {});
    CHECK(tm.semiMajorAxis == 6378137.0);
    CHECK(tm.inverseFlattening == 298.257222101);
  }
  SECTION("a semi-major axis alone keeps the code's flattening") {
    const TransverseMercatorParams tm = inspectWith([](GTIF* g) {
      setShortKey(g, GeogEllipsoidGeoKey, Ellipse_WGS_84);
      setDoubleKey(g, GeogSemiMajorAxisGeoKey, 6378249.145);
    });
    CHECK(tm.semiMajorAxis == 6378249.145);
    CHECK(tm.inverseFlattening == 298.257223563);
  }
  SECTION("a semi-minor axis gives the flattening when the file has no inverse flattening") {
    // Clarke 1866 by its axes: 6378206.4 / (6378206.4 - 6356583.8).
    const TransverseMercatorParams tm = inspectWith([](GTIF* g) {
      setDoubleKey(g, GeogSemiMajorAxisGeoKey, 6378206.4);
      setDoubleKey(g, GeogSemiMinorAxisGeoKey, 6356583.8);
    });
    CHECK(tm.semiMajorAxis == 6378206.4);
    CHECK_THAT(tm.inverseFlattening, WithinAbs(294.9786982138982, 1e-9));
  }
  SECTION("equal axes are a sphere") {
    const TransverseMercatorParams tm = inspectWith([](GTIF* g) {
      setDoubleKey(g, GeogSemiMajorAxisGeoKey, 6371000.0);
      setDoubleKey(g, GeogSemiMinorAxisGeoKey, 6371000.0);
    });
    CHECK(tm.semiMajorAxis == 6371000.0);
    CHECK(tm.inverseFlattening == std::numeric_limits<double>::infinity());
  }
}

TEST_CASE("a UTM file that gives only its EPSG code takes the zone's parameters", "[geotiff]") {
  TempDir dir;
  const GridGeometry g = inspectGeoTiff(writeEpsgFixture(dir.path, 32734));
  CHECK(g.kind == ModelKind::TransverseMercator);
  CHECK(g.tm.falseNorthing == 10000000.0);
  CHECK(g.tm.originLongitudeDeg == 21.0);
  CHECK(g.tm.scale == 0.9996);
  CHECK(g.tm.falseEasting == 500000.0);
}

TEST_CASE("a rotated or sheared file is refused with a pointer to gdalwarp", "[geotiff]") {
  TempDir dir;
  CHECK_THROWS_WITH(inspectGeoTiff(writeRotatedFixture(dir.path)),
                    ContainsSubstring("rotated.tif is rotated or sheared") && ContainsSubstring("gdalwarp"));
  // Either off-diagonal term alone is a shear.
  for (const std::size_t term : {std::size_t{1}, std::size_t{4}}) {
    CAPTURE(term);
    std::array<double, 16> m = {5.0, 0.0, 0.0, 1000.0, 0.0, -5.0, 0.0, 2000.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
    m[term] = 0.5;
    const Path file = dir.path / ("shear-" + std::to_string(term) + ".tif");
    writeFixtureFile(file, {4, 3, kHeights}, [&](TIFF* t) { setTransformation(t, m); }, cityKeys);
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("rotated or sheared"));
  }
}

TEST_CASE("a file that is not north-up, or whose placement is not finite, is refused", "[geotiff]") {
  // Read as it stands, a file whose rows run north or whose columns run west would make a mirror image of the place.
  TempDir dir;
  const Path file = dir.path / "flipped.tif";
  const auto refuse = [&](const std::function<void(TIFF*)>& tags, const char* why) {
    writeFixtureFile(file, {4, 3, kHeights}, tags, cityKeys);
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring(why));
  };
  SECTION("a transformation whose rows run north") {
    refuse([](TIFF* t) { setTransformation(t, {5, 0, 0, 1000, 0, 5, 0, 2000, 0, 0, 0, 0, 0, 0, 0, 1}); }, "north-up");
  }
  SECTION("a transformation whose columns run west") {
    refuse([](TIFF* t) { setTransformation(t, {-5, 0, 0, 1000, 0, -5, 0, 2000, 0, 0, 0, 0, 0, 0, 0, 1}); },
           "north-up");
  }
  SECTION("a negative pixel scale") {
    refuse(
        [](TIFF* t) {
          setTiepoint(t, {0.0, 0.0}, {1000.0, 2000.0});
          setPixelScale(t, 5.0, -5.0);
        },
        "north-up");
  }
  SECTION("a zero pixel scale") {
    refuse(
        [](TIFF* t) {
          setTiepoint(t, {0.0, 0.0}, {1000.0, 2000.0});
          setPixelScale(t, 0.0, 5.0);
        },
        "north-up");
  }
  SECTION("a tiepoint that is not a number") {
    refuse(
        [](TIFF* t) {
          setTiepoint(t, {0.0, 0.0}, {kNaN, 2000.0});
          setPixelScale(t, 5.0, 5.0);
        },
        "not finite");
  }
}

TEST_CASE("a south-oriented file is refused, whether its EPSG code or its parameters say so", "[geotiff]") {
  TempDir dir;
  CHECK_THROWS_WITH(inspectGeoTiff(writeSouthOrientedFixture(dir.path)), ContainsSubstring("south-oriented"));
  // EPSG's Lo15 to Lo33 are 2046 to 2055; the codes either side are something else.
  CHECK_THROWS_WITH(inspectGeoTiff(writeEpsgFixture(dir.path, 2046)), ContainsSubstring("south-oriented"));
  CHECK_THROWS_WITH(inspectGeoTiff(writeEpsgFixture(dir.path, 2055)), ContainsSubstring("south-oriented"));
  CHECK_THROWS_WITH(inspectGeoTiff(writeEpsgFixture(dir.path, 2045)), ContainsSubstring("names EPSG:2045"));
  CHECK_THROWS_WITH(inspectGeoTiff(writeEpsgFixture(dir.path, 2056)), ContainsSubstring("names EPSG:2056"));

  const Path file = dir.path / "south.tif";
  SECTION("the south-oriented method, with the parameters of a Lo system") {
    writeFixtureFile(file, {4, 3, kHeights}, cityTags, [](GTIF* g) {
      setTmKeys(g, lo19(), true);
      setShortKey(g, ProjCoordTransGeoKey, CT_TransvMercator_SouthOriented);
    });
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("south-oriented"));
  }
  SECTION("a Lo code beside ordinary Transverse Mercator parameters") {
    // The code's axes run west and south whatever the parameters say; read as east and north, the model would turn
    // through 180 degrees.
    writeFixtureFile(file, {4, 3, kHeights}, cityTags, [](GTIF* g) {
      setTmKeys(g, lo19(), true);
      setShortKey(g, ProjectedCSTypeGeoKey, 2048);
    });
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("south-oriented"));
  }
}

TEST_CASE("a projection other than Transverse Mercator is refused with its GeoKey code", "[geotiff]") {
  TempDir dir;
  const Path file = dir.path / "lambert.tif";
  writeFixtureFile(file, {4, 3, kHeights}, cityTags, [](GTIF* g) {
    setTmKeys(g, lo19(), true);
    setShortKey(g, ProjCoordTransGeoKey, CT_LambertConfConic_2SP);
  });
  CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("lambert.tif uses a projection this tool does not read "
                                                            "(GeoKey ProjCoordTrans 8)") &&
                                              ContainsSubstring("gdalwarp"));
}

TEST_CASE("a projected file without parameters is refused unless its EPSG code is a UTM zone", "[geotiff]") {
  TempDir dir;
  CHECK_THROWS_WITH(inspectGeoTiff(writeEpsgFixture(dir.path, 2193)),
                    ContainsSubstring("names EPSG:2193 without parameters"));
  const Path file = dir.path / "bare.tif";
  SECTION("a user-defined code") {
    writeFixtureFile(file, {4, 3, kHeights}, cityTags, [](GTIF* g) {
      setShortKey(g, GTModelTypeGeoKey, ModelTypeProjected);
      setShortKey(g, ProjectedCSTypeGeoKey, KvUserDefined);
    });
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("neither Transverse Mercator parameters nor"));
  }
  SECTION("no code at all") {
    writeFixtureFile(file, {4, 3, kHeights}, cityTags,
                     [](GTIF* g) { setShortKey(g, GTModelTypeGeoKey, ModelTypeProjected); });
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("neither Transverse Mercator parameters nor"));
  }
}

TEST_CASE("a file that is neither geographic nor projected is refused", "[geotiff]") {
  TempDir dir;
  const Path file = dir.path / "model.tif";
  SECTION("geocentric") {
    writeFixtureFile(file, {4, 3, kHeights}, cityTags,
                     [](GTIF* g) { setShortKey(g, GTModelTypeGeoKey, ModelTypeGeocentric); });
    CHECK_THROWS_WITH(inspectGeoTiff(file),
                      ContainsSubstring("neither geographic nor projected (GeoKey GTModelType 3)"));
  }
  SECTION("no model type") {
    writeFixtureFile(file, {4, 3, kHeights}, cityTags, [](GTIF* g) { setShortKey(g, GeographicTypeGeoKey, 4326); });
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("no GeoKey GTModelType"));
  }
}

TEST_CASE("lengths other than metres and angles other than degrees are refused", "[geotiff]") {
  // The mesh is built from the file's own numbers as metres: a file in feet would come out three times too wide, or
  // its heights three times too tall.
  struct Case {
    geokey_t key;
    int unit;
    const char* name;
  };
  const Case cases[] = {{ProjLinearUnitsGeoKey, Linear_Foot, "ProjLinearUnits"},
                        {VerticalUnitsGeoKey, Linear_Foot_US_Survey, "VerticalUnits"},
                        {GeogAngularUnitsGeoKey, Angular_Radian, "GeogAngularUnits"}};
  for (const Case& c : cases) {
    CAPTURE(c.name);
    TempDir dir;
    const Path file = dir.path / "units.tif";
    writeFixtureFile(file, {4, 3, kHeights}, cityTags, [&](GTIF* g) {
      setTmKeys(g, lo19(), true);
      setShortKey(g, c.key, c.unit);
    });
    CHECK_THROWS_WITH(inspectGeoTiff(file),
                      ContainsSubstring(std::string("unit ") + std::to_string(c.unit) + " (GeoKey " + c.name + ")"));
  }
  // EPSG gives WGS 84's angles in 9122, "degree (supplier to define representation)", which is degrees.
  TempDir dir;
  const Path file = dir.path / "units.tif";
  writeFixtureFile(file, {4, 3, kHeights}, cityTags, [](GTIF* g) {
    setTmKeys(g, lo19(), true);
    setShortKey(g, GeogAngularUnitsGeoKey, 9122);
  });
  CHECK_NOTHROW(inspectGeoTiff(file));
}

TEST_CASE("Transverse Mercator parameters that cannot be used are refused", "[geotiff]") {
  TempDir dir;
  const Path file = dir.path / "params.tif";
  const auto inspectWith = [&](const std::function<void(GTIF*)>& keys) {
    writeFixtureFile(file, {4, 3, kHeights}, cityTags, keys);
    return inspectGeoTiff(file);
  };
  TransverseMercatorParams p = lo19();
  SECTION("no central meridian, which would otherwise be taken as 0, 19 degrees from the City's") {
    CHECK_THROWS_WITH(inspectWith([](GTIF* g) {
                        cityKeys(g);
                        removeKey(g, ProjNatOriginLongGeoKey);
                      }),
                      ContainsSubstring("central meridian"));
  }
  SECTION("a central meridian that is not a number") {
    p.originLongitudeDeg = kNaN;
    CHECK_THROWS_WITH(inspectWith([&](GTIF* g) { setTmKeys(g, p, true); }),
                      ContainsSubstring("Transverse Mercator parameters"));
  }
  SECTION("a scale of zero") {
    p.scale = 0.0;
    CHECK_THROWS_WITH(inspectWith([&](GTIF* g) { setTmKeys(g, p, true); }),
                      ContainsSubstring("Transverse Mercator parameters"));
  }
  SECTION("an ellipsoid with no size") {
    p.semiMajorAxis = 0.0;
    CHECK_THROWS_WITH(inspectWith([&](GTIF* g) { setTmKeys(g, p, true); }), ContainsSubstring("ellipsoid"));
  }
  SECTION("a flattening above one") {
    p.inverseFlattening = 0.5;
    CHECK_THROWS_WITH(inspectWith([&](GTIF* g) { setTmKeys(g, p, true); }), ContainsSubstring("ellipsoid"));
  }
  SECTION("a GeoKey stored as another type") {
    // libgeotiff copies a key's own size, so a short read as a double would be garbage and a double read as a short
    // would overrun it.
    CHECK_THROWS_WITH(inspectWith([](GTIF* g) {
                        cityKeys(g);
                        setShortKey(g, ProjNatOriginLongGeoKey, 19);
                      }),
                      ContainsSubstring("GeoKey ProjNatOriginLong stored as Short rather than Double"));
  }
}

TEST_CASE("a malformed GeoKey directory is refused with libgeotiff's reason", "[geotiff]") {
  // One key, ProjNatOriginLong, said to be the sixth GeoDoubleParams value when there is only one. libgeotiff would
  // print its complaint to stderr; it belongs in the message.
  TempDir dir;
  const Path file = dir.path / "keys.tif";
  TIFF* tif = XTIFFOpen(file.string().c_str(), "w");
  REQUIRE(tif != nullptr);
  TIFFSetField(tif, TIFFTAG_IMAGEWIDTH, 4u);
  TIFFSetField(tif, TIFFTAG_IMAGELENGTH, 2u);
  TIFFSetField(tif, TIFFTAG_SAMPLESPERPIXEL, 1);
  TIFFSetField(tif, TIFFTAG_BITSPERSAMPLE, 32);
  TIFFSetField(tif, TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
  TIFFSetField(tif, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  TIFFSetField(tif, TIFFTAG_ROWSPERSTRIP, 2u);
  const double tiepoint[6] = {0.0, 0.0, 0.0, -64012.5, -3705051.806, 0.0};
  const double scale[3] = {5.0, 5.0, 0.0};
  const uint16_t directory[8] = {1, 1, 0, 1, ProjNatOriginLongGeoKey, TIFFTAG_GEODOUBLEPARAMS, 1, 5};
  const double doubles[1] = {19.0};
  REQUIRE(TIFFSetField(tif, TIFFTAG_GEOTIEPOINTS, 6, tiepoint) == 1);
  REQUIRE(TIFFSetField(tif, TIFFTAG_GEOPIXELSCALE, 3, scale) == 1);
  REQUIRE(TIFFSetField(tif, TIFFTAG_GEOKEYDIRECTORY, 8, directory) == 1);
  REQUIRE(TIFFSetField(tif, TIFFTAG_GEODOUBLEPARAMS, 1, doubles) == 1);
  std::vector<float> row(4, 1.0f);
  for (uint32_t y = 0; y < 2; ++y) REQUIRE(TIFFWriteScanline(tif, row.data(), y, 0) == 1);
  XTIFFClose(tif);
  CHECK_THROWS_WITH(inspectGeoTiff(file),
                    ContainsSubstring("cannot read the GeoTIFF keys of") && ContainsSubstring("ProjNatOriginLong"));
}

TEST_CASE("a GDAL_NODATA tag that is not a number is refused", "[geotiff]") {
  TempDir dir;
  const Path file = dir.path / "nodata.tif";
  writeFixtureFile(
      file, {4, 3, kHeights},
      [](TIFF* t) {
        setTiepoint(t, {0.0, 0.0}, {-64012.5, -3705051.806});
        setPixelScale(t, 5.0, 5.0);
        setNoDataText(t, "none");
      },
      cityKeys);
  CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("GDAL_NODATA") && ContainsSubstring("'none'"));
}

TEST_CASE("a file that cannot be opened is reported by name", "[geotiff]") {
  TempDir dir;
  const Path absent = dir.path / "absent.tif";
  CHECK_THROWS_WITH(inspectGeoTiff(absent), ContainsSubstring("absent.tif"));
  GridGeometry g;
  g.width = 4;
  g.height = 3;
  CHECK_THROWS_WITH(readWindow(absent, g, kWhole), ContainsSubstring("absent.tif"));
}

TEST_CASE("a file of several bands or of another sample type is refused", "[geotiff]") {
  TempDir dir;
  // RGB16, like the old downloads in the repository's root.
  const Path rgb = dir.path / "rgb16.tif";
  writePlainTiff(rgb, 3, 16, SAMPLEFORMAT_UINT);
  CHECK_THROWS_WITH(inspectGeoTiff(rgb), ContainsSubstring("rgb16.tif") && ContainsSubstring("single-band"));
  struct Format {
    int bits;
    int format;
  };
  for (const Format f : {Format{8, SAMPLEFORMAT_UINT}, Format{8, SAMPLEFORMAT_INT}, Format{32, SAMPLEFORMAT_UINT},
                         Format{16, SAMPLEFORMAT_IEEEFP}}) {
    CAPTURE(f.bits, f.format);
    const Path file = dir.path / "format.tif";
    writePlainTiff(file, 1, f.bits, f.format);
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("only float32, float64, int16, int32 and uint16"));
  }
}

TEST_CASE("a file without a tiepoint or a transformation, or without a pixel scale, is refused", "[geotiff]") {
  TempDir dir;
  const Path file = dir.path / "unplaced.tif";
  SECTION("a pixel scale alone") {
    writeFixtureFile(file, {4, 3, kHeights}, [](TIFF* t) { setPixelScale(t, 5.0, 5.0); }, cityKeys);
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("no ModelTiepoint and no ModelTransformation"));
  }
  SECTION("a tiepoint alone") {
    writeFixtureFile(file, {4, 3, kHeights}, [](TIFF* t) { setTiepoint(t, {0.0, 0.0}, {1000.0, 2000.0}); },
                     cityKeys);
    CHECK_THROWS_WITH(inspectGeoTiff(file), ContainsSubstring("no ModelPixelScale"));
  }
}

TEST_CASE("readWindow refuses a window that is not inside the file", "[geotiff]") {
  TempDir dir;
  const Path file = writeCityFixture(dir.path);
  const GridGeometry g = inspectGeoTiff(file);
  const PixelWindow outside[] = {{-1, 0, 2, 2}, {0, -1, 2, 2}, {0, 0, 0, 1},       {0, 0, 1, 0},
                                 {3, 0, 2, 1},  {0, 2, 1, 2},  {INT_MAX, 0, 1, 1}, {0, 0, INT_MAX, 1}};
  for (const PixelWindow& w : outside) {
    CAPTURE(w.col0, w.row0, w.cols, w.rows);
    CHECK_THROWS_AS(readWindow(file, g, w), std::invalid_argument);
  }
}

TEST_CASE("readWindow reports a file that no longer matches its geometry or cannot be decoded", "[geotiff]") {
  TempDir dir;
  const Path file = dir.path / "lzw.tif";
  writeFixtureFile(file, {4, 3, kHeights, Layout::Strips, SampleKind::Float32, COMPRESSION_LZW}, cityTags, cityKeys);
  const GridGeometry g = inspectGeoTiff(file);
  SECTION("a geometry of another size") {
    GridGeometry wider = g;
    wider.width = 5;
    CHECK_THROWS_WITH(readWindow(file, wider, {0, 0, 5, 3}), ContainsSubstring("lzw.tif has changed"));
  }
  SECTION("a strip that is not LZW any more") {
    std::uint64_t offset = 0;
    {
      TIFF* t = XTIFFOpen(file.string().c_str(), "r");
      REQUIRE(t != nullptr);
      std::uint64_t* offsets = nullptr;
      REQUIRE(TIFFGetField(t, TIFFTAG_STRIPOFFSETS, &offsets) == 1);
      offset = offsets[0];
      XTIFFClose(t);
    }
    {
      std::fstream f(file, std::ios::in | std::ios::out | std::ios::binary);
      f.seekp(static_cast<std::streamoff>(offset));
      const char garbage[4] = {'\xff', '\xff', '\xff', '\xff'};
      f.write(garbage, sizeof garbage);
      REQUIRE(f.good());
    }
    CHECK_THROWS_WITH(readWindow(file, g, kWhole),
                      ContainsSubstring("cannot read row 0 of") && ContainsSubstring("lzw.tif"));
  }
}
