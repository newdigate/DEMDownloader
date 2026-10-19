#pragma once

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <functional>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "demdl/BoundingBox.h"
#include "demdl/DemRaster.h"
#include "demdl/TiffSetup.h"
#include "mesh/TransverseMercator.h"

// Small elevation GeoTIFFs for the reader's tests, written with libtiff and libgeotiff the way other programs write
// them, except the geographic one, which goes through DemRaster as dl_elevation's files do. Every writer throws
// std::runtime_error when libtiff or libgeotiff refuses something, so a broken fixture fails the test that wrote it
// instead of handing the reader a file the test did not mean.

// Strips hold two rows each, so that a window can start inside one; tiles are 16 x 16, the smallest TIFF allows.
enum class Layout { Strips, Tiles };
enum class SampleKind { Float32, Float64, Int16, Int32, UInt16 };

struct RasterPoint {  // a raster position: column i, row j
  double i = 0.0;
  double j = 0.0;
};

struct ModelPoint {  // a model position: easting or longitude x, northing or latitude y
  double x = 0.0;
  double y = 0.0;
};

// A fixture's pixels, row by row from the north-west corner, and how the file stores them.
struct FixturePixels {
  int width = 0;
  int height = 0;
  std::vector<double> heights;
  Layout layout = Layout::Strips;
  SampleKind kind = SampleKind::Float32;
  std::uint16_t compression = COMPRESSION_NONE;
};

namespace fixture_detail {

inline void check(bool ok, const std::string& what) {
  if (ok) return;
  const std::string libtiff = demdl::takeTiffError();
  throw std::runtime_error("fixture: " + what + (libtiff.empty() ? "" : ": " + libtiff));
}

inline int bitsPerSample(SampleKind kind) {
  switch (kind) {
    case SampleKind::Float64:
      return 64;
    case SampleKind::Float32:
    case SampleKind::Int32:
      return 32;
    case SampleKind::Int16:
    case SampleKind::UInt16:
      return 16;
  }
  return 0;
}

inline int sampleFormat(SampleKind kind) {
  switch (kind) {
    case SampleKind::Float32:
    case SampleKind::Float64:
      return SAMPLEFORMAT_IEEEFP;
    case SampleKind::Int16:
    case SampleKind::Int32:
      return SAMPLEFORMAT_INT;
    case SampleKind::UInt16:
      return SAMPLEFORMAT_UINT;
  }
  return 0;
}

// Stores one sample in this machine's byte order, which libtiff writes as the file's. Integer kinds take whole numbers
// in their range only, so a NaN cannot reach a cast.
inline void storeSample(SampleKind kind, double value, unsigned char* out) {
  const auto store = [&](auto sample) { std::memcpy(out, &sample, sizeof sample); };
  const auto whole = [&](auto type) {
    using T = decltype(type);
    check(value == std::trunc(value) && value >= static_cast<double>(std::numeric_limits<T>::min()) &&
              value <= static_cast<double>(std::numeric_limits<T>::max()),
          "height " + std::to_string(value) + " does not fit the sample type");
    store(static_cast<T>(value));
  };
  switch (kind) {
    case SampleKind::Float32:
      store(static_cast<float>(value));
      return;
    case SampleKind::Float64:
      store(value);
      return;
    case SampleKind::Int16:
      whole(std::int16_t{});
      return;
    case SampleKind::Int32:
      whole(std::int32_t{});
      return;
    case SampleKind::UInt16:
      whole(std::uint16_t{});
      return;
  }
}

struct TiffCloser {
  void operator()(TIFF* t) const { XTIFFClose(t); }
};

struct GtifFreer {
  void operator()(GTIF* g) const { GTIFFree(g); }
};

}  // namespace fixture_detail

inline void setShortKey(GTIF* g, geokey_t key, int value) {
  fixture_detail::check(GTIFKeySet(g, key, TYPE_SHORT, 1, value) == 1, "cannot set a short GeoKey");
}

inline void setDoubleKey(GTIF* g, geokey_t key, double value) {
  fixture_detail::check(GTIFKeySet(g, key, TYPE_DOUBLE, 1, value) == 1, "cannot set a double GeoKey");
}

// libgeotiff deletes a key, whatever its type, when the count is -1.
inline void removeKey(GTIF* g, geokey_t key) {
  fixture_detail::check(GTIFKeySet(g, key, TYPE_SHORT, -1) == 1, "cannot remove a GeoKey");
}

inline void setTiepoint(TIFF* t, RasterPoint raster, ModelPoint model) {
  const double tiepoint[6] = {raster.i, raster.j, 0.0, model.x, model.y, 0.0};
  fixture_detail::check(TIFFSetField(t, TIFFTAG_GEOTIEPOINTS, 6, tiepoint) == 1, "cannot set the tiepoint");
}

inline void setPixelScale(TIFF* t, double dx, double dy) {
  const double scale[3] = {dx, dy, 0.0};
  fixture_detail::check(TIFFSetField(t, TIFFTAG_GEOPIXELSCALE, 3, scale) == 1, "cannot set the pixel scale");
}

// ModelTransformation, row-major 4 x 4.
inline void setTransformation(TIFF* t, const std::array<double, 16>& m) {
  fixture_detail::check(TIFFSetField(t, TIFFTAG_GEOTRANSMATRIX, 16, m.data()) == 1, "cannot set the transformation");
}

inline void setNoDataText(TIFF* t, const char* text) {
  fixture_detail::check(TIFFSetField(t, TIFFTAG_GDAL_NODATA, text) == 1, "cannot set GDAL_NODATA");
}

// As GDAL writes it: enough digits to give the same double back.
inline void setNoData(TIFF* t, double value) {
  char text[32];
  std::snprintf(text, sizeof text, "%.17g", value);
  setNoDataText(t, text);
}

// A geographic system in degrees, WGS 84 by code, with no raster type: absent means area.
inline void setGeographicKeys(GTIF* g) {
  setShortKey(g, GTModelTypeGeoKey, ModelTypeGeographic);
  setShortKey(g, GeographicTypeGeoKey, GCS_WGS_84);
  setShortKey(g, GeogAngularUnitsGeoKey, Angular_Degree);
}

// The City of Cape Town's GeoKeys apart from its ellipsoid, with the projection's numbers from params: projected, its
// own code 20482 (which names no EPSG system) beside full Transverse Mercator parameters, metres and degrees.
inline void setTmProjectionKeys(GTIF* g, const demdl::mesh::TransverseMercatorParams& params, bool pixelIsPoint) {
  setShortKey(g, GTModelTypeGeoKey, ModelTypeProjected);
  setShortKey(g, GTRasterTypeGeoKey, pixelIsPoint ? RasterPixelIsPoint : RasterPixelIsArea);
  setShortKey(g, GeographicTypeGeoKey, 4148);  // Hartebeesthoek94
  setShortKey(g, GeogAngularUnitsGeoKey, Angular_Degree);
  setShortKey(g, ProjectedCSTypeGeoKey, 20482);
  setShortKey(g, ProjectionGeoKey, KvUserDefined);
  setShortKey(g, ProjCoordTransGeoKey, CT_TransverseMercator);
  setShortKey(g, ProjLinearUnitsGeoKey, Linear_Meter);
  setDoubleKey(g, ProjNatOriginLongGeoKey, params.originLongitudeDeg);
  setDoubleKey(g, ProjNatOriginLatGeoKey, params.originLatitudeDeg);
  setDoubleKey(g, ProjFalseEastingGeoKey, params.falseEasting);
  setDoubleKey(g, ProjFalseNorthingGeoKey, params.falseNorthing);
  setDoubleKey(g, ProjScaleAtNatOriginGeoKey, params.scale);
  setShortKey(g, VerticalUnitsGeoKey, Linear_Meter);
}

// All of the City's GeoKeys: the projection, and its ellipsoid both named by code (GRS 1980) and given by its axes,
// here params' axes, which may differ from the code's.
inline void setTmKeys(GTIF* g, const demdl::mesh::TransverseMercatorParams& params, bool pixelIsPoint) {
  setTmProjectionKeys(g, params, pixelIsPoint);
  setShortKey(g, GeogEllipsoidGeoKey, Ellipse_GRS_1980);
  setDoubleKey(g, GeogSemiMajorAxisGeoKey, params.semiMajorAxis);
  setDoubleKey(g, GeogInvFlatteningGeoKey, params.inverseFlattening);
}

// Writes pixels as a single-band GeoTIFF; `tags` sets the georeferencing tags on the open file and `keys` the GeoKeys.
inline void writeFixtureFile(const std::filesystem::path& file, const FixturePixels& pixels,
                             const std::function<void(TIFF*)>& tags, const std::function<void(GTIF*)>& keys) {
  using fixture_detail::check;
  const auto samples = static_cast<std::size_t>(pixels.width) * static_cast<std::size_t>(pixels.height);
  check(pixels.width > 0 && pixels.height > 0 && pixels.heights.size() == samples,
        "the heights do not fill " + std::to_string(pixels.width) + " x " + std::to_string(pixels.height));
  // The GDAL_NODATA tag is known only to handles opened after this.
  demdl::ensureTagsRegistered();
  (void)demdl::takeTiffError();
  const std::unique_ptr<TIFF, fixture_detail::TiffCloser> tif(XTIFFOpen(file.string().c_str(), "w"));
  check(tif != nullptr, "cannot create " + file.string());
  TIFF* t = tif.get();

  const auto set = [&](const char* name, uint32_t tag, auto value) {
    check(TIFFSetField(t, tag, value) == 1, std::string("cannot set ") + name);
  };
  set("IMAGEWIDTH", TIFFTAG_IMAGEWIDTH, static_cast<uint32_t>(pixels.width));
  set("IMAGELENGTH", TIFFTAG_IMAGELENGTH, static_cast<uint32_t>(pixels.height));
  set("SAMPLESPERPIXEL", TIFFTAG_SAMPLESPERPIXEL, 1);
  set("BITSPERSAMPLE", TIFFTAG_BITSPERSAMPLE, fixture_detail::bitsPerSample(pixels.kind));
  set("SAMPLEFORMAT", TIFFTAG_SAMPLEFORMAT, fixture_detail::sampleFormat(pixels.kind));
  set("PHOTOMETRIC", TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
  set("PLANARCONFIG", TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
  set("COMPRESSION", TIFFTAG_COMPRESSION, static_cast<int>(pixels.compression));
  constexpr uint32_t kRowsPerStrip = 2;
  constexpr uint32_t kTile = 16;
  if (pixels.layout == Layout::Strips) {
    set("ROWSPERSTRIP", TIFFTAG_ROWSPERSTRIP, kRowsPerStrip);
  } else {
    set("TILEWIDTH", TIFFTAG_TILEWIDTH, kTile);
    set("TILELENGTH", TIFFTAG_TILELENGTH, kTile);
  }
  tags(t);
  {
    const std::unique_ptr<GTIF, fixture_detail::GtifFreer> gtif(GTIFNew(t));
    check(gtif != nullptr, "cannot attach GeoKeys");
    keys(gtif.get());
    check(GTIFWriteKeys(gtif.get()) == 1, "cannot write the GeoKeys");
  }

  const auto bytes = static_cast<std::size_t>(fixture_detail::bitsPerSample(pixels.kind) / 8);
  const auto at = [&](int col, int row) {
    return pixels.heights[static_cast<std::size_t>(row) * static_cast<std::size_t>(pixels.width) +
                          static_cast<std::size_t>(col)];
  };
  if (pixels.layout == Layout::Strips) {
    std::vector<unsigned char> line(static_cast<std::size_t>(pixels.width) * bytes);
    for (int row = 0; row < pixels.height; ++row) {
      for (int col = 0; col < pixels.width; ++col) {
        fixture_detail::storeSample(pixels.kind, at(col, row), &line[static_cast<std::size_t>(col) * bytes]);
      }
      check(TIFFWriteScanline(t, line.data(), static_cast<uint32_t>(row), 0) == 1,
            "cannot write row " + std::to_string(row));
    }
  } else {
    constexpr int kSide = static_cast<int>(kTile);
    std::vector<unsigned char> tile(static_cast<std::size_t>(kSide * kSide) * bytes);
    for (int top = 0; top < pixels.height; top += kSide) {
      for (int left = 0; left < pixels.width; left += kSide) {
        // Padding beyond the image's edge holds 7777, a height no test writes, so a reader that took it would show.
        for (std::size_t i = 0; i < tile.size(); i += bytes) fixture_detail::storeSample(pixels.kind, 7777.0, &tile[i]);
        for (int row = top; row < std::min(top + kSide, pixels.height); ++row) {
          for (int col = left; col < std::min(left + kSide, pixels.width); ++col) {
            const auto offset = static_cast<std::size_t>((row - top) * kSide + (col - left)) * bytes;
            fixture_detail::storeSample(pixels.kind, at(col, row), &tile[offset]);
          }
        }
        check(TIFFWriteTile(t, tile.data(), static_cast<uint32_t>(left), static_cast<uint32_t>(top), 0, 0) >= 0,
              "cannot write the tile at " + std::to_string(left) + ", " + std::to_string(top));
      }
    }
  }
  check(TIFFFlush(t) == 1, "cannot finish " + file.string());
}

// A file as dl_elevation writes it, through DemRaster: WGS 84 degrees, RasterPixelIsArea, the tiepoint on the
// north-west corner of box, GDAL_NODATA -32768 and an ImageDescription starting "dl_elevation;". heights run row by
// row from the north-west corner; NaN is a pixel never fetched and -32768 the server's no-data.
[[nodiscard]] inline std::filesystem::path writeGeographicFixture(const std::filesystem::path& dir, int width,
                                                                  int height, const demdl::BoundingBox& box,
                                                                  const std::vector<double>& heights) {
  fixture_detail::check(heights.size() == static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                        "the heights do not fill the raster");
  auto raster =
      demdl::DemRaster::create(width, height, box, {"Elevation_CCT_GLM2019_5m_DEM", "https://example.test/wms"});
  for (int row = 0; row < height; ++row) {
    for (int col = 0; col < width; ++col) {
      raster.set(col, row, static_cast<float>(heights[static_cast<std::size_t>(row * width + col)]));
    }
  }
  const auto file = dir / "geographic.tif";
  raster.save(file);
  return file;
}

// A projected file with the City of Cape Town's GeoKeys (setTmKeys) and params' projection: the tiepoint ties raster
// position tiepointRaster to model position tiepointModel, pixels are dx east by dy south, and GDAL_NODATA is noData
// when given.
[[nodiscard]] inline std::filesystem::path writeTmFixture(const std::filesystem::path& dir, int width, int height,
                                                          const demdl::mesh::TransverseMercatorParams& params,
                                                          RasterPoint tiepointRaster, ModelPoint tiepointModel,
                                                          double dx, double dy, bool pixelIsPoint,
                                                          std::optional<double> noData,
                                                          const std::vector<double>& heights, Layout layout,
                                                          SampleKind kind) {
  const auto file = dir / "tm.tif";
  writeFixtureFile(
      file, {width, height, heights, layout, kind},
      [&](TIFF* t) {
        setTiepoint(t, tiepointRaster, tiepointModel);
        setPixelScale(t, dx, dy);
        if (noData) setNoData(t, *noData);
      },
      [&](GTIF* g) { setTmKeys(g, params, pixelIsPoint); });
  return file;
}

// A projected file that names an EPSG code and nothing else about its projection, as a writer that leaves the code to
// the reader writes it.
[[nodiscard]] inline std::filesystem::path writeEpsgFixture(const std::filesystem::path& dir, int code) {
  const auto file = dir / ("epsg-" + std::to_string(code) + ".tif");
  writeFixtureFile(
      file, {2, 2, {1.0, 2.0, 3.0, 4.0}},
      [](TIFF* t) {
        setTiepoint(t, {0.0, 0.0}, {261880.0, 6243180.0});
        setPixelScale(t, 5.0, 5.0);
      },
      [code](GTIF* g) {
        setShortKey(g, GTModelTypeGeoKey, ModelTypeProjected);
        setShortKey(g, GTRasterTypeGeoKey, RasterPixelIsArea);
        setShortKey(g, ProjectedCSTypeGeoKey, code);
        setShortKey(g, ProjLinearUnitsGeoKey, Linear_Meter);
      });
  return file;
}

// The City's GeoKeys under a ModelTransformation that turns the raster 30 degrees: both off-diagonal terms are not 0.
[[nodiscard]] inline std::filesystem::path writeRotatedFixture(const std::filesystem::path& dir) {
  const auto file = dir / "rotated.tif";
  const double c = 5.0 * std::cos(std::numbers::pi / 6.0);
  const double s = 5.0 * std::sin(std::numbers::pi / 6.0);
  const std::array<double, 16> m = {c, s, 0.0, -64012.5, s, -c, 0.0, -3705051.806,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
  demdl::mesh::TransverseMercatorParams lo19;
  lo19.originLongitudeDeg = 19.0;
  writeFixtureFile(
      file, {2, 2, {1.0, 2.0, 3.0, 4.0}}, [&](TIFF* t) { setTransformation(t, m); },
      [&](GTIF* g) { setTmKeys(g, lo19, true); });
  return file;
}

// EPSG:2048, Hartebeesthoek94 / Lo19, whose axes run west and south, named with no parameters.
[[nodiscard]] inline std::filesystem::path writeSouthOrientedFixture(const std::filesystem::path& dir) {
  return writeEpsgFixture(dir, 2048);
}
