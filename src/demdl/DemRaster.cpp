#include "demdl/DemRaster.h"

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <cmath>
#include <cstring>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <system_error>
#include <utility>

#include "demdl/WmsQuery.h"

namespace demdl {

namespace {

struct TiffCloser {
  void operator()(TIFF* t) const {
    if (t != nullptr) XTIFFClose(t);
  }
};
using TiffHandle = std::unique_ptr<TIFF, TiffCloser>;

struct GtifCloser {
  void operator()(GTIF* g) const {
    if (g != nullptr) GTIFFree(g);
  }
};
using GtifHandle = std::unique_ptr<GTIF, GtifCloser>;

// libtiff does not know the GDAL_NODATA tag: TIFFSetField refuses it and a reader sees an anonymous
// field. This tag extender registers it on every TIFF handle opened after ensureTagsRegistered().
// It chains to the extender installed before it, because libgeotiff installs one for the GeoTIFF
// tags on its first XTIFFOpen and that one chains in turn, so both run in either install order.
TIFFExtendProc g_parentExtender = nullptr;

void registerGdalTags(TIFF* tif) {
  static const TIFFFieldInfo fields[] = {
      {TIFFTAG_GDAL_NODATA, -1, -1, TIFF_ASCII, FIELD_CUSTOM, 1, 0, const_cast<char*>("GDALNoDataValue")},
  };
  TIFFMergeFieldInfo(tif, fields, 1);
  if (g_parentExtender != nullptr) g_parentExtender(tif);
}

void ensureTagsRegistered() {
  static std::once_flag once;
  std::call_once(once, [] { g_parentExtender = TIFFSetTagExtender(registerGdalTags); });
}

}  // namespace

std::string formatImageDescription(const RasterMetadata& meta) {
  return "dl_elevation;layer=" + meta.layer + ";wms=" + meta.wmsUrl;
}

RasterMetadata parseImageDescription(std::string_view description) {
  RasterMetadata meta;
  std::string_view head = description;
  const auto wmsPos = description.find(";wms=");
  if (wmsPos != std::string_view::npos) {
    meta.wmsUrl = std::string(description.substr(wmsPos + 5));
    head = description.substr(0, wmsPos);
  }
  const auto layerPos = head.find("layer=");
  if (layerPos != std::string_view::npos) {
    const std::string_view value = head.substr(layerPos + 6);
    meta.layer = std::string(value.substr(0, value.find(';')));
  }
  return meta;
}

DemRaster DemRaster::create(int width, int height, const BoundingBox& box, RasterMetadata meta) {
  if (width <= 0 || height <= 0) throw std::invalid_argument("raster dimensions must be positive");
  DemRaster r;
  r.width_ = width;
  r.height_ = height;
  r.box_ = box;
  r.meta_ = std::move(meta);
  r.data_.assign(static_cast<std::size_t>(width) * static_cast<std::size_t>(height),
                 std::numeric_limits<float>::quiet_NaN());
  return r;
}

DemRaster DemRaster::load(const std::filesystem::path& path) {
  ensureTagsRegistered();
  TiffHandle tif(XTIFFOpen(path.string().c_str(), "r"));
  if (!tif) throw std::runtime_error("cannot open " + path.string());
  TIFF* t = tif.get();

  uint32_t width = 0;
  uint32_t height = 0;
  uint16_t samplesPerPixel = 0;
  uint16_t bitsPerSample = 0;
  uint16_t sampleFormat = 0;
  TIFFGetField(t, TIFFTAG_IMAGEWIDTH, &width);
  TIFFGetField(t, TIFFTAG_IMAGELENGTH, &height);
  TIFFGetFieldDefaulted(t, TIFFTAG_SAMPLESPERPIXEL, &samplesPerPixel);
  TIFFGetFieldDefaulted(t, TIFFTAG_BITSPERSAMPLE, &bitsPerSample);
  TIFFGetFieldDefaulted(t, TIFFTAG_SAMPLEFORMAT, &sampleFormat);
  if (samplesPerPixel != 1 || bitsPerSample != 32 || sampleFormat != SAMPLEFORMAT_IEEEFP) {
    throw std::runtime_error(path.string() +
                             " is not a dl_elevation float32 GeoTIFF; files from the previous RGB16 "
                             "format cannot be resumed");
  }
  if (width == 0 || height == 0) throw std::runtime_error(path.string() + " has an empty image");

  uint16_t tieCount = 0;
  double* tiepoints = nullptr;
  if (!TIFFGetField(t, TIFFTAG_GEOTIEPOINTS, &tieCount, &tiepoints) || tieCount < 6 || tiepoints[0] != 0.0 ||
      tiepoints[1] != 0.0) {
    throw std::runtime_error(path.string() + " has no usable GeoTIFF tiepoint");
  }
  uint16_t scaleCount = 0;
  double* scale = nullptr;
  if (!TIFFGetField(t, TIFFTAG_GEOPIXELSCALE, &scaleCount, &scale) || scaleCount < 2) {
    throw std::runtime_error(path.string() + " has no GeoTIFF pixel scale");
  }
  BoundingBox box;
  box.west = tiepoints[3];
  box.north = tiepoints[4];
  box.east = box.west + scale[0] * width;
  box.south = box.north - scale[1] * height;
  if (const std::string problem = box.validate(); !problem.empty()) {
    throw std::runtime_error(path.string() + " has an invalid bounding box: " + problem);
  }

  RasterMetadata meta;
  char* description = nullptr;
  if (TIFFGetField(t, TIFFTAG_IMAGEDESCRIPTION, &description) && description != nullptr) {
    meta = parseImageDescription(description);
  }

  DemRaster r = create(static_cast<int>(width), static_cast<int>(height), box, std::move(meta));
  for (uint32_t y = 0; y < height; ++y) {
    if (TIFFReadScanline(t, &r.data_[r.index(0, static_cast<int>(y))], y, 0) < 0) {
      throw std::runtime_error("read failed on row " + std::to_string(y) + " of " + path.string());
    }
  }
  return r;
}

void DemRaster::save(const std::filesystem::path& path) const {
  ensureTagsRegistered();
  const std::filesystem::path tmp = path.string() + ".tmp";
  const auto fail = [&tmp](const std::string& what) {
    std::error_code ignored;
    std::filesystem::remove(tmp, ignored);
    throw std::runtime_error(what);
  };
  {
    TiffHandle tif(XTIFFOpen(tmp.string().c_str(), "w"));
    if (!tif) fail("cannot create " + tmp.string());
    TIFF* t = tif.get();
    // Checked so that a tag libtiff does not know fails the save instead of being left out.
    const auto setField = [&](const char* name, uint32_t tag, auto... values) {
      if (TIFFSetField(t, tag, values...) != 1) {
        tif.reset();
        fail(std::string("cannot set tag ") + name + " on " + tmp.string());
      }
    };
    setField("IMAGEWIDTH", TIFFTAG_IMAGEWIDTH, static_cast<uint32_t>(width_));
    setField("IMAGELENGTH", TIFFTAG_IMAGELENGTH, static_cast<uint32_t>(height_));
    setField("SAMPLESPERPIXEL", TIFFTAG_SAMPLESPERPIXEL, 1);
    setField("BITSPERSAMPLE", TIFFTAG_BITSPERSAMPLE, 32);
    setField("SAMPLEFORMAT", TIFFTAG_SAMPLEFORMAT, SAMPLEFORMAT_IEEEFP);
    setField("PHOTOMETRIC", TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_MINISBLACK);
    setField("PLANARCONFIG", TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
    setField("COMPRESSION", TIFFTAG_COMPRESSION, COMPRESSION_NONE);
    setField("ORIENTATION", TIFFTAG_ORIENTATION, ORIENTATION_TOPLEFT);
    setField("ROWSPERSTRIP", TIFFTAG_ROWSPERSTRIP, TIFFDefaultStripSize(t, 0));

    const PixelScale scale = box_.pixelScale(width_, height_);
    const double tiepoints[6] = {0.0, 0.0, 0.0, box_.west, box_.north, 0.0};
    const double pixelScale[3] = {scale.lonPerPixel, scale.latPerPixel, 0.0};
    setField("GEOTIEPOINTS", TIFFTAG_GEOTIEPOINTS, 6, tiepoints);
    setField("GEOPIXELSCALE", TIFFTAG_GEOPIXELSCALE, 3, pixelScale);
    setField("GDAL_NODATA", TIFFTAG_GDAL_NODATA, "-32768");
    const std::string description = formatImageDescription(meta_);
    setField("IMAGEDESCRIPTION", TIFFTAG_IMAGEDESCRIPTION, description.c_str());
    const std::string software = std::string("dl_elevation ") + DL_ELEVATION_VERSION;
    setField("SOFTWARE", TIFFTAG_SOFTWARE, software.c_str());

    {
      GtifHandle gtif(GTIFNew(t));
      if (!gtif) {
        tif.reset();
        fail("cannot attach GeoTIFF keys to " + tmp.string());
      }
      const auto setKey = [&](const char* name, geokey_t key, int value) {
        if (GTIFKeySet(gtif.get(), key, TYPE_SHORT, 1, value) != 1) {
          tif.reset();
          fail(std::string("cannot set GeoTIFF key ") + name + " on " + tmp.string());
        }
      };
      setKey("GTModelTypeGeoKey", GTModelTypeGeoKey, ModelTypeGeographic);
      setKey("GTRasterTypeGeoKey", GTRasterTypeGeoKey, RasterPixelIsArea);
      setKey("GeographicTypeGeoKey", GeographicTypeGeoKey, GCS_WGS_84);
      if (GTIFWriteKeys(gtif.get()) != 1) {
        tif.reset();
        fail("cannot write the GeoTIFF keys to " + tmp.string());
      }
    }

    std::vector<float> row(static_cast<std::size_t>(width_));
    for (int y = 0; y < height_; ++y) {
      std::memcpy(row.data(), &data_[index(0, y)], row.size() * sizeof(float));
      if (TIFFWriteScanline(t, row.data(), static_cast<uint32_t>(y), 0) < 0) {
        tif.reset();
        fail("write failed on row " + std::to_string(y) + " of " + tmp.string());
      }
    }
    // XTIFFClose swallows errors from writing the last strip and the directory. Flushing first
    // surfaces them, so a failed write never reaches the rename and the previous checkpoint survives.
    if (TIFFFlush(t) != 1) {
      tif.reset();
      fail("cannot finish writing " + tmp.string());
    }
  }
  std::error_code ec;
  std::filesystem::rename(tmp, path, ec);
  if (ec) fail("cannot rename " + tmp.string() + " to " + path.string() + ": " + ec.message());
}

bool DemRaster::matches(const Config& cfg, std::string& why) const {
  why.clear();
  if (width_ != cfg.width) {
    why += "  width: file " + std::to_string(width_) + ", requested " + std::to_string(cfg.width) + "\n";
  }
  if (height_ != cfg.height) {
    why += "  height: file " + std::to_string(height_) + ", requested " + std::to_string(cfg.height) + "\n";
  }
  const auto edge = [&why](const char* name, double file, double requested) {
    if (std::fabs(file - requested) > kBoxTolerance) {
      why += std::string("  ") + name + ": file " + formatCoordinate(file) + ", requested " +
             formatCoordinate(requested) + "\n";
    }
  };
  edge("north", box_.north, cfg.box.north);
  edge("south", box_.south, cfg.box.south);
  edge("east", box_.east, cfg.box.east);
  edge("west", box_.west, cfg.box.west);
  if (meta_.layer != cfg.layer) {
    why += "  layer: file " + meta_.layer + ", requested " + cfg.layer + "\n";
  }
  return why.empty();
}

std::string DemRaster::wmsUrlWarning(const Config& cfg) const {
  if (meta_.wmsUrl.empty() || meta_.wmsUrl == cfg.wmsUrl) return {};
  return "output was created from " + meta_.wmsUrl + " but this run uses " + cfg.wmsUrl;
}

std::vector<int> DemRaster::todoInRow(int row) const {
  std::vector<int> columns;
  for (int x = 0; x < width_; ++x) {
    if (std::isnan(at(x, row))) columns.push_back(x);
  }
  return columns;
}

std::vector<int> DemRaster::rowsWithWork() const {
  std::vector<int> rows;
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      if (std::isnan(at(x, y))) {
        rows.push_back(y);
        break;
      }
    }
  }
  return rows;
}

std::uint64_t DemRaster::countNaN() const {
  std::uint64_t n = 0;
  for (const float v : data_) {
    if (std::isnan(v)) ++n;
  }
  return n;
}

}  // namespace demdl
