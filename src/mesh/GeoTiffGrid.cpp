#include "mesh/GeoTiffGrid.h"

#include <geotiff.h>
#include <geovalues.h>
#include <tiffio.h>
#include <xtiffio.h>

#include <algorithm>
#include <cctype>
#include <climits>
#include <cstdarg>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <initializer_list>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "demdl/TiffSetup.h"

namespace demdl::mesh {

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

// What the user can do about a file this reader refuses.
constexpr const char* kReproject = "; reproject it first, for example with gdalwarp";

// Throws what, followed by ": <libtiff text>" when libtiff reported errors since the last take.
[[noreturn]] void fail(const std::string& what) {
  std::string message = what;
  const std::string text = takeTiffError();
  if (!text.empty()) message += ": " + text;
  throw std::runtime_error(message);
}

std::string formatNumber(double value) {
  char text[32];
  std::snprintf(text, sizeof text, "%.15g", value);
  return text;
}

TiffHandle openForReading(const std::filesystem::path& path) {
  // The GDAL_NODATA tag is known only to handles opened after this. Errors that earlier calls on this thread left
  // behind are forgotten, so that a failure reports only its own.
  ensureTagsRegistered();
  (void)takeTiffError();
  TiffHandle tif(XTIFFOpen(path.string().c_str(), "r"));
  if (!tif) fail("cannot open " + path.string());
  return tif;
}

enum class SampleType { Float32, Float64, Int16, Int32, UInt16 };

struct ImageLayout {
  int width = 0;
  int height = 0;
  SampleType type = SampleType::Float32;
  std::size_t bytesPerSample = 0;
};

std::string describeSamples(uint16_t format, uint16_t bits) {
  const char* kind = "untyped";
  switch (format) {
    case SAMPLEFORMAT_UINT:
      kind = "unsigned integer";
      break;
    case SAMPLEFORMAT_INT:
      kind = "signed integer";
      break;
    case SAMPLEFORMAT_IEEEFP:
      kind = "floating-point";
      break;
    case SAMPLEFORMAT_COMPLEXINT:
      kind = "complex integer";
      break;
    case SAMPLEFORMAT_COMPLEXIEEEFP:
      kind = "complex floating-point";
      break;
    default:
      break;
  }
  return std::to_string(bits) + "-bit " + kind;
}

// The image's size and sample type, shared by inspectGeoTiff and readWindow so that both refuse the same files.
ImageLayout imageLayout(TIFF* t, const std::string& name) {
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
  if (samplesPerPixel != 1) {
    fail(name + " has " + std::to_string(samplesPerPixel) +
         " samples per pixel; only single-band elevation files are read");
  }
  ImageLayout image;
  if (sampleFormat == SAMPLEFORMAT_IEEEFP && bitsPerSample == 32) {
    image.type = SampleType::Float32;
  } else if (sampleFormat == SAMPLEFORMAT_IEEEFP && bitsPerSample == 64) {
    image.type = SampleType::Float64;
  } else if (sampleFormat == SAMPLEFORMAT_INT && bitsPerSample == 16) {
    image.type = SampleType::Int16;
  } else if (sampleFormat == SAMPLEFORMAT_INT && bitsPerSample == 32) {
    image.type = SampleType::Int32;
  } else if (sampleFormat == SAMPLEFORMAT_UINT && bitsPerSample == 16) {
    image.type = SampleType::UInt16;
  } else {
    fail(name + " has " + describeSamples(sampleFormat, bitsPerSample) +
         " samples; only float32, float64, int16, int32 and uint16 elevations are read");
  }
  if (width == 0 || height == 0) fail(name + " has an empty image");
  constexpr auto kMaxSide = static_cast<uint32_t>(INT_MAX);
  if (width > kMaxSide || height > kMaxSide) {
    fail(name + " is too large: " + std::to_string(width) + " x " + std::to_string(height) + " samples");
  }
  image.width = static_cast<int>(width);
  image.height = static_cast<int>(height);
  image.bytesPerSample = bitsPerSample / 8u;
  return image;
}

// GDAL_NODATA is text. Anything but a number, give or take surrounding spaces, is refused rather than guessed at.
std::optional<double> noDataValue(TIFF* t, const std::string& name) {
  char* text = nullptr;
  if (TIFFGetField(t, TIFFTAG_GDAL_NODATA, &text) != 1 || text == nullptr) return std::nullopt;
  char* end = nullptr;
  const double value = std::strtod(text, &end);
  const char* rest = end;
  while (*rest != '\0' && std::isspace(static_cast<unsigned char>(*rest))) ++rest;
  if (end == text || *rest != '\0') {
    fail(name + " has a GDAL_NODATA value that is not a number: '" + std::string(text) + "'");
  }
  return value;
}

std::string imageDescription(TIFF* t) {
  char* text = nullptr;
  if (TIFFGetField(t, TIFFTAG_IMAGEDESCRIPTION, &text) == 1 && text != nullptr) return text;
  return {};
}

// Where raster position (0, 0) is in the model, and the step from one column, and from one row, to the next.
struct Placement {
  double x0 = 0.0;
  double y0 = 0.0;
  double dx = 0.0;  // east, per column
  double dy = 0.0;  // south, per row
};

Placement placementOf(TIFF* t, const std::string& name) {
  Placement p;
  uint16_t count = 0;
  double* m = nullptr;
  // The standard does not allow a ModelTransformation beside a tiepoint and a pixel scale; if a file has both, the
  // transformation is the one checked for rotation and used.
  if (TIFFGetField(t, TIFFTAG_GEOTRANSMATRIX, &count, &m) == 1 && m != nullptr) {
    if (count < 16) {
      fail(name + " has a ModelTransformation of " + std::to_string(count) + " numbers instead of 16");
    }
    // Row-major 4 x 4: x = m[0] i + m[1] j + m[3] and y = m[4] i + m[5] j + m[7] for raster position (i, j).
    if (m[1] != 0.0 || m[4] != 0.0) fail(name + " is rotated or sheared" + kReproject);
    p = {m[3], m[7], m[0], -m[5]};
  } else {
    uint16_t tieCount = 0;
    double* tie = nullptr;
    if (TIFFGetField(t, TIFFTAG_GEOTIEPOINTS, &tieCount, &tie) != 1 || tie == nullptr || tieCount < 6) {
      fail(name + " has no ModelTiepoint and no ModelTransformation, so its samples cannot be placed");
    }
    uint16_t scaleCount = 0;
    double* scale = nullptr;
    if (TIFFGetField(t, TIFFTAG_GEOPIXELSCALE, &scaleCount, &scale) != 1 || scale == nullptr || scaleCount < 2) {
      fail(name + " has a ModelTiepoint but no ModelPixelScale, so its samples cannot be placed");
    }
    // The first tiepoint ties raster position (i, j) to model position (x, y), so raster (0, 0) is i columns west
    // and j rows north of it. Any further tiepoints are ignored, as GDAL ignores them beside a pixel scale.
    p = {tie[3] - tie[0] * scale[0], tie[4] + tie[1] * scale[1], scale[0], scale[1]};
  }
  if (!std::isfinite(p.x0) || !std::isfinite(p.y0) || !std::isfinite(p.dx) || !std::isfinite(p.dy)) {
    fail(name + " is placed by numbers that are not finite");
  }
  // A step east that is not positive, or a step down that does not go south, would mirror the model; GDAL's reading of
  // a negative ModelPixelScale is a guess that this reader does not make.
  if (!(p.dx > 0.0) || !(p.dy > 0.0)) {
    fail(name + " is not north-up: one column east moves x by " + formatNumber(p.dx) + " and one row down moves y by " +
         formatNumber(-p.dy) + kReproject);
  }
  return p;
}

// A GeoKey's name as the GeoTIFF standard writes it, without libgeotiff's "GeoKey" suffix: "ProjCoordTrans".
std::string keyName(geokey_t key) {
  std::string name = GTIFKeyName(key);
  constexpr std::string_view kSuffix = "GeoKey";
  if (name.ends_with(kSuffix)) name.resize(name.size() - kSuffix.size());
  return name;
}

// libgeotiff's complaints about a malformed key directory, which GTIFNew would print to stderr, collected into the
// std::string given as user data for the exception message instead. Its warnings are dropped, as libtiff's are.
void collectGeoKeyError(GTIF* gtif, int level, const char* format, ...) {
  if (gtif == nullptr || level != LIBGEOTIFF_ERROR) return;
  auto* errors = static_cast<std::string*>(GTIFGetUserData(gtif));
  if (errors == nullptr) return;
  char text[1024];
  va_list ap;
  va_start(ap, format);
  std::vsnprintf(text, sizeof text, format, ap);
  va_end(ap);
  try {
    if (!errors->empty()) *errors += "; ";
    *errors += text;
  } catch (...) {
    // Out of memory: the text is lost, but the failure is still reported.
  }
}

// Typed access to the GeoKeys. A key stored as another type than the standard's is refused rather than read:
// GTIFKeyGet copies the key's own size, so a short read as a double is garbage and a double read as a short overruns
// it.
class GeoKeys {
 public:
  GeoKeys(TIFF* t, std::string name)
      : name_(std::move(name)), gtif_(GTIFNewEx(t, collectGeoKeyError, &errors_)) {
    if (!gtif_) fail("cannot read the GeoTIFF keys of " + name_ + (errors_.empty() ? "" : ": " + errors_));
  }
  // libgeotiff keeps the address of errors_.
  GeoKeys(const GeoKeys&) = delete;
  GeoKeys& operator=(const GeoKeys&) = delete;

  [[nodiscard]] std::optional<int> code(geokey_t key) const {
    if (!present(key, TYPE_SHORT)) return std::nullopt;
    unsigned short value = 0;
    if (GTIFKeyGetSHORT(gtif_.get(), key, &value, 0, 1) != 1) {
      fail("cannot read GeoKey " + keyName(key) + " of " + name_);
    }
    return value;
  }

  [[nodiscard]] std::optional<double> number(geokey_t key) const {
    if (!present(key, TYPE_DOUBLE)) return std::nullopt;
    double value = 0.0;
    if (GTIFKeyGetDOUBLE(gtif_.get(), key, &value, 0, 1) != 1) {
      fail("cannot read GeoKey " + keyName(key) + " of " + name_);
    }
    return value;
  }

 private:
  bool present(geokey_t key, tagtype_t expected) const {
    int size = 0;
    tagtype_t type = TYPE_UNKNOWN;
    if (GTIFKeyInfo(gtif_.get(), key, &size, &type) == 0) return false;
    if (type != expected) {
      fail(name_ + " has GeoKey " + keyName(key) + " stored as " + GTIFTypeName(type) + " rather than " +
           GTIFTypeName(expected));
    }
    return true;
  }

  std::string name_;
  std::string errors_;  // before gtif_, which is handed its address
  GtifHandle gtif_;
};

// The ellipsoid of the file's geographic system, onto the GRS 80 defaults of p. Each axis key overrides its own value,
// so the City's file, which names GRS 1980 and also gives the axes, uses the axes. A missing inverse flattening comes
// from the semi-minor axis when there is one, else from the code, and never defaults to 0, which would mean a sphere.
void readEllipsoid(const GeoKeys& keys, TransverseMercatorParams& p) {
  if (keys.code(GeogEllipsoidGeoKey) == Ellipse_WGS_84) p.inverseFlattening = 298.257223563;  // a is GRS 80's
  if (const std::optional<double> a = keys.number(GeogSemiMajorAxisGeoKey)) p.semiMajorAxis = *a;
  if (const std::optional<double> inverse = keys.number(GeogInvFlatteningGeoKey)) {
    p.inverseFlattening = *inverse;
  } else if (const std::optional<double> b = keys.number(GeogSemiMinorAxisGeoKey)) {
    // Equal axes are a sphere, which TransverseMercatorParams takes as an infinite inverse flattening.
    p.inverseFlattening =
        *b == p.semiMajorAxis ? std::numeric_limits<double>::infinity() : p.semiMajorAxis / (p.semiMajorAxis - *b);
  }
}

// Refuses parameters that could only give nonsense, NaN or a mirrored crop, when the crop box is projected.
void checkProjection(const TransverseMercatorParams& p, const std::string& name) {
  if (!std::isfinite(p.originLongitudeDeg) || !std::isfinite(p.originLatitudeDeg) || !std::isfinite(p.falseEasting) ||
      !std::isfinite(p.falseNorthing) || !(p.scale > 0.0) || !std::isfinite(p.scale)) {
    fail(name + " has Transverse Mercator parameters that are not usable: central meridian " +
         formatNumber(p.originLongitudeDeg) + ", origin latitude " + formatNumber(p.originLatitudeDeg) + ", scale " +
         formatNumber(p.scale) + ", false easting " + formatNumber(p.falseEasting) + ", false northing " +
         formatNumber(p.falseNorthing));
  }
  // An inverse flattening of 0 or infinity is a sphere; between 0 and 1 the flattening would exceed 1.
  if (!(p.semiMajorAxis > 0.0) || !std::isfinite(p.semiMajorAxis) ||
      !(p.inverseFlattening == 0.0 || p.inverseFlattening >= 1.0)) {
    fail(name + " has an ellipsoid that is not usable: semi-major axis " + formatNumber(p.semiMajorAxis) +
         " m, inverse flattening " + formatNumber(p.inverseFlattening));
  }
}

TransverseMercatorParams projectionOf(const GeoKeys& keys, const std::string& name) {
  // EPSG's South African Lo15 to Lo33 grids run west and south. Read as east and north they would turn the model
  // through 180 degrees, so they are refused whatever parameters the file also gives.
  const std::optional<int> code = keys.code(ProjectedCSTypeGeoKey);
  if (code && *code >= 2046 && *code <= 2055) {
    fail(name + " is south-oriented: EPSG:" + std::to_string(*code) + " is a Lo grid whose axes run west and south" +
         kReproject);
  }
  if (const std::optional<int> method = keys.code(ProjCoordTransGeoKey)) {
    if (*method == CT_TransvMercator_SouthOriented) {
      fail(name + " is south-oriented (GeoKey ProjCoordTrans " + std::to_string(*method) +
           "): its axes run west and south" + kReproject);
    }
    if (*method != CT_TransverseMercator) {
      fail(name + " uses a projection this tool does not read (GeoKey ProjCoordTrans " + std::to_string(*method) +
           ")" + kReproject);
    }
    TransverseMercatorParams p;
    readEllipsoid(keys, p);
    // Without it the central meridian would be taken as 0, 19 degrees from the City's own.
    const std::optional<double> centralMeridian = keys.number(ProjNatOriginLongGeoKey);
    if (!centralMeridian) {
      fail(name + " gives Transverse Mercator parameters without a central meridian (GeoKey ProjNatOriginLong)");
    }
    p.originLongitudeDeg = *centralMeridian;
    p.originLatitudeDeg = keys.number(ProjNatOriginLatGeoKey).value_or(0.0);
    p.scale = keys.number(ProjScaleAtNatOriginGeoKey).value_or(1.0);
    p.falseEasting = keys.number(ProjFalseEastingGeoKey).value_or(0.0);
    p.falseNorthing = keys.number(ProjFalseNorthingGeoKey).value_or(0.0);
    checkProjection(p, name);
    return p;
  }
  if (code && *code != KvUserDefined) {
    if (const std::optional<TransverseMercatorParams> utm = utmFromEpsg(*code)) return *utm;
    fail(name + " names EPSG:" + std::to_string(*code) +
         " without parameters, and the only codes known here are the UTM zones, EPSG:32601 to 32660 and 32701 to "
         "32760" +
         kReproject);
  }
  fail(name + " is projected but gives neither Transverse Mercator parameters nor an EPSG code" + kReproject);
}

// The mesh is built from the file's own numbers as metres and degrees: a file in feet would come out three times too
// wide (or its heights three times too tall), and angles in another unit would misplace the central meridian. A
// missing key means metres or degrees, as GeoTIFF readers take it. 9122 is EPSG's "degree (supplier to define
// representation)", the unit EPSG gives WGS 84's angles in.
void checkUnits(const GeoKeys& keys, bool projected, const std::string& name) {
  const auto require = [&](geokey_t key, std::initializer_list<int> accepted, const char* what, const char* unit,
                           const char* remedy) {
    const std::optional<int> given = keys.code(key);
    if (given && std::find(accepted.begin(), accepted.end(), *given) == accepted.end()) {
      fail(name + " gives " + what + " in unit " + std::to_string(*given) + " (GeoKey " + keyName(key) + "), not " +
           unit + remedy);
    }
  };
  require(GeogAngularUnitsGeoKey, {Angular_Degree, 9122}, "angles", "degrees", kReproject);
  if (projected) require(ProjLinearUnitsGeoKey, {Linear_Meter}, "lengths", "metres", kReproject);
  require(VerticalUnitsGeoKey, {Linear_Meter}, "heights", "metres", "; convert them to metres first");
}

// The no-data value as each sample type compares it. A float32 sample is compared with the float the value rounds to:
// writers often print it with fewer digits than it needs (-3.40282346639e+038 for the lowest float32, say), and the
// exact double then matches no float32 sample at all.
struct NoDataMatch {
  explicit NoDataMatch(std::optional<double> v) : value(v), asFloat(v ? static_cast<float>(*v) : 0.0f) {}
  std::optional<double> value;
  float asFloat;
};

constexpr float kNoHeight = std::numeric_limits<float>::quiet_NaN();

float heightOf(float v, const NoDataMatch& noData) {
  return !std::isfinite(v) || (noData.value && v == noData.asFloat) ? kNoHeight : v;
}

float heightOf(double v, const NoDataMatch& noData) {
  if (std::isnan(v) || (noData.value && v == *noData.value)) return kNoHeight;
  const float height = static_cast<float>(v);  // infinite beyond float32's range, so no height either
  return std::isfinite(height) ? height : kNoHeight;
}

template <typename Integer>
float heightOf(Integer v, const NoDataMatch& noData) {
  return noData.value && static_cast<double>(v) == *noData.value ? kNoHeight : static_cast<float>(v);
}

// Converts count samples of type T, in this machine's byte order as libtiff delivers them, to heights.
template <typename T>
void convertSamples(const unsigned char* samples, std::size_t count, float* heights, const NoDataMatch& noData) {
  for (std::size_t i = 0; i < count; ++i) {
    T v;
    std::memcpy(&v, samples + i * sizeof(T), sizeof(T));
    heights[i] = heightOf(v, noData);
  }
}

using Converter = void (*)(const unsigned char*, std::size_t, float*, const NoDataMatch&);

Converter converterFor(SampleType type) {
  switch (type) {
    case SampleType::Float32:
      return convertSamples<float>;
    case SampleType::Float64:
      return convertSamples<double>;
    case SampleType::Int16:
      return convertSamples<std::int16_t>;
    case SampleType::Int32:
      return convertSamples<std::int32_t>;
    case SampleType::UInt16:
      return convertSamples<std::uint16_t>;
  }
  return nullptr;
}

// What readStrips and readTiles share: the window in unsigned numbers, where its heights go, and how to make them.
struct WindowRead {
  std::uint64_t col0, row0, cols, rows;
  std::size_t bytesPerSample;
  Converter convert;
  const NoDataMatch& noData;
  float* heights;

  // Converts count samples into the window's row r (of the file) from column c on.
  void put(const unsigned char* samples, std::uint64_t count, std::uint64_t r, std::uint64_t c) const {
    convert(samples, count, heights + (r - row0) * cols + (c - col0), noData);
  }
};

void readStrips(TIFF* t, const std::string& name, const ImageLayout& image, const WindowRead& w) {
  const tmsize_t lineSize = TIFFScanlineSize(t);
  const std::uint64_t needed = static_cast<std::uint64_t>(image.width) * w.bytesPerSample;
  if (lineSize <= 0 || static_cast<std::uint64_t>(lineSize) < needed) {
    fail("cannot read the rows of " + name);
  }
  std::vector<unsigned char> line(static_cast<std::size_t>(lineSize));
  // Most codecs decode a strip only from its first row, so reading starts there and the rows above the window are
  // decoded and dropped. Uncompressed strips can be entered at any row.
  uint16_t compression = COMPRESSION_NONE;
  uint32_t rowsPerStrip = 0;
  TIFFGetFieldDefaulted(t, TIFFTAG_COMPRESSION, &compression);
  TIFFGetFieldDefaulted(t, TIFFTAG_ROWSPERSTRIP, &rowsPerStrip);
  std::uint64_t first = w.row0;
  if (compression != COMPRESSION_NONE) first = rowsPerStrip == 0 ? 0 : w.row0 - w.row0 % rowsPerStrip;
  for (std::uint64_t row = first; row < w.row0 + w.rows; ++row) {
    if (TIFFReadScanline(t, line.data(), static_cast<uint32_t>(row), 0) < 0) {
      fail("cannot read row " + std::to_string(row) + " of " + name);
    }
    if (row >= w.row0) w.put(line.data() + w.col0 * w.bytesPerSample, w.cols, row, w.col0);
  }
}

void readTiles(TIFF* t, const std::string& name, const WindowRead& w) {
  uint32_t tileWidth = 0;
  uint32_t tileLength = 0;
  TIFFGetField(t, TIFFTAG_TILEWIDTH, &tileWidth);
  TIFFGetField(t, TIFFTAG_TILELENGTH, &tileLength);
  const tmsize_t tileSize = TIFFTileSize(t);
  if (tileWidth == 0 || tileLength == 0 || tileSize <= 0 ||
      static_cast<std::uint64_t>(tileSize) < std::uint64_t{tileWidth} * tileLength * w.bytesPerSample) {
    fail("cannot read the tiles of " + name);
  }
  std::vector<unsigned char> tile(static_cast<std::size_t>(tileSize));
  const std::uint64_t colEnd = w.col0 + w.cols;
  const std::uint64_t rowEnd = w.row0 + w.rows;
  // Each tile that overlaps the window is decoded once, and the part inside the window copied out of it.
  for (std::uint64_t top = w.row0 - w.row0 % tileLength; top < rowEnd; top += tileLength) {
    for (std::uint64_t left = w.col0 - w.col0 % tileWidth; left < colEnd; left += tileWidth) {
      if (TIFFReadTile(t, tile.data(), static_cast<uint32_t>(left), static_cast<uint32_t>(top), 0, 0) < 0) {
        fail("cannot read the tile at column " + std::to_string(left) + ", row " + std::to_string(top) + " of " +
             name);
      }
      const std::uint64_t c0 = std::max(left, w.col0);
      const std::uint64_t c1 = std::min(left + tileWidth, colEnd);
      for (std::uint64_t r = std::max(top, w.row0); r < std::min(top + tileLength, rowEnd); ++r) {
        w.put(tile.data() + ((r - top) * tileWidth + (c0 - left)) * w.bytesPerSample, c1 - c0, r, c0);
      }
    }
  }
}

}  // namespace

GridGeometry inspectGeoTiff(const std::filesystem::path& path) {
  const std::string name = path.string();
  const TiffHandle tif = openForReading(path);
  TIFF* t = tif.get();

  const ImageLayout image = imageLayout(t, name);
  GridGeometry g;
  g.width = image.width;
  g.height = image.height;
  g.description = imageDescription(t);
  g.dlElevation = std::string_view(g.description).starts_with("dl_elevation;");
  g.noData = noDataValue(t, name);
  const Placement placement = placementOf(t, name);

  const GeoKeys keys(t, name);
  const std::optional<int> modelType = keys.code(GTModelTypeGeoKey);
  if (modelType == ModelTypeGeographic) {
    g.kind = ModelKind::Geographic;
    readEllipsoid(keys, g.tm);
  } else if (modelType == ModelTypeProjected) {
    g.kind = ModelKind::TransverseMercator;
    g.tm = projectionOf(keys, name);
  } else {
    const std::string found =
        modelType ? "GeoKey GTModelType " + std::to_string(*modelType) : std::string("it has no GeoKey GTModelType");
    fail(name + " is neither geographic nor projected (" + found + ")" + kReproject);
  }
  checkUnits(keys, g.kind == ModelKind::TransverseMercator, name);
  g.pixelIsPoint = keys.code(GTRasterTypeGeoKey) == RasterPixelIsPoint;

  g.dx = placement.dx;
  g.dy = placement.dy;
  g.sampleX0 = placement.x0;
  g.sampleY0 = placement.y0;
  // An area sample sits at its pixel's centre, half a pixel in from the corner that raster (0, 0) names. dl_elevation's
  // files say area, but the server they come from samples each pixel's north-west corner, so they keep the corner.
  if (!g.pixelIsPoint && !g.dlElevation) {
    g.sampleX0 += 0.5 * g.dx;
    g.sampleY0 -= 0.5 * g.dy;
  }
  return g;
}

std::vector<float> readWindow(const std::filesystem::path& path, const GridGeometry& geometry,
                              const PixelWindow& window) {
  const std::string name = path.string();
  const std::int64_t colEnd = std::int64_t{window.col0} + window.cols;
  const std::int64_t rowEnd = std::int64_t{window.row0} + window.rows;
  if (window.col0 < 0 || window.row0 < 0 || window.cols < 1 || window.rows < 1 || colEnd > geometry.width ||
      rowEnd > geometry.height) {
    throw std::invalid_argument("the window of " + std::to_string(window.cols) + " x " + std::to_string(window.rows) +
                                " samples at column " + std::to_string(window.col0) + ", row " +
                                std::to_string(window.row0) + " is not inside the " + std::to_string(geometry.width) +
                                " x " + std::to_string(geometry.height) + " samples of " + name);
  }
  const TiffHandle tif = openForReading(path);
  TIFF* t = tif.get();
  const ImageLayout image = imageLayout(t, name);
  if (image.width != geometry.width || image.height != geometry.height) {
    fail(name + " has changed since it was inspected: it has " + std::to_string(image.width) + " x " +
         std::to_string(image.height) + " samples, not " + std::to_string(geometry.width) + " x " +
         std::to_string(geometry.height));
  }

  std::vector<float> heights(static_cast<std::size_t>(window.cols) * static_cast<std::size_t>(window.rows));
  const NoDataMatch noData(geometry.noData);
  const WindowRead w{static_cast<std::uint64_t>(window.col0),
                     static_cast<std::uint64_t>(window.row0),
                     static_cast<std::uint64_t>(window.cols),
                     static_cast<std::uint64_t>(window.rows),
                     image.bytesPerSample,
                     converterFor(image.type),
                     noData,
                     heights.data()};
  if (TIFFIsTiled(t)) {
    readTiles(t, name, w);
  } else {
    readStrips(t, name, image, w);
  }
  return heights;
}

}  // namespace demdl::mesh
