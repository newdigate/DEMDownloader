#include "mesh/CropWindow.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <numbers>
#include <stdexcept>
#include <utility>

#include "mesh/TransverseMercator.h"

namespace demdl::mesh {

namespace {

constexpr double kRadiansPerDegree = std::numbers::pi / 180.0;

// How near to a whole sample number, in samples, a fractional one must be to count as that sample (see
// pixelWindowFor). The floating-point error in an index is about 1e-10 samples on a 5 m grid and 2e-9 on one of 25 cm;
// a millionth of a sample is a few micrometres on the City's 5 m grid and far below anything a crop box can mean.
constexpr double kOnSample = 1e-6;

constexpr const char* kCropTooSmall = "the crop window overlaps the file by fewer than 2 x 2 samples";
constexpr const char* kFileTooSmall = "the file has fewer than 2 x 2 samples";

// The crop box in the file's own coordinates: degrees for a geographic file, projected metres for a Transverse
// Mercator one.
struct Rectangle {
  double minX, maxX, minY, maxY;
};

// Written so that a NaN cannot get past: the conversions to int below would be undefined for one.
void checkIsBox(const BoundingBox& b) {
  const bool finite =
      std::isfinite(b.north) && std::isfinite(b.south) && std::isfinite(b.east) && std::isfinite(b.west);
  if (!finite || b.north < b.south || b.east < b.west) {
    throw std::invalid_argument(
        "the crop window must have finite edges, with north not below south and east not west of west");
  }
}

Rectangle boundsInFileCoordinates(const GridGeometry& g, const BoundingBox& crop) {
  if (g.kind == ModelKind::Geographic) return {crop.west, crop.east, crop.south, crop.north};

  const double midLon = (crop.west + crop.east) / 2.0;
  const double midLat = (crop.north + crop.south) / 2.0;
  // Projected, a box's edges are curves, and one that crosses the central meridian (a parallel) or the equator (a
  // meridian) bulges beyond its corners, so the midpoints are taken too (spec 5.1).
  const std::pair<double, double> points[8] = {
      {crop.west, crop.north}, {crop.east, crop.north}, {crop.west, crop.south}, {crop.east, crop.south},
      {midLon, crop.north},    {midLon, crop.south},    {crop.west, midLat},     {crop.east, midLat},
  };
  Rectangle r{std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity(),
              std::numeric_limits<double>::infinity(), -std::numeric_limits<double>::infinity()};
  for (const auto& [lon, lat] : points) {
    // The projection returns NaN only for a coordinate that is not finite, which checkIsBox has ruled out; at its
    // singular point it returns huge numbers, not infinities.
    const Projected p = forwardTransverseMercator(g.tm, lon, lat);
    r.minX = std::min(r.minX, p.x);
    r.maxX = std::max(r.maxX, p.x);
    r.minY = std::min(r.minY, p.y);
    r.maxY = std::max(r.maxY, p.y);
  }
  return r;
}

struct Span {
  int first;
  int last;
};

// The whole samples first to last that cover the fractional sample numbers lo to hi along an axis of count samples,
// clipped to it. last - first is below 1 when the range leaves fewer than two samples.
Span coverRange(double lo, double hi, int count) noexcept {
  // Far from the central meridian a projection returns huge but finite coordinates, and a double beyond int's range
  // has no defined conversion: on x86-64 it gives INT_MIN, which would turn a box reaching the file's east side into
  // one that misses it. So the numbers are clamped as doubles first; one sample beyond either end is as far as
  // clipping needs to see.
  const double limit = static_cast<double>(count);
  lo = std::clamp(lo, -1.0, limit);
  hi = std::clamp(hi, -1.0, limit);
  return {std::max(0, static_cast<int>(std::floor(lo + kOnSample))),
          std::min(count - 1, static_cast<int>(std::ceil(hi - kOnSample)))};
}

// value with the given number of decimals, so fixed(18.2, 7) is "18.2000000". Sized by a first call so that no value,
// however large, is cut short.
std::string fixed(double value, int decimals) {
  const int length = std::snprintf(nullptr, 0, "%.*f", decimals, value);
  if (length < 0) return {};
  std::string text(static_cast<std::size_t>(length), '\0');
  std::snprintf(text.data(), text.size() + 1, "%.*f", decimals, value);
  return text;
}

}  // namespace

PixelWindow pixelWindowFor(const GridGeometry& geometry, const std::optional<BoundingBox>& crop) {
  if (!crop) {
    if (geometry.width < 2 || geometry.height < 2) throw std::runtime_error(kFileTooSmall);
    return {0, 0, geometry.width, geometry.height};
  }
  checkIsBox(*crop);

  const Rectangle r = boundsInFileCoordinates(geometry, *crop);
  // Columns run east from sampleX0 and rows south from sampleY0, so the box's west edge gives the first column and
  // its north edge the first row.
  const Span cols = coverRange((r.minX - geometry.sampleX0) / geometry.dx, (r.maxX - geometry.sampleX0) / geometry.dx,
                               geometry.width);
  const Span rows = coverRange((geometry.sampleY0 - r.maxY) / geometry.dy, (geometry.sampleY0 - r.minY) / geometry.dy,
                               geometry.height);
  // Tested after clipping, so a box that misses the file, which clips to a negative width, is refused with the rest.
  if (cols.last - cols.first < 1 || rows.last - rows.first < 1) throw std::runtime_error(kCropTooSmall);
  return {cols.first, rows.first, cols.last - cols.first + 1, rows.last - rows.first + 1};
}

MetresPerDegree metresPerDegree(double latDeg) noexcept {
  const double phi = latDeg * kRadiansPerDegree;
  MetresPerDegree m;
  m.perDegreeLatitude = 111132.954 - 559.822 * std::cos(2.0 * phi) + 1.175 * std::cos(4.0 * phi);
  m.perDegreeLongitude = 111412.84 * std::cos(phi) - 93.5 * std::cos(3.0 * phi);
  return m;
}

WindowExtent extentInMetres(const GridGeometry& geometry, const PixelWindow& window) {
  WindowExtent e;
  if (geometry.kind == ModelKind::TransverseMercator) {
    e.cellDxM = geometry.dx;
    e.cellDyM = geometry.dy;
  } else {
    // A row's latitude is linear in the row, so the mean of the first and last rows' latitudes is the middle row's.
    const double middleLat =
        (geometry.sampleY(window.row0) + geometry.sampleY(window.row0 + window.rows - 1)) / 2.0;
    const MetresPerDegree m = metresPerDegree(middleLat);
    e.cellDxM = geometry.dx * m.perDegreeLongitude;
    e.cellDyM = geometry.dy * m.perDegreeLatitude;
  }
  e.widthM = (window.cols - 1) * e.cellDxM;
  e.heightM = (window.rows - 1) * e.cellDyM;
  return e;
}

std::string describeWindow(const GridGeometry& geometry, const PixelWindow& window) {
  const int lastCol = window.col0 + window.cols - 1;
  const int lastRow = window.row0 + window.rows - 1;
  std::string text = std::to_string(window.cols) + " x " + std::to_string(window.rows) + " pixels at (" +
                     std::to_string(window.col0) + ", " + std::to_string(window.row0) + "), samples from ";
  if (geometry.kind == ModelKind::Geographic) {
    // Seven decimals are a centimetre, finer than any cell this reads.
    text += "longitude " + fixed(geometry.sampleX(window.col0), 7) + " to " + fixed(geometry.sampleX(lastCol), 7) +
            " and latitude " + fixed(geometry.sampleY(window.row0), 7) + " to " + fixed(geometry.sampleY(lastRow), 7) +
            " degrees";
  } else {
    text += "easting " + fixed(geometry.sampleX(window.col0), 2) + " to " + fixed(geometry.sampleX(lastCol), 2) +
            " m and northing " + fixed(geometry.sampleY(window.row0), 2) + " to " +
            fixed(geometry.sampleY(lastRow), 2) + " m";
  }
  return text;
}

}  // namespace demdl::mesh
