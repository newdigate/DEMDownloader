#include "demdl/DemRaster.h"

#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace demdl {

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
