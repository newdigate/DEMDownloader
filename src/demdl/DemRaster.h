#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "demdl/BoundingBox.h"

namespace demdl {

struct RasterMetadata {
  std::string layer;
  std::string wmsUrl;
};

// "dl_elevation;layer=<layer>;wms=<url>". Fields are separated by ';'. wms is always last and
// extends to the end of the string so a URL may contain any character.
[[nodiscard]] std::string formatImageDescription(const RasterMetadata& meta);
[[nodiscard]] RasterMetadata parseImageDescription(std::string_view description);

// A float32 elevation grid in metres. Row 0 is the north edge, column 0 the west edge.
// NaN means "no value yet"; kNoData means the server has no elevation there.
class DemRaster {
 public:
  static constexpr float kNoData = -32768.0f;

  // Every pixel starts as NaN. Throws std::invalid_argument for a non-positive size.
  [[nodiscard]] static DemRaster create(int width, int height, const BoundingBox& box, RasterMetadata meta);

  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] const BoundingBox& box() const { return box_; }
  [[nodiscard]] const RasterMetadata& metadata() const { return meta_; }

  [[nodiscard]] float at(int x, int y) const { return data_[index(x, y)]; }
  void set(int x, int y, float value) { data_[index(x, y)] = value; }

  [[nodiscard]] std::vector<int> todoInRow(int row) const;  // columns still NaN, ascending
  [[nodiscard]] std::vector<int> rowsWithWork() const;      // rows with at least one NaN, ascending
  [[nodiscard]] std::uint64_t countNaN() const;
  [[nodiscard]] std::uint64_t pixelCount() const { return data_.size(); }

 private:
  DemRaster() = default;
  std::size_t index(int x, int y) const {
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  }

  int width_ = 0;
  int height_ = 0;
  BoundingBox box_;
  RasterMetadata meta_;
  std::vector<float> data_;
};

}  // namespace demdl
