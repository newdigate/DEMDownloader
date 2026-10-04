#pragma once

#include <cassert>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <string>
#include <string_view>
#include <vector>

#include "demdl/BoundingBox.h"
#include "demdl/Config.h"

// Fast-math lets the compiler fold std::isnan to false, and then no pixel would ever be fetched.
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "NaN marks pixels still to fetch; do not build with -ffast-math or -ffinite-math-only"
#endif

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
// Threading: set may run concurrently on distinct cells, and nothing else may overlap a set.
class DemRaster {
 public:
  static constexpr float kNoData = -32768.0f;
  static constexpr double kBoxTolerance = 1e-9;  // degrees, for resume checks

  // Every pixel starts as NaN. Throws std::invalid_argument for a non-positive size.
  [[nodiscard]] static DemRaster create(int width, int height, const BoundingBox& box, RasterMetadata meta);

  // Reads a file written by save(). Throws std::runtime_error with a user-facing message when the
  // file cannot be opened, is not a single-band float32 GeoTIFF (including the previous RGB16
  // format), lacks the georeferencing tags, or describes a box that fails BoundingBox::validate().
  [[nodiscard]] static DemRaster load(const std::filesystem::path& path);

  // Writes <path>.tmp beside the output, then renames it over <path>, so a crash mid-write leaves
  // the previous file intact. Throws std::runtime_error on any failure.
  void save(const std::filesystem::path& path) const;

  // True when this raster can be resumed for cfg. Otherwise `why` lists every differing field,
  // one per line, each indented by two spaces and ending in a newline.
  [[nodiscard]] bool matches(const Config& cfg, std::string& why) const;

  // Non-empty when the file was made from a different WMS URL; a warning, not an error.
  [[nodiscard]] std::string wmsUrlWarning(const Config& cfg) const;

  [[nodiscard]] int width() const { return width_; }
  [[nodiscard]] int height() const { return height_; }
  [[nodiscard]] const BoundingBox& box() const { return box_; }
  [[nodiscard]] const RasterMetadata& metadata() const { return meta_; }

  // at and set require 0 <= x < width() and 0 <= y < height(). Not checked (debug builds assert
  // it), because the Downloader supplies indices from todoInRow and pool jobs must not throw.
  [[nodiscard]] float at(int x, int y) const { return data_[index(x, y)]; }
  void set(int x, int y, float value) { data_[index(x, y)] = value; }

  // Columns still NaN, ascending. Requires 0 <= row < height(); not checked, as for at and set.
  [[nodiscard]] std::vector<int> todoInRow(int row) const;
  [[nodiscard]] std::vector<int> rowsWithWork() const;  // rows with at least one NaN, ascending
  [[nodiscard]] std::uint64_t countNaN() const;
  [[nodiscard]] std::uint64_t pixelCount() const { return data_.size(); }

 private:
  DemRaster() = default;
  std::size_t index(int x, int y) const {
    assert(x >= 0 && x < width_ && y >= 0 && y < height_);
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  }

  int width_ = 0;
  int height_ = 0;
  BoundingBox box_;
  RasterMetadata meta_;
  std::vector<float> data_;
};

}  // namespace demdl
