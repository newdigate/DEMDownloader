#include "mesh/NoDataFill.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <string>

namespace demdl::mesh {

namespace {

// A pixel's number, row * width + col. Thirty-two bits halve the two lists of the search against 64, which matters
// when the window is the City's whole file of 254 million pixels; the cost is a limit of 4,294,967,295 pixels, 16 GiB
// of heights.
using PixelNumber = std::uint32_t;
using PixelList = std::vector<PixelNumber>;

constexpr std::uint64_t kMostPixels = std::numeric_limits<PixelNumber>::max();
constexpr const char* kNoHeight = "the window has no elevation value at all";

// The pixels that have a height and a blank one up, down, left or right of them, in scan order: where the search
// starts. The scan is done before anything is filled, so that it sees the window as it was given.
PixelList heightsBesideBlanks(const std::vector<float>& values, PixelNumber width, PixelNumber rows) {
  PixelList list;
  PixelNumber p = 0;
  for (PixelNumber row = 0; row < rows; ++row) {
    for (PixelNumber col = 0; col < width; ++col, ++p) {
      if (std::isnan(values[p])) continue;
      // A neighbour is looked at only when the row or column is there: stepping left from column 0 or right from the
      // last column would land on the neighbouring row's far end.
      if ((col > 0 && std::isnan(values[p - 1])) || (col + 1 < width && std::isnan(values[p + 1])) ||
          (row > 0 && std::isnan(values[p - width])) || (row + 1 < rows && std::isnan(values[p + width]))) {
        list.push_back(p);
      }
    }
  }
  return list;
}

}  // namespace

FillReport fillNearest(std::vector<float>& values, int cols, int rows) {
  if (cols < 1 || rows < 1) throw std::invalid_argument("the window must have at least one column and one row");
  // Multiplied in 64 bits: 65536 x 65537 wraps in int, and in 32 bits to a count that passes for a small window.
  const std::uint64_t count = static_cast<std::uint64_t>(cols) * static_cast<std::uint64_t>(rows);
  if (count > kMostPixels) {
    throw std::runtime_error("the window has " + std::to_string(count) + " pixels; the fill can count at most " +
                             std::to_string(kMostPixels));
  }
  if (values.size() != count) {
    throw std::invalid_argument("a window of " + std::to_string(cols) + " x " + std::to_string(rows) +
                                " pixels needs " + std::to_string(count) + " heights, not " +
                                std::to_string(values.size()));
  }

  const auto width = static_cast<PixelNumber>(cols);
  const auto size = static_cast<PixelNumber>(count);

  PixelList wave = heightsBesideBlanks(values, width, static_cast<PixelNumber>(rows));
  if (wave.empty()) {
    // Any pixel of a window can be reached from any other by steps up, down, left and right. So when no pixel with a
    // height has a blank one beside it, either no pixel is blank (nothing to do) or none has a height (nothing to fill
    // from).
    if (std::isnan(values.front())) throw std::runtime_error(kNoHeight);
    return {};
  }

  // The search spreads through blank pixels only. That finds the nearest pixel with a height all the same: a pixel
  // with a height standing on a straight route from another would itself be nearer, so every nearest one is reachable
  // through blanks alone.
  //
  // Ties go to the first of the nearest pixels in scan order, in every wave and not only the first, because of the
  // order the waves are made in. The first wave starts from the pixels in scan order. A wave is made by going through
  // the one before in its order, and each pixel hands the height of its own origin to those it fills, so a wave is
  // always grouped by origin, with the origins that come first in scan order at its front. A blank pixel is filled by
  // the first of its neighbours in the wave before it, so from the earliest origin among them, which by induction on
  // the distance is the earliest of its nearest pixels. The order in which one pixel takes its four neighbours only
  // reorders pixels that share an origin, so it cannot change a height.
  FillReport report;
  float lowest = std::numeric_limits<float>::infinity();
  float highest = -std::numeric_limits<float>::infinity();
  PixelList next;
  while (!wave.empty()) {
    next.clear();
    for (const PixelNumber p : wave) {
      const float passedOn = values[p];
      const PixelNumber col = p % width;
      const std::size_t before = next.size();
      // A blank neighbour takes the height and joins the next wave; a neighbour that has one already, because it had
      // it from the start or was reached by an earlier pixel or wave, is left alone.
      const auto reach = [&](PixelNumber q) {
        if (!std::isnan(values[q])) return;
        values[q] = passedOn;
        next.push_back(q);
      };
      if (p >= width) reach(p - width);
      if (col > 0) reach(p - 1);
      if (col + 1 < width) reach(p + 1);
      if (p < size - width) reach(p + width);
      // Only a height that was handed on counts towards the range. A pixel whose neighbours were all filled first by
      // other pixels gave nothing, and its height may not be among the fills at all.
      if (next.size() != before) {
        lowest = std::min(lowest, passedOn);
        highest = std::max(highest, passedOn);
      }
    }
    report.filled += next.size();
    wave.swap(next);
  }
  report.lowestFill = lowest;
  report.highestFill = highest;
  return report;
}

}  // namespace demdl::mesh
