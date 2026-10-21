#include "mesh/ProfileSimplifier.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>

namespace demdl::mesh {

namespace {

std::string formatNumber(double value, int digits) {
  char text[40];
  std::snprintf(text, sizeof text, "%.*g", digits, value);
  return text;
}

// A stretch of the profile still to look through: the heights strictly between first and last, against the straight
// segment from the height at first to the height at last.
struct Run {
  int first, last;
};

// The index to split run at, when it has to be split: that of the height furthest from the run's segment, vertically,
// if it is further than the tolerance. Of equal distances it is the smallest index, because the scan goes upward and
// gives way only to a strictly larger one.
//
// The distance of the height at k from the segment is |h[k] - h[first] - rise (k - first) / length|, where rise is
// h[last] - h[first] and length is last - first. The same length multiplies every distance in the run, so it changes
// neither which is the largest nor which are equal; multiplied through, the distance is
// |(h[k] - h[first]) length - rise (k - first)|, with nothing to divide, and it is compared with tolerance x length.
// Each product is a statement of its own, so that no compiler can fuse it into the subtraction: a fused multiply-add
// rounds once where the two operations round twice, and a platform that has one would find other errors, other ties
// and so other indices than a platform that has not. Where the products are exact (whole-number heights, or any whose
// differences times the length need 53 bits or fewer, as those of a DEM in metres usually do over a side of a few
// thousand steps) nothing is rounded at all, and heights that are equally far off compare equal.
std::optional<int> splitIndex(std::span<const float> heights, Run run, double tolerance) {
  const double length = static_cast<double>(run.last - run.first);
  const double base = heights[run.first];
  const double rise = static_cast<double>(heights[run.last]) - base;
  double worst = 0.0;
  int worstIndex = -1;
  for (int k = run.first + 1; k < run.last; ++k) {
    const double up = (static_cast<double>(heights[k]) - base) * length;
    const double along = rise * static_cast<double>(k - run.first);
    const double off = std::fabs(up - along);
    if (off > worst) {
      worst = off;
      worstIndex = k;
    }
  }
  // worst starts at 0 and the tolerance is not negative, so a split index was found whenever this holds.
  if (worst > tolerance * length) {
    assert(worstIndex > run.first && worstIndex < run.last);
    return worstIndex;
  }
  return std::nullopt;
}

}  // namespace

std::vector<int> simplifyProfile(std::span<const float> heights, double tolerance) {
  if (heights.size() < 2) {
    throw std::invalid_argument("a profile needs at least 2 heights, not " + std::to_string(heights.size()));
  }
  if (!std::isfinite(tolerance) || tolerance < 0.0) {
    throw std::invalid_argument("the tolerance must be finite and not negative, not " + formatNumber(tolerance, 15));
  }
  for (std::size_t i = 0; i < heights.size(); ++i) {
    if (!std::isfinite(heights[i])) {
      throw std::invalid_argument("the height at index " + std::to_string(i) + " is not finite");
    }
  }
  assert(heights.size() <= static_cast<std::size_t>(std::numeric_limits<int>::max()) &&
         "the indices are ints, and the profile has more heights than an int can number");

  const int lastIndex = static_cast<int>(heights.size()) - 1;
  std::vector<int> kept{0, lastIndex};

  // The runs still to be looked through are kept on a stack of our own, not on the call stack: a profile can make every
  // split peel one index off the end of its run (the zigzag of the tests), a chain as long as the profile, and a
  // recursion that deep could overflow the call stack where this only grows a vector. A run with no height strictly
  // between its ends has nothing to look through and is never stacked.
  std::vector<Run> pending;
  const auto pushRun = [&pending](int first, int last) {
    if (last - first >= 2) pending.push_back({first, last});
  };
  pushRun(0, lastIndex);
  while (!pending.empty()) {
    const Run run = pending.back();
    pending.pop_back();
    const std::optional<int> split = splitIndex(heights, run, tolerance);
    if (!split) continue;
    kept.push_back(*split);
    // The right half goes on first so that the left, the run with the smaller first index, comes off first: the runs
    // are taken from the smallest first index up. Which run comes first cannot change what is kept, as each is split
    // by its own two ends alone, but it fixes the order the indices are found in, and they are sorted below anyway.
    pushRun(*split, run.last);
    pushRun(run.first, *split);
  }

  // A split index lies strictly inside the run it splits, and runs never overlap, so no index is kept twice.
  std::sort(kept.begin(), kept.end());
  return kept;
}

}  // namespace demdl::mesh
