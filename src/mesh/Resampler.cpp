#include "mesh/Resampler.h"

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace demdl::mesh {

double quadraticBSpline(double t) noexcept {
  const double a = std::fabs(t);
  if (a < 0.5) return 0.75 - a * a;
  if (a < 1.5) {
    const double d = 1.5 - a;
    return 0.5 * d * d;
  }
  return 0.0;
}

namespace {

// What odd reflection makes of a tap at source position k when the samples are 0 to last. Reflecting about the first
// sample gives v(-j) = 2 v(0) - v(j) and about the last gives v(last + j) = 2 v(last) - v(last - j), so after any
// number of reflections
//   v(k) = sign * v(index) + 2 * firstCount * v(0) + 2 * lastCount * v(last)
// with index inside 0 to last. Each reflection flips the sign, and adds (the sign it had) to the count of the end it
// was made about.
struct FoldedTap {
  std::int64_t index;
  int sign;
  int firstCount;
  int lastCount;
};

FoldedTap fold(std::int64_t k, std::int64_t last) noexcept {
  FoldedTap tap{k, 1, 0, 0};
  // A tap further than `last` outside the samples lands outside the far end and is reflected again. Each reflection
  // takes `last` (at least 1) off its distance from the samples, so this ends. It runs more than once only when the
  // kernel is wider than the source, which takes an output grid of two along that axis.
  while (tap.index < 0 || tap.index > last) {
    if (tap.index < 0) {
      tap.firstCount += tap.sign;
      tap.index = -tap.index;
    } else {
      tap.lastCount += tap.sign;
      tap.index = 2 * last - tap.index;
    }
    tap.sign = -tap.sign;
  }
  return tap;
}

// The weights that turn the samples of one axis into its outputs. Output i reads the samples first[i] up to
// first[i] + count - 1 (count being begin[i + 1] - begin[i]) with the weights weight[begin[i]] and on, contiguous
// because the taps are, and with the odd reflection already folded into them: a reflected tap's weight is taken off
// the sample that is its mirror image and its 2 v(0) or 2 v(last) is put on the end sample, so that no sample outside
// the source is ever looked up. That makes the inner loops plain sums of weight x sample.
struct AxisTaps {
  std::vector<int> first;
  std::vector<std::size_t> begin;
  std::vector<double> weight;
  int widest = 0;  // the most samples any output reads
};

// The taps for `outputs` outputs along an axis of `samples` samples, both at least 2: the definition of the header.
AxisTaps makeTaps(int samples, int outputs) {
  const std::int64_t last = samples - 1;
  const std::int64_t gaps = outputs - 1;
  const double stretch = std::max(1.0, static_cast<double>(last) / static_cast<double>(gaps));
  const double reach = 1.5 * stretch;

  AxisTaps taps;
  taps.first.resize(static_cast<std::size_t>(outputs));
  taps.begin.assign(static_cast<std::size_t>(outputs) + 1, 0);

  for (int i = 0; i < outputs; ++i) {
    // The product is exact in 64 bits and the division rounds once, so the first output is at 0, the last at exactly
    // `last`, and with as many outputs as samples output i is at exactly i.
    const double u = static_cast<double>(static_cast<std::int64_t>(i) * last) / static_cast<double>(gaps);
    // The whole numbers strictly between u - reach and u + reach. A tap exactly on the edge would weigh 0 anyway.
    const auto from = static_cast<std::int64_t>(std::floor(u - reach)) + 1;
    const auto to = static_cast<std::int64_t>(std::ceil(u + reach)) - 1;

    // First pass: the weights' sum, for scaling them to 1, and the range of samples that they land on once folded.
    double total = 0.0;
    std::int64_t lowest = last;
    std::int64_t highest = 0;
    for (std::int64_t k = from; k <= to; ++k) {
      total += quadraticBSpline((static_cast<double>(k) - u) / stretch);
      const FoldedTap tap = fold(k, last);
      lowest = std::min(lowest, tap.index);
      highest = std::max(highest, tap.index);
      // The end samples take the 2 v(0) and 2 v(last) of the reflections, so the range must reach them. (It does
      // anyway: a tap below 0 means the tap at 0 is among them, and likewise at the far end.)
      if (tap.firstCount != 0) lowest = 0;
      if (tap.lastCount != 0) highest = last;
    }

    // Second pass: the scaled weights, folded onto the samples.
    const std::size_t base = taps.weight.size();
    const auto count = static_cast<std::size_t>(highest - lowest + 1);
    taps.weight.resize(base + count, 0.0);
    double* const w = taps.weight.data() + base;
    for (std::int64_t k = from; k <= to; ++k) {
      const double scaled = quadraticBSpline((static_cast<double>(k) - u) / stretch) / total;
      const FoldedTap tap = fold(k, last);
      w[tap.index - lowest] += scaled * tap.sign;
      if (tap.firstCount != 0) w[0 - lowest] += 2.0 * scaled * tap.firstCount;
      if (tap.lastCount != 0) w[last - lowest] += 2.0 * scaled * tap.lastCount;
    }

    const auto at = static_cast<std::size_t>(i);
    taps.first[at] = static_cast<int>(lowest);
    taps.begin[at + 1] = taps.weight.size();
    taps.widest = std::max(taps.widest, static_cast<int>(count));
  }
  return taps;
}

// One row of the source, `in`, resampled across to the outputs that `taps` describes.
void resampleRow(const float* in, const AxisTaps& taps, float* out) {
  const std::size_t outputs = taps.first.size();
  for (std::size_t i = 0; i < outputs; ++i) {
    const double* w = taps.weight.data() + taps.begin[i];
    const float* samples = in + taps.first[i];
    const std::size_t count = taps.begin[i + 1] - taps.begin[i];
    double sum = 0.0;
    for (std::size_t t = 0; t < count; ++t) sum += w[t] * static_cast<double>(samples[t]);
    out[i] = static_cast<float>(sum);
  }
}

}  // namespace

std::vector<float> resampleQuadratic(const std::vector<float>& src, int srcCols, int srcRows, int outCols,
                                     int outRows) {
  if (srcCols < 2 || srcRows < 2) {
    throw std::invalid_argument("the source must be at least 2 x 2, not " + std::to_string(srcCols) + " x " +
                                std::to_string(srcRows));
  }
  // Multiplied in 64 bits: the City's whole file is 254 million samples, and a window of 2^31 would wrap an int.
  const std::uint64_t sourceSize = static_cast<std::uint64_t>(srcCols) * static_cast<std::uint64_t>(srcRows);
  if (src.size() != sourceSize) {
    throw std::invalid_argument("a source of " + std::to_string(srcCols) + " x " + std::to_string(srcRows) +
                                " samples needs " + std::to_string(sourceSize) + " heights, not " +
                                std::to_string(src.size()));
  }
  if (outCols < 2 || outRows < 2) {
    throw std::invalid_argument("the output grid must be at least 2 x 2, not " + std::to_string(outCols) + " x " +
                                std::to_string(outRows));
  }
  assert(std::none_of(src.begin(), src.end(), [](float v) { return std::isnan(v); }) &&
         "the source holds a NaN; fillNearest must run first");

  const AxisTaps across = makeTaps(srcCols, outCols);
  const AxisTaps down = makeTaps(srcRows, outRows);

  const auto width = static_cast<std::size_t>(outCols);
  std::vector<float> out(width * static_cast<std::size_t>(outRows));

  // The row pass is done for a source row when the column pass first needs it, and the result is kept in a ring of
  // `down.widest` rows, the most that any output reads at once: source row r lives in slot r % capacity for as long as
  // the next outputs need it. The outputs reach the source rows in order, so each row is done once, and the memory is
  // a few rows of outCols instead of a copy of the whole source across. (A row that had been overwritten and is
  // wanted again would just be done again: held[] says which row a slot holds, so no order is assumed for
  // correctness, only for speed.)
  const auto capacity = static_cast<std::size_t>(down.widest);
  std::vector<float> ring(capacity * width);
  std::vector<int> held(capacity, -1);
  const auto rowAcross = [&](int r) -> const float* {
    const std::size_t slot = static_cast<std::size_t>(r) % capacity;
    float* const resampled = ring.data() + slot * width;
    if (held[slot] != r) {
      resampleRow(src.data() + static_cast<std::size_t>(r) * static_cast<std::size_t>(srcCols), across, resampled);
      held[slot] = r;
    }
    return resampled;
  };

  // Output row j is the weighted sum of the rows the pass across gave, added row by row along whole rows so that the
  // reads run along memory; the sums are in double.
  std::vector<double> sums(width);
  for (int j = 0; j < outRows; ++j) {
    std::fill(sums.begin(), sums.end(), 0.0);
    const auto at = static_cast<std::size_t>(j);
    const std::size_t begin = down.begin[at];
    const std::size_t count = down.begin[at + 1] - begin;
    for (std::size_t t = 0; t < count; ++t) {
      const double w = down.weight[begin + t];
      const float* const row = rowAcross(down.first[at] + static_cast<int>(t));
      for (std::size_t c = 0; c < width; ++c) sums[c] += w * static_cast<double>(row[c]);
    }
    float* const dst = out.data() + at * width;
    for (std::size_t c = 0; c < width; ++c) dst[c] = static_cast<float>(sums[c]);
  }
  return out;
}

}  // namespace demdl::mesh
