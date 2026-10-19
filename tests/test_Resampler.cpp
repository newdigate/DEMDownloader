#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <functional>
#include <random>
#include <stdexcept>
#include <vector>

#include "mesh/Resampler.h"

using Catch::Matchers::Message;
using demdl::mesh::quadraticBSpline;
using demdl::mesh::resampleQuadratic;

// Where an expected number is not a literal, it follows from the definition in the header: the kernel, the position
// u = i (n - 1) / (m - 1) of output i of m along an axis of n samples, the stretch s = max(1, (n - 1) / (m - 1)), the
// taps within 1.5 s of u, odd reflection past the end samples, weights that sum to 1, the rows then the columns. It is
// worked by hand in the comment beside the case, or by referenceVertex below, which writes the definition out as
// plainly as it can be said (every tap one at a time, each reflection one at a time, no table and no folding of the
// reflected taps into the others) and so shares nothing with the implementation but the definition itself.

namespace {

using Grid = std::vector<float>;

std::size_t cell(int cols, int x, int y) {
  return static_cast<std::size_t>(y) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(x);
}

// A grid of cols x rows heights, row by row from the north-west corner as readWindow and fillNearest give them, whose
// height at column x and row y is f(x, y).
template <typename F>
Grid makeGrid(int cols, int rows, F f) {
  Grid grid(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
  for (int y = 0; y < rows; ++y) {
    for (int x = 0; x < cols; ++x) grid[cell(cols, x, y)] = static_cast<float>(f(x, y));
  }
  return grid;
}

// Heights from low to high in steps of (high - low) / 100000, drawn from std::mt19937, whose output the standard fixes,
// so the grids are the same on every machine; the standard distributions are not, and are not used.
Grid randomGrid(std::mt19937& rng, int cols, int rows, double low, double high) {
  return makeGrid(cols, rows, [&](int, int) {
    return low + (high - low) * static_cast<double>(rng() % 100001) / 100000.0;
  });
}

// The quadratic B-spline as a sum of truncated powers, (t + 3/2)+^2 - 3 (t + 1/2)+^2 + 3 (t - 1/2)+^2 - (t - 3/2)+^2,
// halved: another road to the function that the header gives piece by piece, so that the reference does not lean on
// the code under test for its weights.
double referenceKernel(double t) {
  const auto part = [](double x) { return x > 0.0 ? x * x : 0.0; };
  return (part(t + 1.5) - 3.0 * part(t + 0.5) + 3.0 * part(t - 0.5) - part(t - 1.5)) / 2.0;
}

// Output i of m along an axis of n samples whose k-th is valueAt(k), straight from the definition. The reflection is
// applied one step at a time, v(-k) = 2 v(0) - v(k) and v(last + k) = 2 v(last) - v(last - k), as often as it takes to
// land inside the samples.
double referenceAxis(const std::function<double(long)>& valueAt, long n, long m, long i) {
  const long last = n - 1;
  const double u = static_cast<double>(i) * static_cast<double>(last) / static_cast<double>(m - 1);
  const double s = std::max(1.0, static_cast<double>(last) / static_cast<double>(m - 1));
  const std::function<double(long)> extended = [&](long k) -> double {
    if (k < 0) return 2.0 * valueAt(0) - extended(-k);
    if (k > last) return 2.0 * valueAt(last) - extended(2 * last - k);
    return valueAt(k);
  };
  double sum = 0.0;
  double total = 0.0;
  // A window a little wider than the taps, and the rule applied to each candidate, so that no tap is missed or added by
  // the rounding of a floor or a ceiling.
  const long from = static_cast<long>(std::floor(u - 1.5 * s)) - 1;
  const long to = static_cast<long>(std::ceil(u + 1.5 * s)) + 1;
  for (long k = from; k <= to; ++k) {
    if (std::fabs(static_cast<double>(k) - u) >= 1.5 * s) continue;
    const double w = referenceKernel((static_cast<double>(k) - u) / s);
    total += w;
    sum += w * extended(k);
  }
  return sum / total;
}

// Vertex (i, j) of the output: the columns first, along each row the vertical pass will read, then the rows.
double referenceVertex(const Grid& src, int srcCols, int srcRows, int outCols, int outRows, int i, int j) {
  const std::function<double(long)> alongColumn = [&](long y) {
    const std::function<double(long)> alongRow = [&](long x) {
      return static_cast<double>(src[cell(srcCols, static_cast<int>(x), static_cast<int>(y))]);
    };
    return referenceAxis(alongRow, srcCols, outCols, i);
  };
  return referenceAxis(alongColumn, srcRows, outRows, j);
}

// The largest distance of any output vertex from the reference.
double worstDifference(const Grid& src, int srcCols, int srcRows, int outCols, int outRows, const Grid& out) {
  REQUIRE(out.size() == static_cast<std::size_t>(outCols) * static_cast<std::size_t>(outRows));
  double worst = 0.0;
  for (int j = 0; j < outRows; ++j) {
    for (int i = 0; i < outCols; ++i) {
      const double expected = referenceVertex(src, srcCols, srcRows, outCols, outRows, i, j);
      worst = std::max(worst, std::fabs(static_cast<double>(out[cell(outCols, i, j)]) - expected));
    }
  }
  return worst;
}

// The plane z = 2 x + 3 y (x and y are source indices) resampled, and the largest distance of any output vertex from
// 2 u + 3 v, where u = i (cols - 1) / (outCols - 1) and v = j (rows - 1) / (outRows - 1) are the source position that
// vertex stands for.
double planeError(int cols, int rows, int outCols, int outRows) {
  const Grid src = makeGrid(cols, rows, [](int x, int y) { return 2.0 * x + 3.0 * y; });
  const Grid out = resampleQuadratic(src, cols, rows, outCols, outRows);
  REQUIRE(out.size() == static_cast<std::size_t>(outCols) * static_cast<std::size_t>(outRows));
  double worst = 0.0;
  for (int j = 0; j < outRows; ++j) {
    for (int i = 0; i < outCols; ++i) {
      const double u = static_cast<double>(i) * (cols - 1) / (outCols - 1);
      const double v = static_cast<double>(j) * (rows - 1) / (outRows - 1);
      worst = std::max(worst, std::fabs(static_cast<double>(out[cell(outCols, i, j)]) - (2.0 * u + 3.0 * v)));
    }
  }
  return worst;
}

}  // namespace

// ---- the kernel ----

TEST_CASE("the quadratic B-spline has its pinned values", "[resample]") {
  // 3/4 - t^2 below one half, (3/2 - |t|)^2 / 2 from one half to three halves, 0 beyond. Every figure is a binary
  // fraction, so each is exact.
  CHECK(quadraticBSpline(0.0) == 0.75);
  CHECK(quadraticBSpline(0.25) == 0.6875);   // 3/4 - 1/16
  CHECK(quadraticBSpline(0.5) == 0.5);       // 3/4 - 1/4, and (3/2 - 1/2)^2 / 2
  CHECK(quadraticBSpline(0.75) == 0.28125);  // (3/4)^2 / 2
  CHECK(quadraticBSpline(1.0) == 0.125);     // (1/2)^2 / 2
  CHECK(quadraticBSpline(1.25) == 0.03125);  // (1/4)^2 / 2
  CHECK(quadraticBSpline(1.5) == 0.0);
  CHECK(quadraticBSpline(2.0) == 0.0);
  CHECK(quadraticBSpline(1e9) == 0.0);
}

TEST_CASE("the quadratic B-spline is symmetric and its three integer taps sum to 1", "[resample]") {
  for (const double t : {0.0, 0.1, 0.25, 0.5, 0.6, 0.999, 1.0, 1.4, 1.5, 2.0}) {
    CAPTURE(t);
    CHECK(quadraticBSpline(t) == quadraticBSpline(-t));
  }
  CHECK(quadraticBSpline(-1.0) + quadraticBSpline(0.0) + quadraticBSpline(1.0) == 1.0);
}

TEST_CASE("the quadratic B-spline joins smoothly at one half and at three halves", "[resample]") {
  // Continuous, and with the same slope on both sides (C1): -1 at one half, 0 at three halves, where it reaches zero.
  const double h = 1e-6;
  const double half = quadraticBSpline(0.5);
  CHECK(std::fabs((half - quadraticBSpline(0.5 - h)) / h - (-1.0)) < 1e-5);
  CHECK(std::fabs((quadraticBSpline(0.5 + h) - half) / h - (-1.0)) < 1e-5);
  CHECK(std::fabs(quadraticBSpline(1.5 - h)) < 1e-11);
  CHECK(quadraticBSpline(1.5 + h) == 0.0);
}

TEST_CASE("the quadratic B-spline's taps sum to 1 and balance at any offset between samples", "[resample]") {
  // Wherever the kernel sits between samples, the weights of the samples under it sum to 1 and their centre is the
  // kernel's own position: the two facts that let a surface keep its level and a slope keep its slope. They are
  // what the constant and plane cases below rest on.
  for (int step = 0; step <= 40; ++step) {
    const double u = static_cast<double>(step) / 40.0;
    double sum = 0.0;
    double centre = 0.0;
    for (int k = -3; k <= 4; ++k) {
      sum += quadraticBSpline(k - u);
      centre += k * quadraticBSpline(k - u);
    }
    CAPTURE(u);
    CHECK(std::fabs(sum - 1.0) < 1e-14);
    CHECK(std::fabs(centre - u) < 1e-14);
  }
}

// ---- levels, planes, range ----

TEST_CASE("a constant surface stays constant", "[resample]") {
  struct Shape {
    int srcCols, srcRows, outCols, outRows;
  };
  // The first is the plan's case. The others are the same size, finer, a grid of two (a kernel wider than the
  // source), and a very uneven one.
  for (const Shape s : {Shape{10, 10, 4, 3}, Shape{10, 10, 10, 10}, Shape{10, 10, 31, 17}, Shape{10, 10, 2, 2},
                        Shape{3, 200, 40, 2}}) {
    CAPTURE(s.srcCols, s.srcRows, s.outCols, s.outRows);
    const Grid src(static_cast<std::size_t>(s.srcCols) * static_cast<std::size_t>(s.srcRows), 7.5f);
    const Grid out = resampleQuadratic(src, s.srcCols, s.srcRows, s.outCols, s.outRows);
    REQUIRE(out.size() == static_cast<std::size_t>(s.outCols) * static_cast<std::size_t>(s.outRows));
    double worst = 0.0;
    for (const float v : out) worst = std::max(worst, std::fabs(static_cast<double>(v) - 7.5));
    CHECK(worst < 1e-6);
  }
}

TEST_CASE("a plane stays a plane on a coarser grid, corners included", "[resample]") {
  // 20 x 15 to 7 x 5. The stretches are 19/6 across and 14/4 = 3.5 down. The weights of a stretched kernel sampled at
  // whole samples balance exactly on its centre only when the stretch is a whole number, and nearly otherwise, so the
  // plane comes out a little off: evaluating the definition apart from this code gives 0.0042 at worst. The tolerance
  // is 0.05, which leaves room for that and is far below the units that weights not scaled to sum to 1, a reflection
  // that is not odd, or a wrong position would be out by.
  CHECK(planeError(20, 15, 7, 5) < 0.05);
}

TEST_CASE("a plane stays a plane on a grid of the same size", "[resample]") {
  // The stretch is 1 and every output sits on a sample: 3/4 v(i) + 1/8 (v(i - 1) + v(i + 1)) is v(i) on a plane, and
  // at the border the odd reflection v(-1) = 2 v(0) - v(1) gives 3/4 v(0) + 1/8 (2 v(0) - v(1)) + 1/8 v(1) = v(0).
  CHECK(planeError(20, 15, 20, 15) < 1e-4);
}

TEST_CASE("a plane stays a plane on a finer grid", "[resample]") {
  // 20 x 15 to 41 x 29: the stretch is 1, the outputs fall between samples, and the three taps under the kernel sum to
  // 1 with their centre on the output's position (checked above), so the plane is exact to float rounding.
  CHECK(planeError(20, 15, 41, 29) < 0.01);
}

TEST_CASE("a plane stays a plane when the kernel reaches far past the border", "[resample]") {
  // 40 x 1000 to 5 x 7: down the rows the stretch is 999/6 = 166.5, so an output reads up to 500 rows, half of them
  // past the border for the first and last outputs, and the 1000 rows go through the vertical pass in turn; across, the
  // stretch is 39/4 = 9.75.
  CHECK(planeError(40, 1000, 5, 7) < 0.05);
  // And the other way about.
  const Grid src = makeGrid(1000, 40, [](int x, int y) { return 3.0 * x + 2.0 * y; });
  const Grid out = resampleQuadratic(src, 1000, 40, 7, 5);
  REQUIRE(out.size() == 35);
  double worst = 0.0;
  for (int j = 0; j < 5; ++j) {
    for (int i = 0; i < 7; ++i) {
      const double u = i * 999.0 / 6.0;
      const double v = j * 39.0 / 4.0;
      worst = std::max(worst, std::fabs(static_cast<double>(out[cell(7, i, j)]) - (3.0 * u + 2.0 * v)));
    }
  }
  CHECK(worst < 0.05);
}

TEST_CASE("a source more than 46340 samples long is positioned without overflowing", "[resample]") {
  // The position of output i is i (n - 1) / (m - 1), and for n = m = 100000 the product i (n - 1) reaches 10^10, which
  // does not fit an int (the limit is 46340 squared). Two rows keep the grid small. Same size, so the plane z = 0.01 x
  // + 3 y is exact up to float rounding at heights up to 1000; then a coarse grid of 3 over the same row, whose
  // kernel is some 150000 samples wide and folds back off both ends.
  const int n = 100000;
  const Grid src = makeGrid(n, 2, [](int x, int y) { return 0.01 * x + 3.0 * y; });
  const Grid same = resampleQuadratic(src, n, 2, n, 2);
  REQUIRE(same.size() == 2 * static_cast<std::size_t>(n));
  double worst = 0.0;
  for (int j = 0; j < 2; ++j) {
    for (int i = 0; i < n; i += 7) {
      worst = std::max(worst, std::fabs(static_cast<double>(same[cell(n, i, j)]) - (0.01 * i + 3.0 * j)));
    }
    worst = std::max(worst, std::fabs(static_cast<double>(same[cell(n, n - 1, j)]) - (0.01 * (n - 1) + 3.0 * j)));
  }
  CHECK(worst < 1e-3);

  const Grid coarse = resampleQuadratic(src, n, 2, 3, 2);
  REQUIRE(coarse.size() == 6);
  for (int j = 0; j < 2; ++j) {
    CAPTURE(j);
    CHECK(std::fabs(coarse[cell(3, 0, j)] - 3.0f * static_cast<float>(j)) < 1e-4f);
    CHECK(std::fabs(coarse[cell(3, 1, j)] - (0.01 * 49999.5 + 3.0 * j)) < 1e-2);
    CHECK(std::fabs(coarse[cell(3, 2, j)] - (0.01 * 99999 + 3.0 * j)) < 1e-3);
  }
  // The same down a column of that length, where the ring holds every row at once.
  const Grid column = makeGrid(2, n, [](int x, int y) { return 3.0 * x + 0.01 * y; });
  const Grid shortColumn = resampleQuadratic(column, 2, n, 2, 3);
  REQUIRE(shortColumn.size() == 6);
  for (int i = 0; i < 2; ++i) {
    CAPTURE(i);
    CHECK(std::fabs(shortColumn[cell(2, i, 0)] - 3.0f * static_cast<float>(i)) < 1e-4f);
    CHECK(std::fabs(shortColumn[cell(2, i, 1)] - (0.01 * 49999.5 + 3.0 * i)) < 1e-2);
    CHECK(std::fabs(shortColumn[cell(2, i, 2)] - (0.01 * 99999 + 3.0 * i)) < 1e-3);
  }
}

TEST_CASE("the output never leaves the range of the source", "[resample]") {
  // A tap outside the source is folded onto its mirror image inside, which is nearer to the output's position and so
  // weighs at least as much; what is left on that sample is not negative, and every output is a weighted mean of source
  // heights that cannot be above the highest or below the lowest. The plan's case is the first; the others are the same
  // size, finer, small, and a kernel as wide as the source or wider.
  struct Shape {
    int srcCols, srcRows, outCols, outRows;
  };
  std::mt19937 rng(7001);
  for (const Shape s : {Shape{30, 30, 9, 7}, Shape{30, 30, 30, 30}, Shape{30, 30, 61, 59}, Shape{30, 30, 3, 3},
                        Shape{30, 30, 2, 2}, Shape{17, 41, 5, 12}}) {
    CAPTURE(s.srcCols, s.srcRows, s.outCols, s.outRows);
    const Grid src = randomGrid(rng, s.srcCols, s.srcRows, 0.0, 1000.0);
    const auto [lowest, highest] = std::minmax_element(src.begin(), src.end());
    const Grid out = resampleQuadratic(src, s.srcCols, s.srcRows, s.outCols, s.outRows);
    REQUIRE(out.size() == static_cast<std::size_t>(s.outCols) * static_cast<std::size_t>(s.outRows));
    const auto [outLowest, outHighest] = std::minmax_element(out.begin(), out.end());
    CHECK(*outLowest >= *lowest);
    CHECK(*outHighest <= *highest);
  }
}

// ---- smoothing ----

TEST_CASE("a spike on a grid of its own size is lowered and spread over its neighbours", "[resample]") {
  // 100 at the middle of a 9 x 9 field of 0. At the stretch of 1 each axis has three taps, 1/8, 3/4, 1/8, and the
  // columns are done first: along the spike's row the spike becomes 12.5, 75, 12.5 on columns 3, 4, 5, and every other
  // row stays 0. Down column 4 the 75 becomes 9.375, 56.25, 9.375 on rows 3, 4, 5; down columns 3 and 5 the 12.5
  // becomes 1.5625, 9.375, 1.5625. Nothing is left beyond the 3 x 3 block, which pins the kernel's reach as well.
  Grid src(81, 0.0f);
  src[cell(9, 4, 4)] = 100.0f;
  const Grid out = resampleQuadratic(src, 9, 9, 9, 9);
  REQUIRE(out.size() == 81);
  CHECK(out[cell(9, 4, 4)] < 100.0f);
  CHECK(out[cell(9, 4, 4)] > 50.0f);
  CHECK(std::fabs(out[cell(9, 4, 4)] - 56.25f) < 0.01f);
  for (int y = 3; y <= 5; ++y) {
    for (int x = 3; x <= 5; ++x) {
      if (x == 4 && y == 4) continue;
      CAPTURE(x, y);
      CHECK(out[cell(9, x, y)] > 0.0f);
      const bool sideNeighbour = x == 4 || y == 4;
      CHECK(std::fabs(out[cell(9, x, y)] - (sideNeighbour ? 9.375f : 1.5625f)) < 0.001f);
    }
  }
  for (int y = 0; y < 9; ++y) {
    for (int x = 0; x < 9; ++x) {
      if (x >= 3 && x <= 5 && y >= 3 && y <= 5) continue;
      CAPTURE(x, y);
      CHECK(std::fabs(out[cell(9, x, y)]) < 1e-6f);
    }
  }
}

TEST_CASE("a spike on a finer grid is read at the source spacing, not stretched", "[resample]") {
  // 9 x 9 to 17 x 17: u = i / 2, so output 8 sits on the spike (weight 3/4), outputs 7 and 9 half a sample away
  // (B(1/2) = 1/2) and outputs 6 and 10 a sample away (1/8); 5 and 11 are 1.5 samples away, outside the kernel. A
  // spike of 100 therefore gives 56.25 at (8, 8), 37.5 at (7, 8), 25 at (7, 7), 9.375 at (6, 8), 6.25 at (6, 7),
  // 1.5625 at (6, 6) and nothing beyond. A kernel stretched for the finer grid would be narrower and look different.
  Grid src(81, 0.0f);
  src[cell(9, 4, 4)] = 100.0f;
  const Grid out = resampleQuadratic(src, 9, 9, 17, 17);
  REQUIRE(out.size() == 289);
  const double weight[5] = {0.125, 0.5, 0.75, 0.5, 0.125};  // outputs 6 to 10, along either axis
  for (int j = 0; j < 17; ++j) {
    for (int i = 0; i < 17; ++i) {
      const bool inside = i >= 6 && i <= 10 && j >= 6 && j <= 10;
      const double expected = inside ? 100.0 * weight[i - 6] * weight[j - 6] : 0.0;
      CAPTURE(i, j);
      CHECK(std::fabs(static_cast<double>(out[cell(17, i, j)]) - expected) < 1e-4);
    }
  }
  CHECK(std::fabs(out[cell(17, 8, 8)] - 56.25f) < 1e-4f);
  CHECK(std::fabs(out[cell(17, 7, 8)] - 37.5f) < 1e-4f);
  CHECK(std::fabs(out[cell(17, 6, 7)] - 6.25f) < 1e-4f);
}

TEST_CASE("a row of alternating 0 and 100 down to 64 samples averages out instead of aliasing", "[resample]") {
  // 2048 samples to 64: the stretch is 2047/63 = 32.49, so the kernel spans about 97 samples, 48 on each side. Three
  // taps at the source spacing would give 25 or 75 for an output on a sample (1/8 x 100 + 1/8 x 100, or 3/4 x 100 + 0)
  // and 50 half way between, so most outputs would be more than 10 from 50. The grid has two rows because an output
  // grid must be at least 2 x 2; they are the same, and so are the outputs.
  const Grid src = makeGrid(2048, 2, [](int x, int) { return x % 2 == 0 ? 0.0 : 100.0; });
  const Grid out = resampleQuadratic(src, 2048, 2, 64, 2);
  REQUIRE(out.size() == 128);
  for (int row = 0; row < 2; ++row) {
    // The plan's bound of 10, for every output but the two on the border, which are the end samples (see below).
    for (int i = 1; i <= 62; ++i) {
      CAPTURE(row, i);
      CHECK(std::fabs(out[cell(64, i, row)] - 50.0f) < 10.0f);
    }
    // Away from the border, where no reflected tap is involved, the alternation of 50 either side survives at under a
    // thousandth: 2.6e-4 at worst, by evaluating the definition apart from this code, the sampled kernel's response at
    // the highest frequency being about 5e-6.
    for (int i = 2; i <= 61; ++i) {
      CAPTURE(row, i);
      CHECK(std::fabs(out[cell(64, i, row)] - 50.0f) < 0.001f);
    }
    // The border outputs are the end samples, 0 and 100: see "the corners of the grid are the corners of the source".
    CHECK(out[cell(64, 0, row)] == 0.0f);
    CHECK(out[cell(64, 63, row)] == 100.0f);
  }
}

TEST_CASE("a column of alternating 0 and 100 down to 64 samples averages out the same way", "[resample]") {
  // The vertical pass has its own table and reads the rows through its own window: the same figures down the rows.
  const Grid src = makeGrid(2, 2048, [](int, int y) { return y % 2 == 0 ? 0.0 : 100.0; });
  const Grid out = resampleQuadratic(src, 2, 2048, 2, 64);
  REQUIRE(out.size() == 128);
  for (int col = 0; col < 2; ++col) {
    for (int j = 1; j <= 62; ++j) {
      CAPTURE(col, j);
      CHECK(std::fabs(out[cell(2, col, j)] - 50.0f) < 10.0f);
    }
    for (int j = 2; j <= 61; ++j) {
      CAPTURE(col, j);
      CHECK(std::fabs(out[cell(2, col, j)] - 50.0f) < 0.001f);
    }
    CHECK(out[cell(2, col, 0)] == 0.0f);
    CHECK(out[cell(2, col, 63)] == 100.0f);
  }
}

// ---- the border ----

TEST_CASE("the corners of the grid are the corners of the source", "[resample]") {
  // Output 0 sits on sample 0 and the taps on either side of it weigh the same, so the odd reflection makes each pair
  // v(-k) + v(k) = 2 v(0) and the output is v(0) exactly, whatever the stretch, and likewise at the far end. The
  // grid's corners are therefore the source's corners, which is also what lets a plane be right "including the
  // corners". The shapes run from the same size to a kernel wider than the source, which reflects more than once.
  struct Shape {
    int srcCols, srcRows, outCols, outRows;
  };
  std::mt19937 rng(7002);
  for (const Shape s : {Shape{17, 13, 17, 13}, Shape{17, 13, 40, 30}, Shape{17, 13, 5, 4}, Shape{17, 13, 3, 3},
                        Shape{17, 13, 9, 2}, Shape{10, 10, 2, 2}, Shape{2, 2, 2, 2}, Shape{2, 2, 5, 5},
                        Shape{3, 50, 2, 2}, Shape{400, 6, 3, 9}}) {
    CAPTURE(s.srcCols, s.srcRows, s.outCols, s.outRows);
    const Grid src = randomGrid(rng, s.srcCols, s.srcRows, 1.0, 1000.0);
    const Grid out = resampleQuadratic(src, s.srcCols, s.srcRows, s.outCols, s.outRows);
    REQUIRE(out.size() == static_cast<std::size_t>(s.outCols) * static_cast<std::size_t>(s.outRows));
    CHECK(out[cell(s.outCols, 0, 0)] == src[cell(s.srcCols, 0, 0)]);
    CHECK(out[cell(s.outCols, s.outCols - 1, 0)] == src[cell(s.srcCols, s.srcCols - 1, 0)]);
    CHECK(out[cell(s.outCols, 0, s.outRows - 1)] == src[cell(s.srcCols, 0, s.srcRows - 1)]);
    CHECK(out[cell(s.outCols, s.outCols - 1, s.outRows - 1)] == src[cell(s.srcCols, s.srcCols - 1, s.srcRows - 1)]);
  }
}

TEST_CASE("a 2 x 2 source and a 2 x 2 grid are the same four heights", "[resample]") {
  const Grid src = {3.0f, -4.5f, 12.25f, 800.0f};
  const Grid out = resampleQuadratic(src, 2, 2, 2, 2);
  CHECK(out == src);
}

// ---- worked by hand ----

TEST_CASE("five samples down to three use a stretch of 2, hand-worked", "[resample]") {
  // 1 2 4 8 16 to three outputs: u = 0, 2, 4 and s = 4/2 = 2. Output 1 has taps k = 0 to 4 at (k - 2)/2 = -1, -1/2,
  // 0, 1/2, 1, weights B = 1/8, 1/2, 3/4, 1/2, 1/8, which sum to 2, so 1/16, 1/4, 3/8, 1/4, 1/16 after scaling:
  // 1/16 + 2/4 + 4 x 3/8 + 8/4 + 16/16 = 0.0625 + 0.5 + 1.5 + 2 + 1 = 5.0625. Outputs 0 and 2 are the end samples.
  const Grid src = makeGrid(5, 2, [](int x, int) { return 1 << x; });
  const Grid out = resampleQuadratic(src, 5, 2, 3, 2);
  REQUIRE(out.size() == 6);
  for (int row = 0; row < 2; ++row) {
    CHECK(std::fabs(out[cell(3, 0, row)] - 1.0f) < 1e-5f);
    CHECK(std::fabs(out[cell(3, 1, row)] - 5.0625f) < 1e-5f);
    CHECK(std::fabs(out[cell(3, 2, row)] - 16.0f) < 1e-5f);
  }
}

TEST_CASE("the same five samples down a column give the same three heights", "[resample]") {
  const Grid src = makeGrid(2, 5, [](int, int y) { return 1 << y; });
  const Grid out = resampleQuadratic(src, 2, 5, 2, 3);
  REQUIRE(out.size() == 6);
  for (int col = 0; col < 2; ++col) {
    CHECK(std::fabs(out[cell(2, col, 0)] - 1.0f) < 1e-5f);
    CHECK(std::fabs(out[cell(2, col, 1)] - 5.0625f) < 1e-5f);
    CHECK(std::fabs(out[cell(2, col, 2)] - 16.0f) < 1e-5f);
  }
}

TEST_CASE("two samples up to three give the midpoint between them, hand-worked", "[resample]") {
  // 0 and 8 to three outputs: u = 0, 1/2, 1 and s = 1. Output 1 has the taps k = 0 and 1 at distance 1/2, weight 1/2
  // each (k = -1 and 2 are 1.5 away, where the kernel is 0): 4. Output 0 has the taps -1, 0, 1 with weights 1/8, 3/4,
  // 1/8 and v(-1) = 2 v(0) - v(1) = -8: -1 + 0 + 1 = 0. Output 2 is 8 by the same argument.
  const Grid src = makeGrid(2, 2, [](int x, int) { return 8.0 * x; });
  const Grid out = resampleQuadratic(src, 2, 2, 3, 2);
  REQUIRE(out.size() == 6);
  for (int row = 0; row < 2; ++row) {
    CHECK(std::fabs(out[cell(3, 0, row)] - 0.0f) < 1e-5f);
    CHECK(std::fabs(out[cell(3, 1, row)] - 4.0f) < 1e-5f);
    CHECK(std::fabs(out[cell(3, 2, row)] - 8.0f) < 1e-5f);
  }
}

TEST_CASE("a summit is lowered by the kernel: 0 8 0 up to five samples, hand-worked", "[resample]") {
  // u = 0, 1/2, 1, 3/2, 2 and s = 1. Output 1: taps 0 and 1 at 1/2, weights 1/2 each: 0 x 1/2 + 8 x 1/2 = 4. Output 2
  // sits on the summit: 1/8 x 0 + 3/4 x 8 + 1/8 x 0 = 6, not 8, because a B-spline approximates its samples. Output 3
  // is output 1 mirrored. Outputs 0 and 4 are the ends.
  const Grid src = makeGrid(3, 2, [](int x, int) { return x == 1 ? 8.0 : 0.0; });
  const Grid out = resampleQuadratic(src, 3, 2, 5, 2);
  REQUIRE(out.size() == 10);
  const float expected[5] = {0.0f, 4.0f, 6.0f, 4.0f, 0.0f};
  for (int row = 0; row < 2; ++row) {
    for (int i = 0; i < 5; ++i) {
      CAPTURE(row, i);
      CHECK(std::fabs(out[cell(5, i, row)] - expected[i]) < 1e-5f);
    }
  }
}

// ---- against the definition ----

TEST_CASE("every output follows the definition, coarser, finer, equal, wide, tall, tiny", "[resample]") {
  struct Shape {
    int srcCols, srcRows, outCols, outRows;
  };
  // Stretches from 1 to about 20 on each axis; grids of two or three along an axis, where the kernel is as wide as the
  // source or wider and reflects off both ends (more than once, with two); sources only a few samples across; and
  // 300 x 200 to 17 x 11, in which the rows the vertical pass reads at once are far fewer than the 200 and move down
  // the source as the outputs do.
  const Shape shapes[] = {
      {2, 2, 2, 2},   {2, 2, 3, 3},   {2, 3, 5, 2},   {3, 2, 2, 5},   {3, 3, 2, 2},    {10, 10, 2, 2},
      {10, 7, 3, 2},  {7, 10, 2, 3},  {20, 15, 7, 5}, {20, 15, 20, 15}, {20, 15, 41, 29}, {30, 30, 9, 7},
      {31, 9, 16, 5}, {9, 31, 5, 16}, {64, 48, 9, 7}, {48, 64, 31, 33}, {50, 40, 19, 18}, {40, 50, 60, 41},
      {300, 200, 17, 11},
  };
  std::mt19937 rng(7003);
  for (const Shape& s : shapes) {
    CAPTURE(s.srcCols, s.srcRows, s.outCols, s.outRows);
    // Negative heights as well as positive, so that a sign slip in a reflected tap shows. Float spacing at 1000 is
    // 6e-5, and the row pass is rounded to float before the column pass, so the outputs agree with the reference to
    // about a float's last place: 5e-5 at worst, measured. 3e-4 leaves room for another summation order and is far
    // below what a wrong weight would cause.
    const Grid src = randomGrid(rng, s.srcCols, s.srcRows, -500.0, 1000.0);
    const Grid out = resampleQuadratic(src, s.srcCols, s.srcRows, s.outCols, s.outRows);
    CHECK(worstDifference(src, s.srcCols, s.srcRows, s.outCols, s.outRows, out) < 3e-4);
  }
}

TEST_CASE("mirroring the source mirrors the output", "[resample]") {
  const int srcCols = 23, srcRows = 17, outCols = 9, outRows = 7;
  std::mt19937 rng(7004);
  const Grid src = randomGrid(rng, srcCols, srcRows, 0.0, 1000.0);
  const Grid out = resampleQuadratic(src, srcCols, srcRows, outCols, outRows);

  // East-west: each row reversed.
  const Grid flippedAcross =
      makeGrid(srcCols, srcRows, [&](int x, int y) { return src[cell(srcCols, srcCols - 1 - x, y)]; });
  const Grid outAcross = resampleQuadratic(flippedAcross, srcCols, srcRows, outCols, outRows);
  // North-south: the rows in the opposite order.
  const Grid flippedDown =
      makeGrid(srcCols, srcRows, [&](int x, int y) { return src[cell(srcCols, x, srcRows - 1 - y)]; });
  const Grid outDown = resampleQuadratic(flippedDown, srcCols, srcRows, outCols, outRows);

  double worstAcross = 0.0;
  double worstDown = 0.0;
  for (int j = 0; j < outRows; ++j) {
    for (int i = 0; i < outCols; ++i) {
      const double across = outAcross[cell(outCols, i, j)];
      const double down = outDown[cell(outCols, i, j)];
      worstAcross = std::max(worstAcross, std::fabs(across - out[cell(outCols, outCols - 1 - i, j)]));
      worstDown = std::max(worstDown, std::fabs(down - out[cell(outCols, i, outRows - 1 - j)]));
    }
  }
  // The two sides are the same sums taken in the opposite order, so they agree to a float's last place at worst (6e-5
  // at 1000); measured, they agree exactly.
  CHECK(worstAcross < 1e-4);
  CHECK(worstDown < 1e-4);
}

// ---- arguments ----

TEST_CASE("an output grid smaller than 2 x 2 is refused", "[resample]") {
  const Grid src(6, 1.0f);
  CHECK_THROWS_MATCHES(resampleQuadratic(src, 3, 2, 1, 5), std::invalid_argument,
                       Message("the output grid must be at least 2 x 2, not 1 x 5"));
  CHECK_THROWS_MATCHES(resampleQuadratic(src, 3, 2, 5, 1), std::invalid_argument,
                       Message("the output grid must be at least 2 x 2, not 5 x 1"));
  CHECK_THROWS_MATCHES(resampleQuadratic(src, 3, 2, 0, 0), std::invalid_argument,
                       Message("the output grid must be at least 2 x 2, not 0 x 0"));
  CHECK_THROWS_MATCHES(resampleQuadratic(src, 3, 2, -4, 5), std::invalid_argument,
                       Message("the output grid must be at least 2 x 2, not -4 x 5"));
  CHECK_NOTHROW(resampleQuadratic(src, 3, 2, 2, 2));
}

TEST_CASE("a source smaller than 2 x 2, or of the wrong size, is refused", "[resample]") {
  // A mesh needs a cell, and the reflection about the end samples needs two end samples.
  CHECK_THROWS_MATCHES(resampleQuadratic(Grid(6, 1.0f), 1, 6, 4, 4), std::invalid_argument,
                       Message("the source must be at least 2 x 2, not 1 x 6"));
  CHECK_THROWS_MATCHES(resampleQuadratic(Grid(6, 1.0f), 6, 1, 4, 4), std::invalid_argument,
                       Message("the source must be at least 2 x 2, not 6 x 1"));
  CHECK_THROWS_MATCHES(resampleQuadratic(Grid(), 0, 0, 4, 4), std::invalid_argument,
                       Message("the source must be at least 2 x 2, not 0 x 0"));
  // -2 x -3 has a product equal to the size, which must not pass for a window.
  CHECK_THROWS_MATCHES(resampleQuadratic(Grid(6, 1.0f), -2, -3, 4, 4), std::invalid_argument,
                       Message("the source must be at least 2 x 2, not -2 x -3"));
  // A vector that does not hold cols x rows heights would be read past its end.
  CHECK_THROWS_MATCHES(resampleQuadratic(Grid(6, 1.0f), 3, 3, 4, 4), std::invalid_argument,
                       Message("a source of 3 x 3 samples needs 9 heights, not 6"));
  CHECK_THROWS_MATCHES(resampleQuadratic(Grid(10, 1.0f), 3, 3, 4, 4), std::invalid_argument,
                       Message("a source of 3 x 3 samples needs 9 heights, not 10"));
}
