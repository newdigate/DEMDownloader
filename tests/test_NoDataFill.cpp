#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "mesh/NoDataFill.h"

using Catch::Matchers::Message;
using demdl::mesh::FillReport;
using demdl::mesh::fillNearest;

// A grid is written row by row from its north-west corner, as readWindow gives it, with kNaN for a pixel that has no
// height. Where an expected grid is not a literal it follows from the definition (the fewest steps up, down, left or
// right to a pixel that has a height), worked out pixel by pixel in the comments or by brute force in firstProblem,
// and never from reading the search under test.

namespace {

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr const char* kNoHeight = "the window has no elevation value at all";

using Grid = std::vector<float>;

// The distance between pixels a and b of a grid cols across, in steps of one pixel up, down, left or right.
int steps(int a, int b, int cols) {
  return std::abs(a % cols - b % cols) + std::abs(a / cols - b / cols);
}

// A grid in which each pixel is NaN with the given chance in percent, and every other pixel has a height of its own,
// 1000 plus its number, so that a filled pixel's height says which pixel it was copied from. The generator is
// std::mt19937, whose output the standard fixes, so the grids are the same on every machine; the standard
// distributions are not, and are not used.
Grid randomGrid(std::mt19937& rng, int cols, int rows, unsigned percentBlank) {
  Grid grid(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
  bool anyHeight = false;
  for (std::size_t i = 0; i < grid.size(); ++i) {
    const bool blank = rng() % 100 < percentBlank;
    grid[i] = blank ? kNaN : 1000.0f + static_cast<float>(i);
    anyHeight = anyHeight || !blank;
  }
  // A window with no height at all is an error, tested on its own.
  if (!anyHeight) grid[0] = 1000.0f;
  return grid;
}

// Says what is wrong with after as the nearest-height fill of before (a grid made by randomGrid), or returns "" when
// nothing is. The nearest pixel with a height is found for every blank pixel by looking at all of them, and where
// several are equally near the first in scan order is the one expected (the rule the header states), so this shares
// nothing with the breadth-first search except the definition.
std::string firstProblem(const Grid& before, const Grid& after, int cols, const FillReport& report) {
  const int count = static_cast<int>(before.size());
  std::uint64_t blanks = 0;
  float lowest = std::numeric_limits<float>::infinity();
  float highest = -std::numeric_limits<float>::infinity();
  for (int q = 0; q < count; ++q) {
    if (!std::isnan(before[q])) {
      if (after[q] != before[q]) return "pixel " + std::to_string(q) + " had a height and was changed";
      continue;
    }
    ++blanks;
    if (std::isnan(after[q])) return "pixel " + std::to_string(q) + " is still blank";

    // The first of the nearest: a later pixel has to be strictly nearer to replace it.
    int nearest = std::numeric_limits<int>::max();
    int expected = -1;
    for (int v = 0; v < count; ++v) {
      if (std::isnan(before[v])) continue;
      if (steps(q, v, cols) < nearest) {
        nearest = steps(q, v, cols);
        expected = v;
      }
    }
    // Every valid pixel has a height of its own, so the height says which pixel it came from.
    if (after[q] != before[expected]) {
      return "pixel " + std::to_string(q) + " took the height of pixel " +
             std::to_string(static_cast<int>(after[q]) - 1000) + ", not of pixel " + std::to_string(expected) +
             ", the first of the " + std::to_string(nearest) + "-step pixels";
    }
    lowest = std::min(lowest, after[q]);
    highest = std::max(highest, after[q]);
  }
  if (report.filled != blanks) {
    return "filled is " + std::to_string(report.filled) + ", not " + std::to_string(blanks);
  }
  if (blanks > 0 && (report.lowestFill != lowest || report.highestFill != highest)) {
    return "the range is " + std::to_string(report.lowestFill) + " to " + std::to_string(report.highestFill) +
           ", not " + std::to_string(lowest) + " to " + std::to_string(highest);
  }
  return {};
}

}  // namespace

TEST_CASE("a gap takes the height of the nearest valid pixel along a row", "[fill]") {
  // Pixel 4 is one step from the 5 and one from the 200. Ties go to the first in scan order, the 5, and the 200 is
  // never used: the range reported is of the heights given, 0 to 5, not of the window's, 0 to 200.
  Grid values = {kNaN, kNaN, 0, 5, kNaN, 200};
  const FillReport report = fillNearest(values, 6, 1);
  CHECK(values == Grid{0, 0, 0, 5, 5, 200});
  CHECK(report.filled == 3U);
  CHECK(report.lowestFill == 0.0f);
  CHECK(report.highestFill == 5.0f);
}

TEST_CASE("the same gaps down a column take the same heights", "[fill]") {
  // The row's case turned on its side: a width of 1 makes every neighbour a row above or below.
  Grid values = {kNaN, kNaN, 0, 5, kNaN, 200};
  const FillReport report = fillNearest(values, 1, 6);
  CHECK(values == Grid{0, 0, 0, 5, 5, 200});
  CHECK(report.filled == 3U);
  CHECK(report.lowestFill == 0.0f);
  CHECK(report.highestFill == 5.0f);
}

TEST_CASE("a hole inside a ring takes the ring's height", "[fill]") {
  Grid values(25, 7.0f);
  for (int row = 1; row <= 3; ++row) {
    for (int col = 1; col <= 3; ++col) values[row * 5 + col] = kNaN;
  }
  const FillReport report = fillNearest(values, 5, 5);
  CHECK(values == Grid(25, 7.0f));
  CHECK(report.filled == 9U);
  CHECK(report.lowestFill == 7.0f);
  CHECK(report.highestFill == 7.0f);
}

TEST_CASE("sea beside a coast at 0 m comes out flat at 0 m", "[fill]") {
  Grid values = {0, 0, kNaN, kNaN,
                 0, 0, kNaN, kNaN};
  const FillReport report = fillNearest(values, 4, 2);
  CHECK(values == Grid(8, 0.0f));
  CHECK(report.filled == 4U);
  CHECK(report.lowestFill == 0.0f);
  CHECK(report.highestFill == 0.0f);
}

TEST_CASE("land beyond the data boundary continues as a plateau at the edge height", "[fill]") {
  Grid values = {200, 200, kNaN, kNaN,
                 200, 200, kNaN, kNaN};
  const FillReport report = fillNearest(values, 4, 2);
  CHECK(values == Grid(8, 200.0f));
  CHECK(report.filled == 4U);
  CHECK(report.lowestFill == 200.0f);
  CHECK(report.highestFill == 200.0f);
}

TEST_CASE("each row of a plateau continues at its own edge height", "[fill]") {
  // Row 0 ends at 200 and row 1 at 300; the 100s beside them are farther from the blanks and play no part.
  Grid values = {100, 200, kNaN, kNaN,
                 100, 300, kNaN, kNaN};
  const FillReport report = fillNearest(values, 4, 2);
  CHECK(values == Grid{100, 200, 200, 200,
                       100, 300, 300, 300});
  CHECK(report.filled == 4U);
  CHECK(report.lowestFill == 200.0f);
  CHECK(report.highestFill == 300.0f);
}

TEST_CASE("a window with no height at all is an error", "[fill]") {
  Grid values(9, kNaN);
  REQUIRE_THROWS_MATCHES(fillNearest(values, 3, 3), std::runtime_error, Message(kNoHeight));
  CHECK(std::all_of(values.begin(), values.end(), [](float v) { return std::isnan(v); }));

  Grid single = {kNaN};
  REQUIRE_THROWS_MATCHES(fillNearest(single, 1, 1), std::runtime_error, Message(kNoHeight));
}

TEST_CASE("a window with no blank pixel is returned unchanged", "[fill]") {
  const Grid original = {4, -1.5f, 0, 8.25f, 100, 3};
  Grid values = original;
  const FillReport report = fillNearest(values, 3, 2);
  CHECK(values == original);
  CHECK(report.filled == 0U);
  // Nothing was filled, so there is no range to report.
  CHECK(report.lowestFill == 0.0f);
  CHECK(report.highestFill == 0.0f);

  Grid single = {42};
  const FillReport singleReport = fillNearest(single, 1, 1);
  CHECK(single == Grid{42});
  CHECK(singleReport.filled == 0U);
}

TEST_CASE("the same input twice gives identical output", "[fill]") {
  std::mt19937 rng(7);
  const Grid input = randomGrid(rng, 64, 48, 45);
  Grid first = input;
  Grid second = input;
  const FillReport firstReport = fillNearest(first, 64, 48);
  const FillReport secondReport = fillNearest(second, 64, 48);
  CHECK(first == second);
  CHECK(firstReport.filled == secondReport.filled);
  CHECK(firstReport.lowestFill == secondReport.lowestFill);
  CHECK(firstReport.highestFill == secondReport.highestFill);
  CHECK(firstReport.filled > 0U);

  // Filling what was already filled finds nothing blank and changes nothing.
  Grid again = first;
  const FillReport againReport = fillNearest(again, 64, 48);
  CHECK(again == first);
  CHECK(againReport.filled == 0U);
}

TEST_CASE("pixels equally near two heights take the first of them in scan order", "[fill]") {
  SECTION("a tie three steps out goes to the west") {
    // Pixel 3 is three steps from the 10 and three from the 90, and 10 comes first in scan order. A tie is settled
    // that way however many waves out it is, not only in the first.
    Grid values = {10, kNaN, kNaN, kNaN, kNaN, kNaN, 90};
    const FillReport report = fillNearest(values, 7, 1);
    CHECK(values == Grid{10, 10, 10, 10, 90, 90, 90});
    CHECK(report.filled == 5U);
    CHECK(report.lowestFill == 10.0f);
    CHECK(report.highestFill == 90.0f);
  }
  SECTION("a hole with four equally near neighbours takes the north one") {
    // Scan order is row by row, so of the four the 2 above it comes first.
    Grid values = {1, 2, 3,
                   4, kNaN, 6,
                   7, 8, 9};
    const FillReport report = fillNearest(values, 3, 3);
    CHECK(values == Grid{1, 2, 3,
                         4, 2, 6,
                         7, 8, 9});
    CHECK(report.filled == 1U);
    CHECK(report.lowestFill == 2.0f);
    CHECK(report.highestFill == 2.0f);
  }
  SECTION("a whole diagonal of ties goes to the north-west pixel") {
    // The 10 is at (0,0) and the 90 at (2,4). A pixel is r + c steps from the 10 and 6 - r - c from the 90, so the
    // pixels with r + c = 3, (0,3), (1,2) and (2,1), are three steps from both, and all of them take the 10.
    Grid values = {10,    kNaN, kNaN, kNaN, kNaN,
                   kNaN,  kNaN, kNaN, kNaN, kNaN,
                   kNaN,  kNaN, kNaN, kNaN, 90};
    const FillReport report = fillNearest(values, 5, 3);
    CHECK(values == Grid{10, 10, 10, 10, 90,
                         10, 10, 10, 90, 90,
                         10, 10, 90, 90, 90});
    CHECK(report.filled == 13U);
    CHECK(report.lowestFill == 10.0f);
    CHECK(report.highestFill == 90.0f);
  }
}

TEST_CASE("distance is counted in steps up, down, left and right", "[fill]") {
  // The north-west pixel is three steps south of the 30 and four steps from the 22 (two down, two across), although
  // the 22 is nearer as the crow flies (2.83 pixels against 3). Counting steps gives it the 30; a straight-line or
  // diagonal-step distance would give it the 22. Working every pixel out the same way, no pixel is equally near both:
  //   (0,0) 3 or 4: 30   (0,1) 4 or 3: 22   (0,2) 5 or 2: 22
  //   (1,0) 2 or 3: 30   (1,1) 3 or 2: 22   (1,2) 4 or 1: 22
  //   (2,0) 1 or 2: 30   (2,1) 2 or 1: 22
  //   (3,1) 1 or 2: 30   (3,2) 2 or 1: 22
  Grid values = {kNaN, kNaN, kNaN,
                 kNaN, kNaN, kNaN,
                 kNaN, kNaN, 22,
                 30,   kNaN, kNaN};
  const FillReport report = fillNearest(values, 3, 4);
  CHECK(values == Grid{30, 22, 22,
                       30, 22, 22,
                       30, 22, 22,
                       30, 30, 22});
  CHECK(report.filled == 10U);
  CHECK(report.lowestFill == 22.0f);
  CHECK(report.highestFill == 30.0f);
}

TEST_CASE("a pixel at the end of a row does not reach the start of the next row", "[fill]") {
  // In memory the last pixel of a row is followed by the first of the next, and a search that steps by one index
  // without minding the column would treat them as neighbours.
  SECTION("east edge") {
    // The 1 at (0,3) is next in memory to (1,0), which is truly one step from the 2 at (2,0) and four from the 1.
    Grid values = {kNaN, kNaN, kNaN, 1,
                   kNaN, kNaN, kNaN, kNaN,
                   2,    kNaN, kNaN, kNaN};
    const FillReport report = fillNearest(values, 4, 3);
    CHECK(values == Grid{2, 1, 1, 1,
                         2, 2, 1, 1,
                         2, 2, 2, 1});
    CHECK(report.filled == 10U);
  }
  SECTION("west edge") {
    // The 1 at (1,0) follows (0,3) in memory, which is truly one step from the 2 at (1,3) and four from the 1.
    Grid values = {kNaN, kNaN, kNaN, kNaN,
                   1,    kNaN, kNaN, 2,
                   kNaN, kNaN, kNaN, kNaN};
    const FillReport report = fillNearest(values, 4, 3);
    CHECK(values == Grid{1, 1, 2, 2,
                         1, 1, 2, 2,
                         1, 1, 2, 2});
    CHECK(report.filled == 10U);
  }
}

TEST_CASE("the reported range is of the heights given, below sea level too", "[fill]") {
  // Both heights are negative, which a range that starts from 0 would get wrong.
  Grid values = {-3, kNaN, kNaN, -9};
  const FillReport report = fillNearest(values, 4, 1);
  CHECK(values == Grid{-3, -3, -9, -9});
  CHECK(report.filled == 2U);
  CHECK(report.lowestFill == -9.0f);
  CHECK(report.highestFill == -3.0f);
}

TEST_CASE("every blank pixel takes the height of one of its nearest valid pixels", "[fill]") {
  struct Shape {
    int cols;
    int rows;
  };
  const Shape shapes[] = {{1, 1}, {1, 9}, {9, 1}, {2, 2}, {2, 7}, {7, 2}, {3, 5}, {5, 3}, {16, 13}, {40, 30}};
  std::mt19937 rng(20261019);
  for (const Shape& shape : shapes) {
    for (const unsigned percentBlank : {3U, 20U, 50U, 80U, 97U}) {
      for (int repeat = 0; repeat < 3; ++repeat) {
        CAPTURE(shape.cols, shape.rows, percentBlank, repeat);
        const Grid before = randomGrid(rng, shape.cols, shape.rows, percentBlank);
        Grid after = before;
        const FillReport report = fillNearest(after, shape.cols, shape.rows);
        const std::string problem = firstProblem(before, after, shape.cols, report);
        CHECK(problem == "");
      }
    }
  }
}

TEST_CASE("a long coast fills each row from its own last valid pixel", "[fill]") {
  // Land fills the west half of every row, at 100 plus the row's number, and the east half is blank. A blank pixel's
  // nearest valid pixel is then the last of land in its own row, alone at its distance, so every row comes out flat at
  // its own height: 600 waves over a million pixels.
  const int cols = 1200;
  const int rows = 900;
  const int land = cols / 2;
  Grid values(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), kNaN);
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < land; ++col) {
      values[static_cast<std::size_t>(row * cols + col)] = 100.0f + static_cast<float>(row);
    }
  }
  const FillReport report = fillNearest(values, cols, rows);

  bool everyRowFlat = true;
  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      if (values[static_cast<std::size_t>(row * cols + col)] != 100.0f + static_cast<float>(row)) everyRowFlat = false;
    }
  }
  CHECK(everyRowFlat);
  CHECK(report.filled == static_cast<std::uint64_t>(cols - land) * static_cast<std::uint64_t>(rows));
  CHECK(report.lowestFill == 100.0f);
  CHECK(report.highestFill == 100.0f + static_cast<float>(rows - 1));
}

TEST_CASE("a window with no pixels, or the wrong number of values, is refused", "[fill]") {
  Grid values(6, 1.0f);
  CHECK_THROWS_AS(fillNearest(values, 0, 6), std::invalid_argument);
  CHECK_THROWS_AS(fillNearest(values, 6, 0), std::invalid_argument);
  // -2 x -3 is 6, the number of values held; only a check of each dimension on its own refuses it.
  CHECK_THROWS_AS(fillNearest(values, -2, -3), std::invalid_argument);
  CHECK_THROWS_AS(fillNearest(values, 3, 3), std::invalid_argument);
  CHECK_THROWS_AS(fillNearest(values, 2, 2), std::invalid_argument);
  CHECK(values == Grid(6, 1.0f));

  // An empty vector does add up to a window of no pixels (0 x 5 is 0), so the dimensions are refused on their own.
  Grid none;
  CHECK_THROWS_AS(fillNearest(none, 0, 0), std::invalid_argument);
  CHECK_THROWS_AS(fillNearest(none, 0, 5), std::invalid_argument);
  CHECK_THROWS_AS(fillNearest(none, 5, 0), std::invalid_argument);
}

TEST_CASE("a window too big for 32-bit pixel numbers is refused", "[fill]") {
  // The dimensions are judged before the values are looked at, so no vector of 16 GB is needed to test this, and the
  // refusal is a runtime_error (a limit of the data), not the invalid_argument (a call that does not add up) that the
  // empty vector would otherwise earn.
  Grid none;
  // 65536 x 65537 is 2^32 + 65536 pixels, which wraps to 65536 in 32 bits and to a negative number in an int.
  CHECK_THROWS_MATCHES(fillNearest(none, 65536, 65537), std::runtime_error,
                       Message("the window has 4295032832 pixels; the fill can count at most 4294967295"));
  // 65536 x 65536 is 2^32, one over the most that is counted; 65537 x 65535 is 2^32 - 1, exactly the most, so it gets
  // as far as finding that the empty vector is the wrong size.
  CHECK_THROWS_AS(fillNearest(none, 65536, 65536), std::runtime_error);
  CHECK_THROWS_AS(fillNearest(none, 65537, 65535), std::invalid_argument);
}
