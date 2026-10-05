#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstdlib>
#include <limits>
#include <string>

#include "preview/Palette.h"

using demdl::Rgb;
using demdl::valueColour;

namespace Catch {

// Shows a colour as (r, g, b) when a check fails.
template <>
struct StringMaker<Rgb> {
  static std::string convert(const Rgb& c) {
    return "(" + std::to_string(c.r) + ", " + std::to_string(c.g) + ", " + std::to_string(c.b) + ")";
  }
};

}  // namespace Catch

namespace {

// The largest difference between two colours in any one channel.
int distance(Rgb a, Rgb b) { return std::max({std::abs(a.r - b.r), std::abs(a.g - b.g), std::abs(a.b - b.b)}); }

// The i-th of 10001 evenly spaced samples of [0, 1], both ends included.
float sample(int i) { return static_cast<float>(i) / 10000.0f; }

}  // namespace

TEST_CASE("the status colours are the spec's", "[palette]") {
  CHECK(demdl::kTodoColour == Rgb{40, 40, 40});
  CHECK(demdl::kNoDataColour == Rgb{0, 0, 255});
  CHECK(demdl::kQueuedColour == Rgb{255, 0, 0});
  CHECK(demdl::kInFlightColour == Rgb{0, 255, 0});
  CHECK(demdl::kFailedColour == Rgb{255, 0, 255});
}

TEST_CASE("values run from light blue at the minimum to red at the maximum", "[palette]") {
  CHECK(valueColour(0.0f) == Rgb{0, 128, 255});
  CHECK(valueColour(1.0f) == Rgb{255, 0, 0});
}

TEST_CASE("a value outside 0 to 1 is clamped and NaN counts as the minimum", "[palette]") {
  CHECK(valueColour(-1.0f) == valueColour(0.0f));
  CHECK(valueColour(2.0f) == valueColour(1.0f));
  CHECK(valueColour(-std::numeric_limits<float>::infinity()) == valueColour(0.0f));
  CHECK(valueColour(std::numeric_limits<float>::infinity()) == valueColour(1.0f));
  CHECK(valueColour(std::numeric_limits<float>::quiet_NaN()) == valueColour(0.0f));
}

TEST_CASE("the value colours change by at most 2 per channel from one sample to the next", "[palette]") {
  int largest = 0;
  float where = 0.0f;
  for (int i = 1; i <= 10000; ++i) {
    const int step = distance(valueColour(sample(i - 1)), valueColour(sample(i)));
    if (step > largest) {
      largest = step;
      where = sample(i);
    }
  }
  INFO("the largest step ends at t = " << where);
  CHECK(largest <= 2);
}

TEST_CASE("every value colour is at least 64 from the NoData, Failed and to-do colours", "[palette]") {
  int nearest = 255;
  float where = 0.0f;
  for (int i = 0; i <= 10000; ++i) {
    const Rgb c = valueColour(sample(i));
    const int d = std::min(
        {distance(c, demdl::kNoDataColour), distance(c, demdl::kFailedColour), distance(c, demdl::kTodoColour)});
    if (d < nearest) {
      nearest = d;
      where = sample(i);
    }
  }
  INFO("the nearest is at t = " << where);
  CHECK(nearest >= 64);
}
