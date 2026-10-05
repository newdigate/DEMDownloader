#include <catch2/catch_test_macros.hpp>

#include <limits>
#include <string>
#include <vector>

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

TEST_CASE("no value is drawn in the NoData, Failed or to-do colour", "[palette]") {
  // 10001 evenly spaced samples of [0, 1], both ends included.
  std::vector<float> clashes;
  for (int i = 0; i <= 10000; ++i) {
    const float t = static_cast<float>(i) / 10000.0f;
    const Rgb c = valueColour(t);
    if (c == demdl::kNoDataColour || c == demdl::kFailedColour || c == demdl::kTodoColour) clashes.push_back(t);
  }
  CHECK(clashes == std::vector<float>{});
}
