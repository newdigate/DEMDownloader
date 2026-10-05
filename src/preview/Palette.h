#pragma once

// The preview's colours. No OpenGL or GLFW here, so the tests build it in every configuration.

#include <algorithm>
#include <cmath>

namespace demdl {

struct Rgb {
  unsigned char r, g, b;
  bool operator==(const Rgb&) const = default;
};

// The status colours of spec section 8, one per pixel state that is not a value.
inline constexpr Rgb kTodoColour{40, 40, 40};      // NaN: not fetched yet, or failed in an earlier run
inline constexpr Rgb kNoDataColour{0, 0, 255};     // the server has no elevation there
inline constexpr Rgb kQueuedColour{255, 0, 0};
inline constexpr Rgb kInFlightColour{0, 255, 0};
inline constexpr Rgb kFailedColour{255, 0, 255};

// Maps t in [0, 1] (low to high) onto light blue (0, 128, 255), cyan, green, yellow and red (255, 0, 0).
// Stops short of pure blue (NoData) and never reaches magenta (Failed) or the to-do grey. NaN counts as 0
// and t is clamped to [0, 1] first, so every input has a colour.
[[nodiscard]] inline Rgb valueColour(float t) noexcept {
  const double clamped = std::isnan(t) ? 0.0 : std::clamp(static_cast<double>(t), 0.0, 1.0);
  const double a = (1.0 - clamped) * 3.5;
  const int x = static_cast<int>(std::floor(a));
  const auto y = static_cast<unsigned char>(std::floor(255.0 * (a - x)));
  switch (x) {
    case 0: return {255, y, 0};
    case 1: return {static_cast<unsigned char>(255 - y), 255, 0};
    case 2: return {0, 255, y};
    default: return {0, static_cast<unsigned char>(255 - y), 255};
  }
}

}  // namespace demdl
