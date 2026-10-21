#pragma once

#include <cstdint>

namespace demdl::mesh {

// A vertex of the simplifier's triangulation, addressed by its column and row on the output grid, counted from a corner
// of the chunk being worked on (spec 4.4). Whole numbers, so that the two predicates below can be exact.
struct GridPoint {
  std::int32_t x, y;
};

// The predicates are exact for points whose coordinates all lie in [-kMostPredicateCoordinate,
// kMostPredicateCoordinate]. A chunk is at most 16,384 cells across (spec 4.5), so a point counted from the chunk's own
// corner lies in [0, 16384], well inside that. A debug build asserts the bound; a release build does not check it.
inline constexpr std::int32_t kMostPredicateCoordinate = 16384;

// (b - a) x (c - a): > 0 when a, b, c turn counter-clockwise in a frame with x to the right and y up, < 0 clockwise,
// 0 when in line. It is twice the area of the triangle, with the sign of its turn. Exact for |coordinates| <=
// kMostPredicateCoordinate (the products stay inside 64 bits): the differences are at most 2 x 16384 = 2^15 in size and
// the products 2^30.
[[nodiscard]] std::int64_t orient2d(GridPoint a, GridPoint b, GridPoint c) noexcept;

// For a, b, c counter-clockwise: > 0 when d lies inside their circumcircle, < 0 outside, 0 on it. For a, b, c clockwise
// every sign is reversed. Exact under the same bound: relative to d every coordinate difference is at most 2^15, so
// each squared distance is at most 2^31 and each cross product at most 2^30 (twice the area of a triangle inside a
// square of side 2^15). The three products that are summed are then each at most 2^61, so no product or partial sum
// passes 3 x 2^61, inside 2^63. The determinant itself is smaller still, at most about 12 x 16384^4.
[[nodiscard]] std::int64_t inCircle(GridPoint a, GridPoint b, GridPoint c, GridPoint d) noexcept;

}  // namespace demdl::mesh
