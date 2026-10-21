#include "mesh/Predicates.h"

#include <cassert>

namespace demdl::mesh {

namespace {

// Whether p lies inside the bound of the header. Only the asserts ask, so a release build leaves it unused.
[[maybe_unused]] bool withinBound(GridPoint p) noexcept {
  return p.x >= -kMostPredicateCoordinate && p.x <= kMostPredicateCoordinate && p.y >= -kMostPredicateCoordinate &&
         p.y <= kMostPredicateCoordinate;
}

// p - q, taken in 64 bits.
std::int64_t difference(std::int32_t p, std::int32_t q) noexcept { return std::int64_t{p} - q; }

}  // namespace

std::int64_t orient2d(GridPoint a, GridPoint b, GridPoint c) noexcept {
  assert(withinBound(a) && withinBound(b) && withinBound(c) && "a coordinate is beyond kMostPredicateCoordinate");
  const std::int64_t abx = difference(b.x, a.x);
  const std::int64_t aby = difference(b.y, a.y);
  const std::int64_t acx = difference(c.x, a.x);
  const std::int64_t acy = difference(c.y, a.y);
  return abx * acy - aby * acx;
}

std::int64_t inCircle(GridPoint a, GridPoint b, GridPoint c, GridPoint d) noexcept {
  assert(withinBound(a) && withinBound(b) && withinBound(c) && withinBound(d) &&
         "a coordinate is beyond kMostPredicateCoordinate");
  // The determinant of the rows (x, y, x^2 + y^2) of a, b and c taken relative to d, expanded along the last column.
  // Its sign is positive for d inside the circle when a, b, c run counter-clockwise.
  const std::int64_t adx = difference(a.x, d.x);
  const std::int64_t ady = difference(a.y, d.y);
  const std::int64_t bdx = difference(b.x, d.x);
  const std::int64_t bdy = difference(b.y, d.y);
  const std::int64_t cdx = difference(c.x, d.x);
  const std::int64_t cdy = difference(c.y, d.y);
  const std::int64_t aLift = adx * adx + ady * ady;
  const std::int64_t bLift = bdx * bdx + bdy * bdy;
  const std::int64_t cLift = cdx * cdx + cdy * cdy;
  return aLift * (bdx * cdy - cdx * bdy) + bLift * (cdx * ady - adx * cdy) + cLift * (adx * bdy - bdx * ady);
}

}  // namespace demdl::mesh
