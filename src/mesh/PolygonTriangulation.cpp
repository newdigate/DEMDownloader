#include "mesh/PolygonTriangulation.h"

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <limits>
#include <numeric>
#include <set>
#include <stdexcept>
#include <string>

namespace demdl::mesh {

namespace {

std::string describe(GridPoint p) { return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

// Throws when two vertices of the loop are the same point. It names the repeated point that comes first by column and
// then by row, and the two lowest indices at it.
void refuseRepeatedVertex(std::span<const GridPoint> loop) {
  std::vector<std::uint32_t> order(loop.size());
  std::iota(order.begin(), order.end(), std::uint32_t{0});
  // By column, then row, then index: the copies of a point end up side by side, the lowest index first.
  std::sort(order.begin(), order.end(), [loop](std::uint32_t a, std::uint32_t b) {
    if (loop[a].x != loop[b].x) return loop[a].x < loop[b].x;
    if (loop[a].y != loop[b].y) return loop[a].y < loop[b].y;
    return a < b;
  });
  for (std::size_t i = 1; i < order.size(); ++i) {
    const GridPoint p = loop[order[i - 1]];
    const GridPoint q = loop[order[i]];
    if (p.x == q.x && p.y == q.y) {
      throw std::invalid_argument("the polygon repeats the vertex " + describe(p) + ", at indices " +
                                  std::to_string(order[i - 1]) + " and " + std::to_string(order[i]));
    }
  }
}

// Twice the polygon's signed area: the first vertex and each side not at it make a triangle, a fan, and orient2d is
// twice that triangle's signed area. The fan's areas add up to the polygon's whether it is simple or not, positive for
// a counter-clockwise one, and the sum is exact (every term is at most 2^30 in size).
std::int64_t doubledArea(std::span<const GridPoint> loop) {
  std::int64_t sum = 0;
  for (std::size_t i = 1; i + 1 < loop.size(); ++i) sum += orient2d(loop[0], loop[i], loop[i + 1]);
  return sum;
}

// Whether p is in the counter-clockwise triangle a, b, c or on one of its edges, corners included: on the left of each
// side, or on its line.
bool inClosedTriangle(GridPoint a, GridPoint b, GridPoint c, GridPoint p) noexcept {
  return orient2d(a, b, p) >= 0 && orient2d(b, c, p) >= 0 && orient2d(c, a, p) >= 0;
}

// The vertices still in the polygon, as a circular doubly linked list in loop order: a vertex's predecessor and
// successor are the nearest remaining vertices before and after it in the loop, with the last wrapping round to the
// first. Cutting a vertex off takes it out in a constant number of steps.
class Remaining {
 public:
  explicit Remaining(std::span<const GridPoint> loop) : loop_(loop), before_(loop.size()), after_(loop.size()) {
    const std::size_t n = loop.size();
    for (std::size_t v = 0; v < n; ++v) {
      before_[v] = static_cast<std::uint32_t>(v == 0 ? n - 1 : v - 1);
      after_[v] = static_cast<std::uint32_t>(v + 1 == n ? 0 : v + 1);
    }
  }

  [[nodiscard]] std::uint32_t after(std::uint32_t v) const noexcept { return after_[v]; }
  [[nodiscard]] std::uint32_t before(std::uint32_t v) const noexcept { return before_[v]; }

  // Whether v is an ear: it turns counter-clockwise strictly, and no other remaining vertex is in or on the triangle
  // of its predecessor, itself and its successor. A vertex in line with its neighbours has a triangle without area
  // and is not an ear; a reflex one turns the other way.
  [[nodiscard]] bool isEar(std::uint32_t v) const {
    const std::uint32_t u = before_[v];
    const std::uint32_t w = after_[v];
    const GridPoint a = loop_[u], b = loop_[v], c = loop_[w];
    if (orient2d(a, b, c) <= 0) return false;
    // Every remaining vertex but u, v and w: from the successor of w round to the predecessor of u.
    for (std::uint32_t k = after_[w]; k != u; k = after_[k]) {
      if (inClosedTriangle(a, b, c, loop_[k])) return false;
    }
    return true;
  }

  // Takes v out of the polygon.
  void cut(std::uint32_t v) noexcept {
    const std::uint32_t u = before_[v];
    const std::uint32_t w = after_[v];
    after_[u] = w;
    before_[w] = u;
  }

 private:
  std::span<const GridPoint> loop_;
  std::vector<std::uint32_t> before_, after_;
};

}  // namespace

std::vector<std::array<std::uint32_t, 3>> triangulatePolygon(std::span<const GridPoint> loop) {
  if (loop.size() < 3) {
    throw std::invalid_argument("a polygon needs at least 3 vertices, not " + std::to_string(loop.size()));
  }
  // With every coordinate inside the predicates' bound a loop of distinct points has at most (2 x 16384 + 1)^2 =
  // 1,073,807,361 vertices, so the indices fit in 32 bits. A longer loop breaks the bound or repeats a vertex.
  assert(loop.size() <= std::numeric_limits<std::uint32_t>::max() && "the indices are 32 bits, and the loop is longer");
  refuseRepeatedVertex(loop);
  const std::int64_t twiceArea = doubledArea(loop);
  if (twiceArea < 0) {
    throw std::invalid_argument("the polygon is clockwise: twice its signed area is " + std::to_string(twiceArea) +
                                "; its vertices must run counter-clockwise");
  }
  if (twiceArea == 0) {
    throw std::invalid_argument("the polygon has no area: twice its signed area is 0");
  }

  // The ears are found once and kept, in a set that puts the lowest index first, and a cut tests again only the two
  // vertices whose neighbours it changes. For a simple polygon that is all it takes. An ear stays an ear while its two
  // neighbours stay, as its triangle only loses vertices. And cutting an ear v makes an ear of no vertex but its own
  // neighbours: any other vertex x keeps its triangle, and x could be freed only if v were the one vertex in it. Then
  // nothing of the boundary is inside that triangle but the two edges at v, which can leave it only through the side
  // opposite x. The corner at x is inside the polygon, so the part of the triangle between those edges and that side is
  // outside it, and the polygon holds a half turn or more at v: v would be reflex or in line, and an ear is neither.
  // So the set is always the ears there are, and its first member is what a search from the lowest index would find.
  // (A polygon that crosses itself can break that, and what is cut from it is not specified in any case.)
  Remaining remaining(loop);
  std::set<std::uint32_t> ears;
  for (std::uint32_t v = 0; v < loop.size(); ++v) {
    if (remaining.isEar(v)) ears.insert(ears.end(), v);
  }

  std::vector<std::array<std::uint32_t, 3>> triangles;
  triangles.reserve(loop.size() - 2);
  for (std::size_t left = loop.size(); left >= 3; --left) {
    if (ears.empty()) {
      throw std::invalid_argument("no ear can be found among the " + std::to_string(left) +
                                  " vertices that remain: the polygon crosses itself");
    }
    const std::uint32_t v = *ears.begin();
    ears.erase(ears.begin());
    const std::uint32_t u = remaining.before(v);
    const std::uint32_t w = remaining.after(v);
    triangles.push_back({u, v, w});
    remaining.cut(v);
    if (left == 3) break;  // two vertices are left, and nothing more to test
    for (const std::uint32_t x : {u, w}) {
      if (remaining.isEar(x)) {
        ears.insert(x);
      } else {
        ears.erase(x);
      }
    }
  }
  return triangles;
}

}  // namespace demdl::mesh
