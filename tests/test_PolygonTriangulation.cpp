#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mesh/PolygonTriangulation.h"
#include "mesh/Predicates.h"

using Catch::Matchers::Message;
using demdl::mesh::GridPoint;
using demdl::mesh::kMostPredicateCoordinate;
using demdl::mesh::orient2d;
using demdl::mesh::triangulatePolygon;

// Every expected triple is worked out beside its case from the rule in the header: of the vertices still in the
// polygon, the one with the lowest index that is an ear is cut off. An ear turns counter-clockwise strictly (orient2d
// of its neighbours and itself is positive) and its triangle with its two neighbours holds no other remaining vertex,
// inside or on an edge. The triple is (the vertex before it, the vertex, the vertex after it) in the polygon as it
// then stands. The cases are also held to the contract itself, whichever ears the rule picks, by firstProblem below:
// the right number of triangles, each counter-clockwise, tiling the polygon exactly, with no vertex left on a
// triangle's edge or inside it.

namespace {

using Loop = std::vector<GridPoint>;
using Triangle = std::array<std::uint32_t, 3>;
using Triangles = std::vector<Triangle>;
using Edge = std::pair<std::uint32_t, std::uint32_t>;

std::string text(GridPoint p) { return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

std::string text(const Loop& loop) {
  std::string out;
  for (const GridPoint& p : loop) out += (out.empty() ? "" : " ") + text(p);
  return out;
}

std::string text(const Triangle& t) {
  return "(" + std::to_string(t[0]) + ", " + std::to_string(t[1]) + ", " + std::to_string(t[2]) + ")";
}

std::string text(const Edge& e) { return std::to_string(e.first) + "->" + std::to_string(e.second); }

// Twice the polygon's signed area, by the shoelace formula: the sum over the sides of x_k y_(k+1) - x_(k+1) y_k. It
// takes a road of its own, not orient2d, so that the areas of the triangles are weighed against something else.
std::int64_t doubledArea(const Loop& loop) {
  std::int64_t sum = 0;
  for (std::size_t k = 0; k < loop.size(); ++k) {
    const GridPoint a = loop[k];
    const GridPoint b = loop[(k + 1) % loop.size()];
    sum += std::int64_t{a.x} * b.y - std::int64_t{b.x} * a.y;
  }
  return sum;
}

// Whether p is inside the counter-clockwise triangle a, b, c or on one of its edges, corners included.
bool onOrInside(GridPoint a, GridPoint b, GridPoint c, GridPoint p) {
  return orient2d(a, b, p) >= 0 && orient2d(b, c, p) >= 0 && orient2d(c, a, p) >= 0;
}

// Says what is wrong with triangles as a triangulation of the polygon, or returns "" when nothing is: n - 2 triangles
// of valid indices, each counter-clockwise; the doubled areas add up to the polygon's exactly; each side (k, k + 1) of
// the polygon is walked once, in its direction, by a triangle and never the other way; every other edge of a triangle
// is walked once each way, by the two triangles that share it; and no vertex lies inside a triangle or on one of its
// edges unless it is one of the triangle's three corners.
std::string firstProblem(const Loop& loop, const Triangles& triangles) {
  const std::size_t n = loop.size();
  if (triangles.size() != n - 2) {
    return std::to_string(triangles.size()) + " triangles for " + std::to_string(n) + " vertices, not " +
           std::to_string(n - 2);
  }

  std::int64_t areaSum = 0;
  for (const Triangle& t : triangles) {
    for (const std::uint32_t index : t) {
      if (index >= n) return "the triangle " + text(t) + " names a vertex a polygon of " + std::to_string(n) + " lacks";
    }
    const std::int64_t twice = orient2d(loop[t[0]], loop[t[1]], loop[t[2]]);
    if (twice <= 0) {
      return "the triangle " + text(t) + " is not counter-clockwise: twice its area is " + std::to_string(twice);
    }
    areaSum += twice;
  }
  if (areaSum != doubledArea(loop)) {
    return "twice the triangles' areas add up to " + std::to_string(areaSum) + ", the polygon's is " +
           std::to_string(doubledArea(loop));
  }

  std::map<Edge, int> walked;  // how many triangles walk each edge in each direction
  for (const Triangle& t : triangles) {
    for (std::size_t i = 0; i < 3; ++i) ++walked[Edge{t[i], t[(i + 1) % 3]}];
  }
  std::set<Edge> sides;
  for (std::size_t k = 0; k < n; ++k) {
    const Edge side{static_cast<std::uint32_t>(k), static_cast<std::uint32_t>((k + 1) % n)};
    sides.insert(side);
    if (!walked.contains(side)) return "the side " + text(side) + " of the polygon is in no triangle";
  }
  for (const auto& [edge, count] : walked) {
    const auto opposite = walked.find(Edge{edge.second, edge.first});
    const int against = opposite == walked.end() ? 0 : opposite->second;
    if (sides.contains(edge)) {
      if (count != 1) return "the side " + text(edge) + " of the polygon is in " + std::to_string(count) + " triangles";
      if (against != 0) return "the side " + text(edge) + " of the polygon is also walked the other way";
    } else if (count != 1 || against != 1) {
      return "the edge " + text(edge) + " is walked " + std::to_string(count) + " times that way and " +
             std::to_string(against) + " the other, not once each way";
    }
  }

  for (const Triangle& t : triangles) {
    for (std::uint32_t k = 0; k < n; ++k) {
      if (k == t[0] || k == t[1] || k == t[2]) continue;
      if (onOrInside(loop[t[0]], loop[t[1]], loop[t[2]], loop[k])) {
        return "the vertex " + std::to_string(k) + " " + text(loop[k]) + " is inside the triangle " + text(t) +
               " or on one of its edges";
      }
    }
  }
  return {};
}

void checkTriangulation(const Loop& loop, const Triangles& triangles) {
  const std::string problem = firstProblem(loop, triangles);
  INFO("polygon: " << text(loop));
  INFO("problem: " << problem);
  CHECK(problem.empty());
}

// The same polygon started at another vertex.
Loop startingAt(const Loop& loop, std::size_t start) {
  Loop out = loop;
  std::rotate(out.begin(), out.begin() + static_cast<std::ptrdiff_t>(start), out.end());
  return out;
}

// The polygon through the corners, in order, with every lattice point on its sides as a vertex. Its sides run along the
// axes, which is what the unit steps need.
Loop withLatticePoints(const Loop& corners) {
  Loop loop;
  for (std::size_t k = 0; k < corners.size(); ++k) {
    const GridPoint from = corners[k];
    const GridPoint to = corners[(k + 1) % corners.size()];
    const std::int32_t stepX = (to.x > from.x) - (to.x < from.x);
    const std::int32_t stepY = (to.y > from.y) - (to.y < from.y);
    for (GridPoint p = from; p.x != to.x || p.y != to.y; p.x += stepX, p.y += stepY) loop.push_back(p);
  }
  return loop;
}

// The test's own tracer of a set of unit cells (the cell (x, y) is the square from (x, y) to (x + 1, y + 1)), written
// apart from the production tracer that a later unit adds, so that the two cannot share a mistake.
using Point = std::pair<int, int>;
using Cells = std::set<Point>;

// The four steps from a cell to the cells that share a side with it.
const std::array<Point, 4> kSteps{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};

// The cells and every cell they enclose. An empty cell is outside when a walk through empty cells, one side at a time,
// gets to it from beyond the cells' bounding box; the rest are filled in. The set that results is 4-connected when the
// cells are, has no hole, and touches itself at no corner: two of its cells that meet diagonally with the two cells
// between them empty would leave one of those two reachable only through the corner, and so filled in.
Cells withEnclosedFilled(const Cells& cells) {
  int lowX = cells.begin()->first, highX = lowX, lowY = cells.begin()->second, highY = lowY;
  for (const auto& [x, y] : cells) {
    lowX = std::min(lowX, x);
    highX = std::max(highX, x);
    lowY = std::min(lowY, y);
    highY = std::max(highY, y);
  }
  // The walk may go one cell beyond the box, which is empty all round, so it gets everywhere outside.
  Cells outside{{lowX - 1, lowY - 1}};
  std::vector<Point> pending{{lowX - 1, lowY - 1}};
  while (!pending.empty()) {
    const Point at = pending.back();
    pending.pop_back();
    for (const Point& step : kSteps) {
      const Point to{at.first + step.first, at.second + step.second};
      if (to.first < lowX - 1 || to.first > highX + 1 || to.second < lowY - 1 || to.second > highY + 1) continue;
      if (cells.contains(to) || !outside.insert(to).second) continue;
      pending.push_back(to);
    }
  }
  Cells filled = cells;
  for (int x = lowX; x <= highX; ++x) {
    for (int y = lowY; y <= highY; ++y) {
      if (!outside.contains({x, y})) filled.insert({x, y});
    }
  }
  return filled;
}

// The outline of a set of cells that is 4-connected, has no hole and touches itself at no corner, as lattice points
// counter-clockwise (y up) with every lattice point on it a vertex, starting at the leftmost and of those the lowest.
// Each side of a cell that has no cell of the set beyond it is a boundary edge, walked with the cell on its left, and
// the edges join up into one loop.
Loop outline(const Cells& cells) {
  std::map<Point, Point> next;  // the end of the boundary edge that starts at each point
  const auto add = [&next](Point from, Point to) {
    const bool fresh = next.emplace(from, to).second;
    REQUIRE(fresh);  // a second edge from one point is a corner where the outline touches itself
  };
  for (const auto& [x, y] : cells) {
    if (!cells.contains({x, y - 1})) add({x, y}, {x + 1, y});          // the cell's bottom, walked east
    if (!cells.contains({x + 1, y})) add({x + 1, y}, {x + 1, y + 1});  // its right side, north
    if (!cells.contains({x, y + 1})) add({x + 1, y + 1}, {x, y + 1});  // its top, west
    if (!cells.contains({x - 1, y})) add({x, y + 1}, {x, y});          // its left side, south
  }
  Loop loop;
  const Point start = next.begin()->first;
  Point at = start;
  do {
    loop.push_back({at.first, at.second});
    at = next.at(at);
  } while (at != start);
  REQUIRE(loop.size() == next.size());  // one loop took in every boundary edge
  return loop;
}

// A 4-connected set of between 1 and 60 cells inside the box [0, 12) x [0, 12), grown from a seed cell by random steps:
// a cell already in the set and a direction are drawn, and the cell beside it joins unless it is in the set or beyond
// the box. The engine's output is fixed by the standard and a distribution's is not, so every draw is made from the
// engine's output by hand, as the other tests do.
Cells randomCells(std::mt19937& rng) {
  constexpr int kBox = 12;
  const std::size_t target = 1 + rng() % 60;
  const Point seed{static_cast<int>(rng() % kBox), static_cast<int>(rng() % kBox)};
  Cells cells{seed};
  std::vector<Point> grown{seed};
  while (cells.size() < target) {
    const Point from = grown[rng() % grown.size()];
    const Point step = kSteps[rng() % kSteps.size()];
    const Point to{from.first + step.first, from.second + step.second};
    if (to.first < 0 || to.first >= kBox || to.second < 0 || to.second >= kBox) continue;
    if (cells.insert(to).second) grown.push_back(to);
  }
  return cells;
}

// A corridor one cell wide that winds out from (0, 0) in a square spiral with an empty row or column between its arms:
// the segments run east, north, west, south and so on for 2, 2, 4, 4, 6, 6, ... cells.
Cells spiral(int segments) {
  Cells cells{{0, 0}};
  Point at{0, 0};
  Point step{1, 0};
  for (int s = 0; s < segments; ++s) {
    for (int i = 0; i < 2 * (s / 2 + 1); ++i) {
      at = {at.first + step.first, at.second + step.second};
      cells.insert(at);
    }
    step = {-step.second, step.first};  // a left turn: east to north, north to west, and so on
  }
  return cells;
}

}  // namespace

TEST_CASE("a polygon of three vertices is one triangle", "[polygon]") {
  // The first ear in index order is vertex 0, with the vertex before it, 2, and the vertex after it, 1: the triple
  // (2, 0, 1), counter-clockwise because orient2d((0,3), (0,0), (3,0)) = (0,-3) x (3,-3) = 9 is positive.
  const Loop triangle{{0, 0}, {3, 0}, {0, 3}};
  const Triangles expected{{2, 0, 1}};
  CHECK(triangulatePolygon(triangle) == expected);
  checkTriangulation(triangle, triangulatePolygon(triangle));
}

TEST_CASE("a square of two by two is two triangles, cut at the first ear in index order", "[polygon]") {
  // All four corners turn counter-clockwise and the lowest index is 0, with the neighbours 3 = (0,2) and 1 = (2,0).
  // The triangle (0,2), (0,0), (2,0) has twice its area 4, and the other corner, (2,2), is out of it: on the far side
  // of the edge from (2,0) to (0,2) (orient2d = (-2,2) x (0,2) = -4). So vertex 0 is an ear and (3, 0, 1) is cut off.
  // What is left, 1, 2, 3, is a triangle, and its first vertex is 1: (3, 1, 2).
  const Loop square{{0, 0}, {2, 0}, {2, 2}, {0, 2}};
  const Triangles expected{{3, 0, 1}, {3, 1, 2}};
  const Triangles triangles = triangulatePolygon(square);
  CHECK(triangles == expected);
  checkTriangulation(square, triangles);
}

TEST_CASE("a square with a vertex at every lattice point on its sides is six triangles", "[polygon]") {
  // The corners (0,0), (2,0), (2,2), (0,2) and the middles of the sides: 0 (0,0), 1 (1,0), 2 (2,0), 3 (2,1), 4 (2,2),
  // 5 (1,2), 6 (0,2), 7 (0,1). To begin with every middle is in line with its neighbours, so only 0, 2, 4 and 6 can
  // be ears.
  // - 0, with 7 and 1, is a corner of a triangle of twice its area 1 that nothing else is in or on: cut (7, 0, 1).
  // - 1 now has the neighbours 7 and 2 and turns counter-clockwise (orient2d((0,1), (1,0), (2,0)) = 1), and of the rest
  //   (2,1) is the nearest to its triangle and is beyond the edge from (2,0) to (0,1) (-2): cut (7, 1, 2).
  // - 2, with 7 and 3, then 3, with 7 and 4, then 4, with 7 and 5, then 5, with 7 and 6, each the lowest index
  //   that is an ear and each with nothing but the rest of the fan beyond its far edge: (7, 2, 3), (7, 3, 4), (7, 4, 5)
  //   and (7, 5, 6).
  // Seven is a middle and is never cut off, so it is the apex of a fan. The twice areas are 1, 1, 2, 2, 1, 1: 8, the
  // square's 2 x 4.
  const Loop square = withLatticePoints({{0, 0}, {2, 0}, {2, 2}, {0, 2}});
  REQUIRE(text(square) == "(0, 0) (1, 0) (2, 0) (2, 1) (2, 2) (1, 2) (0, 2) (0, 1)");
  const Triangles expected{{7, 0, 1}, {7, 1, 2}, {7, 2, 3}, {7, 3, 4}, {7, 4, 5}, {7, 5, 6}};
  const Triangles triangles = triangulatePolygon(square);
  CHECK(triangles == expected);
  checkTriangulation(square, triangles);

  for (std::size_t start = 0; start < square.size(); ++start) {
    INFO("started at vertex " << start);
    const Loop rotated = startingAt(square, start);
    checkTriangulation(rotated, triangulatePolygon(rotated));
  }
}

TEST_CASE("an L-shape is four triangles, a corner waiting while a vertex is in or on its triangle", "[polygon]") {
  // 0 (0,0), 1 (4,0), 2 (4,2), 3 (2,2), 4 (2,4), 5 (0,4); 3 is the one reflex vertex. Twice the area is 24.
  // - 0, with 5 and 1, turns counter-clockwise (16) but its triangle (0,4), (0,0), (4,0) has 3 = (2,2) exactly on its
  //   long edge, from (4,0) to (0,4): orient2d = (-4,4) x (-2,2) = 0. It is blocked, though 3 is not inside.
  // - 1, with 0 and 2: the triangle (0,0), (4,0), (4,2) has nothing in it or on it, 3 being beyond the edge from (4,2)
  //   to (0,0) (-4). Cut (0, 1, 2).
  // - 0 now has the neighbours 5 and 2: its triangle (0,4), (0,0), (4,2) has 3 = (2,2) strictly inside. Blocked. The
  //   next in index order, 2, has the neighbours 0 and 3: the triangle (0,0), (4,2), (2,2) has nothing in it. Cut
  //   (0, 2, 3).
  // - 0 has the neighbours 5 and 3: the triangle (0,4), (0,0), (2,2) has nothing in it. Cut (5, 0, 3).
  // - What is left, 3, 4, 5, is a triangle, and its first vertex is 3: (5, 3, 4).
  // The twice areas, 8, 4, 8, 4, add up to 24.
  const Loop corners{{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}};
  const Triangles expected{{0, 1, 2}, {0, 2, 3}, {5, 0, 3}, {5, 3, 4}};
  const Triangles triangles = triangulatePolygon(corners);
  CHECK(triangles == expected);
  checkTriangulation(corners, triangles);
  CHECK(doubledArea(corners) == 24);
}

TEST_CASE("an L-shape with a vertex at every lattice point on its sides triangulates from every first vertex",
          "[polygon]") {
  // 16 vertices around the L, so 14 triangles, wherever the loop starts.
  const Loop sixteen = withLatticePoints({{0, 0}, {4, 0}, {4, 2}, {2, 2}, {2, 4}, {0, 4}});
  REQUIRE(sixteen.size() == 16);
  for (std::size_t start = 0; start < sixteen.size(); ++start) {
    INFO("started at vertex " << start);
    const Loop rotated = startingAt(sixteen, start);
    const Triangles triangles = triangulatePolygon(rotated);
    CHECK(triangles.size() == 14);
    checkTriangulation(rotated, triangles);
  }
}

TEST_CASE("a staircase coast with a vertex at every lattice point is twelve triangles of half a unit", "[polygon]") {
  // The cells (0,0) (1,0) (1,1) (2,1) (2,2) (3,2), each beside the next: an outline of 4 x 6 - 2 x 5 = 14 lattice
  // points, and none inside it. Every lattice point the polygon holds is then a vertex, so by Pick's theorem a triangle
  // on vertices alone, with no other vertex in it or on it, has no lattice point but its corners and an area of 1/2.
  // Twelve triangles of twice the area 1 make the 12 of six cells.
  const Loop coast = outline({{0, 0}, {1, 0}, {1, 1}, {2, 1}, {2, 2}, {3, 2}});
  REQUIRE(text(coast) ==
          "(0, 0) (1, 0) (2, 0) (2, 1) (3, 1) (3, 2) (4, 2) (4, 3) (3, 3) (2, 3) (2, 2) (1, 2) (1, 1) (0, 1)");
  CHECK(doubledArea(coast) == 12);

  for (std::size_t start = 0; start < coast.size(); ++start) {
    INFO("started at vertex " << start);
    const Loop rotated = startingAt(coast, start);
    const Triangles triangles = triangulatePolygon(rotated);
    CHECK(triangles.size() == 12);
    checkTriangulation(rotated, triangles);
    int notHalfAUnit = 0;
    for (const Triangle& t : triangles) {
      if (orient2d(rotated[t[0]], rotated[t[1]], rotated[t[2]]) != 1) ++notHalfAUnit;
    }
    CHECK(notHalfAUnit == 0);
  }
}

TEST_CASE("a vertex on the long edge of a corner's triangle stops that corner being cut", "[polygon]") {
  // The triangle (0,0), (2,0), (2,2) with a fourth vertex (1,1) on its edge from (2,2) back to (0,0), in the order
  // (2,0), (2,2), (1,1), (0,0): vertex 0 = (2,0) turns counter-clockwise (4) with its neighbours 3 = (0,0) and
  // 1 = (2,2), and vertex 2 = (1,1) is on the third edge (orient2d = (-2,-2) x (-1,-1) = 0). Cutting 0 off would leave
  // (1,1) on the edge of a triangle, a T-junction, so it is blocked. Vertex 1 = (2,2), with 0 and 2, is clear ((0,0)
  // is beyond the edge from (1,1) to (2,0): -2): cut (0, 1, 2). Then 0, with 3 and 2: cut (3, 0, 2). Vertex 2, in line
  // with its neighbours to begin with, is a corner of both triangles.
  const Loop loop{{2, 0}, {2, 2}, {1, 1}, {0, 0}};
  const Triangles expected{{0, 1, 2}, {3, 0, 2}};
  const Triangles triangles = triangulatePolygon(loop);
  CHECK(triangles == expected);
  checkTriangulation(loop, triangles);

  // The same on the largest square the predicates are exact for: (m, -m), (m, m), (0, 0), (-m, -m), with (0, 0) on the
  // diagonal between (m, m) and (-m, -m). Each turn is 2 m^2 or more, and (0, 0) sits exactly on an edge of 4 m^2.
  constexpr std::int32_t m = kMostPredicateCoordinate;
  const Loop wide{{m, -m}, {m, m}, {0, 0}, {-m, -m}};
  const Triangles wideTriangles = triangulatePolygon(wide);
  CHECK(wideTriangles == expected);
  checkTriangulation(wide, wideTriangles);
}

TEST_CASE("polygons with slanted sides are triangulated too", "[polygon]") {
  struct Shape {
    const char* name;
    Loop loop;
  };
  const std::vector<Shape> shapes{
      // Convex, with (2,1) in line between (0,0) and (4,2) on a slanted side.
      {"a convex pentagon with a vertex in line on a slanted side", {{0, 0}, {2, 1}, {4, 2}, {3, 4}, {0, 3}}},
      // Reflex at (1,2) only.
      {"a dart", {{0, 0}, {4, 2}, {0, 4}, {1, 2}}},
      // The corners of a 10 x 10 square, each side dented one unit inwards at its middle: four reflex vertices.
      {"a four-pointed star", {{0, 0}, {5, 1}, {10, 0}, {9, 5}, {10, 10}, {5, 9}, {0, 10}, {1, 5}}},
      // A slot two wide and four deep in a 6 x 6 square, open at the top.
      {"a U", {{0, 0}, {6, 0}, {6, 6}, {4, 6}, {4, 2}, {2, 2}, {2, 6}, {0, 6}}},
  };
  for (const Shape& shape : shapes) {
    REQUIRE(doubledArea(shape.loop) > 0);
    for (std::size_t start = 0; start < shape.loop.size(); ++start) {
      INFO(shape.name << " started at vertex " << start);
      const Loop rotated = startingAt(shape.loop, start);
      checkTriangulation(rotated, triangulatePolygon(rotated));
    }
  }
}

TEST_CASE("polygons across the whole bound are triangulated exactly", "[polygon]") {
  constexpr std::int32_t m = kMostPredicateCoordinate;

  // An L: the square from (-m, -m) to (m, m) without its north-east quarter. Twice its area is 6 m^2 = 6 x 2^28, and
  // 3 is the one reflex vertex.
  const Loop ell{{-m, -m}, {m, -m}, {m, 0}, {0, 0}, {0, m}, {-m, m}};
  REQUIRE(doubledArea(ell) == 6 * std::int64_t{m} * m);
  for (std::size_t start = 0; start < ell.size(); ++start) {
    INFO("the L started at vertex " << start);
    const Loop rotated = startingAt(ell, start);
    checkTriangulation(rotated, triangulatePolygon(rotated));
  }

  // A dart whose reflex vertex (1, 0) is as near to the long diagonal from (-m, -m) to (m, m) as the grid allows:
  // orient2d((-m,-m), (m,m), (1,0)) = -2 m. The notch it cuts out of the triangle (-m,-m), (m,-m), (m,m) is a sliver
  // of twice the area 2 m, and what is left is two triangles of about 2 m^2. The corner (m, -m) cannot be cut while
  // (1, 0) is in its triangle.
  const Loop dart{{-m, -m}, {m, -m}, {m, m}, {1, 0}};
  REQUIRE(orient2d({-m, -m}, {m, m}, {1, 0}) == -2 * std::int64_t{m});
  for (std::size_t start = 0; start < dart.size(); ++start) {
    INFO("the dart started at vertex " << start);
    const Loop rotated = startingAt(dart, start);
    checkTriangulation(rotated, triangulatePolygon(rotated));
  }
}

TEST_CASE("500 random rectilinear polygons are triangulated exactly", "[polygon]") {
  // Each polygon is the outer outline of a random 4-connected set of cells, with a vertex at every lattice point on
  // it (concave outlines with straight runs of every length, which is what the unit is for), shifted by (-6, -6) so
  // that coordinates are negative too, and started at a random vertex.
  std::mt19937 rng(7);
  int concave = 0, filledIn = 0, mostVertices = 0, fewestVertices = 1000;
  for (int i = 0; i < 500; ++i) {
    const Cells grown = randomCells(rng);
    const Cells cells = withEnclosedFilled(grown);
    if (cells.size() > grown.size()) ++filledIn;
    Loop loop = outline(cells);
    // The tracer is held to the cells: a loop of twice their count in area is their outline, run counter-clockwise.
    REQUIRE(doubledArea(loop) == 2 * static_cast<std::int64_t>(cells.size()));
    for (GridPoint& p : loop) p = {p.x - 6, p.y - 6};
    loop = startingAt(loop, rng() % loop.size());

    INFO("polygon " << i << " of " << cells.size() << " cells");
    const Triangles triangles = triangulatePolygon(loop);
    checkTriangulation(loop, triangles);

    for (std::size_t k = 0; k < loop.size(); ++k) {
      if (orient2d(loop[(k + loop.size() - 1) % loop.size()], loop[k], loop[(k + 1) % loop.size()]) < 0) {
        ++concave;
        break;
      }
    }
    mostVertices = std::max(mostVertices, static_cast<int>(loop.size()));
    fewestVertices = std::min(fewestVertices, static_cast<int>(loop.size()));
  }
  // The sample is not one-sided: most outlines have a reflex corner, some had a hole to fill in, and the sizes run
  // from a single cell's 4 vertices up to polygons of dozens.
  CHECK(concave > 300);
  CHECK(filledIn > 5);
  CHECK(fewestVertices == 4);
  CHECK(mostVertices >= 60);
}

TEST_CASE("outlines of a couple of thousand vertices are triangulated exactly", "[polygon]") {
  // A staircase coast of 500 steps: 1000 cells, 4 x 1000 - 2 x 999 = 2002 lattice points on the outline and none inside
  // it, so every triangle is half a unit (twice its area 1, as for the small staircase) and there are 2000 of them.
  // The ears that are cut grow into triangles that span hundreds of steps, so the vertices along the far side have to
  // be ruled out again and again.
  Cells steps;
  for (int i = 0; i < 500; ++i) {
    steps.insert({i, i});
    steps.insert({i + 1, i});
  }
  const Loop coast = outline(steps);
  REQUIRE(coast.size() == 2002);
  const Triangles coastTriangles = triangulatePolygon(coast);
  CHECK(coastTriangles.size() == 2000);
  checkTriangulation(coast, coastTriangles);
  int notHalfAUnit = 0;
  for (const Triangle& t : coastTriangles) {
    if (orient2d(coast[t[0]], coast[t[1]], coast[t[2]]) != 1) ++notHalfAUnit;
  }
  CHECK(notHalfAUnit == 0);

  // A spiral corridor of 40 arms, 841 cells: 4 x 841 - 2 x 840 = 1684 lattice points, a reflex and a convex corner at
  // every turn, and a search from the lowest index that passes hundreds of vertices that cannot be ears at every step.
  const Loop winding = outline(spiral(40));
  REQUIRE(winding.size() == 1684);
  const Triangles windingTriangles = triangulatePolygon(winding);
  CHECK(windingTriangles.size() == 1682);
  checkTriangulation(winding, windingTriangles);
}

TEST_CASE("the same polygon gives the same triangles every time", "[polygon]") {
  const Loop coast = outline({{0, 0}, {1, 0}, {1, 1}, {2, 1}, {2, 2}, {3, 2}});
  const Triangles first = triangulatePolygon(coast);
  const Triangles second = triangulatePolygon(coast);
  CHECK(first == second);
  CHECK(first.size() == 12);

  // And a winding one: a spiral corridor of 10 arms, 61 cells, 4 x 61 - 2 x 60 = 124 vertices.
  const Loop winding = outline(spiral(10));
  REQUIRE(winding.size() == 124);
  CHECK(triangulatePolygon(winding) == triangulatePolygon(winding));
}

TEST_CASE("a loop that cannot be triangulated is refused", "[polygon]") {
  SECTION("fewer than three vertices") {
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{}), std::invalid_argument,
                         Message("a polygon needs at least 3 vertices, not 0"));
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}}), std::invalid_argument,
                         Message("a polygon needs at least 3 vertices, not 1"));
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}, {1, 0}}), std::invalid_argument,
                         Message("a polygon needs at least 3 vertices, not 2"));
  }

  SECTION("a clockwise loop") {
    // The square run the other way: the shoelace sum is -8.
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}, {0, 2}, {2, 2}, {2, 0}}), std::invalid_argument,
                         Message("the polygon is clockwise: twice its signed area is -8; its vertices must run "
                                 "counter-clockwise"));
  }

  SECTION("a repeated vertex") {
    // The square with its first vertex repeated at the end, the usual way to close a ring by mistake: indices 0 and 4.
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}, {2, 0}, {2, 2}, {0, 2}, {0, 0}}), std::invalid_argument,
                         Message("the polygon repeats the vertex (0, 0), at indices 0 and 4"));
    // Two triangles that meet at the origin, a butterfly drawn as one loop: counter-clockwise and of area, but (0, 0)
    // is visited twice, at indices 0 and 3.
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}, {2, -1}, {2, 1}, {0, 0}, {-2, 1}, {-2, -1}}),
                         std::invalid_argument, Message("the polygon repeats the vertex (0, 0), at indices 0 and 3"));
  }

  SECTION("a loop without area") {
    // Three vertices in line, and a bow tie whose two lobes cancel: its shoelace sum is 0 + -4 + 4 + 0.
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}, {1, 0}, {2, 0}}), std::invalid_argument,
                         Message("the polygon has no area: twice its signed area is 0"));
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}, {2, 2}, {2, 0}, {0, 2}}), std::invalid_argument,
                         Message("the polygon has no area: twice its signed area is 0"));
  }

  SECTION("a polygon that crosses itself and has area") {
    // A bow tie with unequal lobes, (0,0), (0,1), (3,0), (3,3): the sides from (0,1) to (3,0) and from (3,3) to (0,0)
    // cross at (3/4, 3/4). Twice its signed area is 6 and it passes the earlier checks. Vertices 0 and 1 turn
    // clockwise, and the first ear in index order is 2 = (3,0), with nothing in its triangle. Cutting it off leaves
    // (0,0), (0,1), (3,3), which is clockwise, so none of the three is an ear.
    CHECK_THROWS_MATCHES(triangulatePolygon(Loop{{0, 0}, {0, 1}, {3, 0}, {3, 3}}), std::invalid_argument,
                         Message("no ear can be found among the 3 vertices that remain: the polygon crosses itself"));
  }
}
