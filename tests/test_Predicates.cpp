#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include "mesh/Predicates.h"

using demdl::mesh::GridPoint;
using demdl::mesh::inCircle;
using demdl::mesh::kMostPredicateCoordinate;
using demdl::mesh::orient2d;

// Every expected number is worked out beside its case from the geometry: orient2d is twice the signed area of the
// triangle, and for a, b, c counter-clockwise inCircle is orient2d(a, b, c) x (R^2 - |d - O|^2), where O and R are the
// centre and the radius of the circle through a, b and c. The wide cases are also checked against a reference that
// takes another road: the determinant of the raw coordinates, with no differences to take, in 128 bits, where nothing
// can overflow.

namespace {

using Wide = __int128;  // the reference only: the library itself stays in 64 bits

// A point lifted onto the paraboloid z = x^2 + y^2.
struct Lifted {
  Wide x, y, z;
};

Lifted lift(GridPoint p) {
  const Wide x = p.x;
  const Wide y = p.y;
  return {x, y, x * x + y * y};
}

// The determinant of three rows, expanded along the first.
Wide det3(const Lifted& p, const Lifted& q, const Lifted& r) {
  return p.x * (q.y * r.z - q.z * r.y) - p.y * (q.x * r.z - q.z * r.x) + p.z * (q.x * r.y - q.y * r.x);
}

// The determinant of the rows (x, y, 1) of a, b and c, expanded along the column of ones.
Wide referenceOrient(GridPoint a, GridPoint b, GridPoint c) {
  const Wide ax = a.x, ay = a.y, bx = b.x, by = b.y, cx = c.x, cy = c.y;
  return (bx * cy - by * cx) - (ax * cy - ay * cx) + (ax * by - ay * bx);
}

// The determinant of the rows (x, y, x^2 + y^2, 1) of a, b, c and d, expanded along the column of ones. For a, b, c
// counter-clockwise it is positive when d lies inside their circle.
Wide referenceInCircle(GridPoint a, GridPoint b, GridPoint c, GridPoint d) {
  const Lifted la = lift(a), lb = lift(b), lc = lift(c), ld = lift(d);
  return det3(la, lb, lc) - det3(la, lb, ld) + det3(la, lc, ld) - det3(lb, lc, ld);
}

// Whether both predicates give the reference's value exactly. The comparison is in 128 bits, so a result that had
// wrapped round in 64 can only disagree.
bool agree(GridPoint a, GridPoint b, GridPoint c, GridPoint d) {
  return Wide{orient2d(a, b, c)} == referenceOrient(a, b, c) &&
         Wide{inCircle(a, b, c, d)} == referenceInCircle(a, b, c, d);
}

// How many quadruples have been compared with the reference, how many of them disagreed, and the first that did, for
// the failure message.
struct Tally {
  int checked = 0;
  int disagreements = 0;
  std::string first;

  void add(GridPoint a, GridPoint b, GridPoint c, GridPoint d) {
    ++checked;
    if (agree(a, b, c, d)) return;
    if (++disagreements == 1) {
      std::ostringstream out;
      for (const GridPoint& p : {a, b, c, d}) out << " (" << p.x << ", " << p.y << ")";
      first = "a, b, c, d =" + out.str();
    }
  }
};

}  // namespace

TEST_CASE("orient2d is twice the signed area: positive counter-clockwise, negative clockwise, 0 in line",
          "[predicates]") {
  constexpr std::int32_t m = kMostPredicateCoordinate;

  // (b - a) x (c - a): the legs 1 and 1 make 1, the legs 4 and 3 make 12, and moving the triangle changes nothing.
  CHECK(orient2d({0, 0}, {1, 0}, {0, 1}) == 1);
  CHECK(orient2d({0, 0}, {0, 1}, {1, 0}) == -1);
  CHECK(orient2d({0, 0}, {4, 0}, {0, 3}) == 12);
  CHECK(orient2d({-3, 2}, {1, 2}, {-3, 5}) == 12);

  // Three points on a line, steep, shallow, along an axis and through the origin, and with a point repeated.
  CHECK(orient2d({0, 0}, {1, 1}, {3, 3}) == 0);
  CHECK(orient2d({-2, 5}, {0, 4}, {4, 2}) == 0);
  CHECK(orient2d({5, 5}, {5, -7}, {5, 100}) == 0);
  CHECK(orient2d({7, 7}, {7, 7}, {1, 2}) == 0);

  // At the bound: half of the box as a triangle, 4 m^2 = 2^30 with the legs 2 m; two corners and the middle in line;
  // and a sliver, whose third point is one step off the line through the other two:
  // (2 m, 2 m) x (2 m, 2 m - 1) = 2 m (2 m - 1) - 2 m x 2 m = -2 m.
  CHECK(orient2d({-m, -m}, {m, -m}, {m, m}) == 4 * std::int64_t{m} * m);
  CHECK(orient2d({-m, -m}, {0, 0}, {m, m}) == 0);
  CHECK(orient2d({-m, -m}, {m, m}, {m, m - 1}) == -2 * std::int64_t{m});
}

TEST_CASE("orient2d changes sign under a swap of two points and keeps it under a rotation of the three",
          "[predicates]") {
  constexpr std::int32_t m = kMostPredicateCoordinate;
  struct Triple {
    GridPoint a, b, c;
  };
  const std::vector<Triple> triples{
      {{-5, 7}, {12, -3}, {4, 9}},  // (17, -10) x (9, 2) = 34 + 90 = 124
      {{0, 0}, {4, 0}, {0, 3}},     // 12
      {{-m, -m}, {m, -m}, {m, m}},  // the widest triangle: 4 m^2
      {{-3, -3}, {0, 0}, {5, 5}},   // in line, so 0 in every order
  };
  CHECK(orient2d(triples[0].a, triples[0].b, triples[0].c) == 124);

  for (const Triple& t : triples) {
    const std::int64_t twiceArea = orient2d(t.a, t.b, t.c);
    // The three swaps turn the triangle round.
    CHECK(orient2d(t.b, t.a, t.c) == -twiceArea);
    CHECK(orient2d(t.a, t.c, t.b) == -twiceArea);
    CHECK(orient2d(t.c, t.b, t.a) == -twiceArea);
    // The two rotations only start from another corner.
    CHECK(orient2d(t.b, t.c, t.a) == twiceArea);
    CHECK(orient2d(t.c, t.a, t.b) == twiceArea);
  }
}

TEST_CASE("inCircle is 0 on the circle, positive inside it and negative outside it", "[predicates]") {
  // The unit square's corners (0,0), (1,0), (1,1) lie on the circle of centre (1/2, 1/2) and R^2 = 1/2; orient2d is 1.
  CHECK(inCircle({0, 0}, {1, 0}, {1, 1}, {0, 1}) == 0);   // the fourth corner: on the circle
  CHECK(inCircle({0, 0}, {1, 0}, {1, 1}, {2, 2}) == -4);  // |d - O|^2 = 4.5: 1 x (0.5 - 4.5), outside

  // (0,0), (4,0), (0,4): centre (2,2), R^2 = 8, orient2d 16.
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {1, 1}) == 96);    // |d - O|^2 = 2: 16 x (8 - 2), inside
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {2, 2}) == 128);   // the centre: 16 x 8
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {4, 4}) == 0);     // the fourth corner of the square
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {5, 5}) == -160);  // |d - O|^2 = 18: 16 x (8 - 18), outside

  // A circle that is not a square's: centre (0,0), R^2 = 25, through (5,0), (0,5), (-5,0) counter-clockwise (orient2d
  // 50). The lattice points (3,4), (4,3), (-3,-4) and (0,-5) are on it, (3,3) is inside, (4,4) outside.
  CHECK(inCircle({5, 0}, {0, 5}, {-5, 0}, {3, 4}) == 0);
  CHECK(inCircle({5, 0}, {0, 5}, {-5, 0}, {4, 3}) == 0);
  CHECK(inCircle({5, 0}, {0, 5}, {-5, 0}, {-3, -4}) == 0);
  CHECK(inCircle({5, 0}, {0, 5}, {-5, 0}, {0, -5}) == 0);
  CHECK(inCircle({5, 0}, {0, 5}, {-5, 0}, {3, 3}) == 350);   // 50 x (25 - 18)
  CHECK(inCircle({5, 0}, {0, 5}, {-5, 0}, {4, 4}) == -350);  // 50 x (25 - 32)

  // d on a vertex of the triangle is on its circle.
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {0, 0}) == 0);
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {4, 0}) == 0);
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {0, 4}) == 0);

  // With a, b, c clockwise every sign is reversed: (0,0), (0,4), (4,0) is the triangle above the other way round.
  CHECK(inCircle({0, 0}, {0, 4}, {4, 0}, {1, 1}) == -96);
  CHECK(inCircle({0, 0}, {0, 4}, {4, 0}, {5, 5}) == 160);
}

TEST_CASE("inCircle changes sign when any two of its four points are swapped", "[predicates]") {
  // It is the determinant of four rows, one for each point, so a swap of two rows reverses it. A swap among a, b and c
  // turns the triangle round and the meaning of the sign with it; a swap with d is the one that checks that the
  // formula, which sets d apart as its origin, treats the four alike.
  constexpr std::int32_t m = kMostPredicateCoordinate;
  struct Quadruple {
    GridPoint a, b, c, d;
  };
  const std::vector<Quadruple> quadruples{
      {{0, 0}, {4, 0}, {0, 4}, {1, 1}},        // 96
      {{-5, 7}, {12, -3}, {4, 9}, {-2, 2}},    // an arbitrary one
      {{-m, -m}, {-m, m}, {0, 0}, {m, -m}},    // the widest
      {{0, 0}, {1, 0}, {1, 1}, {0, 1}},        // cocircular, 0 either way round
  };
  CHECK(inCircle({0, 0}, {4, 0}, {0, 4}, {1, 1}) == 96);

  for (const Quadruple& q : quadruples) {
    const std::int64_t value = inCircle(q.a, q.b, q.c, q.d);
    for (std::size_t i = 0; i < 4; ++i) {
      for (std::size_t j = i + 1; j < 4; ++j) {
        std::array<GridPoint, 4> p{{q.a, q.b, q.c, q.d}};
        std::swap(p[i], p[j]);
        CHECK(inCircle(p[0], p[1], p[2], p[3]) == -value);
      }
    }
  }
}

TEST_CASE("at the bound a square's corners are cocircular and one step inside is not, exactly", "[predicates]") {
  constexpr std::int32_t m = kMostPredicateCoordinate;
  const std::int64_t mm = std::int64_t{m} * m;  // m^2 = 2^28
  Tally tally;

  // The corners (0,0), (m,0), (m,m) and (0,m) of a square of side m: the circle has its centre at (m/2, m/2) and
  // R^2 = m^2 / 2, and orient2d(a, b, c) = m^2. The fourth corner is on the circle. The point (1, m - 1), one step in
  // from it, has |d - O|^2 = 2 (m/2 - 1)^2 = m^2 / 2 - 2 m + 2, so R^2 - |d - O|^2 = 2 (m - 1) and the determinant is
  // m^2 x 2 (m - 1).
  const GridPoint a{0, 0}, b{m, 0}, c{m, m};
  CHECK(orient2d(a, b, c) == mm);
  CHECK(inCircle(a, b, c, {0, m}) == 0);
  CHECK(inCircle(a, b, c, {1, m - 1}) == 2 * mm * (m - 1));
  tally.add(a, b, c, {0, m});
  tally.add(a, b, c, {1, m - 1});

  // The same square across the whole bound: (-m,-m), (m,-m), (m,m) and (-m,m), the circle centred on the origin with
  // R^2 = 2 m^2, and orient2d 4 m^2 = 2^30. Here the three products that the determinant is summed from are +2^60,
  // -2^61 and +2^60, and cancel exactly. The point (1 - m, m - 1), one step in from the fourth corner, has
  // |d - O|^2 = 2 (m - 1)^2, so R^2 - |d - O|^2 = 2 (2 m - 1) and the determinant is 4 m^2 x 2 (2 m - 1).
  const GridPoint wa{-m, -m}, wb{m, -m}, wc{m, m};
  CHECK(orient2d(wa, wb, wc) == 4 * mm);
  CHECK(inCircle(wa, wb, wc, {-m, m}) == 0);
  CHECK(inCircle(wa, wb, wc, {1 - m, m - 1}) == 8 * mm * (2 * m - 1));
  tally.add(wa, wb, wc, {-m, m});
  tally.add(wa, wb, wc, {1 - m, m - 1});

  // A determinant as large as the box allows, 2^59: three corners of the box and its middle. (-m,-m), (-m,m), (0,0) are
  // clockwise, orient2d = -2 m^2; their circle has its centre at (-m, 0) and R = m, and for (m,-m) |d - O|^2 = 5 m^2,
  // so R^2 - |d - O|^2 = -4 m^2 and the determinant is 8 m^4.
  CHECK(orient2d({-m, -m}, {-m, m}, {0, 0}) == -2 * mm);
  CHECK(inCircle({-m, -m}, {-m, m}, {0, 0}, {m, -m}) == 8 * mm * mm);
  tally.add({-m, -m}, {-m, m}, {0, 0}, {m, -m});

  INFO("first disagreement: " << tally.first);
  CHECK(tally.disagreements == 0);
}

TEST_CASE("1000 random quadruples across the whole bound agree with the reference", "[predicates]") {
  // The engine's output is fixed by the standard and a distribution's is not, so each coordinate, from -m to m, is
  // made from the engine's output by hand, as the other tests do.
  std::mt19937 rng(3);
  const auto point = [&rng] {
    const std::int32_t x = static_cast<std::int32_t>(rng() % 32769u) - kMostPredicateCoordinate;
    const std::int32_t y = static_cast<std::int32_t>(rng() % 32769u) - kMostPredicateCoordinate;
    return GridPoint{x, y};
  };

  Tally tally;
  int counterClockwise = 0, clockwise = 0, inside = 0, outside = 0;
  for (int i = 0; i < 1000; ++i) {
    const GridPoint a = point();
    const GridPoint b = point();
    const GridPoint c = point();
    const GridPoint d = point();
    tally.add(a, b, c, d);
    const std::int64_t turn = orient2d(a, b, c);
    if (turn > 0) ++counterClockwise;
    if (turn < 0) ++clockwise;
    // Only for a counter-clockwise triangle does the sign of inCircle mean inside or outside.
    if (turn > 0 && inCircle(a, b, c, d) > 0) ++inside;
    if (turn > 0 && inCircle(a, b, c, d) < 0) ++outside;
  }

  INFO("first disagreement: " << tally.first);
  CHECK(tally.checked == 1000);
  CHECK(tally.disagreements == 0);
  // The sample is not one-sided: both turns, and points both inside and outside, came up.
  CHECK(counterClockwise > 100);
  CHECK(clockwise > 100);
  CHECK(inside > 100);
  CHECK(outside > 100);
}

TEST_CASE("the corners, middles and near-middle points of the widest box all agree with the reference",
          "[predicates]") {
  // Five values on each axis make 25 points and 390,625 quadruples, repeated points among them: the box's corners and
  // the middles of its sides, its middle and the points one step from it. The cocircular, in-line and nearly in-line
  // cases that random points hardly ever meet come up, with the largest differences the bound allows.
  constexpr std::int32_t m = kMostPredicateCoordinate;
  std::vector<GridPoint> points;
  for (const std::int32_t x : {-m, -1, 0, 1, m}) {
    for (const std::int32_t y : {-m, -1, 0, 1, m}) points.push_back({x, y});
  }

  Tally tally;
  for (const GridPoint& a : points) {
    for (const GridPoint& b : points) {
      for (const GridPoint& c : points) {
        for (const GridPoint& d : points) tally.add(a, b, c, d);
      }
    }
  }

  INFO("first disagreement: " << tally.first);
  CHECK(tally.checked == 390625);
  CHECK(tally.disagreements == 0);
}

TEST_CASE("shifting all four points by the same vector changes neither predicate", "[predicates]") {
  // Both positions must lie inside the bound that a debug build asserts, so the quadruples are taken from x in [0, m],
  // y in [-m, 0], and the shift (-m, m), the farthest the bound allows, carries them to x in [-m, 0], y in [0, m].
  constexpr std::int32_t m = kMostPredicateCoordinate;
  const std::int64_t mm = std::int64_t{m} * m;
  const auto shifted = [](GridPoint p) { return GridPoint{p.x - m, p.y + m}; };
  const auto unchanged = [&shifted](GridPoint a, GridPoint b, GridPoint c, GridPoint d) {
    return orient2d(shifted(a), shifted(b), shifted(c)) == orient2d(a, b, c) &&
           inCircle(shifted(a), shifted(b), shifted(c), shifted(d)) == inCircle(a, b, c, d);
  };

  // The square of side m of the case above, moved down by m: its fourth corner and a point one step inside. The values
  // are those of that case, before and after the shift.
  const GridPoint a{0, -m}, b{m, -m}, c{m, 0};
  const GridPoint corner{0, 0}, inner{1, -1};
  CHECK(orient2d(a, b, c) == mm);
  CHECK(inCircle(a, b, c, corner) == 0);
  CHECK(inCircle(a, b, c, inner) == 2 * mm * (m - 1));
  CHECK(orient2d(shifted(a), shifted(b), shifted(c)) == mm);
  CHECK(inCircle(shifted(a), shifted(b), shifted(c), shifted(corner)) == 0);
  CHECK(inCircle(shifted(a), shifted(b), shifted(c), shifted(inner)) == 2 * mm * (m - 1));

  // Then random quadruples from the same quarter of the box.
  std::mt19937 rng(5);
  const auto point = [&rng] {
    const std::int32_t x = static_cast<std::int32_t>(rng() % 16385u);
    const std::int32_t y = -static_cast<std::int32_t>(rng() % 16385u);
    return GridPoint{x, y};
  };
  int changed = 0;
  for (int i = 0; i < 500; ++i) {
    const GridPoint ra = point();
    const GridPoint rb = point();
    const GridPoint rc = point();
    const GridPoint rd = point();
    if (!unchanged(ra, rb, rc, rd)) ++changed;
  }
  CHECK(changed == 0);
}
