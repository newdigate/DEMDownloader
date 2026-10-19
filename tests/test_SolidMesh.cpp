#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>
#include <stdexcept>
#include <string>
#include <vector>

#include "mesh/BasePlane.h"
#include "mesh/SolidMesh.h"

using Catch::Matchers::Message;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using demdl::mesh::BasePlane;
using demdl::mesh::checkSolid;
using demdl::mesh::chooseBasePlane;
using demdl::mesh::kMostTrianglesForEdgeCheck;
using demdl::mesh::Mesh;
using demdl::mesh::MeshCheck;
using demdl::mesh::solidMesh;
using demdl::mesh::solidTriangleCount;
using demdl::mesh::surfaceMesh;
using demdl::mesh::Triangle;
using demdl::mesh::Vec3;

// A grid is written row by row from its north-west corner, as resampleQuadratic gives it: vertex (i, j), column i from
// the west and row j from the north, is vertex number j cols + i. Where a check is not a literal, it is worked out here
// from what a closed solid must be, by roads that do not go through the code under test: the faces' normals and areas
// from the vertices, the volume as the prisms under the surface's triangles, the walls' areas as the polygons under the
// sides.

namespace {

using Grid = std::vector<float>;
using Corners = std::array<std::uint32_t, 3>;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

std::size_t cell(int cols, int i, int j) {
  return static_cast<std::size_t>(j) * static_cast<std::size_t>(cols) + static_cast<std::size_t>(i);
}

// A grid of cols x rows heights whose height at column i and row j is f(i, j).
template <typename F>
Grid makeGrid(int cols, int rows, F f) {
  Grid grid(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
  for (int j = 0; j < rows; ++j) {
    for (int i = 0; i < cols; ++i) grid[cell(cols, i, j)] = static_cast<float>(f(i, j));
  }
  return grid;
}

std::array<float, 3> xyz(const Vec3& v) { return {v.x, v.y, v.z}; }

struct D3 {
  double x, y, z;
};

D3 at(const Mesh& mesh, std::uint32_t v) {
  const Vec3& p = mesh.vertices[v];
  return {p.x, p.y, p.z};
}

// (b - a) x (c - a), in double: it points to the side the triangle faces (its vertices run counter-clockwise seen from
// there), and its length is twice the triangle's area.
D3 normalOf(const Mesh& mesh, const Triangle& t) {
  const D3 a = at(mesh, t.a), b = at(mesh, t.b), c = at(mesh, t.c);
  const D3 u{b.x - a.x, b.y - a.y, b.z - a.z};
  const D3 w{c.x - a.x, c.y - a.y, c.z - a.z};
  return {u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x};
}

double length(const D3& v) { return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z); }

std::string describe(const D3& v) {
  return "(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
}

enum class Face { Surface, South, East, North, West, Base };
constexpr std::array<const char*, 6> kFaceName = {"surface", "south wall", "east wall", "north wall", "west wall",
                                                  "base"};
constexpr std::array<Face, 4> kSides = {Face::South, Face::East, Face::North, Face::West};

std::size_t index(Face face) { return static_cast<std::size_t>(face); }

// Which face of a solid a triangle belongs to, told by where its vertices are: a base triangle has all three at the
// plane, a wall triangle all three in its side's plane, and a surface triangle, which spans two rows and two columns of
// the grid, neither. east and north are the x of the east side and the y of the north side.
Face faceOf(const Mesh& solid, const Triangle& t, float east, float north, float planeZ) {
  const std::array<Vec3, 3> v = {solid.vertices[t.a], solid.vertices[t.b], solid.vertices[t.c]};
  const auto all = [&](auto test) { return test(v[0]) && test(v[1]) && test(v[2]); };
  if (all([&](const Vec3& p) { return p.z == planeZ; })) return Face::Base;
  if (all([](const Vec3& p) { return p.y == 0.0f; })) return Face::South;
  if (all([&](const Vec3& p) { return p.x == east; })) return Face::East;
  if (all([&](const Vec3& p) { return p.y == north; })) return Face::North;
  if (all([](const Vec3& p) { return p.x == 0.0f; })) return Face::West;
  return Face::Surface;
}

// The top vertices of a side in the order the boundary is walked, counter-clockwise seen from above: the south side
// from west to east, the east side from south to north, the north side from east to west, the west side from north to
// south.
std::vector<std::uint32_t> sideTop(Face side, int cols, int rows) {
  std::vector<std::uint32_t> top;
  const auto add = [&](int i, int j) { top.push_back(static_cast<std::uint32_t>(cell(cols, i, j))); };
  if (side == Face::South) for (int i = 0; i < cols; ++i) add(i, rows - 1);
  if (side == Face::East) for (int j = rows - 1; j >= 0; --j) add(cols - 1, j);
  if (side == Face::North) for (int i = cols - 1; i >= 0; --i) add(i, 0);
  if (side == Face::West) for (int j = 0; j < rows; ++j) add(0, j);
  return top;
}

// The bottom corners under a side's first and last top vertex. The solid numbers its bottom corners after the surface,
// under the south-west, south-east, north-east and north-west corners in that order.
std::array<std::uint32_t, 2> sideCorners(Face side, int cols, int rows) {
  const auto n = static_cast<std::uint32_t>(cols * rows);
  switch (side) {
    case Face::South: return {n + 0, n + 1};
    case Face::East: return {n + 1, n + 2};
    case Face::North: return {n + 2, n + 3};
    default: return {n + 3, n + 0};
  }
}

// Says what is wrong with the faces of a solid that solidMesh made from a grid of cols x rows, or returns "" when
// nothing is: every surface triangle faces up, every wall triangle lies in its side's plane and faces out of the solid,
// every base triangle faces down; each side has one triangle per top vertex, the base two; and the triangles of each
// wall and of the base cover it once, their areas adding up to its own (a wall folded over itself covers some of it
// twice, with some triangles facing in).
std::string faceProblem(const Mesh& solid, int cols, int rows, float planeZ) {
  const float east = solid.vertices[static_cast<std::size_t>(cols - 1)].x;
  const float north = solid.vertices[0].y;
  std::array<std::size_t, 6> count{};
  std::array<double, 6> area{};
  for (std::size_t k = 0; k < solid.triangles.size(); ++k) {
    const Triangle& t = solid.triangles[k];
    const Face face = faceOf(solid, t, east, north, planeZ);
    const D3 n = normalOf(solid, t);
    bool outward = false;
    switch (face) {
      case Face::Surface: outward = n.z > 0; break;
      case Face::South: outward = n.y < 0 && n.x == 0 && n.z == 0; break;
      case Face::East: outward = n.x > 0 && n.y == 0 && n.z == 0; break;
      case Face::North: outward = n.y > 0 && n.x == 0 && n.z == 0; break;
      case Face::West: outward = n.x < 0 && n.y == 0 && n.z == 0; break;
      case Face::Base: outward = n.z < 0 && n.x == 0 && n.y == 0; break;
    }
    if (!outward) {
      return "triangle " + std::to_string(k) + " of the " + kFaceName[index(face)] + " faces " + describe(n);
    }
    ++count[index(face)];
    area[index(face)] += length(n) / 2.0;
  }

  const auto c = static_cast<std::size_t>(cols);
  const auto r = static_cast<std::size_t>(rows);
  const std::array<std::size_t, 6> expected = {2 * (c - 1) * (r - 1), c, r, c, r, 2};
  for (std::size_t f = 0; f < count.size(); ++f) {
    if (count[f] != expected[f]) {
      return std::string("the ") + kFaceName[f] + " has " + std::to_string(count[f]) + " triangles, not " +
             std::to_string(expected[f]);
    }
  }

  const auto close = [](double got, double want) { return std::fabs(got - want) <= 1e-9 * std::fabs(want); };
  for (const Face side : kSides) {
    // The wall's polygon, from the bottom edge up to its top vertices: trapezoids under each top edge.
    const std::vector<std::uint32_t> top = sideTop(side, cols, rows);
    const bool alongX = side == Face::South || side == Face::North;
    double polygon = 0;
    for (std::size_t k = 0; k + 1 < top.size(); ++k) {
      const D3 p = at(solid, top[k]), q = at(solid, top[k + 1]);
      const double run = alongX ? std::fabs(q.x - p.x) : std::fabs(q.y - p.y);
      polygon += run * ((p.z + q.z) / 2.0 - planeZ);
    }
    if (!close(area[index(side)], polygon)) {
      return std::string("the ") + kFaceName[index(side)] + "'s triangles cover " + std::to_string(area[index(side)]) +
             ", its polygon " + std::to_string(polygon);
    }
  }
  if (!close(area[index(Face::Base)], static_cast<double>(east) * static_cast<double>(north))) {
    return "the base's triangles cover " + std::to_string(area[index(Face::Base)]);
  }
  return "";
}

// The volume between the surface and the plane z = planeZ: for each surface triangle, its area seen from above times
// its mean height over the plane, which is exact for a surface of flat triangles. It is the solid's volume by another
// road than checkSolid's, which sums over every face and never looks at a plane.
double prismVolume(const Mesh& solid, std::size_t surfaceTriangles, float planeZ) {
  double volume = 0;
  for (std::size_t k = 0; k < surfaceTriangles; ++k) {
    const Triangle& t = solid.triangles[k];
    const double mean = (at(solid, t.a).z + at(solid, t.b).z + at(solid, t.c).z) / 3.0 - planeZ;
    volume += normalOf(solid, t).z / 2.0 * mean;
  }
  return volume;
}

// The triangles of one face, each as its three vertex numbers in increasing order, sorted: what the face is made of,
// whatever order its triangles come in and wherever each starts.
std::vector<Corners> trianglesOf(const Mesh& solid, int cols, Face face, float planeZ) {
  const float east = solid.vertices[static_cast<std::size_t>(cols - 1)].x;
  const float north = solid.vertices[0].y;
  std::vector<Corners> found;
  for (const Triangle& t : solid.triangles) {
    if (faceOf(solid, t, east, north, planeZ) != face) continue;
    Corners c = {t.a, t.b, t.c};
    std::sort(c.begin(), c.end());
    found.push_back(c);
  }
  std::sort(found.begin(), found.end());
  return found;
}

// The plan's wall for a side, in the same form: a fan from the bottom corner under the side's first top vertex to each
// pair of neighbouring top vertices, then one triangle from the last top vertex to the next bottom corner.
std::vector<Corners> planFan(Face side, int cols, int rows) {
  const std::vector<std::uint32_t> top = sideTop(side, cols, rows);
  const auto [start, end] = sideCorners(side, cols, rows);
  std::vector<Corners> fan;
  for (std::size_t k = 1; k < top.size(); ++k) fan.push_back({start, top[k - 1], top[k]});
  fan.push_back({start, end, top.back()});
  for (Corners& c : fan) std::sort(c.begin(), c.end());
  std::sort(fan.begin(), fan.end());
  return fan;
}

// The brief's box: a 2 x 2 grid of heights 10 over 100 m by 50 m, no exaggeration or scaling, level 0, closed down to
// z = -5.
Mesh box() { return solidMesh(surfaceMesh(Grid(4, 10.0f), 2, 2, 100, 50, 1, 1, 0), 2, 2, -5.0f); }

// Heights from low to high in steps of (high - low) / 100000, drawn from std::mt19937, whose output the standard fixes,
// so the grids are the same on every machine; the standard distributions are not, and are not used.
Grid randomGrid(std::mt19937& rng, int cols, int rows, double low, double high) {
  return makeGrid(cols, rows,
                  [&](int, int) { return low + (high - low) * static_cast<double>(rng() % 100001) / 100000.0; });
}

// A rough ridge line: a random walk whose step keeps nine tenths of the last one, floored at 0, as a stand-in for real
// terrain. With along, each row carries its own walk from west to east; without, each column from north to south.
Grid walkGrid(std::mt19937& rng, int cols, int rows, bool along) {
  Grid grid(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows));
  const int lines = along ? rows : cols;
  const int length = along ? cols : rows;
  for (int line = 0; line < lines; ++line) {
    double height = 0, step = 0;
    for (int k = 0; k < length; ++k) {
      step = 0.9 * step + static_cast<double>(static_cast<int>(rng() % 601) - 300) / 100.0;
      height = std::max(0.0, height + step);
      grid[along ? cell(cols, k, line) : cell(cols, line, k)] = static_cast<float>(height);
    }
  }
  return grid;
}

}  // namespace

TEST_CASE("a 2 x 2 surface has row 0 in the north and faces up", "[solid]") {
  const Mesh surface = surfaceMesh(Grid(4, 10.0f), 2, 2, 100, 50, 1, 1, 0);
  REQUIRE(surface.vertices.size() == 4);
  CHECK(xyz(surface.vertices[0]) == std::array<float, 3>{0, 50, 10});    // north-west
  CHECK(xyz(surface.vertices[1]) == std::array<float, 3>{100, 50, 10});  // north-east
  CHECK(xyz(surface.vertices[2]) == std::array<float, 3>{0, 0, 10});     // south-west
  CHECK(xyz(surface.vertices[3]) == std::array<float, 3>{100, 0, 10});   // south-east
  REQUIRE(surface.triangles.size() == 2);
  for (const Triangle& t : surface.triangles) {
    const D3 n = normalOf(surface, t);
    CHECK(n.x == 0);
    CHECK(n.y == 0);
    CHECK(n.z > 0);
    CHECK_THAT(length(n) / 2, WithinAbs(2500.0, 1e-9));  // half the cell each
  }
}

TEST_CASE("a 2 x 2 grid closes into a box of 8 vertices and 12 triangles with the box's volume", "[solid]") {
  const Mesh solid = box();
  CHECK(solid.vertices.size() == 8);
  CHECK(solidTriangleCount(2, 2) == 12);
  CHECK(solid.triangles.size() == 12);
  const MeshCheck check = checkSolid(solid);
  CHECK(check.triangles == 12);
  CHECK(check.edgesChecked);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK_THAT(check.signedVolume, WithinAbs(100.0 * 50.0 * 15.0, 1e-3));
  CHECK(check.ok());
  CHECK(faceProblem(solid, 2, 2, -5.0f).empty());
}

TEST_CASE("a 5 x 4 sloping grid closes into 44 triangles holding the volume under the slope", "[solid]") {
  // Heights 10 + i + 2 j over 100 m by 60 m: a plane, which any triangulation of its samples reproduces, so the volume
  // down to -5 is the area times the height over -5 at the centre, i = 2 and j = 1.5: 6000 x (10 + 2 + 3 + 5) = 120000.
  const Mesh surface = surfaceMesh(makeGrid(5, 4, [](int i, int j) { return 10 + i + 2 * j; }), 5, 4, 100, 60, 1, 1, 0);
  const Mesh solid = solidMesh(surface, 5, 4, -5.0f);
  CHECK(solidTriangleCount(5, 4) == 2 * 4 * 3 + 2 * 9 + 2);
  CHECK(solid.triangles.size() == 44);
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK(check.ok());
  CHECK_THAT(check.signedVolume, WithinAbs(120000.0, 1e-6));
  CHECK(faceProblem(solid, 5, 4, -5.0f).empty());
}

TEST_CASE("solidTriangleCount is 2 (cols - 1)(rows - 1) + 2 (cols + rows) + 2, in 64 bits", "[solid]") {
  CHECK(solidTriangleCount(2, 2) == 12);
  CHECK(solidTriangleCount(4, 3) == 28);
  CHECK(solidTriangleCount(4, 4) == 36);
  CHECK(solidTriangleCount(8, 6) == 100);
  CHECK(solidTriangleCount(2, 7) == 2 * 6 + 2 * 9 + 2);
  // 2 x 65535 x 65535 + 2 x 131072 + 2 is past what 32 bits hold.
  CHECK(solidTriangleCount(65536, 65536) == 8589934596ULL);
  // No mesh is made below 2 x 2.
  CHECK(solidTriangleCount(1, 5) == 0);
  CHECK(solidTriangleCount(5, 1) == 0);
  CHECK(solidTriangleCount(0, 0) == 0);
  CHECK(solidTriangleCount(-3, 4) == 0);
}

TEST_CASE("the solid starts with the surface, then the four bottom corners under its corners", "[solid]") {
  const Mesh surface = surfaceMesh(makeGrid(4, 3, [](int i, int j) { return 3 * i + j; }), 4, 3, 30, 20, 1, 1, 0);
  const Mesh solid = solidMesh(surface, 4, 3, -2.0f);
  REQUIRE(solid.vertices.size() == 12 + 4);
  REQUIRE(solid.triangles.size() == solidTriangleCount(4, 3));
  for (std::size_t v = 0; v < surface.vertices.size(); ++v) CHECK(xyz(solid.vertices[v]) == xyz(surface.vertices[v]));
  for (std::size_t t = 0; t < surface.triangles.size(); ++t) {
    CHECK(solid.triangles[t].a == surface.triangles[t].a);
    CHECK(solid.triangles[t].b == surface.triangles[t].b);
    CHECK(solid.triangles[t].c == surface.triangles[t].c);
  }
  CHECK(xyz(solid.vertices[12]) == std::array<float, 3>{0, 0, -2});    // under the south-west corner
  CHECK(xyz(solid.vertices[13]) == std::array<float, 3>{30, 0, -2});   // south-east
  CHECK(xyz(solid.vertices[14]) == std::array<float, 3>{30, 20, -2});  // north-east
  CHECK(xyz(solid.vertices[15]) == std::array<float, 3>{0, 20, -2});   // north-west
}

TEST_CASE("every face points out of the solid, also where the terrain's edge hides part of itself", "[solid]") {
  // A fan from one bottom corner over a side's top vertices faces outward only while each top vertex, looked at from
  // that corner, appears lower than the one before it. Past a valley that a nearer rise hides from the corner, its
  // triangles would face inward and overlap the others. These grids make that happen on every side: a chessboard of 100
  // and 0, spikes beside the corners, notches, random heights, and random walks at print scale.
  struct Case {
    const char* name;
    int cols, rows;
    Grid heights;
    double scale;
  };
  std::mt19937 rng(20261019);
  std::vector<Case> cases;
  cases.push_back({"chessboard", 5, 3, makeGrid(5, 3, [](int i, int j) { return (i + j) % 2 == 0 ? 100 : 0; }), 1});
  cases.push_back({"spikes", 7, 6, makeGrid(7, 6, [](int i, int j) {
                     const bool besideCorner = (i == 1 || i == 5) != (j == 1 || j == 4);
                     const bool onEdge = i == 0 || i == 6 || j == 0 || j == 5;
                     return besideCorner && onEdge ? 1000 : 0;
                   }), 1});
  cases.push_back({"notches", 9, 7, makeGrid(9, 7, [](int i, int j) {
                     const bool notch = i == 1 || i == 4 || i == 7 || j == 1 || j == 3 || j == 5;
                     return notch ? 0 : 500;
                   }), 1});
  for (const auto& [c, r] : std::vector<std::pair<int, int>>{{2, 2}, {2, 9}, {9, 2}, {3, 3}, {12, 7}, {40, 30}}) {
    cases.push_back({"random", c, r, randomGrid(rng, c, r, 0, 1000), 1});
    cases.push_back({"random at 0.02", c, r, randomGrid(rng, c, r, -40, 1600), 0.02});
  }
  cases.push_back({"walk along", 400, 3, walkGrid(rng, 400, 3, true), 0.02});
  cases.push_back({"walk down", 3, 400, walkGrid(rng, 3, 400, false), 0.02});

  for (const Case& c : cases) {
    INFO(c.name << ", " << c.cols << " x " << c.rows << " at scale " << c.scale);
    const auto [lowest, highest] = std::minmax_element(c.heights.begin(), c.heights.end());
    const BasePlane base = chooseBasePlane(*lowest, *highest, std::nullopt, 1, c.scale, 2);
    const double width = 25.0 * (c.cols - 1), height = 20.0 * (c.rows - 1);
    const Mesh surface = surfaceMesh(c.heights, c.cols, c.rows, width, height, 1, c.scale, base.levelM);
    const auto planeZ = static_cast<float>(base.planeZ);
    const Mesh solid = solidMesh(surface, c.cols, c.rows, planeZ);
    CHECK(solid.triangles.size() == solidTriangleCount(c.cols, c.rows));
    CHECK(faceProblem(solid, c.cols, c.rows, planeZ) == "");
    const MeshCheck check = checkSolid(solid);
    CHECK(check.nonManifoldEdges == 0);
    CHECK(check.zeroAreaTriangles == 0);
    CHECK(check.ok());
    CHECK_THAT(check.signedVolume, WithinRel(prismVolume(solid, surface.triangles.size(), planeZ), 1e-9));
  }
}

TEST_CASE("a side in line with a bottom corner gets no triangle without area", "[solid]") {
  // The south side runs x = 0, 1, 2, 3 at heights 5, 1, 3, 2 over a plane at -1. Seen from the south-west bottom
  // corner, (0, -1), the top vertices at x = 1 and x = 2 are in line: (1 + 1) / 1 = (3 + 1) / 2. A fan from that
  // corner would make a triangle of the corner and those two with no area, which the self-check refuses.
  const Grid heights = makeGrid(4, 2, [](int i, int j) {
    constexpr std::array<int, 4> south = {5, 1, 3, 2};
    return j == 1 ? south[static_cast<std::size_t>(i)] : 4;
  });
  const Mesh solid = solidMesh(surfaceMesh(heights, 4, 2, 3, 1, 1, 1, 0), 4, 2, -1.0f);
  CHECK(faceProblem(solid, 4, 2, -1.0f) == "");
  const MeshCheck check = checkSolid(solid);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.ok());
  CHECK_THAT(check.signedVolume, WithinRel(prismVolume(solid, 6, -1.0f), 1e-12));
}

TEST_CASE("where the plan's fan faces outward all along, each wall is that fan", "[solid]") {
  // On a flat grid every top vertex, looked at from the side's first bottom corner, appears lower than the one before
  // it, and on a plane too (each side is a straight line that does not pass through the corner), so the plan's fan is a
  // proper cut there and is what solidMesh makes.
  struct Case {
    int cols, rows;
    Grid heights;
  };
  const std::vector<Case> cases = {
      {2, 2, Grid(4, 10.0f)},
      {6, 4, Grid(24, 10.0f)},
      {5, 4, makeGrid(5, 4, [](int i, int j) { return 10 + i + 2 * j; })},
  };
  for (const Case& c : cases) {
    INFO(c.cols << " x " << c.rows);
    const Mesh solid = solidMesh(surfaceMesh(c.heights, c.cols, c.rows, 100, 60, 1, 1, 0), c.cols, c.rows, -5.0f);
    for (const Face side : kSides) {
      INFO(kFaceName[index(side)]);
      CHECK(trianglesOf(solid, c.cols, side, -5.0f) == planFan(side, c.cols, c.rows));
    }
  }
}

TEST_CASE("a height below the level is lifted to the level", "[solid]") {
  // Level -10 m, z-scale 2, scale 0.5: the -50 m height comes out at -10 x 2 x 0.5 = -10, the others at 10 x 2 x 0.5.
  const Mesh surface = surfaceMesh(Grid{-50.0f, 10.0f, -10.0f, 10.0f}, 2, 2, 100, 50, 2, 0.5, -10);
  CHECK(surface.vertices[0].z == static_cast<float>(-10.0 * 2 * 0.5));
  CHECK(surface.vertices[1].z == static_cast<float>(10.0 * 2 * 0.5));
  CHECK(surface.vertices[2].z == static_cast<float>(-10.0 * 2 * 0.5));  // at the level already
  CHECK(surface.vertices[3].z == static_cast<float>(10.0 * 2 * 0.5));

  // A height that is not a number is not lifted to the level, which would hide it: it stays NaN, for solidMesh to
  // refuse.
  const Mesh holed = surfaceMesh(Grid{10.0f, kNaN, 10.0f, 10.0f}, 2, 2, 100, 50, 1, 1, 0);
  CHECK(std::isnan(holed.vertices[1].z));
}

TEST_CASE("the north-east corner of the grid is the north-east corner of the model", "[solid]") {
  // Height 100 at column 2, row 0 of a 3 x 2 grid, 0 elsewhere: the vertex furthest east and north holds it, so the
  // model is not mirrored either way.
  const Grid heights = makeGrid(3, 2, [](int i, int j) { return i == 2 && j == 0 ? 100 : 0; });
  const Mesh surface = surfaceMesh(heights, 3, 2, 200, 100, 1, 1, 0);
  const auto northEast = std::max_element(surface.vertices.begin(), surface.vertices.end(),
                                          [](const Vec3& a, const Vec3& b) { return a.x + a.y < b.x + b.y; });
  CHECK(northEast->x == 200);
  CHECK(northEast->y == 100);
  CHECK(northEast->z == 100);
  for (const Vec3& v : surface.vertices) {
    if (&v != &*northEast) CHECK(v.z == 0);
  }
}

TEST_CASE("positions are scaled by the scale and heights by the z-scale times the scale", "[solid]") {
  // 5 x 4 over 100 m by 60 m at --scale 0.02 and --z-scale 3: columns 0.5 apart, rows 0.4 apart, rows from the north.
  const Mesh surface =
      surfaceMesh(makeGrid(5, 4, [](int i, int j) { return 100 * i + j; }), 5, 4, 100, 60, 3, 0.02, 0);
  for (int j = 0; j < 4; ++j) {
    for (int i = 0; i < 5; ++i) {
      const Vec3& v = surface.vertices[cell(5, i, j)];
      CHECK(v.x == static_cast<float>(i * 100.0 / 4 * 0.02));
      CHECK(v.y == static_cast<float>((3 - j) * 60.0 / 3 * 0.02));
      CHECK(v.z == static_cast<float>((100.0 * i + j) * 3 * 0.02));
    }
  }
  CHECK_THAT(surface.vertices[cell(5, 4, 0)].x, WithinAbs(2.0, 1e-6));
  CHECK_THAT(surface.vertices[cell(5, 4, 0)].y, WithinAbs(1.2, 1e-6));
}

TEST_CASE("a triangle wound the wrong way makes three bad edges", "[solid]") {
  // Each of its edges then runs the same way as the neighbour's across it, and no triangle runs back along it.
  Mesh solid = box();
  std::swap(solid.triangles[0].b, solid.triangles[0].c);
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 3);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK_FALSE(check.ok());
}

TEST_CASE("a hole makes three bad edges", "[solid]") {
  Mesh solid = box();
  solid.triangles.pop_back();
  const MeshCheck check = checkSolid(solid);
  CHECK(check.triangles == 11);
  CHECK(check.nonManifoldEdges == 3);
  CHECK_FALSE(check.ok());
}

TEST_CASE("a triangle twice over makes three bad edges, each an edge of three triangles", "[solid]") {
  Mesh solid = box();
  solid.triangles.push_back(solid.triangles[0]);
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 3);
  CHECK_FALSE(check.ok());
}

TEST_CASE("a triangle with a vertex twice has an edge from that vertex to itself", "[solid]") {
  // Vertices 0 (the top of the north-west corner) and 4 (the bottom of the south-west corner) share no edge of the box,
  // so the triangle (0, 4, 4) adds an edge from 0 to 4 and one back, which pair up, and an edge from 4 to 4, which is
  // bad. It has no area either.
  Mesh solid = box();
  solid.triangles.push_back(Triangle{0, 4, 4});
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 1);
  CHECK(check.zeroAreaTriangles == 1);
  CHECK_FALSE(check.ok());
}

TEST_CASE("a closed mesh that encloses nothing is refused", "[solid]") {
  // One triangle and the same triangle wound the other way: closed, every edge shared by two triangles running
  // opposite ways, both with an area, and a volume of exactly 0.
  Mesh sheet;
  sheet.vertices = {{0, 0, 1}, {1, 0, 1}, {0, 1, 1}};
  sheet.triangles = {{0, 1, 2}, {0, 2, 1}};
  const MeshCheck check = checkSolid(sheet);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK(check.signedVolume == 0);
  CHECK_FALSE(check.ok());
}

TEST_CASE("a closed mesh with a zero-area triangle is reported and refused", "[solid]") {
  // The surface's diagonal runs from vertex 0, (0, 50, 10), to vertex 3, (100, 0, 10). The triangle that runs from 0 to
  // 3 is cut in two at the diagonal's midpoint m, (50, 25, 10), and the triangle (3, m, 0) closes the mesh again along
  // the diagonal: every edge is still shared by two triangles running opposite ways, and the volume is the same, but
  // (3, m, 0) has its three points in line.
  Mesh solid = box();
  std::size_t found = solid.triangles.size();
  std::uint32_t w = 0;
  for (std::size_t k = 0; k < solid.triangles.size() && found == solid.triangles.size(); ++k) {
    const Triangle& t = solid.triangles[k];
    const std::array<std::uint32_t, 3> v = {t.a, t.b, t.c};
    for (std::size_t s = 0; s < 3; ++s) {
      if (v[s] == 0 && v[(s + 1) % 3] == 3) {
        found = k;
        w = v[(s + 2) % 3];
      }
    }
  }
  REQUIRE(found < solid.triangles.size());
  const auto m = static_cast<std::uint32_t>(solid.vertices.size());
  solid.vertices.push_back({50, 25, 10});
  solid.triangles[found] = Triangle{0, m, w};
  solid.triangles.push_back(Triangle{m, 3, w});
  solid.triangles.push_back(Triangle{3, m, 0});
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 1);
  CHECK_THAT(check.signedVolume, WithinAbs(75000.0, 1e-6));
  CHECK_FALSE(check.ok());
}

TEST_CASE("a solid turned inside out has a negative volume and is refused", "[solid]") {
  Mesh solid = box();
  for (Triangle& t : solid.triangles) std::swap(t.b, t.c);
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK_THAT(check.signedVolume, WithinAbs(-75000.0, 1e-3));
  CHECK_FALSE(check.ok());
}

TEST_CASE("a vertex that is not a number spoils the areas and the volume", "[solid]") {
  Mesh solid = box();
  solid.vertices[0].z = kNaN;
  const auto touching = static_cast<std::uint64_t>(
      std::count_if(solid.triangles.begin(), solid.triangles.end(),
                    [](const Triangle& t) { return t.a == 0 || t.b == 0 || t.c == 0; }));
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == touching);
  CHECK(std::isnan(check.signedVolume));
  CHECK_FALSE(check.ok());
}

TEST_CASE("above 10,000,000 triangles the edges go unchecked, while areas and the volume still are", "[solid]") {
  // Copies of one face of the corner tetrahedron, (1, 0, 0), (0, 1, 0), (0, 0, 1), whose a . (b x c) is 1, so each adds
  // 1/6 to the volume. The mesh is nowhere closed, which only the edge check would see.
  CHECK(kMostTrianglesForEdgeCheck == 10'000'000);
  Mesh mesh;
  mesh.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  mesh.triangles.assign(kMostTrianglesForEdgeCheck + 1, Triangle{1, 2, 3});
  MeshCheck check = checkSolid(mesh);
  CHECK_FALSE(check.edgesChecked);
  CHECK(check.triangles == 10'000'001);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK_THAT(check.signedVolume, WithinRel(10'000'001 / 6.0, 1e-12));
  CHECK(check.ok());

  mesh.triangles.back() = Triangle{1, 2, 2};
  check = checkSolid(mesh);
  CHECK(check.zeroAreaTriangles == 1);
  CHECK_FALSE(check.ok());
}

TEST_CASE("checkSolid refuses a triangle that names a vertex the mesh does not have", "[solid]") {
  Mesh mesh;
  mesh.vertices = {{0, 0, 0}, {1, 0, 0}, {0, 1, 0}};
  mesh.triangles = {{0, 1, 2}, {0, 2, 3}};
  REQUIRE_THROWS_MATCHES(checkSolid(mesh), std::invalid_argument,
                         Message("triangle 1 names vertex 3, but the mesh has 3 vertices"));
}

TEST_CASE("surfaceMesh refuses a grid or a value it cannot make a surface of", "[solid]") {
  const Grid four(4, 1.0f);
  REQUIRE_THROWS_MATCHES(surfaceMesh(Grid(2, 1.0f), 1, 2, 100, 50, 1, 1, 0), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 1 x 2"));
  REQUIRE_THROWS_MATCHES(surfaceMesh(Grid(2, 1.0f), 2, 1, 100, 50, 1, 1, 0), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 2 x 1"));
  REQUIRE_THROWS_MATCHES(surfaceMesh(Grid(5, 1.0f), 2, 2, 100, 50, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 2 x 2 vertices needs 4 heights, not 5"));
  // Vertex numbers are 32 bits and the solid adds four bottom corners, so a grid may have 4294967292 vertices: judged
  // before the heights are looked at, so these need none.
  REQUIRE_THROWS_MATCHES(surfaceMesh(Grid{}, 2, 2147483647, 100, 50, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 2 x 2147483647 vertices is too large: 32-bit vertex numbers leave room "
                                 "for 4294967292 besides the four bottom corners"));
  REQUIRE_THROWS_MATCHES(surfaceMesh(Grid{}, 4, 1073741823, 100, 50, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 4 x 1073741823 vertices needs 4294967292 heights, not 0"));
  // 65536 x 65537 is 2^32 + 65536, which in 32 bits would wrap round to a count of 65536.
  REQUIRE_THROWS_MATCHES(surfaceMesh(Grid{}, 65536, 65537, 100, 50, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 65536 x 65537 vertices is too large: 32-bit vertex numbers leave room "
                                 "for 4294967292 besides the four bottom corners"));

  constexpr double inf = std::numeric_limits<double>::infinity();
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  for (const auto& [w, h] : std::vector<std::pair<double, double>>{{0, 50}, {100, -1}, {nan, 50}, {100, inf}}) {
    CHECK_THROWS_AS(surfaceMesh(four, 2, 2, w, h, 1, 1, 0), std::invalid_argument);
  }
  REQUIRE_THROWS_MATCHES(surfaceMesh(four, 2, 2, 0, 50, 1, 1, 0), std::invalid_argument,
                         Message("the window must be wider and taller than 0 m and finite, not 0 x 50 m"));
  for (const auto& [z, s] : std::vector<std::pair<double, double>>{{0, 1}, {1, -0.02}, {inf, 1}, {1, nan}}) {
    CHECK_THROWS_AS(surfaceMesh(four, 2, 2, 100, 50, z, s, 0), std::invalid_argument);
  }
  REQUIRE_THROWS_MATCHES(surfaceMesh(four, 2, 2, 100, 50, 1, -0.02, 0), std::invalid_argument,
                         Message("the z-scale and the scale must be above 0 and finite, not 1 and -0.02"));
  REQUIRE_THROWS_MATCHES(surfaceMesh(four, 2, 2, 100, 50, 1, 1, nan), std::invalid_argument,
                         Message("the level must be finite, not nan"));
}

TEST_CASE("solidMesh refuses a surface it cannot close", "[solid]") {
  const Mesh surface = surfaceMesh(Grid(6, 10.0f), 3, 2, 100, 50, 1, 1, 0);
  REQUIRE_THROWS_MATCHES(solidMesh(surface, 1, 6, -5.0f), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 1 x 6"));
  REQUIRE_THROWS_MATCHES(solidMesh(surface, 3, 3, -5.0f), std::invalid_argument,
                         Message("a grid of 3 x 3 vertices needs a surface of 9 vertices, not 6"));
  REQUIRE_THROWS_MATCHES(solidMesh(surface, 3, 2, kNaN), std::invalid_argument,
                         Message("the base plane must be finite, not nan"));
  REQUIRE_THROWS_MATCHES(solidMesh(surface, 3, 2, std::numeric_limits<float>::infinity()), std::invalid_argument,
                         Message("the base plane must be finite, not inf"));
  // A wall must reach down from every top vertex, so the plane must be below the whole surface.
  REQUIRE_THROWS_MATCHES(solidMesh(surface, 3, 2, 10.0f), std::invalid_argument,
                         Message("the base plane at 10 is not below the surface: vertex 0 is at 10"));
  Mesh holed = surface;
  holed.vertices[4].z = kNaN;
  REQUIRE_THROWS_MATCHES(solidMesh(holed, 3, 2, -5.0f), std::invalid_argument,
                         Message("the base plane at -5 is not below the surface: vertex 4 is at nan"));
}
