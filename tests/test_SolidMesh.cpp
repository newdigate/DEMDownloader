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
#include <utility>
#include <vector>

#include "mesh/BasePlane.h"
#include "mesh/SolidMesh.h"

using Catch::Matchers::Message;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;
using demdl::mesh::BasePlane;
using demdl::mesh::Boundary;
using demdl::mesh::checkSolid;
using demdl::mesh::chooseBasePlane;
using demdl::mesh::gridBoundary;
using demdl::mesh::kMostTrianglesForEdgeCheck;
using demdl::mesh::Mesh;
using demdl::mesh::MeshCheck;
using demdl::mesh::solidFromSurface;
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

// The fan wall for a side, in the same form: a fan from the bottom corner under the side's first top vertex to each
// pair of neighbouring top vertices, then one triangle from the last top vertex to the next bottom corner.
std::vector<Corners> fanWall(Face side, int cols, int rows) {
  const std::vector<std::uint32_t> top = sideTop(side, cols, rows);
  const auto [start, end] = sideCorners(side, cols, rows);
  std::vector<Corners> fan;
  for (std::size_t k = 1; k < top.size(); ++k) fan.push_back({start, top[k - 1], top[k]});
  fan.push_back({start, end, top.back()});
  for (Corners& c : fan) std::sort(c.begin(), c.end());
  std::sort(fan.begin(), fan.end());
  return fan;
}

// The reference box: a 2 x 2 grid of heights 10 over 100 m by 50 m, no exaggeration or scaling, level 0, closed down to
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

// A triangle's three vertex numbers as the mesh has them, where their order says which way the triangle faces.
using Wound = std::array<std::uint32_t, 3>;

// The triangles of a mesh from the one numbered first on, as they stand.
std::vector<Wound> woundFrom(const Mesh& mesh, std::size_t first) {
  std::vector<Wound> found;
  for (std::size_t k = first; k < mesh.triangles.size(); ++k) {
    found.push_back({mesh.triangles[k].a, mesh.triangles[k].b, mesh.triangles[k].c});
  }
  return found;
}

// A surface that is not a grid, built by hand over a rectangle of 100 m by 50 m, and its outline: 3 vertices on the
// south side (x = 0, 30, 100), 2 on the east side, 4 on the north side (x = 100, 70, 20, 0) and 2 on the west side. It
// has 7 vertices, every one of them on the outline, and 5 triangles that face up and cover the rectangle once. Heights
// are in model units, and the base plane is at kIrregularPlane, 4 under the zero:
//
//   vertex         0     1     2      3       4      5      6
//   x, y           0, 0  30, 0 100, 0 100, 50 70, 50 20, 50 0, 50
//   z              10    30    5      12      10     50     20
//   above plane    14    34    9      16      14     54     24
//
// Vertices 0, 2, 3 and 6 are the south-west, south-east, north-east and north-west corners. Seen from the bottom corner
// under vertex 0, vertex 1 appears lower than vertex 0 and vertex 2 lower than vertex 1, so the south wall is the plain
// corner fan. The north wall is not: seen from the corner under vertex 3, vertex 5 (54 over 80) appears higher than
// vertex 4 (14 over 30), so the fan would fold over the valley at vertex 4, and the cut bridges it with a triangle
// between three top vertices. The areas and the volume in the tests below are worked out by hand from this table.
constexpr float kIrregularPlane = -4.0f;
constexpr std::array<std::uint32_t, 7> kInOrder = {0, 1, 2, 3, 4, 5, 6};

struct Irregular {
  Mesh surface;
  Boundary boundary;
};

// The surface and outline above, with the table's vertex v numbered number[v] in the mesh: kInOrder numbers them as the
// table does.
Irregular irregular(const std::array<std::uint32_t, 7>& number = kInOrder) {
  const std::array<Vec3, 7> table = {
      {{0, 0, 10}, {30, 0, 30}, {100, 0, 5}, {100, 50, 12}, {70, 50, 10}, {20, 50, 50}, {0, 50, 20}}};
  Irregular result;
  result.surface.vertices.resize(table.size());
  for (std::size_t v = 0; v < table.size(); ++v) result.surface.vertices[number[v]] = table[v];
  for (const Wound& t : std::vector<Wound>{{0, 1, 6}, {1, 5, 6}, {1, 4, 5}, {1, 2, 4}, {2, 3, 4}}) {
    result.surface.triangles.push_back({number[t[0]], number[t[1]], number[t[2]]});
  }
  const std::array<std::vector<std::uint32_t>, 4> sides = {{{0, 1, 2}, {2, 3}, {3, 4, 5, 6}, {6, 0}}};
  for (std::size_t s = 0; s < sides.size(); ++s) {
    for (const std::uint32_t v : sides[s]) result.boundary.sides[s].push_back(number[v]);
  }
  return result;
}

// What solidFromSurface says when it refuses with std::invalid_argument, or "" when it builds the solid.
std::string refusal(const Mesh& surface, const Boundary& boundary, float planeZ) {
  try {
    static_cast<void>(solidFromSurface(surface, boundary, planeZ));
  } catch (const std::invalid_argument& refused) {
    return refused.what();
  }
  return "";
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

TEST_CASE("where the plain corner fan faces outward all along, each wall is that fan", "[solid]") {
  // On a flat grid every top vertex, looked at from the side's first bottom corner, appears lower than the one before
  // it, and on a plane too (each side is a straight line that does not pass through the corner), so the plain corner
  // fan is a proper cut there and is what solidMesh makes.
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
      CHECK(trianglesOf(solid, c.cols, side, -5.0f) == fanWall(side, c.cols, c.rows));
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

TEST_CASE("gridBoundary walks the grid's outline counter-clockwise from the south-west corner", "[solid]") {
  using Side = std::vector<std::uint32_t>;
  const Boundary four = gridBoundary(4, 3);
  CHECK(four.sides[0] == Side{8, 9, 10, 11});  // south, from west to east
  CHECK(four.sides[1] == Side{11, 7, 3});      // east, from south to north
  CHECK(four.sides[2] == Side{3, 2, 1, 0});    // north, from east to west
  CHECK(four.sides[3] == Side{0, 4, 8});       // west, from north to south

  // The same lists as sideTop works them out for the tests above, for a cell, for a row of cells and a column of them,
  // and for grids long both ways; and each corner ends one side and starts the next, the west side's last being the
  // south side's first.
  for (const auto& [cols, rows] : std::vector<std::pair<int, int>>{{2, 2}, {2, 9}, {9, 2}, {6, 5}, {40, 30}}) {
    INFO(cols << " x " << rows);
    const Boundary b = gridBoundary(cols, rows);
    for (std::size_t s = 0; s < 4; ++s) {
      CHECK(b.sides[s] == sideTop(kSides[s], cols, rows));
      CHECK(b.sides[s].back() == b.sides[(s + 1) % 4].front());
    }
  }

  REQUIRE_THROWS_MATCHES(gridBoundary(1, 5), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 1 x 5"));
  REQUIRE_THROWS_MATCHES(gridBoundary(5, 1), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 5 x 1"));
  // 65536 x 65537 is 2^32 + 65536 vertices, more than 32-bit vertex numbers count, judged as surfaceMesh and solidMesh
  // judge it, before any list is made.
  REQUIRE_THROWS_MATCHES(gridBoundary(65536, 65537), std::invalid_argument,
                         Message("a grid of 65536 x 65537 vertices is too large: 32-bit vertex numbers leave room "
                                 "for 4294967292 besides the four bottom corners"));
}

TEST_CASE("solidFromSurface on a grid's boundary makes the solid solidMesh makes, vertex for vertex and triangle for "
          "triangle",
          "[solid]") {
  // A rough 6 x 5 surface at print scale, each row a random walk from west to east, closed down to a margin under its
  // lowest point (a plinth of 0, which a thicker one would not let the walls fold over).
  std::mt19937 rng(20261021);
  const Grid heights = walkGrid(rng, 6, 5, true);
  const auto [lowest, highest] = std::minmax_element(heights.begin(), heights.end());
  const BasePlane base = chooseBasePlane(*lowest, *highest, std::nullopt, 1, 0.02, 0);
  const Mesh surface = surfaceMesh(heights, 6, 5, 125, 80, 1, 0.02, base.levelM);
  const auto planeZ = static_cast<float>(base.planeZ);

  const Mesh general = solidFromSurface(surface, gridBoundary(6, 5), planeZ);
  const Mesh grid = solidMesh(surface, 6, 5, planeZ);
  REQUIRE(general.vertices.size() == 6 * 5 + 4);
  REQUIRE(general.vertices.size() == grid.vertices.size());
  REQUIRE(general.triangles.size() == solidTriangleCount(6, 5));
  REQUIRE(general.triangles.size() == grid.triangles.size());
  for (std::size_t v = 0; v < general.vertices.size(); ++v) {
    INFO("vertex " << v);
    CHECK(xyz(general.vertices[v]) == xyz(grid.vertices[v]));
  }
  CHECK(woundFrom(general, 0) == woundFrom(grid, 0));
  CHECK(faceProblem(general, 6, 5, planeZ) == "");

  // The walls here are not all the plain corner fan: a wall triangle between three top vertices, none of them a bottom
  // corner, bridges a valley, so the equality above covers the stack's cut as well.
  const auto surfaceVertices = static_cast<std::uint32_t>(surface.vertices.size());
  const auto bridges = std::count_if(general.triangles.begin() + static_cast<std::ptrdiff_t>(surface.triangles.size()),
                                     general.triangles.end(), [surfaceVertices](const Triangle& t) {
                                       return t.a < surfaceVertices && t.b < surfaceVertices && t.c < surfaceVertices;
                                     });
  CHECK(bridges > 0);
}

TEST_CASE("the walls of a grid are cut as they were when each side was a first vertex and a step", "[solid]") {
  // The chessboard of 100 and 0 over 100 m by 40 m, 5 x 3, closed down to z = -5: the triangles after the surface's 16
  // are what solidMesh made before it took its sides from a boundary, kept here as they came. On every side a peak
  // hides the valley beyond it from the corner the fan starts at, so no wall is the plain corner fan, and the longer
  // south and north walls each hold a triangle of three top vertices, (11, 13, 12) and (3, 1, 2). The heights are whole
  // numbers over coordinates that are exact in a float, so no rounding can change a cut. The surface is vertices 0 to
  // 14 and the bottom corners 15 to 18.
  const Mesh surface = surfaceMesh(makeGrid(5, 3, [](int i, int j) { return (i + j) % 2 == 0 ? 100 : 0; }), 5, 3, 100,
                                   40, 1, 1, 0);
  REQUIRE(surface.triangles.size() == 16);
  const std::vector<Wound> expected = {
      {15, 11, 10}, {11, 13, 12}, {15, 13, 11}, {16, 13, 15}, {16, 14, 13},  // south
      {16, 9, 14},  {17, 9, 16},  {17, 4, 9},                                // east
      {17, 3, 4},   {3, 1, 2},    {17, 1, 3},   {18, 1, 17},  {18, 0, 1},    // north
      {18, 5, 0},   {15, 5, 18},  {15, 10, 5},                               // west
      {15, 18, 17}, {15, 17, 16},                                            // base
  };
  CHECK(woundFrom(solidFromSurface(surface, gridBoundary(5, 3), -5.0f), 16) == expected);
  CHECK(woundFrom(solidMesh(surface, 5, 3, -5.0f), 16) == expected);
}

TEST_CASE("solidTriangleCount of a boundary is the surface's triangles, a wall triangle per top vertex and the base's "
          "two",
          "[solid]") {
  static_assert(noexcept(solidTriangleCount(std::declval<const Boundary&>(), std::uint64_t{0})));
  CHECK(solidTriangleCount(irregular().boundary, 5) == 5 + 3 + 2 + 4 + 2 + 2);
  // On a grid's boundary it is the grid's count, 64 bits wide: 2 (cols - 1)(rows - 1) surface triangles.
  for (const auto& [cols, rows] :
       std::vector<std::pair<int, int>>{{2, 2}, {4, 3}, {8, 6}, {2, 7}, {65535, 65535}}) {
    INFO(cols << " x " << rows);
    const auto cellsAcross = static_cast<std::uint64_t>(cols - 1);
    const auto cellsDown = static_cast<std::uint64_t>(rows - 1);
    CHECK(solidTriangleCount(gridBoundary(cols, rows), 2 * cellsAcross * cellsDown) == solidTriangleCount(cols, rows));
  }
  // It only adds up sizes: an empty side counts for 0, though no solid is made of it.
  CHECK(solidTriangleCount(Boundary{}, 0) == 2);
  CHECK(solidTriangleCount(Boundary{}, 7) == 9);
  CHECK(solidTriangleCount(Boundary{}, 8'589'934'592ULL) == 8'589'934'594ULL);
}

TEST_CASE("a surface with an irregular outline is closed with walls cut as monotone polygons and a base", "[solid]") {
  // Once with the vertices numbered as the table of irregular() has them and once shuffled: nothing may depend on the
  // numbers, which a simplified surface will have in no particular order.
  constexpr std::array<std::uint32_t, 7> shuffled = {4, 0, 6, 2, 5, 1, 3};
  for (const auto& number : {kInOrder, shuffled}) {
    INFO("the table's vertex 0 is numbered " << number[0]);
    const Irregular in = irregular(number);
    const Mesh solid = solidFromSurface(in.surface, in.boundary, kIrregularPlane);

    // 5 surface triangles, 3 + 2 + 4 + 2 wall triangles, one for each top vertex of a side, and the base's 2.
    CHECK(solidTriangleCount(in.boundary, in.surface.triangles.size()) == 18);
    REQUIRE(solid.vertices.size() == 7 + 4);
    REQUIRE(solid.triangles.size() == 18);

    // The surface comes first, unchanged, then four bottom corners under the first vertex of the south, east, north
    // and west sides: vertices 0, 2, 3 and 6 of the table.
    for (std::size_t v = 0; v < 7; ++v) CHECK(xyz(solid.vertices[v]) == xyz(in.surface.vertices[v]));
    std::vector<Wound> firstFive = woundFrom(solid, 0);
    firstFive.resize(5);
    CHECK(firstFive == woundFrom(in.surface, 0));
    CHECK(xyz(solid.vertices[7]) == std::array<float, 3>{0, 0, -4});
    CHECK(xyz(solid.vertices[8]) == std::array<float, 3>{100, 0, -4});
    CHECK(xyz(solid.vertices[9]) == std::array<float, 3>{100, 50, -4});
    CHECK(xyz(solid.vertices[10]) == std::array<float, 3>{0, 50, -4});

    // Every triangle faces the way its face must, and each face has the triangles it should.
    std::array<std::size_t, 6> count{};
    std::array<double, 6> area{};
    for (std::size_t k = 0; k < solid.triangles.size(); ++k) {
      const Face face = faceOf(solid, solid.triangles[k], 100.0f, 50.0f, kIrregularPlane);
      const D3 n = normalOf(solid, solid.triangles[k]);
      INFO("triangle " << k << " of the " << kFaceName[index(face)] << " faces " << describe(n));
      switch (face) {
        case Face::Surface: CHECK(n.z > 0); break;
        case Face::South: CHECK((n.y < 0 && n.x == 0 && n.z == 0)); break;
        case Face::East: CHECK((n.x > 0 && n.y == 0 && n.z == 0)); break;
        case Face::North: CHECK((n.y > 0 && n.x == 0 && n.z == 0)); break;
        case Face::West: CHECK((n.x < 0 && n.y == 0 && n.z == 0)); break;
        case Face::Base: CHECK((n.z < 0 && n.x == 0 && n.y == 0)); break;
      }
      ++count[index(face)];
      area[index(face)] += length(n) / 2.0;
    }
    CHECK(count == std::array<std::size_t, 6>{5, 3, 2, 4, 2, 2});
    // The triangles of a wall cover the polygon under its side once, worked out by hand as the trapezoids between
    // neighbouring top vertices, with the heights above the plane.
    // South: 30 x (14 + 34) / 2 + 70 x (34 + 9) / 2. East: 50 x (9 + 16) / 2. North: 30 x (16 + 14) / 2 +
    // 50 x (14 + 54) / 2 + 20 x (54 + 24) / 2. West: 50 x (24 + 14) / 2. The base: 100 x 50.
    CHECK_THAT(area[index(Face::South)], WithinAbs(2225.0, 1e-9));
    CHECK_THAT(area[index(Face::East)], WithinAbs(625.0, 1e-9));
    CHECK_THAT(area[index(Face::North)], WithinAbs(2930.0, 1e-9));
    CHECK_THAT(area[index(Face::West)], WithinAbs(950.0, 1e-9));
    CHECK_THAT(area[index(Face::Base)], WithinAbs(5000.0, 1e-9));

    // A closed solid facing out, whose volume is the prisms under the five surface triangles: areas 750, 500, 1250,
    // 1750 and 750 at mean heights 24, 37 1/3, 34, 19 and 13 above the plane, 366500 / 3 in all.
    const MeshCheck check = checkSolid(solid);
    CHECK(check.triangles == 18);
    CHECK(check.nonManifoldEdges == 0);
    CHECK(check.zeroAreaTriangles == 0);
    CHECK(check.ok());
    CHECK_THAT(check.signedVolume, WithinRel(366500.0 / 3.0, 1e-9));
    CHECK_THAT(check.signedVolume, WithinRel(prismVolume(solid, 5, kIrregularPlane), 1e-9));

    // The cut itself, from the table's numbers (the bottom corners 7 to 10 keep theirs). South: the plain fan from the
    // corner under vertex 0. East: two top vertices, a trapezoid in two. North: vertex 5 hides vertex 4 from the corner
    // under vertex 3, so (4, 6, 5) closes the valley between top vertices before the rest is fanned. West: a trapezoid.
    std::vector<Wound> expected = {
        {7, 1, 0},  {7, 2, 1}, {8, 2, 7},               // south
        {8, 3, 2},  {9, 3, 8},                          // east
        {9, 4, 3},  {4, 6, 5}, {9, 6, 4}, {10, 6, 9},   // north
        {10, 0, 6}, {7, 0, 10},                         // west
        {7, 10, 9}, {7, 9, 8},                          // base
    };
    for (Wound& t : expected) {
      for (std::uint32_t& v : t) {
        if (v < 7) v = number[v];
      }
    }
    CHECK(woundFrom(solid, 5) == expected);
  }
}

TEST_CASE("solidFromSurface refuses an outline or a plane it cannot close a solid on", "[solid]") {
  const Irregular in = irregular();
  const float plane = kIrregularPlane;
  CHECK(refusal(in.surface, in.boundary, plane) == "");

  // A side needs a first and a last vertex, a wall being the polygon between them.
  Boundary one = in.boundary;
  one.sides[1] = {2};
  CHECK(refusal(in.surface, one, plane) == "the east side must have at least 2 vertices, not 1");
  Boundary none = in.boundary;
  none.sides[0].clear();
  CHECK(refusal(in.surface, none, plane) == "the south side must have at least 2 vertices, not 0");
  CHECK(refusal(Mesh{}, Boundary{}, 0.0f) == "the south side must have at least 2 vertices, not 0");

  // Consecutive sides share their corner: here the west side's last is not the south side's first, then the east
  // side's is not the north side's.
  Boundary apart = in.boundary;
  apart.sides[3] = {6, 1};
  CHECK(refusal(in.surface, apart, plane) ==
        "the west side ends at vertex 1, but the south side starts at vertex 0: consecutive sides must share their "
        "corner");
  apart = in.boundary;
  apart.sides[1] = {2, 4};
  CHECK(refusal(in.surface, apart, plane) ==
        "the east side ends at vertex 4, but the north side starts at vertex 3: consecutive sides must share their "
        "corner");

  // Every vertex number must be one of the surface's, which has vertices 0 to 6: the north-west corner as vertex 7, and
  // as the largest number there is.
  Boundary outside = in.boundary;
  outside.sides[2] = {3, 4, 5, 7};
  outside.sides[3] = {7, 0};
  CHECK(refusal(in.surface, outside, plane) == "the north side names vertex 7, but the surface has 7 vertices");
  outside.sides[2] = {3, 4, 5, 4294967295U};
  outside.sides[3] = {4294967295U, 0};
  CHECK(refusal(in.surface, outside, plane) ==
        "the north side names vertex 4294967295, but the surface has 7 vertices");

  // A wall is flat: the south and north sides keep their y and the east and west sides their x, exactly.
  Mesh slanted = in.surface;
  slanted.vertices[1].y = 1.0f;
  CHECK(refusal(slanted, in.boundary, plane) ==
        "the south side is not flat: vertex 1 is at y = 1, but vertex 0, the side's first, is at y = 0");
  slanted = in.surface;
  slanted.vertices[3].x = 99.0f;
  CHECK(refusal(slanted, in.boundary, plane) ==
        "the east side is not flat: vertex 3 is at x = 99, but vertex 2, the side's first, is at x = 100");
  slanted = in.surface;
  slanted.vertices[5].y = kNaN;
  CHECK(refusal(slanted, in.boundary, plane) ==
        "the north side is not flat: vertex 5 is at y = nan, but vertex 3, the side's first, is at y = 50");

  // The vertices follow one another strictly along the side, in the order the boundary is walked: the south side's x
  // increasing, so neither a vertex beyond the next nor one as far along as the one before is let through.
  Mesh swapped = in.surface;
  swapped.vertices[1].x = 120.0f;
  CHECK(refusal(swapped, in.boundary, plane) ==
        "the south side must run from west to east with x strictly increasing, but vertex 2 at x = 100 follows "
        "vertex 1 at x = 120");
  swapped.vertices[1].x = 0.0f;
  CHECK(refusal(swapped, in.boundary, plane) ==
        "the south side must run from west to east with x strictly increasing, but vertex 1 at x = 0 follows vertex "
        "0 at x = 0");
  // The north side's x decreases, and the east side goes by y, increasing.
  swapped = in.surface;
  swapped.vertices[4].x = 10.0f;
  CHECK(refusal(swapped, in.boundary, plane) ==
        "the north side must run from east to west with x strictly decreasing, but vertex 5 at x = 20 follows "
        "vertex 4 at x = 10");
  swapped = in.surface;
  swapped.vertices[3].y = 0.0f;
  CHECK(refusal(swapped, in.boundary, plane) ==
        "the east side must run from south to north with y strictly increasing, but vertex 3 at y = 0 follows "
        "vertex 2 at y = 0");
  // An outline walked clockwise, the south side from east to west and so on round, has every corner and every wall flat
  // and would turn each wall inside out: its first side is refused for running the wrong way.
  Boundary clockwise;
  clockwise.sides[0] = {2, 1, 0};
  clockwise.sides[1] = {0, 6};
  clockwise.sides[2] = {6, 5, 4, 3};
  clockwise.sides[3] = {3, 2};
  CHECK(refusal(in.surface, clockwise, plane) ==
        "the south side must run from west to east with x strictly increasing, but vertex 1 at x = 30 follows "
        "vertex 2 at x = 100");

  // The plane is finite and below every vertex of the surface, on a side or not, a wall having to reach down from each
  // top vertex: here it meets vertex 2, at 5; then an eighth vertex inside the outline lies under it; then one has no
  // height at all.
  CHECK(refusal(in.surface, in.boundary, kNaN) == "the base plane must be finite, not nan");
  CHECK(refusal(in.surface, in.boundary, -std::numeric_limits<float>::infinity()) ==
        "the base plane must be finite, not -inf");
  CHECK(refusal(in.surface, in.boundary, 5.0f) == "the base plane at 5 is not below the surface: vertex 2 is at 5");
  Mesh sunk = in.surface;
  sunk.vertices.push_back({50, 25, -9});
  CHECK(refusal(sunk, in.boundary, plane) == "the base plane at -4 is not below the surface: vertex 7 is at -9");
  Mesh holed = in.surface;
  holed.vertices[5].z = kNaN;
  CHECK(refusal(holed, in.boundary, plane) == "the base plane at -4 is not below the surface: vertex 5 is at nan");
}

TEST_CASE("solidMesh refuses a grid surface whose sides were bent out of line", "[solid]") {
  // surfaceMesh gives every side the same x or y all along it and makes them run in order; a surface moved afterwards
  // is refused by the outline's checks, as solidFromSurface refuses it.
  Mesh bent = surfaceMesh(Grid(6, 10.0f), 3, 2, 100, 50, 1, 1, 0);
  bent.vertices[4].y = 1.0f;  // the middle of the south side, the last row
  REQUIRE_THROWS_MATCHES(
      solidMesh(bent, 3, 2, -5.0f), std::invalid_argument,
      Message("the south side is not flat: vertex 4 is at y = 1, but vertex 3, the side's first, is at y = 0"));
}
