#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <sys/resource.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <initializer_list>
#include <limits>
#include <map>
#include <numbers>
#include <optional>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include "IslandFixture.h"
#include "mesh/Footprint.h"
#include "mesh/Predicates.h"
#include "mesh/ProfileSimplifier.h"
#include "mesh/Simplifier.h"
#include "mesh/SolidMesh.h"

using Catch::Matchers::Message;
using demdl::mesh::BlockFootprint;
using demdl::mesh::blockFootprint;
using demdl::mesh::BlockRect;
using demdl::mesh::BlockResult;
using demdl::mesh::BlockSimplifier;
using demdl::mesh::Boundary;
using demdl::mesh::checkSolid;
using demdl::mesh::footprintCells;
using demdl::mesh::GridPoint;
using demdl::mesh::inCircle;
using demdl::mesh::kChunkCells;
using demdl::mesh::kMostChunkCells;
using demdl::mesh::Mesh;
using demdl::mesh::MeshCheck;
using demdl::mesh::Piece;
using demdl::mesh::PieceOutline;
using demdl::mesh::SimplifiedSurface;
using demdl::mesh::simplifyGrid;
using demdl::mesh::simplifyProfile;
using demdl::mesh::solidFromSurface;
using demdl::mesh::solidTriangleCount;
using demdl::mesh::surfaceMesh;
using demdl::mesh::Triangle;
using demdl::mesh::Vec3;

// A grid is written row by row from its north-west corner, as resampleQuadratic gives it: sample (i, j), column i from
// the west and row j from the north. The guarantees of spec 4.3 are checked on what simplifyGrid returns by
// checkGuarantees below, by roads that do not go through the code under test: a vertex's sample from its position in
// the test's frame, the surface over a sample from the triangle found to hold it, the closed solid from
// solidFromSurface and checkSolid. Where a case expects a count, a vertex or a side, the reason is worked out beside
// it.

namespace {

using Grid = std::vector<float>;

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

// Heights from low up to, not including, high in steps of (high - low) / 100000, drawn from std::mt19937, whose output
// the standard fixes, so the grids are the same on every machine; the standard distributions are not, and are not
// used.
Grid randomGrid(std::mt19937& rng, int cols, int rows, double low, double high) {
  return makeGrid(cols, rows,
                  [&](int, int) { return low + (high - low) * static_cast<double>(rng() % 100000) / 100000.0; });
}

// 64 x 48 random heights in [0, 100) from seed 11.
Grid randomGrid64x48() {
  std::mt19937 rng(11);
  return randomGrid(rng, 64, 48, 0, 100);
}

// The 64 x 48 "fractal" grid, or a grid of another size from the same formula: three waves, 50 m high over 64 columns,
// 20 m over 48 rows and 5 m over 16 steps on the diagonal. Its curvature asks for a vertex every few samples at a
// tolerance of 2 m, so most samples are left out.
Grid fractalGrid(int cols = 64, int rows = 48) {
  constexpr double pi = std::numbers::pi;
  return makeGrid(cols, rows, [](int i, int j) {
    return 50 * std::sin(2 * pi * i / 64) + 20 * std::sin(2 * pi * j / 48) + 5 * std::sin(2 * pi * (i + j) / 16);
  });
}

// The most this process has held in memory so far, in MB. getrusage counts it in bytes on macOS and in kilobytes on
// Linux and the BSDs. It only ever rises.
double peakResidentMB() {
  rusage usage{};
  getrusage(RUSAGE_SELF, &usage);
#if defined(__APPLE__)
  return static_cast<double>(usage.ru_maxrss) / (1024.0 * 1024.0);
#else
  return static_cast<double>(usage.ru_maxrss) / 1024.0;
#endif
}

// The test's frame: columns kCellX apart and rows kCellY apart in model units, no exaggeration and no scaling, and a
// level below every height used here, so that sample (i, j) of a grid of `rows` rows is placed at x = kCellX i,
// y = kCellY (rows - 1 - j) and z = its height. All of them are whole numbers or a float's own value, so the positions
// are exact and so is the barycentric test of surfaceHeightAt. The cells are not square, so that a mix-up of the axes
// shows.
constexpr double kCellX = 2.0;
constexpr double kCellY = 3.0;
constexpr double kLevel = -1e6;

SimplifiedSurface simplify(const Grid& heights, int cols, int rows, double tolerance, int chunkCells = kChunkCells) {
  return simplifyGrid(heights, cols, rows, tolerance, kCellX * (cols - 1), kCellY * (rows - 1), 1, 1, kLevel,
                      chunkCells);
}

struct Sample {
  int i, j;
};

// The sample a vertex stands on in the test's frame, or nullopt when it stands on none.
std::optional<Sample> sampleOf(const Vec3& v, int cols, int rows) {
  const double across = static_cast<double>(v.x) / kCellX;
  const double up = static_cast<double>(v.y) / kCellY;
  if (across != std::floor(across) || up != std::floor(up)) return std::nullopt;
  const Sample at{static_cast<int>(across), rows - 1 - static_cast<int>(up)};
  if (at.i < 0 || at.i >= cols || at.j < 0 || at.j >= rows) return std::nullopt;
  return at;
}

std::string describe(const Sample& at) { return "(" + std::to_string(at.i) + ", " + std::to_string(at.j) + ")"; }

// Where vertex v stands, as "(i, j)", for the cases that look for one sample.
std::string whereIs(const Mesh& mesh, std::size_t v, int cols, int rows) {
  if (v >= mesh.vertices.size()) return "no vertex " + std::to_string(v);
  const std::optional<Sample> at = sampleOf(mesh.vertices[v], cols, rows);
  return at ? describe(*at) : "on no sample";
}

std::string whereIs(const SimplifiedSurface& s, std::size_t v, int cols, int rows) {
  return whereIs(s.surface, v, cols, rows);
}

// The samples a list of vertices stands on, in its order, as "(i, j) (i, j) ...".
std::string samplesOf(const SimplifiedSurface& s, const std::vector<std::uint32_t>& vertices, int cols, int rows) {
  std::string found;
  for (const std::uint32_t v : vertices) {
    const std::optional<Sample> at = sampleOf(s.surface.vertices.at(v), cols, rows);
    found += (found.empty() ? "" : " ") + (at ? describe(*at) : std::string("off the samples"));
  }
  return found;
}

// The samples side `side` of the boundary (0 south, 1 east, 2 north, 3 west) stands on, in the order it lists them.
std::string sideSamples(const SimplifiedSurface& s, std::size_t side, int cols, int rows) {
  return samplesOf(s, s.boundary.sides[side], cols, rows);
}

// (b - a) x (c - a), its z, in double: positive when a, b, c run counter-clockwise seen from above.
double turn(double ax, double ay, double bx, double by, double cx, double cy) {
  return (bx - ax) * (cy - ay) - (by - ay) * (cx - ax);
}

double turn(const Vec3& a, const Vec3& b, const Vec3& c) { return turn(a.x, a.y, b.x, b.y, c.x, c.y); }

// Which triangle holds each sample, sample (i, j) at j cols + i: the first, in the surface's order, whose closed area
// holds the sample's position in the test's frame, by barycentric weights in double that count a point on an edge as
// inside; -1 where none does, which is a gap. at[v] is the sample vertex v stands on, and a triangle is tried only on
// the samples of its bounding box, so that a grid of some size is checked in time. The frame's coordinates are whole
// numbers, so the test is exact.
std::vector<std::int64_t> trianglesHolding(const Mesh& mesh, const std::vector<Sample>& at, int cols, int rows) {
  std::vector<std::int64_t> holder(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), -1);
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const Triangle& tri = mesh.triangles[t];
    const Vec3& a = mesh.vertices.at(tri.a);
    const Vec3& b = mesh.vertices.at(tri.b);
    const Vec3& c = mesh.vertices.at(tri.c);
    if (!(turn(a, b, c) > 0)) continue;
    const Sample& p = at.at(tri.a);
    const Sample& q = at.at(tri.b);
    const Sample& r = at.at(tri.c);
    for (int j = std::min({p.j, q.j, r.j}); j <= std::max({p.j, q.j, r.j}); ++j) {
      for (int i = std::min({p.i, q.i, r.i}); i <= std::max({p.i, q.i, r.i}); ++i) {
        std::int64_t& holding = holder[cell(cols, i, j)];
        const double x = kCellX * i;
        const double y = kCellY * (rows - 1 - j);
        if (holding < 0 && turn(x, y, b.x, b.y, c.x, c.y) >= 0 && turn(a.x, a.y, x, y, c.x, c.y) >= 0 &&
            turn(a.x, a.y, b.x, b.y, x, y) >= 0) {
          holding = static_cast<std::int64_t>(t);
        }
      }
    }
  }
  return holder;
}

// The height of the surface over sample (i, j): the plane through the corners of the triangle that holds it, as
// trianglesHolding found it, there. NaN when no triangle holds it.
double surfaceHeightAt(const Mesh& mesh, const std::vector<std::int64_t>& holder, int cols, int rows, int i, int j) {
  const std::int64_t t = holder.at(cell(cols, i, j));
  if (t < 0) return std::numeric_limits<double>::quiet_NaN();
  const Triangle& tri = mesh.triangles[static_cast<std::size_t>(t)];
  const Vec3& a = mesh.vertices[tri.a];
  const Vec3& b = mesh.vertices[tri.b];
  const Vec3& c = mesh.vertices[tri.c];
  const double x = kCellX * i;
  const double y = kCellY * (rows - 1 - j);
  const double wa = turn(x, y, b.x, b.y, c.x, c.y);
  const double wb = turn(a.x, a.y, x, y, c.x, c.y);
  const double wc = turn(a.x, a.y, b.x, b.y, x, y);
  return (wa * a.z + wb * b.z + wc * c.z) / turn(a, b, c);
}

// Whether sample `at` lies on side `side` of the rectangle (0 south, 1 east, 2 north, 3 west), and whether `next`
// follows `at` along it in the order a Boundary walks it: the south side from west to east, the east side from south
// to north, the north side from east to west and the west side from north to south.
bool onSide(std::size_t side, const Sample& at, int cols, int rows) {
  switch (side) {
    case 0: return at.j == rows - 1;
    case 1: return at.i == cols - 1;
    case 2: return at.j == 0;
    default: return at.i == 0;
  }
}

bool follows(std::size_t side, const Sample& at, const Sample& next) {
  switch (side) {
    case 0: return next.i > at.i;
    case 1: return next.j < at.j;
    case 2: return next.i < at.i;
    default: return next.j > at.j;
  }
}

// Where the vertices of a surface in the test's frame stand: sample[v] is the sample vertex v stands on, and
// vertexOn[j cols + i] the vertex that stands on sample (i, j), or -1. Returns what is wrong, or "": a vertex that
// stands on no sample, two on one, or one at another height than its sample's in g.
std::string placeVertices(const Mesh& mesh, const Grid& g, int cols, int rows, std::vector<Sample>& sample,
                          std::vector<std::int64_t>& vertexOn) {
  vertexOn.assign(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), -1);
  sample.assign(mesh.vertices.size(), Sample{0, 0});
  for (std::size_t v = 0; v < mesh.vertices.size(); ++v) {
    const Vec3& p = mesh.vertices[v];
    const std::optional<Sample> at = sampleOf(p, cols, rows);
    if (!at) {
      return "vertex " + std::to_string(v) + " at x = " + std::to_string(p.x) + ", y = " + std::to_string(p.y) +
             " stands on no sample";
    }
    sample[v] = *at;
    const std::size_t k = cell(cols, at->i, at->j);
    if (vertexOn[k] >= 0) {
      return "vertices " + std::to_string(vertexOn[k]) + " and " + std::to_string(v) + " both stand on " +
             describe(*at);
    }
    if (p.z != g[k]) {
      return "vertex " + std::to_string(v) + " on " + describe(*at) + " is at z = " + std::to_string(p.z) +
             ", not the sample's height " + std::to_string(g[k]);
    }
    vertexOn[k] = static_cast<std::int64_t>(v);
  }
  return "";
}

// Asserts the guarantees of spec 4.3 on what simplifyGrid made of the grid g of cols x rows heights at tolerance tol,
// in the test's frame:
// - every vertex stands on a sample, at that sample's height, and no two stand on the same one;
// - the four corners are vertices;
// - each side of the boundary runs along its edge of the rectangle, in the order Boundary asks, from its corner to the
//   next, so that consecutive sides share their corner; and every vertex on an edge of the rectangle is on a side;
// - every triangle faces up, every vertex is used, and the triangles' areas add up to the rectangle's, so that with the
//   closed solid at the end they cover it once, with no gap and no overlap;
// - the surface is within the tolerance of every sample, vertically;
// - the surface closes, with walls on its sides and a base below its lowest point, into a solid checkSolid accepts.
void checkGuarantees(const SimplifiedSurface& s, const Grid& g, int cols, int rows, double tol) {
  INFO(cols << " x " << rows << " at tolerance " << tol);
  const Mesh& mesh = s.surface;
  const auto c = static_cast<std::uint64_t>(cols);
  const auto r = static_cast<std::uint64_t>(rows);
  CHECK(s.gridVertices == c * r);
  CHECK(s.gridTriangles == 2 * (c - 1) * (r - 1));

  std::vector<std::int64_t> vertexOn;
  std::vector<Sample> sample;
  std::string problem = placeVertices(mesh, g, cols, rows, sample, vertexOn);
  REQUIRE(problem == "");

  // The first vertices of the south, east, north and west sides: the south-west, south-east, north-east and north-west
  // corners.
  const std::array<std::int64_t, 4> corners = {
      vertexOn[cell(cols, 0, rows - 1)], vertexOn[cell(cols, cols - 1, rows - 1)], vertexOn[cell(cols, cols - 1, 0)],
      vertexOn[cell(cols, 0, 0)]};
  for (const std::int64_t corner : corners) CHECK(corner >= 0);

  std::vector<bool> onBoundary(mesh.vertices.size(), false);
  for (std::size_t side = 0; side < 4 && problem.empty(); ++side) {
    const std::vector<std::uint32_t>& list = s.boundary.sides[side];
    const std::string name = "side " + std::to_string(side);
    if (list.size() < 2 || list.front() != corners[side] || list.back() != corners[(side + 1) % 4]) {
      problem = name + " does not run from its corner to the next";
      break;
    }
    for (std::size_t k = 0; k < list.size() && problem.empty(); ++k) {
      if (list[k] >= mesh.vertices.size()) {
        problem = name + " names vertex " + std::to_string(list[k]) + ", which there is not";
      } else if (!onSide(side, sample[list[k]], cols, rows)) {
        problem = name + " holds vertex " + std::to_string(list[k]) + " on " + describe(sample[list[k]]) +
                  ", off its edge";
      } else if (k > 0 && !follows(side, sample[list[k - 1]], sample[list[k]])) {
        problem = name + " goes back from " + describe(sample[list[k - 1]]) + " to " + describe(sample[list[k]]);
      } else {
        onBoundary[list[k]] = true;
      }
    }
  }
  for (std::size_t v = 0; v < mesh.vertices.size() && problem.empty(); ++v) {
    const Sample& at = sample[v];
    const bool onEdge = at.i == 0 || at.i == cols - 1 || at.j == 0 || at.j == rows - 1;
    if (onEdge && !onBoundary[v]) {
      problem = "vertex " + std::to_string(v) + " on the edge at " + describe(at) + " is on no side of the boundary";
    }
  }
  CHECK(problem == "");

  std::vector<bool> used(mesh.vertices.size(), false);
  double doubledArea = 0;
  for (std::size_t k = 0; k < mesh.triangles.size() && problem.empty(); ++k) {
    const Triangle& t = mesh.triangles[k];
    if (t.a >= mesh.vertices.size() || t.b >= mesh.vertices.size() || t.c >= mesh.vertices.size()) {
      problem = "triangle " + std::to_string(k) + " names a vertex there is not";
      break;
    }
    const double z = turn(mesh.vertices[t.a], mesh.vertices[t.b], mesh.vertices[t.c]);
    if (!(z > 0)) {
      problem = "triangle " + std::to_string(k) + " of " + describe(sample[t.a]) + ", " + describe(sample[t.b]) +
                " and " + describe(sample[t.c]) + " does not face up: its turn is " + std::to_string(z);
    }
    doubledArea += z;
    used[t.a] = used[t.b] = used[t.c] = true;
  }
  CHECK(problem == "");
  CHECK(doubledArea == 2.0 * kCellX * (cols - 1) * kCellY * (rows - 1));
  CHECK(std::count(used.begin(), used.end(), false) == 0);

  const std::vector<std::int64_t> holder =
      problem.empty() ? trianglesHolding(mesh, sample, cols, rows) : std::vector<std::int64_t>{};
  for (int j = 0; j < rows && problem.empty(); ++j) {
    for (int i = 0; i < cols && problem.empty(); ++i) {
      const double z = surfaceHeightAt(mesh, holder, cols, rows, i, j);
      const double h = g[cell(cols, i, j)];
      if (!(std::fabs(z - h) <= tol + 1e-6)) {
        problem = "the surface over " + describe({i, j}) + " is at " + std::to_string(z) + ", " +
                  std::to_string(std::fabs(z - h)) + " off the sample's height " + std::to_string(h);
      }
    }
  }
  CHECK(problem == "");

  const auto lowest = std::min_element(mesh.vertices.begin(), mesh.vertices.end(),
                                       [](const Vec3& a, const Vec3& b) { return a.z < b.z; });
  REQUIRE(lowest != mesh.vertices.end());
  const Mesh solid = solidFromSurface(mesh, s.boundary, lowest->z - 1.0f);
  CHECK(solid.triangles.size() == solidTriangleCount(s.boundary, mesh.triangles.size()));
  const MeshCheck check = checkSolid(solid);
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK(check.ok());
}

// Says how the sides of the boundary of a grid that is one chunk differ from what simplifyProfile keeps of the grid's
// outer rows and columns at the tolerance, or returns "" when they do not: each row simplified from west to east and
// each column from north to south, as two chunks must for a side they share to keep the same samples from both. The
// boundary lists the north side from east to west and the east side from south to north, so their samples are
// compared reversed.
std::string sideProblem(const SimplifiedSurface& s, const Grid& g, int cols, int rows, double tol) {
  const auto row = [&](int j) {
    std::vector<float> line;
    for (int i = 0; i < cols; ++i) line.push_back(g[cell(cols, i, j)]);
    std::string kept;
    for (const int i : simplifyProfile(line, tol)) kept += (kept.empty() ? "" : " ") + describe({i, j});
    return kept;
  };
  const auto column = [&](int i) {
    std::vector<float> line;
    for (int j = 0; j < rows; ++j) line.push_back(g[cell(cols, i, j)]);
    std::string kept;
    for (const int j : simplifyProfile(line, tol)) kept += (kept.empty() ? "" : " ") + describe({i, j});
    return kept;
  };
  const auto reversed = [&](std::size_t side) {
    std::vector<std::uint32_t> list = s.boundary.sides[side];
    std::reverse(list.begin(), list.end());
    return samplesOf(s, list, cols, rows);
  };
  const std::array<std::string, 4> found = {sideSamples(s, 0, cols, rows), reversed(1), reversed(2),
                                            sideSamples(s, 3, cols, rows)};
  const std::array<std::string, 4> wanted = {row(rows - 1), column(cols - 1), row(0), column(0)};
  constexpr std::array<const char*, 4> name = {"south", "east", "north", "west"};
  for (std::size_t side = 0; side < 4; ++side) {
    if (found[side] != wanted[side]) {
      return std::string("the ") + name[side] + " side holds " + found[side] + ", where its profile keeps " +
             wanted[side];
    }
  }
  return "";
}

// Says where the surface is not a Delaunay triangulation, or returns "" when it is: for every edge between two
// triangles, the vertex across it from one of them lies on or outside the circle through that one's corners, as
// inCircle finds it on the samples' columns and rows, counted from the south so that the triangles turn
// counter-clockwise. (On or outside: four samples on one circle may be joined either way.)
std::string delaunayProblem(const Mesh& mesh, int cols, int rows) {
  std::vector<GridPoint> at;
  for (const Vec3& v : mesh.vertices) {
    const std::optional<Sample> sample = sampleOf(v, cols, rows);
    if (!sample) return "a vertex stands on no sample";
    at.push_back({sample->i, rows - 1 - sample->j});
  }
  const auto point = [&](std::uint32_t v) { return at.at(v); };
  const auto where = [&](std::uint32_t v) { return describe({at.at(v).x, rows - 1 - at.at(v).y}); };
  std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> opposite;  // directed edge to the third corner
  for (const Triangle& t : mesh.triangles) {
    opposite[{t.a, t.b}] = t.c;
    opposite[{t.b, t.c}] = t.a;
    opposite[{t.c, t.a}] = t.b;
  }
  for (const auto& [edge, third] : opposite) {
    const auto across = opposite.find({edge.second, edge.first});
    if (across == opposite.end()) continue;  // an edge of the outline
    if (inCircle(point(edge.first), point(edge.second), point(third), point(across->second)) > 0) {
      return "the vertex across the edge from " + where(edge.first) + " to " + where(edge.second) +
             " lies inside the circle of the triangle on the other side";
    }
  }
  return "";
}

// Where the lattice of chunks cuts a run of `cells` cells: every chunkCells cells from the start, and at the end, where
// the last chunk is cut short. 19 cells in chunks of 8 are cut at 0, 8, 16 and 19.
std::vector<int> cuts(int cells, int chunkCells) {
  std::vector<int> at;
  for (int k = 0; k < cells; k += chunkCells) at.push_back(k);
  at.push_back(cells);
  return at;
}

// A chunk of the lattice simplifyGrid cuts a grid into, chunkCells cells a side from the north-west corner (spec 4.5):
// the samples from column west to column east and from row north to row south, its sides included. It is chunk (a, b)
// of the lattice, a counted from the west and b from the north.
struct Chunk {
  int a, b;
  int west, north, east, south;
};

// The chunks of a grid of cols x rows samples, row by row from the north and along each row from the west, the order
// simplifyGrid takes them in.
std::vector<Chunk> latticeOf(int cols, int rows, int chunkCells) {
  const std::vector<int> across = cuts(cols - 1, chunkCells);
  const std::vector<int> down = cuts(rows - 1, chunkCells);
  std::vector<Chunk> chunks;
  for (std::size_t b = 0; b + 1 < down.size(); ++b) {
    for (std::size_t a = 0; a + 1 < across.size(); ++a) {
      chunks.push_back({static_cast<int>(a), static_cast<int>(b), across[a], down[b], across[a + 1], down[b + 1]});
    }
  }
  return chunks;
}

std::string chunkName(const Chunk& c) { return "chunk (" + std::to_string(c.a) + ", " + std::to_string(c.b) + ")"; }

// Whether chunk c holds sample `at`, on its sides or inside them.
bool holds(const Chunk& c, const Sample& at) {
  return c.west <= at.i && at.i <= c.east && c.north <= at.j && at.j <= c.south;
}

// What the triangles of each chunk have along each of its sides: by chunk, in the order of `chunks`, and by side (0
// south, 1 east, 2 north, 3 west), the samples their corners stand on there, as "(i, j) (i, j) ...", from west to east
// along a row and from north to south along a column, whichever way the chunk's outline walks the side. A triangle is
// the chunk's that holds its three corners; `problem` names a triangle with a corner on no sample, or that no one
// chunk holds, which crosses a seam.
struct ChunkSideVertices {
  std::vector<std::array<std::string, 4>> along;
  std::string problem;
};

ChunkSideVertices verticesAlongChunkSides(const SimplifiedSurface& s, int cols, int rows,
                                          const std::vector<Chunk>& chunks) {
  ChunkSideVertices found;
  std::vector<std::array<std::set<int>, 4>> on(chunks.size());  // the columns along a row, the rows along a column
  for (std::size_t t = 0; t < s.surface.triangles.size() && found.problem.empty(); ++t) {
    const Triangle& tri = s.surface.triangles[t];
    const std::array<std::uint32_t, 3> vertex = {tri.a, tri.b, tri.c};
    std::array<Sample, 3> corner{};
    for (std::size_t k = 0; k < 3 && found.problem.empty(); ++k) {
      const std::optional<Sample> at = sampleOf(s.surface.vertices.at(vertex[k]), cols, rows);
      if (at) {
        corner[k] = *at;
      } else {
        found.problem = "triangle " + std::to_string(t) + " has a corner on no sample";
      }
    }
    if (!found.problem.empty()) break;
    const auto chunk = std::find_if(chunks.begin(), chunks.end(), [&](const Chunk& c) {
      return holds(c, corner[0]) && holds(c, corner[1]) && holds(c, corner[2]);
    });
    if (chunk == chunks.end()) {
      found.problem = "triangle " + std::to_string(t) + " of " + describe(corner[0]) + ", " + describe(corner[1]) +
                      " and " + describe(corner[2]) + " lies in no one chunk";
      break;
    }
    std::array<std::set<int>, 4>& sides = on[static_cast<std::size_t>(chunk - chunks.begin())];
    for (const Sample& at : corner) {
      if (at.j == chunk->south) sides[0].insert(at.i);
      if (at.i == chunk->east) sides[1].insert(at.j);
      if (at.j == chunk->north) sides[2].insert(at.i);
      if (at.i == chunk->west) sides[3].insert(at.j);
    }
  }
  found.along.resize(chunks.size());
  for (std::size_t k = 0; k < chunks.size(); ++k) {
    const std::array<int, 4> line = {chunks[k].south, chunks[k].east, chunks[k].north, chunks[k].west};
    for (std::size_t side = 0; side < 4; ++side) {
      std::string& text = found.along[k][side];
      for (const int position : on[k][side]) {
        const Sample at = side % 2 == 0 ? Sample{position, line[side]} : Sample{line[side], position};
        text += (text.empty() ? "" : " ") + describe(at);
      }
    }
  }
  return found;
}

// What simplifyProfile keeps of side `side` of chunk c (0 south, 1 east, 2 north, 3 west) at the tolerance, as "(i, j)
// (i, j) ...": the side's segment of its row simplified from west to east, or of its column from north to south.
std::string keptAlong(const Grid& g, int cols, const Chunk& c, std::size_t side, double tol) {
  const bool alongRow = side % 2 == 0;
  const int line = std::array{c.south, c.east, c.north, c.west}[side];
  const int first = alongRow ? c.west : c.north;
  const int last = alongRow ? c.east : c.south;
  std::vector<float> heights;
  for (int k = first; k <= last; ++k) heights.push_back(g[alongRow ? cell(cols, k, line) : cell(cols, line, k)]);
  std::string kept;
  for (const int k : simplifyProfile(heights, tol)) {
    kept += (kept.empty() ? "" : " ") + describe(alongRow ? Sample{first + k, line} : Sample{line, first + k});
  }
  return kept;
}

// Says where the chunks of the lattice do not meet as spec 4.4 and 4.5 have them meet, or returns "" when they do:
// every triangle lies in one chunk; two chunks that share a side have the very same vertices along it, so that the seam
// between them closes with no T-junction; and those vertices, like the ones along a side on the window's edge, are the
// samples simplifyProfile keeps of the side, its segment of a row simplified from west to east and of a column from
// north to south, whichever way the chunks' outlines walk it.
std::string chunkSideProblem(const SimplifiedSurface& s, const Grid& g, int cols, int rows, double tol,
                             int chunkCells) {
  const std::vector<Chunk> chunks = latticeOf(cols, rows, chunkCells);
  const ChunkSideVertices found = verticesAlongChunkSides(s, cols, rows, chunks);
  if (!found.problem.empty()) return found.problem;
  constexpr std::array<const char*, 4> name = {"south", "east", "north", "west"};
  const auto across = static_cast<std::size_t>(chunks.back().a) + 1;
  for (std::size_t k = 0; k < chunks.size(); ++k) {
    const Chunk& c = chunks[k];
    // Its east side is the west side of the next chunk along the row, and its south side the north side of the chunk
    // below it.
    if (k % across + 1 < across && found.along[k][1] != found.along[k + 1][3]) {
      return chunkName(c) + " has " + found.along[k][1] + " along its east side, and " + chunkName(chunks[k + 1]) +
             " has " + found.along[k + 1][3] + " along the same side";
    }
    if (k + across < chunks.size() && found.along[k][0] != found.along[k + across][2]) {
      return chunkName(c) + " has " + found.along[k][0] + " along its south side, and " +
             chunkName(chunks[k + across]) + " has " + found.along[k + across][2] + " along the same side";
    }
    for (std::size_t side = 0; side < 4; ++side) {
      const std::string kept = keptAlong(g, cols, c, side, tol);
      if (found.along[k][side] != kept) {
        return chunkName(c) + " has " + found.along[k][side] + " along its " + name[side] +
               " side, where the side's profile keeps " + kept;
      }
    }
  }
  return "";
}

// Whether a vertex stands on sample (i, j).
bool isVertexAt(const Mesh& mesh, int cols, int rows, int i, int j) {
  return std::any_of(mesh.vertices.begin(), mesh.vertices.end(), [&](const Vec3& v) {
    const std::optional<Sample> at = sampleOf(v, cols, rows);
    return at && at->i == i && at->j == j;
  });
}

bool isVertexAt(const SimplifiedSurface& s, int cols, int rows, int i, int j) {
  return isVertexAt(s.surface, cols, rows, i, j);
}

// --- Blocks and footprints (spec 4.5, 5.2, 5.3) ----------------------------------------------------------------------
//
// A BlockSimplifier is held to the same guarantees block by block: each block's triangles, the range of the surface its
// BlockResult gives, and each piece's outline, against what the piece's runs and simplifyProfile say it must be,
// worked out here again from the runs; then the surface against the grid at every sample of the footprint.

using Flags = std::vector<std::uint8_t>;  // footprintCells' result: a flag a cell, row by row from the north-west

std::string where(GridPoint p) { return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

std::string where(const std::vector<GridPoint>& points) {
  std::string text;
  for (const GridPoint& p : points) text += (text.empty() ? "" : " ") + where(p);
  return text;
}

std::string describeBlock(const BlockRect& rect) {
  return "the block of columns " + std::to_string(rect.firstColumn) + " to " + std::to_string(rect.lastColumn) +
         " and rows " + std::to_string(rect.firstRow) + " to " + std::to_string(rect.lastRow);
}

bool same(GridPoint p, GridPoint q) { return p.x == q.x && p.y == q.y; }

// Whether cell (x, y) of a grid cols samples across, a cell on the grid, is in the footprint `cells`.
bool cellIn(const Flags& cells, int cols, int x, int y) {
  return cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(cols - 1) + static_cast<std::size_t>(x)] != 0;
}

// Whether sample (i, j) is a corner of a cell of the footprint: inside a piece, or on its outline.
bool inFootprint(const Flags& cells, int cols, int rows, int i, int j) {
  for (const int y : {j - 1, j}) {
    for (const int x : {i - 1, i}) {
      if (x >= 0 && y >= 0 && x < cols - 1 && y < rows - 1 && cellIn(cells, cols, x, y)) return true;
    }
  }
  return false;
}

// A BlockSimplifier in the test's frame over the grid g of cols x rows heights, which must outlive it.
BlockSimplifier blockSimplifier(const Grid& g, int cols, int rows, double tolerance) {
  return {g, cols, rows, tolerance, kCellX * (cols - 1), kCellY * (rows - 1), 1, 1, kLevel};
}

// The lattice of blocks of blockCols x blockRows cells from the north-west corner of a grid of cols x rows samples, the
// last ones each way cut short, row by row from the north and along each row from the west: the order a
// BlockSimplifier is given them in.
std::vector<BlockRect> blockLattice(int cols, int rows, int blockCols, int blockRows) {
  std::vector<BlockRect> blocks;
  for (int top = 0; top < rows - 1; top += blockRows) {
    for (int left = 0; left < cols - 1; left += blockCols) {
      blocks.push_back({left, top, std::min(left + blockCols, cols - 1), std::min(top + blockRows, rows - 1)});
    }
  }
  return blocks;
}

// A block as the simplifier was given it, with its footprint, and what came back.
struct SimplifiedBlock {
  BlockRect rect;
  BlockFootprint footprint;
  BlockResult result;
};

// A grid simplified block by block over its footprint: the surface, and the blocks in the order they were simplified.
struct SimplifiedFootprint {
  Mesh surface;
  std::vector<SimplifiedBlock> blocks;
};

// The grid g of cols x rows heights simplified at tolerance tol over the footprint `cells`, which footprintCells gave,
// block by block in the order of rects, each block's pieces from blockFootprint with none dropped.
SimplifiedFootprint simplifyFootprint(const Grid& g, const Flags& cells, int cols, int rows, double tol,
                                      const std::vector<BlockRect>& rects) {
  BlockSimplifier simplifier = blockSimplifier(g, cols, rows, tol);
  SimplifiedFootprint s;
  for (const BlockRect& rect : rects) {
    SimplifiedBlock block{rect, blockFootprint(cells, cols, rows, rect, kCellX * kCellY, 0.0), {}};
    block.result = simplifier.simplifyBlock(rect, &block.footprint);
    s.blocks.push_back(std::move(block));
  }
  s.surface = simplifier.takeSurface();
  return s;
}

// The island of IslandFixture.h, its heights filled, and its footprint.
struct Island {
  Grid grid = islandGrid();
  Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows);

  // The island simplified at tolerance tol in blocks of blockCols x blockRows cells.
  [[nodiscard]] SimplifiedFootprint simplified(double tol, int blockCols, int blockRows) const {
    return simplifyFootprint(grid, cells, kIslandCols, kIslandRows, tol,
                             blockLattice(kIslandCols, kIslandRows, blockCols, blockRows));
  }
};

// What a piece's outline must be (spec 5.2, 5.3): the samples its runs keep, in the loop's order from its first sample.
// Each run is simplified by simplifyProfile from its heights in g, a run along a row from west to east and one along a
// column from north to south, whichever way the loop walks it, and its samples are taken in reverse where the loop
// walks it the other way. A run's last sample is the next run's first, and comes with that one.
std::vector<GridPoint> keptOutline(const Piece& piece, const Grid& g, int cols, double tol) {
  std::vector<GridPoint> kept;
  for (std::size_t r = 0; r < piece.runStarts.size(); ++r) {
    const GridPoint from = piece.loop.at(piece.runStarts[r]);
    const GridPoint to = piece.loop.at(r + 1 < piece.runStarts.size() ? piece.runStarts[r + 1] : 0);
    const bool alongRow = from.y == to.y;
    const int first = alongRow ? std::min(from.x, to.x) : std::min(from.y, to.y);
    const int last = alongRow ? std::max(from.x, to.x) : std::max(from.y, to.y);
    std::vector<float> heights;
    for (int k = first; k <= last; ++k) heights.push_back(g[alongRow ? cell(cols, k, from.y) : cell(cols, from.x, k)]);
    std::vector<int> keep = simplifyProfile(heights, tol);
    if (alongRow ? from.x > to.x : from.y > to.y) std::reverse(keep.begin(), keep.end());
    keep.pop_back();
    for (const int k : keep) kept.push_back(alongRow ? GridPoint{first + k, from.y} : GridPoint{from.x, first + k});
  }
  return kept;
}

// A piece's outline cut at its runs' starts: each run's two ends, and the outline's vertices from one end to the other,
// both included, with the samples they stand on.
struct OutlineRun {
  GridPoint from, to;
  std::vector<std::uint32_t> vertices;
  std::vector<GridPoint> samples;

  [[nodiscard]] bool alongRow() const { return from.y == to.y; }
  [[nodiscard]] int line() const { return alongRow() ? from.y : from.x; }
  [[nodiscard]] int low() const { return alongRow() ? std::min(from.x, to.x) : std::min(from.y, to.y); }
  [[nodiscard]] int high() const { return alongRow() ? std::max(from.x, to.x) : std::max(from.y, to.y); }
};

// The runs of a piece's outline as a block's result gives it, its points counted from the block's corner. Every run's
// ends must be among the outline's vertices, as checkGuarantees asks.
std::vector<OutlineRun> runsOf(const Piece& piece, const PieceOutline& outline, const BlockRect& rect) {
  std::vector<OutlineRun> runs;
  const std::size_t n = outline.points.size();
  std::size_t k = 0;  // the outline's vertex at the start of the run
  for (std::size_t r = 0; r < piece.runStarts.size() && n > 0; ++r) {
    OutlineRun run{piece.loop.at(piece.runStarts[r]),
                   piece.loop.at(r + 1 < piece.runStarts.size() ? piece.runStarts[r + 1] : 0),
                   {},
                   {}};
    for (std::size_t steps = 0; steps <= n; ++steps, ++k) {
      const GridPoint p = outline.points.at(k % n);
      const GridPoint at{p.x + rect.firstColumn, p.y + rect.firstRow};
      run.vertices.push_back(outline.vertices.at(k % n));
      run.samples.push_back(at);
      if (steps > 0 && same(at, run.to)) break;
    }
    runs.push_back(std::move(run));
  }
  return runs;
}

// Asserts the guarantees of spec 4.3 inside a footprint (spec 5.2) on what a BlockSimplifier made of the grid g of
// cols x rows heights at tolerance tol, block by block, in the test's frame; `cells` is the footprint every block's
// pieces were taken from, with none dropped:
// - every vertex stands on a sample, at that sample's height, and no two stand on the same one;
// - the blocks' triangles come one block after another, each block's in the range its result gives, each of them in
//   its block and facing up, and every vertex is used;
// - each piece has its outline, and it is what keptOutline says: the samples its runs keep, in the loop's order, each
//   point being its vertex's sample counted from the block's corner; so every vertex of it is a sample on the piece's
//   outline, and no other sample of the loop is a vertex at all;
// - a block's triangles have each edge once at most, and the edges that one of them has and none has the other way
//   round are exactly the edges of its pieces' outlines, each the way the outline runs: so the triangles cover the
//   pieces once, with no gap, no overlap and nothing beyond, and a wall can stand on every outline edge; their areas
//   add up to the pieces' cells', and no triangle's middle lies over a cell out of the footprint;
// - over every sample inside a piece or on its outline the surface is within the tolerance of the sample, vertically,
//   and over no sample beyond the footprint is there a surface at all.
void checkGuarantees(const SimplifiedFootprint& s, const Grid& g, const Flags& cells, int cols, int rows, double tol) {
  INFO(cols << " x " << rows << " at tolerance " << tol << " in " << s.blocks.size() << " blocks");
  const Mesh& mesh = s.surface;
  std::vector<std::int64_t> vertexOn;
  std::vector<Sample> sample;
  std::string problem = placeVertices(mesh, g, cols, rows, sample, vertexOn);
  REQUIRE(problem == "");

  std::vector<bool> used(mesh.vertices.size(), false);
  double doubledArea = 0;
  std::uint64_t pieceCells = 0;
  std::size_t next = 0;  // where the next block's triangles start
  for (const SimplifiedBlock& block : s.blocks) {
    const BlockRect& rect = block.rect;
    const BlockResult& result = block.result;
    INFO(describeBlock(rect));
    REQUIRE(block.footprint.dropped.empty());
    CHECK_FALSE(result.rectangle.has_value());
    CHECK(result.firstTriangle == next);
    REQUIRE(result.firstTriangle + result.triangles <= mesh.triangles.size());
    next = result.firstTriangle + result.triangles;
    pieceCells += block.footprint.cells;

    std::set<std::pair<std::uint32_t, std::uint32_t>> edges;  // the block's triangles' edges, each the way it runs
    for (std::size_t t = result.firstTriangle; t < next && problem.empty(); ++t) {
      const Triangle& tri = mesh.triangles[t];
      const std::array<std::uint32_t, 3> corner = {tri.a, tri.b, tri.c};
      const std::string name = "triangle " + std::to_string(t);
      if (std::any_of(corner.begin(), corner.end(), [&](std::uint32_t v) { return v >= mesh.vertices.size(); })) {
        problem = name + " names a vertex there is not";
        break;
      }
      for (const std::uint32_t v : corner) {
        const Sample& at = sample[v];
        if (at.i < rect.firstColumn || at.i > rect.lastColumn || at.j < rect.firstRow || at.j > rect.lastRow) {
          problem = name + " has a corner on " + describe(at) + ", outside its block";
        }
        used[v] = true;
      }
      const Vec3& a = mesh.vertices[tri.a];
      const Vec3& b = mesh.vertices[tri.b];
      const Vec3& c = mesh.vertices[tri.c];
      const double z = turn(a, b, c);
      if (!(z > 0)) problem = name + " does not face up: its turn is " + std::to_string(z);
      doubledArea += z;
      for (std::size_t k = 0; k < 3; ++k) {
        if (!edges.insert({corner[k], corner[(k + 1) % 3]}).second) {
          problem = "the edge from " + describe(sample[corner[k]]) + " to " + describe(sample[corner[(k + 1) % 3]]) +
                    " is in two triangles the same way";
        }
      }
      // A triangle's middle is strictly inside it, so every cell it touches is in the footprint.
      const double column = (static_cast<double>(a.x) + b.x + c.x) / 3.0 / kCellX;
      const double row = (rows - 1) - (static_cast<double>(a.y) + b.y + c.y) / 3.0 / kCellY;
      const int x = std::clamp(static_cast<int>(std::floor(column)), 0, cols - 2);
      const int y = std::clamp(static_cast<int>(std::floor(row)), 0, rows - 2);
      if (!cellIn(cells, cols, x, y)) {
        problem = name + "'s middle lies over the cell (" + std::to_string(x) + ", " + std::to_string(y) +
                  "), which is out of the footprint";
      }
    }
    REQUIRE(problem == "");

    REQUIRE(result.pieces.size() == block.footprint.pieces.size());
    std::set<std::pair<std::uint32_t, std::uint32_t>> outlineEdges;
    for (std::size_t k = 0; k < result.pieces.size() && problem.empty(); ++k) {
      const Piece& piece = block.footprint.pieces[k];
      const PieceOutline& outline = result.pieces[k];
      const std::string name = "the piece whose loop starts at " + where(piece.loop.at(0));
      const std::vector<GridPoint> wanted = keptOutline(piece, g, cols, tol);
      std::vector<GridPoint> found, points;
      for (const std::uint32_t v : outline.vertices) {
        found.push_back(v < sample.size() ? GridPoint{sample[v].i, sample[v].j} : GridPoint{-1, -1});
      }
      for (const GridPoint& p : outline.points) points.push_back({p.x + rect.firstColumn, p.y + rect.firstRow});
      if (where(found) != where(wanted)) {
        problem = name + " has the outline " + where(found) + ", where its runs keep " + where(wanted);
        break;
      }
      if (where(points) != where(wanted)) {
        problem = name + "'s outline has the points " + where(points) + " for the vertices on " + where(wanted);
        break;
      }
      std::set<std::pair<int, int>> kept;
      for (const GridPoint& p : wanted) kept.insert({p.x, p.y});
      for (const GridPoint& p : piece.loop) {
        if (!kept.contains({p.x, p.y}) && vertexOn[cell(cols, p.x, p.y)] >= 0) {
          problem = name + " has a vertex on " + where(p) + ", which its runs do not keep";
          break;
        }
      }
      for (std::size_t m = 0; m < outline.vertices.size(); ++m) {
        if (!outlineEdges.insert({outline.vertices[m], outline.vertices[(m + 1) % outline.vertices.size()]}).second) {
          problem = name + " has the edge from " + where(found[m]) + " twice";
        }
      }
    }
    REQUIRE(problem == "");

    for (const auto& [a, b] : edges) {
      const bool across = edges.contains({b, a});
      if (across == outlineEdges.contains({a, b})) {
        problem = "the edge from " + describe(sample[a]) + " to " + describe(sample[b]) +
                  (across ? " is an outline's, and has a triangle across it" : " has no triangle across it, and is no "
                                                                               "outline's");
        break;
      }
    }
    for (const auto& [a, b] : outlineEdges) {
      if (!edges.contains({a, b})) {
        problem = "the outline's edge from " + describe(sample.at(a)) + " to " + describe(sample.at(b)) +
                  " is no triangle's";
        break;
      }
    }
    CHECK(problem == "");
    problem.clear();
  }
  CHECK(next == mesh.triangles.size());
  CHECK(std::count(used.begin(), used.end(), false) == 0);
  CHECK(doubledArea == 2.0 * kCellX * kCellY * static_cast<double>(pieceCells));

  const std::vector<std::int64_t> holder = trianglesHolding(mesh, sample, cols, rows);
  for (int j = 0; j < rows && problem.empty(); ++j) {
    for (int i = 0; i < cols && problem.empty(); ++i) {
      const double z = surfaceHeightAt(mesh, holder, cols, rows, i, j);
      const double h = g[cell(cols, i, j)];
      if (!inFootprint(cells, cols, rows, i, j)) {
        if (!std::isnan(z)) problem = "the sample " + describe({i, j}) + " is beyond the footprint, under a triangle";
      } else if (!(std::fabs(z - h) <= tol + 1e-6)) {
        problem = "the surface over " + describe({i, j}) + " is at " + std::to_string(z) + ", " +
                  std::to_string(std::fabs(z - h)) + " off the sample's height " + std::to_string(h);
      }
    }
  }
  CHECK(problem == "");
}

// Says where two blocks that share a side do not meet along it as spec 5.3 has them meet, or returns "" when they do.
// Every run of either block's outlines along the side's line, a row when alongRow and otherwise a column, is either run
// by the other block too, between the same two ends the other way, with the very same vertex numbers in the other
// order; or it is the one block's alone, its coast, and overlaps no run of the other's along the line. matched counts
// the runs the two have in common and matchedVertices their vertices.
std::string sharedSideProblem(const SimplifiedBlock& a, const SimplifiedBlock& b, bool alongRow, int line, int& matched,
                              int& matchedVertices) {
  const auto along = [&](const SimplifiedBlock& block) {
    std::vector<OutlineRun> found;
    for (std::size_t k = 0; k < block.result.pieces.size() && k < block.footprint.pieces.size(); ++k) {
      for (OutlineRun& run : runsOf(block.footprint.pieces[k], block.result.pieces[k], block.rect)) {
        if (run.alongRow() == alongRow && run.line() == line) found.push_back(std::move(run));
      }
    }
    return found;
  };
  const std::vector<OutlineRun> first = along(a), second = along(b);
  const auto twinOf = [](const OutlineRun& run, const std::vector<OutlineRun>& others) {
    return std::find_if(others.begin(), others.end(), [&](const OutlineRun& other) {
      return other.low() == run.low() && other.high() == run.high();
    });
  };
  const auto overlapProblem = [](const OutlineRun& run, const std::vector<OutlineRun>& others) -> std::string {
    for (const OutlineRun& other : others) {
      if (std::max(run.low(), other.low()) < std::min(run.high(), other.high())) {
        return "the run from " + where(run.from) + " to " + where(run.to) + " overlaps the other block's from " +
               where(other.from) + " to " + where(other.to);
      }
    }
    return "";
  };
  for (const OutlineRun& run : first) {
    const auto twin = twinOf(run, second);
    if (twin == second.end()) {
      const std::string problem = overlapProblem(run, second);
      if (!problem.empty()) return problem;
      continue;
    }
    const std::vector<std::uint32_t> reversed(twin->vertices.rbegin(), twin->vertices.rend());
    if (!same(twin->from, run.to) || !same(twin->to, run.from) || reversed != run.vertices) {
      return "the run from " + where(run.from) + " to " + where(run.to) + " has " + where(run.samples) +
             " in one block, and the other has " + where(twin->samples) + " from " + where(twin->from);
    }
    ++matched;
    matchedVertices += static_cast<int>(run.vertices.size());
  }
  for (const OutlineRun& run : second) {
    if (twinOf(run, first) != first.end()) continue;
    const std::string problem = overlapProblem(run, first);
    if (!problem.empty()) return problem;
  }
  return "";
}

// Says where a run keeps other samples than the run between the same two ends keeps in another simplification of the
// grid, or returns "" when none does: a run is simplified from its own samples by the fixed rule, however the grid is
// cut into blocks (spec 5.3). common counts the runs compared.
std::string commonRunProblem(const SimplifiedFootprint& s, const SimplifiedFootprint& reference, int& common) {
  using Key = std::tuple<bool, int, int, int>;
  const auto runs = [](const SimplifiedFootprint& f) {
    std::map<Key, std::vector<std::pair<int, int>>> found;  // by line and ends, the samples in order along the line
    for (const SimplifiedBlock& block : f.blocks) {
      for (std::size_t k = 0; k < block.result.pieces.size() && k < block.footprint.pieces.size(); ++k) {
        for (const OutlineRun& run : runsOf(block.footprint.pieces[k], block.result.pieces[k], block.rect)) {
          std::vector<std::pair<int, int>> samples;
          for (const GridPoint& p : run.samples) samples.push_back({p.x, p.y});
          std::sort(samples.begin(), samples.end());
          found[{run.alongRow(), run.line(), run.low(), run.high()}] = samples;
        }
      }
    }
    return found;
  };
  const auto wanted = runs(reference);
  for (const auto& [key, samples] : runs(s)) {
    const auto twin = wanted.find(key);
    if (twin == wanted.end()) continue;
    ++common;
    if (twin->second != samples) {
      const auto& [alongRow, line, low, high] = key;
      return std::string("the run along ") + (alongRow ? "row " : "column ") + std::to_string(line) + " from " +
             std::to_string(low) + " to " + std::to_string(high) + " keeps other samples in the two";
    }
  }
  return "";
}

// A mask of cols x rows samples drawn from rng: rectangles of data with rectangles of no-data cut out of them, pairs of
// rectangles of data that share a corner sample on either diagonal, or noise, so that coasts, lakes, channels, islets
// and pieces that touch at a vertex all turn up in the footprint. Every draw is made from the engine's output by hand
// and into a variable of its own, since the standard fixes the engine's output but neither a distribution's nor the
// order a call's arguments are worked out in.
Flags randomMask(std::mt19937& rng, int cols, int rows) {
  Flags mask(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), 0);
  const auto draw = [&rng](int below) { return static_cast<int>(rng() % static_cast<unsigned>(below)); };
  const auto fill = [&](int x0, int y0, int w, int h, std::uint8_t value) {
    for (int y = std::max(y0, 0); y < std::min(y0 + h, rows); ++y) {
      for (int x = std::max(x0, 0); x < std::min(x0 + w, cols); ++x) mask[cell(cols, x, y)] = value;
    }
  };
  const int style = draw(3);
  if (style == 0) {
    const int land = 1 + draw(5), water = draw(9), most = std::max(cols, rows);
    for (int k = 0; k < land + water; ++k) {
      const int x = draw(cols), y = draw(rows);
      const int w = 1 + draw(k < land ? most : 4), h = 1 + draw(k < land ? most : 4);
      fill(x, y, w, h, k < land ? 1 : 0);
    }
  } else if (style == 1) {
    for (int pairs = 1 + draw(4); pairs > 0; --pairs) {
      const int x = draw(cols), y = draw(rows), a = 2 + draw(4), b = 2 + draw(4), c = 2 + draw(4), d = 2 + draw(4);
      const bool southEast = draw(2) == 0;
      fill(x, y, a, b, 1);
      // The second shares the first's south-east corner sample as its north-west one, or its south-west corner sample
      // as its north-east one.
      fill(southEast ? x + a - 1 : x - c + 1, y + b - 1, c, d, 1);
    }
  } else {
    const int percent = 40 + draw(58);
    for (std::uint8_t& v : mask) v = draw(100) < percent ? 1 : 0;
  }
  return mask;
}

// Says how a block's rectangle differs from the Boundary of the block's own four sides, or returns "": the south side
// from the block's south-west corner to its south-east one, the east side up to the north-east corner, the north side
// back to the north-west corner and the west side down to the south-west one, each along its edge of the block and
// always onwards.
std::string rectangleProblem(const Mesh& mesh, const Boundary& boundary, const BlockRect& rect, int cols, int rows) {
  const std::array<Sample, 4> corner = {Sample{rect.firstColumn, rect.lastRow}, Sample{rect.lastColumn, rect.lastRow},
                                        Sample{rect.lastColumn, rect.firstRow},
                                        Sample{rect.firstColumn, rect.firstRow}};
  constexpr std::array<const char*, 4> name = {"south", "east", "north", "west"};
  for (std::size_t side = 0; side < 4; ++side) {
    std::vector<Sample> at;
    for (const std::uint32_t v : boundary.sides[side]) {
      const std::optional<Sample> found =
          v < mesh.vertices.size() ? sampleOf(mesh.vertices[v], cols, rows) : std::nullopt;
      if (!found) return std::string("the ") + name[side] + " side has a vertex on no sample";
      at.push_back(*found);
    }
    const Sample from = corner[side], to = corner[(side + 1) % 4];
    if (at.size() < 2 || at.front().i != from.i || at.front().j != from.j || at.back().i != to.i ||
        at.back().j != to.j) {
      return std::string("the ") + name[side] + " side does not run from " + describe(from) + " to " + describe(to);
    }
    for (std::size_t k = 1; k < at.size(); ++k) {
      const Sample& p = at[k - 1];
      const Sample& q = at[k];
      const bool onwards = (side == 0 && q.j == p.j && q.i > p.i) || (side == 1 && q.i == p.i && q.j < p.j) ||
                           (side == 2 && q.j == p.j && q.i < p.i) || (side == 3 && q.i == p.i && q.j > p.j);
      if (!onwards) return std::string("the ") + name[side] + " side goes from " + describe(p) + " to " + describe(q);
    }
  }
  return "";
}

}  // namespace

TEST_CASE("a plane of any slope comes out as its four corners and two triangles", "[simplify]") {
  // Every sample of a plane lies on the plane through the corners, so none is off at any tolerance. The slopes are
  // binary fractions, so that the float heights lie on the plane exactly, except for the last plane's: its heights, up
  // to 1.8 m, are rounded to a float by up to 6e-8 m, so the samples are off the corners' plane by a few times that,
  // and it is held to a tolerance of 1e-5 m.
  struct Case {
    double perColumn, perRow, offset, tolerance;
  };
  for (const Case& p : std::vector<Case>{{2, 3, 0, 0.5}, {2, 3, 0, 1e-9}, {-7.5, 0.25, 1000, 1e-9},
                                         {300, -200, 0, 1e-9}, {0.1, -0.3, 0, 1e-5}}) {
    INFO("h = " << p.perColumn << " i + " << p.perRow << " j + " << p.offset << " at tolerance " << p.tolerance);
    const Grid plane = makeGrid(9, 7, [&](int i, int j) { return p.perColumn * i + p.perRow * j + p.offset; });
    const SimplifiedSurface s = simplify(plane, 9, 7, p.tolerance);
    CHECK(s.surface.vertices.size() == 4);
    CHECK(s.surface.triangles.size() == 2);
    CHECK(s.gridVertices == 63);
    CHECK(s.gridTriangles == 96);  // 2 x 8 x 6
    checkGuarantees(s, plane, 9, 7, p.tolerance);
  }
}

TEST_CASE("a spike is a vertex until the tolerance reaches its height", "[simplify]") {
  // A 9 x 7 grid of 0 with 10 at (4, 3), the middle of the diagonal from the north-west corner to the south-east one.
  // The sides are flat, so they keep their corners only, and the spike, 10 off the plane of the corners, is the first
  // sample inserted: vertex 4. It falls on the edge between the first two triangles, which it splits into four, one to
  // each side of the rectangle. Each of those holds samples near the spike more than 5 under its slope (7.5 beside the
  // spike to the west and east, 6.67 a row from it to the north and south), so each gives one more vertex, a sample
  // next to the spike, after which every slope runs down to 0 within a step of it: at most 9 vertices.
  const Grid spike = makeGrid(9, 7, [](int i, int j) { return i == 4 && j == 3 ? 10 : 0; });
  const SimplifiedSurface five = simplify(spike, 9, 7, 5);
  CHECK(whereIs(five, 4, 9, 7) == "(4, 3)");
  CHECK(five.surface.vertices.size() <= 9);
  checkGuarantees(five, spike, 9, 7, 5);

  // At a tolerance of 10 the spike's own error, 10, is allowed, so only the corners are left.
  const SimplifiedSurface ten = simplify(spike, 9, 7, 10);
  CHECK(ten.surface.vertices.size() == 4);
  CHECK(ten.surface.triangles.size() == 2);
  checkGuarantees(ten, spike, 9, 7, 10);
}

TEST_CASE("a spike on a side is kept by the side's profile", "[simplify]") {
  // A 9 x 7 grid of 0 with 10 at (4, 6) on the south side, at tolerance 5. The south side's profile is 0 0 0 0 10 0 0 0
  // 0, which simplifyProfile splits at the spike (10 off the chord, over 5), then at 3 (the chord from 0 to the spike
  // passes 7.5 over it, more than the 5 and 2.5 over columns 2 and 1, and over 5) and at 5 the same way, and nowhere
  // else: the chords from the corners to columns 3 and 5 lie flat on the 0s. The other sides are flat.
  const Grid spike = makeGrid(9, 7, [](int i, int j) { return i == 4 && j == 6 ? 10 : 0; });
  const SimplifiedSurface s = simplify(spike, 9, 7, 5);
  CHECK(sideSamples(s, 0, 9, 7) == "(0, 6) (3, 6) (4, 6) (5, 6) (8, 6)");
  CHECK(sideSamples(s, 1, 9, 7) == "(8, 6) (8, 0)");
  CHECK(sideSamples(s, 2, 9, 7) == "(8, 0) (0, 0)");
  CHECK(sideSamples(s, 3, 9, 7) == "(0, 0) (0, 6)");
  checkGuarantees(s, spike, 9, 7, 5);
}

TEST_CASE("random heights keep every guarantee from every sample a vertex up to two triangles", "[simplify]") {
  const Grid noise = randomGrid64x48();
  for (const double tol : {0.0, 0.5, 5.0, 50.0, 200.0}) {
    INFO("tolerance " << tol);
    const SimplifiedSurface s = simplify(noise, 64, 48, tol);
    checkGuarantees(s, noise, 64, 48, tol);
    CHECK(sideProblem(s, noise, 64, 48, tol) == "");
    // At 0 every sample is off every plane through others, so all 64 x 48 are vertices. At 200, above the heights'
    // whole range of 100, the corners' two triangles are within the tolerance of everything (spec 4.3).
    if (tol == 0.0) CHECK(s.surface.vertices.size() == 3072);
    if (tol == 200.0) {
      CHECK(s.surface.vertices.size() == 4);
      CHECK(s.surface.triangles.size() == 2);
    }
  }
}

TEST_CASE("a smooth grid is held within its tolerance by under half of its samples", "[simplify]") {
  const Grid fractal = fractalGrid();
  const SimplifiedSurface s = simplify(fractal, 64, 48, 2);
  checkGuarantees(s, fractal, 64, 48, 2);
  CHECK(sideProblem(s, fractal, 64, 48, 2) == "");
  CHECK(s.surface.vertices.size() < 3072 / 2);
}

TEST_CASE("the faces are as round as the samples allow: no vertex lies inside the circle of a face across an edge",
          "[simplify]") {
  // Lawson's flips after every insertion keep the triangulation Delaunay, which keeps slivers out where the samples
  // let it. The guarantees alone would let any triangulation through, however thin its faces.
  const Grid noise = randomGrid64x48();
  for (const double tol : {0.5, 5.0, 50.0}) {
    INFO("random heights at tolerance " << tol);
    CHECK(delaunayProblem(simplify(noise, 64, 48, tol).surface, 64, 48) == "");
  }
  CHECK(delaunayProblem(simplify(fractalGrid(), 64, 48, 2).surface, 64, 48) == "");
  std::mt19937 rng(20261021);
  CHECK(delaunayProblem(simplify(randomGrid(rng, 2, 9, 0, 100), 2, 9, 0).surface, 2, 9) == "");
}

TEST_CASE("a tolerance far below the heights' resolution makes every deviating sample a vertex, and ends",
          "[simplify]") {
  // The random heights are floats on steps of 0.001, so no sample lies within 1e-9 of a plane through others: every
  // one is inserted, and then there is nothing left to insert. (Were one ever to coincide, a new seed would be the
  // remedy.)
  const Grid noise = randomGrid64x48();
  const SimplifiedSurface s = simplify(noise, 64, 48, 1e-9);
  CHECK(s.surface.vertices.size() == 3072);
  checkGuarantees(s, noise, 64, 48, 1e-9);

  // At tolerance 0 on the smooth grid nearly every sample comes in too, and many faces end with their own corners as
  // the only samples off their plane by anything, by the rounding of the plane at them. A corner is never inserted
  // again, so this ends, with every vertex a sample of its own.
  const Grid fractal = fractalGrid();
  checkGuarantees(simplify(fractal, 64, 48, 0), fractal, 64, 48, 0);
}

TEST_CASE("a sample its side's profile drops is never inserted, even where rounding puts it a hair over the tolerance",
          "[simplify]") {
  // Spec 4.4: the interior never adds a vertex to a side. A 4 x 3 grid of 0 with the south side 0 1 1 1, at a
  // tolerance of 2/3. The chord from the south-west corner, 0, to the south-east one, 1, passes 1/3 and 2/3 at columns
  // 1 and 2, so their heights are 2/3 and 1/3 off it, and simplifyProfile, working without a division, drops both: an
  // error equal to the tolerance is allowed. The plane of the face over that edge, worked out in double, comes to 1/3
  // rounded down at column 1, which leaves that sample 1 - 1/3 a hair over 2/3 rounded; it lies on the side, so it is
  // not looked at, and the corners are all there is. Every other sample is within 2/3 of the corners' two faces.
  const Grid edge = makeGrid(4, 3, [](int i, int j) { return j == 2 && i > 0 ? 1 : 0; });
  const SimplifiedSurface s = simplify(edge, 4, 3, 2.0 / 3.0);
  CHECK(s.surface.vertices.size() == 4);
  CHECK(sideSamples(s, 0, 4, 3) == "(0, 2) (3, 2)");
  checkGuarantees(s, edge, 4, 3, 2.0 / 3.0);
}

TEST_CASE("a long side is built into the outline in memory for its faces alone", "[simplify]") {
  // A 2 x 2049 ribbon of random heights at tolerance 0, one standard chunk long: both long sides keep every sample.
  // Inserting the second side's samples one after another, each new one takes over its predecessor's fan of edges
  // across to the first side, so the outline costs 1024 x 1024 flips, each rewriting two faces. Nothing of them needs
  // keeping while the outline is built, every face being scanned once it is done; the first version kept a note of
  // every face it wrote, and peaked 25 MB higher here (98 MB at 2 x 4097, 1.3 GB at 2 x 16385), for a surface of 4098
  // vertices and 4096 triangles. Without the notes the run grows by about 1 MB. The peak resident set only rises, so
  // the check sees this run's growth only beyond what the process held before: run on its own, as ctest runs every
  // case, it sees all of it.
  std::mt19937 rng(20261021);
  const Grid noise = randomGrid(rng, 2, 2049, 0, 100);
  const double before = peakResidentMB();
  const SimplifiedSurface s = simplify(noise, 2, 2049, 0);
  const double grewMB = peakResidentMB() - before;
  CHECK(s.surface.vertices.size() == 4098);
  CHECK(s.surface.triangles.size() == 4096);
  CHECK(grewMB < 8);
  const auto lowest = std::min_element(s.surface.vertices.begin(), s.surface.vertices.end(),
                                       [](const Vec3& a, const Vec3& b) { return a.z < b.z; });
  CHECK(checkSolid(solidFromSurface(s.surface, s.boundary, lowest->z - 1.0f)).ok());
}

TEST_CASE("grids of two columns or two rows give a box or a ribbon", "[simplify]") {
  // On a plane only the corners are left; on random heights at tolerance 0 every sample is kept, all of them on the two
  // long sides, and the ribbon between them is 2 (n - 1) triangles, as many as the full grid's.
  std::mt19937 rng(20261021);
  for (const auto& [cols, rows] : std::vector<std::pair<int, int>>{{2, 2}, {2, 9}, {9, 2}}) {
    INFO(cols << " x " << rows);
    const Grid plane = makeGrid(cols, rows, [](int i, int j) { return 2 * i + 3 * j; });
    const SimplifiedSurface flat = simplify(plane, cols, rows, 0.5);
    CHECK(flat.surface.vertices.size() == 4);
    CHECK(flat.surface.triangles.size() == 2);
    checkGuarantees(flat, plane, cols, rows, 0.5);

    const Grid noise = randomGrid(rng, cols, rows, 0, 100);
    const SimplifiedSurface ribbon = simplify(noise, cols, rows, 0);
    CHECK(ribbon.surface.vertices.size() == static_cast<std::size_t>(cols * rows));
    CHECK(ribbon.surface.triangles.size() == static_cast<std::size_t>(2 * (cols - 1) * (rows - 1)));
    checkGuarantees(ribbon, noise, cols, rows, 0);
  }
}

TEST_CASE("the same grid, tolerance and chunk size give the same surface", "[simplify]") {
  // Chunks of another size make another surface, but each size makes the same one every time.
  const Grid fractal = fractalGrid();
  for (const int chunkCells : {kChunkCells, 16, 7}) {
    INFO("chunks of " << chunkCells << " cells");
    const SimplifiedSurface first = simplify(fractal, 64, 48, 2, chunkCells);
    const SimplifiedSurface second = simplify(fractal, 64, 48, 2, chunkCells);
    REQUIRE(first.surface.vertices.size() == second.surface.vertices.size());
    REQUIRE(first.surface.triangles.size() == second.surface.triangles.size());
    for (std::size_t v = 0; v < first.surface.vertices.size(); ++v) {
      const Vec3& a = first.surface.vertices[v];
      const Vec3& b = second.surface.vertices[v];
      CHECK((a.x == b.x && a.y == b.y && a.z == b.z));
    }
    for (std::size_t t = 0; t < first.surface.triangles.size(); ++t) {
      const Triangle& a = first.surface.triangles[t];
      const Triangle& b = second.surface.triangles[t];
      CHECK((a.a == b.a && a.b == b.b && a.c == b.c));
    }
    CHECK(first.boundary.sides == second.boundary.sides);
  }
}

TEST_CASE("every side is simplified from west to east or from north to south, whichever way the boundary walks it",
          "[simplify]") {
  // simplifyProfile settles a tie by the order its heights come in, so a side shared by two chunks keeps the same
  // samples from both only if each simplifies it the same way: a row from west to east, a column from north to south,
  // whichever way the outline walks it. On a 5 x 5 grid
  // every side's profile is 0 10 0 10 0, 0 inside. At tolerance 9.9 the chord between the ends is 10 below both 10s,
  // and the first, the one nearer the side's start, is split at; the chord from it to the far end then passes 6.67
  // from the two heights left between them, within the tolerance. So the kept samples are the first two and the last,
  // counted from the west or from the north; counted from the east or the south they would be the first, the fourth
  // and the last. The boundary lists the north side from east to west and the east side from south to north, so there
  // the kept samples come reversed.
  const Grid ring = makeGrid(5, 5, [](int i, int j) {
    const bool onRing = i == 0 || i == 4 || j == 0 || j == 4;
    return onRing && (i + j) % 2 == 1 ? 10 : 0;
  });
  const SimplifiedSurface s = simplify(ring, 5, 5, 9.9);
  CHECK(sideSamples(s, 0, 5, 5) == "(0, 4) (1, 4) (4, 4)");  // south, columns 0, 1, 4 from the west
  CHECK(sideSamples(s, 1, 5, 5) == "(4, 4) (4, 1) (4, 0)");  // east, rows 0, 1, 4 from the north, reversed
  CHECK(sideSamples(s, 2, 5, 5) == "(4, 0) (1, 0) (0, 0)");  // north, columns 0, 1, 4 from the west, reversed
  CHECK(sideSamples(s, 3, 5, 5) == "(0, 0) (0, 1) (0, 4)");  // west, rows 0, 1, 4 from the north
  checkGuarantees(s, ring, 5, 5, 9.9);
}

TEST_CASE("of samples equally far off, the one in the smallest row is inserted first, then the smallest column",
          "[simplify]") {
  // Flat 9 x 7 grids of 0 with spikes of 10, all equally far off the corners' plane, at tolerance 5. The first two
  // triangles are cut by the diagonal from (0, 0) to (8, 6), the north-east one holding the samples with j <= 0.75 i.
  // The first sample inserted after the corners, vertex 4, is the spike the rule picks.
  const auto spikes = [](std::initializer_list<std::pair<int, int>> at) {
    return makeGrid(9, 7, [at](int i, int j) {
      return std::find(at.begin(), at.end(), std::pair{i, j}) != at.end() ? 10 : 0;
    });
  };
  // In two triangles: (1, 3) in the south-west one, (7, 4) in the north-east one, which is the first triangle made.
  const Grid apart = spikes({{7, 4}, {1, 3}});
  const SimplifiedSurface first = simplify(apart, 9, 7, 5);
  CHECK(whereIs(first, 4, 9, 7) == "(1, 3)");
  checkGuarantees(first, apart, 9, 7, 5);
  // In one triangle, the north-east: row 1 before row 2 although column 3 is further west, and column 5 before 7.
  const Grid together = spikes({{7, 1}, {3, 2}, {5, 1}});
  const SimplifiedSurface second = simplify(together, 9, 7, 5);
  CHECK(whereIs(second, 4, 9, 7) == "(5, 1)");
  checkGuarantees(second, together, 9, 7, 5);
}

TEST_CASE("the corners come first, then the rest of the boundary, then the interior", "[simplify]") {
  const Grid noise = randomGrid64x48();
  const SimplifiedSurface s = simplify(noise, 64, 48, 5);
  checkGuarantees(s, noise, 64, 48, 5);
  const std::size_t count = s.surface.vertices.size();
  REQUIRE(count > 4);
  std::vector<std::string> firstFour;
  for (std::size_t v = 0; v < 4; ++v) firstFour.push_back(whereIs(s, v, 64, 48));
  std::sort(firstFour.begin(), firstFour.end());
  CHECK(firstFour == std::vector<std::string>{"(0, 0)", "(0, 47)", "(63, 0)", "(63, 47)"});

  // Once the first vertex off the edge has come, no vertex on the edge follows.
  std::size_t interior = 0;
  std::size_t lastOnEdge = 0;
  for (std::size_t v = 0; v < count; ++v) {
    const Sample at = *sampleOf(s.surface.vertices[v], 64, 48);
    const bool onEdge = at.i == 0 || at.i == 63 || at.j == 0 || at.j == 47;
    if (onEdge) {
      lastOnEdge = v;
    } else if (interior == 0) {
      interior = v;
    }
  }
  // Side vertices besides the corners, and interior ones, so that the order is put to the test.
  REQUIRE(interior > 4);
  CHECK(lastOnEdge < interior);
}

TEST_CASE("the vertices stand where surfaceMesh puts the same samples, and the tolerance is in metres", "[simplify]") {
  // The fractal grid over 1234.5 m by 987.25 m at --z-scale 2.5 and --scale 0.02, with the level at -10 m, above its
  // lowest heights (it dips to about -75 m), so that some vertices are lifted to it as surfaceMesh lifts them. Each
  // vertex must be a vertex of surfaceMesh's full grid exactly, the corners among them; and since the tolerance is in
  // the heights' metres, the frame changes where the vertices go but not which samples are kept or how they are joined.
  const Grid fractal = fractalGrid();
  const SimplifiedSurface placed = simplifyGrid(fractal, 64, 48, 2, 1234.5, 987.25, 2.5, 0.02, -10);
  const Mesh full = surfaceMesh(fractal, 64, 48, 1234.5, 987.25, 2.5, 0.02, -10);
  std::map<std::pair<float, float>, float> heightAt;
  for (const Vec3& v : full.vertices) heightAt[{v.x, v.y}] = v.z;
  std::string problem;
  for (std::size_t v = 0; v < placed.surface.vertices.size() && problem.empty(); ++v) {
    const Vec3& p = placed.surface.vertices[v];
    const auto found = heightAt.find({p.x, p.y});
    if (found == heightAt.end() || found->second != p.z) {
      problem = "vertex " + std::to_string(v) + " at (" + std::to_string(p.x) + ", " + std::to_string(p.y) + ", " +
                std::to_string(p.z) + ") is not a vertex of the full grid";
    }
  }
  CHECK(problem == "");
  const auto hasVertex = [&](const Vec3& corner) {
    return std::any_of(placed.surface.vertices.begin(), placed.surface.vertices.end(),
                       [&](const Vec3& v) { return v.x == corner.x && v.y == corner.y && v.z == corner.z; });
  };
  for (const std::size_t corner : {0, 63, 64 * 47, 64 * 48 - 1}) CHECK(hasVertex(full.vertices[corner]));
  CHECK(std::any_of(placed.surface.vertices.begin(), placed.surface.vertices.end(),
                    [](const Vec3& v) { return v.z == static_cast<float>(-10.0 * 2.5 * 0.02); }));

  const SimplifiedSurface plain = simplify(fractal, 64, 48, 2);
  REQUIRE(placed.surface.triangles.size() == plain.surface.triangles.size());
  for (std::size_t t = 0; t < plain.surface.triangles.size(); ++t) {
    const Triangle& a = placed.surface.triangles[t];
    const Triangle& b = plain.surface.triangles[t];
    CHECK((a.a == b.a && a.b == b.b && a.c == b.c));
  }
  CHECK(placed.boundary.sides == plain.boundary.sides);

  const auto lowest = std::min_element(placed.surface.vertices.begin(), placed.surface.vertices.end(),
                                       [](const Vec3& a, const Vec3& b) { return a.z < b.z; });
  const MeshCheck check = checkSolid(solidFromSurface(placed.surface, placed.boundary, lowest->z - 1.0f));
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.zeroAreaTriangles == 0);
  CHECK(check.ok());
}

TEST_CASE("a plane over a lattice of chunks is two triangles a chunk", "[simplify]") {
  // 20 x 20 samples are 19 cells each way, in chunks of 8, 8 and 3 cells: 3 x 3 chunks. Each chunk's sides are
  // straight, so they keep their ends, and its interior lies on the plane of its corners, so each chunk is its four
  // corners and two triangles: the 4 x 4 corners of the lattice, each once, and 9 x 2 triangles. Spec 4.3's two
  // triangles in all are for a grid that is one chunk; the seams cost the rest (spec 4.5).
  const Grid plane = makeGrid(20, 20, [](int i, int j) { return 2 * i + 3 * j; });
  const SimplifiedSurface s = simplify(plane, 20, 20, 0.5, 8);
  CHECK(s.surface.vertices.size() == 16);
  CHECK(s.surface.triangles.size() == 18);
  std::set<std::string> found;
  for (std::size_t v = 0; v < s.surface.vertices.size(); ++v) found.insert(whereIs(s, v, 20, 20));
  CHECK(found.size() == s.surface.vertices.size());  // no two vertices on one sample
  std::set<std::string> lattice;
  for (const int j : {0, 8, 16, 19}) {
    for (const int i : {0, 8, 16, 19}) lattice.insert(describe({i, j}));
  }
  CHECK(found == lattice);
  checkGuarantees(s, plane, 20, 20, 0.5);
  CHECK(chunkSideProblem(s, plane, 20, 20, 0.5, 8) == "");
}

TEST_CASE("chunks of 16 and 7 cells keep every guarantee across their seams, with at most twice one chunk's vertices",
          "[simplify]") {
  // The fractal grid's 63 x 47 cells are 4 x 3 chunks of 16 cells, or 9 x 7 of 7. Every seam closes and the tolerance
  // holds across it. A seam's profile keeps a sample every few, as the interior around it would have, so the chunks
  // cost vertices over one triangulation of the whole grid, but fewer than as many again.
  const Grid fractal = fractalGrid();
  const std::size_t whole = simplify(fractal, 64, 48, 2).surface.vertices.size();
  for (const int chunkCells : {16, 7}) {
    INFO("chunks of " << chunkCells << " cells");
    const SimplifiedSurface s = simplify(fractal, 64, 48, 2, chunkCells);
    checkGuarantees(s, fractal, 64, 48, 2);
    CHECK(chunkSideProblem(s, fractal, 64, 48, 2, chunkCells) == "");
    CHECK(s.surface.vertices.size() <= 2 * whole);
  }
}

TEST_CASE("two chunks that share a side have the same vertices on it, those its profile keeps from the west or north",
          "[simplify]") {
  // A 9 x 9 grid in chunks of 4 cells, 2 x 2 of them, with seams along column 4 and row 4. Along every side of every
  // chunk, shared or on the window's edge, the heights are 0 10 0 10 0, as in the case of one chunk above, and 0
  // inside. At 9.9 a side keeps its first two samples and its last, counted from the west or the north, and would keep
  // its first, fourth and last counted from the east or the south. The chunk west of a seam walks it from south to
  // north and the one east of it from north to south; the chunk north of a seam walks it from west to east and the one
  // south of it from east to west. Both must have the side as it was simplified once, the fixed way.
  const Grid lines =
      makeGrid(9, 9, [](int i, int j) { return (i % 4 == 0 || j % 4 == 0) && (i + j) % 2 == 1 ? 10 : 0; });
  const SimplifiedSurface s = simplify(lines, 9, 9, 9.9, 4);
  checkGuarantees(s, lines, 9, 9, 9.9);
  const ChunkSideVertices found = verticesAlongChunkSides(s, 9, 9, latticeOf(9, 9, 4));
  REQUIRE(found.problem == "");
  // Chunks (0, 0), (1, 0), (0, 1) and (1, 1), in that order. Column 4: the east sides of the western chunks and the
  // west sides of the eastern ones.
  CHECK(found.along[0][1] == "(4, 0) (4, 1) (4, 4)");
  CHECK(found.along[1][3] == "(4, 0) (4, 1) (4, 4)");
  CHECK(found.along[2][1] == "(4, 4) (4, 5) (4, 8)");
  CHECK(found.along[3][3] == "(4, 4) (4, 5) (4, 8)");
  // Row 4: the south sides of the northern chunks and the north sides of the southern ones.
  CHECK(found.along[0][0] == "(0, 4) (1, 4) (4, 4)");
  CHECK(found.along[2][2] == "(0, 4) (1, 4) (4, 4)");
  CHECK(found.along[1][0] == "(4, 4) (5, 4) (8, 4)");
  CHECK(found.along[3][2] == "(4, 4) (5, 4) (8, 4)");
  CHECK(chunkSideProblem(s, lines, 9, 9, 9.9, 4) == "");
}

TEST_CASE("a step along a seam and a spike on the corner of four chunks are kept, and the seams stay closed",
          "[simplify]") {
  // A flat 20 x 20 grid in chunks of 8 cells, at tolerance 2, with a step of 10 along column 8, the seam between the
  // first two chunks, from row 0 down to row 8, where it ends in a spike on the corner the first four chunks share:
  // samples (8, 0) to (8, 8) are 10 and the rest 0. A chunk's corners are always vertices, so the spike and the step's
  // ends, (8, 0) and (8, 8), are. Each side that runs off the step keeps the sample at its foot, as its profile has
  // it: the chord from 10 down to 0 over the side's 8 cells passes 8.75 over the first 0, more than over any other and
  // more than 2, and beyond that the side is flat. That is (8, 9) on the seam below the spike, (7, 8) and (9, 8) on
  // the seams either side of it, and (7, 0) and (9, 0) on the north edge. The step rises as a wall in both chunks
  // beside it, and every chunk at the corner rises to the spike; every triangle must still face up, and the solid
  // close (checkGuarantees).
  const Grid g = makeGrid(20, 20, [](int i, int j) { return i == 8 && j <= 8 ? 10 : 0; });
  const SimplifiedSurface s = simplify(g, 20, 20, 2, 8);
  for (const auto& [i, j] : std::vector<std::pair<int, int>>{{8, 8}, {8, 0}, {8, 9}, {7, 8}, {9, 8}, {7, 0}, {9, 0}}) {
    INFO("sample " << describe({i, j}));
    CHECK(isVertexAt(s, 20, 20, i, j));
  }
  checkGuarantees(s, g, 20, 20, 2);
  CHECK(chunkSideProblem(s, g, 20, 20, 2, 8) == "");
}

TEST_CASE("the vertices come chunk by chunk from the north-west, each chunk's outline before its interior",
          "[simplify]") {
  // On a plane in chunks of 8 cells each chunk is its four corners, and the first chunk's come first.
  const Grid plane = makeGrid(20, 20, [](int i, int j) { return 2 * i + 3 * j; });
  const SimplifiedSurface flat = simplify(plane, 20, 20, 0.5, 8);
  REQUIRE(flat.surface.vertices.size() >= 4);
  std::vector<std::string> firstFour;
  for (std::size_t v = 0; v < 4; ++v) firstFour.push_back(whereIs(flat, v, 20, 20));
  std::sort(firstFour.begin(), firstFour.end());
  CHECK(firstFour == std::vector<std::string>{"(0, 0)", "(0, 8)", "(8, 0)", "(8, 8)"});

  // On the fractal grid in chunks of 16 cells, 4 x 3 of them, a vertex comes with the first chunk that holds its
  // sample, sides included, the chunks taken row by row from the north and along each row from the west; and of a
  // chunk's vertices, those on its outline come first. Every chunk brings one at least, its south-east corner.
  const Grid fractal = fractalGrid();
  const SimplifiedSurface s = simplify(fractal, 64, 48, 2, 16);
  const std::vector<Chunk> chunks = latticeOf(64, 48, 16);
  std::vector<bool> brought(chunks.size(), false);
  std::string problem;
  std::size_t current = 0;    // the chunk whose vertices are coming
  bool inside = false;        // whether its interior's have begun
  std::size_t interiors = 0;  // vertices inside their chunks, so that the order is put to the test
  for (std::size_t v = 0; v < s.surface.vertices.size() && problem.empty(); ++v) {
    const std::optional<Sample> at = sampleOf(s.surface.vertices[v], 64, 48);
    REQUIRE(at);
    const auto first = static_cast<std::size_t>(
        std::find_if(chunks.begin(), chunks.end(), [&](const Chunk& c) { return holds(c, *at); }) - chunks.begin());
    REQUIRE(first < chunks.size());
    const Chunk& c = chunks[first];
    const bool onOutline = at->i == c.west || at->i == c.east || at->j == c.north || at->j == c.south;
    if (first < current) {
      problem = "vertex " + std::to_string(v) + " on " + describe(*at) + ", of " + chunkName(c) + ", comes after " +
                chunkName(chunks[current]) + "'s";
    } else if (first == current && inside && onOutline) {
      problem = "vertex " + std::to_string(v) + " on " + describe(*at) + ", on the outline of " + chunkName(c) +
                ", comes after its interior's";
    }
    if (first > current) {
      current = first;
      inside = false;
    }
    if (!onOutline) {
      inside = true;
      ++interiors;
    }
    brought[first] = true;
  }
  CHECK(problem == "");
  CHECK(std::count(brought.begin(), brought.end(), false) == 0);
  CHECK(interiors > 0);
}

TEST_CASE("a 300 x 300 grid keeps every guarantee as one chunk and in chunks of 64 cells", "[simplify]") {
  // The fractal formula over 300 x 300 samples, nearly five of its longest waves across: a sanity run on a grid of some
  // size, as one chunk and as 5 x 5 chunks, the last ones 43 cells.
  const Grid fractal = fractalGrid(300, 300);
  for (const int chunkCells : {kChunkCells, 64}) {
    INFO("chunks of " << chunkCells << " cells");
    const SimplifiedSurface s = simplify(fractal, 300, 300, 2, chunkCells);
    checkGuarantees(s, fractal, 300, 300, 2);
    CHECK(chunkSideProblem(s, fractal, 300, 300, 2, chunkCells) == "");
  }
}

TEST_CASE("a grid wider than the largest chunk is simplified chunk by chunk", "[simplify]") {
  // 16386 x 2 samples are 16385 cells across, one more than the largest chunk the predicates are exact for. Flat, each
  // chunk is its four corners and two triangles: in chunks of 16384 cells that is two chunks, the second one cell
  // wide; in the standard chunks of 2048, eight of 2048 cells and a ninth of one.
  const Grid flat(2 * 16386, 0.0f);
  const SimplifiedSurface widest = simplify(flat, 16386, 2, 0.5, kMostChunkCells);
  CHECK(widest.surface.vertices.size() == 6);
  CHECK(widest.surface.triangles.size() == 4);
  checkGuarantees(widest, flat, 16386, 2, 0.5);
  const SimplifiedSurface standard = simplify(flat, 16386, 2, 0.5);
  CHECK(standard.surface.vertices.size() == 20);
  CHECK(standard.surface.triangles.size() == 18);
  checkGuarantees(standard, flat, 16386, 2, 0.5);
}

TEST_CASE("simplifyGrid refuses a grid, a tolerance, a chunk size or a frame it cannot work with", "[simplify]") {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  constexpr double inf = std::numeric_limits<double>::infinity();
  const Grid g = makeGrid(9, 7, [](int i, int j) { return i + j; });

  REQUIRE_THROWS_MATCHES(simplifyGrid(Grid(7, 0.0f), 1, 7, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 1 x 7"));
  REQUIRE_THROWS_MATCHES(simplifyGrid(Grid(9, 0.0f), 9, 1, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 9 x 1"));
  // Every sample may be kept, and vertex numbers are 32 bits with room left for the solid's four bottom corners: a grid
  // of more samples is refused as surfaceMesh refuses it, before its heights are looked at. One row fewer fits, and
  // fails on its heights instead.
  REQUIRE_THROWS_MATCHES(simplifyGrid(Grid{}, 65536, 65536, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 65536 x 65536 vertices is too large: 32-bit vertex numbers leave room for "
                                 "4294967292 besides the four bottom corners"));
  REQUIRE_THROWS_MATCHES(simplifyGrid(Grid{}, 65536, 65535, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 65536 x 65535 vertices needs 4294901760 heights, not 0"));
  REQUIRE_THROWS_MATCHES(simplifyGrid(Grid(62, 0.0f), 9, 7, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 9 x 7 vertices needs 63 heights, not 62"));

  Grid holed = g;
  holed[cell(9, 4, 3)] = std::numeric_limits<float>::quiet_NaN();
  REQUIRE_THROWS_MATCHES(simplifyGrid(holed, 9, 7, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the height at column 4, row 3 is not finite"));
  // An infinite height cannot be held to any tolerance either.
  holed[cell(9, 4, 3)] = std::numeric_limits<float>::infinity();
  REQUIRE_THROWS_MATCHES(simplifyGrid(holed, 9, 7, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the height at column 4, row 3 is not finite"));

  REQUIRE_THROWS_MATCHES(simplifyGrid(g, 9, 7, -1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the tolerance must be finite and not negative, not -1"));
  CHECK_THROWS_AS(simplifyGrid(g, 9, 7, nan, 10, 10, 1, 1, 0), std::invalid_argument);
  CHECK_THROWS_AS(simplifyGrid(g, 9, 7, inf, 10, 10, 1, 1, 0), std::invalid_argument);

  REQUIRE_THROWS_MATCHES(simplifyGrid(g, 9, 7, 1, 10, 10, 1, 1, 0, 0), std::invalid_argument,
                         Message("a chunk must be 1 to 16384 cells a side, not 0"));
  REQUIRE_THROWS_MATCHES(simplifyGrid(g, 9, 7, 1, 10, 10, 1, 1, 0, kMostChunkCells + 1), std::invalid_argument,
                         Message("a chunk must be 1 to 16384 cells a side, not 16385"));
  CHECK_NOTHROW(simplifyGrid(g, 9, 7, 1, 10, 10, 1, 1, 0, 1));
  CHECK_NOTHROW(simplifyGrid(g, 9, 7, 1, 10, 10, 1, 1, 0, 16384));

  // surfaceMesh's checks of the frame, with its messages.
  REQUIRE_THROWS_MATCHES(simplifyGrid(g, 9, 7, 1, 10, 10, 1, 0, 0), std::invalid_argument,
                         Message("the z-scale and the scale must be above 0 and finite, not 1 and 0"));
  REQUIRE_THROWS_MATCHES(simplifyGrid(g, 9, 7, 1, 0, 18, 1, 1, 0), std::invalid_argument,
                         Message("the window must be wider and taller than 0 m and finite, not 0 x 18 m"));
  CHECK_THROWS_AS(simplifyGrid(g, 9, 7, 1, 10, inf, 1, 1, 0), std::invalid_argument);
  CHECK_THROWS_AS(simplifyGrid(g, 9, 7, 1, 10, 10, -2, 1, 0), std::invalid_argument);
  REQUIRE_THROWS_MATCHES(simplifyGrid(g, 9, 7, 1, 10, 10, 1, 1, nan), std::invalid_argument,
                         Message("the level must be finite, not nan"));
}

TEST_CASE("window blocks give the surface simplifyGrid gives with chunks of their size", "[simplify]") {
  // simplifyGrid is a BlockSimplifier over its lattice with no footprint (spec 4.5), so the same lattice of window
  // blocks, given in its order, must give its surface vertex for vertex and triangle for triangle. Each block's
  // triangles follow the last one's, and its rectangle is the Boundary of its own four sides.
  const Grid fractal = fractalGrid();
  const std::vector<BlockRect> rects = blockLattice(64, 48, 16, 16);
  REQUIRE(rects.size() == 12);  // 63 x 47 cells: 4 x 3 blocks, the last ones 15 cells across and down
  BlockSimplifier simplifier = blockSimplifier(fractal, 64, 48, 2);
  std::vector<BlockResult> results;
  for (const BlockRect& rect : rects) results.push_back(simplifier.simplifyBlock(rect, nullptr));
  const Mesh& blocks = simplifier.surface();
  const SimplifiedSurface chunks = simplify(fractal, 64, 48, 2, 16);
  REQUIRE(blocks.vertices.size() == chunks.surface.vertices.size());
  REQUIRE(blocks.triangles.size() == chunks.surface.triangles.size());
  for (std::size_t v = 0; v < blocks.vertices.size(); ++v) {
    const Vec3& a = blocks.vertices[v];
    const Vec3& b = chunks.surface.vertices[v];
    CHECK((a.x == b.x && a.y == b.y && a.z == b.z));
  }
  for (std::size_t t = 0; t < blocks.triangles.size(); ++t) {
    const Triangle& a = blocks.triangles[t];
    const Triangle& b = chunks.surface.triangles[t];
    CHECK((a.a == b.a && a.b == b.b && a.c == b.c));
  }

  std::size_t next = 0;
  for (std::size_t k = 0; k < rects.size(); ++k) {
    INFO(describeBlock(rects[k]));
    const BlockResult& result = results[k];
    CHECK(result.firstTriangle == next);
    next = result.firstTriangle + result.triangles;
    CHECK(result.pieces.empty());
    REQUIRE(result.rectangle.has_value());
    CHECK(rectangleProblem(blocks, *result.rectangle, rects[k], 64, 48) == "");
  }
  CHECK(next == blocks.triangles.size());
}

TEST_CASE("a piece that is its whole block, with one run a side, is simplified as the window block is", "[simplify]") {
  // A footprint of every cell: each block's one piece is the block itself, and with the cells across every side in,
  // its runs are its four sides whole, the very sides a window block simplifies. Such a piece is simplified as the
  // window block is, so that --footprint data over a window without no-data gives --footprint window's surface (Review
  // Focus 4), and its outline is the rectangle's, from the block's north-west corner down its west side, along its
  // south side, up its east side and back along its north side.
  const Grid fractal = fractalGrid();
  const Flags everyCell(63 * 47, 1);
  const std::vector<BlockRect> rects = blockLattice(64, 48, 16, 16);
  const SimplifiedFootprint pieces = simplifyFootprint(fractal, everyCell, 64, 48, 2, rects);
  checkGuarantees(pieces, fractal, everyCell, 64, 48, 2);
  BlockSimplifier windows = blockSimplifier(fractal, 64, 48, 2);
  std::vector<BlockResult> results;
  for (const BlockRect& rect : rects) results.push_back(windows.simplifyBlock(rect, nullptr));
  const Mesh& window = windows.surface();
  REQUIRE(pieces.surface.vertices.size() == window.vertices.size());
  REQUIRE(pieces.surface.triangles.size() == window.triangles.size());
  for (std::size_t v = 0; v < window.vertices.size(); ++v) {
    const Vec3& a = pieces.surface.vertices[v];
    const Vec3& b = window.vertices[v];
    CHECK((a.x == b.x && a.y == b.y && a.z == b.z));
  }
  for (std::size_t t = 0; t < window.triangles.size(); ++t) {
    const Triangle& a = pieces.surface.triangles[t];
    const Triangle& b = window.triangles[t];
    CHECK((a.a == b.a && a.b == b.b && a.c == b.c));
  }
  for (std::size_t k = 0; k < rects.size(); ++k) {
    INFO(describeBlock(rects[k]));
    REQUIRE(pieces.blocks[k].result.pieces.size() == 1);
    const auto& [south, east, north, west] = results[k].rectangle->sides;
    std::vector<std::uint32_t> wanted;
    for (const std::vector<std::uint32_t>* side : {&west, &south, &east, &north}) {
      wanted.insert(wanted.end(), side->begin(), side->end() - 1);
    }
    CHECK(pieces.blocks[k].result.pieces[0].vertices == wanted);
  }
}

TEST_CASE("a footprint block keeps every guarantee inside its pieces, leaves the sea out and covers the lake",
          "[simplify]") {
  // The island in one block over the whole grid, at 0.5 m: four pieces, the island with its lake and channel filled,
  // the two blobs and the islet (the lone sample makes no cell). checkGuarantees holds the outlines to their runs, the
  // surface to the tolerance at every sample of the footprint, and finds nothing over the sea.
  const Island island;
  const SimplifiedFootprint s = island.simplified(0.5, kIslandCols - 1, kIslandRows - 1);
  REQUIRE(s.blocks.size() == 1);
  const BlockResult& result = s.blocks[0].result;
  REQUIRE(result.pieces.size() == 4);
  checkGuarantees(s, island.grid, island.cells, kIslandCols, kIslandRows, 0.5);
  // Every edge between two triangles is Delaunay: the seed's flips and the insertions' keep it so, the outline's edges
  // aside.
  CHECK(delaunayProblem(s.surface, kIslandCols, kIslandRows) == "");

  std::vector<Sample> sample;
  std::vector<std::int64_t> vertexOn;
  REQUIRE(placeVertices(s.surface, island.grid, kIslandCols, kIslandRows, sample, vertexOn) == "");
  const std::vector<std::int64_t> holder = trianglesHolding(s.surface, sample, kIslandCols, kIslandRows);
  const auto surfaceAt = [&](int i, int j) {
    return surfaceHeightAt(s.surface, holder, kIslandCols, kIslandRows, i, j);
  };
  // The lake and the channel: no-data in the file, filled with their nearest heights, inside the island's piece.
  for (const auto& [i, j] : std::vector<std::pair<int, int>>{{7, 5}, {8, 5}, {7, 6}, {8, 6}, {5, 5}, {6, 5}}) {
    INFO("the lake's sample " << describe({i, j}));
    CHECK(std::fabs(surfaceAt(i, j) - island.grid[cell(kIslandCols, i, j)]) <= 0.5 + 1e-6);
  }
  // The sea: at the grid's corner, between the island and the islet, between the tab and the blobs, and the lone
  // sample.
  for (const auto& [i, j] : std::vector<std::pair<int, int>>{{0, 0}, {5, 12}, {16, 4}, {18, 13}}) {
    INFO("the sea's sample " << describe({i, j}));
    CHECK(std::isnan(surfaceAt(i, j)));
  }
  // The knoll's summit is 15 m over the plane that the island's outline lies on, all but the channel's mouth, so it is
  // the first sample inserted inside the island, the vertex after the island's outline, and the surface is at its
  // height.
  const std::int64_t summit = vertexOn[cell(kIslandCols, kIslandKnollColumn, kIslandKnollRow)];
  CHECK(summit == static_cast<std::int64_t>(result.pieces[0].vertices.size()));
  CHECK(surfaceAt(kIslandKnollColumn, kIslandKnollRow) ==
        island.grid[cell(kIslandCols, kIslandKnollColumn, kIslandKnollRow)]);
}

TEST_CASE("blocks that share a side have the same vertices on it, those its runs keep, cut short or not",
          "[simplify]") {
  // Spec 5.3: a run that both blocks' pieces have along their common side is simplified once, in the fixed direction,
  // and both take its samples, to which addShared gives one vertex number each; the two outlines walk the side the two
  // ways, so the two runs are the same numbers in the other order. A run along the side that one block has alone is
  // that block's coast, and overlaps none of the other's. And a run between the same two ends keeps the same samples
  // however the grid is cut into blocks, as the island in one block shows.
  //
  // In blocks of 12 x 9 cells, column 12 cuts the tab off the island, and the run the two have there, from (12, 4) to
  // (12, 8), crosses the knoll's west slope and summit: 50, 62.75, 68, 65.75 and 56 m, none within 0.5 m of a chord, so
  // all five samples are kept. Row 9 cuts the island's south strip off, a run of the plane's that keeps its two ends.
  // In blocks of 13 x 10, cut short at the east and the south, column 13 cuts the tab in two, and the run there crosses
  // the knoll's east slope, 52, 61, 66.25, 64 and 58 m, all five kept; row 10 is the island's south coast, the
  // north-western block's alone. The south-eastern block has only the lone sample, which makes no cell, so no piece.
  //
  // The runs that the blocks and the one block over the whole grid both have: the blobs' and the islet's, 12; and in
  // 12 x 9 blocks the island's north coast and the end of its east coast above the tab, the tab's other three sides and
  // the island's south coast, 6 more; in 13 x 10, the island's west, south and north coasts, the end of its east coast
  // below the tab and the one above it, and the tab's east side, 6 more.
  const Island island;
  const SimplifiedFootprint whole = island.simplified(0.5, kIslandCols - 1, kIslandRows - 1);
  struct Case {
    int blockCols, blockRows, matchedRuns, matchedVertices;
  };
  for (const Case& c : {Case{12, 9, 2, 7}, Case{13, 10, 1, 5}}) {
    INFO("blocks of " << c.blockCols << " x " << c.blockRows << " cells");
    const SimplifiedFootprint s = island.simplified(0.5, c.blockCols, c.blockRows);
    REQUIRE(s.blocks.size() == 4);
    checkGuarantees(s, island.grid, island.cells, kIslandCols, kIslandRows, 0.5);
    CHECK(s.blocks[3].result.pieces.empty());
    CHECK(s.blocks[3].result.triangles == 0);
    int matched = 0, matchedVertices = 0;
    CHECK(sharedSideProblem(s.blocks[0], s.blocks[1], false, c.blockCols, matched, matchedVertices) == "");
    CHECK(sharedSideProblem(s.blocks[2], s.blocks[3], false, c.blockCols, matched, matchedVertices) == "");
    CHECK(sharedSideProblem(s.blocks[0], s.blocks[2], true, c.blockRows, matched, matchedVertices) == "");
    CHECK(sharedSideProblem(s.blocks[1], s.blocks[3], true, c.blockRows, matched, matchedVertices) == "");
    CHECK(matched == c.matchedRuns);
    CHECK(matchedVertices == c.matchedVertices);
    int common = 0;
    CHECK(commonRunProblem(s, whole, common) == "");
    CHECK(common == 18);
  }
}

TEST_CASE("every run is simplified from west to east or from north to south, whichever way the outline walks it",
          "[simplify]") {
  // As for a chunk's sides: simplifyProfile settles a tie by the order its heights come in, so a run that two blocks'
  // pieces share keeps the same samples in both only if each simplifies it the same way. A piece of 4 x 4 cells,
  // columns and rows 2 to 6 of samples, in a sea two samples wide: its loop walks its west side from north to south
  // and its south side from west to east, the fixed ways, but its east side from south to north and its north side
  // from east to west. Every side's heights are 0 10 0 10 0, 0 inside, and at 9.9 a side keeps its first two samples
  // and its last counted from the west or the north (the case of the chunk above), and would keep its first, its
  // fourth and its last counted from the east or the south.
  const int cols = 9, rows = 9;
  Flags mask(cols * rows, 0);
  for (int j = 2; j <= 6; ++j) {
    for (int i = 2; i <= 6; ++i) mask[cell(cols, i, j)] = 1;
  }
  const Flags cells = footprintCells(mask, cols, rows);
  const Grid ring = makeGrid(cols, rows, [](int i, int j) {
    const bool onRing = (i == 2 || i == 6 || j == 2 || j == 6) && i >= 2 && i <= 6 && j >= 2 && j <= 6;
    return onRing && (i + j) % 2 == 1 ? 10 : 0;
  });
  const SimplifiedFootprint s = simplifyFootprint(ring, cells, cols, rows, 9.9, {{0, 0, cols - 1, rows - 1}});
  REQUIRE(s.blocks.size() == 1);
  REQUIRE(s.blocks[0].result.pieces.size() == 1);
  std::vector<GridPoint> found;
  for (const std::uint32_t v : s.blocks[0].result.pieces[0].vertices) {
    const std::optional<Sample> at = sampleOf(s.surface.vertices.at(v), cols, rows);
    REQUIRE(at);
    found.push_back({at->i, at->j});
  }
  // Down the west side rows 2 and 3, along the south side columns 2 and 3, up the east side rows 6 and 3, and back
  // along the north side columns 6 and 3: each run's last sample is the next run's first.
  CHECK(where(found) == "(2, 2) (2, 3) (2, 6) (3, 6) (6, 6) (6, 3) (6, 2) (3, 2)");
  checkGuarantees(s, ring, cells, cols, rows, 9.9);
}

TEST_CASE("a sample that a piece's run drops is never inserted, even where rounding puts it a hair over the tolerance",
          "[simplify]") {
  // Spec 5.2: the outline keeps its runs' samples and no other. A sample a run drops is within the tolerance of the
  // outline edge over it, and so of the plane of the face that has that edge, but that plane is worked out in double
  // with a division, and can put the sample a hair beyond the tolerance. It lies on the outline, which no scan looks
  // at, so it is never inserted. Both pieces here were found by trying every rectangular piece of up to nine cells with
  // heights of 0 and 1 against a simplifier whose scans looked at the outline: it inserts the dropped sample in both.
  // In both the run is 1 0 0 0 from the north, at a tolerance of 2/3: its chord passes 2/3 over the sample after the 1,
  // exactly the tolerance, which simplifyProfile allows, so the run keeps its two ends only.
  struct Case {
    const char* name;
    int width, height;              // the piece in cells, its north-west corner at the sample (2, 2)
    std::vector<std::string> ones;  // the piece's heights by rows from its north side, the rest of the grid 0
    GridPoint dropped;
  };
  const std::vector<Case> cases = {
      // The west run's (2, 3), the first sample of its row in the face over the west side.
      {"a strip of 1 x 3 cells", 1, 3, {"10", "00", "00", "00"}, {2, 3}},
      // The east run's (4, 3), the last of two samples of its row in the face over the east side.
      {"a piece of 2 x 3 cells", 2, 3, {"001", "110", "000", "000"}, {4, 3}},
  };
  for (const Case& c : cases) {
    INFO(c.name);
    const int cols = c.width + 5, rows = c.height + 5;
    Flags mask(static_cast<std::size_t>(cols * rows), 0);
    Grid g(static_cast<std::size_t>(cols * rows), 0.0f);
    for (int j = 0; j <= c.height; ++j) {
      for (int i = 0; i <= c.width; ++i) {
        mask[cell(cols, 2 + i, 2 + j)] = 1;
        g[cell(cols, 2 + i, 2 + j)] = c.ones[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] == '1' ? 1 : 0;
      }
    }
    const Flags cells = footprintCells(mask, cols, rows);
    const SimplifiedFootprint s = simplifyFootprint(g, cells, cols, rows, 2.0 / 3.0, {{0, 0, cols - 1, rows - 1}});
    REQUIRE(s.blocks[0].result.pieces.size() == 1);
    CHECK_FALSE(isVertexAt(s.surface, cols, rows, c.dropped.x, c.dropped.y));
    checkGuarantees(s, g, cells, cols, rows, 2.0 / 3.0);
  }

  // And at tolerance 0 on the island, where every run along the plane keeps its ends only.
  const Island island;
  const std::vector<std::pair<int, int>> lattices = {{kIslandCols - 1, kIslandRows - 1}, {12, 9}};
  for (const auto& [blockCols, blockRows] : lattices) {
    INFO("the island in blocks of " << blockCols << " x " << blockRows << " cells");
    checkGuarantees(island.simplified(0, blockCols, blockRows), island.grid, island.cells, kIslandCols, kIslandRows, 0);
  }
}

TEST_CASE("the flips over a piece's seed leave every edge between two of its faces Delaunay", "[simplify]") {
  // A flat piece whose outline is a diamond of steps, every turn of it a run's end and so a vertex: nothing inside is
  // off the flat faces, so nothing is inserted, and the triangulation is the seed of triangulatePolygon after Lawson's
  // flips over it alone. Each flip must test again all four edges round its quadrilateral, the two that met the new
  // edge's ends as well as the two across from them, for none to be left with a vertex inside a neighbour's circle.
  const int cols = 27, rows = 27;
  Flags mask(cols * rows, 0);
  for (int j = 0; j < rows; ++j) {
    for (int i = 0; i < cols; ++i) mask[cell(cols, i, j)] = std::abs(i - 13) + std::abs(j - 13) <= 10 ? 1 : 0;
  }
  const Flags cells = footprintCells(mask, cols, rows);
  const Grid flat(static_cast<std::size_t>(cols * rows), 0.0f);
  const SimplifiedFootprint s = simplifyFootprint(flat, cells, cols, rows, 0, {{0, 0, cols - 1, rows - 1}});
  REQUIRE(s.blocks[0].result.pieces.size() == 1);
  const std::vector<GridPoint>& loop = s.blocks[0].footprint.pieces[0].loop;
  std::size_t turns = 0;
  for (std::size_t k = 0; k < loop.size(); ++k) {
    const GridPoint a = loop[(k + loop.size() - 1) % loop.size()], b = loop[k], c = loop[(k + 1) % loop.size()];
    if (b.x - a.x != c.x - b.x || b.y - a.y != c.y - b.y) ++turns;
  }
  const std::size_t outline = s.blocks[0].result.pieces[0].vertices.size();
  CHECK(outline == turns);  // the flat runs keep their ends only
  CHECK(outline > 60);
  CHECK(s.surface.vertices.size() == outline);  // and nothing inside comes in
  CHECK(s.surface.triangles.size() == outline - 2);
  checkGuarantees(s, flat, cells, cols, rows, 0);
  CHECK(delaunayProblem(s.surface, cols, rows) == "");
}

TEST_CASE("pieces that touch at a vertex share that one vertex, in one block or either side of a block's side",
          "[simplify]") {
  // Spec 5.2: the two blobs touch at (19, 4), where both outlines turn, so that both runs there end at it and keep it,
  // and addShared gives it one vertex number, which both outlines hold. They share nothing else.
  const Island island;
  const auto shared = [](const PieceOutline& a, const PieceOutline& b) {
    std::vector<std::uint32_t> both;
    for (const std::uint32_t v : a.vertices) {
      if (std::find(b.vertices.begin(), b.vertices.end(), v) != b.vertices.end()) both.push_back(v);
    }
    return both;
  };
  // The piece of a block whose loop starts at `first`.
  const auto pieceFrom = [](const SimplifiedBlock& block, GridPoint first) -> const PieceOutline& {
    for (std::size_t k = 0; k < block.footprint.pieces.size(); ++k) {
      if (same(block.footprint.pieces[k].loop.at(0), first)) return block.result.pieces.at(k);
    }
    FAIL("no piece's loop starts at " << where(first));
    throw std::logic_error("unreachable");
  };

  SECTION("in one block") {
    const SimplifiedFootprint s = island.simplified(0.5, kIslandCols - 1, kIslandRows - 1);
    checkGuarantees(s, island.grid, island.cells, kIslandCols, kIslandRows, 0.5);
    REQUIRE(s.blocks.size() == 1);
    const std::vector<std::uint32_t> both = shared(pieceFrom(s.blocks[0], {17, 2}), pieceFrom(s.blocks[0], {19, 4}));
    REQUIRE(both.size() == 1);
    CHECK(whereIs(s.surface, both[0], kIslandCols, kIslandRows) == "(19, 4)");
  }

  SECTION("in the blocks either side of column 19, blob A in the western one and blob B in the eastern one") {
    const SimplifiedFootprint s = island.simplified(0.5, 19, kIslandRows - 1);
    checkGuarantees(s, island.grid, island.cells, kIslandCols, kIslandRows, 0.5);
    REQUIRE(s.blocks.size() == 2);
    const std::vector<std::uint32_t> both = shared(pieceFrom(s.blocks[0], {17, 2}), pieceFrom(s.blocks[1], {19, 4}));
    REQUIRE(both.size() == 1);
    CHECK(whereIs(s.surface, both[0], kIslandCols, kIslandRows) == "(19, 4)");
  }
}

TEST_CASE("the vertices come block by block and piece by piece, each piece's outline before its inside", "[simplify]") {
  // A vertex comes with the first block, and in it the first piece, that has its sample on its outline or inside it,
  // and of each piece's vertices those of its outline that no block or piece before it brought come first, then those
  // inside it. In blocks of 12 x 9 cells the island's pieces either side of column 12 and of row 9 share their runs'
  // vertices there, and the knoll and the lake give vertices inside.
  const Island island;
  const SimplifiedFootprint s = island.simplified(0.5, 12, 9);
  checkGuarantees(s, island.grid, island.cells, kIslandCols, kIslandRows, 0.5);
  std::vector<bool> placed(s.surface.vertices.size(), false);
  std::uint32_t next = 0;  // the number the next new vertex must have
  int sharedOutlineVertices = 0, insideVertices = 0;
  std::string problem;
  for (const SimplifiedBlock& block : s.blocks) {
    for (std::size_t k = 0; k < block.result.pieces.size() && problem.empty(); ++k) {
      const Piece& piece = block.footprint.pieces[k];
      const PieceOutline& outline = block.result.pieces[k];
      const std::string name = describeBlock(block.rect) + "'s piece from " + where(piece.loop.at(0));
      // Its outline's new vertices take the next numbers, in some order.
      std::set<std::uint32_t> fresh;
      for (const std::uint32_t v : outline.vertices) {
        if (v < placed.size() && placed[v]) {
          ++sharedOutlineVertices;
        } else {
          fresh.insert(v);
        }
      }
      if (!fresh.empty() && (*fresh.begin() != next || *fresh.rbegin() != next + fresh.size() - 1)) {
        problem = name + "'s new outline vertices are not the " + std::to_string(fresh.size()) + " from " +
                  std::to_string(next);
        break;
      }
      for (const std::uint32_t v : fresh) placed[v] = true;
      next += static_cast<std::uint32_t>(fresh.size());
      // Then the vertices inside it: strictly inside its loop's bounding box, and on no sample of the loop. The pieces
      // of a block here have bounding boxes apart, so the next piece's outline is not taken for this one's inside.
      int west = piece.loop[0].x, east = west, north = piece.loop[0].y, south = north;
      std::set<std::pair<int, int>> onLoop;
      for (const GridPoint& p : piece.loop) {
        west = std::min(west, p.x);
        east = std::max(east, p.x);
        north = std::min(north, p.y);
        south = std::max(south, p.y);
        onLoop.insert({p.x, p.y});
      }
      while (next < s.surface.vertices.size()) {
        const std::optional<Sample> at = sampleOf(s.surface.vertices[next], kIslandCols, kIslandRows);
        REQUIRE(at);
        const bool inside =
            at->i > west && at->i < east && at->j > north && at->j < south && !onLoop.contains({at->i, at->j});
        if (!inside) break;
        placed[next++] = true;
        ++insideVertices;
      }
    }
  }
  CHECK(problem == "");
  CHECK(next == s.surface.vertices.size());
  CHECK(sharedOutlineVertices > 0);
  CHECK(insideVertices > 0);
}

TEST_CASE("the same grid, tolerance and blocks give the same surface and the same outlines", "[simplify]") {
  const Island island;
  const SimplifiedFootprint first = island.simplified(0.5, 12, 9);
  const SimplifiedFootprint second = island.simplified(0.5, 12, 9);
  REQUIRE(first.surface.vertices.size() == second.surface.vertices.size());
  REQUIRE(first.surface.triangles.size() == second.surface.triangles.size());
  for (std::size_t v = 0; v < first.surface.vertices.size(); ++v) {
    const Vec3& a = first.surface.vertices[v];
    const Vec3& b = second.surface.vertices[v];
    CHECK((a.x == b.x && a.y == b.y && a.z == b.z));
  }
  for (std::size_t t = 0; t < first.surface.triangles.size(); ++t) {
    const Triangle& a = first.surface.triangles[t];
    const Triangle& b = second.surface.triangles[t];
    CHECK((a.a == b.a && a.b == b.b && a.c == b.c));
  }
  REQUIRE(first.blocks.size() == second.blocks.size());
  for (std::size_t k = 0; k < first.blocks.size(); ++k) {
    const BlockResult& a = first.blocks[k].result;
    const BlockResult& b = second.blocks[k].result;
    CHECK(a.firstTriangle == b.firstTriangle);
    CHECK(a.triangles == b.triangles);
    REQUIRE(a.pieces.size() == b.pieces.size());
    for (std::size_t p = 0; p < a.pieces.size(); ++p) {
      CHECK(a.pieces[p].vertices == b.pieces[p].vertices);
      CHECK(where(a.pieces[p].points) == where(b.pieces[p].points));
    }
  }
}

TEST_CASE("random footprints in random lattices keep every guarantee and meet along every shared side",
          "[simplify]") {
  // Footprints of random masks (coasts, lakes, islets, slits and pieces touching at a vertex), random heights, smooth,
  // noise or small whole numbers, a random tolerance and a random lattice of blocks, the last ones cut short: the whole
  // grid as one block and the lattice are each held to checkGuarantees, every side two blocks of the lattice share to
  // sharedSideProblem, and every run the two simplifications have in common to commonRunProblem. The whole numbers come
  // with tolerances of 1/3 and 2/3, at which their chords pass many samples exactly at the tolerance: the ties that a
  // run's direction settles, and the dropped samples that rounding puts a hair beyond the tolerance.
  std::mt19937 rng(20261022);
  int pieces = 0, blocksWithPieces = 0, matched = 0, matchedVertices = 0, common = 0, contacts = 0, wholeBlocks = 0;
  for (int trial = 0; trial < 150; ++trial) {
    const int cols = 3 + static_cast<int>(rng() % 26);
    const int rows = 3 + static_cast<int>(rng() % 26);
    const Flags cells = footprintCells(randomMask(rng, cols, rows), cols, rows);
    const int style = static_cast<int>(rng() % 3);
    const double phase = static_cast<double>(rng() % 628) / 100.0;
    Grid g;
    if (style == 0) {
      g = makeGrid(cols, rows, [phase](int i, int j) {
        return 6 * std::sin(i / 3.0 + phase) * std::cos(j / 4.0) + 0.3 * i - 0.2 * j;
      });
    } else if (style == 1) {
      g = randomGrid(rng, cols, rows, 0, 10);
    } else {
      g = makeGrid(cols, rows, [&rng](int, int) { return static_cast<double>(rng() % 3); });
    }
    const double tol =
        style == 2 ? std::array{1.0 / 3.0, 2.0 / 3.0}[rng() % 2] : std::array{0.0, 0.25, 1.0, 4.0}[rng() % 4];
    const int blockCols = 1 + static_cast<int>(rng() % static_cast<unsigned>(cols - 1));
    const int blockRows = 1 + static_cast<int>(rng() % static_cast<unsigned>(rows - 1));
    INFO("trial " << trial << ": " << cols << " x " << rows << ", heights of style " << style << ", tolerance " << tol
                  << ", blocks of " << blockCols << " x " << blockRows);

    const SimplifiedFootprint whole = simplifyFootprint(g, cells, cols, rows, tol, {{0, 0, cols - 1, rows - 1}});
    checkGuarantees(whole, g, cells, cols, rows, tol);
    CHECK(delaunayProblem(whole.surface, cols, rows) == "");
    contacts += static_cast<int>(whole.blocks[0].footprint.cornerContacts);

    const std::vector<BlockRect> rects = blockLattice(cols, rows, blockCols, blockRows);
    const SimplifiedFootprint s = simplifyFootprint(g, cells, cols, rows, tol, rects);
    checkGuarantees(s, g, cells, cols, rows, tol);
    const std::size_t across = static_cast<std::size_t>((cols - 2) / blockCols + 1);
    for (std::size_t k = 0; k < s.blocks.size(); ++k) {
      const BlockRect& rect = s.blocks[k].rect;
      if (k % across + 1 < across) {
        CHECK(sharedSideProblem(s.blocks[k], s.blocks[k + 1], false, rect.lastColumn, matched, matchedVertices) ==
              "");
      }
      if (k + across < s.blocks.size()) {
        CHECK(sharedSideProblem(s.blocks[k], s.blocks[k + across], true, rect.lastRow, matched, matchedVertices) ==
              "");
      }
      pieces += static_cast<int>(s.blocks[k].result.pieces.size());
      if (!s.blocks[k].result.pieces.empty()) ++blocksWithPieces;
      const std::uint64_t blockCells = static_cast<std::uint64_t>(rect.lastColumn - rect.firstColumn) *
                                       static_cast<std::uint64_t>(rect.lastRow - rect.firstRow);
      if (s.blocks[k].footprint.cells == blockCells) ++wholeBlocks;
    }
    CHECK(commonRunProblem(s, whole, common) == "");
  }
  // The draw is not one-sided: pieces in many blocks, runs shared between blocks and keeping samples between their
  // ends, runs common to both simplifications, contacts, and blocks that are all footprint.
  CHECK(pieces > 700);
  CHECK(blocksWithPieces > 550);
  CHECK(matched > 450);
  CHECK(matchedVertices > 2 * matched + 300);
  CHECK(common > 1200);
  CHECK(contacts > 25);
  CHECK(wholeBlocks > 120);
}

TEST_CASE("simplifyBlock refuses a block it cannot simplify, and leaves the surface as it was", "[simplify]") {
  const Island island;
  BlockSimplifier simplifier = blockSimplifier(island.grid, kIslandCols, kIslandRows, 0.5);
  const BlockFootprint whole = blockFootprint(island.cells, kIslandCols, kIslandRows, {0, 0, 23, 17}, 1.0, 0.0);

  SECTION("a block wider or taller than the largest chunk") {
    const Grid flat(2 * 16386, 0.0f);
    BlockSimplifier wide = blockSimplifier(flat, 16386, 2, 0.5);
    CHECK_THROWS_MATCHES(wide.simplifyBlock({0, 0, 16385, 1}, nullptr), std::invalid_argument,
                         Message("the block of columns 0 to 16385 and rows 0 to 1 is 16385 cells wide; a block is at "
                                 "most 16384 cells a side"));
    CHECK(wide.surface().vertices.empty());
    BlockSimplifier tall = blockSimplifier(flat, 2, 16386, 0.5);
    CHECK_THROWS_MATCHES(tall.simplifyBlock({0, 0, 1, 16385}, nullptr), std::invalid_argument,
                         Message("the block of columns 0 to 1 and rows 0 to 16385 is 16385 cells tall; a block is at "
                                 "most 16384 cells a side"));
    // One cell fewer is a block.
    CHECK(wide.simplifyBlock({0, 0, 16384, 1}, nullptr).triangles == 2);
  }

  SECTION("a block outside the grid, or with no cell") {
    CHECK_THROWS_MATCHES(simplifier.simplifyBlock({0, 0, 24, 17}, nullptr), std::invalid_argument,
                         Message("the block of columns 0 to 24 and rows 0 to 17 is not inside the grid of 24 x 18 "
                                 "vertices"));
    CHECK_THROWS_MATCHES(simplifier.simplifyBlock({-1, 0, 12, 17}, &whole), std::invalid_argument,
                         Message("the block of columns -1 to 12 and rows 0 to 17 is not inside the grid of 24 x 18 "
                                 "vertices"));
    CHECK_THROWS_MATCHES(simplifier.simplifyBlock({0, 0, 12, 18}, nullptr), std::invalid_argument,
                         Message("the block of columns 0 to 12 and rows 0 to 18 is not inside the grid of 24 x 18 "
                                 "vertices"));
    CHECK_THROWS_MATCHES(simplifier.simplifyBlock({5, 0, 5, 17}, nullptr), std::invalid_argument,
                         Message("the block of columns 5 to 5 and rows 0 to 17 has no cell"));
    CHECK_THROWS_MATCHES(simplifier.simplifyBlock({12, 0, 4, 17}, nullptr), std::invalid_argument,
                         Message("the block of columns 12 to 4 and rows 0 to 17 has no cell"));
  }

  SECTION("a footprint whose pieces are not the block's") {
    // The whole grid's pieces given for the north-west block of 12 x 9 cells: the island's loop goes down its west
    // coast to row 10, past the block's south side.
    CHECK_THROWS_MATCHES(simplifier.simplifyBlock({0, 0, 12, 9}, &whole), std::invalid_argument,
                         Message("the footprint's piece whose loop starts at (4, 2) has the sample (4, 10), outside "
                                 "the block of columns 0 to 12 and rows 0 to 9"));
  }

  SECTION("a piece that is not a loop of runs") {
    const BlockRect rect{0, 0, 23, 17};
    // The islet's loop, (4, 13) to (5, 13), with its four runs, broken one way at a time.
    const Piece islet = whole.pieces.at(3);
    REQUIRE(same(islet.loop.at(0), {4, 13}));
    const auto refusal = [&](const Piece& piece) {
      BlockFootprint f;
      f.pieces = {piece};
      f.cells = piece.cells;
      try {
        (void)simplifier.simplifyBlock(rect, &f);
      } catch (const std::invalid_argument& e) {
        return std::string(e.what());
      }
      return std::string("no refusal");
    };
    Piece jump = islet;
    jump.loop.erase(jump.loop.begin() + 1);  // (4, 13) to (4, 15)
    jump.runStarts = {0, 1, 3, 5};
    CHECK(refusal(jump) == "the footprint's piece whose loop starts at (4, 13) steps from (4, 13) to (4, 15), not one "
                           "cell along a grid line");
    Piece bent = islet;
    bent.runStarts = {0, 4, 6};  // the run from (4, 13) turns at (4, 15)
    CHECK(refusal(bent) == "the footprint's piece whose loop starts at (4, 13) has a run from (4, 13) that turns at "
                           "(4, 15)");
    Piece unordered = islet;
    unordered.runStarts = {0, 4, 2, 6};
    CHECK(refusal(unordered) == "the footprint's piece whose loop starts at (4, 13) has its runs start at 0, 4, 2, 6: "
                                "they must start at 0 and rise, within its loop of 8 samples");
    Piece beyond = islet;
    beyond.runStarts = {0, 2, 4, 6, 8};
    CHECK(refusal(beyond) == "the footprint's piece whose loop starts at (4, 13) has its runs start at 0, 2, 4, 6, 8: "
                             "they must start at 0 and rise, within its loop of 8 samples");
    Piece clockwise = islet;
    std::reverse(clockwise.loop.begin() + 1, clockwise.loop.end());  // (4, 13) (5, 13) (6, 13) ... east first
    CHECK(refusal(clockwise) == "the footprint's piece whose loop starts at (4, 13) runs clockwise seen from above");
    Piece tiny = islet;
    tiny.loop = {{4, 13}, {4, 14}};
    tiny.runStarts = {0, 1};
    CHECK(refusal(tiny) == "the footprint's piece whose loop starts at (4, 13) has 2 samples, too few for a loop");
    // Two cells that touch at (5, 14), walked round as one loop: every step a turn, the loop counter-clockwise, and the
    // sample (5, 14) on it twice.
    Piece pinched = islet;
    pinched.loop = {{4, 13}, {4, 14}, {5, 14}, {5, 15}, {6, 15}, {6, 14}, {5, 14}, {5, 13}};
    pinched.runStarts = {0, 1, 2, 3, 4, 5, 6, 7};
    pinched.cells = 2;
    CHECK(refusal(pinched) == "the footprint's piece whose loop starts at (4, 13) meets itself at (5, 14)");
  }

  SECTION("a block after the surface has been taken") {
    (void)simplifier.takeSurface();
    CHECK_THROWS_AS(simplifier.simplifyBlock({0, 0, 12, 9}, nullptr), std::logic_error);
  }

  // Nothing of a refused block reaches the surface.
  CHECK(simplifier.surface().vertices.empty());
  CHECK(simplifier.surface().triangles.empty());
}

TEST_CASE("a BlockSimplifier refuses a grid, a tolerance or a frame as simplifyGrid does", "[simplify]") {
  constexpr double nan = std::numeric_limits<double>::quiet_NaN();
  const Grid g = makeGrid(9, 7, [](int i, int j) { return i + j; });
  REQUIRE_THROWS_MATCHES(BlockSimplifier(Grid(7, 0.0f), 1, 7, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 1 x 7"));
  REQUIRE_THROWS_MATCHES(BlockSimplifier(Grid(62, 0.0f), 9, 7, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 9 x 7 vertices needs 63 heights, not 62"));
  REQUIRE_THROWS_MATCHES(BlockSimplifier(Grid{}, 65536, 65536, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("a grid of 65536 x 65536 vertices is too large: 32-bit vertex numbers leave room for "
                                 "4294967292 besides the four bottom corners"));
  Grid holed = g;
  holed[cell(9, 4, 3)] = std::numeric_limits<float>::quiet_NaN();
  REQUIRE_THROWS_MATCHES(BlockSimplifier(holed, 9, 7, 1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the height at column 4, row 3 is not finite"));
  REQUIRE_THROWS_MATCHES(BlockSimplifier(g, 9, 7, -1, 10, 10, 1, 1, 0), std::invalid_argument,
                         Message("the tolerance must be finite and not negative, not -1"));
  REQUIRE_THROWS_MATCHES(BlockSimplifier(g, 9, 7, 1, 10, 10, 1, 0, 0), std::invalid_argument,
                         Message("the z-scale and the scale must be above 0 and finite, not 1 and 0"));
  REQUIRE_THROWS_MATCHES(BlockSimplifier(g, 9, 7, 1, 0, 18, 1, 1, 0), std::invalid_argument,
                         Message("the window must be wider and taller than 0 m and finite, not 0 x 18 m"));
  REQUIRE_THROWS_MATCHES(BlockSimplifier(g, 9, 7, 1, 10, 10, 1, 1, nan), std::invalid_argument,
                         Message("the level must be finite, not nan"));
}
