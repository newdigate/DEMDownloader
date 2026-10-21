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
#include <utility>
#include <vector>

#include "mesh/Predicates.h"
#include "mesh/ProfileSimplifier.h"
#include "mesh/Simplifier.h"
#include "mesh/SolidMesh.h"

using Catch::Matchers::Message;
using demdl::mesh::checkSolid;
using demdl::mesh::GridPoint;
using demdl::mesh::inCircle;
using demdl::mesh::kChunkCells;
using demdl::mesh::kMostChunkCells;
using demdl::mesh::Mesh;
using demdl::mesh::MeshCheck;
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
std::string whereIs(const SimplifiedSurface& s, std::size_t v, int cols, int rows) {
  if (v >= s.surface.vertices.size()) return "no vertex " + std::to_string(v);
  const std::optional<Sample> at = sampleOf(s.surface.vertices[v], cols, rows);
  return at ? describe(*at) : "on no sample";
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
std::vector<std::int64_t> trianglesHolding(const SimplifiedSurface& s, const std::vector<Sample>& at, int cols,
                                           int rows) {
  std::vector<std::int64_t> holder(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), -1);
  for (std::size_t t = 0; t < s.surface.triangles.size(); ++t) {
    const Triangle& tri = s.surface.triangles[t];
    const Vec3& a = s.surface.vertices.at(tri.a);
    const Vec3& b = s.surface.vertices.at(tri.b);
    const Vec3& c = s.surface.vertices.at(tri.c);
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
double surfaceHeightAt(const SimplifiedSurface& s, const std::vector<std::int64_t>& holder, int cols, int rows, int i,
                       int j) {
  const std::int64_t t = holder.at(cell(cols, i, j));
  if (t < 0) return std::numeric_limits<double>::quiet_NaN();
  const Triangle& tri = s.surface.triangles[static_cast<std::size_t>(t)];
  const Vec3& a = s.surface.vertices[tri.a];
  const Vec3& b = s.surface.vertices[tri.b];
  const Vec3& c = s.surface.vertices[tri.c];
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

  std::vector<std::int64_t> vertexOn(static_cast<std::size_t>(c * r), -1);
  std::vector<Sample> sample(mesh.vertices.size());
  std::string problem;
  for (std::size_t v = 0; v < mesh.vertices.size() && problem.empty(); ++v) {
    const Vec3& p = mesh.vertices[v];
    const std::optional<Sample> at = sampleOf(p, cols, rows);
    if (!at) {
      problem = "vertex " + std::to_string(v) + " at x = " + std::to_string(p.x) + ", y = " + std::to_string(p.y) +
                " stands on no sample";
      break;
    }
    sample[v] = *at;
    const std::size_t k = cell(cols, at->i, at->j);
    if (vertexOn[k] >= 0) {
      problem = "vertices " + std::to_string(vertexOn[k]) + " and " + std::to_string(v) + " both stand on " +
                describe(*at);
    } else if (p.z != g[k]) {
      problem = "vertex " + std::to_string(v) + " on " + describe(*at) + " is at z = " + std::to_string(p.z) +
                ", not the sample's height " + std::to_string(g[k]);
    }
    vertexOn[k] = static_cast<std::int64_t>(v);
  }
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
      problem.empty() ? trianglesHolding(s, sample, cols, rows) : std::vector<std::int64_t>{};
  for (int j = 0; j < rows && problem.empty(); ++j) {
    for (int i = 0; i < cols && problem.empty(); ++i) {
      const double z = surfaceHeightAt(s, holder, cols, rows, i, j);
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
std::string delaunayProblem(const SimplifiedSurface& s, int cols, int rows) {
  std::vector<GridPoint> at;
  for (const Vec3& v : s.surface.vertices) {
    const std::optional<Sample> sample = sampleOf(v, cols, rows);
    if (!sample) return "a vertex stands on no sample";
    at.push_back({sample->i, rows - 1 - sample->j});
  }
  const auto point = [&](std::uint32_t v) { return at.at(v); };
  const auto where = [&](std::uint32_t v) { return describe({at.at(v).x, rows - 1 - at.at(v).y}); };
  std::map<std::pair<std::uint32_t, std::uint32_t>, std::uint32_t> opposite;  // directed edge to the third corner
  for (const Triangle& t : s.surface.triangles) {
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
bool isVertexAt(const SimplifiedSurface& s, int cols, int rows, int i, int j) {
  return std::any_of(s.surface.vertices.begin(), s.surface.vertices.end(), [&](const Vec3& v) {
    const std::optional<Sample> at = sampleOf(v, cols, rows);
    return at && at->i == i && at->j == j;
  });
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
    CHECK(delaunayProblem(simplify(noise, 64, 48, tol), 64, 48) == "");
  }
  CHECK(delaunayProblem(simplify(fractalGrid(), 64, 48, 2), 64, 48) == "");
  std::mt19937 rng(20261021);
  CHECK(delaunayProblem(simplify(randomGrid(rng, 2, 9, 0, 100), 2, 9, 0), 2, 9) == "");
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
