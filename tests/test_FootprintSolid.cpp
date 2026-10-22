#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <numeric>
#include <random>
#include <span>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "IslandFixture.h"
#include "TempDir.h"
#include "mesh/Footprint.h"
#include "mesh/FootprintSolid.h"
#include "mesh/Predicates.h"
#include "mesh/Simplifier.h"
#include "mesh/SolidMesh.h"
#include "mesh/StlWriter.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::Message;
using Catch::Matchers::WithinRel;
using demdl::mesh::BlockFootprint;
using demdl::mesh::blockFootprint;
using demdl::mesh::BlockRect;
using demdl::mesh::BlockResult;
using demdl::mesh::BlockSimplifier;
using demdl::mesh::checkSolid;
using demdl::mesh::downwardFacingTriangles;
using demdl::mesh::footprintCells;
using demdl::mesh::GridPoint;
using demdl::mesh::kMostPredicateCoordinate;
using demdl::mesh::Mesh;
using demdl::mesh::MeshCheck;
using demdl::mesh::orient2d;
using demdl::mesh::PieceOutline;
using demdl::mesh::readBinaryStl;
using demdl::mesh::solidFromPieces;
using demdl::mesh::solidTriangleCount;
using demdl::mesh::Triangle;
using demdl::mesh::Vec3;
using demdl::mesh::writeBinaryStl;

// A footprint solid is held to what spec 6 asks of every solid, by roads that do not go through the code under test:
// checkSolid for the closed mesh and its volume, the volume as the prisms under the surface's triangles, the shells as
// the triangles that share edges, each face's normal from its vertices (the rule of SolidMesh.cpp's normalOf, written
// again below), and the side of a wall that is outside a piece by probing the piece's outline with a point on either
// side. The surfaces are the BlockSimplifier's over the island of IslandFixture.h and over random footprints, and two
// hand-made pieces whose solids are worked out here by hand.

namespace {

using Grid = std::vector<float>;
using Flags = std::vector<std::uint8_t>;  // footprintCells' result: a flag a cell, row by row from the north-west

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

// The test's frame: columns kCellX apart and rows kCellY apart in model units, no exaggeration and no scaling, and a
// level below every height, so that sample (i, j) of a grid of `rows` rows stands at x = kCellX i, y = kCellY
// (rows - 1 - j) and z = its height. The cells are not square, so that a mix-up of the axes shows. The base plane is
// under the lowest height of any grid here.
constexpr double kCellX = 2.0;
constexpr double kCellY = 3.0;
constexpr double kLevel = -1e6;
constexpr double kTolerance = 0.5;
constexpr float kPlane = -5.0f;
// How far from a wall's middle solidProblem looks for the piece's inside and outside: well under the smallest cell of
// any test (0.7 model units) and well over the rounding of a float.
constexpr double kProbe = 0.02;

struct D3 {
  double x, y, z;
};

D3 at(const Mesh& mesh, std::uint32_t v) {
  const Vec3& p = mesh.vertices[v];
  return {p.x, p.y, p.z};
}

// (b - a) x (c - a), in double: it points to the side the triangle faces, the side from which its vertices run
// counter-clockwise, and its length is twice the triangle's area.
D3 normalOf(const Mesh& mesh, const Triangle& t) {
  const D3 a = at(mesh, t.a), b = at(mesh, t.b), c = at(mesh, t.c);
  const D3 u{b.x - a.x, b.y - a.y, b.z - a.z};
  const D3 w{c.x - a.x, c.y - a.y, c.z - a.z};
  return {u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x};
}

// The volume between the surface, the first `surfaceTriangles` triangles of the mesh, and the plane z = plane: for each
// triangle, its area seen from above times its mean height over the plane, which is exact for a surface of flat
// triangles. With shellOf (each triangle's shell, from shellsOf) only the triangles of shell `shell` are counted. It is
// the solid's volume by another road than checkSolid's, which sums over every face and never looks at a plane.
double prismVolume(const Mesh& solid, std::size_t surfaceTriangles, float plane,
                   const std::vector<std::size_t>* shellOf = nullptr, std::size_t shell = 0) {
  double volume = 0;
  for (std::size_t k = 0; k < surfaceTriangles; ++k) {
    if (shellOf != nullptr && (*shellOf)[k] != shell) continue;
    const Triangle& t = solid.triangles[k];
    const double mean = (at(solid, t.a).z + at(solid, t.b).z + at(solid, t.c).z) / 3.0 - plane;
    volume += normalOf(solid, t).z / 2.0 * mean;
  }
  return volume;
}

// a . ((b - a) x (c - a)), six times the signed volume of the tetrahedron from the origin to the triangle.
double sixVolumes(const Mesh& mesh, const Triangle& t) {
  const D3 a = at(mesh, t.a);
  const D3 n = normalOf(mesh, t);
  return a.x * n.x + a.y * n.y + a.z * n.z;
}

// The shells of a mesh: its triangles joined by the edges they share, a vertex shared joining nothing. Each shell has
// a number from 0, in the order of its first triangle.
struct Shells {
  std::vector<std::size_t> of;  // each triangle's shell
  std::size_t count = 0;
};

Shells shellsOf(const Mesh& mesh) {
  std::vector<std::size_t> parent(mesh.triangles.size());
  std::iota(parent.begin(), parent.end(), std::size_t{0});
  const auto find = [&parent](std::size_t t) {
    while (parent[t] != t) {
      parent[t] = parent[parent[t]];
      t = parent[t];
    }
    return t;
  };
  std::map<std::pair<std::uint32_t, std::uint32_t>, std::size_t> firstOnEdge;
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const Triangle& tri = mesh.triangles[t];
    for (const auto& [from, to] : std::array<std::pair<std::uint32_t, std::uint32_t>, 3>{
             {{tri.a, tri.b}, {tri.b, tri.c}, {tri.c, tri.a}}}) {
      const auto edge = std::make_pair(std::min(from, to), std::max(from, to));
      const auto [first, inserted] = firstOnEdge.emplace(edge, t);
      if (!inserted) parent[find(t)] = find(first->second);
    }
  }
  Shells shells;
  std::map<std::size_t, std::size_t> numbered;
  for (std::size_t t = 0; t < mesh.triangles.size(); ++t) {
    const std::size_t root = find(t);
    const auto [found, inserted] = numbered.emplace(root, numbered.size());
    shells.of.push_back(found->second);
  }
  shells.count = numbered.size();
  return shells;
}

// Whether (x, y) lies inside the polygon of the outline's vertices seen from above, by a ray towards +x.
bool inside(const Mesh& mesh, const std::vector<std::uint32_t>& loop, double x, double y) {
  bool in = false;
  for (std::size_t i = 0, j = loop.size() - 1; i < loop.size(); j = i++) {
    const D3 p = at(mesh, loop[i]), q = at(mesh, loop[j]);
    if ((p.y > y) != (q.y > y) && x < (q.x - p.x) * (y - p.y) / (q.y - p.y) + p.x) in = !in;
  }
  return in;
}

// Twice the area of the polygon of the outline's vertices seen from above, positive for counter-clockwise.
double doubledArea(const Mesh& mesh, const std::vector<std::uint32_t>& loop) {
  double sum = 0;
  for (std::size_t i = 0; i < loop.size(); ++i) {
    const D3 p = at(mesh, loop[i]), q = at(mesh, loop[(i + 1) % loop.size()]);
    sum += p.x * q.y - q.x * p.y;
  }
  return sum;
}

std::string describe(const D3& v) {
  return "(" + std::to_string(v.x) + ", " + std::to_string(v.y) + ", " + std::to_string(v.z) + ")";
}

// The number of the first bottom copy of each piece, in the numbering of the header: piece p's k-th outline vertex has
// its copy at first[p] + k.
std::vector<std::size_t> firstCopies(std::size_t surfaceVertices, const std::vector<PieceOutline>& pieces) {
  std::vector<std::size_t> first;
  std::size_t next = surfaceVertices;
  for (const PieceOutline& piece : pieces) {
    first.push_back(next);
    next += piece.vertices.size();
  }
  return first;
}

// The three parts of what spec 6 and the header of solidFromPieces ask of a solid `solid`, made from `surface` and
// `pieces` over the base plane `plane`, each by roads of this file's own and each returning what is wrong or "":
//
// layoutProblem: the counts, the surface as a prefix of the solid, and the bottom copies under their vertices at the
// plane.
std::string layoutProblem(const Mesh& surface, const std::vector<PieceOutline>& pieces, const Mesh& solid,
                          float plane) {
  const std::size_t vertices = surface.vertices.size(), triangles = surface.triangles.size();
  std::size_t copies = 0;
  for (const PieceOutline& piece : pieces) copies += piece.vertices.size();
  if (solid.vertices.size() != vertices + copies) {
    return "the solid has " + std::to_string(solid.vertices.size()) + " vertices, not " +
           std::to_string(vertices + copies);
  }
  if (solid.triangles.size() != solidTriangleCount(pieces, triangles)) {
    return "the solid has " + std::to_string(solid.triangles.size()) + " triangles, not " +
           std::to_string(solidTriangleCount(pieces, triangles));
  }
  for (std::size_t v = 0; v < vertices; ++v) {
    const Vec3 &a = solid.vertices[v], &b = surface.vertices[v];
    if (a.x != b.x || a.y != b.y || a.z != b.z) return "vertex " + std::to_string(v) + " of the surface has changed";
  }
  for (std::size_t t = 0; t < triangles; ++t) {
    const Triangle &a = solid.triangles[t], &b = surface.triangles[t];
    if (a.a != b.a || a.b != b.b || a.c != b.c) return "triangle " + std::to_string(t) + " of the surface has changed";
  }
  const std::vector<std::size_t> first = firstCopies(vertices, pieces);
  for (std::size_t p = 0; p < pieces.size(); ++p) {
    for (std::size_t k = 0; k < pieces[p].vertices.size(); ++k) {
      const Vec3 &top = surface.vertices[pieces[p].vertices[k]], &bottom = solid.vertices[first[p] + k];
      if (bottom.x != top.x || bottom.y != top.y || bottom.z != plane) {
        return "the copy of vertex " + std::to_string(pieces[p].vertices[k]) + " is at " +
               describe(at(solid, static_cast<std::uint32_t>(first[p] + k)));
      }
    }
  }
  return "";
}

// closureProblem: checkSolid finds no bad edge, no triangle without area and a volume above 0, and that volume is the
// prisms under the surface; there is one shell for each piece, and each is closed (a volume above 0 of its own) and
// encloses the prisms under its own part of the surface.
std::string closureProblem(const Mesh& surface, const std::vector<PieceOutline>& pieces, const Mesh& solid,
                           float plane) {
  const MeshCheck check = checkSolid(solid);
  if (!check.edgesChecked) return "the solid is too big for the edge check";
  if (check.nonManifoldEdges != 0) return std::to_string(check.nonManifoldEdges) + " bad edges";
  if (check.zeroAreaTriangles != 0) return std::to_string(check.zeroAreaTriangles) + " triangles without area";
  if (!check.ok()) return "checkSolid is not satisfied";
  const std::size_t triangles = surface.triangles.size();
  const double prisms = prismVolume(solid, triangles, plane);
  if (std::fabs(check.signedVolume - prisms) > 1e-6 * std::fabs(prisms)) {
    return "the volume is " + std::to_string(check.signedVolume) + ", the prisms under the surface " +
           std::to_string(prisms);
  }

  const Shells shells = shellsOf(solid);
  if (shells.count != pieces.size()) {
    return std::to_string(shells.count) + " shells for " + std::to_string(pieces.size()) + " pieces";
  }
  std::vector<double> six(shells.count, 0.0);
  for (std::size_t t = 0; t < solid.triangles.size(); ++t) six[shells.of[t]] += sixVolumes(solid, solid.triangles[t]);
  for (std::size_t s = 0; s < shells.count; ++s) {
    const double want = prismVolume(solid, triangles, plane, &shells.of, s);
    if (!(six[s] > 0) || std::fabs(six[s] / 6 - want) > 1e-6 * std::fabs(want)) {
      return "shell " + std::to_string(s) + " encloses " + std::to_string(six[s] / 6) +
             ", the prisms under its part of the surface " + std::to_string(want);
    }
  }
  return "";
}

// facesProblem: every triangle past the surface's is a wall or a base of one piece. A base triangle has its three
// corners among the bottom copies and faces straight down. A wall triangle has a corner or two among the copies and the
// others on the outline, is upright, and has a point just outside the piece on the side it faces and one just inside on
// the other, wherever the outline bends. Per piece there are 2 n wall triangles and n - 2 base triangles, the base
// covering the piece's polygon once.
std::string facesProblem(const Mesh& surface, const std::vector<PieceOutline>& pieces, const Mesh& solid) {
  const std::size_t vertices = surface.vertices.size(), triangles = surface.triangles.size();
  const std::vector<std::size_t> first = firstCopies(vertices, pieces);
  const auto pieceOfCopy = [&](std::uint32_t v) {
    for (std::size_t p = 0; p < pieces.size(); ++p) {
      if (v >= first[p] && v < first[p] + pieces[p].vertices.size()) return p;
    }
    return pieces.size();
  };
  std::vector<std::size_t> walls(pieces.size(), 0), bases(pieces.size(), 0);
  std::vector<double> baseArea(pieces.size(), 0.0);
  for (std::size_t t = triangles; t < solid.triangles.size(); ++t) {
    const Triangle& tri = solid.triangles[t];
    const std::array<std::uint32_t, 3> corners = {tri.a, tri.b, tri.c};
    std::size_t piece = pieces.size(), below = 0;
    for (const std::uint32_t v : corners) {
      if (v < vertices) continue;
      ++below;
      const std::size_t owner = pieceOfCopy(v);
      if (piece != pieces.size() && owner != piece) {
        return "triangle " + std::to_string(t) + " joins two pieces' copies";
      }
      piece = owner;
    }
    if (below == 0 || piece == pieces.size()) return "triangle " + std::to_string(t) + " has no bottom copy";
    const std::vector<std::uint32_t>& loop = pieces[piece].vertices;
    const D3 n = normalOf(solid, tri);
    if (below == 3) {
      if (n.x != 0 || n.y != 0 || !(n.z < 0)) return "base triangle " + std::to_string(t) + " faces " + describe(n);
      baseArea[piece] += -n.z / 2;
      ++bases[piece];
      continue;
    }
    for (const std::uint32_t v : corners) {
      if (v < vertices && std::find(loop.begin(), loop.end(), v) == loop.end()) {
        return "wall triangle " + std::to_string(t) + " names vertex " + std::to_string(v) + ", not on piece " +
               std::to_string(piece) + "'s outline";
      }
    }
    const double along = std::sqrt(n.x * n.x + n.y * n.y);
    if (!(along > 0) || std::fabs(n.z) > 1e-9 * along) {
      return "wall triangle " + std::to_string(t) + " is not upright: it faces " + describe(n);
    }
    // A point a little way out from the triangle's middle, on the side it faces, must be outside the piece, and one on
    // the other side inside it.
    const D3 a = at(solid, tri.a), b = at(solid, tri.b), c = at(solid, tri.c);
    const double x = (a.x + b.x + c.x) / 3, y = (a.y + b.y + c.y) / 3;
    const double step = kProbe / along;
    if (inside(solid, loop, x + n.x * step, y + n.y * step) || !inside(solid, loop, x - n.x * step, y - n.y * step)) {
      return "wall triangle " + std::to_string(t) + " of piece " + std::to_string(piece) + " faces " + describe(n) +
             ", into the piece";
    }
    ++walls[piece];
  }
  for (std::size_t p = 0; p < pieces.size(); ++p) {
    const std::size_t n = pieces[p].vertices.size();
    if (walls[p] != 2 * n || bases[p] != n - 2) {
      return "piece " + std::to_string(p) + " has " + std::to_string(walls[p]) + " wall triangles and " +
             std::to_string(bases[p]) + " base triangles, not " + std::to_string(2 * n) + " and " +
             std::to_string(n - 2);
    }
    const double polygon = doubledArea(solid, pieces[p].vertices) / 2;
    if (!(polygon > 0) || std::fabs(baseArea[p] - polygon) > 1e-9 * polygon) {
      return "piece " + std::to_string(p) + "'s base covers " + std::to_string(baseArea[p]) + ", its polygon " +
             std::to_string(polygon);
    }
  }
  return "";
}

// Everything above, in order: what is wrong with `solid`, or "". The surface itself must face up, as BlockSimplifier's
// does.
std::string solidProblem(const Mesh& surface, const std::vector<PieceOutline>& pieces, const Mesh& solid, float plane) {
  if (downwardFacingTriangles(surface) != 0) return "the surface itself has a triangle that does not face up";
  if (std::string problem = layoutProblem(surface, pieces, solid, plane); !problem.empty()) return problem;
  if (std::string problem = closureProblem(surface, pieces, solid, plane); !problem.empty()) return problem;
  return facesProblem(surface, pieces, solid);
}

// The solid through an STL and back: written by writeBinaryStl and read by readBinaryStl, which merges the corners
// that stand at one position into one vertex, as STL readers do (Blender's importer, which check_stl in
// blender_scene.py uses, among them). A solid comes back closed only when no two of its vertices that triangles use
// stand at one place.
Mesh throughStl(const Mesh& solid) {
  const TempDir dir;
  writeBinaryStl(solid, dir.path / "solid.stl", "footsolid test");
  return readBinaryStl(dir.path / "solid.stl");
}

// --- Surfaces from BlockSimplifier --------------------------------------------------------------------------------

// A block as it was simplified, with its footprint and the pieces' outlines, and its own surface: every vertex of the
// run, which the pieces' outlines number, and only the block's triangles. (dem2mesh gathers the vertices a block uses
// and renumbers them; a vertex that no triangle uses is harmless here.)
struct SolidBlock {
  BlockRect rect;
  BlockFootprint footprint;
  BlockResult result;
  Mesh surface;
};

// The blocks of a lattice from the north-west corner of a grid of cols x rows samples, blockCols x blockRows cells
// each, the last ones each way cut short, row by row from the north and along each row from the west.
std::vector<BlockRect> blockLattice(int cols, int rows, int blockCols, int blockRows) {
  std::vector<BlockRect> blocks;
  for (int top = 0; top < rows - 1; top += blockRows) {
    for (int left = 0; left < cols - 1; left += blockCols) {
      blocks.push_back({left, top, std::min(left + blockCols, cols - 1), std::min(top + blockRows, rows - 1)});
    }
  }
  return blocks;
}

// The grid g (filled, as the simplifier is given it) simplified at tolerance tol block by block over the footprint
// `cells`, which footprintCells gave with the vertices `joined` of the contacts it joined, in the order of rects, each
// block's pieces from blockFootprint with none dropped. Cells are cellX x cellY model units.
std::vector<SolidBlock> simplifiedBlocks(const Grid& g, const Flags& cells, int cols, int rows, double tol,
                                         const std::vector<BlockRect>& rects, double cellX = kCellX,
                                         double cellY = kCellY, std::span<const GridPoint> joined = {}) {
  BlockSimplifier simplifier(g, cols, rows, tol, cellX * (cols - 1), cellY * (rows - 1), 1, 1, kLevel);
  std::vector<SolidBlock> blocks;
  for (const BlockRect& rect : rects) {
    SolidBlock block{rect, blockFootprint(cells, cols, rows, rect, cellX * cellY, 0.0, joined), {}, {}};
    block.result = simplifier.simplifyBlock(rect, &block.footprint);
    blocks.push_back(std::move(block));
  }
  const Mesh all = simplifier.takeSurface();
  for (SolidBlock& block : blocks) {
    block.surface.vertices = all.vertices;
    const auto from = all.triangles.begin() + static_cast<std::ptrdiff_t>(block.result.firstTriangle);
    block.surface.triangles.assign(from, from + static_cast<std::ptrdiff_t>(block.result.triangles));
  }
  return blocks;
}

// The island simplified in the blocks of rects, at kTolerance.
std::vector<SolidBlock> islandBlocks(const std::vector<BlockRect>& rects) {
  const Grid grid = islandGrid();
  std::vector<GridPoint> joined;
  const Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows, &joined);
  return simplifiedBlocks(grid, cells, kIslandCols, kIslandRows, kTolerance, rects, kCellX, kCellY, joined);
}

// Blocks round one piece of the island each: the sea beyond the piece is a sample or more wide on every side, and the
// block holds no other piece.
constexpr BlockRect kIslandRect{3, 1, 16, 12};  // columns 4 to 14 and rows 2 to 10: the island with its lake and tab
constexpr BlockRect kBlobsRect{16, 1, 22, 7};   // the two blobs, which touched at (19, 4), and their bridge (19, 3)
constexpr BlockRect kIsletRect{3, 12, 7, 16};   // columns 4 to 6 and rows 13 to 15
constexpr BlockRect kWholeRect{0, 0, kIslandCols - 1, kIslandRows - 1};

// The vertices of an outline that are in line with their neighbours, and the ones that turn clockwise seen from above
// (reflex ones). Rows run southward in `points`, so a counter-clockwise turn seen from above is a negative orient2d.
int inLine(const PieceOutline& outline) {
  int count = 0;
  const std::size_t n = outline.points.size();
  for (std::size_t k = 0; k < n; ++k) {
    count += orient2d(outline.points[(k + n - 1) % n], outline.points[k], outline.points[(k + 1) % n]) == 0 ? 1 : 0;
  }
  return count;
}

int reflex(const PieceOutline& outline) {
  int count = 0;
  const std::size_t n = outline.points.size();
  for (std::size_t k = 0; k < n; ++k) {
    count += orient2d(outline.points[(k + n - 1) % n], outline.points[k], outline.points[(k + 1) % n]) > 0 ? 1 : 0;
  }
  return count;
}

}  // namespace

TEST_CASE("a rectangle piece closes into a box: the surface, the bottoms, two triangles an edge, the base",
          "[footsolid]") {
  // A piece of 2 x 1 cells, 4 by 3 model units, with the corners 10, 20, 30 and 40 high, over a base plane at 5. Its
  // outline starts at the north-west corner and goes south, east, north and west, as a piece's loop does, so its
  // points are (0, 0), (0, 1), (2, 1) and (2, 0) in (column, row), and its vertices the corners in that order. The
  // surface is cut along the north-west to south-east diagonal.
  Mesh surface;
  // North-west, south-west, south-east and north-east.
  surface.vertices = {{0, 3, 10}, {0, 0, 20}, {4, 0, 30}, {4, 3, 40}};
  surface.triangles = {{0, 1, 2}, {0, 2, 3}};
  const PieceOutline outline{{0, 1, 2, 3}, {{0, 0}, {0, 1}, {2, 1}, {2, 0}}};
  const Mesh solid = solidFromPieces(surface, {outline}, 5.0f);

  // The vertices: the surface's four, then the copies of the outline's, under each at the plane.
  REQUIRE(solid.vertices.size() == 8);
  const std::array<Vec3, 8> vertices = {{{0, 3, 10}, {0, 0, 20}, {4, 0, 30}, {4, 3, 40},
                                         {0, 3, 5}, {0, 0, 5}, {4, 0, 5}, {4, 3, 5}}};
  for (std::size_t v = 0; v < vertices.size(); ++v) {
    INFO("vertex " << v);
    CHECK((solid.vertices[v].x == vertices[v].x && solid.vertices[v].y == vertices[v].y &&
           solid.vertices[v].z == vertices[v].z));
  }

  // The triangles: the surface's two; then, for the edge from vertex k to the next (a, b) with the copies a' and b',
  // (a, a', b') and (a, b', b); then the base, the two triangles that ear clipping cuts from the square (3, 0, 1 and 3,
  // 1, 2 in the outline's indices) written backwards over the copies.
  const std::array<std::array<std::uint32_t, 3>, 12> triangles = {{
      {0, 1, 2}, {0, 2, 3},  // the surface
      {0, 4, 5}, {0, 5, 1},  // the west wall, from the north-west corner to the south-west one
      {1, 5, 6}, {1, 6, 2},  // the south wall
      {2, 6, 7}, {2, 7, 3},  // the east wall
      {3, 7, 4}, {3, 4, 0},  // the north wall, back to the first vertex
      {5, 4, 7}, {6, 5, 7},  // the base
  }};
  REQUIRE(solid.triangles.size() == triangles.size());
  CHECK(solidTriangleCount({outline}, 2) == 12);
  for (std::size_t t = 0; t < triangles.size(); ++t) {
    INFO("triangle " << t);
    CHECK(solid.triangles[t].a == triangles[t][0]);
    CHECK(solid.triangles[t].b == triangles[t][1]);
    CHECK(solid.triangles[t].c == triangles[t][2]);
  }

  // The prisms under the surface: (10 + 20 + 30) / 3 - 5 = 15 and (10 + 30 + 40) / 3 - 5 = 65 / 3, each over 6 square
  // units, so 90 + 130 = 220.
  const MeshCheck check = checkSolid(solid);
  CHECK(check.ok());
  CHECK(check.nonManifoldEdges == 0);
  CHECK_THAT(check.signedVolume, WithinRel(220.0, 1e-12));
  CHECK(solidProblem(surface, {outline}, solid, 5.0f) == "");
}

TEST_CASE("a piece with a reflex corner and vertices in line closes into one shell", "[footsolid]") {
  // An L of three cells, the 2 x 2 block of cells without its north-east one, every sample of its boundary a vertex:
  //
  //   0 -- 7            (column, row)       0 (0,0)  1 (0,1)  2 (0,2)  3 (1,2)  4 (2,2)  5 (2,1)  6 (1,1)  7 (1,0)
  //   |    |
  //   1    6 -- 5       vertices 1 and 3 stand in line with their neighbours, and 6 is the reflex corner.
  //   |         |
  //   2 -- 3 -- 4
  //
  // The cells are 2 by 3 model units; the heights rise to the east and the south, with a bump of 4 on the reflex
  // corner.
  const auto point = [](std::uint32_t v) {
    constexpr std::array<GridPoint, 8> points = {{{0, 0}, {0, 1}, {0, 2}, {1, 2}, {2, 2}, {2, 1}, {1, 1}, {1, 0}}};
    return points[v];
  };
  Mesh surface;
  PieceOutline outline;
  for (std::uint32_t v = 0; v < 8; ++v) {
    const GridPoint p = point(v);
    surface.vertices.push_back({static_cast<float>(2 * p.x), static_cast<float>(3 * (2 - p.y)),
                                static_cast<float>(10 + 3 * p.x + 2 * p.y + (v == 6 ? 4 : 0))});
    outline.vertices.push_back(v);
    outline.points.push_back(p);
  }
  // Cell (0,0) of vertices 0, 7, 1, 6 (north-west, north-east, south-west, south-east), cell (0,1) of 1, 6, 2, 3 and
  // cell (1,1) of 6, 5, 3, 4, each as the north-west to south-east cut makes it: (nw, sw, se) and (nw, se, ne).
  surface.triangles = {{0, 1, 6}, {0, 6, 7}, {1, 2, 3}, {1, 3, 6}, {6, 3, 4}, {6, 4, 5}};
  REQUIRE(downwardFacingTriangles(surface) == 0);
  REQUIRE(inLine(outline) == 2);
  REQUIRE(reflex(outline) == 1);

  const Mesh solid = solidFromPieces(surface, {outline}, 0.0f);
  CHECK(solid.vertices.size() == 16);
  CHECK(solid.triangles.size() == 6 + 16 + 6);
  CHECK(solidProblem(surface, {outline}, solid, 0.0f) == "");
}

TEST_CASE("the islet closes into a box whose walls face away from its middle", "[footsolid]") {
  // A square of 2 x 2 cells on the plane 20 + 2 i + 1.5 j: every run keeps only its ends at any tolerance, so the
  // outline is the four corners and the surface two triangles, and the solid a box of 8 vertices and 12 triangles with
  // a sloping top. Its walls face away from the middle of the base.
  const std::vector<SolidBlock> blocks = islandBlocks({kIsletRect});
  const SolidBlock& islet = blocks.at(0);
  REQUIRE(islet.result.pieces.size() == 1);
  REQUIRE(islet.result.pieces[0].vertices.size() == 4);
  REQUIRE(islet.surface.triangles.size() == 2);

  const Mesh solid = solidFromPieces(islet.surface, islet.result.pieces, kPlane);
  CHECK(solid.triangles.size() == solidTriangleCount(islet.result.pieces, 2));
  CHECK(solid.triangles.size() == 12);
  CHECK(solidProblem(islet.surface, islet.result.pieces, solid, kPlane) == "");

  // The wall of the edge from vertex k to the next, (a, a', b') and (a, b', b), is the triangles 2 k and 2 k + 1 after
  // the surface's. Each has a normal that is horizontal, perpendicular to the edge and on the right of the walk, which
  // is away from the piece, and away from the middle of the base as well, the box being convex. The middle of the base
  // is the mean of the four copies.
  D3 middle{0, 0, 0};
  for (std::size_t v = islet.surface.vertices.size(); v < solid.vertices.size(); ++v) {
    middle.x += solid.vertices[v].x / 4;
    middle.y += solid.vertices[v].y / 4;
    middle.z += solid.vertices[v].z / 4;
  }
  const PieceOutline& outline = islet.result.pieces[0];
  const std::size_t surfaceTriangles = islet.surface.triangles.size();
  for (std::size_t k = 0; k < 4; ++k) {
    const D3 a = at(solid, outline.vertices[k]), b = at(solid, outline.vertices[(k + 1) % 4]);
    const D3 edge{b.x - a.x, b.y - a.y, 0};
    for (std::size_t half = 0; half < 2; ++half) {
      const Triangle& tri = solid.triangles[surfaceTriangles + 2 * k + half];
      INFO("edge " << k << ", wall triangle " << half);
      const D3 n = normalOf(solid, tri);
      CHECK(n.z == 0);                          // upright
      CHECK(n.x * edge.x + n.y * edge.y == 0);  // perpendicular to the edge
      CHECK(n.x * edge.y - n.y * edge.x > 0);   // on the right of the walk
      const D3 p = at(solid, tri.a), q = at(solid, tri.b), r = at(solid, tri.c);
      const D3 centre{(p.x + q.x + r.x) / 3, (p.y + q.y + r.y) / 3, 0};
      CHECK((centre.x - middle.x) * n.x + (centre.y - middle.y) * n.y > 0);  // away from the middle of the base
    }
  }
  // The base faces down.
  for (std::size_t t = solid.triangles.size() - 2; t < solid.triangles.size(); ++t) {
    const D3 n = normalOf(solid, solid.triangles[t]);
    CHECK((n.x == 0 && n.y == 0 && n.z < 0));
  }
}

TEST_CASE("the island with its lake and tab closes, its outline bending both ways with vertices in line",
          "[footsolid]") {
  // The island's one piece, the lake and channel filled: 9 x 9 samples with a tab on the east, a concave outline. The
  // channel's mouth on the west side keeps a sample there, a vertex in line with its neighbours.
  const std::vector<SolidBlock> blocks = islandBlocks({kIslandRect});
  const SolidBlock& island = blocks.at(0);
  REQUIRE(island.result.pieces.size() == 1);
  const PieceOutline& outline = island.result.pieces[0];
  REQUIRE(outline.vertices.size() > 4);
  REQUIRE(inLine(outline) >= 1);
  REQUIRE(reflex(outline) >= 2);
  REQUIRE(island.surface.vertices.size() > outline.vertices.size());  // the knoll and the lake's heights are inside

  const Mesh solid = solidFromPieces(island.surface, island.result.pieces, kPlane);
  CHECK(solid.vertices.size() == island.surface.vertices.size() + outline.vertices.size());
  CHECK(solid.triangles.size() == island.surface.triangles.size() + 3 * outline.vertices.size() - 2);
  CHECK(solidProblem(island.surface, island.result.pieces, solid, kPlane) == "");

  // Through an STL and back it is the same closed solid: no two of its vertices stand at one place.
  const Mesh merged = throughStl(solid);
  const MeshCheck check = checkSolid(merged);
  CHECK(merged.vertices.size() == solid.vertices.size());
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.ok());
}

TEST_CASE("two blobs that touched at a vertex are one piece, joined by a bridge cell, and close into one shell, "
          "through an STL and back too",
          "[footsolid]") {
  // Spec 5.2 as the footprint now has it: the blobs of 2 x 2 cells touched at the sample (19, 4), and footprintCells
  // put in the cell (19, 3) there, so that they are one piece of 9 cells with one outline, which closes into one shell.
  // No two of its vertices stand at one place, so it comes back from an STL as it went: an STL reader that merges the
  // corners at one position merges none.
  const std::vector<SolidBlock> blocks = islandBlocks({kBlobsRect});
  const SolidBlock& blobs = blocks.at(0);
  REQUIRE(blobs.result.pieces.size() == 1);
  CHECK(blobs.footprint.pieces.at(0).cells == 9);
  CHECK(blobs.footprint.cornerContacts == 1);

  const Mesh solid = solidFromPieces(blobs.surface, blobs.result.pieces, kPlane);
  CHECK(solidProblem(blobs.surface, blobs.result.pieces, solid, kPlane) == "");
  CHECK(shellsOf(solid).count == 1);

  const Mesh merged = throughStl(solid);
  const MeshCheck check = checkSolid(merged);
  CHECK(check.triangles == solid.triangles.size());
  CHECK(merged.vertices.size() == solid.vertices.size());
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.ok());
}

TEST_CASE("outlines that touch at a vertex make two shells, which a reader that merges corners finds an edge of four "
          "faces under the contact",
          "[footsolid]") {
  // footprintCells joins pieces that touch at a vertex (Footprint.h), so no block's outlines do; this case shows why.
  // Outlines that touch anyway, here two squares of one cell made by hand that share the vertex (1, 1), make two
  // shells that meet at that one top vertex, each with a bottom copy of its own under it: in memory every edge has its
  // two triangles, and checkSolid finds no bad edge. But an STL has no shared vertices, and its readers merge the
  // corners that stand at one position (readBinaryStl does, and so does Blender's importer, as check_stl in
  // blender_scene.py says): the two copies under the contact become one vertex, and the upright side that both shells
  // have under it, the side of the wall quads of the edges that meet there, becomes one edge in four triangles.
  // Whatever the numbering, the geometry is two prisms touching along a line.
  //
  //   0 -- 3              (column, row)    0 (0,0)  1 (0,1)  2 (1,1)  3 (1,0)     square A, the cell (0, 0)
  //   |  A |                               2 (1,1)  4 (1,2)  5 (2,2)  6 (2,1)     square B, the cell (1, 1)
  //   1 -- 2 -- 6
  //        |  B |         The cells are kCellX by kCellY model units, the vertices 10 to 16 high, the plane at 0.
  //        4 -- 5
  const std::array<GridPoint, 7> points = {{{0, 0}, {0, 1}, {1, 1}, {1, 0}, {1, 2}, {2, 2}, {2, 1}}};
  Mesh surface;
  for (std::uint32_t v = 0; v < points.size(); ++v) {
    surface.vertices.push_back({static_cast<float>(kCellX * points[v].x),
                                static_cast<float>(kCellY * (2 - points[v].y)), static_cast<float>(10 + v)});
  }
  // Each square cut along its north-west to south-east diagonal: (nw, sw, se) and (nw, se, ne).
  surface.triangles = {{0, 1, 2}, {0, 2, 3}, {2, 4, 5}, {2, 5, 6}};
  REQUIRE(downwardFacingTriangles(surface) == 0);
  const std::vector<PieceOutline> pieces{{{0, 1, 2, 3}, {points[0], points[1], points[2], points[3]}},
                                         {{2, 4, 5, 6}, {points[2], points[4], points[5], points[6]}}};
  const Mesh solid = solidFromPieces(surface, pieces, 0.0f);
  CHECK(solidProblem(surface, pieces, solid, 0.0f) == "");
  REQUIRE(checkSolid(solid).nonManifoldEdges == 0);
  REQUIRE(shellsOf(solid).count == 2);
  const Vec3 contact = surface.vertices[2];

  const Mesh merged = throughStl(solid);
  const MeshCheck check = checkSolid(merged);
  CHECK(check.triangles == solid.triangles.size());
  CHECK(merged.vertices.size() == solid.vertices.size() - 1);  // the two copies under the contact are one now
  CHECK(check.nonManifoldEdges == 1);
  CHECK_FALSE(check.ok());

  // That edge is the upright line from the contact down to the plane.
  const auto indexAt = [&](const Vec3& p) {
    for (std::size_t v = 0; v < merged.vertices.size(); ++v) {
      if (merged.vertices[v].x == p.x && merged.vertices[v].y == p.y && merged.vertices[v].z == p.z) return v;
    }
    return merged.vertices.size();
  };
  const std::size_t top = indexAt(contact), bottom = indexAt({contact.x, contact.y, 0.0f});
  REQUIRE(top < merged.vertices.size());
  REQUIRE(bottom < merged.vertices.size());
  std::size_t faces = 0;
  for (const Triangle& t : merged.triangles) {
    const std::array<std::uint32_t, 3> corners = {t.a, t.b, t.c};
    const auto has = [&](std::size_t v) { return std::find(corners.begin(), corners.end(), v) != corners.end(); };
    if (has(top) && has(bottom)) ++faces;
  }
  CHECK(faces == 4);
}

TEST_CASE("the island's three pieces in one block make three closed shells, through an STL and back too",
          "[footsolid]") {
  // The whole grid as one block: the island with the tab, the joined blobs and the islet (the lone sample makes no
  // cell). The numbering of the bottoms goes piece by piece in the order of the pieces. No two pieces touch, so the
  // solid comes back from an STL as it went.
  const std::vector<SolidBlock> blocks = islandBlocks({kWholeRect});
  const SolidBlock& all = blocks.at(0);
  REQUIRE(all.result.pieces.size() == 3);
  const Mesh solid = solidFromPieces(all.surface, all.result.pieces, kPlane);
  CHECK(solidProblem(all.surface, all.result.pieces, solid, kPlane) == "");
  CHECK(checkSolid(solid).ok());

  const Mesh merged = throughStl(solid);
  const MeshCheck check = checkSolid(merged);
  CHECK(merged.vertices.size() == solid.vertices.size());
  CHECK(check.nonManifoldEdges == 0);
  CHECK(check.ok());
}

TEST_CASE("blocks of a lattice, the last ones cut short, close piece by piece", "[footsolid]") {
  // Spec 5.3, Review Focus 3: the island in lattices whose blocks do not divide the 23 x 17 cells, so that pieces are
  // cut along the sides of blocks and the blocks of the east and south are short. 13 x 10 is the lattice of the brief.
  // Where a short block holds land its pieces close too; in 7 x 5 and 3 x 3 blocks the short ones, 2 columns or 2 rows
  // wide, fall in the sea beyond the island's tab, the blobs and the islet.
  struct Lattice {
    int blockCols, blockRows;
    bool shortBlockHoldsLand;
  };
  for (const Lattice& lattice : {Lattice{13, 10, true}, Lattice{12, 9, true}, Lattice{7, 5, false},
                                 Lattice{5, 4, true}, Lattice{3, 3, false}}) {
    INFO("blocks of " << lattice.blockCols << " x " << lattice.blockRows << " cells");
    const std::vector<BlockRect> rects = blockLattice(kIslandCols, kIslandRows, lattice.blockCols, lattice.blockRows);
    const std::vector<SolidBlock> blocks = islandBlocks(rects);
    std::size_t pieces = 0, shortWithPieces = 0;
    for (const SolidBlock& block : blocks) {
      if (block.result.pieces.empty()) continue;
      const BlockRect& rect = block.rect;
      INFO("the block of columns " << rect.firstColumn << " to " << rect.lastColumn << " and rows " << rect.firstRow
                                   << " to " << rect.lastRow);
      const Mesh solid = solidFromPieces(block.surface, block.result.pieces, kPlane);
      CHECK(solidProblem(block.surface, block.result.pieces, solid, kPlane) == "");
      pieces += block.result.pieces.size();
      if (rect.lastColumn - rect.firstColumn < lattice.blockCols || rect.lastRow - rect.firstRow < lattice.blockRows) {
        ++shortWithPieces;
      }
    }
    // The island's pieces are all there, cut or not, and where a short block holds land its pieces close too.
    CHECK(pieces >= 4);
    if (lattice.shortBlockHoldsLand) CHECK(shortWithPieces >= 1);
  }
}

TEST_CASE("random footprints in random lattices close in every block", "[footsolid]") {
  // Masks of rectangles of data with rectangles of no-data cut out, pairs of rectangles that share a corner sample, and
  // noise, so that coasts, lakes, islets, slits and pieces touching at a vertex (which footprintCells joins) all turn
  // up; smooth, noisy or whole-number heights; cells of other sizes than the island's; a random tolerance and lattice.
  // Every block with pieces is held to solidProblem, and its solid, through an STL and back, to checkSolid.
  std::mt19937 rng(20261022);
  // Every draw is made from the engine's output by hand and into a variable of its own, since the standard fixes the
  // engine's output but neither a distribution's nor the order a call's arguments are worked out in.
  const auto draw = [&rng](int below) { return static_cast<int>(rng() % static_cast<unsigned>(below)); };
  std::size_t blocksWithPieces = 0, pieces = 0, contacts = 0, vertices = 0;
  for (int trial = 0; trial < 80; ++trial) {
    const int cols = 3 + draw(26);
    const int rows = 3 + draw(26);
    Flags mask(static_cast<std::size_t>(cols) * static_cast<std::size_t>(rows), 0);
    const auto fill = [&](int x0, int y0, int w, int h, std::uint8_t value) {
      for (int y = std::max(y0, 0); y < std::min(y0 + h, rows); ++y) {
        for (int x = std::max(x0, 0); x < std::min(x0 + w, cols); ++x) mask[cell(cols, x, y)] = value;
      }
    };
    const int maskStyle = draw(3);
    if (maskStyle == 0) {
      const int land = 1 + draw(5), water = draw(9), most = std::max(cols, rows);
      for (int k = 0; k < land + water; ++k) {
        const int x = draw(cols), y = draw(rows);
        const int w = 1 + draw(k < land ? most : 4), h = 1 + draw(k < land ? most : 4);
        fill(x, y, w, h, k < land ? 1 : 0);
      }
    } else if (maskStyle == 1) {
      for (int pairs = 1 + draw(4); pairs > 0; --pairs) {
        const int x = draw(cols), y = draw(rows), a = 2 + draw(4), b = 2 + draw(4), c = 2 + draw(4), d = 2 + draw(4);
        const bool southEast = draw(2) == 0;
        fill(x, y, a, b, 1);
        fill(southEast ? x + a - 1 : x - c + 1, y + b - 1, c, d, 1);  // shares a corner sample of the first
      }
    } else {
      const int percent = 40 + draw(58);
      for (std::uint8_t& v : mask) v = draw(100) < percent ? 1 : 0;
    }
    std::vector<GridPoint> joined;
    const Flags cells = footprintCells(mask, cols, rows, &joined);

    const int heightStyle = draw(3);
    const double phase = static_cast<double>(draw(628)) / 100.0;
    Grid g;
    if (heightStyle == 0) {
      g = makeGrid(cols, rows, [phase](int i, int j) {
        return 6 * std::sin(i / 3.0 + phase) * std::cos(j / 4.0) + 0.3 * i - 0.2 * j;
      });
    } else if (heightStyle == 1) {
      g = makeGrid(cols, rows, [&](int, int) { return static_cast<double>(draw(100000)) / 10000.0; });
    } else {
      g = makeGrid(cols, rows, [&](int, int) { return static_cast<double>(draw(3)); });
    }
    const double tolerance = std::array{0.0, 0.25, 1.0, 4.0}[static_cast<std::size_t>(draw(4))];
    const double cellX = std::array{kCellX, 2.3, 0.7}[static_cast<std::size_t>(draw(3))];
    const double cellY = std::array{kCellY, 3.1, 1.9}[static_cast<std::size_t>(draw(3))];
    const int blockCols = 1 + draw(cols - 1), blockRows = 1 + draw(rows - 1);
    INFO("trial " << trial << ": " << cols << " x " << rows << " samples, cells " << cellX << " x " << cellY
                  << ", heights of style " << heightStyle << ", tolerance " << tolerance << ", blocks of " << blockCols
                  << " x " << blockRows);

    const std::vector<SolidBlock> blocks =
        simplifiedBlocks(g, cells, cols, rows, tolerance, blockLattice(cols, rows, blockCols, blockRows), cellX, cellY,
                         joined);
    for (const SolidBlock& block : blocks) {
      if (block.result.pieces.empty()) continue;
      INFO("the block of columns " << block.rect.firstColumn << " to " << block.rect.lastColumn << " and rows "
                                   << block.rect.firstRow << " to " << block.rect.lastRow);
      // The heights run from -6 - 0.2 * 28 = -11.6 at the lowest (the smooth style) to 14.4, so a plane at -20 is under
      // every vertex.
      const Mesh solid = solidFromPieces(block.surface, block.result.pieces, -20.0f);
      const std::string problem = solidProblem(block.surface, block.result.pieces, solid, -20.0f);
      CHECK(problem == "");
      // No two of the block's pieces touch, so the solid comes back from an STL still closed.
      const MeshCheck merged = checkSolid(throughStl(solid));
      CHECK(merged.nonManifoldEdges == 0);
      CHECK(merged.ok());
      ++blocksWithPieces;
      pieces += block.result.pieces.size();
      contacts += block.footprint.cornerContacts;
      vertices += solid.vertices.size();
    }
  }
  // The draw is not one-sided: pieces in many blocks, and corner contacts that footprintCells joined among them.
  CHECK(blocksWithPieces > 450);
  CHECK(pieces > 500);
  CHECK(contacts > 10);
  CHECK(vertices > 40000);
}

TEST_CASE("no pieces make no shell: the solid is the surface", "[footsolid]") {
  const std::vector<SolidBlock> blocks = islandBlocks({kIsletRect});
  const SolidBlock& islet = blocks.at(0);
  const Mesh solid = solidFromPieces(islet.surface, {}, kPlane);
  REQUIRE(solid.vertices.size() == islet.surface.vertices.size());
  REQUIRE(solid.triangles.size() == islet.surface.triangles.size());
  for (std::size_t v = 0; v < solid.vertices.size(); ++v) {
    const Vec3 &got = solid.vertices[v], &want = islet.surface.vertices[v];
    CHECK((got.x == want.x && got.y == want.y && got.z == want.z));
  }
  for (std::size_t t = 0; t < solid.triangles.size(); ++t) {
    const Triangle &got = solid.triangles[t], &want = islet.surface.triangles[t];
    CHECK((got.a == want.a && got.b == want.b && got.c == want.c));
  }
  // The plane is still held under the surface, pieces or none.
  CHECK_THROWS_AS(solidFromPieces(islet.surface, {}, 1000.0f), std::invalid_argument);
}

namespace {

// A surface, an outline and a plane that solidFromPieces accepts, for the refusals to spoil one thing at a time: the
// box of the first case, over a plane at 5.
struct Box {
  Mesh surface;
  PieceOutline outline{{0, 1, 2, 3}, {{0, 0}, {0, 1}, {2, 1}, {2, 0}}};
  float plane = 5.0f;

  Box() {
    surface.vertices = {{0, 3, 10}, {0, 0, 20}, {4, 0, 30}, {4, 3, 40}};
    surface.triangles = {{0, 1, 2}, {0, 2, 3}};
  }

  // What solidFromPieces says when it refuses with std::invalid_argument, or "" when it builds the solid.
  [[nodiscard]] std::string refusal(const std::vector<PieceOutline>& pieces) const {
    try {
      static_cast<void>(solidFromPieces(surface, pieces, plane));
    } catch (const std::invalid_argument& refused) {
      return refused.what();
    }
    return "";
  }
  [[nodiscard]] std::string refusal() const { return refusal({outline}); }
};

}  // namespace

TEST_CASE("an outline of fewer than 3 vertices makes no shell", "[footsolid]") {
  Box box;
  REQUIRE(box.refusal() == "");
  box.outline = {{0, 1}, {{0, 0}, {0, 1}}};
  CHECK(box.refusal() == "piece 0 must have at least 3 vertices, not 2");
  box.outline = {{}, {}};
  CHECK(box.refusal() == "piece 0 must have at least 3 vertices, not 0");
  // The piece is named by its place in the list.
  const Box good;
  CHECK(box.refusal({good.outline, good.outline, {{0}, {{0, 0}}}}) == "piece 2 must have at least 3 vertices, not 1");
  REQUIRE_THROWS_AS(solidFromPieces(box.surface, {box.outline}, box.plane), std::invalid_argument);
}

TEST_CASE("an outline whose points do not match its vertices is refused", "[footsolid]") {
  Box box;
  box.outline.points.pop_back();
  CHECK(box.refusal() == "piece 0 has 4 vertices but 3 points: each vertex needs its point");
  box.outline.points.push_back({2, 0});
  box.outline.points.push_back({1, 0});
  CHECK(box.refusal() == "piece 0 has 4 vertices but 5 points: each vertex needs its point");
}

TEST_CASE("an outline that names a vertex the surface lacks is refused", "[footsolid]") {
  Box box;
  box.outline.vertices[3] = 4;
  CHECK(box.refusal() == "piece 0 names vertex 4, but the surface has 4 vertices");
  box.outline.vertices[3] = 4294967295U;
  CHECK(box.refusal() == "piece 0 names vertex 4294967295, but the surface has 4 vertices");
}

TEST_CASE("a point beyond the exact predicates' bound is refused, one on the bound is not", "[footsolid]") {
  static_assert(kMostPredicateCoordinate == 16384);
  Box box;
  // The same square 16384 columns and rows out: on the bound exactly.
  box.outline.points = {{0, 0}, {0, 16384}, {16384, 16384}, {16384, 0}};
  CHECK(box.refusal() == "");
  box.outline.points[2] = {16385, 16384};
  CHECK(box.refusal() ==
        "piece 0 has the point (16385, 16384), beyond the 16384 that the exact predicates allow: a point is counted "
        "from its block's corner");
  box.outline.points[2] = {2, -16385};
  CHECK_THAT(box.refusal(), ContainsSubstring("the point (2, -16385)"));
  // The bound is judged before the point is negated: the extreme of an int is no overflow.
  box.outline.points[2] = {std::numeric_limits<std::int32_t>::min(), std::numeric_limits<std::int32_t>::min()};
  CHECK_THAT(box.refusal(), ContainsSubstring("beyond the 16384"));
}

TEST_CASE("a clockwise outline, or one without area, is refused", "[footsolid]") {
  Box box;
  // The same square the other way round: the points and the vertices both reversed.
  box.outline = {{3, 2, 1, 0}, {{2, 0}, {2, 1}, {0, 1}, {0, 0}}};
  CHECK_THAT(box.refusal(), ContainsSubstring("piece 0's outline cannot be cut into a base: the polygon is clockwise"));
  // Three vertices in line.
  box.outline = {{0, 1, 2}, {{0, 0}, {1, 0}, {2, 0}}};
  CHECK_THAT(box.refusal(), ContainsSubstring("piece 0's outline cannot be cut into a base: the polygon has no area"));
  // A point twice.
  box.outline = {{0, 1, 2, 3}, {{0, 0}, {0, 1}, {2, 1}, {0, 0}}};
  CHECK_THAT(box.refusal(), ContainsSubstring("piece 0's outline cannot be cut into a base: the polygon repeats"));
  // The piece is named by its place in the list.
  const Box good;
  const PieceOutline clockwise{{3, 2, 1, 0}, {{2, 0}, {2, 1}, {0, 1}, {0, 0}}};
  CHECK_THAT(box.refusal({good.outline, clockwise}), ContainsSubstring("piece 1's outline"));
}

TEST_CASE("a base plane that is not finite, or not under every vertex of the surface, is refused", "[footsolid]") {
  Box box;
  box.plane = std::numeric_limits<float>::quiet_NaN();
  CHECK(box.refusal() == "the base plane must be finite, not nan");
  box.plane = std::numeric_limits<float>::infinity();
  CHECK(box.refusal() == "the base plane must be finite, not inf");
  box.plane = -std::numeric_limits<float>::infinity();
  CHECK(box.refusal() == "the base plane must be finite, not -inf");

  // A vertex at the plane has no wall to stand on, and neither has one below it. The first is named.
  box.plane = 10.0f;
  CHECK(box.refusal() == "the base plane at 10 is not below the surface: vertex 0 is at 10");
  box.plane = 25.0f;
  CHECK(box.refusal() == "the base plane at 25 is not below the surface: vertex 0 is at 10");
  box.plane = 9.99f;
  CHECK(box.refusal() == "");

  // A vertex that no outline names is held above the plane as well, and a height that is not a number fails.
  box.plane = 5.0f;
  box.surface.vertices.push_back({1, 1, 5.0f});
  CHECK(box.refusal() == "the base plane at 5 is not below the surface: vertex 4 is at 5");
  box.surface.vertices[4].z = std::numeric_limits<float>::quiet_NaN();
  CHECK(box.refusal() == "the base plane at 5 is not below the surface: vertex 4 is at nan");
  box.surface.vertices[4].z = 6.0f;
  CHECK(box.refusal() == "");
}

TEST_CASE("the refusals come in the order of the header", "[footsolid]") {
  Box box;
  // A structure fault before the plane's, and the plane's before an outline that only the triangulation can refuse.
  box.plane = std::numeric_limits<float>::quiet_NaN();
  box.outline = {{0, 1}, {{0, 0}, {0, 1}}};
  CHECK(box.refusal() == "piece 0 must have at least 3 vertices, not 2");
  box.outline = {{3, 2, 1, 0}, {{2, 0}, {2, 1}, {0, 1}, {0, 0}}};
  CHECK(box.refusal() == "the base plane must be finite, not nan");
  box.plane = 99.0f;
  CHECK(box.refusal() == "the base plane at 99 is not below the surface: vertex 0 is at 10");
  box.plane = 5.0f;
  CHECK_THAT(box.refusal(), ContainsSubstring("clockwise"));
  // The structure of every piece is judged before any is cut: a later piece's fault is reported before an earlier
  // piece's clockwise outline.
  const Box good;
  CHECK(box.refusal({box.outline, {{0, 1}, {{0, 0}, {0, 1}}}}) == "piece 1 must have at least 3 vertices, not 2");
  CHECK_THAT(box.refusal({good.outline, box.outline}), ContainsSubstring("piece 1's outline cannot be cut"));
}

TEST_CASE("solidTriangleCount is the surface's triangles and 3 n - 2 for each piece of n vertices", "[footsolid]") {
  const auto piece = [](std::size_t n) {
    PieceOutline outline;
    outline.vertices.assign(n, 0);
    outline.points.assign(n, GridPoint{0, 0});
    return outline;
  };
  const std::vector<PieceOutline> none;
  CHECK(solidTriangleCount(none, 0) == 0);
  CHECK(solidTriangleCount(none, 7) == 7);
  CHECK(solidTriangleCount({piece(3)}, 10) == 10 + 7);  // 6 wall triangles and a base of 1
  CHECK(solidTriangleCount({piece(4)}, 2) == 12);       // the box
  CHECK(solidTriangleCount({piece(4), piece(8)}, 5) == 5 + 10 + 22);
  CHECK(solidTriangleCount({piece(1000)}, 0) == 2998);
  // Counted in 64 bits.
  const std::uint64_t most = std::uint64_t{1} << 40;
  CHECK(solidTriangleCount({piece(4)}, most) == most + 10);
  // A piece of fewer than 3 vertices, which solidFromPieces refuses, adds nothing.
  CHECK(solidTriangleCount({piece(0), piece(1), piece(2)}, 9) == 9);
  static_assert(noexcept(solidTriangleCount(std::declval<const std::vector<PieceOutline>&>(), std::uint64_t{0})));

  // And it is the size of the solid that solidFromPieces makes.
  const Box box;
  CHECK(solidFromPieces(box.surface, {box.outline, box.outline}, 5.0f).triangles.size() ==
        solidTriangleCount({box.outline, box.outline}, box.surface.triangles.size()));
}
