#include "mesh/FootprintSolid.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "mesh/PolygonTriangulation.h"
#include "mesh/Predicates.h"

namespace demdl::mesh {

namespace {

// Vertex numbers are 32 bits: a solid has the numbers 0 to 2^32 - 1, and so 2^32 vertices at most.
constexpr std::uint64_t kMostVertices = std::uint64_t{std::numeric_limits<std::uint32_t>::max()} + 1;

std::string formatNumber(double value, int digits) {
  char text[40];
  std::snprintf(text, sizeof text, "%.*g", digits, value);
  return text;
}

std::string thePiece(std::size_t piece) { return "piece " + std::to_string(piece); }

// Checks the outline of piece number `which` as far as its structure goes, and throws for the first thing wrong. No
// vertex is looked at before its number has been checked, and no point is used before it has been held to the bound.
void checkOutline(const Mesh& surface, const PieceOutline& outline, std::size_t which) {
  const std::size_t n = outline.vertices.size();
  if (n < 3) {
    throw std::invalid_argument(thePiece(which) + " must have at least 3 vertices, not " + std::to_string(n));
  }
  if (outline.points.size() != n) {
    throw std::invalid_argument(thePiece(which) + " has " + std::to_string(n) + " vertices but " +
                                std::to_string(outline.points.size()) + " points: each vertex needs its point");
  }
  for (const std::uint32_t v : outline.vertices) {
    if (v >= surface.vertices.size()) {
      throw std::invalid_argument(thePiece(which) + " names vertex " + std::to_string(v) + ", but the surface has " +
                                  std::to_string(surface.vertices.size()) + " vertices");
    }
  }
  for (const GridPoint p : outline.points) {
    if (p.x < -kMostPredicateCoordinate || p.x > kMostPredicateCoordinate || p.y < -kMostPredicateCoordinate ||
        p.y > kMostPredicateCoordinate) {
      throw std::invalid_argument(thePiece(which) + " has the point (" + std::to_string(p.x) + ", " +
                                  std::to_string(p.y) + "), beyond the " + std::to_string(kMostPredicateCoordinate) +
                                  " that the exact predicates allow: a point is counted from its block's corner");
    }
  }
}

// Adds the shell of piece number `which` to the solid: the bottom copies of its outline's vertices, which are numbered
// from firstBottom, then the walls, then the base (the header has the layout and the winding).
void addShell(Mesh& solid, const Mesh& surface, const PieceOutline& outline, std::size_t which,
              std::uint32_t firstBottom, float planeZ) {
  const auto n = static_cast<std::uint32_t>(outline.vertices.size());
  for (const std::uint32_t v : outline.vertices) {
    const Vec3& top = surface.vertices[v];
    solid.vertices.push_back({top.x, top.y, planeZ});
  }

  // The wall of the edge from the vertex k to the next: seen from the right of the edge, from outside, a is on the left
  // and b on the right, and (a, a', b') and (a, b', b) both run counter-clockwise.
  for (std::uint32_t k = 0; k < n; ++k) {
    const std::uint32_t next = k + 1 == n ? 0 : k + 1;
    const std::uint32_t a = outline.vertices[k], b = outline.vertices[next];
    solid.triangles.push_back({a, firstBottom + k, firstBottom + next});
    solid.triangles.push_back({a, firstBottom + next, b});
  }

  // Rows run southward in `points` and triangulatePolygon wants y to the north, so the rows are negated and its triples
  // turn counter-clockwise seen from above. Written backwards they turn clockwise and face down.
  std::vector<GridPoint> seenFromAbove;
  seenFromAbove.reserve(n);
  for (const GridPoint p : outline.points) seenFromAbove.push_back({p.x, -p.y});
  std::vector<std::array<std::uint32_t, 3>> base;
  try {
    base = triangulatePolygon(seenFromAbove);
  } catch (const std::invalid_argument& refused) {
    throw std::invalid_argument(thePiece(which) + "'s outline cannot be cut into a base: " + refused.what());
  }
  for (const auto& [p, q, r] : base) solid.triangles.push_back({firstBottom + r, firstBottom + q, firstBottom + p});
}

}  // namespace

Mesh solidFromPieces(const Mesh& surface, const std::vector<PieceOutline>& pieces, float planeZ) {
  std::uint64_t vertexCount = surface.vertices.size();
  for (const PieceOutline& outline : pieces) vertexCount += outline.vertices.size();
  if (vertexCount > kMostVertices) {
    throw std::invalid_argument("a solid of " + std::to_string(vertexCount) +
                                " vertices is too large: 32-bit vertex numbers leave room for " +
                                std::to_string(kMostVertices) + ", the surface's " +
                                std::to_string(surface.vertices.size()) + " and a bottom copy of each outline vertex");
  }
  for (std::size_t which = 0; which < pieces.size(); ++which) checkOutline(surface, pieces[which], which);
  if (!std::isfinite(planeZ)) {
    throw std::invalid_argument("the base plane must be finite, not " + formatNumber(planeZ, 9));
  }
  for (std::size_t v = 0; v < surface.vertices.size(); ++v) {
    // Asked this way round so that a NaN height fails as well.
    if (!(surface.vertices[v].z > planeZ)) {
      throw std::invalid_argument("the base plane at " + formatNumber(planeZ, 9) +
                                  " is not below the surface: vertex " + std::to_string(v) + " is at " +
                                  formatNumber(surface.vertices[v].z, 9));
    }
  }

  Mesh solid;
  solid.vertices.reserve(static_cast<std::size_t>(vertexCount));
  solid.vertices.insert(solid.vertices.end(), surface.vertices.begin(), surface.vertices.end());
  solid.triangles.reserve(static_cast<std::size_t>(solidTriangleCount(pieces, surface.triangles.size())));
  solid.triangles.insert(solid.triangles.end(), surface.triangles.begin(), surface.triangles.end());
  // The numbers fit: vertexCount is at most 2^32, so no bottom copy is numbered past 2^32 - 1. Only the number after
  // the last piece's can reach 2^32 and wrap, and it is never used.
  auto firstBottom = static_cast<std::uint32_t>(surface.vertices.size());
  for (std::size_t which = 0; which < pieces.size(); ++which) {
    addShell(solid, surface, pieces[which], which, firstBottom, planeZ);
    firstBottom += static_cast<std::uint32_t>(pieces[which].vertices.size());
  }
  return solid;
}

std::uint64_t solidTriangleCount(const std::vector<PieceOutline>& pieces, std::uint64_t surfaceTriangles) noexcept {
  std::uint64_t count = surfaceTriangles;
  for (const PieceOutline& outline : pieces) {
    const std::uint64_t n = outline.vertices.size();
    if (n >= 3) count += 3 * n - 2;
  }
  return count;
}

}  // namespace demdl::mesh
