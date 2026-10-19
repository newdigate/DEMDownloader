#include "mesh/SolidMesh.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace demdl::mesh {

namespace {

// Vertex numbers are 32 bits, and the solid adds its four bottom corners to the grid's vertices.
constexpr std::uint64_t kMostGridVertices = std::uint64_t{std::numeric_limits<std::uint32_t>::max()} + 1 - 4;

std::string formatNumber(double value, int digits) {
  char text[40];
  std::snprintf(text, sizeof text, "%.*g", digits, value);
  return text;
}

std::string gridSize(int cols, int rows) { return std::to_string(cols) + " x " + std::to_string(rows); }

// What the surface and the solid both need of the grid: at least 2 x 2, a cell, and few enough vertices for 32-bit
// vertex numbers with the four bottom corners. Returns the number of vertices.
std::uint64_t checkGrid(int cols, int rows) {
  if (cols < 2 || rows < 2) {
    throw std::invalid_argument("the grid must be at least 2 x 2 vertices, not " + gridSize(cols, rows));
  }
  // Multiplied in 64 bits: 65536 x 65536 is past what an int or any 32-bit count holds.
  const std::uint64_t count = static_cast<std::uint64_t>(cols) * static_cast<std::uint64_t>(rows);
  if (count > kMostGridVertices) {
    throw std::invalid_argument("a grid of " + gridSize(cols, rows) +
                                " vertices is too large: 32-bit vertex numbers leave room for " +
                                std::to_string(kMostGridVertices) + " besides the four bottom corners");
  }
  return count;
}

struct Vec3d {
  double x, y, z;
};

Vec3d toDouble(const Vec3& v) { return {v.x, v.y, v.z}; }

// (b - a) x (c - a), in double from the float coordinates: it points to the side the triangle faces, the side from
// which its vertices run counter-clockwise, and its length is twice the triangle's area. solidMesh decides which way a
// wall triangle faces with this very function, and checkSolid measures areas with it, so a triangle the walls were
// cut with because it faces out is also one that checkSolid finds has an area.
Vec3d normalOf(const Vec3& a, const Vec3& b, const Vec3& c) {
  const Vec3d p = toDouble(a), q = toDouble(b), r = toDouble(c);
  const Vec3d u{q.x - p.x, q.y - p.y, q.z - p.z};
  const Vec3d w{r.x - p.x, r.y - p.y, r.z - p.z};
  return {u.y * w.z - u.z * w.y, u.z * w.x - u.x * w.z, u.x * w.y - u.y * w.x};
}

// One side of the solid, walked counter-clockwise seen from above: its top vertices are first + k step for k from 0 to
// count - 1, startCorner and endCorner are the bottom corners under the first and the last, and the wall faces along
// axis (0 for x, 1 for y) towards sign.
struct Side {
  std::int64_t first, step, count;
  std::uint32_t startCorner, endCorner;
  int axis;
  double sign;
};

// Twice the area of triangle (a, b, c) of a wall, positive when it faces out of the solid and negative when it faces
// in. A wall's vertices all lie in the plane of its side, so the normal lies along the side's axis.
double areaFacingOut(const Mesh& mesh, const Side& side, std::uint32_t a, std::uint32_t b, std::uint32_t c) {
  const Vec3d n = normalOf(mesh.vertices[a], mesh.vertices[b], mesh.vertices[c]);
  return side.sign * (side.axis == 0 ? n.x : n.y);
}

// Cuts one wall into triangles facing out (the header says why). The wall is the polygon whose bottom edge runs from
// startCorner to endCorner and whose top vertices stand above it, each one further along the side than the one before
// and all above the plane: a monotone polygon, cut without new vertices the textbook way, with a stack.
// - The stack holds the start corner and, above it, the top vertices still waiting for the triangles under them. No
//   vertex on it sticks up above the line from the one under it to the one over it, so the run they make climbs from
//   the start corner ever more steeply, like the side of a bowl.
// - A new top vertex closes a triangle with the top two of the stack when that triangle faces out, which is when the
//   top of the stack sticks up above the line from the one under it to the new vertex: the triangle then lies inside
//   the polygon, under the top of the stack, which it takes off. That goes on down the stack until a triangle would
//   not face out, and then the new vertex waits on the stack too.
// - At the end every vertex still waiting is seen from the end corner: the run climbs ever more steeply, so the line
//   from the end corner to any of its vertices passes under the rest of it. A fan from the end corner over the run
//   closes the wall, and its first triangle holds the bottom edge.
// The first new vertex always closes the triangle of the start corner and the first two top vertices, the start corner
// being right under the first. So when every triangle of the plan's fan from the start corner faces out, each new
// vertex closes just its own triangle of that fan, the stack is then the start corner and the newest vertex, and the
// end fan is the plan's closing triangle. Every top vertex after the first closes as many triangles as it takes off the
// stack, and the end fan one fewer than the stack holds, so a side of n + 1 top vertices gives n + 1 triangles whatever
// the heights are.
void addWall(Mesh& mesh, const Side& side) {
  const auto top = [&](std::int64_t k) { return static_cast<std::uint32_t>(side.first + k * side.step); };
  std::vector<std::uint32_t> stack;
  stack.reserve(static_cast<std::size_t>(side.count) + 1);
  stack.push_back(side.startCorner);
  stack.push_back(top(0));
  for (std::int64_t k = 1; k < side.count; ++k) {
    const std::uint32_t next = top(k);
    std::uint32_t waiting = stack.back();
    stack.pop_back();
    while (!stack.empty() && areaFacingOut(mesh, side, stack.back(), next, waiting) > 0) {
      mesh.triangles.push_back({stack.back(), next, waiting});
      waiting = stack.back();
      stack.pop_back();
    }
    stack.push_back(waiting);
    stack.push_back(next);
  }
  for (std::size_t i = 0; i + 1 < stack.size(); ++i) {
    mesh.triangles.push_back({side.endCorner, stack[i + 1], stack[i]});
  }
}

// The edges of a mesh that are not shared by exactly two triangles running along them in opposite directions, each
// counted once (checkSolid's header has the cases).
std::uint64_t countBadEdges(const std::vector<Triangle>& triangles) {
  const auto key = [](std::uint32_t from, std::uint32_t to) { return std::uint64_t{from} << 32 | to; };
  std::vector<std::uint64_t> edges;
  edges.reserve(3 * triangles.size());
  for (const Triangle& t : triangles) {
    edges.push_back(key(t.a, t.b));
    edges.push_back(key(t.b, t.c));
    edges.push_back(key(t.c, t.a));
  }
  std::sort(edges.begin(), edges.end());

  std::uint64_t bad = 0;
  for (std::size_t i = 0; i < edges.size();) {
    const std::uint64_t edge = edges[i];
    std::size_t end = i + 1;
    while (end < edges.size() && edges[end] == edge) ++end;
    const std::size_t times = end - i;
    const auto from = static_cast<std::uint32_t>(edge >> 32);
    const auto to = static_cast<std::uint32_t>(edge);
    if (from == to) {
      ++bad;  // a triangle with a vertex twice; no solid has such an edge
    } else {
      const auto back = std::equal_range(edges.begin(), edges.end(), key(to, from));
      const auto backTimes = static_cast<std::size_t>(back.second - back.first);
      // An edge is judged from its key that has the smaller vertex first. When it has no key that way round at all,
      // it is judged from the key it has, which is then bad.
      if (from < to) {
        if (times != 1 || backTimes != 1) ++bad;
      } else if (backTimes == 0) {
        ++bad;
      }
    }
    i = end;
  }
  return bad;
}

}  // namespace

Mesh surfaceMesh(const std::vector<float>& heights, int cols, int rows, double widthM, double heightM, double zScale,
                 double scale, double levelM) {
  const std::uint64_t count = checkGrid(cols, rows);
  if (heights.size() != count) {
    throw std::invalid_argument("a grid of " + gridSize(cols, rows) + " vertices needs " + std::to_string(count) +
                                " heights, not " + std::to_string(heights.size()));
  }
  const auto positive = [](double value) { return value > 0 && std::isfinite(value); };
  if (!positive(widthM) || !positive(heightM)) {
    throw std::invalid_argument("the window must be wider and taller than 0 m and finite, not " +
                                formatNumber(widthM, 15) + " x " + formatNumber(heightM, 15) + " m");
  }
  if (!positive(zScale) || !positive(scale)) {
    throw std::invalid_argument("the z-scale and the scale must be above 0 and finite, not " +
                                formatNumber(zScale, 15) + " and " + formatNumber(scale, 15));
  }
  if (!std::isfinite(levelM)) throw std::invalid_argument("the level must be finite, not " + formatNumber(levelM, 15));

  // Each column's x and each row's y are worked out once, so the vertices along a side share them exactly: each wall
  // is then exactly flat, and the bottom corners copy the top corners' x and y.
  std::vector<float> xs(static_cast<std::size_t>(cols));
  std::vector<float> ys(static_cast<std::size_t>(rows));
  for (int i = 0; i < cols; ++i) xs[static_cast<std::size_t>(i)] = static_cast<float>(i * widthM / (cols - 1) * scale);
  for (int j = 0; j < rows; ++j) {
    ys[static_cast<std::size_t>(j)] = static_cast<float>((rows - 1 - j) * heightM / (rows - 1) * scale);
  }

  Mesh mesh;
  mesh.vertices.reserve(count);
  for (std::size_t j = 0, v = 0; j < ys.size(); ++j) {
    for (std::size_t i = 0; i < xs.size(); ++i, ++v) {
      // std::max returns its first argument unless that is less than the second, so a NaN height stays NaN rather than
      // turning into the level.
      const double z = std::max(static_cast<double>(heights[v]), levelM) * zScale * scale;
      mesh.vertices.push_back({xs[i], ys[j], static_cast<float>(z)});
    }
  }

  const auto width = static_cast<std::uint32_t>(cols);
  mesh.triangles.reserve(2 * static_cast<std::size_t>(cols - 1) * static_cast<std::size_t>(rows - 1));
  for (std::uint32_t j = 0; j + 1 < static_cast<std::uint32_t>(rows); ++j) {
    for (std::uint32_t i = 0; i + 1 < width; ++i) {
      const std::uint32_t northWest = j * width + i;
      const std::uint32_t northEast = northWest + 1;
      const std::uint32_t southWest = northWest + width;
      const std::uint32_t southEast = southWest + 1;
      mesh.triangles.push_back({northWest, southWest, southEast});
      mesh.triangles.push_back({northWest, southEast, northEast});
    }
  }
  return mesh;
}

Mesh solidMesh(const Mesh& surface, int cols, int rows, float planeZ) {
  const std::uint64_t count = checkGrid(cols, rows);
  if (surface.vertices.size() != count) {
    throw std::invalid_argument("a grid of " + gridSize(cols, rows) + " vertices needs a surface of " +
                                std::to_string(count) + " vertices, not " + std::to_string(surface.vertices.size()));
  }
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

  const auto c = static_cast<std::int64_t>(cols);
  const auto r = static_cast<std::int64_t>(rows);
  const auto n = static_cast<std::uint32_t>(count);

  Mesh solid;
  solid.vertices.reserve(surface.vertices.size() + 4);
  solid.vertices.insert(solid.vertices.end(), surface.vertices.begin(), surface.vertices.end());
  // Vertices n to n + 3, under the south-west, south-east, north-east and north-west corners.
  for (const std::int64_t corner : {(r - 1) * c, (r - 1) * c + c - 1, c - 1, std::int64_t{0}}) {
    const Vec3& above = surface.vertices[static_cast<std::size_t>(corner)];
    solid.vertices.push_back({above.x, above.y, planeZ});
  }

  solid.triangles.reserve(surface.triangles.size() + static_cast<std::size_t>(2 * (c + r) + 2));
  solid.triangles.insert(solid.triangles.end(), surface.triangles.begin(), surface.triangles.end());
  addWall(solid, Side{.first = (r - 1) * c, .step = 1, .count = c, .startCorner = n, .endCorner = n + 1, .axis = 1,
                      .sign = -1.0});  // south, west to east, facing south
  addWall(solid, Side{.first = (r - 1) * c + c - 1, .step = -c, .count = r, .startCorner = n + 1, .endCorner = n + 2,
                      .axis = 0, .sign = 1.0});  // east, south to north, facing east
  addWall(solid, Side{.first = c - 1, .step = -1, .count = c, .startCorner = n + 2, .endCorner = n + 3, .axis = 1,
                      .sign = 1.0});  // north, east to west, facing north
  addWall(solid, Side{.first = 0, .step = c, .count = r, .startCorner = n + 3, .endCorner = n, .axis = 0,
                      .sign = -1.0});  // west, north to south, facing west
  // The base, clockwise seen from above so that it faces down.
  solid.triangles.push_back({n, n + 3, n + 2});
  solid.triangles.push_back({n, n + 2, n + 1});
  return solid;
}

std::uint64_t solidTriangleCount(int cols, int rows) noexcept {
  if (cols < 2 || rows < 2) return 0;
  const auto c = static_cast<std::uint64_t>(cols);
  const auto r = static_cast<std::uint64_t>(rows);
  return 2 * (c - 1) * (r - 1) + 2 * (c + r) + 2;
}

bool MeshCheck::ok() const {
  return (!edgesChecked || nonManifoldEdges == 0) && zeroAreaTriangles == 0 && signedVolume > 0;
}

MeshCheck checkSolid(const Mesh& mesh) {
  MeshCheck check;
  check.triangles = mesh.triangles.size();
  const std::size_t vertexCount = mesh.vertices.size();
  double sixVolumes = 0;
  for (std::size_t k = 0; k < mesh.triangles.size(); ++k) {
    const Triangle& t = mesh.triangles[k];
    for (const std::uint32_t v : {t.a, t.b, t.c}) {
      if (v >= vertexCount) {
        throw std::invalid_argument("triangle " + std::to_string(k) + " names vertex " + std::to_string(v) +
                                    ", but the mesh has " + std::to_string(vertexCount) + " vertices");
      }
    }
    const Vec3d a = toDouble(mesh.vertices[t.a]);
    const Vec3d normal = normalOf(mesh.vertices[t.a], mesh.vertices[t.b], mesh.vertices[t.c]);
    // Asked this way round so that a normal that is not a number counts as no area.
    if (!(normal.x * normal.x + normal.y * normal.y + normal.z * normal.z > 0)) ++check.zeroAreaTriangles;
    // a . (b x c) = a . ((b - a) x (c - a)), six times the signed volume of the tetrahedron from the origin.
    sixVolumes += a.x * normal.x + a.y * normal.y + a.z * normal.z;
  }
  check.signedVolume = sixVolumes / 6;

  if (check.triangles > kMostTrianglesForEdgeCheck) {
    check.edgesChecked = false;
  } else {
    check.nonManifoldEdges = countBadEdges(mesh.triangles);
  }
  return check;
}

}  // namespace demdl::mesh
