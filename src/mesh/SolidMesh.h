#pragma once

#include <cstdint>
#include <vector>

namespace demdl::mesh {

// A triangle mesh in model units: x east, y north, z up, the window's south-west corner at the origin (spec 5.2). A
// triangle is three vertex numbers in counter-clockwise order seen from the side it faces.
struct Vec3 {
  float x, y, z;
};

struct Triangle {
  std::uint32_t a, b, c;
};

struct Mesh {
  std::vector<Vec3> vertices;
  std::vector<Triangle> triangles;
};

// The top surface of the terrain, from a grid of cols x rows heights in metres given row by row from the north-west
// corner, as resampleQuadratic gives them. Vertex (i, j), column i from the west and row j from the north, is vertex
// number j cols + i, at
//   x = i widthM / (cols - 1) scale,  y = (rows - 1 - j) heightM / (rows - 1) scale,  z = max(h, levelM) zScale scale.
// Row 0 is the north edge, at the largest y, so the first vertex is the north-west corner; read the other way round
// the model would be a mirror image, which nothing downstream would notice. The vertices span the window, from 0 to
// widthM scale across and from 0 to heightM scale up. A height below levelM is lifted to it (spec 5.3): with the level
// at or below the lowest height, where chooseBasePlane puts it, there is none, and the lift only guarantees that every
// vertex stands a margin above the base. A NaN height stays NaN, and solidMesh refuses it.
//
// Each cell is two triangles, cut along its north-west to south-east diagonal and wound counter-clockwise seen from
// above, so that every normal points up: 2 (cols - 1)(rows - 1) triangles, cell by cell in the order of the vertices.
//
// Throws std::invalid_argument when cols or rows is below 2; when the grid has more vertices than 32-bit vertex numbers
// can count together with the solid's four bottom corners (judged before heights is looked at); when heights does not
// hold cols x rows values; when widthM, heightM, zScale or scale is not above 0 and finite; when levelM is not finite.
[[nodiscard]] Mesh surfaceMesh(const std::vector<float>& heights, int cols, int rows, double widthM, double heightM,
                               double zScale, double scale, double levelM);

// The printable solid (spec 5.5): the surface, four upright walls and a flat base at z = planeZ, closed along every
// seam because the walls and the base are built on the surface's own edge vertices. It starts with the surface's
// vertices and triangles, unchanged and in the same order, so the surface is a prefix of the solid; then come four
// bottom corners, under the south-west, south-east, north-east and north-west corners in that order; then the walls,
// south, east, north and west; then the base.
//
// The base is two triangles over the four bottom corners, facing down. A wall is the flat polygon between one side of
// the surface and the base: the side's top vertices, in the order the boundary is walked counter-clockwise seen from
// above (the south side from west to east, the east side from south to north, the north side from east to west, the
// west side from north to south), and the bottom corners under the first and the last of them. Each wall is cut into
// triangles facing out of the solid without adding a vertex, so its bottom edge is a single edge shared with the base
// and its upright edges are shared with the next walls: no T-junction anywhere, which two triangles down to the plane
// per boundary edge would leave along the base. A side of n + 1 top vertices is cut into n + 1 triangles, so the solid
// has solidTriangleCount(cols, rows) of them.
//
// Where it works, the cut is the plan's fan: from the bottom corner under the side's first top vertex to each pair of
// neighbouring top vertices, then one triangle from the last top vertex to the next bottom corner. It works while each
// top vertex, looked at from that corner, appears lower than the one before it. Past a valley that a nearer rise hides
// from the corner the fan's triangles would face into the solid and lie over others, and where the corner and two top
// vertices fall in line one would have no area. Terrain does that all the time: on random-walk sides of 500 heights
// at --scale 0.02, over a third of the fan's triangles would face in. A folded fan is still closed and encloses the
// right volume, so checkSolid would pass it; its faces would be wrong all the same. So each wall is cut as a monotone
// polygon, with a stack: each top vertex in turn closes every triangle facing out that it can with the vertices
// waiting on the stack, and whatever still waits at the end is a fan from the next bottom corner, which sees all of
// it. Where every triangle of the plan's fan faces out, this cut is that fan exactly; elsewhere it bridges each hidden
// valley with triangles between top vertices.
//
// surface is what surfaceMesh made for the same cols and rows. Throws std::invalid_argument when cols or rows is below
// 2, when surface does not hold cols x rows vertices or holds more than 32-bit vertex numbers can count with the four
// bottom corners, when planeZ is not finite, or when a vertex of the surface is not above planeZ (a NaN one included),
// a wall having to reach down from every top vertex to the base.
[[nodiscard]] Mesh solidMesh(const Mesh& surface, int cols, int rows, float planeZ);

// 2 (cols - 1)(rows - 1) + 2 (cols + rows) + 2: two surface triangles per cell, one wall triangle per top vertex of
// each side (cols, rows, cols and rows of them), and the base's two. 12 for a 2 x 2 grid, a box. Counted in 64 bits; 0
// when cols or rows is below 2, which makes no mesh.
[[nodiscard]] std::uint64_t solidTriangleCount(int cols, int rows) noexcept;

// What checkSolid found (spec 6).
struct MeshCheck {
  std::uint64_t triangles = 0, nonManifoldEdges = 0, zeroAreaTriangles = 0;
  double signedVolume = 0;
  bool edgesChecked = true;

  // A closed solid facing out: no bad edge (when the edges were checked), no triangle without area, a volume above 0.
  [[nodiscard]] bool ok() const;
};

// The most triangles whose edges checkSolid checks: the check sorts three 64-bit keys per triangle, 240 MB at this
// size.
inline constexpr std::uint64_t kMostTrianglesForEdgeCheck = 10'000'000;

// Checks that a mesh is a closed solid facing out (spec 6), as the proof of solidMesh's work and of what an STL reader
// gives back:
// - nonManifoldEdges: each edge must be shared by exactly two triangles that run along it in opposite directions, which
//   is what makes a surface closed and wound the same way all over. Every directed edge goes into one sorted list as a
//   64-bit key, from << 32 | to, and an edge is good when its key is there once and its reverse once. Each edge that is
//   not good is counted once, whichever way round it was found: the edge of a hole, an edge of three triangles or more,
//   an edge along which two neighbours are wound the same way, an edge from a vertex to itself. This is done only when
//   the mesh has at most kMostTrianglesForEdgeCheck triangles; above that, edgesChecked is false and nonManifoldEdges
//   stays 0.
// - zeroAreaTriangles: the triangles whose normal (b - a) x (c - a), in double, is zero, which takes three points in
//   line, or is not a number.
// - signedVolume: the sum of a . ((b - a) x (c - a)) / 6 over the triangles, in double. By the divergence theorem that
//   is the volume a closed mesh encloses when it is wound counter-clockwise seen from outside, and minus that when it
//   is inside out. A NaN coordinate makes it NaN, which ok() refuses.
// That there are solidTriangleCount(cols, rows) triangles is for the caller to check, who knows the grid.
//
// Throws std::invalid_argument when a triangle names a vertex the mesh does not have.
[[nodiscard]] MeshCheck checkSolid(const Mesh& mesh);

}  // namespace demdl::mesh
