#pragma once

#include <cstdint>
#include <vector>

#include "mesh/SolidMesh.h"

namespace demdl::mesh {

inline constexpr int kChunkCells = 2048;       // the lattice without tiles (spec 4.5)
inline constexpr int kMostChunkCells = 16384;  // the largest side the predicates are exact for

struct SimplifiedSurface {
  Mesh surface;        // in model units, in the frame of surfaceMesh
  Boundary boundary;   // the window's outline, for solidFromSurface
  std::uint64_t gridVertices = 0;   // cols x rows
  std::uint64_t gridTriangles = 0;  // 2 (cols - 1)(rows - 1)
};

// Simplifies a grid of cols x rows heights in metres, row by row from the north-west corner as resampleQuadratic
// gives them, to a TIN within toleranceM of every sample (spec 4.3), then places it in model units exactly as
// surfaceMesh would: x = i widthM / (cols - 1) scale, y = (rows - 1 - j) heightM / (rows - 1) scale,
// z = max(h, levelM) zScale scale. Chunks of chunkCells x chunkCells cells from the north-west corner (spec 4.5).
// Vertices come chunk by chunk, each chunk's boundary vertices before its interior ones in insertion order, and a
// vertex shared by chunks appears once. Triangles are counter-clockwise seen from above.
// Throws std::invalid_argument when cols or rows is below 2, heights does not hold cols x rows values or holds a
// NaN, toleranceM is negative or not finite, chunkCells is below 1 or above kMostChunkCells, or widthM, heightM,
// zScale, scale or levelM fails surfaceMesh's checks.
//
// Within toleranceM means the guarantees of spec 4.3: every vertex is a sample, at its exact height; the four corners
// are vertices, and the rectangle's sides hold boundary vertices only, the ones Boundary lists; at every sample the
// surface is at most toleranceM above or below it, in the heights' metres, whatever zScale and scale do to them
// afterwards; the triangles make one sheet, with no overlap, no gap and no vertex left unused; and the same grid,
// tolerance and chunkCells give the same surface on every run and platform. A plane of any slope comes out as two
// triangles a chunk, so as its four corners and two triangles when the grid is one chunk. The guarantees hold on the
// heights before the lift to levelM: a levelM above a sample's height lifts that vertex to it and can break them, which
// dem2mesh never does, its base plane never lifting.
//
// How (spec 4.4): each side of a chunk is simplified first, on its own, by simplifyProfile, a row from west to east and
// a column from north to south whichever way the outline walks it; the samples it keeps are the chunk's boundary
// vertices, and no other sample on a side is ever inserted, so two chunks that share a side share its vertices. Then
// the interior by greedy insertion: the sample furthest from the surface, vertically, goes in next, of equally far ones
// the one in the smallest row and then the smallest column, until none is further than toleranceM. The triangulation is
// kept Delaunay with the exact predicates of Predicates.h on the chunk's own columns and rows. Its memory, beyond
// heights and the result, is the chunk's alone: about 50 bytes a triangle, 24 a vertex and up to a few heap entries of
// 24 bytes a triangle. The sides' samples go in one after another along each side, and where a side and the one
// opposite it both keep nearly all their samples (noise at a tolerance near 0), each new sample of the later one takes
// over its predecessor's fan of edges across to the other: about a quarter of the product of their counts in flips,
// some 30 ms for a chunk 2048 cells long and 2 s for one of 16384 in a Release build, and no memory beyond the faces.
//
// The lattice (spec 4.5): the chunks are taken row by row from the north and along each row from the west, the last
// ones each way cut short. A side two chunks share is simplified once, for both, and the vertices on it, like those on
// a corner that up to four chunks share, go into the output with the first chunk to reach them. The window's Boundary
// is the chunks' sides along its edges, joined at the corners they share. The seams keep samples that one
// triangulation of the whole grid might not, so a plane comes out as two triangles a chunk; on terrain the totals come
// out close either way. Beyond the chunk at hand, a run holds only the samples the chunks' sides keep and their vertex
// numbers.
//
// A grid of more samples than 32-bit vertex numbers can count with the solid's four bottom corners is refused with
// std::invalid_argument before heights is looked at, as surfaceMesh refuses it. So is an infinite height, as a NaN is:
// no tolerance can bound it.
[[nodiscard]] SimplifiedSurface simplifyGrid(const std::vector<float>& heights, int cols, int rows,
                                             double toleranceM, double widthM, double heightM, double zScale,
                                             double scale, double levelM, int chunkCells = kChunkCells);

}  // namespace demdl::mesh
