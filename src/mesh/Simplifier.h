#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "mesh/Footprint.h"
#include "mesh/Predicates.h"
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
// numbers. It is a BlockSimplifier, below, over this lattice of window blocks.
//
// A grid of more samples than 32-bit vertex numbers can count with the solid's four bottom corners is refused with
// std::invalid_argument before heights is looked at, as surfaceMesh refuses it. So is an infinite height, as a NaN is:
// no tolerance can bound it.
[[nodiscard]] SimplifiedSurface simplifyGrid(const std::vector<float>& heights, int cols, int rows,
                                             double toleranceM, double widthM, double heightM, double zScale,
                                             double scale, double levelM, int chunkCells = kChunkCells);

// A piece's outline as the surface's vertex numbers (the samples the runs kept) with their grid coordinates, for
// the walls and the base.
struct PieceOutline {
  std::vector<std::uint32_t> vertices;  // counter-clockwise seen from above, closed, no repeat
  // The same vertices' (column, row) relative to the block's first column and row, within the predicates' bound.
  std::vector<GridPoint> points;
};

struct BlockResult {
  std::size_t firstTriangle = 0, triangles = 0;  // this block's triangles in surface()
  std::optional<Boundary> rectangle;             // set for a window block: its four sides, for solidFromSurface
  std::vector<PieceOutline> pieces;              // set for a footprint block, one per kept piece
};

// Simplifies a grid block by block into one surface in model units, Part A's engine per block (spec 4.4, 4.5),
// so that blocks share the vertices their common sides keep. Construct it once per grid; call simplifyBlock for
// each block in lattice order (row by row from the north-west); read surface() when done. A window block
// (footprint nullptr) is simplified as a chunk of Part A. A footprint block is simplified piece by piece: each
// piece's runs are simplified by the side cache in the fixed direction, the kept samples make the outline,
// triangulatePolygon seeds the piece, legalisation and greedy insertion run over the samples strictly inside the
// piece (never a sample on the outline), and the piece's outline edges are hull edges that no flip crosses.
// Spec 4.3's guarantees hold inside every piece. Throws as simplifyGrid does, and std::invalid_argument when a
// block exceeds kMostChunkCells a side or lies outside the grid.
//
// The grid and the frame are simplifyGrid's, checked when the simplifier is made as simplifyGrid checks them, in its
// order and with its messages, and heights must outlive the simplifier. simplifyGrid is one of these over its lattice
// of window blocks. A block is rect's columns and rows of vertices, its sides included, as Tiles.h lays the blocks out:
// at least one cell each way and at most kMostChunkCells, and blocks share the vertices of the sides between them. Each
// block is given once; the order decides only the vertices' numbers and where each block's triangles come.
//
// A block's result: its triangles, from firstTriangle in surface(), counter-clockwise seen from above. For a window
// block, rectangle is its four sides as simplifyGrid's Boundary has the window's, each from a corner to the next
// beginning at the south-west one. For a footprint block, pieces holds an outline for each of footprint->pieces, in
// their order, footprint->dropped being left out: the samples its runs keep, in the order of the piece's loop from its
// first sample. The block's triangles cover its pieces once, and the only edges of them with no triangle of the block
// across are the outlines' edges, which is what the walls stand on. A footprint block with no piece has no triangle.
//
// Vertices come block by block: in a window block as in a chunk of simplifyGrid's, its outline's first; in a footprint
// block piece by piece, each piece's outline vertices before its inside's. A vertex on a block's side, which the block
// beside it may have too, or on a piece's outline, which a piece that touches it at a vertex may have too, goes into
// the surface once, with the first block or piece to reach it: so blocks that share a side meet on the very same
// vertices along it, and two pieces that touch at a vertex share that one (which footprintCells' pieces never do, as it
// joins them).
//
// The runs (spec 5.3): a run is the straight stretch of a piece's loop from one run start to the next. It is simplified
// by simplifyProfile from west to east along a row and from north to south along a column, whichever way the loop walks
// it, through the cache the blocks' sides go through, which knows it by its line and its ends; its ends are always
// kept. So a run that two blocks' pieces both have keeps the same samples in both, and the same as in any other cut of
// the grid into blocks that has the run. No other sample of a piece's outline is ever a vertex.
//
// Inside each piece, the guarantees of spec 4.3 as simplifyGrid has them inside its rectangle: every vertex is a sample
// at its exact height; at every sample inside the piece or on its outline the surface is within toleranceM vertically;
// the triangles cover the piece once, with no overlap, no gap and no vertex unused; and the same grid, tolerance,
// blocks and footprints give the same surface on every run. A piece that is its whole block, with the block's four
// sides as its only runs (as when the cells across each side are all in or all out, or the side is the grid's edge),
// is simplified as a window block is, from the same sides, to the same triangles, so that a footprint without no-data
// gives the surface of the window; its outline is the rectangle's, from the north-west corner down the west side.
//
// The pieces are blockFootprint's for rect, and each is checked before anything is added: its loop must hold four
// samples or more, all in the block and none twice, each one cell from the next along a grid line, and turn
// counter-clockwise seen from above; its run starts must begin at 0 and rise within the loop, and every run be
// straight. Otherwise std::invalid_argument names the piece by its loop's first sample. A refused block leaves the
// surface as it was. simplifyBlock after takeSurface throws std::logic_error.
//
// Memory, beyond heights and the surface: for the block at hand, a bit for each of its samples when it has a
// footprint, marking its outlines' samples, and the faces of the window block or of the piece being triangulated, as
// for a chunk of simplifyGrid's; from block to block, the samples the sides and runs keep, with their vertex numbers.
// Time: a window block's is a chunk's; a piece's is a chunk's of the same area, plus triangulatePolygon on its kept
// outline, up to quadratic in its vertices, and Lawson's flips over that seed.
class BlockSimplifier {
 public:
  BlockSimplifier(const std::vector<float>& heights, int cols, int rows, double toleranceM, double widthM,
                  double heightM, double zScale, double scale, double levelM);
  ~BlockSimplifier();
  BlockSimplifier(const BlockSimplifier&) = delete;
  BlockSimplifier& operator=(const BlockSimplifier&) = delete;

  [[nodiscard]] BlockResult simplifyBlock(BlockRect rect, const BlockFootprint* footprint);
  [[nodiscard]] const Mesh& surface() const noexcept;
  [[nodiscard]] Mesh takeSurface();  // moves the surface out

 private:
  struct State;
  std::unique_ptr<State> state_;
};

}  // namespace demdl::mesh
