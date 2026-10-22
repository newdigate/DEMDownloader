#pragma once

#include <cstdint>
#include <span>
#include <vector>

#include "mesh/Predicates.h"

// dataMaskOf tells a height from no-data with std::isfinite, which fast-math lets the compiler fold to true (as in
// GeoTiffGrid.h), and then the sea would count as land.
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "NaN marks no data; do not build with -ffast-math or -ffinite-math-only"
#endif

namespace demdl::mesh {

// The footprint of spec 5.2: where the solid of --footprint data stands. The window's no-data, as readWindow gives it
// before the fill, is the mask (dataMaskOf); it is carried to the output grid by the nearest sample
// (resampleMaskNearest); the cells of that grid that have data at all four corners, closed, with their enclosed holes
// filled and their corner contacts joined, are the footprint (footprintCells); and each block takes its pieces of it,
// outlined (blockFootprint). The cells are worked out once for the whole grid, not per block (deviation 3), so that
// two blocks see the same cells along the side they share. The unit takes a mask, not the DEM's no-data as such, so
// that a second mask could be added to it (spec 5.2, "leaving room").
//
// Coordinates are the output grid's: a vertex is (column, row), counted from 0 at the north-west corner, columns
// running east and rows south. The cell (c, r) is the square whose corners are the vertices (c, r), (c + 1, r),
// (c, r + 1) and (c + 1, r + 1); a grid of cols x rows vertices has (cols - 1) x (rows - 1) cells, held row by row
// from the north-west as the samples are. Everything here is whole numbers and scan order, so the same mask gives the
// same footprint, pieces and outlines on every run and platform.

// Which samples of a window have data: 1 where the height is finite, 0 where it is NaN (readWindow's no-data).
[[nodiscard]] std::vector<std::uint8_t> dataMaskOf(const std::vector<float>& window);

// The mask on the output grid: output vertex (i, j) takes the status of the nearest source sample, at the position
// u_i = i (srcCols - 1) / (outCols - 1) rounded to the nearest sample (half rounds down), likewise down the rows.
// Those are the positions resampleQuadratic gives its outputs, so a vertex has data when the sample nearest to where
// its height was taken has. The positions are worked out in whole numbers, so nothing is rounded but the halves, and
// a grid of the source's own size is the source again. The values are copied as they are. Throws
// std::invalid_argument when srcCols or srcRows is below 2, mask does not hold srcCols x srcRows values, or outCols or
// outRows is below 2, as resampleQuadratic does.
[[nodiscard]] std::vector<std::uint8_t> resampleMaskNearest(const std::vector<std::uint8_t>& mask, int srcCols,
                                                            int srcRows, int outCols, int outRows);

// The footprint's cells on a grid of cols x rows vertices: (cols - 1) x (rows - 1) flags, row by row from the
// north-west. A cell is in when all four of its vertices have data; the result is then closed with a 3 x 3
// neighbourhood (dilated, then eroded); then every 4-connected region of cells still out that touches the grid's
// edge stays out, and every enclosed one is turned in; then every corner contact is joined (below). Throws
// std::invalid_argument when cols or rows is below 2 or mask does not hold cols x rows values.
//
// The second form also appends to *joined, when joined is not null, the vertex (column, row) of each corner contact it
// joined, in the order they were joined (see the join below); the first gives the same cells.
//
// A vertex has data when its value in mask is not 0; the flags are 1 for a cell in and 0 for one out.
//
// The closing: the dilation turns in every cell that has a cell in within the 3 x 3 square around it, and the erosion
// then keeps in only the cells whose whole square is in. Beyond the grid counts as out for the dilation and as in for
// the erosion, so the closing only adds cells, and a footprint that reaches the grid's edge keeps its edge cells. What
// it adds are the gaps between cells in that are one or two cells across. A no-data sample is a corner of the cells on
// both sides of it, so a channel of no-data one sample wide leaves out a strip two cells wide, which is filled along
// its length, and a channel two samples wide leaves out three cells, which stay out. The closing does not join two
// pieces that touch only at a vertex (the join does), and the steps of a coast stay as they are. With beyond the grid
// counting as in for the erosion, a strip of out cells one cell deep along the grid's edge is filled too, as no-data
// samples on the edge itself make: a slit between the data and the world beyond the window.
//
// The enclosed test: a breadth-first search through the cells still out, a side at a time, from those on the grid's
// edge; whatever it does not reach is turned in. A lake joined to the sea by a channel the closing filled is filled
// too, and so, with a crop, is a bay whose mouth lies beyond the window (spec 5.2). Out cells that meet the sea only at
// a vertex are enclosed and filled.
//
// The join: a vertex with four cells round it is a corner contact when they are a diagonal pair in and the other pair
// out. Two pieces that met there would each stand a wall on the upright line under the vertex, and once an STL reader
// had merged their corners that line would be one edge of four faces, which every checker calls non-manifold. So the
// contact is joined: the north-west-most of its two empty cells, the one in the smaller row (and of equal rows the one
// in the smaller column, which never decides here), is turned in, a cell of made-up ground like a filled hole, whose
// corners take their heights from the fill. Every contact is joined at once, then the corners of the cells put in are
// looked at again, and so on until none is a contact; cells are only ever added, so this ends. After the closing and
// the fill one round is all it takes: a cell is out after the closing only inside a 3 x 3 square of cells all out (or
// the part of one on the grid), so the empty cell north of a contact is a bottom corner of such a square, the cells
// beside it in that square stay out, and putting it in makes no new contact and cuts no region of the sea off the
// grid's edge. Nor do two contacts put in the same cell, so the joins are independent and their order changes nothing.
// The vertices are listed as they were joined, which with the one round is row by row from the north-west. A chain of
// pieces touching corner to corner becomes one piece, with a cell at each link.
//
// So the footprint has no hole; no piece of it meets itself at a vertex, since a piece that did would enclose one of
// the two cells off that vertex's diagonal, which the fill fills; and no two of its cells meet only at a vertex, so no
// two pieces touch, in a block or across a block's side.
//
// Memory beyond mask and the result: two rows of cells for the closing, two lists of cell numbers (8 bytes each) as
// long as the longest wave of the search, about as long as the coast, and the contacts found. The result is a byte a
// cell, 254 MB for the City's whole file at full resolution. Time is a few passes over the cells, the join's a look at
// every vertex.
[[nodiscard]] std::vector<std::uint8_t> footprintCells(const std::vector<std::uint8_t>& mask, int cols, int rows);
[[nodiscard]] std::vector<std::uint8_t> footprintCells(const std::vector<std::uint8_t>& mask, int cols, int rows,
                                                       std::vector<GridPoint>* joined);

struct BlockRect { int firstColumn, firstRow, lastColumn, lastRow; };  // vertex indices, sides included

// One piece of a block's footprint: its outline as every sample on it in order, counter-clockwise seen from above
// (x east, y north: with rows numbered southward that is orient2d negative turns in (column, row)), closed (the
// first sample is not repeated), and the indices into `loop` where its straight runs start: a run ends where the
// outline turns, and on a block side also where the status of the cell across the side changes, so that the
// neighbouring block splits the same runs. cells is the piece's area in cells.
//
// The loop starts at the north-west corner of the piece's first cell, row by row from the north-west (its northmost
// row, and in that row its westmost cell), and goes south from there. That corner is always a turn, so runStarts[0] is
// 0. Run i runs from loop[runStarts[i]] to loop[runStarts[i + 1]], both included, and the last one back to loop[0]; a
// run lies along one row or one column of the grid, and consecutive samples are one cell apart.
struct Piece {
  std::vector<GridPoint> loop;        // grid coordinates (column, row) of the window
  std::vector<std::uint32_t> runStarts;
  std::uint64_t cells = 0;
};

struct BlockFootprint {
  std::vector<Piece> pieces;          // kept, largest first, ties by the north-west-most cell
  std::vector<Piece> dropped;         // under minPieceUnits2
  std::uint64_t cells = 0;            // footprint cells in the block, kept pieces only
  std::uint64_t cornerContacts = 0;   // contacts footprintCells joined at vertices strictly inside the block
};

// The pieces of the footprint inside rect: 4-connected components of the cells in rect's cell range, each outlined
// and split into runs as Piece says. A component whose cells x cellAreaUnits2 is below minPieceUnits2 goes to
// dropped. `cells` is footprintCells' result for the whole grid, which is also consulted across rect's sides to
// split the runs. Throws std::invalid_argument when rect is not inside the grid or has no cell.
//
// rect's cell range is the cells of columns firstColumn to lastColumn - 1 and rows firstRow to lastRow - 1. A cell
// beyond it is out as far as the block's pieces go, whatever `cells` says, so a piece of the footprint that crosses a
// block side is a piece of each block, cut along the side.
//
// The outline: the sides of the piece's cells that have no cell of the piece beyond them, walked with the piece on the
// left seen from above. At each vertex the walk turns left when the cell ahead on its left is out, goes straight on
// when that one is in and the one ahead on its right is out, and turns right when both are in. So where two pieces
// touch at a vertex, which they never do in footprintCells' result but may in cells from elsewhere, the walk turns
// back along its own piece's side and never crosses to the other piece, and each piece's loop holds that vertex once.
//
// The runs: on a block side that is not the grid's edge, a run also ends at every sample where the cells across the
// side, the cells beyond rect before and after the sample, differ. Every run along such a side is then a stretch over
// which the cells on both sides of the line keep their status, and the block beyond the side, which outlines its own
// cells along the same line by the same rule, has the same runs wherever both blocks' pieces have the line as outline:
// the same samples at their ends, walked the other way. On the grid's edge there is nothing across, and only the
// turns end runs.
//
// Corner contacts: `joined` is the list of the vertices of the contacts footprintCells joined over the whole grid, and
// cornerContacts counts those strictly inside rect (the four cells round each in rect's cell range), whether the piece
// that took the joining cell is kept or dropped. A joined contact on a block side has one cell of its diagonal in each
// block, so neither counts it. With joined empty nothing is counted, and two pieces of cells from elsewhere that touch
// at a vertex are outlined each on its own and not counted. Dropped pieces are outlined as kept ones are, so that they
// can be listed, and come in the same order.
//
// Memory: a byte for each of rect's cells, two lists of cell numbers as long as the longest wave of the breadth-first
// search that finds each piece, and the outlines. Time: a pass over rect's cells and a walk round each piece.
//
// Also throws std::invalid_argument when cols or rows is below 2, cells does not hold (cols - 1) x (rows - 1) flags,
// cellAreaUnits2 is not above 0 and finite, or minPieceUnits2 is negative or not finite; and when a piece has a hole
// or meets itself at a vertex, which no footprintCells result has, because one loop cannot outline it.
[[nodiscard]] BlockFootprint blockFootprint(const std::vector<std::uint8_t>& cells, int cols, int rows,
                                            BlockRect rect, double cellAreaUnits2, double minPieceUnits2,
                                            std::span<const GridPoint> joined = {});

}  // namespace demdl::mesh
