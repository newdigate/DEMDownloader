#pragma once

#include <string>

#include "mesh/Footprint.h"

namespace demdl::mesh {

// The block lattice of spec 5.3: --tile splits the output grid into square blocks, each written as its own STL, and
// --blocks says which of them to write. This unit is the arithmetic of that and nothing else. It is whole numbers
// apart from one division and its rounding, so the same arguments give the same lattice on every platform.
//
// Coordinates are the output grid's, as in Footprint.h: a vertex is (column, row), counted from 0 at the north-west
// corner, columns running east and rows south, and a grid of cols x rows vertices has (cols - 1) x (rows - 1) cells.
// Blocks are counted in cells, since those are what the simplifier sees, and laid out from the north-west corner. A
// block n cells across has n + 1 vertices across, and blocks share their boundary vertices: a block's last column is
// the next block's first column, and likewise down the rows. The blocks on the east and south edges are cut short
// where the cells run out. The lattice is numbered from 0, row r down and column c across, as the vertices are; the
// names of the blocks and the text of --blocks count from 1, as the summary prints them.

// The block lattice over an output grid of cols x rows vertices: blocks of blockCols x blockRows cells from the
// north-west corner, `across` x `down` of them, the last ones each way cut short.
//
//   blockCols = round(tileUnits / (scale x cellXM)) clamped to [1, cols - 1], likewise blockRows with cellYM and
//   rows - 1; across = ceil((cols - 1) / blockCols), likewise down.
//
// tileUnits is --tile, a block's side in model units, and scale is --scale, the model units to a metre. cellXM and
// cellYM are the output grid's cell sizes in metres, the window's extent over cols - 1 and rows - 1. They are equal
// for a projected file and differ for a geographic one, which is why the two sides of a block are counted apart: each
// block is as close to square as the grid allows. A half cell rounds up. A tile of under half a cell is blocks of one
// cell, and a tile larger than the grid is one block of the whole grid.
//
// Throws std::invalid_argument when cols or rows is below 2, when tileUnits, scale, cellXM or cellYM is not above 0
// and finite, and when a block side would exceed kMostChunkCells (mesh/Simplifier.h), the largest side the exact
// predicates are good for (spec 4.5). The message of the last names the side, wide or tall, and suggests a smaller
// --tile or a coarser --grid. A tile larger than a grid that is itself wider or taller than the limit is refused too,
// since its one block is the whole grid. The first fault found is the one named, in the order above and the width
// before the height.
struct TileLayout { int cols, rows, blockCols, blockRows, across, down; };
[[nodiscard]] TileLayout tileLayout(int cols, int rows, double tileUnits, double scale, double cellXM, double cellYM);

// Block (r, c), 0-based, as vertex indices, sides included: columns c x blockCols to (c + 1) x blockCols, and rows
// likewise, the east and south edges cut short at the grid's last column and row. Neighbours share the vertices of
// the side between them. Throws std::invalid_argument when the block is outside the lattice, r not below `down` or
// c not below `across`, or either below 0. `layout` is one that tileLayout returned.
[[nodiscard]] BlockRect blockRect(const TileLayout& layout, int r, int c);

// Whether block (r, c) has fewer cells across than blockCols or fewer down than blockRows, which only a block on the
// east or south edge can. False for a block outside the lattice, which has nothing to be cut short.
[[nodiscard]] bool cutShort(const TileLayout& layout, int r, int c) noexcept;

// "r03_c05": the block's name in file names, row and column counted from 1, each zero-padded to the number of decimal
// digits of `down` and of `across` (at least 1), so that the names sort in lattice order. 7 down and 9 across need
// one digit each: "r3_c5". Throws std::invalid_argument for a block outside the lattice.
[[nodiscard]] std::string blockName(const TileLayout& layout, int r, int c);

// --blocks ROWS:COLS: the blocks to write, each part a number N or a range A-B, 1-based and inclusive, as the names
// count. `all` holds when no --blocks was given, and then every block is selected.
struct BlockSelection {
  bool all = true;
  int firstRow = 1, lastRow = 1, firstCol = 1, lastCol = 1;
  // Whether block (r, c) is selected. r and c are the lattice's, from 0; the ranges are compared as the 1-based
  // numbers they are. A range that reaches past the lattice picks the blocks that exist, and says nothing of the
  // rest: it is for the caller to report a selection that reaches too far.
  [[nodiscard]] bool selected(int r, int c) const noexcept;
};

// Reads the text of --blocks: ROWS:COLS, each part a number N or a range A-B of them. A number is one or more digits
// with a value from 1 to the largest int; leading zeros are allowed, since the names carry them (03:05 is 3:5), and
// nothing else is: no sign, space or separator. `all` is false in the result. Empty text is not ROWS:COLS (a run
// without --blocks has a default BlockSelection, and does not call this).
//
// Throws std::invalid_argument for text that is not that, or whose range ends below its start. The messages are
// usage errors that complete "--blocks: ", the way the command-line parser's do for the flags it checks, and the
// caller puts the flag in front. They quote the text that was typed, with a carriage return and any other stray
// character shown as an escape (\r, \xE2), so that it can be seen. Of several faults the first found is the one named,
// in this order: the form, then a number below 1 or above the largest int, then a range that ends below its start; the
// rows before the columns within each.
[[nodiscard]] BlockSelection parseBlocks(const std::string& text);

}  // namespace demdl::mesh
