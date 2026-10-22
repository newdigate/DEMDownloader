#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include "mesh/Footprint.h"
#include "mesh/Simplifier.h"
#include "mesh/Tiles.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::Message;
using Catch::Matchers::MessageMatches;
using demdl::mesh::BlockRect;
using demdl::mesh::blockName;
using demdl::mesh::blockRect;
using demdl::mesh::BlockSelection;
using demdl::mesh::cutShort;
using demdl::mesh::kMostChunkCells;
using demdl::mesh::parseBlocks;
using demdl::mesh::TileLayout;
using demdl::mesh::tileLayout;

// Every expected number follows from the rules of spec 5.3, worked out beside its case. A block is
// round(tileUnits / (scale x cell)) cells a side, kept between 1 cell and the grid; the blocks are counted from the
// north-west corner, share their boundary vertices, and the ones on the east and south edges are cut short where the
// cells run out. Names and --blocks count from 1, the lattice from 0. The property case holds the unit to what a
// tiling is (tilingProblem), never to a reading of its own formulas.

namespace {

// A block's rectangle as text, so that a failed check shows all four numbers.
std::string show(const BlockRect& rect) {
  return "columns " + std::to_string(rect.firstColumn) + " to " + std::to_string(rect.lastColumn) + ", rows " +
         std::to_string(rect.firstRow) + " to " + std::to_string(rect.lastRow);
}

// A selection as text: "all", or its 1-based inclusive ranges.
std::string show(const BlockSelection& selection) {
  if (selection.all) return "all";
  return "rows " + std::to_string(selection.firstRow) + " to " + std::to_string(selection.lastRow) + ", columns " +
         std::to_string(selection.firstCol) + " to " + std::to_string(selection.lastCol);
}

// The City's whole file at 1:50,000 with --tile 100: 12912 x 19696 samples 5 m apart, so a block of 100 units is
// 100 / (0.02 x 5) = 1000 cells, 5000 m on the ground.
TileLayout city() { return tileLayout(12912, 19696, 100, 0.02, 5, 5); }

// Says what is wrong with the lattice as a tiling of its grid's (cols - 1) x (rows - 1) cells, or returns "" when
// nothing is. Counting the blocks by stepping along each side must give across and down. Each block is 1 to blockCols
// cells wide and 1 to blockRows tall; starts where the block to its west (or north) ended, and at 0 when there is
// none; has the rows of the block to its west and the columns of the one to its north; the last ones end on the grid's
// last vertex; and cutShort is true for exactly the blocks smaller than the lattice's.
std::string tilingProblem(const TileLayout& l) {
  int across = 0;
  for (int x = 0; x < l.cols - 1; x += l.blockCols) ++across;
  int down = 0;
  for (int y = 0; y < l.rows - 1; y += l.blockRows) ++down;
  if (l.across != across || l.down != down) {
    return "across x down is " + std::to_string(l.across) + " x " + std::to_string(l.down) + ", not " +
           std::to_string(across) + " x " + std::to_string(down);
  }
  for (int r = 0; r < l.down; ++r) {
    for (int c = 0; c < l.across; ++c) {
      const std::string where = "block (" + std::to_string(r) + ", " + std::to_string(c) + ") ";
      const BlockRect rect = blockRect(l, r, c);
      const int width = rect.lastColumn - rect.firstColumn;
      const int height = rect.lastRow - rect.firstRow;
      if (width < 1 || width > l.blockCols || height < 1 || height > l.blockRows) {
        return where + "is " + std::to_string(width) + " x " + std::to_string(height) + " cells";
      }
      if (c > 0) {
        const BlockRect west = blockRect(l, r, c - 1);
        if (rect.firstColumn != west.lastColumn) return where + "does not start where the block to its west ended";
        if (rect.firstRow != west.firstRow || rect.lastRow != west.lastRow) {
          return where + "has other rows than the block to its west";
        }
      } else if (rect.firstColumn != 0) {
        return where + "starts at column " + std::to_string(rect.firstColumn) + ", not 0";
      }
      if (r > 0) {
        const BlockRect north = blockRect(l, r - 1, c);
        if (rect.firstRow != north.lastRow) return where + "does not start where the block to its north ended";
        if (rect.firstColumn != north.firstColumn || rect.lastColumn != north.lastColumn) {
          return where + "has other columns than the block to its north";
        }
      } else if (rect.firstRow != 0) {
        return where + "starts at row " + std::to_string(rect.firstRow) + ", not 0";
      }
      if (c == l.across - 1 && rect.lastColumn != l.cols - 1) return where + "does not end on the last column";
      if (r == l.down - 1 && rect.lastRow != l.rows - 1) return where + "does not end on the last row";
      const bool smaller = width < l.blockCols || height < l.blockRows;
      if (cutShort(l, r, c) != smaller) {
        return where + "is " + std::to_string(width) + " x " + std::to_string(height) + " cells in a lattice of " +
               std::to_string(l.blockCols) + " x " + std::to_string(l.blockRows) + ", and cutShort says " +
               (smaller ? "false" : "true");
      }
    }
  }
  return {};
}

// The names of every block of the lattice, row by row from the north-west.
std::vector<std::string> allNames(const TileLayout& l) {
  std::vector<std::string> names;
  for (int r = 0; r < l.down; ++r) {
    for (int c = 0; c < l.across; ++c) names.push_back(blockName(l, r, c));
  }
  return names;
}

// The names of the blocks that a selection picks from the lattice, in lattice order.
std::vector<std::string> picked(const TileLayout& l, const BlockSelection& selection) {
  std::vector<std::string> names;
  for (int r = 0; r < l.down; ++r) {
    for (int c = 0; c < l.across; ++c) {
      if (selection.selected(r, c)) names.push_back(blockName(l, r, c));
    }
  }
  return names;
}

// What parseBlocks says of text that is not ROWS:COLS of numbers and ranges, up to the text itself, quoted.
const std::string kBadForm = "must be ROWS:COLS, each part a whole number from 1 or a range such as 3-5, not ";

// The message for a side of blocks over the limit; `side` is "wide" or "tall".
std::string tooLarge(int cells, const std::string& side) {
  return "a block would be " + std::to_string(cells) + " cells " + side +
         ", but a block side may be at most 16384 cells: use a smaller --tile or a coarser --grid";
}

}  // namespace

// --- tileLayout, blockRect, cutShort ---------------------------------------------------------------------------------

TEST_CASE("the City's whole file at 1:50,000 in blocks of 100 units is 13 across and 20 down", "[tiles]") {
  // 100 / (0.02 x 5) = 1000 cells a side. The 12911 cells across are 12 blocks of 1000 and one of 911, so 13; the
  // 19695 cells down are 19 blocks of 1000 and one of 695, so 20.
  const TileLayout l = city();
  CHECK(l.cols == 12912);
  CHECK(l.rows == 19696);
  CHECK(l.blockCols == 1000);
  CHECK(l.blockRows == 1000);
  CHECK(l.across == 13);
  CHECK(l.down == 20);
}

TEST_CASE("blocks run from the north-west corner and share their boundary vertices", "[tiles]") {
  const TileLayout l = city();
  CHECK(show(blockRect(l, 0, 0)) == "columns 0 to 1000, rows 0 to 1000");
  // A block of 1000 cells has 1001 vertices across, the last being the next block's first.
  CHECK(show(blockRect(l, 0, 1)) == "columns 1000 to 2000, rows 0 to 1000");
  CHECK(show(blockRect(l, 1, 0)) == "columns 0 to 1000, rows 1000 to 2000");
  CHECK(show(blockRect(l, 18, 11)) == "columns 11000 to 12000, rows 18000 to 19000");
  CHECK_FALSE(cutShort(l, 0, 0));
  CHECK_FALSE(cutShort(l, 18, 11));
}

TEST_CASE("the east and south edges end in blocks cut short, on the grid's last vertex", "[tiles]") {
  const TileLayout l = city();
  // 12000 + 911 = 12911 and 19000 + 695 = 19695 are the last column and the last row.
  CHECK(show(blockRect(l, 19, 12)) == "columns 12000 to 12911, rows 19000 to 19695");
  CHECK(cutShort(l, 19, 12));
  CHECK(show(blockRect(l, 0, 12)) == "columns 12000 to 12911, rows 0 to 1000");  // cut short by its width alone
  CHECK(cutShort(l, 0, 12));
  CHECK(show(blockRect(l, 19, 0)) == "columns 0 to 1000, rows 19000 to 19695");  // and by its height alone
  CHECK(cutShort(l, 19, 0));
  CHECK(show(blockRect(l, 19, 11)) == "columns 11000 to 12000, rows 19000 to 19695");
  CHECK(cutShort(l, 19, 11));

  // The east column of 20 blocks and the south row of 13 share the corner: 32 are cut short, which is the count the
  // summary line of a City run gives. All 260 blocks tile the 12911 x 19695 cells.
  int cut = 0;
  for (int r = 0; r < l.down; ++r) {
    for (int c = 0; c < l.across; ++c) cut += cutShort(l, r, c) ? 1 : 0;
  }
  CHECK(cut == 32);
  CHECK(tilingProblem(l) == "");
}

TEST_CASE("a grid the blocks divide exactly has no block cut short", "[tiles]") {
  // 2000 x 1000 cells in blocks of 100: 20 across and 10 down, the last blocks ending exactly on the last vertices.
  const TileLayout l = tileLayout(2001, 1001, 100, 1, 1, 1);
  CHECK(l.blockCols == 100);
  CHECK(l.blockRows == 100);
  CHECK(l.across == 20);
  CHECK(l.down == 10);
  CHECK(show(blockRect(l, 9, 19)) == "columns 1900 to 2000, rows 900 to 1000");
  for (int r = 0; r < l.down; ++r) {
    for (int c = 0; c < l.across; ++c) {
      CAPTURE(r, c);
      CHECK_FALSE(cutShort(l, r, c));
    }
  }
}

TEST_CASE("the blocks tile the grid exactly, whatever the grid and the block size", "[tiles]") {
  // Cells 1 m wide and 2 m tall, so that blocks are not square in cells either; tiles from under a cell to far more
  // than the grid, on grids that the blocks divide exactly and on grids that they do not.
  for (const int cols : {2, 3, 7, 10, 11, 12, 101}) {
    for (const int rows : {2, 3, 8, 10, 11, 100}) {
      for (const double tile : {0.4, 1.0, 2.0, 3.0, 7.0, 10.0, 100.0, 5000.0}) {
        const TileLayout l = tileLayout(cols, rows, tile, 1, 1, 2);
        INFO("cols " << cols << ", rows " << rows << ", tile " << tile << ": blocks of " << l.blockCols << " x "
                     << l.blockRows << " cells, " << l.across << " across and " << l.down << " down");
        CHECK(l.cols == cols);
        CHECK(l.rows == rows);
        CHECK(tilingProblem(l) == "");
      }
    }
  }
}

TEST_CASE("a geographic grid has cells of different width and height, so blocks of different cells", "[tiles]") {
  // Cells 92.2 m wide and 110.9 m tall, as a grid in degrees has at about 34 degrees south. Blocks of 100 units at
  // --scale 0.01 are 10 km: 10000 / 92.2 = 108.46 cells wide and 10000 / 110.9 = 90.17 cells tall.
  const TileLayout l = tileLayout(2001, 1801, 100, 0.01, 92.2, 110.9);
  CHECK(l.blockCols == 108);
  CHECK(l.blockRows == 90);
  CHECK(l.blockCols == std::lround(100 / (0.01 * 92.2)));
  CHECK(l.blockRows == std::lround(100 / (0.01 * 110.9)));
  CHECK(l.across == 19);  // 2000 cells: 18 blocks of 108 and one of 56
  CHECK(l.down == 20);    // 1800 cells: 20 blocks of 90, exactly
  CHECK(show(blockRect(l, 19, 18)) == "columns 1944 to 2000, rows 1710 to 1800");
  CHECK(cutShort(l, 19, 18));
  CHECK(cutShort(l, 0, 18));        // 56 cells wide
  CHECK_FALSE(cutShort(l, 19, 0));  // the rows divide exactly, so the south edge is not cut short
  CHECK(tilingProblem(l) == "");
}

TEST_CASE("a tile larger than the grid is one block of the whole grid", "[tiles]") {
  const TileLayout l = tileLayout(11, 7, 1000, 1, 1, 1);
  CHECK(l.blockCols == 10);
  CHECK(l.blockRows == 6);
  CHECK(l.across == 1);
  CHECK(l.down == 1);
  CHECK(show(blockRect(l, 0, 0)) == "columns 0 to 10, rows 0 to 6");
  CHECK_FALSE(cutShort(l, 0, 0));  // the block is the grid, not a piece of a bigger one
}

TEST_CASE("a tile of under one cell is blocks of one cell", "[tiles]") {
  const TileLayout l = tileLayout(5, 4, 0.3, 1, 1, 1);
  CHECK(l.blockCols == 1);
  CHECK(l.blockRows == 1);
  CHECK(l.across == 4);
  CHECK(l.down == 3);
  CHECK(show(blockRect(l, 2, 3)) == "columns 3 to 4, rows 2 to 3");
}

TEST_CASE("the cells of a block are rounded to the nearest, and a half cell rounds up", "[tiles]") {
  // Halves are exact in binary, so these are the rule's own halves; 0.5 is also a tile of under one cell.
  CHECK(tileLayout(100, 100, 0.5, 1, 1, 1).blockCols == 1);
  CHECK(tileLayout(100, 100, 1.5, 1, 1, 1).blockCols == 2);
  CHECK(tileLayout(100, 100, 2.5, 1, 1, 1).blockCols == 3);
  CHECK(tileLayout(100, 100, 3.5, 1, 1, 1).blockCols == 4);
  CHECK(tileLayout(100, 100, 2.4, 1, 1, 1).blockCols == 2);
  CHECK(tileLayout(100, 100, 2.6, 1, 1, 1).blockCols == 3);
  // The rows take the cell height, and the scale turns units into metres: 5 units at --scale 0.5 are 10 m, and over
  // cells 4 m tall that is 2.5 cells, which rounds up to 3; over cells 1 m wide it is 10.
  const TileLayout l = tileLayout(100, 100, 5, 0.5, 1, 4);
  CHECK(l.blockCols == 10);
  CHECK(l.blockRows == 3);
}

TEST_CASE("a ratio past any int is a block of the whole grid or of one cell, not an overflow", "[tiles]") {
  // The product of scale and cell underflows to 0 here, so the ratio is infinite.
  const TileLayout whole = tileLayout(5, 4, 1e300, 1e-300, 1e-300, 1e-300);
  CHECK(whole.blockCols == 4);
  CHECK(whole.blockRows == 3);
  CHECK(whole.across == 1);
  CHECK(whole.down == 1);
  // It overflows to infinity here, so the ratio is 0.
  const TileLayout single = tileLayout(5, 4, 1e-300, 1e200, 1e200, 1e200);
  CHECK(single.blockCols == 1);
  CHECK(single.blockRows == 1);
  CHECK(single.across == 4);
  CHECK(single.down == 3);
  // A finite ratio far beyond an int.
  CHECK(tileLayout(5, 4, 1e300, 1, 1, 1).blockCols == 4);
}

TEST_CASE("a grid of the most vertices an int counts is laid out without overflow", "[tiles]") {
  constexpr int kMost = std::numeric_limits<int>::max();  // 2147483647 vertices, 2147483646 cells
  const TileLayout l = tileLayout(kMost, kMost, 1000, 1, 1, 1);
  CHECK(l.blockCols == 1000);
  CHECK(l.blockRows == 1000);
  CHECK(l.across == 2147484);  // 2147483 blocks of 1000 cells and one of 646
  CHECK(l.down == 2147484);
  const int last = l.across - 1;
  CHECK(show(blockRect(l, last, last)) == "columns 2147483000 to 2147483646, rows 2147483000 to 2147483646");
  CHECK(cutShort(l, last, last));
  CHECK_FALSE(cutShort(l, last - 1, last - 1));
  CHECK(blockName(l, last, last) == "r2147484_c2147484");
}

TEST_CASE("a block side above 16384 cells is refused, naming the side", "[tiles]") {
  static_assert(kMostChunkCells == 16384);
  // 16384 is the most a side may have.
  const TileLayout most = tileLayout(16385, 16385, 16384, 1, 1, 1);
  CHECK(most.blockCols == 16384);
  CHECK(most.blockRows == 16384);
  CHECK(most.across == 1);
  CHECK(most.down == 1);
  // One more is too wide, or too tall.
  REQUIRE_THROWS_MATCHES(tileLayout(16386, 100, 16385, 1, 1, 1), std::invalid_argument,
                         Message(tooLarge(16385, "wide")));
  REQUIRE_THROWS_MATCHES(tileLayout(100, 16386, 16385, 1, 1, 1), std::invalid_argument,
                         Message(tooLarge(16385, "tall")));
  // When both are, the width is named.
  REQUIRE_THROWS_MATCHES(tileLayout(20001, 20001, 20000, 1, 1, 1), std::invalid_argument,
                         Message(tooLarge(20000, "wide")));
}

TEST_CASE("a tile that makes the whole grid one block is refused when the grid is above 16384 cells", "[tiles]") {
  // The City as one block: 100000 units is 1e6 cells, which the grid cuts to 12911 across (fine) and 19695 down.
  REQUIRE_THROWS_MATCHES(tileLayout(12912, 19696, 100000, 0.02, 5, 5), std::invalid_argument,
                         Message(tooLarge(19695, "tall")));
  REQUIRE_THROWS_MATCHES(tileLayout(20001, 100, 1e12, 1, 1, 1), std::invalid_argument,
                         Message(tooLarge(20000, "wide")));
}

TEST_CASE("a grid above 16384 cells a side is fine in blocks of fewer", "[tiles]") {
  const TileLayout l = tileLayout(100000, 50000, 1000, 1, 1, 1);
  CHECK(l.blockCols == 1000);
  CHECK(l.blockRows == 1000);
  CHECK(l.across == 100);  // 99999 cells
  CHECK(l.down == 50);     // 49999 cells
}

TEST_CASE("tileLayout refuses a grid of under 2 x 2 vertices", "[tiles]") {
  const std::string head = "the grid must be at least 2 x 2 vertices, not ";
  REQUIRE_THROWS_MATCHES(tileLayout(1, 5, 1, 1, 1, 1), std::invalid_argument, Message(head + "1 x 5"));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 1, 1, 1, 1, 1), std::invalid_argument, Message(head + "5 x 1"));
  REQUIRE_THROWS_MATCHES(tileLayout(0, 0, 1, 1, 1, 1), std::invalid_argument, Message(head + "0 x 0"));
  REQUIRE_THROWS_MATCHES(tileLayout(-3, 4, 1, 1, 1, 1), std::invalid_argument, Message(head + "-3 x 4"));
}

TEST_CASE("tileLayout refuses a tile, scale or cell size that is not above 0", "[tiles]") {
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, 0, 1, 1, 1), std::invalid_argument,
                         Message("the tile size must be above 0 and finite, not 0"));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, -1, 1, 1, 1), std::invalid_argument,
                         Message("the tile size must be above 0 and finite, not -1"));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, 1, 0, 1, 1), std::invalid_argument,
                         Message("the scale must be above 0 and finite, not 0"));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, 1, 1, -2.5, 1), std::invalid_argument,
                         Message("the cell width must be above 0 and finite, not -2.5"));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, 1, 1, 1, 0), std::invalid_argument,
                         Message("the cell height must be above 0 and finite, not 0"));
}

TEST_CASE("tileLayout refuses NaN and infinity wherever they stand", "[tiles]") {
  const double bad = GENERATE(std::numeric_limits<double>::quiet_NaN(), std::numeric_limits<double>::infinity(),
                              -std::numeric_limits<double>::infinity());
  CAPTURE(bad);
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, bad, 1, 1, 1), std::invalid_argument,
                         MessageMatches(ContainsSubstring("the tile size must be above 0 and finite")));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, 1, bad, 1, 1), std::invalid_argument,
                         MessageMatches(ContainsSubstring("the scale must be above 0 and finite")));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, 1, 1, bad, 1), std::invalid_argument,
                         MessageMatches(ContainsSubstring("the cell width must be above 0 and finite")));
  REQUIRE_THROWS_MATCHES(tileLayout(5, 5, 1, 1, 1, bad), std::invalid_argument,
                         MessageMatches(ContainsSubstring("the cell height must be above 0 and finite")));
}

TEST_CASE("blockRect and blockName refuse a block outside the lattice, and cutShort says false for it", "[tiles]") {
  const TileLayout l = city();  // 20 rows and 13 columns of blocks
  const auto outside = [](int r, int c) {
    return "the block at row " + std::to_string(r) + ", column " + std::to_string(c) +
           " is outside the lattice of 20 rows and 13 columns, counted from 0";
  };
  REQUIRE_THROWS_MATCHES(blockRect(l, 20, 0), std::invalid_argument, Message(outside(20, 0)));
  REQUIRE_THROWS_MATCHES(blockRect(l, 0, 13), std::invalid_argument, Message(outside(0, 13)));
  REQUIRE_THROWS_MATCHES(blockRect(l, -1, 0), std::invalid_argument, Message(outside(-1, 0)));
  REQUIRE_THROWS_MATCHES(blockRect(l, 0, -1), std::invalid_argument, Message(outside(0, -1)));
  REQUIRE_THROWS_MATCHES(blockName(l, 20, 12), std::invalid_argument, Message(outside(20, 12)));
  REQUIRE_THROWS_MATCHES(blockName(l, 19, 13), std::invalid_argument, Message(outside(19, 13)));
  // There is no block to be cut short, and cutShort cannot throw.
  static_assert(noexcept(cutShort(l, 0, 0)));
  CHECK_FALSE(cutShort(l, 20, 12));
  CHECK_FALSE(cutShort(l, 19, 13));
  CHECK_FALSE(cutShort(l, -1, 0));
  CHECK_FALSE(cutShort(l, 0, -1));
}

// --- blockName -------------------------------------------------------------------------------------------------------

TEST_CASE("block names count from 1 and pad to the digits the counts need", "[tiles]") {
  const TileLayout l = city();  // 20 down and 13 across need two digits each
  CHECK(blockName(l, 2, 4) == "r03_c05");
  CHECK(blockName(l, 0, 0) == "r01_c01");
  CHECK(blockName(l, 19, 12) == "r20_c13");
  // 7 down and 9 across need one digit each, so nothing pads.
  const TileLayout small = tileLayout(10, 8, 1, 1, 1, 1);
  REQUIRE(small.down == 7);
  REQUIRE(small.across == 9);
  CHECK(blockName(small, 2, 4) == "r3_c5");
  CHECK(blockName(small, 0, 0) == "r1_c1");
  CHECK(blockName(small, 6, 8) == "r7_c9");
}

TEST_CASE("each side pads to its own count, and 100 blocks need three digits", "[tiles]") {
  // 10 down need two digits, 3 across need one.
  const TileLayout tall = tileLayout(4, 11, 1, 1, 1, 1);
  REQUIRE(tall.across == 3);
  REQUIRE(tall.down == 10);
  CHECK(blockName(tall, 0, 0) == "r01_c1");
  CHECK(blockName(tall, 9, 2) == "r10_c3");
  // 100 across need three digits.
  const TileLayout wide = tileLayout(101, 2, 1, 1, 1, 1);
  REQUIRE(wide.across == 100);
  REQUIRE(wide.down == 1);
  CHECK(blockName(wide, 0, 0) == "r1_c001");
  CHECK(blockName(wide, 0, 98) == "r1_c099");
  CHECK(blockName(wide, 0, 99) == "r1_c100");
  // 99 need two.
  const TileLayout narrower = tileLayout(100, 2, 1, 1, 1, 1);
  REQUIRE(narrower.across == 99);
  CHECK(blockName(narrower, 0, 0) == "r1_c01");
  CHECK(blockName(narrower, 0, 98) == "r1_c99");
}

TEST_CASE("names sort in lattice order, so that a directory lists the blocks row by row", "[tiles]") {
  for (const TileLayout& l : {city(), tileLayout(10, 8, 1, 1, 1, 1), tileLayout(4, 11, 1, 1, 1, 1),
                              tileLayout(101, 2, 1, 1, 1, 1), tileLayout(100, 2, 1, 1, 1, 1)}) {
    const std::vector<std::string> names = allNames(l);
    REQUIRE(names.size() == static_cast<std::size_t>(l.across) * static_cast<std::size_t>(l.down));
    for (std::size_t i = 1; i < names.size(); ++i) {
      INFO(names[i - 1] << " should sort before " << names[i]);
      CHECK(names[i - 1] < names[i]);
    }
  }
}

// --- parseBlocks and BlockSelection ----------------------------------------------------------------------------------

TEST_CASE("parseBlocks reads ROWS:COLS, each part a number or an inclusive range from 1", "[tiles]") {
  CHECK(show(parseBlocks("3-5:2-4")) == "rows 3 to 5, columns 2 to 4");
  CHECK(show(parseBlocks("7:12")) == "rows 7 to 7, columns 12 to 12");
  CHECK(show(parseBlocks("1-1:1-1")) == "rows 1 to 1, columns 1 to 1");
  CHECK(show(parseBlocks("4-9:6")) == "rows 4 to 9, columns 6 to 6");  // a range and a number
  CHECK(show(parseBlocks("6:4-9")) == "rows 6 to 6, columns 4 to 9");
  CHECK_FALSE(parseBlocks("7:12").all);
}

TEST_CASE("a default BlockSelection selects every block", "[tiles]") {
  const BlockSelection everything;
  CHECK(show(everything) == "all");
  CHECK(everything.selected(0, 0));
  CHECK(everything.selected(19, 12));
  CHECK(everything.selected(std::numeric_limits<int>::max(), std::numeric_limits<int>::max()));
}

TEST_CASE("selected counts the lattice from 0 against ranges counted from 1", "[tiles]") {
  // Rows 3 to 5 and columns 2 to 4 are the lattice's rows 2 to 4 and columns 1 to 3.
  const BlockSelection s = parseBlocks("3-5:2-4");
  CHECK(s.selected(2, 1));  // the four corners of the selection
  CHECK(s.selected(2, 3));
  CHECK(s.selected(4, 1));
  CHECK(s.selected(4, 3));
  CHECK(s.selected(3, 2));
  CHECK_FALSE(s.selected(1, 2));  // the row above, the row below, the column to the west, the column to the east
  CHECK_FALSE(s.selected(5, 2));
  CHECK_FALSE(s.selected(3, 0));
  CHECK_FALSE(s.selected(3, 4));
  CHECK_FALSE(s.selected(0, 0));
  for (int r = 0; r < 9; ++r) {
    for (int c = 0; c < 9; ++c) {
      CAPTURE(r, c);
      CHECK(s.selected(r, c) == (r + 1 >= 3 && r + 1 <= 5 && c + 1 >= 2 && c + 1 <= 4));
    }
  }

  const BlockSelection one = parseBlocks("7:12");
  CHECK(one.selected(6, 11));
  CHECK_FALSE(one.selected(5, 11));
  CHECK_FALSE(one.selected(7, 11));
  CHECK_FALSE(one.selected(6, 10));
  CHECK_FALSE(one.selected(6, 12));

  const BlockSelection first = parseBlocks("1-1:1-1");
  CHECK(first.selected(0, 0));
  CHECK_FALSE(first.selected(0, 1));
  CHECK_FALSE(first.selected(1, 0));
  CHECK_FALSE(first.selected(1, 1));
}

TEST_CASE("the --blocks text counts as the names do: 3:5 is the block named r03_c05", "[tiles]") {
  const TileLayout l = city();
  CHECK(picked(l, parseBlocks("3:5")) == std::vector<std::string>{"r03_c05"});
  CHECK(picked(l, parseBlocks("3-5:2-4")) ==
        std::vector<std::string>{"r03_c02", "r03_c03", "r03_c04", "r04_c02", "r04_c03", "r04_c04", "r05_c02",
                                 "r05_c03", "r05_c04"});
  CHECK(picked(l, BlockSelection{}) == allNames(l));
}

TEST_CASE("a selection beyond the lattice is not an error here: it picks the blocks that exist", "[tiles]") {
  // main reports a range that reaches past the lattice; the unit only says which blocks a selection holds.
  const TileLayout l = city();  // 20 rows and 13 columns
  CHECK(picked(l, parseBlocks("21:1")).empty());
  CHECK(picked(l, parseBlocks("1:14")).empty());
  CHECK(picked(l, parseBlocks("19-25:12-30")) ==
        std::vector<std::string>{"r19_c12", "r19_c13", "r20_c12", "r20_c13"});
}

TEST_CASE("parseBlocks accepts leading zeros, as the block names carry them, and the largest int", "[tiles]") {
  CHECK(show(parseBlocks("03:05")) == "rows 3 to 3, columns 5 to 5");
  CHECK(show(parseBlocks("003-005:0012")) == "rows 3 to 5, columns 12 to 12");
  // Far more digits than an int holds, but a value of 1.
  CHECK(show(parseBlocks("00000000000000000000000000000001:1")) == "rows 1 to 1, columns 1 to 1");

  constexpr int kMost = std::numeric_limits<int>::max();
  const BlockSelection last = parseBlocks("2147483647:2147483647");
  CHECK(show(last) == "rows 2147483647 to 2147483647, columns 2147483647 to 2147483647");
  CHECK(last.selected(kMost - 1, kMost - 1));
  CHECK_FALSE(last.selected(0, 0));
  CHECK_FALSE(last.selected(kMost, kMost));  // block number kMost + 1: nothing to overflow
  const BlockSelection everything = parseBlocks("1-2147483647:1-2147483647");
  CHECK(everything.selected(0, 0));
  CHECK(everything.selected(kMost - 1, kMost - 1));
}

TEST_CASE("parseBlocks refuses text that is not ROWS:COLS of numbers and ranges", "[tiles]") {
  const std::string text = GENERATE(as<std::string>{}, "", "7", "3-5", "a:b", ":", "3:", ":3", "::", "1:2:3", "3-:1",
                                    "-3:1", "3--5:1", "1-2-3:1", "1:2-", "1:-2", " 1:2", "1: 2", "1:2 ", "+1:2",
                                    "1.5:2", "1,5:2", "0x1:2", "1;2", "3 5", "1e3:1", "all", "3-5:", "3:5-");
  CAPTURE(text);
  REQUIRE_THROWS_MATCHES(parseBlocks(text), std::invalid_argument, Message(kBadForm + "'" + text + "'"));
}

TEST_CASE("parseBlocks refuses a block number below 1 or above what an int holds", "[tiles]") {
  const std::string head = "block numbers run from 1 to 2147483647, not ";
  REQUIRE_THROWS_MATCHES(parseBlocks("0:1"), std::invalid_argument, Message(head + "0 in '0:1'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("1:0"), std::invalid_argument, Message(head + "0 in '1:0'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("00:1"), std::invalid_argument, Message(head + "00 in '00:1'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("0-3:1"), std::invalid_argument, Message(head + "0 in '0-3:1'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("3-0:1"), std::invalid_argument, Message(head + "0 in '3-0:1'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("2147483648:1"), std::invalid_argument,
                         Message(head + "2147483648 in '2147483648:1'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("1-2147483648:1"), std::invalid_argument,
                         Message(head + "2147483648 in '1-2147483648:1'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("1:99999999999999999999"), std::invalid_argument,
                         Message(head + "99999999999999999999 in '1:99999999999999999999'"));
}

TEST_CASE("parseBlocks refuses a range that ends below its start", "[tiles]") {
  const std::string head = "a range must end at or above its start, not ";
  REQUIRE_THROWS_MATCHES(parseBlocks("5-3:1"), std::invalid_argument, Message(head + "5-3 in '5-3:1'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("1:5-3"), std::invalid_argument, Message(head + "5-3 in '1:5-3'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("05-4:1"), std::invalid_argument, Message(head + "05-4 in '05-4:1'"));
  // A range that ends where it starts is a block.
  CHECK(show(parseBlocks("5-5:2-2")) == "rows 5 to 5, columns 2 to 2");
}

TEST_CASE("parseBlocks names the first fault it finds: the form, then a number, then a range's order", "[tiles]") {
  // 5-3 is reversed, but x is no number: the form comes first.
  REQUIRE_THROWS_MATCHES(parseBlocks("5-3:x"), std::invalid_argument, Message(kBadForm + "'5-3:x'"));
  // The same range with a 0 after it: a number comes before the order.
  REQUIRE_THROWS_MATCHES(parseBlocks("5-3:0"), std::invalid_argument,
                         Message("block numbers run from 1 to 2147483647, not 0 in '5-3:0'"));
  // Two reversed ranges: the rows' is named.
  REQUIRE_THROWS_MATCHES(parseBlocks("5-3:9-8"), std::invalid_argument,
                         Message("a range must end at or above its start, not 5-3 in '5-3:9-8'"));
}

TEST_CASE("parseBlocks shows stray characters in the text it refuses, as the other usage errors do", "[tiles]") {
  // A carriage return from a script edited on Windows is the likeliest; it must not move the cursor in the message.
  REQUIRE_THROWS_MATCHES(parseBlocks("1:2\r"), std::invalid_argument, Message(kBadForm + "'1:2\\r'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("1:2\n"), std::invalid_argument, Message(kBadForm + "'1:2\\n'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("1:2\t"), std::invalid_argument, Message(kBadForm + "'1:2\\t'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("1\\2"), std::invalid_argument, Message(kBadForm + "'1\\\\2'"));
  REQUIRE_THROWS_MATCHES(parseBlocks("'1:2"), std::invalid_argument, Message(kBadForm + "'\\'1:2'"));
  // A full-width 3 (U+FF13) is not a digit; the bytes of UTF-8 are shown in hex.
  REQUIRE_THROWS_MATCHES(parseBlocks("1:\xEF\xBC\x93"), std::invalid_argument,
                         Message(kBadForm + "'1:\\xEF\\xBC\\x93'"));
  REQUIRE_THROWS_MATCHES(parseBlocks(std::string("1:2\0", 4)), std::invalid_argument,
                         Message(kBadForm + "'1:2\\x00'"));
}
