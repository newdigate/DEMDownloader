#include "mesh/Footprint.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>

namespace demdl::mesh {

namespace {

std::string gridSize(int cols, int rows) { return std::to_string(cols) + " x " + std::to_string(rows); }

std::string formatNumber(double value) {
  char text[40];
  std::snprintf(text, sizeof text, "%.15g", value);
  return text;
}

std::string describe(GridPoint p) { return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

std::string describe(const BlockRect& rect) {
  return "the block of columns " + std::to_string(rect.firstColumn) + " to " + std::to_string(rect.lastColumn) +
         " and rows " + std::to_string(rect.firstRow) + " to " + std::to_string(rect.lastRow);
}

// --- resampleMaskNearest ---------------------------------------------------------------------------------------------

// For each of outCount outputs along an axis, the source sample nearest to where it stands: output i stands at
// i (srcCount - 1) / (outCount - 1), and a position half way between two samples goes to the lower one. In whole
// numbers: i x (srcCount - 1) is below 2^62, and the remainder of the division says which side of half way it is.
std::vector<std::size_t> nearestSamples(int srcCount, int outCount) {
  const auto span = static_cast<std::uint64_t>(srcCount - 1);
  const auto steps = static_cast<std::uint64_t>(outCount - 1);
  std::vector<std::size_t> nearest(static_cast<std::size_t>(outCount));
  for (std::uint64_t i = 0; i < nearest.size(); ++i) {
    const std::uint64_t scaled = i * span;
    const std::uint64_t below = scaled / steps;
    nearest[i] = static_cast<std::size_t>(2 * (scaled % steps) > steps ? below + 1 : below);
  }
  return nearest;
}

// --- footprintCells --------------------------------------------------------------------------------------------------

// One half of the 3 x 3 closing over a grid of width x height cells, in place. A dilation turns a cell in when any
// cell of the square around it is in; an erosion keeps it in only when all of them are. The square is a row of three
// times a column of three, so the pass is made along the rows and then down the columns, each step reading the cells as
// they were before it: along a row the cell before is carried in a variable, and down the columns the row above is
// kept in a copy. Beyond the grid counts as out for a dilation and as in for an erosion.
template <bool kDilation>
void squarePass(std::vector<std::uint8_t>& cells, std::size_t width, std::size_t height) {
  constexpr std::uint8_t kBeyond = kDilation ? 0 : 1;
  const auto combine = [](std::uint8_t a, std::uint8_t b, std::uint8_t c) {
    if constexpr (kDilation) {
      return static_cast<std::uint8_t>(a | b | c);
    } else {
      return static_cast<std::uint8_t>(a & b & c);
    }
  };
  for (std::size_t y = 0; y < height; ++y) {
    std::uint8_t* row = cells.data() + y * width;
    std::uint8_t before = kBeyond;  // the cell to the west as it was
    for (std::size_t x = 0; x < width; ++x) {
      const std::uint8_t self = row[x];
      row[x] = combine(before, self, x + 1 < width ? row[x + 1] : kBeyond);
      before = self;
    }
  }
  std::vector<std::uint8_t> above(width, kBeyond), self(width);  // the rows above and at y as they were
  for (std::size_t y = 0; y < height; ++y) {
    std::uint8_t* row = cells.data() + y * width;
    const std::uint8_t* below = y + 1 < height ? row + width : nullptr;
    std::copy(row, row + width, self.begin());
    for (std::size_t x = 0; x < width; ++x) row[x] = combine(above[x], self[x], below ? below[x] : kBeyond);
    above.swap(self);
  }
}

// Turns in every region of out cells that does not reach the grid's edge. The out cells that do are found by a
// breadth-first search from those on the edge, through out cells, a side at a time: each wave marks the out cells
// beside the one before. Whatever is not marked is in. Only the current wave and the next are held.
void fillEnclosed(std::vector<std::uint8_t>& cells, std::size_t width, std::size_t height) {
  constexpr std::uint8_t kSea = 2;  // out and joined to the edge
  const std::size_t count = width * height;
  std::vector<std::size_t> wave, next;
  const auto reach = [&](std::size_t c) {
    if (cells[c] != 0) return;
    cells[c] = kSea;
    next.push_back(c);
  };
  for (std::size_t x = 0; x < width; ++x) {
    reach(x);
    reach(count - width + x);
  }
  for (std::size_t y = 0; y < height; ++y) {
    reach(y * width);
    reach(y * width + width - 1);
  }
  while (!next.empty()) {
    wave.swap(next);
    next.clear();
    for (const std::size_t c : wave) {
      const std::size_t x = c % width;
      if (c >= width) reach(c - width);
      if (x > 0) reach(c - 1);
      if (x + 1 < width) reach(c + 1);
      if (c + width < count) reach(c + width);
    }
  }
  for (std::uint8_t& v : cells) v = v == kSea ? 0 : 1;
}

// --- blockFootprint --------------------------------------------------------------------------------------------------

// The footprint's cells as a block sees them: in() is true for a cell of the footprint inside the block's cell range
// and false for every other cell, beyond the grid included; across() reads the whole grid's cells, inside the block or
// not, for cells known to be on the grid.
class BlockView {
 public:
  BlockView(const std::vector<std::uint8_t>& cells, int cols, const BlockRect& rect)
      : cells_(cells), width_(static_cast<std::size_t>(cols - 1)), rect_(rect) {}

  [[nodiscard]] bool in(std::int64_t x, std::int64_t y) const noexcept {
    return x >= rect_.firstColumn && x < rect_.lastColumn && y >= rect_.firstRow && y < rect_.lastRow &&
           across(x, y);
  }
  [[nodiscard]] bool in(GridPoint cell) const noexcept { return in(cell.x, cell.y); }

  [[nodiscard]] bool across(std::int64_t x, std::int64_t y) const noexcept {
    return cells_[static_cast<std::size_t>(y) * width_ + static_cast<std::size_t>(x)] != 0;
  }

 private:
  const std::vector<std::uint8_t>& cells_;
  std::size_t width_;
  BlockRect rect_;
};

// The four ways along the grid lines, as steps in (column, row), each a right turn from the one before seen from
// above: east, south, west, north.
constexpr std::array<GridPoint, 4> kStep{{{1, 0}, {0, 1}, {-1, 0}, {0, -1}}};
constexpr std::size_t kSouth = 1;

// The cell ahead and to the left, seen from above, of a walk that has come to a vertex going kStep[d], as an offset
// from the vertex to the cell's own north-west corner: going east, the cell north-east of the vertex; going south, the
// one south-east; going west, the one south-west; going north, the one north-west. The cell ahead and to the right is
// the one ahead and to the left of the way a right turn faces.
constexpr std::array<GridPoint, 4> kAheadLeft{{{0, -1}, {0, 0}, {-1, 0}, {-1, -1}}};

GridPoint plus(GridPoint a, GridPoint b) noexcept { return {a.x + b.x, a.y + b.y}; }
bool same(GridPoint a, GridPoint b) noexcept { return a.x == b.x && a.y == b.y; }
bool rowMajorLess(GridPoint a, GridPoint b) noexcept { return a.y != b.y ? a.y < b.y : a.x < b.x; }

// A piece as the search and the walk find it, before it is kept or dropped.
struct Outlined {
  Piece piece;
  std::vector<GridPoint> contacts;  // where the walk met a cell in across a diagonal: another piece's, or its own
};

// The outline of the piece whose first cell is `first`, walked from the cell's north-west corner going south with the
// piece on the left, as the header says. sides is the number of the piece's cell sides with no cell of it beyond, so
// the number of steps round its whole outline; a loop that takes fewer leaves a hole's sides out. contacts receives
// every vertex where the cell ahead on the left was out and the one ahead on the right in: a vertex with the piece on
// one diagonal and a cell in on the other, which is another piece's unless the piece meets itself there.
std::vector<GridPoint> walkOutline(const BlockView& block, GridPoint first, std::uint64_t sides,
                                   std::vector<GridPoint>& contacts) {
  std::vector<GridPoint> loop;
  GridPoint at = first;
  std::size_t heading = kSouth;
  do {
    loop.push_back(at);
    at = plus(at, kStep[heading]);
    const bool left = block.in(plus(at, kAheadLeft[heading]));
    const bool right = block.in(plus(at, kAheadLeft[(heading + 1) % 4]));
    if (!left) {
      if (right) contacts.push_back(at);
      heading = (heading + 3) % 4;
    } else if (right) {
      heading = (heading + 1) % 4;
    }
    // The walk goes from side to side of the piece and takes each at most once, so it cannot outrun their count.
    if (loop.size() > sides) throw std::logic_error("the walk round a piece took more steps than it has sides");
  } while (!(same(at, first) && heading == kSouth));
  return loop;
}

// The indices of loop where its runs start: where it turns, and on a side of rect that is not on the grid's edge,
// where the cells across that side before and after the sample differ.
std::vector<std::uint32_t> runStartsOf(const std::vector<GridPoint>& loop, const BlockView& block,
                                       const BlockRect& rect, int cols, int rows) {
  std::vector<std::uint32_t> starts;
  const std::size_t n = loop.size();
  for (std::size_t k = 0; k < n; ++k) {
    const GridPoint before = loop[(k + n - 1) % n], at = loop[k], after = loop[(k + 1) % n];
    bool start = at.x - before.x != after.x - at.x || at.y - before.y != after.y - at.y;
    // A straight stretch never passes a corner of rect, where the outline always turns, so the cells looked at below
    // are on the grid: on the column of cells across the west or east side, the rows either side of the sample, and
    // likewise along the north and south sides.
    if (!start && before.x == at.x) {
      if (at.x == rect.lastColumn && rect.lastColumn < cols - 1) {
        start = block.across(rect.lastColumn, at.y - 1) != block.across(rect.lastColumn, at.y);
      } else if (at.x == rect.firstColumn && rect.firstColumn > 0) {
        start = block.across(rect.firstColumn - 1, at.y - 1) != block.across(rect.firstColumn - 1, at.y);
      }
    } else if (!start) {
      if (at.y == rect.lastRow && rect.lastRow < rows - 1) {
        start = block.across(at.x - 1, rect.lastRow) != block.across(at.x, rect.lastRow);
      } else if (at.y == rect.firstRow && rect.firstRow > 0) {
        start = block.across(at.x - 1, rect.firstRow - 1) != block.across(at.x, rect.firstRow - 1);
      }
    }
    if (start) starts.push_back(static_cast<std::uint32_t>(k));
  }
  return starts;
}

const char* const kFootprintCellsOnly = "blockFootprint takes the cells footprintCells gives";

}  // namespace

std::vector<std::uint8_t> dataMaskOf(const std::vector<float>& window) {
  std::vector<std::uint8_t> mask(window.size());
  std::transform(window.begin(), window.end(), mask.begin(),
                 [](float h) { return static_cast<std::uint8_t>(std::isfinite(h) ? 1 : 0); });
  return mask;
}

std::vector<std::uint8_t> resampleMaskNearest(const std::vector<std::uint8_t>& mask, int srcCols, int srcRows,
                                              int outCols, int outRows) {
  if (srcCols < 2 || srcRows < 2) {
    throw std::invalid_argument("the source must be at least 2 x 2, not " + gridSize(srcCols, srcRows));
  }
  const std::uint64_t sourceSize = static_cast<std::uint64_t>(srcCols) * static_cast<std::uint64_t>(srcRows);
  if (mask.size() != sourceSize) {
    throw std::invalid_argument("a source of " + gridSize(srcCols, srcRows) + " samples needs " +
                                std::to_string(sourceSize) + " mask values, not " + std::to_string(mask.size()));
  }
  if (outCols < 2 || outRows < 2) {
    throw std::invalid_argument("the output grid must be at least 2 x 2, not " + gridSize(outCols, outRows));
  }
  const std::vector<std::size_t> across = nearestSamples(srcCols, outCols);
  const std::vector<std::size_t> down = nearestSamples(srcRows, outRows);
  const auto sourceWidth = static_cast<std::size_t>(srcCols);
  std::vector<std::uint8_t> out(static_cast<std::size_t>(outCols) * static_cast<std::size_t>(outRows));
  auto o = out.begin();
  for (const std::size_t j : down) {
    const std::uint8_t* row = mask.data() + j * sourceWidth;
    for (const std::size_t i : across) *o++ = row[i];
  }
  return out;
}

std::vector<std::uint8_t> footprintCells(const std::vector<std::uint8_t>& mask, int cols, int rows) {
  if (cols < 2 || rows < 2) {
    throw std::invalid_argument("the grid must be at least 2 x 2 vertices, not " + gridSize(cols, rows));
  }
  const std::uint64_t count = static_cast<std::uint64_t>(cols) * static_cast<std::uint64_t>(rows);
  if (mask.size() != count) {
    throw std::invalid_argument("a grid of " + gridSize(cols, rows) + " vertices needs " + std::to_string(count) +
                                " mask values, not " + std::to_string(mask.size()));
  }
  const auto vertexWidth = static_cast<std::size_t>(cols);
  const std::size_t width = vertexWidth - 1, height = static_cast<std::size_t>(rows) - 1;
  std::vector<std::uint8_t> cells(width * height);
  for (std::size_t y = 0; y < height; ++y) {
    const std::uint8_t* north = mask.data() + y * vertexWidth;
    const std::uint8_t* south = north + vertexWidth;
    std::uint8_t* out = cells.data() + y * width;
    for (std::size_t x = 0; x < width; ++x) {
      out[x] = north[x] != 0 && north[x + 1] != 0 && south[x] != 0 && south[x + 1] != 0 ? 1 : 0;
    }
  }
  squarePass<true>(cells, width, height);
  squarePass<false>(cells, width, height);
  fillEnclosed(cells, width, height);
  return cells;
}

BlockFootprint blockFootprint(const std::vector<std::uint8_t>& cells, int cols, int rows, BlockRect rect,
                              double cellAreaUnits2, double minPieceUnits2) {
  if (cols < 2 || rows < 2) {
    throw std::invalid_argument("the grid must be at least 2 x 2 vertices, not " + gridSize(cols, rows));
  }
  const std::uint64_t cellCount = static_cast<std::uint64_t>(cols - 1) * static_cast<std::uint64_t>(rows - 1);
  if (cells.size() != cellCount) {
    throw std::invalid_argument("a grid of " + gridSize(cols, rows) + " vertices has " + std::to_string(cellCount) +
                                " cells, not " + std::to_string(cells.size()));
  }
  if (rect.firstColumn < 0 || rect.firstRow < 0 || rect.lastColumn > cols - 1 || rect.lastRow > rows - 1) {
    throw std::invalid_argument(describe(rect) + " is not inside the grid of " + gridSize(cols, rows) + " vertices");
  }
  if (rect.lastColumn <= rect.firstColumn || rect.lastRow <= rect.firstRow) {
    throw std::invalid_argument(describe(rect) + " has no cell");
  }
  if (!std::isfinite(cellAreaUnits2) || cellAreaUnits2 <= 0.0) {
    throw std::invalid_argument("the area of a cell must be above 0 and finite, not " + formatNumber(cellAreaUnits2));
  }
  if (!std::isfinite(minPieceUnits2) || minPieceUnits2 < 0.0) {
    throw std::invalid_argument("the least area of a piece must be 0 or more and finite, not " +
                                formatNumber(minPieceUnits2));
  }

  const BlockView block(cells, cols, rect);
  // The block's cells, numbered row by row from its north-west corner: 1 for a cell of the footprint that no search
  // has reached yet, 0 for every other.
  const auto width = static_cast<std::size_t>(rect.lastColumn - rect.firstColumn);
  const auto height = static_cast<std::size_t>(rect.lastRow - rect.firstRow);
  std::vector<std::uint8_t> unreached(width * height);
  for (std::size_t y = 0; y < height; ++y) {
    for (std::size_t x = 0; x < width; ++x) {
      unreached[y * width + x] = block.across(rect.firstColumn + static_cast<std::int64_t>(x),
                                              rect.firstRow + static_cast<std::int64_t>(y)) ? 1 : 0;
    }
  }

  // The pieces in the order of their first cells, each found by a breadth-first search from it that counts its cells
  // and the sides of them that face out of it, then walked round.
  std::vector<Outlined> found;
  std::vector<std::size_t> wave, next;
  for (std::size_t seed = 0; seed < unreached.size(); ++seed) {
    if (unreached[seed] == 0) continue;
    unreached[seed] = 0;
    next.assign(1, seed);
    std::uint64_t pieceCells = 0, sides = 0;
    while (!next.empty()) {
      wave.swap(next);
      next.clear();
      for (const std::size_t c : wave) {
        ++pieceCells;
        const std::size_t x = c % width, y = c / width;
        const std::int64_t gx = rect.firstColumn + static_cast<std::int64_t>(x);
        const std::int64_t gy = rect.firstRow + static_cast<std::int64_t>(y);
        // A neighbour in the footprint and the block is the same piece's; any other is beyond one of its sides.
        const auto side = [&](bool inBlock, std::int64_t nx, std::int64_t ny, std::size_t neighbour) {
          if (!inBlock || !block.across(nx, ny)) {
            ++sides;
          } else if (unreached[neighbour] != 0) {
            unreached[neighbour] = 0;
            next.push_back(neighbour);
          }
        };
        side(y > 0, gx, gy - 1, c - width);
        side(x > 0, gx - 1, gy, c - 1);
        side(x + 1 < width, gx + 1, gy, c + 1);
        side(y + 1 < height, gx, gy + 1, c + width);
      }
    }

    const GridPoint first{static_cast<std::int32_t>(rect.firstColumn + static_cast<std::int64_t>(seed % width)),
                          static_cast<std::int32_t>(rect.firstRow + static_cast<std::int64_t>(seed / width))};
    if (sides > std::numeric_limits<std::uint32_t>::max()) {
      throw std::invalid_argument("the piece whose first cell is " + describe(first) + " has an outline of " +
                                  std::to_string(sides) + " samples, more than 32-bit indices can count");
    }
    Outlined outlined;
    outlined.piece.cells = pieceCells;
    outlined.piece.loop = walkOutline(block, first, sides, outlined.contacts);
    if (outlined.piece.loop.size() != sides) {
      throw std::invalid_argument("the piece whose first cell is " + describe(first) + " has a hole: " +
                                  kFootprintCellsOnly + ", in which every enclosed region is filled");
    }
    // A vertex met twice is one where the piece meets itself: across the diagonal from it was its own cell.
    std::sort(outlined.contacts.begin(), outlined.contacts.end(), rowMajorLess);
    const auto twice = std::adjacent_find(outlined.contacts.begin(), outlined.contacts.end(), same);
    if (twice != outlined.contacts.end()) {
      throw std::invalid_argument("the piece whose first cell is " + describe(first) +
                                  " touches itself at the vertex " + describe(*twice) + ": " + kFootprintCellsOnly +
                                  ", in which no piece does");
    }
    outlined.piece.runStarts = runStartsOf(outlined.piece.loop, block, rect, cols, rows);
    found.push_back(std::move(outlined));
  }

  // Largest first; the sort is stable, so pieces of the same size keep the order of their first cells.
  std::stable_sort(found.begin(), found.end(),
                   [](const Outlined& a, const Outlined& b) { return a.piece.cells > b.piece.cells; });
  BlockFootprint result;
  std::vector<GridPoint> keptContacts;
  for (Outlined& o : found) {
    if (static_cast<double>(o.piece.cells) * cellAreaUnits2 < minPieceUnits2) {
      result.dropped.push_back(std::move(o.piece));
      continue;
    }
    result.cells += o.piece.cells;
    keptContacts.insert(keptContacts.end(), o.contacts.begin(), o.contacts.end());
    result.pieces.push_back(std::move(o.piece));
  }
  // A contact between two kept pieces is met by the walks round both, and no vertex by more than two: its four cells
  // hold the two diagonal cells of the two pieces and nothing else.
  std::sort(keptContacts.begin(), keptContacts.end(), rowMajorLess);
  for (std::size_t i = 1; i < keptContacts.size(); ++i) {
    if (same(keptContacts[i], keptContacts[i - 1])) ++result.cornerContacts;
  }
  return result;
}

}  // namespace demdl::mesh
