#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <map>
#include <random>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mesh/Footprint.h"
#include "mesh/Predicates.h"

using Catch::Matchers::Message;
using demdl::mesh::BlockFootprint;
using demdl::mesh::blockFootprint;
using demdl::mesh::BlockRect;
using demdl::mesh::dataMaskOf;
using demdl::mesh::footprintCells;
using demdl::mesh::GridPoint;
using demdl::mesh::orient2d;
using demdl::mesh::Piece;
using demdl::mesh::resampleMaskNearest;

// Grids are drawn as rows of characters from the north-west corner, '#' for a sample with data or a cell in the
// footprint and '.' for the rest. The cell (c, r) is the square whose corners are the samples (c, r), (c + 1, r),
// (c, r + 1) and (c + 1, r + 1). Expected cells, loops and runs are worked out from the header's rules beside each
// case. The random cases hold the unit to a plain restatement of those rules (referenceFourCorners, referenceClosing,
// referenceEnclosedFilled, componentsIn, expectedRunStarts) and to what an outline is (outlineProblem), never to a
// reading of the code under test.

namespace {

// ---------------------------------------------------------------------------------------------------------------------
// The island, kept apart from the rest of the file so that it can be lifted into a fixture header as it stands.

constexpr int kIslandCols = 24;
constexpr int kIslandRows = 18;

// The island's samples, kIslandCols x kIslandRows, row by row from the north-west: '#' data, '.' no-data. Columns are
// counted from 0 at the west and rows from 0 at the north.
// - The island: 9 x 9 samples, columns 4 to 12 and rows 2 to 10, with a tab of 2 x 5 samples, columns 13 and 14 and
//   rows 4 to 8, that carries it east across column 12, so that a block side down column 12 or 13 cuts it, and one
//   along row 9 does too.
// - Inside it a lake of 2 x 2 no-data samples, columns 7 and 8 and rows 5 and 6, joined to the sea by a channel of
//   no-data one sample wide along row 5, columns 4 to 6.
// - An islet of 3 x 3 samples, columns 4 to 6 and rows 13 to 15.
// - A single data sample alone, at column 18 and row 13.
// - Two blobs of 3 x 3 samples, columns 17 to 19 and rows 2 to 4, and columns 19 to 21 and rows 4 to 6, which share the
//   sample at column 19 and row 4: as cells, two blobs of 2 x 2 that touch at that one vertex.
// Every feature is at least two no-data samples from the grid's edge and from every other feature. A gap of two
// samples is three cells, and the closing joins nothing across a gap of three.
std::vector<std::string> islandMaskRows() {
  return {
      "........................",  //  0
      "........................",  //  1
      "....#########....###....",  //  2
      "....#########....###....",  //  3
      "....###########..#####..",  //  4
      ".........######....###..",  //  5
      "....###..######....###..",  //  6
      "....###########.........",  //  7
      "....###########.........",  //  8
      "....#########...........",  //  9
      "....#########...........",  // 10
      "........................",  // 11
      "........................",  // 12
      "....###...........#.....",  // 13
      "....###.................",  // 14
      "....###.................",  // 15
      "........................",  // 16
      "........................",  // 17
  };
}

// The island's samples as a mask: 1 for data, 0 for no-data, row by row from the north-west.
std::vector<std::uint8_t> islandMask() {
  std::vector<std::uint8_t> mask;
  for (const std::string& row : islandMaskRows()) {
    for (const char c : row) mask.push_back(c == '#' ? 1 : 0);
  }
  return mask;
}

// End of the island.
// ---------------------------------------------------------------------------------------------------------------------

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInfinity = std::numeric_limits<float>::infinity();

// The seed of the random cases, so that a failure can be repeated.
constexpr unsigned kRandomSeed = 20261022;

using Flags = std::vector<std::uint8_t>;  // a mask of samples or a grid's cells, 1 or 0, row by row
using Rows = std::vector<std::string>;    // a picture
using Loop = std::vector<GridPoint>;
using Cell = std::pair<int, int>;         // (column, row)
using Indices = std::vector<std::uint32_t>;

// The four cells that share a side with a cell, as steps.
const std::vector<Cell> kSides{{1, 0}, {0, 1}, {-1, 0}, {0, -1}};

// footprintCells of the island: kIslandCols - 1 by kIslandRows - 1 cells, '#' in the footprint. Worked out in the
// case that checks it.
Rows islandFootprint() {
  return {
      ".......................",  //  0
      ".......................",  //  1
      "....########.....##....",  //  2
      "....########.....##....",  //  3
      "....##########.....##..",  //  4
      "....##########.....##..",  //  5
      "....##########.........",  //  6
      "....##########.........",  //  7
      "....########...........",  //  8
      "....########...........",  //  9
      ".......................",  // 10
      ".......................",  // 11
      ".......................",  // 12
      "....##.................",  // 13
      "....##.................",  // 14
      ".......................",  // 15
      ".......................",  // 16
  };
}

// The values of a picture, '#' as 1 and anything else as 0, row by row.
Flags valuesOf(const Rows& picture) {
  Flags values;
  for (const std::string& row : picture) {
    for (const char c : row) values.push_back(c == '#' ? 1 : 0);
  }
  return values;
}

// The picture of flags laid out `width` across.
Rows pictureOf(const Flags& flags, int width) {
  Rows rows;
  for (std::size_t i = 0; i < flags.size(); ++i) {
    if (i % static_cast<std::size_t>(width) == 0) rows.emplace_back();
    rows.back() += flags[i] != 0 ? '#' : '.';
  }
  return rows;
}

std::string text(GridPoint p) { return "(" + std::to_string(p.x) + ", " + std::to_string(p.y) + ")"; }

std::string text(const Loop& loop) {
  std::string out;
  for (const GridPoint& p : loop) out += (out.empty() ? "" : " ") + text(p);
  return out;
}

std::string text(const BlockRect& rect) {
  return "columns " + std::to_string(rect.firstColumn) + " to " + std::to_string(rect.lastColumn) + " and rows " +
         std::to_string(rect.firstRow) + " to " + std::to_string(rect.lastRow);
}

bool same(GridPoint a, GridPoint b) { return a.x == b.x && a.y == b.y; }

// The samples of the piece's loop where its runs start.
Loop runStartSamples(const Piece& piece) {
  Loop samples;
  for (const std::uint32_t k : piece.runStarts) samples.push_back(piece.loop.at(k));
  return samples;
}

// Whether the footprint has the cell (x, y) of a grid cols vertices across. The cell must be on the grid.
bool cellIn(const Flags& cells, int cols, int x, int y) {
  return cells[static_cast<std::size_t>(y) * static_cast<std::size_t>(cols - 1) + static_cast<std::size_t>(x)] != 0;
}

// footprintCells said as plainly as it can be, from the words of the header, in its three steps: each on a grid of
// its own, every 3 x 3 square looked at cell by cell, and the cells joined to the edge found by a depth-first walk. It
// shares nothing with the unit but the definitions. A plane is width x height cells, indexed [row][column].
using Plane = std::vector<std::vector<bool>>;

// The cells whose four corners all have data.
Plane referenceFourCorners(const Flags& mask, int cols, int rows) {
  const auto sample = [&](int x, int y) { return mask[static_cast<std::size_t>(y * cols + x)] != 0; };
  Plane cells(static_cast<std::size_t>(rows - 1), std::vector<bool>(static_cast<std::size_t>(cols - 1)));
  for (int y = 0; y + 1 < rows; ++y) {
    for (int x = 0; x + 1 < cols; ++x) {
      cells[y][x] = sample(x, y) && sample(x + 1, y) && sample(x, y + 1) && sample(x + 1, y + 1);
    }
  }
  return cells;
}

// The closing with a 3 x 3 square: dilated (any cell of the square in, the cells beyond the grid not counted), then
// eroded (every cell of the square in, the cells beyond the grid counting as in).
Plane referenceClosing(const Plane& cells) {
  const int height = static_cast<int>(cells.size()), width = static_cast<int>(cells[0].size());
  const auto onGrid = [&](int x, int y) { return x >= 0 && x < width && y >= 0 && y < height; };
  Plane dilated = cells, closed = cells;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      bool any = false;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) any = any || (onGrid(x + dx, y + dy) && cells[y + dy][x + dx]);
      }
      dilated[y][x] = any;
    }
  }
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      bool all = true;
      for (int dy = -1; dy <= 1; ++dy) {
        for (int dx = -1; dx <= 1; ++dx) all = all && (!onGrid(x + dx, y + dy) || dilated[y + dy][x + dx]);
      }
      closed[y][x] = all;
    }
  }
  return closed;
}

// Every cell in but those still out that a walk through out cells, a side at a time, reaches from a cell on the
// grid's edge.
Plane referenceEnclosedFilled(const Plane& cells) {
  const int height = static_cast<int>(cells.size()), width = static_cast<int>(cells[0].size());
  Plane joinedToEdge(cells.size(), std::vector<bool>(cells[0].size()));
  std::vector<Cell> pending;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      const bool onEdge = x == 0 || y == 0 || x == width - 1 || y == height - 1;
      if (onEdge && !cells[y][x]) {
        joinedToEdge[y][x] = true;
        pending.push_back({x, y});
      }
    }
  }
  while (!pending.empty()) {
    const Cell at = pending.back();
    pending.pop_back();
    for (const Cell& step : kSides) {
      const int x = at.first + step.first, y = at.second + step.second;
      if (x < 0 || x >= width || y < 0 || y >= height || cells[y][x] || joinedToEdge[y][x]) continue;
      joinedToEdge[y][x] = true;
      pending.push_back({x, y});
    }
  }
  Plane filled = cells;
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) filled[y][x] = !joinedToEdge[y][x];
  }
  return filled;
}

// A plane as flags, row by row.
Flags flagsOf(const Plane& plane) {
  Flags flags;
  for (const std::vector<bool>& row : plane) {
    for (const bool in : row) flags.push_back(in ? 1 : 0);
  }
  return flags;
}

// A 4-connected set of the footprint's cells inside a block, with the first of them row by row from the north-west.
struct Component {
  std::set<Cell> cells;
  Cell first;
};

// The 4-connected sets of footprint cells in rect's cell range, in the order of their first cells, found by a
// depth-first walk of the test's own.
std::vector<Component> componentsIn(const Flags& cells, int cols, const BlockRect& rect) {
  const auto in = [&](int x, int y) {
    return x >= rect.firstColumn && x < rect.lastColumn && y >= rect.firstRow && y < rect.lastRow &&
           cellIn(cells, cols, x, y);
  };
  std::set<Cell> seen;
  std::vector<Component> components;
  for (int y = rect.firstRow; y < rect.lastRow; ++y) {
    for (int x = rect.firstColumn; x < rect.lastColumn; ++x) {
      if (!in(x, y) || seen.contains({x, y})) continue;
      Component component;
      component.first = {x, y};
      std::vector<Cell> pending{{x, y}};
      seen.insert({x, y});
      while (!pending.empty()) {
        const Cell at = pending.back();
        pending.pop_back();
        component.cells.insert(at);
        for (const Cell& step : kSides) {
          const Cell to{at.first + step.first, at.second + step.second};
          if (in(to.first, to.second) && seen.insert(to).second) pending.push_back(to);
        }
      }
      components.push_back(std::move(component));
    }
  }
  return components;
}

// The indices of loop where a run starts, by the header's rule: where the outline turns, and at a sample on a side of
// rect that is not on the grid's edge where the cells across that side, before and after the sample, differ.
Indices expectedRunStarts(const Loop& loop, const BlockRect& rect, const Flags& cells, int cols, int rows) {
  Indices starts;
  const std::size_t n = loop.size();
  for (std::size_t k = 0; k < n; ++k) {
    const GridPoint before = loop[(k + n - 1) % n], at = loop[k], after = loop[(k + 1) % n];
    bool start = at.x - before.x != after.x - at.x || at.y - before.y != after.y - at.y;
    if (!start && before.x == at.x) {
      // Along a column: across the east side is the column of cells lastColumn, across the west side firstColumn - 1.
      int across = -1;
      if (at.x == rect.lastColumn && rect.lastColumn < cols - 1) across = rect.lastColumn;
      if (at.x == rect.firstColumn && rect.firstColumn > 0) across = rect.firstColumn - 1;
      if (across >= 0) start = cellIn(cells, cols, across, at.y - 1) != cellIn(cells, cols, across, at.y);
    } else if (!start) {
      // Along a row: across the south side is the row of cells lastRow, across the north side firstRow - 1.
      int across = -1;
      if (at.y == rect.lastRow && rect.lastRow < rows - 1) across = rect.lastRow;
      if (at.y == rect.firstRow && rect.firstRow > 0) across = rect.firstRow - 1;
      if (across >= 0) start = cellIn(cells, cols, at.x - 1, across) != cellIn(cells, cols, at.x, across);
    }
    if (start) starts.push_back(static_cast<std::uint32_t>(k));
  }
  return starts;
}

// Says what is wrong with piece as the outline of component, a piece of the footprint inside rect, or returns "" when
// nothing is. The loop must: hold the component's count of cells; start at the north-west corner of the component's
// first cell; step one cell at a time along a grid line, with a cell of the component on its left and none on its
// right, seen from above (north up), so that it runs counter-clockwise; take in every side of the component's cells
// that has no cell of the component beyond it, so that it is the whole outline and there is no hole; visit no sample
// twice; turn negatively in (column, row) by orient2d four times more often than positively; enclose twice as much
// area as the cells' count by the shoelace formula with y north; and start its runs where the header says.
std::string outlineProblem(const Piece& piece, const Component& component, const BlockRect& rect, const Flags& cells,
                           int cols, int rows) {
  const Loop& loop = piece.loop;
  const std::size_t n = loop.size();
  const auto has = [&](int x, int y) { return component.cells.contains({x, y}); };
  if (piece.cells != component.cells.size()) {
    return "the piece has " + std::to_string(piece.cells) + " cells, not " + std::to_string(component.cells.size());
  }
  if (n < 4) return "the loop has " + std::to_string(n) + " samples";
  const GridPoint corner{component.first.first, component.first.second};
  if (!same(loop[0], corner)) {
    return "the loop starts at " + text(loop[0]) + ", not at the north-west corner of the first cell, " + text(corner);
  }

  std::set<std::pair<int, int>> visited;
  std::int64_t twiceAreaRowsDown = 0;
  int negativeTurns = 0, positiveTurns = 0;
  for (std::size_t k = 0; k < n; ++k) {
    const GridPoint p = loop[k], q = loop[(k + 1) % n];
    if (!visited.insert({p.x, p.y}).second) return "the loop visits " + text(p) + " twice";
    const int dx = q.x - p.x, dy = q.y - p.y;
    // The cell on the left of the step and the cell on its right, seen from above with north up (rows run south).
    Cell left, right;
    if (dx == 1 && dy == 0) {  // east: north on the left
      left = {p.x, p.y - 1};
      right = {p.x, p.y};
    } else if (dx == -1 && dy == 0) {  // west: south on the left
      left = {p.x - 1, p.y};
      right = {p.x - 1, p.y - 1};
    } else if (dx == 0 && dy == 1) {  // south: east on the left
      left = {p.x, p.y};
      right = {p.x - 1, p.y};
    } else if (dx == 0 && dy == -1) {  // north: west on the left
      left = {p.x - 1, p.y - 1};
      right = {p.x, p.y - 1};
    } else {
      return "the step from " + text(p) + " to " + text(q) + " is not one cell along a grid line";
    }
    if (!has(left.first, left.second) || has(right.first, right.second)) {
      return "the step from " + text(p) + " to " + text(q) + " does not have the piece on its left only";
    }
    twiceAreaRowsDown += std::int64_t{p.x} * q.y - std::int64_t{q.x} * p.y;
    const std::int64_t turn = orient2d(loop[(k + n - 1) % n], p, q);
    if (turn < 0) ++negativeTurns;
    if (turn > 0) ++positiveTurns;
  }
  std::size_t sides = 0;
  for (const auto& [x, y] : component.cells) {
    for (const Cell& step : kSides) sides += has(x + step.first, y + step.second) ? 0 : 1;
  }
  if (sides != n) {
    return "the component has " + std::to_string(sides) + " sides on its outline and the loop " + std::to_string(n);
  }
  if (negativeTurns - positiveTurns != 4) {
    return std::to_string(negativeTurns) + " negative turns and " + std::to_string(positiveTurns) + " positive";
  }
  // With y north, y = -row: the shoelace sum changes sign.
  const std::int64_t twiceArea = -twiceAreaRowsDown;
  if (twiceArea != 2 * static_cast<std::int64_t>(component.cells.size())) {
    return "twice the area seen from above is " + std::to_string(twiceArea) + ", not twice the " +
           std::to_string(component.cells.size()) + " cells";
  }
  const Indices expected = expectedRunStarts(loop, rect, cells, cols, rows);
  if (piece.runStarts != expected) {
    Loop wanted;
    for (const std::uint32_t k : expected) wanted.push_back(loop[k]);
    return "the runs start at " + text(runStartSamples(piece)) + ", not at " + text(wanted);
  }
  return {};
}

// Says what is wrong with f as blockFootprint's result for rect, or returns "" when nothing is: the kept pieces and the
// dropped ones are the components the test finds, split by area at minPiece and each in order of size, ties by the
// first cell; each is outlined as outlineProblem wants; the cells add up; and the corner contacts are the vertices
// strictly inside the block whose four cells are two kept pieces on one diagonal and not kept on the other.
std::string blockProblem(const BlockFootprint& f, const Flags& cells, int cols, int rows, const BlockRect& rect,
                         double cellArea, double minPiece) {
  std::vector<Component> components = componentsIn(cells, cols, rect);
  // Largest first; equal sizes keep the order of their first cells, which is the order the walk found them in.
  std::stable_sort(components.begin(), components.end(),
                   [](const Component& a, const Component& b) { return a.cells.size() > b.cells.size(); });
  std::vector<const Component*> kept, dropped;
  for (const Component& c : components) {
    (static_cast<double>(c.cells.size()) * cellArea < minPiece ? dropped : kept).push_back(&c);
  }
  if (f.pieces.size() != kept.size() || f.dropped.size() != dropped.size()) {
    return std::to_string(f.pieces.size()) + " pieces kept and " + std::to_string(f.dropped.size()) +
           " dropped, not " + std::to_string(kept.size()) + " and " + std::to_string(dropped.size());
  }
  std::uint64_t keptCells = 0;
  for (std::size_t i = 0; i < kept.size(); ++i) {
    const std::string problem = outlineProblem(f.pieces[i], *kept[i], rect, cells, cols, rows);
    if (!problem.empty()) return "kept piece " + std::to_string(i) + ": " + problem;
    keptCells += kept[i]->cells.size();
  }
  for (std::size_t i = 0; i < dropped.size(); ++i) {
    const std::string problem = outlineProblem(f.dropped[i], *dropped[i], rect, cells, cols, rows);
    if (!problem.empty()) return "dropped piece " + std::to_string(i) + ": " + problem;
  }
  if (f.cells != keptCells) return "cells is " + std::to_string(f.cells) + ", not " + std::to_string(keptCells);

  std::map<Cell, bool> keptAt;  // every footprint cell of the block: whether its piece is kept
  for (const Component* c : kept) {
    for (const Cell& cell : c->cells) keptAt[cell] = true;
  }
  for (const Component* c : dropped) {
    for (const Cell& cell : c->cells) keptAt[cell] = false;
  }
  const auto isKept = [&](int x, int y) {
    const auto it = keptAt.find({x, y});
    return it != keptAt.end() && it->second;
  };
  std::uint64_t contacts = 0;
  for (int y = rect.firstRow + 1; y < rect.lastRow; ++y) {
    for (int x = rect.firstColumn + 1; x < rect.lastColumn; ++x) {
      const bool nw = isKept(x - 1, y - 1), ne = isKept(x, y - 1), sw = isKept(x - 1, y), se = isKept(x, y);
      if ((nw && se && !ne && !sw) || (ne && sw && !nw && !se)) ++contacts;
    }
  }
  if (f.cornerContacts != contacts) {
    return std::to_string(f.cornerContacts) + " corner contacts, not " + std::to_string(contacts);
  }
  return {};
}

// For every sample on the loops of a block's pieces, kept and dropped: whether a run starts there.
std::map<std::pair<int, int>, bool> runStartsBySample(const BlockFootprint& f) {
  std::map<std::pair<int, int>, bool> starts;
  for (const std::vector<Piece>* pieces : {&f.pieces, &f.dropped}) {
    for (const Piece& piece : *pieces) {
      for (const GridPoint& p : piece.loop) starts[{p.x, p.y}] = false;
      for (const std::uint32_t k : piece.runStarts) starts[{piece.loop.at(k).x, piece.loop.at(k).y}] = true;
    }
  }
  return starts;
}

// Says what is wrong with the runs of two blocks along the side they share, or returns "" when nothing is. The side
// runs from `from` to `to` (a stretch of one column or one row); an edge of it is shared when the cells on both sides
// of it are in the footprint, so both blocks walk it. At every sample at the end of a shared edge both blocks must
// start a run, or neither: a run starts there exactly when the sample ends a stretch of shared edges, which is what
// lets both blocks simplify the shared stretches from the same samples.
std::string sharedSideProblem(const BlockFootprint& first, const BlockFootprint& second, GridPoint from, GridPoint to,
                              const Flags& cells, int cols) {
  const auto firstStarts = runStartsBySample(first), secondStarts = runStartsBySample(second);
  const bool alongColumn = from.x == to.x;
  const int length = alongColumn ? to.y - from.y : to.x - from.x;
  // Edge e runs from the sample e to the sample e + 1 along the side, counted from `from`.
  const auto shared = [&](int e) {
    if (e < 0 || e >= length) return false;
    if (alongColumn) return cellIn(cells, cols, from.x - 1, from.y + e) && cellIn(cells, cols, from.x, from.y + e);
    return cellIn(cells, cols, from.x + e, from.y - 1) && cellIn(cells, cols, from.x + e, from.y);
  };
  for (int s = 0; s <= length; ++s) {
    if (!shared(s - 1) && !shared(s)) continue;
    const std::pair<int, int> at = alongColumn ? std::pair{from.x, from.y + s} : std::pair{from.x + s, from.y};
    const auto a = firstStarts.find(at), b = secondStarts.find(at);
    const std::string where = "(" + std::to_string(at.first) + ", " + std::to_string(at.second) + ")";
    if (a == firstStarts.end() || b == secondStarts.end()) return where + " is not on both blocks' loops";
    const bool endsStretch = !shared(s - 1) || !shared(s);
    if (a->second != endsStretch || b->second != endsStretch) {
      return where + (endsStretch ? " ends a shared stretch" : " is inside a shared stretch") +
             ", and the blocks start a run there: " + (a->second ? "yes" : "no") + " and " + (b->second ? "yes" : "no");
    }
  }
  return {};
}

// A mask of cols x rows samples in one of three styles drawn for it: noise at a density drawn too; rectangles of data
// with rectangles of no-data cut out of them; or pairs of rectangles of data that share one corner sample, on either
// diagonal. So coasts, islands, lakes, channels and slits of every width turn up, and pieces that touch at a corner.
// Every draw is made from the engine's output by hand, since the standard fixes the engine's output and not a
// distribution's.
Flags randomMask(std::mt19937& rng, int cols, int rows) {
  Flags mask(static_cast<std::size_t>(cols * rows), 0);
  const auto draw = [&rng](int below) { return static_cast<int>(rng() % static_cast<unsigned>(below)); };
  // The samples of columns x0 to x0 + w - 1 and rows y0 to y0 + h - 1 that are on the grid.
  const auto fill = [&](int x0, int y0, int w, int h, std::uint8_t value) {
    for (int y = std::max(y0, 0); y < std::min(y0 + h, rows); ++y) {
      for (int x = std::max(x0, 0); x < std::min(x0 + w, cols); ++x) {
        mask[static_cast<std::size_t>(y * cols + x)] = value;
      }
    }
  };
  const int style = draw(3);
  if (style == 0) {
    const int percent = 40 + draw(58);  // 40 to 97 percent of the samples have data
    for (std::uint8_t& v : mask) v = draw(100) < percent ? 1 : 0;
  } else if (style == 1) {
    const int land = 1 + draw(5), water = draw(9), most = std::max(cols, rows);
    for (int i = 0; i < land; ++i) fill(draw(cols), draw(rows), 1 + draw(most), 1 + draw(most), 1);
    for (int i = 0; i < water; ++i) fill(draw(cols), draw(rows), 1 + draw(4), 1 + draw(4), 0);
  } else {
    const int pairs = 1 + draw(4);
    for (int i = 0; i < pairs; ++i) {
      const int x = draw(cols), y = draw(rows), a = 2 + draw(4), b = 2 + draw(4), c = 2 + draw(4), d = 2 + draw(4);
      fill(x, y, a, b, 1);
      // The second shares the first's south-east corner sample as its north-west one, or its south-west corner as
      // its north-east one.
      fill(draw(2) == 0 ? x + a - 1 : x - c + 1, y + b - 1, c, d, 1);
    }
  }
  return mask;
}

// The block lattice of blockCols x blockRows cells from the north-west corner of a grid of cols x rows vertices, the
// last blocks each way cut short, as rectangles row by row.
std::vector<std::vector<BlockRect>> lattice(int cols, int rows, int blockCols, int blockRows) {
  std::vector<std::vector<BlockRect>> blocks;
  for (int top = 0; top < rows - 1; top += blockRows) {
    blocks.emplace_back();
    for (int left = 0; left < cols - 1; left += blockCols) {
      blocks.back().push_back({left, top, std::min(left + blockCols, cols - 1), std::min(top + blockRows, rows - 1)});
    }
  }
  return blocks;
}

// A lake of 2 x 2 no-data samples in a field of 10 x 7, joined to the sea at the north by a channel `width` samples
// wide: 14 x 11 samples.
Rows lakeWithChannel(int width) {
  Rows rows(11, std::string(14, '.'));
  for (int y = 2; y <= 8; ++y) {
    for (int x = 2; x <= 11; ++x) rows[y][x] = '#';
  }
  for (int y = 5; y <= 6; ++y) {
    for (int x = 6; x <= 7; ++x) rows[y][x] = '.';
  }
  for (int y = 2; y <= 4; ++y) {
    for (int x = 6; x < 6 + width; ++x) rows[y][x] = '.';
  }
  return rows;
}

}  // namespace

TEST_CASE("dataMaskOf gives 1 for a height and 0 for no-data", "[footprint]") {
  // A height of 0 m is data like any other: the file's GDAL_NODATA is NaN by the time a window has been read.
  CHECK(dataMaskOf({12.5f, kNaN, 0.0f, -37.9f, kNaN}) == Flags{1, 0, 1, 1, 0});
  // readWindow never gives an infinite height, and no terrain has one: it counts as no data.
  CHECK(dataMaskOf({kInfinity, -kInfinity, 1590.1f}) == Flags{0, 0, 1});
  CHECK(dataMaskOf({}).empty());
}

TEST_CASE("resampleMaskNearest takes each output vertex from the nearest source sample", "[footprint]") {
  // 4 x 3 samples:
  //   1 0 1 0
  //   0 1 1 0
  //   0 1 0 1
  const Flags mask = valuesOf({"#.#.", ".##.", ".#.#"});

  SECTION("to 7 x 5") {
    // Output column i stands at i (4 - 1) / (7 - 1) = i / 2: 0, 0.5, 1, 1.5, 2, 2.5, 3, and a half goes to the lower
    // sample, so the columns come from 0, 0, 1, 1, 2, 2, 3. Output row j stands at j (3 - 1) / (5 - 1) = j / 2, so the
    // rows come from 0, 0, 1, 1, 2. The corners are the source's corners, and the middle, (3, 2), stands at (1.5, 1)
    // and takes the sample (1, 1). Where a half could have gone either way the two samples differ: (1, 0) is 1 from
    // (0, 0) and not 0 from (1, 0); (0, 1) is 1 from (0, 0) and not 0 from (0, 1); (4, 3) is 1 from (2, 1) and not 0
    // from (2, 2).
    const Flags out = resampleMaskNearest(mask, 4, 3, 7, 5);
    CHECK(pictureOf(out, 7) == Rows{"##..##.", "##..##.", "..####.", "..####.", "..##..#"});
    CHECK(out[0] == 1);           // north-west corner, from (0, 0)
    CHECK(out[6] == 0);           // north-east corner, from (3, 0)
    CHECK(out[4 * 7] == 0);       // south-west corner, from (0, 2)
    CHECK(out[4 * 7 + 6] == 1);   // south-east corner, from (3, 2)
    CHECK(out[2 * 7 + 3] == 1);   // the middle, from (1, 1)
  }

  SECTION("to 2 x 2, the corners") {
    // Column i stands at 3 i and row j at 2 j: the four corners of the source.
    CHECK(resampleMaskNearest(mask, 4, 3, 2, 2) == Flags{1, 0, 0, 1});
  }

  SECTION("past half way goes to the next sample") {
    // 6 samples to 4: output i stands at 5 i / 3 = 0, 1.67, 3.33, 5, the nearest samples 0, 2, 3, 5. Truncating would
    // take 0, 1, 3, 5 instead, which this row tells apart.
    CHECK(resampleMaskNearest(valuesOf({".#.##.", ".#.##."}), 6, 2, 4, 2) == Flags{0, 0, 1, 0, 0, 0, 1, 0});
  }

  SECTION("the source's own size gives the source") {
    CHECK(resampleMaskNearest(mask, 4, 3, 4, 3) == mask);
  }

  SECTION("refusals") {
    CHECK_THROWS_MATCHES(resampleMaskNearest(Flags{1, 0}, 1, 2, 4, 4), std::invalid_argument,
                         Message("the source must be at least 2 x 2, not 1 x 2"));
    CHECK_THROWS_MATCHES(resampleMaskNearest(mask, 4, 4, 7, 5), std::invalid_argument,
                         Message("a source of 4 x 4 samples needs 16 mask values, not 12"));
    CHECK_THROWS_MATCHES(resampleMaskNearest(mask, 4, 3, 7, 1), std::invalid_argument,
                         Message("the output grid must be at least 2 x 2, not 7 x 1"));
  }
}

TEST_CASE("the island's footprint: the lake and the channel are in, the sea and the lone sample out", "[footprint]") {
  // Cells with all four corners in the data first: the island's 9 x 9 samples give 8 x 8 cells, columns 4 to 11 and
  // rows 2 to 9, and the tab's samples with those of column 12 give the cells of columns 12 and 13, rows 4 to 7. A
  // no-data sample is a corner of the four cells around it, so the lake's four samples leave out the 3 x 3 cells of
  // columns 6 to 8 and rows 4 to 6, and the channel's three samples on row 5 those of columns 3 to 6 and rows 4 and 5:
  // an opening from the sea to the lake two cells tall, a slit.
  //
  // The closing then fills the slit: each of its cells (4, 4), (5, 4), (4, 5) and (5, 5) has a cell of the island in
  // the 3 x 3 square around it (row 3 or row 6 of the cells), so the dilation turns it in; and every cell of the square
  // around it is then in, so the erosion leaves it in. The lake's middle cell (7, 5) has nothing in its square, so the
  // dilation leaves it out, and the erosion takes out again every cell whose square holds it: the lake's 3 x 3 cells.
  // The sea stays out the same way, and so do the island's steps, (12, 3) and (12, 8) for instance: the square around
  // each holds a cell, (13, 2) and (13, 9), that has no cell in its own. The lake is then enclosed, out cells that no
  // walk through out cells reaches from the grid's edge, and is turned in.
  //
  // The islet's 3 x 3 samples give 2 x 2 cells, columns 4 and 5 and rows 13 and 14, and each blob 2 x 2 cells; the
  // lone sample at (18, 13) is a corner of the cells (17, 12), (18, 12), (17, 13) and (18, 13), none of whose other
  // corners has data, so it makes no cell. The closing joins neither the blobs, which meet at the vertex (19, 4) only
  // (the cell (19, 3) between them has the cell (20, 2) in its square, with nothing in its own square), nor anything
  // across the gaps of three cells.
  const Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows);
  REQUIRE(cells.size() == static_cast<std::size_t>((kIslandCols - 1) * (kIslandRows - 1)));
  CHECK(pictureOf(cells, kIslandCols - 1) == islandFootprint());

  // The same, cell by cell, for the claims of the case's name.
  CHECK(cellIn(cells, kIslandCols, 7, 5));   // the lake's middle cell, enclosed and filled
  CHECK(cellIn(cells, kIslandCols, 6, 4));   // a lake cell beside the channel
  CHECK(cellIn(cells, kIslandCols, 4, 4));   // the channel's cells, closed as a slit
  CHECK(cellIn(cells, kIslandCols, 5, 5));
  CHECK_FALSE(cellIn(cells, kIslandCols, 3, 4));   // the sea beyond the channel's mouth
  CHECK_FALSE(cellIn(cells, kIslandCols, 0, 0));   // the sea at the grid's corner
  CHECK_FALSE(cellIn(cells, kIslandCols, 17, 12));  // the lone sample's four cells
  CHECK_FALSE(cellIn(cells, kIslandCols, 18, 12));
  CHECK_FALSE(cellIn(cells, kIslandCols, 17, 13));
  CHECK_FALSE(cellIn(cells, kIslandCols, 18, 13));
  int islet = 0;
  for (int y = 11; y <= 16; ++y) {
    for (int x = 0; x <= 10; ++x) islet += cellIn(cells, kIslandCols, x, y) ? 1 : 0;
  }
  CHECK(islet == 4);
  CHECK(std::count(cells.begin(), cells.end(), 1) == 72 + 4 + 4 + 4);
}

TEST_CASE("a channel one sample wide is closed and a wider one stays open", "[footprint]") {
  // A field of 10 x 7 samples, columns 2 to 11 and rows 2 to 8, gives 9 x 6 cells. The lake's samples, columns 6 and 7
  // and rows 5 and 6, leave out the cells of columns 5 to 7 and rows 4 to 6, and a channel from it to the north coast
  // the cells of rows 1 to 4 that its samples are corners of.
  SECTION("one sample wide: two cells, closed, and the lake enclosed and filled") {
    // The channel, column 6 and rows 2 to 4, leaves out the cells of columns 5 and 6: a slit two cells wide.
    const Rows mask = lakeWithChannel(1);
    const Flags cells = footprintCells(valuesOf(mask), 14, 11);
    CHECK(pictureOf(cells, 13) == Rows{".............", ".............", "..#########..", "..#########..",
                                       "..#########..", "..#########..", "..#########..", "..#########..",
                                       ".............", "............."});
  }

  SECTION("two samples wide: three cells, open, and the lake joined to the sea") {
    // Columns 6 and 7 leave out the cells of columns 5 to 7, three wide. The dilation turns in the two cells at the
    // sides but not the one in the middle, whose square holds no cell, and the erosion then takes the sides out again:
    // the channel and the lake stay out, 3 x 5 cells, columns 5 to 7 and rows 2 to 6.
    const Flags cells = footprintCells(valuesOf(lakeWithChannel(2)), 14, 11);
    CHECK(pictureOf(cells, 13) == Rows{".............", ".............", "..###...###..", "..###...###..",
                                       "..###...###..", "..###...###..", "..###...###..", "..#########..",
                                       ".............", "............."});
    CHECK_FALSE(cellIn(cells, 14, 6, 5));  // the lake
  }

  SECTION("three samples wide: four cells, open") {
    const Flags cells = footprintCells(valuesOf(lakeWithChannel(3)), 14, 11);
    CHECK(pictureOf(cells, 13) == Rows{".............", ".............", "..###....##..", "..###....##..",
                                       "..###....##..", "..###...###..", "..###...###..", "..#########..",
                                       ".............", "............."});
    CHECK_FALSE(cellIn(cells, 14, 6, 5));
  }
}

TEST_CASE("a grid all of data is all in and a grid without data has no cell", "[footprint]") {
  // The erosion counts what lies beyond the grid as in, so the cells along the edge are kept.
  const Flags all = footprintCells(Flags(7 * 5, 1), 7, 5);
  CHECK(all == Flags(6 * 4, 1));
  CHECK(footprintCells(Flags(7 * 5, 0), 7, 5) == Flags(6 * 4, 0));
  // The smallest grid, one cell: in with four data corners, out with one missing.
  CHECK(footprintCells(Flags{1, 1, 1, 1}, 2, 2) == Flags{1});
  CHECK(footprintCells(Flags{1, 1, 0, 1}, 2, 2) == Flags{0});
}

TEST_CASE("the closing treats the world beyond the grid as data", "[footprint]") {
  // A column of no-data samples on the grid's west edge leaves out the one column of cells it is a corner of. Between
  // the data and the edge, which counts as in, that is a slit one cell wide, and the closing fills it. A column of
  // no-data one sample further in leaves out two columns of cells, which stay out, as two columns at the edge do.
  CHECK(footprintCells(valuesOf(Rows(6, ".#######")), 8, 6) == Flags(7 * 5, 1));
  CHECK(pictureOf(footprintCells(valuesOf(Rows(6, "#.######")), 8, 6), 7) == Rows(5, "..#####"));
  CHECK(pictureOf(footprintCells(valuesOf(Rows(6, "..######")), 8, 6), 7) == Rows(5, "..#####"));
  // So does a notch of any width one cell deep in the edge row, as no-data samples on the edge itself make: the
  // dilation fills it from the row inside, and the erosion finds every cell around it in, the grid's edge included.
  CHECK(footprintCells(valuesOf({"#...#####", "#########", "#########"}), 9, 3) == Flags(8 * 2, 1));
}

TEST_CASE("footprintCells follows its rules on random masks", "[footprint]") {
  // The unit closes in place along the rows and then down the columns and finds the sea by waves; the reference steps
  // look at every 3 x 3 square on grids of their own and walk depth first. They must agree cell for cell.
  std::mt19937 rng(kRandomSeed);
  int changedByClosing = 0, changedByFill = 0, empty = 0, full = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const int cols = 2 + static_cast<int>(rng() % 39), rows = 2 + static_cast<int>(rng() % 39);
    const Flags mask = randomMask(rng, cols, rows);
    INFO("trial " << trial << ", " << cols << " x " << rows);
    const Plane fourCorners = referenceFourCorners(mask, cols, rows);
    const Plane closed = referenceClosing(fourCorners);
    const Plane filled = referenceEnclosedFilled(closed);
    const Flags cells = footprintCells(mask, cols, rows);
    CHECK(pictureOf(cells, cols - 1) == pictureOf(flagsOf(filled), cols - 1));

    // How often each step made a difference, so that the sample is seen not to be one-sided.
    if (closed != fourCorners) ++changedByClosing;
    if (filled != closed) ++changedByFill;
    if (std::count(cells.begin(), cells.end(), 1) == 0) ++empty;
    if (std::count(cells.begin(), cells.end(), 0) == 0) ++full;
  }
  CHECK(changedByClosing > 130);
  CHECK(changedByFill > 20);
  CHECK(empty > 10);
  CHECK(full > 7);
}

TEST_CASE("footprintCells refuses a grid it cannot have cells for", "[footprint]") {
  CHECK_THROWS_MATCHES(footprintCells(Flags{1, 1}, 1, 2), std::invalid_argument,
                       Message("the grid must be at least 2 x 2 vertices, not 1 x 2"));
  CHECK_THROWS_MATCHES(footprintCells(Flags{1, 1, 1}, 3, 1), std::invalid_argument,
                       Message("the grid must be at least 2 x 2 vertices, not 3 x 1"));
  CHECK_THROWS_MATCHES(footprintCells(Flags(11, 1), 4, 3), std::invalid_argument,
                       Message("a grid of 4 x 3 vertices needs 12 mask values, not 11"));
}

TEST_CASE("the island's pieces over the whole grid, their outlines and their runs", "[footprint]") {
  const Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows);
  const BlockRect whole{0, 0, kIslandCols - 1, kIslandRows - 1};
  // Cells of a quarter of a square unit, so the islet's 4 cells are 1 square unit; nothing is dropped below 0.
  const BlockFootprint f = blockFootprint(cells, kIslandCols, kIslandRows, whole, 0.25, 0.0);
  const std::string problem = blockProblem(f, cells, kIslandCols, kIslandRows, whole, 0.25, 0.0);
  INFO("problem: " << problem);
  CHECK(problem.empty());

  // The island with its lake and channel, 8 x 8 + 2 x 4 = 72 cells, then the three of 4 cells by their north-west-most
  // cells: blob A's (17, 2) on row 2, blob B's (19, 4) on row 4, the islet's (4, 13) on row 13.
  REQUIRE(f.pieces.size() == 4);
  CHECK(f.dropped.empty());
  CHECK(f.cells == 84);
  CHECK(f.pieces[0].cells == 72);
  CHECK(f.pieces[1].cells == 4);
  CHECK(f.pieces[2].cells == 4);
  CHECK(f.pieces[3].cells == 4);

  // The island's loop starts at the north-west corner of its first cell, (4, 2), and goes south down its west coast,
  // the lake and channel being filled: counter-clockwise seen from above, with the island on its left. Every sample on
  // the outline is in it, 2 x 8 + 2 x 8 + 2 x 2 = 36 of them. The grid's edge is far away, so the runs start at the
  // eight turns and nowhere else.
  CHECK(text(f.pieces[0].loop) ==
        "(4, 2) (4, 3) (4, 4) (4, 5) (4, 6) (4, 7) (4, 8) (4, 9) (4, 10) (5, 10) (6, 10) (7, 10) (8, 10) (9, 10) "
        "(10, 10) (11, 10) (12, 10) (12, 9) (12, 8) (13, 8) (14, 8) (14, 7) (14, 6) (14, 5) (14, 4) (13, 4) (12, 4) "
        "(12, 3) (12, 2) (11, 2) (10, 2) (9, 2) (8, 2) (7, 2) (6, 2) (5, 2)");
  CHECK(f.pieces[0].runStarts == Indices{0, 8, 16, 18, 20, 24, 26, 28});
  CHECK(text(runStartSamples(f.pieces[0])) == "(4, 2) (4, 10) (12, 10) (12, 8) (14, 8) (14, 4) (12, 4) (12, 2)");

  // The blobs, each a square of 2 x 2 cells: eight samples and four runs. Both loops hold the vertex (19, 4) where
  // they touch, blob A's fifth sample and blob B's first: blob A's walk, coming east along its south side to (19, 4),
  // turns north up its own east side rather than going on along blob B's west side.
  CHECK(text(f.pieces[1].loop) == "(17, 2) (17, 3) (17, 4) (18, 4) (19, 4) (19, 3) (19, 2) (18, 2)");
  CHECK(f.pieces[1].runStarts == Indices{0, 2, 4, 6});
  CHECK(text(f.pieces[2].loop) == "(19, 4) (19, 5) (19, 6) (20, 6) (21, 6) (21, 5) (21, 4) (20, 4)");
  CHECK(f.pieces[2].runStarts == Indices{0, 2, 4, 6});
  CHECK(text(f.pieces[3].loop) == "(4, 13) (4, 14) (4, 15) (5, 15) (6, 15) (6, 14) (6, 13) (5, 13)");
  CHECK(f.pieces[3].runStarts == Indices{0, 2, 4, 6});

  // The vertex (19, 4) has blob A's cell (18, 3) to its north-west and blob B's (19, 4) to its south-east, and the
  // cells (19, 3) and (18, 4) on the other diagonal are not in the footprint.
  CHECK(f.cornerContacts == 1);

  // Each loop, seen from above with y north, turns counter-clockwise: its shoelace area is positive, twice its cells.
  for (const Piece& piece : f.pieces) {
    std::int64_t twice = 0;
    for (std::size_t k = 0; k < piece.loop.size(); ++k) {
      const GridPoint a = piece.loop[k], b = piece.loop[(k + 1) % piece.loop.size()];
      twice += std::int64_t{a.x} * -b.y - std::int64_t{b.x} * -a.y;
    }
    CHECK(twice == 2 * static_cast<std::int64_t>(piece.cells));
  }
}

TEST_CASE("pieces smaller than the least area are dropped, and a contact counts only between kept pieces",
          "[footprint]") {
  const Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows);
  const BlockRect whole{0, 0, kIslandCols - 1, kIslandRows - 1};

  SECTION("an area equal to the islet's keeps it: below means below") {
    const BlockFootprint f = blockFootprint(cells, kIslandCols, kIslandRows, whole, 0.25, 1.0);
    CHECK(f.pieces.size() == 4);
    CHECK(f.dropped.empty());
    CHECK(f.cornerContacts == 1);
  }

  SECTION("just above the islet's area drops it, and the blobs of the same area with it") {
    const double least = std::nextafter(1.0, 2.0);
    const BlockFootprint f = blockFootprint(cells, kIslandCols, kIslandRows, whole, 0.25, least);
    const std::string problem = blockProblem(f, cells, kIslandCols, kIslandRows, whole, 0.25, least);
    INFO("problem: " << problem);
    CHECK(problem.empty());
    REQUIRE(f.pieces.size() == 1);
    CHECK(f.pieces[0].cells == 72);
    // The dropped pieces in the same order as kept ones: by area, then by their north-west-most cells.
    REQUIRE(f.dropped.size() == 3);
    CHECK(same(f.dropped[0].loop.at(0), {17, 2}));
    CHECK(same(f.dropped[1].loop.at(0), {19, 4}));
    CHECK(same(f.dropped[2].loop.at(0), {4, 13}));
    CHECK(f.dropped[2].cells == 4);
    // Dropped pieces are outlined all the same, so that they can be listed.
    CHECK(text(f.dropped[2].loop) == "(4, 13) (4, 14) (4, 15) (5, 15) (6, 15) (6, 14) (6, 13) (5, 13)");
    // Only the kept island's cells count, and the blobs, dropped, touch no kept piece.
    CHECK(f.cells == 72);
    CHECK(f.cornerContacts == 0);
  }
}

TEST_CASE("a block side splits the runs where the cells across it change, as the block beyond splits them",
          "[footprint]") {
  const Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows);
  // The west half, columns 0 to 12, and the east half, columns 12 to 23 of the vertices: they share column 12.
  const BlockRect west{0, 0, 12, kIslandRows - 1};
  const BlockRect east{12, 0, kIslandCols - 1, kIslandRows - 1};
  const BlockFootprint w = blockFootprint(cells, kIslandCols, kIslandRows, west, 1.0, 0.0);
  const BlockFootprint e = blockFootprint(cells, kIslandCols, kIslandRows, east, 1.0, 0.0);
  const std::string westProblem = blockProblem(w, cells, kIslandCols, kIslandRows, west, 1.0, 0.0);
  const std::string eastProblem = blockProblem(e, cells, kIslandCols, kIslandRows, east, 1.0, 0.0);
  INFO("west: " << westProblem);
  INFO("east: " << eastProblem);
  CHECK(westProblem.empty());
  CHECK(eastProblem.empty());

  // The west half holds the island without its tab, 8 x 8 cells, and the islet.
  REQUIRE(w.pieces.size() == 2);
  const Piece& island = w.pieces[0];
  CHECK(island.cells == 64);
  // Its east coast is the block's east side, column 12, which the loop walks north from (12, 10) to (12, 2). Across
  // it are the tab's cells of column 12, in on rows 4 to 7 and out on rows 3 and 8: the status changes at the samples
  // (12, 8) and (12, 4), and a run starts at each though the loop goes straight on. So the side is three runs: from
  // (12, 10) to (12, 8) beside the sea, from (12, 8) to (12, 4) beside the tab, and from (12, 4) to (12, 2) beside the
  // sea again.
  CHECK(text(runStartSamples(island)) == "(4, 2) (4, 10) (12, 10) (12, 8) (12, 4) (12, 2)");
  CHECK(island.runStarts == Indices{0, 8, 16, 18, 22, 24});

  // The east half holds the tab, 2 x 4 cells, and the blobs. The tab's west side is the same column 12, walked south
  // from (12, 4) to (12, 8) beside the island's cells, which are in all the way: one run, between the same two samples
  // as the west half's middle run on that line.
  REQUIRE(e.pieces.size() == 3);
  const Piece& tab = e.pieces[0];
  CHECK(tab.cells == 8);
  CHECK(text(tab.loop) ==
        "(12, 4) (12, 5) (12, 6) (12, 7) (12, 8) (13, 8) (14, 8) (14, 7) (14, 6) (14, 5) (14, 4) (13, 4)");
  CHECK(text(runStartSamples(tab)) == "(12, 4) (12, 8) (14, 8) (14, 4)");
  // The blobs' contact is inside the east half.
  CHECK(e.cornerContacts == 1);
  CHECK(w.cornerContacts == 0);

  // The two blocks agree along the whole of their shared side.
  const std::string sharedProblem = sharedSideProblem(w, e, {12, 0}, {12, kIslandRows - 1}, cells, kIslandCols);
  INFO("shared side: " << sharedProblem);
  CHECK(sharedProblem.empty());
}

TEST_CASE("an outline along the grid's edge has runs that end at its turns only", "[footprint]") {
  // A grid of 8 x 6 samples, all data but the four samples of columns 3 and 4 and rows 0 and 1, on the north edge: the
  // cells of columns 2 to 4 and rows 0 and 1 are out, a notch three cells wide and two deep, too wide for the closing.
  // The whole grid is one piece of 35 - 6 = 29 cells whose loop runs along the grid's edge, where there is nothing
  // across to split a run at, so its runs start at its eight turns: the grid's four corners and the notch's four.
  const Rows mask{"###..###", "###..###", "########", "########", "########", "########"};
  const Flags cells = footprintCells(valuesOf(mask), 8, 6);
  REQUIRE(pictureOf(cells, 7) == Rows{"##...##", "##...##", "#######", "#######", "#######"});
  const BlockRect whole{0, 0, 7, 5};
  const BlockFootprint f = blockFootprint(cells, 8, 6, whole, 1.0, 0.0);
  const std::string problem = blockProblem(f, cells, 8, 6, whole, 1.0, 0.0);
  INFO("whole: " << problem);
  CHECK(problem.empty());
  REQUIRE(f.pieces.size() == 1);
  CHECK(f.pieces[0].cells == 29);
  CHECK(f.pieces[0].loop.size() == 28);
  CHECK(text(runStartSamples(f.pieces[0])) == "(0, 0) (0, 5) (7, 5) (7, 0) (5, 0) (5, 2) (2, 2) (2, 0)");

  // A block inside the grid, columns 1 to 6 and rows 3 and 4: its sides are not the grid's edge, and across each of
  // them the cells are in all along, so its runs start only at its four corners.
  const BlockRect inner{1, 3, 6, 4};
  const BlockFootprint g = blockFootprint(cells, 8, 6, inner, 1.0, 0.0);
  const std::string innerProblem = blockProblem(g, cells, 8, 6, inner, 1.0, 0.0);
  INFO("inner: " << innerProblem);
  CHECK(innerProblem.empty());
  REQUIRE(g.pieces.size() == 1);
  CHECK(text(runStartSamples(g.pieces[0])) == "(1, 3) (1, 4) (6, 4) (6, 3)");

  // The block of columns 1 to 6 and rows 2 to 4 has the notch across its north side, which is not the grid's edge:
  // the cells across it are in at column 1, out at columns 2 to 4 and in at column 5, so the run along that side,
  // walked west from (6, 2) to (1, 2), splits at (5, 2) and (2, 2), where they change.
  const BlockRect below{1, 2, 6, 4};
  const BlockFootprint h = blockFootprint(cells, 8, 6, below, 1.0, 0.0);
  const std::string belowProblem = blockProblem(h, cells, 8, 6, below, 1.0, 0.0);
  INFO("below: " << belowProblem);
  CHECK(belowProblem.empty());
  REQUIRE(h.pieces.size() == 1);
  CHECK(text(runStartSamples(h.pieces[0])) == "(1, 2) (1, 4) (6, 4) (6, 2) (5, 2) (2, 2)");
}

TEST_CASE("a corner contact on a block side is counted by neither block", "[footprint]") {
  // Blocks that meet down column 19, through the vertex (19, 4) where the blobs touch: blob A is in the west block and
  // blob B in the east one, so each block sees one cell of the diagonal and neither counts the contact.
  const Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows);
  const BlockRect west{12, 0, 19, kIslandRows - 1};
  const BlockRect east{19, 0, kIslandCols - 1, kIslandRows - 1};
  const BlockFootprint w = blockFootprint(cells, kIslandCols, kIslandRows, west, 1.0, 0.0);
  const BlockFootprint e = blockFootprint(cells, kIslandCols, kIslandRows, east, 1.0, 0.0);
  const std::string westProblem = blockProblem(w, cells, kIslandCols, kIslandRows, west, 1.0, 0.0);
  const std::string eastProblem = blockProblem(e, cells, kIslandCols, kIslandRows, east, 1.0, 0.0);
  INFO("west: " << westProblem);
  INFO("east: " << eastProblem);
  CHECK(westProblem.empty());
  CHECK(eastProblem.empty());
  REQUIRE(w.pieces.size() == 2);  // the tab and blob A
  REQUIRE(e.pieces.size() == 1);  // blob B
  CHECK(w.cornerContacts == 0);
  CHECK(e.cornerContacts == 0);
  // Both blocks' loops hold the vertex, as a turn.
  CHECK(text(runStartSamples(w.pieces[1])) == "(17, 2) (17, 4) (19, 4) (19, 2)");
  CHECK(text(runStartSamples(e.pieces[0])) == "(19, 4) (19, 6) (21, 6) (21, 4)");
}

TEST_CASE("random footprints cut into blocks are outlined by the rules and agree along shared sides", "[footprint]") {
  // Random masks, their footprints, and a lattice of blocks over each with a random size, the last blocks each way
  // cut short. Every block is held to blockProblem, and every side two blocks share to sharedSideProblem.
  std::mt19937 rng(kRandomSeed + 1);
  int blocksWithPieces = 0, dropped = 0, contacts = 0, wholeGridContacts = 0, splits = 0, sharedStretches = 0;
  for (int trial = 0; trial < 300; ++trial) {
    const int cols = 3 + static_cast<int>(rng() % 30), rows = 3 + static_cast<int>(rng() % 30);
    const Flags mask = randomMask(rng, cols, rows);
    const Flags cells = footprintCells(mask, cols, rows);
    const int blockCols = 1 + static_cast<int>(rng() % static_cast<unsigned>(cols - 1));
    const int blockRows = 1 + static_cast<int>(rng() % static_cast<unsigned>(rows - 1));
    const double least = static_cast<double>(rng() % 8);  // cells of 1 square unit: pieces under 0 to 7 cells drop
    const auto blocks = lattice(cols, rows, blockCols, blockRows);

    // The whole grid as one block, nothing dropped: every corner contact is inside it and counts.
    {
      const BlockRect whole{0, 0, cols - 1, rows - 1};
      const BlockFootprint f = blockFootprint(cells, cols, rows, whole, 1.0, 0.0);
      const std::string problem = blockProblem(f, cells, cols, rows, whole, 1.0, 0.0);
      INFO("trial " << trial << ", " << cols << " x " << rows << ", the whole grid: " << problem);
      CHECK(problem.empty());
      wholeGridContacts += static_cast<int>(f.cornerContacts);
    }

    std::vector<std::vector<BlockFootprint>> results;
    for (const auto& row : blocks) {
      results.emplace_back();
      for (const BlockRect& rect : row) {
        INFO("trial " << trial << ", " << cols << " x " << rows << ", block of " << text(rect));
        results.back().push_back(blockFootprint(cells, cols, rows, rect, 1.0, least));
        const BlockFootprint& f = results.back().back();
        const std::string problem = blockProblem(f, cells, cols, rows, rect, 1.0, least);
        INFO("problem: " << problem);
        CHECK(problem.empty());
        if (!f.pieces.empty()) ++blocksWithPieces;
        dropped += static_cast<int>(f.dropped.size());
        contacts += static_cast<int>(f.cornerContacts);
        for (const Piece& piece : f.pieces) {
          for (const std::uint32_t k : piece.runStarts) {
            const std::size_t n = piece.loop.size();
            const GridPoint a = piece.loop[(k + n - 1) % n], b = piece.loop.at(k), c = piece.loop[(k + 1) % n];
            if (b.x - a.x == c.x - b.x && b.y - a.y == c.y - b.y) ++splits;  // a run start that is not a turn
          }
        }
      }
    }

    for (std::size_t r = 0; r < blocks.size(); ++r) {
      for (std::size_t c = 0; c < blocks[r].size(); ++c) {
        const BlockRect& rect = blocks[r][c];
        INFO("trial " << trial << ", block of " << text(rect));
        if (c + 1 < blocks[r].size()) {
          const GridPoint from{rect.lastColumn, rect.firstRow}, to{rect.lastColumn, rect.lastRow};
          const std::string problem = sharedSideProblem(results[r][c], results[r][c + 1], from, to, cells, cols);
          INFO("east side: " << problem);
          CHECK(problem.empty());
          for (int y = rect.firstRow; y < rect.lastRow; ++y) {
            if (cellIn(cells, cols, rect.lastColumn - 1, y) && cellIn(cells, cols, rect.lastColumn, y) &&
                (y == rect.firstRow || !cellIn(cells, cols, rect.lastColumn - 1, y - 1) ||
                 !cellIn(cells, cols, rect.lastColumn, y - 1))) {
              ++sharedStretches;
            }
          }
        }
        if (r + 1 < blocks.size()) {
          const GridPoint from{rect.firstColumn, rect.lastRow}, to{rect.lastColumn, rect.lastRow};
          const std::string problem = sharedSideProblem(results[r][c], results[r + 1][c], from, to, cells, cols);
          INFO("south side: " << problem);
          CHECK(problem.empty());
        }
      }
    }
  }
  // The sample is not one-sided: pieces, drops, corner contacts, runs split on block sides and stretches that two
  // blocks share all turn up, many times.
  CHECK(blocksWithPieces > 600);
  CHECK(dropped > 400);
  CHECK(contacts > 10);
  CHECK(wholeGridContacts > 50);
  CHECK(splits > 100);
  CHECK(sharedStretches > 500);
}

TEST_CASE("the same footprint gives the same pieces every time", "[footprint]") {
  const Flags first = footprintCells(islandMask(), kIslandCols, kIslandRows);
  const Flags second = footprintCells(islandMask(), kIslandCols, kIslandRows);
  CHECK(first == second);
  const BlockRect west{0, 0, 12, kIslandRows - 1};
  const BlockFootprint a = blockFootprint(first, kIslandCols, kIslandRows, west, 0.25, 0.5);
  const BlockFootprint b = blockFootprint(first, kIslandCols, kIslandRows, west, 0.25, 0.5);
  REQUIRE(a.pieces.size() == b.pieces.size());
  for (std::size_t i = 0; i < a.pieces.size(); ++i) {
    CHECK(text(a.pieces[i].loop) == text(b.pieces[i].loop));
    CHECK(a.pieces[i].runStarts == b.pieces[i].runStarts);
    CHECK(a.pieces[i].cells == b.pieces[i].cells);
  }
  CHECK(a.dropped.size() == b.dropped.size());
  CHECK(a.cells == b.cells);
  CHECK(a.cornerContacts == b.cornerContacts);
}

TEST_CASE("blockFootprint refuses a block it cannot outline", "[footprint]") {
  const Flags cells = footprintCells(islandMask(), kIslandCols, kIslandRows);
  const auto outline = [&](BlockRect rect) { return blockFootprint(cells, kIslandCols, kIslandRows, rect, 1.0, 0.0); };

  SECTION("a block outside the grid") {
    CHECK_THROWS_MATCHES(outline({0, 0, 24, 17}), std::invalid_argument,
                         Message("the block of columns 0 to 24 and rows 0 to 17 is not inside the grid of 24 x 18 "
                                 "vertices"));
    CHECK_THROWS_MATCHES(outline({-1, 0, 12, 17}), std::invalid_argument,
                         Message("the block of columns -1 to 12 and rows 0 to 17 is not inside the grid of 24 x 18 "
                                 "vertices"));
    CHECK_THROWS_MATCHES(outline({0, 0, 12, 18}), std::invalid_argument,
                         Message("the block of columns 0 to 12 and rows 0 to 18 is not inside the grid of 24 x 18 "
                                 "vertices"));
  }

  SECTION("a block of one column or one row of vertices, which has no cell") {
    CHECK_THROWS_MATCHES(outline({5, 0, 5, 17}), std::invalid_argument,
                         Message("the block of columns 5 to 5 and rows 0 to 17 has no cell"));
    CHECK_THROWS_MATCHES(outline({0, 9, 23, 9}), std::invalid_argument,
                         Message("the block of columns 0 to 23 and rows 9 to 9 has no cell"));
    CHECK_THROWS_MATCHES(outline({12, 0, 4, 17}), std::invalid_argument,
                         Message("the block of columns 12 to 4 and rows 0 to 17 has no cell"));
  }

  SECTION("cells that are not the grid's") {
    CHECK_THROWS_MATCHES(blockFootprint(cells, 1, 18, {0, 0, 0, 17}, 1.0, 0.0), std::invalid_argument,
                         Message("the grid must be at least 2 x 2 vertices, not 1 x 18"));
    CHECK_THROWS_MATCHES(blockFootprint(cells, 23, 18, {0, 0, 12, 17}, 1.0, 0.0), std::invalid_argument,
                         Message("a grid of 23 x 18 vertices has 374 cells, not 391"));
  }

  SECTION("an area that is not a size") {
    const BlockRect whole{0, 0, 23, 17};
    CHECK_THROWS_MATCHES(blockFootprint(cells, kIslandCols, kIslandRows, whole, 0.0, 0.0), std::invalid_argument,
                         Message("the area of a cell must be above 0 and finite, not 0"));
    CHECK_THROWS_MATCHES(blockFootprint(cells, kIslandCols, kIslandRows, whole, kNaN, 0.0), std::invalid_argument,
                         Message("the area of a cell must be above 0 and finite, not nan"));
    CHECK_THROWS_MATCHES(blockFootprint(cells, kIslandCols, kIslandRows, whole, 1.0, -1.0), std::invalid_argument,
                         Message("the least area of a piece must be 0 or more and finite, not -1"));
    CHECK_THROWS_MATCHES(blockFootprint(cells, kIslandCols, kIslandRows, whole, 1.0, kInfinity), std::invalid_argument,
                         Message("the least area of a piece must be 0 or more and finite, not inf"));
  }
}

TEST_CASE("on any cells, blockFootprint outlines every piece or refuses the first that one loop cannot outline",
          "[footprint]") {
  // Cells drawn at random, not footprintCells' result, so that pieces with holes and pieces that meet themselves at a
  // vertex turn up. A piece has a hole when some cell beyond it cannot be reached from beyond its bounding box by steps
  // through cells beyond it, a side at a time; it meets itself at a vertex when it has the two cells of one diagonal
  // there and neither of the other two. The unit must refuse the block, naming the first such piece in the order of
  // their first cells, exactly when there is one, and otherwise give what blockProblem wants.
  std::mt19937 rng(kRandomSeed + 2);
  int refused = 0, outlined = 0;
  for (int trial = 0; trial < 400; ++trial) {
    const int cols = 3 + static_cast<int>(rng() % 14), rows = 3 + static_cast<int>(rng() % 14);
    const int percent = 40 + static_cast<int>(rng() % 55);
    Flags cells(static_cast<std::size_t>((cols - 1) * (rows - 1)));
    for (std::uint8_t& v : cells) v = static_cast<int>(rng() % 100) < percent ? 1 : 0;
    // Half the blocks are the whole grid, the rest anywhere in it.
    BlockRect rect{0, 0, cols - 1, rows - 1};
    if (rng() % 2 == 0) {
      const auto draw = [&rng](int below) { return static_cast<int>(rng() % static_cast<unsigned>(below)); };
      rect.firstColumn = draw(cols - 1);
      rect.firstRow = draw(rows - 1);
      rect.lastColumn = rect.firstColumn + 1 + draw(cols - 1 - rect.firstColumn);
      rect.lastRow = rect.firstRow + 1 + draw(rows - 1 - rect.firstRow);
    }
    INFO("trial " << trial << ", " << cols << " x " << rows << ", block of " << text(rect));

    std::string expected;  // the start of the message for the first piece that cannot be outlined, if any
    for (const Component& c : componentsIn(cells, cols, rect)) {
      int lowX = c.first.first, highX = lowX, lowY = c.first.second, highY = lowY;
      for (const auto& [x, y] : c.cells) {
        lowX = std::min(lowX, x);
        highX = std::max(highX, x);
        lowY = std::min(lowY, y);
        highY = std::max(highY, y);
      }
      std::set<Cell> beyond{{lowX - 1, lowY - 1}};
      std::vector<Cell> pending{{lowX - 1, lowY - 1}};
      while (!pending.empty()) {
        const Cell at = pending.back();
        pending.pop_back();
        for (const Cell& step : kSides) {
          const Cell to{at.first + step.first, at.second + step.second};
          if (to.first < lowX - 1 || to.first > highX + 1 || to.second < lowY - 1 || to.second > highY + 1) continue;
          if (c.cells.contains(to) || !beyond.insert(to).second) continue;
          pending.push_back(to);
        }
      }
      const auto box = static_cast<std::size_t>((highX - lowX + 3) * (highY - lowY + 3));
      const bool hole = beyond.size() + c.cells.size() != box;
      bool meetsItself = false;
      for (int y = lowY; y <= highY + 1; ++y) {
        for (int x = lowX; x <= highX + 1; ++x) {
          const bool nw = c.cells.contains({x - 1, y - 1}), ne = c.cells.contains({x, y - 1});
          const bool sw = c.cells.contains({x - 1, y}), se = c.cells.contains({x, y});
          meetsItself = meetsItself || (nw && se && !ne && !sw) || (ne && sw && !nw && !se);
        }
      }
      if (hole || meetsItself) {
        expected = "the piece whose first cell is (" + std::to_string(c.first.first) + ", " +
                   std::to_string(c.first.second) + ") ";
        break;
      }
    }

    if (expected.empty()) {
      const BlockFootprint f = blockFootprint(cells, cols, rows, rect, 1.0, 2.0);
      const std::string problem = blockProblem(f, cells, cols, rows, rect, 1.0, 2.0);
      INFO("problem: " << problem);
      CHECK(problem.empty());
      ++outlined;
    } else {
      INFO("expected a refusal starting: " << expected);
      try {
        (void)blockFootprint(cells, cols, rows, rect, 1.0, 2.0);
        FAIL_CHECK("no refusal");
      } catch (const std::invalid_argument& e) {
        CHECK(std::string(e.what()).starts_with(expected));
      }
      ++refused;
    }
  }
  CHECK(refused > 50);
  CHECK(outlined > 50);
}

TEST_CASE("blockFootprint refuses cells with a hole or a piece that touches itself", "[footprint]") {
  // footprintCells never gives either: it fills every enclosed region, and a piece that touched itself at a vertex
  // would enclose one of the two cells off its diagonal. Cells from elsewhere could, and an outline of one loop cannot
  // describe them.
  SECTION("a ring of cells round a hole") {
    // 5 x 5 vertices: the 3 x 3 cells of columns and rows 0 to 2 but the middle one, (1, 1).
    const Flags ring = valuesOf({"###.", "#.#.", "###.", "...."});
    CHECK_THROWS_MATCHES(blockFootprint(ring, 5, 5, {0, 0, 4, 4}, 1.0, 0.0), std::invalid_argument,
                         Message("the piece whose first cell is (0, 0) has a hole: blockFootprint takes the cells "
                                 "footprintCells gives, in which every enclosed region is filled"));
  }

  SECTION("a piece that meets itself at a vertex") {
    // The cells (1, 1) and (2, 2) meet at the vertex (2, 2), with (2, 1) and (1, 2) out, and are joined round (2, 1)
    // by the cells (1, 0), (2, 0), (3, 0), (3, 1) and (3, 2).
    const Flags pinched = valuesOf({".###", ".#.#", "..##", "...."});
    CHECK_THROWS_MATCHES(blockFootprint(pinched, 5, 5, {0, 0, 4, 4}, 1.0, 0.0), std::invalid_argument,
                         Message("the piece whose first cell is (1, 0) touches itself at the vertex (2, 2): "
                                 "blockFootprint takes the cells footprintCells gives, in which no piece does"));
  }
}
