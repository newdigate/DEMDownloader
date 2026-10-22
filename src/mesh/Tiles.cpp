#include "mesh/Tiles.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <limits>
#include <optional>
#include <stdexcept>
#include <string>
#include <string_view>

#include "mesh/Simplifier.h"

namespace demdl::mesh {

namespace {

std::string formatNumber(double value) {
  char text[40];
  std::snprintf(text, sizeof text, "%.15g", value);
  return text;
}

std::string gridSize(int cols, int rows) { return std::to_string(cols) + " x " + std::to_string(rows); }

// --- tileLayout ------------------------------------------------------------------------------------------------------

void requirePositive(const char* what, double value) {
  if (!(std::isfinite(value) && value > 0)) {
    throw std::invalid_argument(std::string(what) + " must be above 0 and finite, not " + formatNumber(value));
  }
}

// The cells along one axis of a block: round(tileUnits / (scale x cellM)), kept from 1 to the grid's cells, which are
// vertices - 1. The clamp is made on the double, before the conversion, so that a ratio of any size, an infinite one
// among them (a product that underflowed to 0), is a block of the whole grid and not an int that overflowed. The
// ratio cannot be NaN, the tile being finite and above 0; were it, the block would be one cell.
int blockCells(double tileUnits, double scale, double cellM, int vertices) {
  const double ratio = std::round(tileUnits / (scale * cellM));
  const int grid = vertices - 1;
  if (ratio >= static_cast<double>(grid)) return grid;
  if (ratio > 1) return static_cast<int>(ratio);
  return 1;
}

// ceil(cells / blockCells) for both above 0, without the overflow of cells + blockCells - 1.
int blocksFor(int cells, int blockCells) { return cells / blockCells + (cells % blockCells != 0 ? 1 : 0); }

std::string tooLarge(int cells, const char* side) {
  return "a block would be " + std::to_string(cells) + " cells " + side + ", but a block side may be at most " +
         std::to_string(kMostChunkCells) + " cells: use a smaller --tile or a coarser --grid";
}

// --- blockRect, cutShort, blockName ----------------------------------------------------------------------------------

bool inLattice(const TileLayout& layout, int r, int c) noexcept {
  return r >= 0 && r < layout.down && c >= 0 && c < layout.across;
}

void requireBlock(const TileLayout& layout, int r, int c) {
  if (inLattice(layout, r, c)) return;
  throw std::invalid_argument("the block at row " + std::to_string(r) + ", column " + std::to_string(c) +
                              " is outside the lattice of " + std::to_string(layout.down) + " rows and " +
                              std::to_string(layout.across) + " columns, counted from 0");
}

// Where block `index` would end along an axis whose blocks have blockCells cells: (index + 1) x blockCells. For the
// last block that is past the grid's last vertex when the block is cut short, and on a grid near the most an int counts
// it is past what an int holds, so it is worked out in 64 bits.
std::int64_t blockEnd(int index, int blockCells) { return (std::int64_t{index} + 1) * blockCells; }

// number in decimal, zero-padded to the digits that `count` has.
std::string padded(int number, int count) {
  std::string digits = std::to_string(number);
  const std::size_t width = std::to_string(count).size();
  if (digits.size() < width) digits.insert(0, width - digits.size(), '0');
  return digits;
}

// --- parseBlocks -----------------------------------------------------------------------------------------------------

// `text` in single quotes, readable whatever it holds, with the escapes of the command line's other usage errors
// (demdl::cli::quoted in app/CliChecks.h, which the library units do not include): \r, \n and \t are shown as those
// escapes, a backslash as \\, a single quote as \', and any other byte outside printable ASCII (0x20..0x7E) as \xHH,
// so that a carriage return left in a script by another system's editor shows.
std::string quoted(std::string_view text) {
  static constexpr char kHex[] = "0123456789ABCDEF";
  std::string out = "'";
  for (const char c : text) {
    const auto byte = static_cast<unsigned char>(c);
    switch (c) {
      case '\r': out += "\\r"; break;
      case '\n': out += "\\n"; break;
      case '\t': out += "\\t"; break;
      case '\\': out += "\\\\"; break;
      case '\'': out += "\\'"; break;
      default:
        if (byte >= 0x20 && byte <= 0x7E) {
          out += c;
        } else {
          out += "\\x";
          out += kHex[byte >> 4];
          out += kHex[byte & 0x0F];
        }
    }
  }
  return out + "'";
}

// One part of ROWS:COLS as typed: the part itself, and the digits of its first number and of its last, which are the
// same text for a lone number.
struct PartText {
  std::string_view typed, first, last;
};

// Digits and only digits, at least one. ASCII only, so that the result does not depend on the locale.
bool allDigits(std::string_view text) {
  if (text.empty()) return false;
  return std::all_of(text.begin(), text.end(), [](char c) { return c >= '0' && c <= '9'; });
}

// N or A-B, as the digits of its numbers; nothing for any other text, an empty number, a second dash, a sign and a
// colon among them.
std::optional<PartText> splitPart(std::string_view part) {
  const std::size_t dash = part.find('-');
  const std::string_view first = part.substr(0, dash);
  const std::string_view last = dash == std::string_view::npos ? first : part.substr(dash + 1);
  if (!allDigits(first) || !allDigits(last)) return std::nullopt;
  return PartText{part, first, last};
}

// The value of digits, from 1 to the largest int; nothing for 0 and for a value an int cannot hold. A leading zero
// adds nothing to the value, so a run of them cannot overflow it.
std::optional<int> blockNumber(std::string_view digits) {
  std::int64_t value = 0;
  for (const char digit : digits) {
    value = value * 10 + (digit - '0');
    if (value > std::numeric_limits<int>::max()) return std::nullopt;
  }
  if (value < 1) return std::nullopt;
  return static_cast<int>(value);
}

std::invalid_argument badForm(const std::string& text) {
  return std::invalid_argument("must be ROWS:COLS, each part a whole number from 1 or a range such as 3-5, not " +
                               quoted(text));
}

// The digits of a number, and the two numbers and the dash of a range, are only ever digits and a dash by the time they
// are named, so badNumber and badOrder print them as they are and quote only the whole text.
std::invalid_argument badNumber(std::string_view digits, const std::string& text) {
  return std::invalid_argument("block numbers run from 1 to " + std::to_string(std::numeric_limits<int>::max()) +
                               ", not " + std::string(digits) + " in " + quoted(text));
}

std::invalid_argument badOrder(std::string_view range, const std::string& text) {
  return std::invalid_argument("a range must end at or above its start, not " + std::string(range) + " in " +
                               quoted(text));
}

struct Range {
  int first, last;
};

// The numbers of a part; a lone number is a range of one. The first number is read, and refused, before the last.
Range numbersOf(const PartText& part, const std::string& text) {
  const auto number = [&text](std::string_view digits) {
    const std::optional<int> value = blockNumber(digits);
    if (!value) throw badNumber(digits, text);
    return *value;
  };
  return {number(part.first), number(part.last)};
}

}  // namespace

TileLayout tileLayout(int cols, int rows, double tileUnits, double scale, double cellXM, double cellYM) {
  if (cols < 2 || rows < 2) {
    throw std::invalid_argument("the grid must be at least 2 x 2 vertices, not " + gridSize(cols, rows));
  }
  requirePositive("the tile size", tileUnits);
  requirePositive("the scale", scale);
  requirePositive("the cell width", cellXM);
  requirePositive("the cell height", cellYM);

  const int blockCols = blockCells(tileUnits, scale, cellXM, cols);
  const int blockRows = blockCells(tileUnits, scale, cellYM, rows);
  if (blockCols > kMostChunkCells) throw std::invalid_argument(tooLarge(blockCols, "wide"));
  if (blockRows > kMostChunkCells) throw std::invalid_argument(tooLarge(blockRows, "tall"));
  return {cols, rows, blockCols, blockRows, blocksFor(cols - 1, blockCols), blocksFor(rows - 1, blockRows)};
}

BlockRect blockRect(const TileLayout& layout, int r, int c) {
  requireBlock(layout, r, c);
  // The first column and row are inside the grid, since the block exists; the last are cut back to its last vertex.
  return {c * layout.blockCols, r * layout.blockRows,
          static_cast<int>(std::min<std::int64_t>(blockEnd(c, layout.blockCols), layout.cols - 1)),
          static_cast<int>(std::min<std::int64_t>(blockEnd(r, layout.blockRows), layout.rows - 1))};
}

bool cutShort(const TileLayout& layout, int r, int c) noexcept {
  if (!inLattice(layout, r, c)) return false;
  // A block has its full cells unless the grid's last vertex comes before the end that a full block would have.
  return blockEnd(c, layout.blockCols) > layout.cols - 1 || blockEnd(r, layout.blockRows) > layout.rows - 1;
}

std::string blockName(const TileLayout& layout, int r, int c) {
  requireBlock(layout, r, c);
  return "r" + padded(r + 1, layout.down) + "_c" + padded(c + 1, layout.across);
}

bool BlockSelection::selected(int r, int c) const noexcept {
  // The ranges are 1-based and r and c from 0: r is in firstRow..lastRow when r + 1 is, without the r + 1 that could
  // overflow.
  return all || (r >= firstRow - 1 && r < lastRow && c >= firstCol - 1 && c < lastCol);
}

BlockSelection parseBlocks(const std::string& text) {
  const std::string_view view = text;
  const std::size_t colon = view.find(':');
  if (colon == std::string_view::npos) throw badForm(text);
  const std::optional<PartText> rows = splitPart(view.substr(0, colon));
  // A second colon is not a digit or a dash, so it fails here.
  const std::optional<PartText> cols = splitPart(view.substr(colon + 1));
  if (!rows || !cols) throw badForm(text);

  const Range rowNumbers = numbersOf(*rows, text);
  const Range colNumbers = numbersOf(*cols, text);
  if (rowNumbers.last < rowNumbers.first) throw badOrder(rows->typed, text);
  if (colNumbers.last < colNumbers.first) throw badOrder(cols->typed, text);

  BlockSelection selection;
  selection.all = false;
  selection.firstRow = rowNumbers.first;
  selection.lastRow = rowNumbers.last;
  selection.firstCol = colNumbers.first;
  selection.lastCol = colNumbers.last;
  return selection;
}

}  // namespace demdl::mesh
