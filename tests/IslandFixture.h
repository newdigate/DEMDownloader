#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <string>
#include <vector>

#include "demdl/BoundingBox.h"
#include "mesh/NoDataFill.h"

// The island: a synthetic coast of 24 x 18 samples for the footprint, the simplifier over it, its solids and dem2mesh's
// blocks (Part B's spec, section 7), shared so that each of those tests works on the same land. Columns are counted
// from 0 at the west and rows from 0 at the north, as the output grid counts them.

inline constexpr int kIslandCols = 24;
inline constexpr int kIslandRows = 18;

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
//   sample at column 19 and row 4: as cells, two blobs of 2 x 2 that touch at that one vertex. footprintCells joins
//   them there by putting in the cell (19, 3), the north-west-most of the vertex's two empty cells, so that they make
//   one piece of 9 cells and footprintCells reports (19, 4) as a joined contact.
// Every feature is at least two no-data samples from the grid's edge and from every other feature. A gap of two
// samples is three cells, and the closing joins nothing across a gap of three.
inline std::vector<std::string> islandMaskRows() {
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
inline std::vector<std::uint8_t> islandMask() {
  std::vector<std::uint8_t> mask;
  for (const std::string& row : islandMaskRows()) {
    for (const char c : row) mask.push_back(c == '#' ? 1 : 0);
  }
  return mask;
}

// The knoll's summit. It stands where the tab joins the island, not at the island's middle, which is the lake: there
// it is a sample strictly inside the island's one piece over the whole grid, so that the simplifier has to insert it,
// and it is on column 12, where a lattice of blocks 12 cells wide cuts the tab off the island, so that the run the two
// blocks share there keeps samples between its ends. Its slopes reach column 13 as well, where blocks 13 cells wide
// cut the tab.
inline constexpr int kIslandKnollColumn = 12;
inline constexpr int kIslandKnollRow = 6;

// The island's heights in metres as readWindow would give them, row by row from the north-west: NaN where the mask has
// no data, and elsewhere 20 + 2 i + 1.5 j at column i and row j, a plane rising to the east and the south, plus a knoll
// of 15 m and radius 2 centred on (kIslandKnollColumn, kIslandKnollRow): 15 (1 - d^2 / 4) at a distance d under 2
// samples from it, so 15 m at the summit, 11.25 m beside it and 7.5 m diagonally beside it. Every height is a multiple
// of 0.25 m, exact in a float, so a straight run along the plane keeps only its ends at any tolerance.
inline std::vector<float> islandHeights() {
  const std::vector<std::uint8_t> mask = islandMask();
  std::vector<float> heights(mask.size());
  for (int j = 0; j < kIslandRows; ++j) {
    for (int i = 0; i < kIslandCols; ++i) {
      const auto k = static_cast<std::size_t>(j) * kIslandCols + static_cast<std::size_t>(i);
      const int di = i - kIslandKnollColumn, dj = j - kIslandKnollRow;
      const int d2 = di * di + dj * dj;
      const double knoll = d2 < 4 ? 15.0 * (1.0 - d2 / 4.0) : 0.0;
      heights[k] = mask[k] != 0 ? static_cast<float>(20.0 + 2.0 * i + 1.5 * j + knoll)
                                : std::numeric_limits<float>::quiet_NaN();
    }
  }
  return heights;
}

// The island's heights after fillNearest, as the simplifier is given them: every no-data sample takes the height of
// its nearest data sample, the lake and the channel included.
inline std::vector<float> islandGrid() {
  std::vector<float> heights = islandHeights();
  (void)demdl::mesh::fillNearest(heights, kIslandCols, kIslandRows);
  return heights;
}

// A WGS 84 box for the island as a file of dl_elevation's, near Robben Island: 0.0006 degrees of longitude by 0.0005
// of latitude a pixel, which at latitude -33.8 are 55.56 m by 55.46 m, so that the cells are square to within 0.2
// percent and a block 12 cells wide is round(12 x 55.56 / 55.46) = 12 cells tall.
inline demdl::BoundingBox islandBox() {
  return {-33.80, -33.80 - kIslandRows * 0.0005, 18.36 + kIslandCols * 0.0006, 18.36};
}
