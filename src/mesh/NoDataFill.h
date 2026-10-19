#pragma once

#include <cstdint>
#include <vector>

// The fill finds its work by asking whether a height is NaN, which fast-math lets the compiler answer "no" to (as in
// GeoTiffGrid.h), and then nothing would be filled.
#if defined(__FINITE_MATH_ONLY__) && __FINITE_MATH_ONLY__
#error "NaN marks no data; do not build with -ffast-math or -ffinite-math-only"
#endif

namespace demdl::mesh {

// What fillNearest did. lowestFill and highestFill are the lowest and highest heights it gave to blank pixels, not
// those of the whole window, and stay 0 when there was nothing to fill.
struct FillReport {
  std::uint64_t filled = 0;
  float lowestFill = 0, highestFill = 0;
};

// Gives every NaN pixel of a window the height of the nearest pixel that has one, so that sea comes out flat at about
// coast level, land beyond a data boundary continues as a plateau at the edge's height instead of dropping off a cliff,
// and a gap in an unfinished download takes its surroundings' height instead of becoming a pit (spec 5.3). values holds
// the window row by row from its north-west corner, cols x rows heights as readWindow gives them, and is changed in
// place. A pixel that has a height is never changed, and a blank one gets a copy of one pixel's height, never an
// average, so the heights it ends up with are heights the file has.
//
// Nearest is counted in steps up, down, left or right, so a pixel diagonally next to another is two steps from it. It
// is found by one breadth-first search that starts from every pixel that has a height at once: each wave gives the
// blank pixels one step further out the height of the pixel they were reached from. That is one pass over the window
// and one visit to each blank pixel, not a search per pixel.
//
// Where several pixels with a height are equally near a blank one, it takes the first of them in scan order (row by row
// from the north-west, so the northmost, and in that row the westmost). The outcome depends on the values alone: the
// same input gives the same output, on any machine.
//
// Besides the window, two lists of pixel numbers are held (32 bits each), each as long as the longest wave, which is
// the length of the front: a thin line for a coast or a boundary, and at worst (blanks scattered like a chessboard)
// about the window's own size again.
//
// Throws std::runtime_error "the window has no elevation value at all" when every pixel is NaN, there being nothing to
// fill from, and std::runtime_error when the window has more than 4,294,967,295 pixels, the most that a 32-bit pixel
// number can count (judged before values is looked at). Throws std::invalid_argument when cols or rows is below 1 or
// values does not hold cols x rows heights. Nothing is changed when it throws.
[[nodiscard]] FillReport fillNearest(std::vector<float>& values, int cols, int rows);

}  // namespace demdl::mesh
