#pragma once

#include <vector>

namespace demdl::mesh {

// The quadratic B-spline kernel: 3/4 - t^2 for |t| < 1/2, (3/2 - |t|)^2 / 2 for 1/2 <= |t| < 3/2, and 0 beyond. It is
// symmetric, never negative, has a continuous slope, and its area is 1. Laid on whole samples it weighs three of them,
// 1/8, 3/4, 1/8. Laid anywhere between samples, the weights of the samples under it sum to 1 and balance on its centre,
// which is what lets a resampled surface keep its level and its slope. NaN gives 0.
[[nodiscard]] double quadraticBSpline(double t) noexcept;

// Resamples a grid of heights to another size with the kernel above, so that a coarse output has rounded ridges rather
// than stair steps and a fine one is smooth between the samples (spec 5.4). src holds srcCols x srcRows heights row by
// row from the north-west corner, as readWindow and fillNearest give them, with no NaN (checked by an assert in debug
// builds; in a release build a NaN spoils every output within 1.5 stretches of it). The result holds outCols x outRows
// heights in the same order.
//
// The first and last outputs along an axis sit on the first and last samples, so the output covers the same ground as
// the source: output i of outCols stands at the source position u_i = i (srcCols - 1) / (outCols - 1), which falls
// between samples in general. The same down the rows with srcRows and outRows. An output is the mean of the source
// samples k with |k - u_i| < 1.5 s, each weighed by quadraticBSpline((k - u_i) / s), the weights scaled to sum to 1.
// The stretch s is max(1, (srcCols - 1) / (outCols - 1)):
// - an output grid as fine as the source or finer has s = 1: the kernel runs at the source spacing and takes the three
//   samples around u_i (two when u_i is half way between), interpolating between them;
// - a coarser grid has s > 1: the kernel is stretched to the output spacing, so an output averages the samples under
//   its footprint with quadratic weights. That antialiases, with no stair steps and no moire, and smooths in the one
//   pass.
//
// Taps outside the source are taken by odd reflection about the end sample, v(-k) = 2 v(0) - v(k) and the same past
// the last sample, again and again when the kernel is wider than the source. Unlike a mirror, this carries a slope on
// to the border instead of bending it flat, so a plane stays a plane right up to the edge. The implementation folds
// each reflected tap into the samples inside the source (its mirror image and the end sample), which gives the same
// sum without looking anything up outside the source.
//
// Separable: every source row is resampled to outCols, then those rows are resampled down to outRows by the same
// rule. Sums are made in double and rounded to float at the end of each pass.
//
// What follows from that, each pinned by a test:
// - A constant surface comes out constant.
// - A plane comes out as the plane, at the positions u_i and v_j. Exactly, to float rounding, when s is a whole
//   number, which includes every grid as fine as the source or finer. Otherwise the sampled kernel's centre is a
//   little off its position: by evaluating the definition separately, a plane is off by at most 0.018 of its height
//   step per source sample (at a stretch of about 1.6) and by under 0.004 of it once the stretch is 3.5 or more.
// - Nothing leaves the source's range. With three outputs or more along an axis, folding the reflected taps in leaves
//   every weight at least 0: a tap outside the source has its mirror image among the taps inside, nearer to u_i, which
//   weighs at least as much, and only the difference is left on that sample. So each output is a weighted mean of
//   heights, no higher than the highest and no lower than the lowest. With two outputs, both are end samples.
// - The grid's corners are the source's corners, exactly. Along each axis the first output is the first sample and the
//   last output the last: the taps either side of an end sample weigh the same, so their reflections cancel the
//   difference from that sample whatever the stretch. The price is that a border is smoothed along itself but not
//   across it: a row of 2048 samples alternating 0 and 100, down to 64, keeps its 0 at the first output, where the next
//   reads 48 and the rest 50.
// - A summit is lowered, as a B-spline approximates its samples and does not interpolate them. 100 alone in a field of
//   0 comes out as 56.25 (3/4 x 3/4) at an output on it when the kernel runs at the source spacing, which is the same
//   size or finer, and as less on a coarser grid, where the kernel is wider and the summit is shared among more.
//
// Memory beyond src and the result is a few rows, not a second copy of the source: the pass along the rows is done for
// a source row when the pass down the columns first needs it and kept only while the next outputs still read it. That
// is at most 3 s + 1 rows of outCols floats, s being the stretch down the rows, and never more rows than the source has
// (which, for a grid of only two or three rows, is all of them): a ring of rows, filled in the order the outputs reach
// them.
//
// Throws std::invalid_argument when srcCols or srcRows is below 2 (a mesh needs a cell, and the reflection needs two
// end samples), when src does not hold srcCols x srcRows heights, or when outCols or outRows is below 2 (the first and
// last outputs sit on the first and last samples, so a single output has no place). Nothing is read from src then.
[[nodiscard]] std::vector<float> resampleQuadratic(const std::vector<float>& src, int srcCols, int srcRows, int outCols,
                                                   int outRows);

}  // namespace demdl::mesh
