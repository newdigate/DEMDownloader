#pragma once

#include <span>
#include <vector>

namespace demdl::mesh {

// The indices to keep of a profile of heights along a line, ascending, the first and the last always among them,
// such that every height is within `tolerance` (vertical distance, metres) of the straight segment between the kept
// indices either side of it. Douglas and Peucker's split rule: a segment whose worst height is further than the
// tolerance is split at that height, the smallest index winning a tie, until none is. Deterministic. Throws
// std::invalid_argument when heights has fewer than 2 values or a value that is not finite (a NaN, or an infinity,
// which no tolerance can bound), or tolerance is negative or not finite.
//
// This is the one-dimensional rule that settles a chunk's sides before its interior is touched (spec 4.4): two chunks
// that share a side simplify the same heights by it, so they keep the same vertices. The caller copies a column of the
// grid into a contiguous run first. The indices are ints, so heights has at most INT_MAX values; a debug build asserts
// it, and the longest side the simplifier takes is 16,385 (spec 4.5).
//
// What follows from the rule:
// - The distance is vertical, in the heights' own unit, never perpendicular: it is the distance the finished surface
//   is held to (spec 4.3). An error equal to the tolerance is allowed, so tolerance 0 drops exactly the heights that
//   lie on their segment and keeps only the ends of each straight run.
// - A lone spike drags its neighbours in. The segment from an end n steps away up to a spike of height H passes
//   H (n - 1) / n over the sample beside it, nearly all of H unless n is small, so a spike is kept alone only when the
//   tolerance is close to H.
// - The same heights and tolerance give the same indices on every run, and are built to on every platform: the
//   distances are worked in double without a division, and each product is rounded on its own so that a fused
//   multiply-add cannot change an answer.
//
// The cost is that of Douglas and Peucker's rule: about n log n when each split falls well inside its segment, and
// O(n^2) when it falls beside an end every time, as in a zigzag growing by one a step, or among many equal heights (the
// smallest index of a tie is taken). The longest side, 16,385 heights, is then about 134 million looks. The work is a
// stack of segments in a vector rather than recursion, so that case cannot overflow the call stack.
[[nodiscard]] std::vector<int> simplifyProfile(std::span<const float> heights, double tolerance);

}  // namespace demdl::mesh
