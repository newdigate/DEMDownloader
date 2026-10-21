#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <initializer_list>
#include <limits>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "mesh/ProfileSimplifier.h"

using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::Message;
using demdl::mesh::simplifyProfile;

// Every expected set follows from the rule in the header and is worked out beside its case. The chord of a run is the
// straight line between its two kept ends; a height's error is its vertical distance from the chord; the worst height
// of a run is split at when its error is over the tolerance, and of equal errors the smallest index is taken. The
// cases are also held to the contract itself by firstProblem below, which measures every height against its chord one
// at a time, so that an expected set that is wrong by hand is found out.

namespace {

using Profile = std::vector<float>;
using Indices = std::vector<int>;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();
constexpr float kInf = std::numeric_limits<float>::infinity();

// How much more than the tolerance firstProblem lets an error be. The chord is worked in double and so is the library's
// multiplied-out form, and the two can differ by a few units in the last place of a number of about 1000, 1e-13. A
// true violation would have to be smaller than a billionth of a metre to hide in that; with heights in whole metres,
// off a chord of at most 256 steps, none is smaller than 1/256.
constexpr double kRounding = 1e-9;

// count heights of 0 with the given heights raised at the given indices.
Profile flatWith(std::size_t count, std::initializer_list<std::pair<std::size_t, float>> raised) {
  Profile heights(count, 0.0f);
  for (const auto& [index, height] : raised) heights.at(index) = height;
  return heights;
}

// Says what is wrong with kept as the simplification of heights at the tolerance, or returns "" when nothing is: the
// indices must run strictly upward from 0 to the last, and no height may be further than the tolerance, vertically,
// from the straight segment between the kept indices either side of it. The segment is evaluated height by height, as
// the definition says it, where the library multiplies the distance out to avoid a division.
std::string firstProblem(const Profile& heights, const Indices& kept, double tolerance) {
  const int last = static_cast<int>(heights.size()) - 1;
  if (kept.size() < 2) return "fewer than 2 indices are kept";
  if (kept.front() != 0) return "the first index is " + std::to_string(kept.front()) + ", not 0";
  if (kept.back() != last) {
    return "the last index is " + std::to_string(kept.back()) + ", not " + std::to_string(last);
  }
  for (std::size_t i = 1; i < kept.size(); ++i) {
    if (kept[i] <= kept[i - 1]) {
      return "index " + std::to_string(kept[i]) + " comes after " + std::to_string(kept[i - 1]);
    }
  }
  for (std::size_t i = 1; i < kept.size(); ++i) {
    const int first = kept[i - 1];
    const int end = kept[i];
    for (int k = first + 1; k < end; ++k) {
      const double chord = heights[first] + (double{heights[end]} - heights[first]) * (k - first) / (end - first);
      const double off = std::fabs(double{heights[k]} - chord);
      if (off > tolerance + kRounding) {
        return "the height at " + std::to_string(k) + " is " + std::to_string(off) + " from the segment between " +
               std::to_string(first) + " and " + std::to_string(end) + ", over the tolerance " +
               std::to_string(tolerance);
      }
    }
  }
  return {};
}

// Douglas and Peucker's rule written as plainly as it can be said, for whole-number heights and a whole tolerance: by
// recursion, with every error multiplied by its run's length so that it is a whole number too and nothing is rounded
// or divided, where the library iterates and works in double. It shares the rule with the code under test and nothing
// else.
void referenceSplit(const Profile& heights, int first, int last, std::int64_t tolerance, Indices& kept) {
  const std::int64_t length = last - first;
  const std::int64_t base = static_cast<std::int64_t>(heights[first]);
  const std::int64_t rise = static_cast<std::int64_t>(heights[last]) - base;
  std::int64_t worst = -1;
  int worstIndex = -1;
  for (int k = first + 1; k < last; ++k) {
    const std::int64_t off = std::abs((static_cast<std::int64_t>(heights[k]) - base) * length - rise * (k - first));
    if (off > worst) {  // strictly: of equal errors the first met, the smallest index, stays
      worst = off;
      worstIndex = k;
    }
  }
  if (worstIndex < 0 || worst <= tolerance * length) return;
  kept.push_back(worstIndex);
  referenceSplit(heights, first, worstIndex, tolerance, kept);
  referenceSplit(heights, worstIndex, last, tolerance, kept);
}

Indices referenceKept(const Profile& heights, std::int64_t tolerance) {
  const int last = static_cast<int>(heights.size()) - 1;
  Indices kept{0, last};
  referenceSplit(heights, 0, last, tolerance, kept);
  std::sort(kept.begin(), kept.end());
  return kept;
}

// How many checks were made, how many found something wrong, and the first that did, for the failure message.
struct Tally {
  int checked = 0;
  int wrong = 0;
  std::string first;

  void add(const std::string& problem, const std::string& where) {
    ++checked;
    if (problem.empty()) return;
    if (++wrong == 1) first = where + ": " + problem;
  }
};

}  // namespace

TEST_CASE("a constant profile keeps only its two ends, at any tolerance", "[profile]") {
  // Every height is on the chord, so no error is over any tolerance, 0 included. The levels are a round number, one
  // that is not exact in binary and a depth below sea level.
  for (const float level : {7.25f, 0.1f, -37.9f}) {
    const Profile heights(100, level);
    for (const double tolerance : {0.0, 0.001, 1.0, 1000.0}) {
      INFO("level " << level << ", tolerance " << tolerance);
      CHECK(simplifyProfile(heights, tolerance) == Indices{0, 99});
    }
  }
}

TEST_CASE("a straight ramp has no error, so it keeps only its two ends", "[profile]") {
  // h[k] = 3 k: the chord from (0, 0) to (99, 297) passes through every height exactly. The same for a ramp down
  // through sea level, 100 - 2.5 k, whose products are all multiples of 0.5 and so exact too, so that tolerance 0
  // is as good as 1.
  Profile up(100), down(100);
  for (std::size_t k = 0; k < up.size(); ++k) {
    up[k] = 3.0f * static_cast<float>(k);
    down[k] = 100.0f - 2.5f * static_cast<float>(k);
  }
  for (const double tolerance : {0.0, 1.0}) {
    INFO("tolerance " << tolerance);
    CHECK(simplifyProfile(up, tolerance) == Indices{0, 99});
    CHECK(simplifyProfile(down, tolerance) == Indices{0, 99});
  }
}

TEST_CASE("a single spike drags its neighbours in until the tolerance covers the slope beside it", "[profile]") {
  // 100 heights, all 0 but a 10 at index 40. The chord over the whole profile is flat at 0 and the spike is 10 off it.
  // - Tolerance 10 is that error exactly, which is allowed: nothing is split, {0, 99}.
  // - Below 10 the spike is split at, and the zeros beside it are then measured against a chord that climbs to it. On
  //   [0, 40] the chord rises 10/40 a step, so it passes 9.75 over the 0 at 39; on [40, 99] it falls 10/59 a step,
  //   so it passes 10 - 10/59 = 9.83 over the 0 at 41. The other zeros are nearer their chords.
  // - Tolerance 9.9 covers both slopes: {0, 40, 99}.
  // - Tolerance 9.75 covers the 9.75 exactly, which is allowed, but not the 9.83, so 41 is split at too:
  //   {0, 40, 41, 99}.
  // - Tolerance 5 covers neither, so both 39 and 41 are split at: {0, 39, 40, 41, 99}, and the rest is flat.
  // A spike alone, {0, 40, 99}, takes a tolerance of at least 10 - 10/59 = 9.83.
  const Profile heights = flatWith(100, {{40, 10.0f}});
  struct Row {
    double tolerance;
    Indices expected;
  };
  const std::vector<Row> rows{
      {10.0, {0, 99}},
      {9.9, {0, 40, 99}},
      {9.75, {0, 40, 41, 99}},
      {5.0, {0, 39, 40, 41, 99}},
  };
  for (const Row& row : rows) {
    INFO("tolerance " << row.tolerance);
    const Indices kept = simplifyProfile(heights, row.tolerance);
    CHECK(kept == row.expected);
    const std::string problem = firstProblem(heights, kept, row.tolerance);
    INFO("problem: " << problem);
    CHECK(problem.empty());
  }
}

TEST_CASE("of two heights equally far off, the smaller index is split at first", "[profile]") {
  // 100 heights, all 0 but a 10 at 30 and a 10 at 60. The chord over the whole profile is flat at 0 and both spikes
  // are 10 off it, so they tie, and the rule takes the 30.
  const Profile heights = flatWith(100, {{30, 10.0f}, {60, 10.0f}});

  SECTION("at tolerance 5 both are kept, each with its neighbours, whichever goes first") {
    // After the 30: [0, 30] climbs 10/30 a step, so the 0 at 29 is 9.67 under the chord, and [30, 99] falls 10/69 a
    // step, so the 0 at 31 is 9.86 under it (the spike at 60 is only 4.35 over it). Both 29 and 31 are split at, and
    // [31, 99] is flat at 0 with the 60 still 10 over: split at it. The 0 at 59 is then 10 x 28/29 = 9.66 under the
    // chord of [31, 60], the 0 at 61 is 10 - 10/39 = 9.74 under that of [60, 99], and both are split at.
    const Indices expected{0, 29, 30, 31, 59, 60, 61, 99};
    const Indices kept = simplifyProfile(heights, 5.0);
    CHECK(kept == expected);
    const std::string problem = firstProblem(heights, kept, 5.0);
    INFO("problem: " << problem);
    CHECK(problem.empty());
  }

  SECTION("at tolerance 9.9 the first split is the only one") {
    // [0, 30] has its worst at 29, 9.67, and [30, 99] at 31, 9.86; the spike at 60 is 4.35 over the chord of [30, 99].
    // All are within 9.9, so the 30 alone is kept: {0, 30, 99}. Had the 60 been split at first, [0, 60] would be
    // within 9.9 as well (the 0 at 59 is 9.83 under its chord, the spike at 30 is 5 over it), and so would [60, 99]
    // (9.74 at 61): the answer would have been {0, 60, 99}. The tie rule is what chooses between them.
    const Indices kept = simplifyProfile(heights, 9.9);
    CHECK(kept == Indices{0, 30, 99});
    const std::string problem = firstProblem(heights, kept, 9.9);
    INFO("problem: " << problem);
    CHECK(problem.empty());
  }
}

TEST_CASE("tolerance 0 keeps only the ends of each straight run", "[profile]") {
  // {0, 1, 2, 3, 2, 1, 0}: the chord over the whole profile is flat at 0 and the 3 at index 3 is the furthest, 3 off.
  // The two halves then rise and fall by 1 a step exactly, so no height is off its chord at all, which tolerance 0
  // allows.
  const Profile heights{0, 1, 2, 3, 2, 1, 0};
  CHECK(simplifyProfile(heights, 0.0) == Indices{0, 3, 6});
}

TEST_CASE("a profile of two values is kept whole and one of three needs a single look", "[profile]") {
  // Two values have nothing between them to be off.
  const Profile two{5.0f, 9.0f};
  CHECK(simplifyProfile(two, 0.0) == Indices{0, 1});
  CHECK(simplifyProfile(two, 1000.0) == Indices{0, 1});

  // {0, 10, 0}: the 10 is 10 off the flat chord, so it is kept below 10 and dropped at 10 and above.
  const Profile three{0.0f, 10.0f, 0.0f};
  CHECK(simplifyProfile(three, 9.99) == Indices{0, 1, 2});
  CHECK(simplifyProfile(three, 10.0) == Indices{0, 2});
}

TEST_CASE("a zigzag that grows by one a step is peeled one index at a time", "[profile]") {
  // h[k] = (-1)^k k for k = 0 to 2000: 0, -1, 2, -3, 4, ... The chord of [0, m] is the line through the origin with
  // slope (-1)^m. The indices of m's parity are on it and each index of the other parity is 2 k off it, so the worst
  // is the largest of those, m - 1, which is 2 (m - 1) off. Every split therefore peels the index beside the end,
  // and what is left, [0, m - 1], has the same shape: the longest chain of splits a profile of 2001 heights can make,
  // each looking through every height left. It goes on while 2 (m - 1) is over the tolerance T, so down to
  // m = T/2 + 1: for T = 500 the last run is [0, 251], whose worst, 250, is 500 off, exactly T, which is allowed.
  // What is kept is 0 and 251 to 2000. For T = 0 it is everything.
  Profile heights(2001);
  for (std::size_t k = 0; k < heights.size(); ++k) {
    heights[k] = (k % 2 == 0 ? 1.0f : -1.0f) * static_cast<float>(k);
  }

  Indices fromTheEnd{0};
  for (int k = 251; k <= 2000; ++k) fromTheEnd.push_back(k);
  CHECK(simplifyProfile(heights, 500.0) == fromTheEnd);

  Indices everything(2001);
  std::iota(everything.begin(), everything.end(), 0);
  CHECK(simplifyProfile(heights, 0.0) == everything);
}

TEST_CASE("on random profiles every height is within the tolerance of its segment, and the result is the rule's",
          "[profile]") {
  // 200 profiles of 257 heights in [0, 1000) at tolerances 0, 1, 10, 100 and 1000, in two sets: whole metres, which
  // the reference below can work in integers exactly, and steps of a millimetre (as floats, which are not exact), which
  // are held to the contract alone. The generator is std::mt19937, whose output the standard fixes, so the profiles are
  // the same on every machine; the standard distributions are not, and are not used.
  //
  // For each: the indices begin at 0, end at 256, ascend strictly, and no height is over the tolerance from its segment
  // (firstProblem); the same input gives the same indices twice; for whole metres, the indices are the reference's
  // exactly, ties and all; and a tolerance as wide as the range keeps only the ends, since no chord between heights
  // below 1000 leaves a height more than 999.999 off it.
  constexpr int kProfiles = 200;
  constexpr std::size_t kLength = 257;
  const std::vector<std::int64_t> tolerances{0, 1, 10, 100, 1000};

  std::mt19937 rng(7);
  Tally tally;
  std::vector<std::size_t> keptWhole(tolerances.size(), 0);  // indices kept in all, per tolerance, for whole metres
  for (const bool whole : {true, false}) {
    for (int number = 0; number < kProfiles; ++number) {
      Profile heights(kLength);
      for (float& height : heights) {
        height = whole ? static_cast<float>(rng() % 1000u) : static_cast<float>(rng() % 1000000u) / 1000.0f;
      }
      for (std::size_t t = 0; t < tolerances.size(); ++t) {
        const std::int64_t tolerance = tolerances[t];
        const std::string where = std::string(whole ? "whole metres" : "millimetres") + " profile " +
                                  std::to_string(number) + ", tolerance " + std::to_string(tolerance);
        const Indices kept = simplifyProfile(heights, static_cast<double>(tolerance));
        tally.add(firstProblem(heights, kept, static_cast<double>(tolerance)), where);
        const bool repeated = simplifyProfile(heights, static_cast<double>(tolerance)) == kept;
        tally.add(repeated ? "" : "a second call differs", where);
        if (whole) tally.add(kept == referenceKept(heights, tolerance) ? "" : "differs from the reference", where);
        if (tolerance == 1000) tally.add(kept == Indices{0, 256} ? "" : "keeps more than the two ends", where);
        if (whole) keptWhole[t] += kept.size();
      }
    }
  }

  // Every profile and tolerance had the contract and the repeat checked; whole metres the reference as well; and at
  // 1000 the two ends: the loops ran, and nothing was skipped.
  constexpr int kCombinations = kProfiles * 5;
  INFO("first problem: " << tally.first);
  CHECK(tally.checked == (3 * kCombinations + kProfiles) + (2 * kCombinations + kProfiles));
  CHECK(tally.wrong == 0);
  // The sample covers the range: a tighter tolerance keeps more.
  for (std::size_t t = 1; t < keptWhole.size(); ++t) {
    INFO("tolerance " << tolerances[t - 1] << " kept " << keptWhole[t - 1] << " indices, " << tolerances[t] << " kept "
                      << keptWhole[t]);
    CHECK(keptWhole[t - 1] > keptWhole[t]);
  }
}

TEST_CASE("a profile or a tolerance that cannot be used is refused", "[profile]") {
  const Profile fine{0.0f, 5.0f, 0.0f};

  SECTION("fewer than 2 heights") {
    CHECK_THROWS_MATCHES(simplifyProfile(Profile{}, 1.0), std::invalid_argument,
                         Message("a profile needs at least 2 heights, not 0"));
    CHECK_THROWS_MATCHES(simplifyProfile(Profile{3.0f}, 1.0), std::invalid_argument,
                         Message("a profile needs at least 2 heights, not 1"));
  }

  SECTION("a NaN, in the middle or at either end") {
    CHECK_THROWS_MATCHES(simplifyProfile(Profile{0.0f, kNaN, 0.0f}, 1.0), std::invalid_argument,
                         Message("the height at index 1 is not finite"));
    CHECK_THROWS_MATCHES(simplifyProfile(Profile{kNaN, 0.0f, 0.0f}, 1.0), std::invalid_argument,
                         Message("the height at index 0 is not finite"));
    CHECK_THROWS_MATCHES(simplifyProfile(Profile{0.0f, 0.0f, kNaN}, 1.0), std::invalid_argument,
                         Message("the height at index 2 is not finite"));
  }

  SECTION("an infinity, which no tolerance can bound") {
    CHECK_THROWS_MATCHES(simplifyProfile(Profile{0.0f, kInf, 0.0f}, 1.0), std::invalid_argument,
                         Message("the height at index 1 is not finite"));
    CHECK_THROWS_MATCHES(simplifyProfile(Profile{-kInf, 0.0f, 0.0f}, 1.0), std::invalid_argument,
                         Message("the height at index 0 is not finite"));
  }

  SECTION("a tolerance that is negative, NaN or infinite") {
    for (const double tolerance : {-1.0, -0.001, std::numeric_limits<double>::quiet_NaN(),
                                   std::numeric_limits<double>::infinity(),
                                   -std::numeric_limits<double>::infinity()}) {
      INFO("tolerance " << tolerance);
      CHECK_THROWS_WITH(simplifyProfile(fine, tolerance),
                        ContainsSubstring("the tolerance must be finite and not negative"));
    }
  }

  SECTION("a tolerance of 0 is not refused") { CHECK(simplifyProfile(fine, 0.0) == Indices{0, 1, 2}); }
}
