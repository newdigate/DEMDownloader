#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <optional>

#include "mesh/BasePlane.h"

using Catch::Matchers::WithinAbs;
using demdl::mesh::BasePlane;
using demdl::mesh::chooseBasePlane;

// Every expected number follows from the rule (spec 5.3), worked out beside its case: the level is the base asked for,
// or sea level, lowered to the lowest height when it is above it; the plane is a margin below the level, 1 percent of
// the height range and at least 1 m; z is metres times the z-scale times the scale; and when the solid would then be
// thinner than the plinth under its lowest point, the plane is pushed down, in model units, to exactly the plinth.

TEST_CASE("a Table Mountain window at sea level is pushed down to the plinth at --scale 0.02", "[baseplane]") {
  // Heights 0 to 1086 m. The margin is 10.86 m, so the plane would be -10.86 m, -0.2172 units: the sea would print
  // 0.2 mm thick. The plane goes to 0 - 2 = -2 units instead, which is -2 / 0.02 = -100 m.
  const BasePlane base = chooseBasePlane(0, 1086, std::nullopt, 1, 0.02, 2);
  CHECK(base.levelM == 0);
  CHECK_FALSE(base.loweredToLowest);
  CHECK_THAT(base.marginM, WithinAbs(10.86, 1e-9));
  CHECK(base.plinthApplied);
  CHECK_THAT(base.planeZ, WithinAbs(-2.0, 1e-9));
  CHECK(base.planeM == base.planeZ / (1 * 0.02));
  CHECK_THAT(base.planeM, WithinAbs(-100.0, 1e-9));
}

TEST_CASE("the City's whole file reaches below sea level, so the level is lowered to its lowest point", "[baseplane]") {
  // Heights -37.9 to 1590.1 m. Sea level is above -37.9, so the level drops to it. The range is 1628 m, the margin
  // 16.28 m and the plane -37.9 - 16.28 = -54.18 m, which at --scale 1 is 16.28 units under the lowest point: thicker
  // than the plinth.
  const BasePlane base = chooseBasePlane(-37.9, 1590.1, std::nullopt, 1, 1, 2);
  CHECK(base.loweredToLowest);
  CHECK(base.levelM == -37.9);
  CHECK_THAT(base.marginM, WithinAbs(16.28, 1e-9));
  CHECK_THAT(base.planeM, WithinAbs(-54.18, 1e-9));
  CHECK_FALSE(base.plinthApplied);
  CHECK(base.planeZ == base.planeM);
}

TEST_CASE("a base asked for above the terrain is lowered to the lowest point", "[baseplane]") {
  // --base 100 over heights 50 to 300: the level drops to 50; the margin is max(1, 2.5) = 2.5 m; the plane is 47.5 m,
  // 2.5 units under the lowest point.
  const BasePlane base = chooseBasePlane(50, 300, 100.0, 1, 1, 2);
  CHECK(base.loweredToLowest);
  CHECK(base.levelM == 50);
  CHECK_THAT(base.marginM, WithinAbs(2.5, 1e-12));
  CHECK_THAT(base.planeM, WithinAbs(47.5, 1e-12));
  CHECK(base.planeZ == base.planeM);
  CHECK_FALSE(base.plinthApplied);
}

TEST_CASE("a base asked for at the lowest point or below the terrain is kept", "[baseplane]") {
  // --base -100 under heights 0 to 300: kept; the margin is 3 m and the plane -103 m.
  const BasePlane below = chooseBasePlane(0, 300, -100.0, 1, 1, 2);
  CHECK_FALSE(below.loweredToLowest);
  CHECK(below.levelM == -100);
  CHECK_THAT(below.marginM, WithinAbs(3.0, 1e-12));
  CHECK_THAT(below.planeM, WithinAbs(-103.0, 1e-12));
  CHECK(below.planeZ == below.planeM);
  CHECK_FALSE(below.plinthApplied);

  // --base 50 over heights 50 to 300: the level is the lowest point already, so it is not lowered.
  const BasePlane at = chooseBasePlane(50, 300, 50.0, 1, 1, 2);
  CHECK_FALSE(at.loweredToLowest);
  CHECK(at.levelM == 50);
}

TEST_CASE("a flat window above sea level keeps sea level and the 1 m margin", "[baseplane]") {
  // Heights all 10 m and no --base: sea level, 0, is below them, so nothing is lowered. The range is 0, so the margin
  // is its least, 1 m, and the plane is -1 m, 11 units under the terrain: thicker than the plinth.
  const BasePlane base = chooseBasePlane(10, 10, std::nullopt, 1, 1, 2);
  CHECK(base.levelM == 0);
  CHECK_FALSE(base.loweredToLowest);
  CHECK(base.marginM == 1);
  CHECK(base.planeM == -1);
  CHECK(base.planeZ == -1);
  CHECK_FALSE(base.plinthApplied);
}

TEST_CASE("a flat window with a base asked for above it is lowered, then pushed down to the plinth", "[baseplane]") {
  // --base 20 over a flat 10 m: the level drops to 10 and the plane to 9, only 1 unit under the terrain, so the 2-unit
  // plinth puts it at 10 - 2 = 8.
  const BasePlane base = chooseBasePlane(10, 10, 20.0, 1, 1, 2);
  CHECK(base.loweredToLowest);
  CHECK(base.levelM == 10);
  CHECK(base.marginM == 1);
  CHECK(base.plinthApplied);
  CHECK(base.planeZ == 8);
  CHECK(base.planeM == 8);
}

TEST_CASE("a solid exactly as thick as the plinth is left as it is", "[baseplane]") {
  // As above with a 1-unit plinth: the plane at 9 is exactly 1 unit under the flat 10, which is thick enough.
  const BasePlane base = chooseBasePlane(10, 10, 20.0, 1, 1, 1);
  CHECK_FALSE(base.plinthApplied);
  CHECK(base.planeM == 9);
  CHECK(base.planeZ == 9);
}

TEST_CASE("the margin is 1 percent of the window's height range, at least 1 m", "[baseplane]") {
  // With no plinth the plane is the level minus the margin, so the margin shows in planeM as well.
  CHECK(chooseBasePlane(0, 50, std::nullopt, 1, 1, 0).marginM == 1);  // 0.5 m is less than the least
  CHECK_THAT(chooseBasePlane(0, 100, std::nullopt, 1, 1, 0).marginM, WithinAbs(1.0, 1e-12));
  CHECK_THAT(chooseBasePlane(0, 250, std::nullopt, 1, 1, 0).marginM, WithinAbs(2.5, 1e-12));
  CHECK_THAT(chooseBasePlane(0, 250, std::nullopt, 1, 1, 0).planeM, WithinAbs(-2.5, 1e-12));
  CHECK_THAT(chooseBasePlane(-500, 1500, std::nullopt, 1, 1, 0).marginM, WithinAbs(20.0, 1e-12));
  // The range is the terrain's own, highest minus lowest, not measured from the level: heights 100 to 300 above sea
  // level give 2 m, where 1 percent of the 300 m from sea level to the top would be 3 m.
  const BasePlane high = chooseBasePlane(100, 300, std::nullopt, 1, 1, 0);
  CHECK_THAT(high.marginM, WithinAbs(2.0, 1e-12));
  CHECK_THAT(high.planeM, WithinAbs(-2.0, 1e-12));
}

TEST_CASE("the z-scale and the scale multiply the plane's z by their product", "[baseplane]") {
  // The City's numbers with no plinth: the plane stays at -54.18 m, and its z is -54.18 x 3 x 0.02 = -54.18 x 0.06.
  const BasePlane plain = chooseBasePlane(-37.9, 1590.1, std::nullopt, 1, 1, 0);
  const BasePlane scaled = chooseBasePlane(-37.9, 1590.1, std::nullopt, 3, 0.02, 0);
  CHECK_FALSE(scaled.plinthApplied);
  CHECK_THAT(scaled.planeM, WithinAbs(plain.planeM, 1e-12));
  CHECK_THAT(scaled.planeZ, WithinAbs(plain.planeZ * 0.06, 1e-9));
  CHECK_THAT(chooseBasePlane(-37.9, 1590.1, std::nullopt, 3, 1, 0).planeZ, WithinAbs(plain.planeZ * 3, 1e-9));
  CHECK_THAT(chooseBasePlane(-37.9, 1590.1, std::nullopt, 1, 0.02, 0).planeZ, WithinAbs(plain.planeZ * 0.02, 1e-9));
}

TEST_CASE("the plinth is measured in model units under the lowest point", "[baseplane]") {
  // Table Mountain at --scale 1: the 10.86 m margin is 10.86 units, thicker than 2, so no plinth.
  const BasePlane life = chooseBasePlane(0, 1086, std::nullopt, 1, 1, 2);
  CHECK_FALSE(life.plinthApplied);
  CHECK_THAT(life.planeZ, WithinAbs(-10.86, 1e-9));

  // With --z-scale 3 at --scale 0.02 the margin is 10.86 x 0.06 = 0.6516 units: the plinth puts the plane at -2 units,
  // which in metres before exaggeration is -2 / 0.06 = -33.33 m.
  const BasePlane tall = chooseBasePlane(0, 1086, std::nullopt, 3, 0.02, 2);
  CHECK(tall.plinthApplied);
  CHECK_THAT(tall.planeZ, WithinAbs(-2.0, 1e-9));
  CHECK_THAT(tall.planeM, WithinAbs(-2.0 / 0.06, 1e-9));

  // From the lowest point, not from the level: heights 100 to 300 m at --scale 0.01 keep sea level and put the plane at
  // -2 m, -0.02 units, which is 1.02 units under the lowest point, 1 unit. The plinth puts it at 1 - 2 = -1 unit,
  // -100 m. (Measured from the level the plane would have gone to -2 units, -200 m.)
  const BasePlane hill = chooseBasePlane(100, 300, std::nullopt, 1, 0.01, 2);
  CHECK_FALSE(hill.loweredToLowest);
  CHECK(hill.plinthApplied);
  CHECK_THAT(hill.planeZ, WithinAbs(-1.0, 1e-9));
  CHECK_THAT(hill.planeM, WithinAbs(-100.0, 1e-9));

  // A plinth of 0 never applies: the margin is at least 1 m.
  CHECK_FALSE(chooseBasePlane(0, 1086, std::nullopt, 1, 0.02, 0).plinthApplied);
}
