#pragma once

#include <optional>

namespace demdl::mesh {

// Where the flat bottom of the solid goes (spec 5.3), in metres before exaggeration and in model units:
// - levelM is the level the terrain is closed down to: the base asked for (--base), or sea level, 0, when none was;
//   lowered to the window's lowest height when it is above it, so that no ground hangs below the base, and then
//   loweredToLowest is set. surfaceMesh lifts any height below it to it.
// - marginM is how far below the level the plane is put, so that no wall has zero height where the terrain touches the
//   level: 1 percent of the window's height range, at least 1 m.
// - planeM and planeZ are the plane, levelM - marginM, in metres and in model units (times the z-scale, then times
//   the scale). planeM is what the run reports; planeZ is where solidMesh puts the base.
// - plinthApplied is set when the solid would otherwise be thinner than the plinth, in model units, under its lowest
//   point. The plane is then pushed down to exactly that thickness, planeZ = lowest x zScale x scale - plinth, and
//   planeM is that plane in metres again. In a Table Mountain window at --scale 0.02 the margin alone puts the base
//   0.2 mm under the sea, which would print one layer thick.
struct BasePlane {
  double levelM = 0.0;
  double marginM = 0.0;
  double planeM = 0.0;
  double planeZ = 0.0;
  bool loweredToLowest = false;
  bool plinthApplied = false;
};

// lowestM and highestM are the window's lowest and highest heights after the no-data fill, and requestedM is --base
// when it was given. zScale and scale are above 0 and plinth is 0 or more, all finite, as the command line makes them,
// so nothing here can fail. A plinth of 0 never applies, the margin being at least 1 m.
[[nodiscard]] BasePlane chooseBasePlane(double lowestM, double highestM, std::optional<double> requestedM,
                                        double zScale, double scale, double plinth) noexcept;

}  // namespace demdl::mesh
