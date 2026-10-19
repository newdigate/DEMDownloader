#include "mesh/BasePlane.h"

#include <algorithm>

namespace demdl::mesh {

BasePlane chooseBasePlane(double lowestM, double highestM, std::optional<double> requestedM, double zScale,
                          double scale, double plinth) noexcept {
  BasePlane base;
  base.levelM = requestedM.value_or(0.0);
  if (base.levelM > lowestM) {
    base.levelM = lowestM;
    base.loweredToLowest = true;
  }
  base.marginM = std::max(1.0, 0.01 * (highestM - lowestM));
  base.planeM = base.levelM - base.marginM;
  base.planeZ = base.planeM * zScale * scale;

  // The thickness is judged where the print is thinnest, under the lowest point, and in model units, the units the
  // slicer prints: a margin of metres can be a fraction of a millimetre once scaled.
  const double lowestZ = lowestM * zScale * scale;
  if (lowestZ - base.planeZ < plinth) {
    base.planeZ = lowestZ - plinth;
    base.planeM = base.planeZ / (zScale * scale);
    base.plinthApplied = true;
  }
  return base;
}

}  // namespace demdl::mesh
