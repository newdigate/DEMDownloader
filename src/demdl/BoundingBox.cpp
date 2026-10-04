#include "demdl/BoundingBox.h"

#include <cmath>

namespace demdl {

std::string BoundingBox::validate() const {
  if (!(std::isfinite(north) && std::isfinite(south) && std::isfinite(east) && std::isfinite(west))) {
    return "bounding box values must be finite numbers";
  }
  if (north < -90.0 || north > 90.0 || south < -90.0 || south > 90.0) {
    return "latitudes must be within [-90, 90]";
  }
  if (east < -180.0 || east > 180.0 || west < -180.0 || west > 180.0) {
    return "longitudes must be within [-180, 180]";
  }
  if (!(north > south)) return "north must be greater than south";
  if (!(east > west)) return "east must be greater than west";
  return {};
}

PixelScale BoundingBox::pixelScale(int width, int height) const {
  return {(east - west) / width, (north - south) / height};
}

bool BoundingBox::approxEquals(const BoundingBox& other, double tolerance) const {
  return std::fabs(north - other.north) <= tolerance && std::fabs(south - other.south) <= tolerance &&
         std::fabs(east - other.east) <= tolerance && std::fabs(west - other.west) <= tolerance;
}

}  // namespace demdl
