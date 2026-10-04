#pragma once

#include <string>

namespace demdl {

struct PixelScale {
  double lonPerPixel = 0.0;
  double latPerPixel = 0.0;
};

// A WGS84 box in decimal degrees. north > south and east > west when valid.
struct BoundingBox {
  double north = 0.0;
  double south = 0.0;
  double east = 0.0;
  double west = 0.0;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  [[nodiscard]] std::string validate() const;
  // Degrees per pixel for a grid of the given size. width and height must be positive; callers
  // validate them (Config::validate, DemRaster::create). Not checked here.
  PixelScale pixelScale(int width, int height) const;
  // Absolute, inclusive, per-edge tolerance in degrees. A NaN edge never compares equal.
  bool approxEquals(const BoundingBox& other, double tolerance) const;
};

}  // namespace demdl
