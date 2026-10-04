#pragma once

#include <string>

namespace demdl {

struct PixelScale {
  double lonPerPixel;
  double latPerPixel;
};

// A WGS84 box in decimal degrees. north > south and east > west when valid.
struct BoundingBox {
  double north = 0.0;
  double south = 0.0;
  double east = 0.0;
  double west = 0.0;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  std::string validate() const;
  PixelScale pixelScale(int width, int height) const;
  bool approxEquals(const BoundingBox& other, double tolerance) const;
};

}  // namespace demdl
