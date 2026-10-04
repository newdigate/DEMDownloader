#pragma once

#include <string>
#include <string_view>

#include "demdl/Config.h"

namespace demdl {

struct ParseResult {
  enum class Kind { Value, NoData, ParseError };
  Kind kind;
  double value = 0.0;  // valid when kind == Value
};

// RFC 3986 percent-encoding: unreserved characters pass through, everything else becomes %XX.
std::string percentEncode(std::string_view s);

// Degrees with nine fixed decimals (about 0.1 mm), used in URLs and user-facing reports alike.
std::string formatCoordinate(double degrees);

// Request target (path plus query) of the WMS 1.3.0 GetFeatureInfo request for pixel column x, row y.
// The box is sent as south,west,north,east, the axis order WMS 1.3.0 requires for EPSG:4326.
// Throws std::invalid_argument when cfg.wmsUrl does not parse.
std::string featureInfoUrl(const Config& cfg, int x, int y);

// "BAND 0: <number>" anywhere in the body is a value, whatever else the body says. Without the
// marker, a body containing "ServiceException" (an XML exception report, which WMS servers send
// with HTTP 200 for faults such as an unknown layer) is ParseError so that the caller retries it;
// any other body is NoData. A marker without a parseable number is ParseError, and so is a
// non-finite one such as "nan" or "inf" and one outside float range, because the raster stores
// float32; a Value therefore always fits in a float.
ParseResult parseElevation(std::string_view body);

}  // namespace demdl
