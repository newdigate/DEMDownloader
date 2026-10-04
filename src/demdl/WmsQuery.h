#pragma once

#include <string>
#include <string_view>

#include "demdl/Config.h"

namespace demdl {

struct ParseResult {
  enum class Kind { Value, NoData, ParseError };
  Kind kind = Kind::ParseError;  // so a value-initialised result is never a 0 m Value
  double value = 0.0;            // valid when kind == Value
};

// RFC 3986 percent-encoding: unreserved characters pass through, everything else becomes %XX.
std::string percentEncode(std::string_view s);

// Degrees with nine fixed decimals (about 0.1 mm), used in URLs and user-facing reports alike.
// Relies on the C locale for the decimal point; nothing in this program calls setlocale.
std::string formatCoordinate(double degrees);

// Request target (path plus query) of the WMS 1.3.0 GetFeatureInfo request for pixel column x, row y.
// The box is sent as south,west,north,east, the axis order WMS 1.3.0 requires for EPSG:4326.
// cfg.layer is taken unencoded and percent-encoded here, so a name that is already encoded would
// be double-encoded. Requires 0 <= x < cfg.width and 0 <= y < cfg.height; this is not checked,
// because the Downloader supplies raster indices and pool jobs must not throw.
// Throws std::invalid_argument when cfg.wmsUrl does not parse.
std::string featureInfoUrl(const Config& cfg, int x, int y);

// Classifies the text/plain body of a GetFeatureInfo response.
// "BAND 0:" anywhere in the body is a band, whatever else the body says. The numeric prefix of the
// rest of that line is the value: trailing text such as " m" is ignored, and the number must be on
// the marker's own line. It is a Value only when finite and within float range (the raster stores
// float32, so a Value always fits in a float); no number, "nan", "inf" and 1e300 are ParseError.
// Without the marker the body is NoData only when it is not blank (empty or only whitespace), does
// not contain "ServiceException" and does not start with markup ('<' as the first non-whitespace
// character). NoData is stored as -32768 and never re-requested, so any other body, such as an
// HTML error page or an OWS exception report, is ParseError and the caller retries it instead.
// Relies on the C locale for the decimal point; nothing in this program calls setlocale.
ParseResult parseElevation(std::string_view body);

}  // namespace demdl
