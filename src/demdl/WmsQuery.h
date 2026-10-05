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

// Classifies the text/plain body of a GetFeatureInfo response. A Value is stored for good, and so is
// NoData (as -32768, never requested again), while a ParseError is retried. The rules follow two
// answers recorded from the City of Cape Town server on 2026-10-05 (the fixtures in
// tests/test_WmsQuery.cpp), which carry the height as "BAND 0: <number>" and a mask as
// "BAND 1: <number>", each on a line of its own: 366.2 and 255 on land, 0 and 0 at sea.
// - Without "BAND 0:" the body is ParseError, whatever else it says: empty, an HTML error page, an
//   exception report or a line such as "Error: layer not found".
// - "BAND 0:" may stand anywhere in the body. The number that starts the rest of its line is the
//   height, and it must be followed by whitespace, '<' or the end of the body: trailing text such as
//   " m" or "</pre>" is ignored, but "12,5", "1.#QNAN" and "12abc" are ParseError. The number must be
//   on the marker's own line, finite and within float range (the raster stores float32, so a Value
//   always fits in a float); no number, "nan", "inf" and 1e300 are ParseError.
// - The first line that starts with "BAND 1:" (at the start of the body or right after a '\n' or a
//   '\r', the characters that end a line) is the mask, and its number is read by the same rules: 0 is
//   NoData, any other number gives the height as the Value, and a mask that does not parse is
//   ParseError, because a false NoData is permanent while a ParseError is only retried. "BAND 1:"
//   inside another line is ignored, and the search goes on past it. An unusable height is ParseError
//   whatever the mask says.
// - Without a mask line the height is the Value, so layers and servers without a mask band still work.
// Relies on the C locale for the decimal point; nothing in this program calls setlocale.
ParseResult parseElevation(std::string_view body);

}  // namespace demdl
