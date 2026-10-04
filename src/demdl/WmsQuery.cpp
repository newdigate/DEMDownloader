#include "demdl/WmsQuery.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>

namespace demdl {

namespace {

// The character tests are ASCII only, so the result does not depend on the locale: bytes at or
// above 0x80 are never unreserved and are always percent-encoded.
bool isUnreserved(unsigned char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
         c == '.' || c == '_' || c == '~';
}

}  // namespace

std::string percentEncode(std::string_view s) {
  static constexpr char hex[] = "0123456789ABCDEF";
  std::string out;
  out.reserve(s.size());
  for (const unsigned char c : s) {
    if (isUnreserved(c)) {
      out.push_back(static_cast<char>(c));
    } else {
      out.push_back('%');
      out.push_back(hex[c >> 4]);
      out.push_back(hex[c & 0x0F]);
    }
  }
  return out;
}

std::string formatCoordinate(double degrees) {
  // snprintf, because std::to_chars for double is unavailable on this libc++ at the default
  // deployment target.
  char buf[64];
  std::snprintf(buf, sizeof buf, "%.9f", degrees);
  return buf;
}

std::string featureInfoUrl(const Config& cfg, int x, int y) {
  const auto ep = parseWmsUrl(cfg.wmsUrl);
  if (!ep) throw std::invalid_argument("invalid WMS URL: " + cfg.wmsUrl);
  const std::string layer = percentEncode(cfg.layer);
  std::string url = ep->path;
  url += "?SERVICE=WMS&VERSION=1.3.0&REQUEST=GetFeatureInfo&BBOX=";
  url += formatCoordinate(cfg.box.south) + "%2C" + formatCoordinate(cfg.box.west) + "%2C" +
         formatCoordinate(cfg.box.north) + "%2C" + formatCoordinate(cfg.box.east);
  url += "&CRS=EPSG%3A4326&WIDTH=" + std::to_string(cfg.width) + "&HEIGHT=" + std::to_string(cfg.height);
  url += "&LAYERS=" + layer + "&STYLES=&FORMAT=image%2Fpng&QUERY_LAYERS=" + layer;
  url += "&INFO_FORMAT=text%2Fplain&I=" + std::to_string(x) + "&J=" + std::to_string(y);
  return url;
}

ParseResult parseElevation(std::string_view body) {
  static constexpr std::string_view marker = "BAND 0:";
  const auto pos = body.find(marker);
  if (pos == std::string_view::npos) {
    // NoData is stored as -32768 and never re-requested, so a body that is not plainly "no band
    // here" must be retried instead: an empty or blank one, markup (an HTML error page or an OWS
    // exception report; text/plain was requested, so markup is never a valid answer) and anything
    // that mentions a ServiceException (WMS servers report faults such as an unknown layer so).
    const auto first = body.find_first_not_of(" \t\n\v\f\r");
    if (first == std::string_view::npos || body[first] == '<' ||
        body.find("ServiceException") != std::string_view::npos) {
      return {ParseResult::Kind::ParseError, 0.0};
    }
    return {ParseResult::Kind::NoData, 0.0};
  }
  const std::string_view rest = body.substr(pos + marker.size());
  const auto lineEnd = rest.find_first_of("\r\n");
  // strtod needs a null-terminated string; libc++ on AppleClang 15 has no floating-point from_chars.
  const std::string token(rest.substr(0, lineEnd));
  const char* begin = token.c_str();
  char* end = nullptr;
  const double value = std::strtod(begin, &end);
  // strtod also accepts "nan" and "inf". Neither is an elevation, and a NaN would read as an
  // unfetched pixel and be requested again on every run. The raster stores float32, so values
  // above FLT_MAX are refused too. A handful just above would still round to FLT_MAX; that does
  // not matter, because no elevation is anywhere near 1e38.
  if (end == begin || !std::isfinite(value) || std::fabs(value) > std::numeric_limits<float>::max()) {
    return {ParseResult::Kind::ParseError, 0.0};
  }
  return {ParseResult::Kind::Value, value};
}

}  // namespace demdl
