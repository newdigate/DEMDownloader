#include "demdl/WmsQuery.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <optional>
#include <stdexcept>

namespace demdl {

namespace {

// The character tests are ASCII only, so the result does not depend on the locale: bytes at or
// above 0x80 are never unreserved and are always percent-encoded.
bool isUnreserved(unsigned char c) {
  return (c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') || (c >= '0' && c <= '9') || c == '-' ||
         c == '.' || c == '_' || c == '~';
}

// Whitespace as isspace defines it in the C locale, without consulting the locale.
bool isWhitespace(char c) {
  return c == ' ' || c == '\t' || c == '\n' || c == '\v' || c == '\f' || c == '\r';
}

// The number that starts `rest`, the text after a band marker, or nothing when there is no usable
// one. Only the marker's own line counts.
std::optional<double> leadingNumber(std::string_view rest) {
  const auto lineEnd = rest.find_first_of("\r\n");
  // strtod needs a null-terminated string; libc++ on AppleClang 15 has no floating-point from_chars.
  const std::string token(rest.substr(0, lineEnd));
  const char* begin = token.c_str();
  char* end = nullptr;
  const double value = std::strtod(begin, &end);
  // The number must be followed by whitespace, '<' or the end of the body: no number contains '<', so
  // markup such as "</pre>" may end one. strtod stops at the first character it cannot use, so "12,5",
  // "1.#QNAN" and "12abc" would otherwise give 12, 1 and 12 m, and a mask of "0abc" NoData, all stored
  // for good and never fetched again; as a parse error the pixel is retried.
  const bool ended = end == begin + token.size() || isWhitespace(*end) || *end == '<';
  // strtod also accepts "nan" and "inf". Neither is an elevation, and a NaN would read as an
  // unfetched pixel and be requested again on every run. The raster stores float32, so values
  // above FLT_MAX are refused too. A handful just above would still round to FLT_MAX; that does
  // not matter, because no elevation is anywhere near 1e38.
  if (end == begin || !ended || !std::isfinite(value) ||
      std::fabs(value) > std::numeric_limits<float>::max()) {
    return std::nullopt;
  }
  return value;
}

// The text after the first `marker` that starts a line, at the start of the body or right after a
// '\n', or nothing when no line starts with it.
std::optional<std::string_view> afterLineStart(std::string_view body, std::string_view marker) {
  for (auto pos = body.find(marker); pos != std::string_view::npos; pos = body.find(marker, pos + 1)) {
    if (pos == 0 || body[pos - 1] == '\n') return body.substr(pos + marker.size());
  }
  return std::nullopt;
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
  static constexpr std::string_view heightMarker = "BAND 0:";
  static constexpr std::string_view maskMarker = "BAND 1:";
  // The server sends BAND 0 on land and at sea alike (see the recorded responses in
  // tests/test_WmsQuery.cpp), so a body without it is no answer: an empty one, an HTML error page, an
  // exception report or a line such as "Error: layer not found". NoData is stored as -32768 and never
  // requested again, so such a body is a parse error, and the pixel is retried.
  const auto pos = body.find(heightMarker);
  if (pos == std::string_view::npos) return {ParseResult::Kind::ParseError, 0.0};
  const std::optional<double> height = leadingNumber(body.substr(pos + heightMarker.size()));
  if (!height) return {ParseResult::Kind::ParseError, 0.0};
  // BAND 1 is the layer's mask, 255 where there is an elevation and 0 where there is none. At sea the
  // server answers "BAND 0: 0" with "BAND 1: 0", so the height alone would store a real 0 m for every
  // sea pixel, for good. Only a line of its own counts, so the marker quoted inside another line cannot
  // mark a pixel as no data, and a mask that does not parse is a parse error, retried, because a false
  // NoData is permanent. Without a mask line the height stands, so layers and servers without a mask
  // band keep working.
  const std::optional<std::string_view> maskText = afterLineStart(body, maskMarker);
  if (!maskText) return {ParseResult::Kind::Value, *height};
  const std::optional<double> mask = leadingNumber(*maskText);
  if (!mask) return {ParseResult::Kind::ParseError, 0.0};
  if (*mask == 0.0) return {ParseResult::Kind::NoData, 0.0};
  return {ParseResult::Kind::Value, *height};
}

}  // namespace demdl
