#include "demdl/Config.h"

#include <cstddef>
#include <cstdint>

namespace demdl {

namespace {

// The character tests are ASCII only, so the result does not depend on the locale.
bool isDigit(char c) { return c >= '0' && c <= '9'; }

bool isHexDigit(char c) { return isDigit(c) || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F'); }

// One or more characters from A-Za-z0-9._-
bool isHostname(const std::string& host) {
  if (host.empty()) return false;
  for (const char c : host) {
    const bool letter = (c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z');
    if (!letter && !isDigit(c) && c != '.' && c != '_' && c != '-') return false;
  }
  return true;
}

// "[", one or more hex digits or colons, "]".
bool isIpv6Literal(const std::string& host) {
  if (host.size() < 3 || host.front() != '[' || host.back() != ']') return false;
  for (std::size_t i = 1; i + 1 < host.size(); ++i) {
    if (!isHexDigit(host[i]) && host[i] != ':') return false;
  }
  return true;
}

// 1 to 5 decimal digits with a value from 1 to 65535.
bool isPort(const std::string& text) {
  if (text.empty() || text.size() > 5) return false;
  int value = 0;
  for (const char c : text) {
    if (!isDigit(c)) return false;
    value = value * 10 + (c - '0');
  }
  return value >= 1 && value <= 65535;
}

// "host" or "host:port". The port follows the first ':' that is not inside the brackets of an
// IPv6 literal.
bool isAuthority(const std::string& authority) {
  std::size_t hostEnd = 0;
  if (!authority.empty() && authority.front() == '[') {
    const auto close = authority.find(']');
    if (close == std::string::npos) return false;
    hostEnd = close + 1;
  } else {
    hostEnd = authority.find(':');
    if (hostEnd == std::string::npos) hostEnd = authority.size();
  }
  const std::string host = authority.substr(0, hostEnd);
  if (!isIpv6Literal(host) && !isHostname(host)) return false;
  if (hostEnd == authority.size()) return true;
  return authority[hostEnd] == ':' && isPort(authority.substr(hostEnd + 1));
}

}  // namespace

std::optional<WmsEndpoint> parseWmsUrl(const std::string& url) {
  // Grammar: "http" or "https", "://", a host with an optional ":port", then nothing or a path
  // starting with "/". The client appends its own query to the path, so a query or fragment in the
  // URL is refused, as are userinfo, whitespace and control characters.
  for (const char c : url) {
    const auto byte = static_cast<unsigned char>(c);
    if (byte <= 0x20 || byte == 0x7F || c == '?' || c == '#') return std::nullopt;
  }
  const auto schemeEnd = url.find("://");
  if (schemeEnd == std::string::npos) return std::nullopt;
  const std::string scheme = url.substr(0, schemeEnd);
  if (scheme != "http" && scheme != "https") return std::nullopt;
  const std::string rest = url.substr(schemeEnd + 3);
  const auto slash = rest.find('/');
  const std::string authority = rest.substr(0, slash);
  if (!isAuthority(authority)) return std::nullopt;
  WmsEndpoint ep;
  ep.origin = scheme + "://" + authority;
  ep.path = slash == std::string::npos ? std::string("/") : rest.substr(slash);
  return ep;
}

std::string Config::validate() const {
  if (const std::string err = box.validate(); !err.empty()) return err;
  if (width <= 0 || height <= 0) return "width and height must be positive";
  if (std::int64_t{width} * height > kMaxPixels) {
    return "width x height must be at most " + std::to_string(kMaxPixels) +
           " pixels (2 GiB of float32; a classic TIFF file holds at most 4 GiB)";
  }
  if (output.empty()) return "output path must not be empty";
  // "." and ".." name directories even without a trailing separator.
  if (!output.has_filename() || output.filename() == "." || output.filename() == "..") {
    return "output must name a file";
  }
  if (!parseWmsUrl(wmsUrl)) {
    return "wms-url must be http:// or https:// followed by a host name, an IPv4 address or an "
           "[IPv6 address], an optional :port (1 to 65535) and an optional path; user names, queries (?), "
           "fragments (#), spaces and control characters are not allowed";
  }
  if (layer.empty()) return "layer must not be empty";
  if (layer.find(';') != std::string::npos) return "layer must not contain ';'";
  if (threads < 1) return "threads must be at least 1";
  if (threads > 64) return "threads must be at most 64";
  if (maxRetries < 0) return "max-retries must be at least 0";
  if (maxRetries > 100) return "max-retries must be at most 100";
  if (maxConsecutiveFailures < 1) return "max-consecutive-failures must be at least 1";
  return {};
}

}  // namespace demdl
