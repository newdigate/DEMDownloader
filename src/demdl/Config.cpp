#include "demdl/Config.h"

namespace demdl {

std::optional<WmsEndpoint> parseWmsUrl(const std::string& url) {
  const auto schemeEnd = url.find("://");
  if (schemeEnd == std::string::npos) return std::nullopt;
  const std::string scheme = url.substr(0, schemeEnd);
  if (scheme != "http" && scheme != "https") return std::nullopt;
  const std::string rest = url.substr(schemeEnd + 3);
  const auto slash = rest.find('/');
  const std::string hostPort = rest.substr(0, slash);
  // The authority must be a non-empty host with an optional port. A query or fragment without a
  // path is refused rather than swallowed into the origin.
  if (hostPort.empty() || hostPort.front() == ':' || hostPort.back() == ':' ||
      hostPort.find_first_of(" \t?#") != std::string::npos) {
    return std::nullopt;
  }
  WmsEndpoint ep;
  ep.origin = scheme + "://" + hostPort;
  ep.path = slash == std::string::npos ? std::string("/") : rest.substr(slash);
  return ep;
}

std::string Config::validate() const {
  if (const std::string err = box.validate(); !err.empty()) return err;
  if (width <= 0 || height <= 0) return "width and height must be positive";
  if (output.empty()) return "output path must not be empty";
  if (!parseWmsUrl(wmsUrl)) {
    return "wms-url must look like scheme://host[:port]/path with scheme http or https";
  }
  if (layer.empty()) return "layer must not be empty";
  if (threads < 1) return "threads must be at least 1";
  if (maxRetries < 0) return "max-retries must be at least 0";
  if (maxConsecutiveFailures < 1) return "max-consecutive-failures must be at least 1";
  return {};
}

}  // namespace demdl
