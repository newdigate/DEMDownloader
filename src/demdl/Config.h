#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "demdl/BoundingBox.h"

namespace demdl {

struct WmsEndpoint {
  std::string origin;  // "scheme://host[:port]", what httplib::Client is constructed with
  std::string path;    // whatever followed the authority, as given; "/" when the URL had none
};

// Splits "scheme://host[:port][/path]". The scheme is http or https (case-sensitive), the host is a
// hostname (A-Za-z0-9._-) or a bracketed IPv6 literal, and the port, if given, is 1 to 65535.
// Userinfo, queries, fragments, whitespace and control characters are refused, and so is anything
// else that does not fit: the result is then nullopt.
[[nodiscard]] std::optional<WmsEndpoint> parseWmsUrl(const std::string& url);

// Every runtime option, mirroring the command-line flags one to one.
// No command-line parsing lives here.
struct Config {
  BoundingBox box;
  int width = 0;
  int height = 0;
  std::filesystem::path output;  // relative to the current directory; must name a file
  std::string wmsUrl = "https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets";
  std::string layer = "Elevation_CCT_GLM2019_5m_DEM";
  int threads = 8;     // capped at 64 to go easy on a public server
  int maxRetries = 8;  // retries after the first attempt; 0 means a single attempt
  int maxConsecutiveFailures = 100;
  std::string caBundle;  // empty means the system trust store; no effect when insecure is set
  bool insecure = false;
  bool preview = false;
  bool overwrite = false;
  bool dryRun = false;
  bool verbose = false;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  [[nodiscard]] std::string validate() const;
};

}  // namespace demdl
