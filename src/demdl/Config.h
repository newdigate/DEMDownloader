#pragma once

#include <filesystem>
#include <optional>
#include <string>

#include "demdl/BoundingBox.h"

namespace demdl {

struct WmsEndpoint {
  std::string origin;  // "https://host[:port]", what httplib::Client is constructed with
  std::string path;    // "/erdas-iws/ogc/wms/GeoSpatial%20Datasets", kept as given
};

// Splits "scheme://host[:port]/path". nullopt unless the scheme is http or https and a host is present.
std::optional<WmsEndpoint> parseWmsUrl(const std::string& url);

// Every runtime option. Mirrors the command-line flags one to one; no parsing lives here.
struct Config {
  BoundingBox box;
  int width = 0;
  int height = 0;
  std::filesystem::path output;
  std::string wmsUrl = "https://cityimg.capetown.gov.za/erdas-iws/ogc/wms/GeoSpatial%20Datasets";
  std::string layer = "Elevation_CCT_GLM2019_5m_DEM";
  int threads = 8;
  int maxRetries = 8;
  int maxConsecutiveFailures = 100;
  std::string caBundle;
  bool insecure = false;
  bool preview = false;
  bool overwrite = false;
  bool dryRun = false;
  bool verbose = false;

  // Empty when valid, otherwise the first problem found, phrased for the user.
  std::string validate() const;
};

}  // namespace demdl
