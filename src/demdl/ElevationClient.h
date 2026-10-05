#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <random>
#include <string>

#include "demdl/Config.h"
#include "demdl/PixelFetcher.h"
#include "demdl/RetryPolicy.h"

namespace httplib {
class Client;
}

namespace demdl {

// Fetches one pixel's elevation over HTTP or HTTPS with keep-alive, retries and backoff.
// One instance per worker thread; instances are not thread-safe.
class ElevationClient : public PixelFetcher {
 public:
  // Throws std::invalid_argument when cfg.wmsUrl does not parse, or when httplib cannot set up a
  // client for its origin (is_valid() is false, for example without a TLS context).
  ElevationClient(const Config& cfg, RetryPolicy policy);
  ~ElevationClient() override;

  // An exception during an attempt becomes that attempt's error ("exception: ..." or "unknown
  // exception") and is retried like any other failure. Not noexcept: building error text allocates.
  [[nodiscard]] FetchResult fetch(int x, int y, const std::atomic<bool>& stop) override;

 private:
  // Sleeps in 100 ms slices; returns false as soon as stop is set.
  static bool sleepUnlessStopped(std::chrono::milliseconds delay, const std::atomic<bool>& stop);
  // Writes the first unparseable body of the process to stderr, once across all instances: a blank
  // body as its byte count, any other as its first 200 bytes with control characters shown as '.'.
  static void logUnparseableBodyOnce(const std::string& body);

  Config cfg_;
  RetryPolicy policy_;
  std::unique_ptr<httplib::Client> client_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
};

}  // namespace demdl
