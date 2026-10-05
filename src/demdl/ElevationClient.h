#pragma once

#include <atomic>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <string_view>

#include "demdl/Config.h"
#include "demdl/PixelFetcher.h"
#include "demdl/RetryPolicy.h"

namespace httplib {
class Client;
}

namespace demdl {

// Fetches one pixel's elevation over HTTP or HTTPS with keep-alive, retries and backoff.
// One instance per worker thread; instances are not thread-safe.
//
// Requests identify themselves as "dl_elevation/<version>". Timeouts: 10 s to connect, 10 s per
// write, 30 s per read and 60 s overall per request, so a server trickling its answer cannot hold a
// worker indefinitely. A stop cannot interrupt a request already on the wire. Once connected, a
// request ends within the 60 s overall cap, which counts from the start of the request; resolving,
// connecting (per address) and each TLS handshake wait are bounded separately by the 10 s connection
// timeout. The client relies on the process ignoring SIGPIPE, as main arranges: httplib's client
// sets neither SO_NOSIGPIPE nor MSG_NOSIGNAL. Two warnings go to stderr, each once per process across
// all instances: the error of the first failed attempt, marked "(retrying)" or "(not retried)", and
// the first unparseable body.
class ElevationClient : public PixelFetcher {
 public:
  // Uses cfg.wmsUrl, layer, box, width, height, insecure and caBundle. The box and the grid must
  // describe the raster being filled, which main's matches() check guarantees. cfg.maxRetries is not
  // read: the policy carries it, and main maps one onto the other.
  // Throws std::invalid_argument when cfg.wmsUrl does not parse, or when httplib cannot create a
  // client for its origin (is_valid() is false, for example without a TLS context).
  ElevationClient(const Config& cfg, RetryPolicy policy);
  ~ElevationClient() override;

  // Failed attempts are retried per the policy, except certificate errors (a CA bundle that cannot
  // be loaded, a chain that does not verify, a certificate that does not cover the host): those fail
  // at once, because only a configuration change cures them. A CA bundle that failed to load stays
  // failed: httplib loads it once per client, so every later fetch returns the same error, with
  // attempts == 1, without sending a request. An exception during an attempt becomes that attempt's
  // error ("exception: ..." or "unknown exception") and is retried like any other failure. Not
  // noexcept: building error text allocates.
  [[nodiscard]] FetchResult fetch(int x, int y, const std::atomic<bool>& stop) override;

 private:
  Config cfg_;
  RetryPolicy policy_;
  std::unique_ptr<httplib::Client> client_;
  std::mt19937 rng_;
  std::uniform_real_distribution<double> unit_{0.0, 1.0};
  std::optional<std::string> caBundleFailure_;  // the error of a failed CA-bundle load, once one failed
};

// The warning line for the first unparseable response body, without a trailing newline. A blank body
// is reported by its size, any other by its first 200 bytes with every byte outside printable ASCII
// shown as '.', so that the warning stays on one line and cannot drive the terminal. Exposed for
// tests.
[[nodiscard]] std::string describeUnparseableBody(std::string_view body);

}  // namespace demdl
