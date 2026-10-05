#include "demdl/ElevationClient.h"

#include <httplib.h>

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <utility>

#include "demdl/WmsQuery.h"

namespace demdl {

namespace {

// Transport errors that no retry can cure, only a configuration change: the CA bundle cannot be
// loaded, the chain does not verify, or the certificate does not cover the host. Retrying would
// also hide the cause: httplib loads the bundle once per client, so a retry after a failed load
// reports the verification failure that follows instead.
bool needsConfigurationChange(httplib::Error error) {
  return error == httplib::Error::SSLLoadingCerts || error == httplib::Error::SSLServerVerification ||
         error == httplib::Error::SSLServerHostnameVerification;
}

// Sleeps in 100 ms slices; returns false as soon as stop is set.
bool sleepUnlessStopped(std::chrono::milliseconds delay, const std::atomic<bool>& stop) {
  std::chrono::milliseconds remaining = delay;
  while (remaining.count() > 0) {
    if (stop.load()) return false;
    const std::chrono::milliseconds slice = std::min(remaining, std::chrono::milliseconds(100));
    std::this_thread::sleep_for(slice);
    remaining -= slice;
  }
  return !stop.load();
}

// Writes the error of the first failed attempt in the process, once across all instances, so that a
// wrong path, a bad CA bundle or an unverifiable chain shows within seconds even without --verbose.
void logFirstFailureOnce(const std::string& error) noexcept {
  static std::once_flag flag;
  try {
    // stderr, so it cannot corrupt the progress lines on stdout.
    std::call_once(flag, [&error] { std::cerr << "warning: first failed request: " + error + "\n"; });
  } catch (...) {
    // A warning must never become a failure of its own.
  }
}

// Writes the first unparseable body of the process, once across all instances.
void logUnparseableBodyOnce(std::string_view body) noexcept {
  static std::once_flag flag;
  try {
    // stderr, so it cannot corrupt the progress lines on stdout.
    std::call_once(flag, [body] { std::cerr << describeUnparseableBody(body) + "\n"; });
  } catch (...) {
    // A warning must never become a failure of its own.
  }
}

}  // namespace

std::string describeUnparseableBody(std::string_view body) {
  const std::string prefix = "warning: first unparseable response body ";
  if (body.find_first_not_of(" \t\n\v\f\r") == std::string_view::npos) {
    // Blank text would print as nothing, so give the size instead.
    return prefix + "was " + std::to_string(body.size()) + " bytes of whitespace";
  }
  // Every byte outside printable ASCII becomes '.', so the warning stays on one line and only
  // printable ASCII reaches the terminal: neither control characters nor UTF-8-encoded C1 controls.
  std::string shown(body.substr(0, 200));
  for (char& c : shown) {
    const auto byte = static_cast<unsigned char>(c);
    if (byte < 0x20 || byte > 0x7E) c = '.';
  }
  return prefix + "(" + std::to_string(body.size()) + " bytes, up to 200 shown): " + shown;
}

ElevationClient::ElevationClient(const Config& cfg, RetryPolicy policy)
    : cfg_(cfg), policy_(policy), rng_(std::random_device{}()) {
  const auto endpoint = parseWmsUrl(cfg_.wmsUrl);
  if (!endpoint) throw std::invalid_argument("invalid WMS URL: " + cfg_.wmsUrl);
  client_ = std::make_unique<httplib::Client>(endpoint->origin);
  // httplib reports a client it could not set up through is_valid(); the calls below could then
  // dereference a null implementation.
  if (!client_->is_valid()) {
    throw std::invalid_argument("cannot create an HTTP client for origin: " + endpoint->origin);
  }
  client_->set_default_headers({{"User-Agent", "dl_elevation/" DL_ELEVATION_VERSION}});
  client_->set_keep_alive(true);
  client_->set_connection_timeout(10, 0);
  client_->set_read_timeout(30, 0);
  client_->set_write_timeout(10, 0);
  client_->set_max_timeout(60 * 1000);  // milliseconds, counted from the start of each request
  client_->enable_server_certificate_verification(!cfg_.insecure);
  if (!cfg_.caBundle.empty()) client_->set_ca_cert_path(cfg_.caBundle);
}

ElevationClient::~ElevationClient() = default;

FetchResult ElevationClient::fetch(int x, int y, const std::atomic<bool>& stop) {
  FetchResult result;
  for (int attempt = 0;; ++attempt) {
    result.attempts = attempt + 1;
    std::string error;
    bool retryable = true;
    // fetch must not throw (see PixelFetcher), so building the target, the request and the parse
    // run inside the try, and an exception is this attempt's error.
    try {
      const httplib::Result response = client_->Get(featureInfoUrl(cfg_, x, y));
      if (!response) {
        error = "transport: " + httplib::to_string(response.error());
        retryable = !needsConfigurationChange(response.error());
      } else if (response->status != 200) {
        error = "http " + std::to_string(response->status);
      } else {
        const ParseResult parsed = parseElevation(response->body);
        if (parsed.kind == ParseResult::Kind::Value) {
          result.kind = FetchResult::Kind::Value;
          result.value = static_cast<float>(parsed.value);
          return result;
        }
        if (parsed.kind == ParseResult::Kind::NoData) {
          result.kind = FetchResult::Kind::NoData;
          return result;
        }
        error = "unparseable body";
        logUnparseableBodyOnce(response->body);
      }
    } catch (const std::exception& e) {
      error = std::string("exception: ") + e.what();
    } catch (...) {
      error = "unknown exception";
    }
    logFirstFailureOnce(error);
    if (!retryable || attempt >= policy_.maxRetries || stop.load()) {
      result.kind = FetchResult::Kind::Failed;
      result.error = std::move(error);
      return result;
    }
    const std::chrono::milliseconds delay = policy_.delayFor(attempt, unit_(rng_));
    if (!sleepUnlessStopped(delay, stop)) {
      result.kind = FetchResult::Kind::Failed;
      result.error = "stopped during backoff after: " + error;
      return result;
    }
  }
}

}  // namespace demdl
