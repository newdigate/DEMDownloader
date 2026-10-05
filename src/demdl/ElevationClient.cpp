#include "demdl/ElevationClient.h"

#include <httplib.h>

#include <algorithm>
#include <exception>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <thread>

#include "demdl/WmsQuery.h"

namespace demdl {

ElevationClient::ElevationClient(const Config& cfg, RetryPolicy policy)
    : cfg_(cfg), policy_(policy), rng_(std::random_device{}()) {
  const auto endpoint = parseWmsUrl(cfg_.wmsUrl);
  if (!endpoint) throw std::invalid_argument("invalid WMS URL: " + cfg_.wmsUrl);
  client_ = std::make_unique<httplib::Client>(endpoint->origin);
  // httplib reports a client it could not set up through is_valid(); the calls below could then
  // dereference a null implementation.
  if (!client_->is_valid()) throw std::invalid_argument("invalid WMS origin: " + endpoint->origin);
  client_->set_keep_alive(true);
  client_->set_connection_timeout(10, 0);
  client_->set_read_timeout(30, 0);
  client_->set_write_timeout(10, 0);
  client_->enable_server_certificate_verification(!cfg_.insecure);
  if (!cfg_.caBundle.empty()) client_->set_ca_cert_path(cfg_.caBundle);
}

ElevationClient::~ElevationClient() = default;

FetchResult ElevationClient::fetch(int x, int y, const std::atomic<bool>& stop) {
  FetchResult result;
  for (int attempt = 0;; ++attempt) {
    result.attempts = attempt + 1;
    std::string error;
    // fetch must not throw (see PixelFetcher), so building the target, the request and the parse
    // run inside the try, and an exception is this attempt's error.
    try {
      const httplib::Result response = client_->Get(featureInfoUrl(cfg_, x, y));
      if (!response) {
        error = "transport: " + httplib::to_string(response.error());
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
    if (attempt >= policy_.maxRetries || stop.load()) {
      result.kind = FetchResult::Kind::Failed;
      result.error = error;
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

bool ElevationClient::sleepUnlessStopped(std::chrono::milliseconds delay, const std::atomic<bool>& stop) {
  std::chrono::milliseconds remaining = delay;
  while (remaining.count() > 0) {
    if (stop.load()) return false;
    const std::chrono::milliseconds slice = std::min(remaining, std::chrono::milliseconds(100));
    std::this_thread::sleep_for(slice);
    remaining -= slice;
  }
  return !stop.load();
}

void ElevationClient::logUnparseableBodyOnce(const std::string& body) {
  static std::once_flag flag;
  std::call_once(flag, [&body] {
    std::string line = "warning: first unparseable response body ";
    if (body.find_first_not_of(" \t\n\v\f\r") == std::string::npos) {
      // Printing blank text would show nothing, so give the size instead.
      line += "was " + std::to_string(body.size()) + " bytes of whitespace";
    } else {
      // ASCII control characters (below 0x20, and DEL) become '.', so the warning stays on one line
      // and cannot drive the terminal; bytes from 0x80 up pass through to keep UTF-8 readable.
      std::string shown = body.substr(0, 200);
      for (char& c : shown) {
        const auto byte = static_cast<unsigned char>(c);
        if (byte < 0x20 || byte == 0x7F) c = '.';
      }
      line += "(" + std::to_string(body.size()) + " bytes, up to 200 shown): " + shown;
    }
    // stderr, so the warning never mixes with the progress lines on stdout.
    std::cerr << line + "\n";
  });
}

}  // namespace demdl
