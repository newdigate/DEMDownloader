#pragma once

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include "demdl/PixelFetcher.h"

// Blocks every caller of wait() until release() is called, and counts how many have arrived;
// awaitArrivals() lets a test wait for that count without spinning.
struct Gate {
  std::mutex m;
  std::condition_variable cv;
  bool open = false;
  std::atomic<int> waiting{0};

  void wait() {
    std::unique_lock<std::mutex> lock(m);
    ++waiting;  // under the lock, so awaitArrivals() cannot miss the notification below
    cv.notify_all();
    cv.wait(lock, [this] { return open; });
  }
  void release() {
    {
      std::lock_guard<std::mutex> lock(m);
      open = true;
    }
    cv.notify_all();
  }
  // True once at least n callers have reached wait(), false if the timeout passes first.
  [[nodiscard]] bool awaitArrivals(int n, std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(m);
    return cv.wait_for(lock, timeout, [this, n] { return waiting.load() >= n; });
  }
};

// Answers fetches from a lambda; optionally blocks on a Gate, counts calls, and counts the calls
// that find `stop` set once the gate has opened.
struct FakeFetcher : demdl::PixelFetcher {
  std::function<demdl::FetchResult(int x, int y)> behaviour;
  std::atomic<int>* calls = nullptr;
  std::atomic<int>* sawStop = nullptr;
  Gate* gate = nullptr;

  [[nodiscard]] demdl::FetchResult fetch(int x, int y, const std::atomic<bool>& stop) override {
    if (gate != nullptr) gate->wait();
    if (sawStop != nullptr && stop.load()) ++*sawStop;
    if (calls != nullptr) ++*calls;
    return behaviour(x, y);
  }
};

[[nodiscard]] inline demdl::FetchResult valueResult(float value) {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::Value;
  r.value = value;
  r.attempts = 1;
  return r;
}

[[nodiscard]] inline demdl::FetchResult noDataResult() {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::NoData;
  r.attempts = 1;
  return r;
}

[[nodiscard]] inline demdl::FetchResult failedResult(std::string error) {
  demdl::FetchResult r;
  r.kind = demdl::FetchResult::Kind::Failed;
  r.attempts = 3;
  r.error = std::move(error);
  return r;
}
