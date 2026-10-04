#pragma once

#include <atomic>
#include <condition_variable>
#include <functional>
#include <mutex>
#include <string>
#include <utility>

#include "demdl/PixelFetcher.h"

// Blocks every caller of wait() until release() is called, and counts how many have arrived.
struct Gate {
  std::mutex m;
  std::condition_variable cv;
  bool open = false;
  std::atomic<int> waiting{0};

  void wait() {
    ++waiting;
    std::unique_lock<std::mutex> lock(m);
    cv.wait(lock, [this] { return open; });
  }
  void release() {
    {
      std::lock_guard<std::mutex> lock(m);
      open = true;
    }
    cv.notify_all();
  }
};

// Answers fetches from a lambda; optionally blocks on a Gate and counts calls.
struct FakeFetcher : demdl::PixelFetcher {
  std::function<demdl::FetchResult(int x, int y)> behaviour;
  std::atomic<int>* calls = nullptr;
  Gate* gate = nullptr;

  demdl::FetchResult fetch(int x, int y, const std::atomic<bool>&) override {
    if (gate != nullptr) gate->wait();
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
