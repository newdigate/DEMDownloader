#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <string>

#include "FakeFetcher.h"
#include "RecordingSink.h"
#include "demdl/StopToken.h"

using demdl::StopReason;
using demdl::StopToken;

TEST_CASE("a stop token keeps the first reason", "[stoptoken]") {
  StopToken t;
  CHECK_FALSE(t.stop.load());
  CHECK(t.reason() == StopReason::None);
  t.request(StopReason::Signal);
  CHECK(t.stop.load());
  CHECK(t.reason() == StopReason::Signal);
  t.request(StopReason::WindowClosed);
  CHECK(t.reason() == StopReason::Signal);
}

TEST_CASE("describe names every reason", "[stoptoken]") {
  CHECK(std::string(demdl::describe(StopReason::None)) == "finished");
  CHECK(std::string(demdl::describe(StopReason::Signal)) == "interrupted");
  CHECK(std::string(demdl::describe(StopReason::WindowClosed)) == "preview window closed");
  CHECK(std::string(demdl::describe(StopReason::ConsecutiveFailures)) == "too many consecutive failures");
  CHECK(std::string(demdl::describe(StopReason::WriteError)) == "checkpoint write failed");
}

TEST_CASE("the test doubles compile and behave", "[stoptoken]") {
  FakeFetcher fetcher;
  std::atomic<int> calls{0};
  fetcher.calls = &calls;
  fetcher.behaviour = [](int x, int) { return valueResult(static_cast<float>(x)); };
  std::atomic<bool> stop{false};
  const demdl::FetchResult r = fetcher.fetch(3, 0, stop);
  CHECK(r.kind == demdl::FetchResult::Kind::Value);
  CHECK(r.value == 3.0f);
  CHECK(calls.load() == 1);

  RecordingSink sink;
  sink.onPixelState({1, 2, demdl::PixelState::Failed, 0.0f, 2, "boom"});
  sink.onPixelState({1, 3, demdl::PixelState::Value, 4.5f, 1, {}});
  CHECK(sink.count(demdl::PixelState::Failed) == 1);
  CHECK(sink.count(demdl::PixelState::Value) == 1);
  CHECK(sink.pixels.size() == 2);
}
