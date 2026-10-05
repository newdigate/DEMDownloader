#include <catch2/catch_test_macros.hpp>

#include <atomic>
#include <chrono>
#include <string>
#include <thread>

#include "FakeFetcher.h"
#include "RecordingSink.h"
#include "demdl/StopToken.h"

using demdl::StopReason;
using demdl::StopToken;

TEST_CASE("a stop token keeps the first reason", "[stoptoken]") {
  StopToken t;
  CHECK_FALSE(t.requested());
  CHECK(t.reason() == StopReason::None);
  t.request(StopReason::Signal);
  CHECK(t.requested());
  CHECK(t.reason() == StopReason::Signal);
  t.request(StopReason::WindowClosed);
  CHECK(t.requested());
  CHECK(t.reason() == StopReason::Signal);
}

TEST_CASE("the flag handed to fetchers follows the token", "[stoptoken]") {
  StopToken t;
  const std::atomic<bool>& flag = t.flag();
  CHECK_FALSE(flag.load());
  CHECK(flag.load() == t.requested());
  t.request(StopReason::WriteError);
  CHECK(flag.load());
  CHECK(flag.load() == t.requested());
}

TEST_CASE("describe names every reason", "[stoptoken]") {
  CHECK(std::string(demdl::describe(StopReason::None)) == "finished");
  CHECK(std::string(demdl::describe(StopReason::Signal)) == "interrupted");
  CHECK(std::string(demdl::describe(StopReason::WindowClosed)) == "preview window closed");
  CHECK(std::string(demdl::describe(StopReason::ConsecutiveFailures)) == "too many consecutive failures");
  CHECK(std::string(demdl::describe(StopReason::WriteError)) == "checkpoint write failed");
}

TEST_CASE("describe falls back for a value outside the enum", "[stoptoken]") {
  CHECK(std::string(demdl::describe(static_cast<StopReason>(99))) == "unknown");
}

TEST_CASE("the test doubles compile and behave", "[doubles]") {
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
  CHECK_FALSE(sink.plan.has_value());
  sink.onRunStarted({100, 40, 2, 5});
  REQUIRE(sink.plan.has_value());
  CHECK(sink.plan->pixelsTodo == 40);
  CHECK(sink.plan->rowsTotal == 5);
  sink.onPixelState({1, 2, demdl::PixelState::Failed, 0.0f, 2, "boom"});
  sink.onPixelState({1, 3, demdl::PixelState::Value, 4.5f, 1, {}});
  CHECK(sink.count(demdl::PixelState::Failed) == 1);
  CHECK(sink.count(demdl::PixelState::Value) == 1);
  CHECK(sink.pixels.size() == 2);
  const RecordingSink& view = sink;
  CHECK(view.count(demdl::PixelState::Failed) == 1);
}

TEST_CASE("a gate holds callers until released and reports arrivals", "[doubles]") {
  Gate gate;
  CHECK_FALSE(gate.awaitArrivals(1, std::chrono::milliseconds(10)));  // nobody has come yet

  std::atomic<int> calls{0};
  FakeFetcher first;
  FakeFetcher second;
  for (FakeFetcher* fetcher : {&first, &second}) {
    fetcher->gate = &gate;
    fetcher->calls = &calls;
    fetcher->behaviour = [](int, int) { return noDataResult(); };
  }
  std::atomic<bool> stop{false};
  demdl::FetchResult firstResult;
  demdl::FetchResult secondResult;
  std::thread a([&] { firstResult = first.fetch(1, 0, stop); });
  std::thread b([&] { secondResult = second.fetch(2, 0, stop); });

  const bool bothArrived = gate.awaitArrivals(2, std::chrono::seconds(5));
  const bool atLeastOne = gate.awaitArrivals(1, std::chrono::milliseconds(10));
  const bool thirdArrived = gate.awaitArrivals(3, std::chrono::milliseconds(10));
  const int callsWhileGated = calls.load();
  gate.release();
  a.join();
  b.join();

  CHECK(bothArrived);
  CHECK(atLeastOne);
  CHECK_FALSE(thirdArrived);
  CHECK(callsWhileGated == 0);
  CHECK(calls.load() == 2);
  CHECK(firstResult.kind == demdl::FetchResult::Kind::NoData);
  CHECK(secondResult.kind == demdl::FetchResult::Kind::NoData);
}

TEST_CASE("a fake fetcher counts the calls that see the stop flag set", "[doubles]") {
  StopToken token;
  std::atomic<bool> otherFlag{false};
  std::atomic<int> sawStop{0};
  FakeFetcher fetcher;
  fetcher.sawStop = &sawStop;
  fetcher.behaviour = [](int, int) { return noDataResult(); };

  CHECK(fetcher.fetch(0, 0, token.flag()).kind == demdl::FetchResult::Kind::NoData);
  CHECK(sawStop.load() == 0);
  token.request(StopReason::Signal);
  CHECK(fetcher.fetch(1, 0, token.flag()).kind == demdl::FetchResult::Kind::NoData);
  CHECK(sawStop.load() == 1);
  // A fetcher handed some other flag does not see the stop, which is how a test notices a
  // Downloader that passes the wrong one.
  CHECK(fetcher.fetch(2, 0, otherFlag).kind == demdl::FetchResult::Kind::NoData);
  CHECK(sawStop.load() == 1);
}

TEST_CASE("a fake fetcher reads the stop flag after the gate opens", "[doubles]") {
  StopToken token;
  Gate gate;
  std::atomic<int> sawStop{0};
  FakeFetcher fetcher;
  fetcher.gate = &gate;
  fetcher.sawStop = &sawStop;
  fetcher.behaviour = [](int, int) { return noDataResult(); };

  demdl::FetchResult result;
  std::thread worker([&] { result = fetcher.fetch(0, 0, token.flag()); });
  const bool arrived = gate.awaitArrivals(1, std::chrono::seconds(5));
  token.request(StopReason::WindowClosed);  // raised while the fetcher is held at the gate
  gate.release();
  worker.join();

  CHECK(arrived);
  CHECK(sawStop.load() == 1);
  CHECK(result.kind == demdl::FetchResult::Kind::NoData);
}
