#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <thread>

#include "FakeFetcher.h"
#include "RecordingSink.h"
#include "TempDir.h"
#include "demdl/Downloader.h"

using namespace std::chrono_literals;
using Catch::Matchers::ContainsSubstring;
using demdl::BoundingBox;
using demdl::DemRaster;
using demdl::Downloader;
using demdl::DownloaderOptions;
using demdl::FetchResult;
using demdl::FetcherFactory;
using demdl::PixelState;
using demdl::StopReason;
using demdl::StopToken;

namespace {

const BoundingBox kBox{-34.0, -34.3, 18.5, 18.1};

float expectedValue(int x, int y) { return static_cast<float>(x * 10 + y); }

struct Fixture {
  TempDir dir;
  std::filesystem::path output = dir.path / "out.tif";
  RecordingSink sink;
  StopToken stop;
  std::atomic<int> calls{0};
  std::atomic<int> sawStop{0};
  Gate gate;

  FetcherFactory factory(std::function<FetchResult(int, int)> behaviour, bool blocking = false) {
    return [this, behaviour, blocking](int) -> std::unique_ptr<demdl::PixelFetcher> {
      auto f = std::make_unique<FakeFetcher>();
      f->behaviour = behaviour;
      f->calls = &calls;
      f->sawStop = &sawStop;
      f->gate = blocking ? &gate : nullptr;
      return f;
    };
  }
};

}  // namespace

TEST_CASE("only NaN pixels are fetched and finished rows are skipped", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(4, 3, kBox, {});
  for (int x = 0; x < 4; ++x) raster.set(x, 1, 99.0f);  // row 1 complete
  raster.set(0, 2, 7.0f);                               // row 2 half done
  raster.set(1, 2, DemRaster::kNoData);
  Downloader d(raster, f.output, f.factory([](int x, int y) { return valueResult(expectedValue(x, y)); }),
               f.sink, f.stop, DownloaderOptions{2, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.pixelsFetched == 6);
  CHECK(summary.noData == 0);
  CHECK(summary.failed == 0);
  CHECK(summary.rowsSaved == 2);
  CHECK(summary.remainingNaN == 0);
  CHECK(summary.stopReason == StopReason::None);
  CHECK(summary.stopMessage == "finished");
  CHECK(f.calls.load() == 6);
  REQUIRE(f.sink.plan.has_value());
  CHECK(f.sink.plan->pixelsTotal == 12);
  CHECK(f.sink.plan->pixelsTodo == 6);
  CHECK(f.sink.plan->rowsTodo == 2);
  CHECK(f.sink.plan->rowsTotal == 3);
  REQUIRE(f.sink.rows.size() == 2);
  CHECK(f.sink.rows[0].row == 0);
  CHECK(f.sink.rows[0].ok == 4);
  CHECK(f.sink.rows[0].pixelsDoneTotal == 10);
  CHECK(f.sink.rows[1].row == 2);
  CHECK(f.sink.rows[1].pixelsDoneTotal == 12);
  CHECK(f.sink.count(PixelState::Queued) == 6);
  CHECK(f.sink.count(PixelState::InFlight) == 6);
  CHECK(f.sink.count(PixelState::Value) == 6);
  CHECK(raster.at(3, 0) == expectedValue(3, 0));
  CHECK(raster.at(0, 2) == 7.0f);
  CHECK(raster.at(2, 2) == expectedValue(2, 2));

  const auto onDisk = DemRaster::load(f.output);
  CHECK(onDisk.countNaN() == 0);
  CHECK(onDisk.at(3, 2) == expectedValue(3, 2));
}

TEST_CASE("NoData and failures are recorded and counted", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(4, 1, kBox, {});
  Downloader d(raster, f.output, f.factory([](int x, int y) {
                 if (x == 0) return noDataResult();
                 if (x == 1) return failedResult("http 500");
                 return valueResult(expectedValue(x, y));
               }),
               f.sink, f.stop, DownloaderOptions{1, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.pixelsFetched == 2);
  CHECK(summary.noData == 1);
  CHECK(summary.failed == 1);
  CHECK(summary.remainingNaN == 1);
  CHECK(summary.stopReason == StopReason::None);
  CHECK(raster.at(0, 0) == DemRaster::kNoData);
  CHECK(std::isnan(raster.at(1, 0)));
  CHECK(f.sink.count(PixelState::Failed) == 1);
  CHECK(f.sink.count(PixelState::NoData) == 1);
  CHECK(f.sink.count(PixelState::Value) == 2);
  REQUIRE(f.sink.rows.size() == 1);
  CHECK(f.sink.rows[0].ok == 2);
  CHECK(f.sink.rows[0].noData == 1);
  CHECK(f.sink.rows[0].failed == 1);
  CHECK(f.sink.rows[0].pixelsDoneTotal == 3);
}

TEST_CASE("a stop request saves the partial row and goes no further", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(4, 2, kBox, {});
  Downloader d(raster, f.output,
               f.factory([](int x, int y) { return valueResult(expectedValue(x, y)); }, /*blocking=*/true),
               f.sink, f.stop, DownloaderOptions{2, 100});

  std::thread runner([&d] { (void)d.run(); });
  // Opens the gate and joins the runner however the test exits, so a failed REQUIRE cannot destroy
  // a joinable std::thread, which would call std::terminate.
  struct ReleaseAndJoin {
    Gate& gate;
    std::thread& runner;
    ~ReleaseAndJoin() {
      gate.release();
      if (runner.joinable()) runner.join();
    }
  } guard{f.gate, runner};
  REQUIRE(f.gate.awaitArrivals(2, 5s));
  CHECK(d.inFlight() == 2);

  f.stop.request(StopReason::Signal);
  f.gate.release();
  runner.join();

  CHECK(f.sawStop.load() == 2);
  REQUIRE(f.sink.summary.has_value());
  CHECK(f.sink.summary->stopReason == StopReason::Signal);
  CHECK(f.sink.summary->stopMessage == "interrupted");
  CHECK(f.sink.summary->pixelsFetched == 2);
  CHECK(f.sink.summary->rowsSaved == 1);
  CHECK(f.sink.summary->remainingNaN == 6);
  CHECK(f.calls.load() == 2);
  CHECK(d.inFlight() == 0);
  REQUIRE(f.sink.rows.size() == 1);
  CHECK(f.sink.rows[0].row == 0);
  CHECK(f.sink.rows[0].ok == 2);

  const auto onDisk = DemRaster::load(f.output);
  CHECK(onDisk.countNaN() == 6);
  CHECK(onDisk.todoInRow(0).size() == 2);
  CHECK(onDisk.todoInRow(1).size() == 4);
}

TEST_CASE("too many consecutive failures stop the run", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(10, 1, kBox, {});
  Downloader d(raster, f.output, f.factory([](int, int) { return failedResult("connection refused"); }),
               f.sink, f.stop, DownloaderOptions{1, 3});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::ConsecutiveFailures);
  CHECK(summary.stopMessage == "server unreachable: 3 consecutive failures");
  CHECK(summary.failed == 3);
  CHECK(f.calls.load() == 3);
  CHECK(summary.remainingNaN == 10);
  CHECK(summary.rowsSaved == 1);
  CHECK(std::filesystem::exists(f.output));
}

TEST_CASE("a complete raster makes no requests and writes nothing", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(3, 2, kBox, {});
  for (int y = 0; y < 2; ++y) {
    for (int x = 0; x < 3; ++x) raster.set(x, y, 1.0f);
  }
  Downloader d(raster, f.output, f.factory([](int, int) { return valueResult(0.0f); }), f.sink, f.stop,
               DownloaderOptions{2, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(f.calls.load() == 0);
  CHECK(summary.rowsSaved == 0);
  CHECK(summary.remainingNaN == 0);
  REQUIRE(f.sink.plan.has_value());
  CHECK(f.sink.plan->pixelsTodo == 0);
  CHECK(f.sink.plan->rowsTodo == 0);
  REQUIRE(f.sink.summary.has_value());
  CHECK_FALSE(std::filesystem::exists(f.output));
}

TEST_CASE("an exception inside a job is recorded as a failure, not propagated", "[downloader]") {
  Fixture f;
  auto raster = DemRaster::create(3, 1, kBox, {});
  Downloader d(raster, f.output, f.factory([](int x, int y) {
                 if (x == 1) throw std::runtime_error("boom");
                 return valueResult(expectedValue(x, y));
               }),
               f.sink, f.stop, DownloaderOptions{1, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.failed == 1);
  CHECK(summary.pixelsFetched == 2);
  CHECK(summary.remainingNaN == 1);
  CHECK(d.inFlight() == 0);
  const auto failure = std::find_if(f.sink.pixels.begin(), f.sink.pixels.end(),
                                    [](const demdl::PixelEvent& e) { return e.state == PixelState::Failed; });
  REQUIRE(failure != f.sink.pixels.end());
  CHECK(failure->x == 1);
  CHECK_THAT(failure->error, ContainsSubstring("boom"));
}

TEST_CASE("a write failure after a stop request is still reported", "[downloader]") {
  Fixture f;
  REQUIRE(std::filesystem::create_directory(f.output));  // the checkpoint cannot replace a directory
  auto raster = DemRaster::create(2, 1, kBox, {});
  // A stop requested before run() would end the run before any save, so the first fetch requests it.
  // The second pixel is then skipped, and the row's save fails after the stop.
  Downloader d(raster, f.output, f.factory([&stop = f.stop](int x, int y) {
                 stop.request(StopReason::Signal);
                 return valueResult(expectedValue(x, y));
               }),
               f.sink, f.stop, DownloaderOptions{1, 100});

  const demdl::RunSummary summary = d.run();

  CHECK(summary.stopReason == StopReason::Signal);
  CHECK_THAT(summary.stopMessage, ContainsSubstring("interrupted"));
  CHECK_THAT(summary.stopMessage, ContainsSubstring("checkpoint write failed"));
  CHECK(summary.rowsSaved == 0);
}
