#include "demdl/Downloader.h"

#include <ctpl_stl.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace demdl {

namespace {

// Counts one fetch in Downloader::inFlight() for as long as it lives.
class InFlightGuard {
 public:
  explicit InFlightGuard(std::atomic<int>& count) noexcept : count_(count) { ++count_; }
  ~InFlightGuard() { --count_; }
  InFlightGuard(const InFlightGuard&) = delete;
  InFlightGuard& operator=(const InFlightGuard&) = delete;

 private:
  std::atomic<int>& count_;
};

}  // namespace

Downloader::Downloader(DemRaster& raster, std::filesystem::path output, FetcherFactory factory,
                       ProgressSink& sink, StopToken& stop, DownloaderOptions options)
    : raster_(raster),
      output_(std::move(output)),
      factory_(std::move(factory)),
      sink_(sink),
      stop_(stop),
      options_(options) {
  if (options_.threads < 1) throw std::invalid_argument("threads must be at least 1");
  if (options_.maxConsecutiveFailures < 1) throw std::invalid_argument("maxConsecutiveFailures must be at least 1");
}

RunSummary Downloader::run() {
  using Clock = std::chrono::steady_clock;
  const auto runStart = Clock::now();

  // Made before the first callback, so a factory that fails leaves the sink untouched.
  std::vector<std::unique_ptr<PixelFetcher>> fetchers;
  fetchers.reserve(static_cast<std::size_t>(options_.threads));
  for (int i = 0; i < options_.threads; ++i) {
    fetchers.push_back(factory_(i));
    if (!fetchers.back()) throw std::invalid_argument("fetcher factory returned null");
  }

  const std::vector<int> rows = raster_.rowsWithWork();
  RunPlan plan;
  plan.pixelsTotal = raster_.pixelCount();
  plan.pixelsTodo = raster_.countNaN();
  plan.rowsTodo = static_cast<int>(rows.size());
  plan.rowsTotal = raster_.height();
  sink_.onRunStarted(plan);

  RunTally runTally;
  RowTally rowTally;
  std::uint64_t doneTotal = plan.pixelsTotal - plan.pixelsTodo;  // non-NaN pixels in the last checkpoint
  int rowsSaved = 0;
  bool checkpointWriteFailed = false;
  std::string writeError;
  // Declared after everything its jobs use, so its destructor, which waits for them, runs first.
  ctpl::thread_pool pool(options_.threads);

  for (const int row : rows) {
    if (stop_.requested()) break;
    const auto rowStart = Clock::now();
    const std::vector<int> todo = raster_.todoInRow(row);
    for (const int x : todo) sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::Queued});

    rowTally.reset();
    std::vector<std::future<void>> futures;
    futures.reserve(todo.size());
    try {
      for (const int x : todo) {
        if (stop_.requested()) break;  // stop queueing immediately
        futures.push_back(pool.push([this, &fetchers, &rowTally, &runTally, x, row](int workerId) noexcept {
          fetchPixel(*fetchers[static_cast<std::size_t>(workerId)], x, row, rowTally, runTally);
        }));
      }
    } catch (...) {
      pool.stop(false);  // discard the queued jobs and wait for the running ones before unwinding
      throw;
    }
    for (auto& f : futures) f.get();

    try {
      raster_.save(output_);
    } catch (const std::exception& e) {
      checkpointWriteFailed = true;
      writeError = e.what();
      stop_.request(StopReason::WriteError);
      break;
    }
    ++rowsSaved;
    doneTotal += static_cast<std::uint64_t>(rowTally.ok.load() + rowTally.noData.load());
    RowReport report;
    report.row = row;
    report.rowsTotal = raster_.height();
    report.ok = rowTally.ok.load();
    report.noData = rowTally.noData.load();
    report.failed = rowTally.failed.load();
    report.elapsed = Clock::now() - rowStart;
    report.pixelsDoneTotal = doneTotal;
    report.pixelsTotal = plan.pixelsTotal;
    sink_.onRowSaved(report);
  }

  RunSummary summary;
  summary.pixelsFetched = runTally.fetched.load();
  summary.noData = runTally.noData.load();
  summary.failed = runTally.failed.load();
  summary.remainingNaN = plan.pixelsTotal - doneTotal;  // the last checkpoint's count, not memory's
  summary.rowsSaved = rowsSaved;
  summary.elapsed = Clock::now() - runStart;
  summary.stopReason = stop_.reason();
  summary.checkpointWriteFailed = checkpointWriteFailed;
  if (summary.stopReason == StopReason::ConsecutiveFailures) {
    summary.stopMessage =
        "server unreachable: " + std::to_string(options_.maxConsecutiveFailures) + " consecutive failures";
    const std::lock_guard<std::mutex> lock(runTally.limitMutex);
    if (runTally.limitError) summary.stopMessage += "; last error: " + *runTally.limitError;
  } else {
    summary.stopMessage = describe(summary.stopReason);
  }
  if (checkpointWriteFailed) {
    const std::string failure = std::string(describe(StopReason::WriteError)) + ": " + writeError;
    if (summary.stopReason == StopReason::WriteError) {
      summary.stopMessage = failure;
    } else {
      summary.stopMessage += "; " + failure;  // an earlier reason was kept, but the failure still shows
    }
  }
  sink_.onRunFinished(summary);
  return summary;
}

void Downloader::fetchPixel(PixelFetcher& fetcher, int x, int row, RowTally& rowTally, RunTally& runTally) noexcept {
  if (stop_.requested()) return;  // queued before the stop, never started
  FetchResult result;
  {
    const InFlightGuard guard(inFlight_);
    // Fetchers and sinks must not throw. Should the InFlight callback or the fetch do so anyway, the
    // pixel is recorded as Failed; an exception anywhere else ends the process, as this is noexcept.
    try {
      sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::InFlight});
      result = fetcher.fetch(x, row, stop_.flag());
    } catch (const std::exception& e) {
      result = FetchResult{.kind = FetchResult::Kind::Failed, .error = std::string("exception: ") + e.what()};
    } catch (...) {
      result = FetchResult{.kind = FetchResult::Kind::Failed, .error = "unknown exception"};
    }
  }
  switch (result.kind) {
    case FetchResult::Kind::Value:
      raster_.set(x, row, result.value);
      runTally.consecutiveFailures.store(0);
      ++rowTally.ok;
      ++runTally.fetched;
      sink_.onPixelState(PixelEvent{
          .x = x, .y = row, .state = PixelState::Value, .value = result.value, .attempts = result.attempts});
      break;
    case FetchResult::Kind::NoData:
      raster_.set(x, row, DemRaster::kNoData);
      runTally.consecutiveFailures.store(0);
      ++rowTally.noData;
      ++runTally.noData;
      sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::NoData, .attempts = result.attempts});
      break;
    case FetchResult::Kind::Failed: {
      ++rowTally.failed;
      ++runTally.failed;
      const int streak = ++runTally.consecutiveFailures;
      sink_.onPixelState(PixelEvent{
          .x = x, .y = row, .state = PixelState::Failed, .attempts = result.attempts, .error = result.error});
      if (streak >= options_.maxConsecutiveFailures) {
        {
          const std::lock_guard<std::mutex> lock(runTally.limitMutex);
          if (!runTally.limitError) runTally.limitError = std::move(result.error);  // the first to reach it
        }
        stop_.request(StopReason::ConsecutiveFailures);
      }
      break;
    }
  }
}

}  // namespace demdl
