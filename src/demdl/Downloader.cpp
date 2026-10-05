#include "demdl/Downloader.h"

#include <ctpl_stl.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace demdl {

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

  const std::vector<int> rows = raster_.rowsWithWork();
  RunPlan plan;
  plan.pixelsTotal = raster_.pixelCount();
  plan.pixelsTodo = raster_.countNaN();
  plan.rowsTodo = static_cast<int>(rows.size());
  plan.rowsTotal = raster_.height();
  sink_.onRunStarted(plan);

  ctpl::thread_pool pool(options_.threads);
  std::vector<std::unique_ptr<PixelFetcher>> fetchers;
  fetchers.reserve(static_cast<std::size_t>(options_.threads));
  for (int i = 0; i < options_.threads; ++i) fetchers.push_back(factory_(i));

  std::atomic<int> consecutiveFailures{0};
  std::atomic<std::uint64_t> fetched{0};
  std::atomic<std::uint64_t> noData{0};
  std::atomic<std::uint64_t> failed{0};
  std::uint64_t doneTotal = plan.pixelsTotal - plan.pixelsTodo;
  int rowsSaved = 0;
  std::string writeError;

  for (const int row : rows) {
    if (stop_.requested()) break;
    const auto rowStart = Clock::now();
    const std::vector<int> todo = raster_.todoInRow(row);
    for (const int x : todo) sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::Queued});

    std::atomic<int> rowOk{0};
    std::atomic<int> rowNoData{0};
    std::atomic<int> rowFailed{0};
    std::vector<std::future<void>> futures;
    futures.reserve(todo.size());
    for (const int x : todo) {
      if (stop_.requested()) break;  // stop queueing immediately
      futures.push_back(pool.push([&, x, row](int workerId) noexcept {
        if (stop_.requested()) return;  // queued before the stop, never started
        const auto recordFailure = [&](const std::string& error, int attempts) {
          ++rowFailed;
          ++failed;
          const int streak = ++consecutiveFailures;
          sink_.onPixelState(
              PixelEvent{.x = x, .y = row, .state = PixelState::Failed, .attempts = attempts, .error = error});
          if (streak >= options_.maxConsecutiveFailures) stop_.request(StopReason::ConsecutiveFailures);
        };
        // Fetchers and sinks must not throw; should one do so anyway, the pixel is recorded as Failed.
        // The job is noexcept, so an exception escaping these handlers terminates the process instead
        // of making f.get() below rethrow while queued jobs still use this row's locals.
        bool counted = false;  // inFlight_ includes this job
        try {
          ++inFlight_;
          counted = true;
          sink_.onPixelState(PixelEvent{.x = x, .y = row, .state = PixelState::InFlight});
          const FetchResult result = fetchers[static_cast<std::size_t>(workerId)]->fetch(x, row, stop_.flag());
          --inFlight_;
          counted = false;
          switch (result.kind) {
            case FetchResult::Kind::Value:
              raster_.set(x, row, result.value);
              consecutiveFailures.store(0);
              ++rowOk;
              ++fetched;
              sink_.onPixelState(PixelEvent{
                  .x = x, .y = row, .state = PixelState::Value, .value = result.value, .attempts = result.attempts});
              break;
            case FetchResult::Kind::NoData:
              raster_.set(x, row, DemRaster::kNoData);
              consecutiveFailures.store(0);
              ++rowNoData;
              ++noData;
              sink_.onPixelState(
                  PixelEvent{.x = x, .y = row, .state = PixelState::NoData, .attempts = result.attempts});
              break;
            case FetchResult::Kind::Failed:
              recordFailure(result.error, result.attempts);
              break;
          }
        } catch (const std::exception& e) {
          if (counted) --inFlight_;
          recordFailure(std::string("exception: ") + e.what(), 0);
        } catch (...) {
          if (counted) --inFlight_;
          recordFailure("unknown exception", 0);
        }
      }));
    }
    for (auto& f : futures) f.get();

    try {
      raster_.save(output_);
    } catch (const std::exception& e) {
      writeError = e.what();
      stop_.request(StopReason::WriteError);
      break;
    }
    ++rowsSaved;
    doneTotal += static_cast<std::uint64_t>(rowOk.load() + rowNoData.load());
    RowReport report;
    report.row = row;
    report.rowsTotal = raster_.height();
    report.ok = rowOk.load();
    report.noData = rowNoData.load();
    report.failed = rowFailed.load();
    report.elapsed = Clock::now() - rowStart;
    report.pixelsDoneTotal = doneTotal;
    report.pixelsTotal = plan.pixelsTotal;
    sink_.onRowSaved(report);
  }

  RunSummary summary;
  summary.pixelsFetched = fetched.load();
  summary.noData = noData.load();
  summary.failed = failed.load();
  summary.remainingNaN = raster_.countNaN();
  summary.rowsSaved = rowsSaved;
  summary.elapsed = Clock::now() - runStart;
  summary.stopReason = stop_.reason();
  switch (summary.stopReason) {
    case StopReason::ConsecutiveFailures:
      summary.stopMessage =
          "server unreachable: " + std::to_string(options_.maxConsecutiveFailures) + " consecutive failures";
      break;
    case StopReason::WriteError:
      summary.stopMessage = writeError;
      break;
    default:
      summary.stopMessage = describe(summary.stopReason);
      break;
  }
  // A stop requested before the failed save keeps its reason, so the write error is reported with it.
  if (!writeError.empty() && summary.stopReason != StopReason::WriteError) {
    summary.stopMessage += "; checkpoint write failed: " + writeError;
  }
  sink_.onRunFinished(summary);
  return summary;
}

}  // namespace demdl
