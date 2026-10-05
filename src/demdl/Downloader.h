#pragma once

#include <atomic>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>

#include "demdl/DemRaster.h"
#include "demdl/PixelFetcher.h"
#include "demdl/ProgressSink.h"
#include "demdl/StopToken.h"

namespace demdl {

// Creates the fetcher for one worker thread. run() calls it once per worker, before onRunStarted.
using FetcherFactory = std::function<std::unique_ptr<PixelFetcher>(int workerId)>;

struct DownloaderOptions {
  int threads = 8;
  int maxConsecutiveFailures = 100;
};

// Walks the rows that still contain NaN pixels, fetches those pixels through a thread pool,
// saves the raster after each row and reports progress.
//
// Call run() exactly once. It blocks until the run ends, so when a preview window is active it must
// not run on the main thread, which belongs to the GLFW event loop; main calls it under std::async.
// Pixel callbacks reach the sink from the pool threads and from the thread calling run() (Queued);
// row and run callbacks come only from the thread calling run().
//
// Errors: run() makes every fetcher before its first callback, so an exception from the factory, or
// std::invalid_argument for a null fetcher, leaves run() before the sink hears anything. Inside a pool
// job an exception from the InFlight callback or the fetch is recorded as a Failed pixel. Jobs are
// noexcept, so any other exception in one (a sink callback reporting the result, or building the
// error text under memory pressure) calls std::terminate instead of unwinding past the pool.
class Downloader {
 public:
  // Throws std::invalid_argument when threads or maxConsecutiveFailures is below 1.
  Downloader(DemRaster& raster, std::filesystem::path output, FetcherFactory factory, ProgressSink& sink,
             StopToken& stop, DownloaderOptions options);

  [[nodiscard]] RunSummary run();

  // Pixels currently inside a fetch call. For the "stopping" message.
  [[nodiscard]] int inFlight() const { return inFlight_.load(); }

 private:
  // Counters the pool jobs update concurrently: RowTally is reset before each row, RunTally spans
  // the run.
  struct RowTally {
    std::atomic<int> ok{0};
    std::atomic<int> noData{0};
    std::atomic<int> failed{0};
    void reset() noexcept {
      ok = 0;
      noData = 0;
      failed = 0;
    }
  };
  struct RunTally {
    std::atomic<std::uint64_t> fetched{0};
    std::atomic<std::uint64_t> noData{0};
    std::atomic<std::uint64_t> failed{0};
    std::atomic<int> consecutiveFailures{0};  // reset by every Value or NoData
  };

  // One pool job: unless a stop came first, fetches pixel (x, row), stores the result, updates the
  // tallies and reports the pixel's final state.
  void fetchPixel(PixelFetcher& fetcher, int x, int row, RowTally& rowTally, RunTally& runTally) noexcept;

  DemRaster& raster_;
  std::filesystem::path output_;
  FetcherFactory factory_;
  ProgressSink& sink_;
  StopToken& stop_;
  DownloaderOptions options_;
  std::atomic<int> inFlight_{0};
};

}  // namespace demdl
