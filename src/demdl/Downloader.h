#pragma once

#include <atomic>
#include <filesystem>
#include <functional>
#include <memory>

#include "demdl/DemRaster.h"
#include "demdl/PixelFetcher.h"
#include "demdl/ProgressSink.h"
#include "demdl/StopToken.h"

namespace demdl {

// Creates the fetcher for one worker thread. Called once per worker before the first row.
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
// row and run callbacks come only from the thread calling run(). A pool job catches every exception
// and records its pixel as Failed, while an exception from the factory propagates out of run().
// Jobs are noexcept, so an exception escaping even those handlers (recording a failure under memory
// pressure, say) calls std::terminate instead of unwinding past the pool.
class Downloader {
 public:
  // Throws std::invalid_argument when threads or maxConsecutiveFailures is below 1.
  Downloader(DemRaster& raster, std::filesystem::path output, FetcherFactory factory, ProgressSink& sink,
             StopToken& stop, DownloaderOptions options);

  [[nodiscard]] RunSummary run();

  // Pixels currently inside a fetch call. For the "stopping" message.
  [[nodiscard]] int inFlight() const { return inFlight_.load(); }

 private:
  DemRaster& raster_;
  std::filesystem::path output_;
  FetcherFactory factory_;
  ProgressSink& sink_;
  StopToken& stop_;
  DownloaderOptions options_;
  std::atomic<int> inFlight_{0};
};

}  // namespace demdl
