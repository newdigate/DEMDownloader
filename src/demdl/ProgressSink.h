#pragma once

#include <chrono>
#include <cstdint>
#include <string>

namespace demdl {

enum class PixelState { Queued, InFlight, Value, NoData, Failed };

enum class StopReason { None, Signal, WindowClosed, ConsecutiveFailures, WriteError };

[[nodiscard]] constexpr const char* describe(StopReason reason) noexcept {
  switch (reason) {
    case StopReason::None: return "finished";
    case StopReason::Signal: return "interrupted";
    case StopReason::WindowClosed: return "preview window closed";
    case StopReason::ConsecutiveFailures: return "too many consecutive failures";
    case StopReason::WriteError: return "checkpoint write failed";
  }
  return "unknown";
}

struct RunPlan {
  std::uint64_t pixelsTotal = 0;
  std::uint64_t pixelsTodo = 0;
  int rowsTodo = 0;
  int rowsTotal = 0;
};

struct PixelEvent {
  int x = 0;
  int y = 0;
  PixelState state = PixelState::Queued;
  float value = 0.0f;   // valid when state == Value
  int attempts = 0;
  std::string error{};  // set when state == Failed
};

struct RowReport {
  int row = 0;  // 0-based
  int rowsTotal = 0;
  int ok = 0;
  int noData = 0;
  int failed = 0;
  std::chrono::duration<double> elapsed{0.0};
  std::uint64_t pixelsDoneTotal = 0;  // non-NaN pixels in the whole raster after this save
  std::uint64_t pixelsTotal = 0;
};

struct RunSummary {
  std::uint64_t pixelsFetched = 0;
  std::uint64_t noData = 0;
  std::uint64_t failed = 0;
  std::uint64_t remainingNaN = 0;  // NaN pixels in the last successfully written file
  int rowsSaved = 0;
  std::chrono::duration<double> elapsed{0.0};
  StopReason stopReason = StopReason::None;
  std::string stopMessage{};
  bool checkpointWriteFailed = false;  // a save threw, even if stopReason names an earlier reason
};

// Callbacks must not throw: pixel events are delivered inside pool jobs. Pixel events may arrive
// concurrently from worker threads and from the downloader thread (Queued comes from the
// downloader); row and run events come from the downloader thread. Implementations guard their
// own state.
class ProgressSink {
 public:
  virtual ~ProgressSink() = default;
  virtual void onRunStarted(const RunPlan& plan) = 0;
  virtual void onPixelState(const PixelEvent& event) = 0;
  virtual void onRowSaved(const RowReport& report) = 0;
  virtual void onRunFinished(const RunSummary& summary) = 0;
};

}  // namespace demdl
