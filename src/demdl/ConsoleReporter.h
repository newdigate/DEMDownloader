#pragma once

#include <array>
#include <cstddef>
#include <iosfwd>
#include <mutex>
#include <string>

#include "demdl/ProgressSink.h"

namespace demdl {

// "41.2s" under a minute, "12m05s" under an hour, otherwise "13h31m" (minutes truncated). The unit
// is chosen after rounding, so 59.95 gives "1m00s" and 3599.6 gives "1h00m". NaN and negative
// inputs give "0.0s", and inputs above a billion hours give "1000000000h00m".
[[nodiscard]] std::string formatDuration(double seconds);

// Headless progress: one line per saved row, optional per-pixel lines, a final summary.
// Each callback writes its lines whole under one mutex, so concurrent pixel events never interleave.
// Callbacks never throw except on allocation failure while formatting (std::bad_alloc), which is
// why they are not noexcept. The stream must keep exceptions disabled, as std::cout does by default.
class ConsoleReporter : public ProgressSink {
 public:
  // `out` must outlive the reporter. `verbose` adds a line per resolved pixel.
  ConsoleReporter(std::ostream& out, bool verbose);

  void onRunStarted(const RunPlan& plan) override;
  void onPixelState(const PixelEvent& event) override;
  void onRowSaved(const RowReport& report) override;
  void onRunFinished(const RunSummary& summary) override;

 private:
  struct RowSample {
    double pixels = 0.0;   // ok + noData + failed
    double seconds = 0.0;  // the row's elapsed time
  };

  std::ostream& out_;
  const bool verbose_;
  std::mutex mutex_;
  // The last ten saved rows, for rate and ETA: a ring that never allocates. Unused slots are zero.
  std::array<RowSample, 10> window_{};
  std::size_t next_ = 0;  // the slot the next saved row overwrites
};

}  // namespace demdl
