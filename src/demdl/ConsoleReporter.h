#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <iosfwd>
#include <mutex>
#include <string>

#include "demdl/ProgressSink.h"

namespace demdl {

// "41.2s" under a minute, "12m05s" under an hour, otherwise "13h31m" (minutes truncated). The unit
// is chosen after rounding, so 59.95 gives "1m00s" and 3599.6 gives "1h00m". NaN and negative
// inputs give "0.0s", and inputs above a billion hours give "1000000000h00m".
[[nodiscard]] std::string formatDuration(double seconds);

// Writes the finish lines of a run: the totals, "stopped: <message>" if it stopped, and, while the
// file still has NaN pixels, how to finish the job (retry this run's failures, or resume the rest).
// Does not flush. ConsoleReporter::onRunFinished prints these lines; main calls this itself after a
// --preview run, whose sink prints nothing.
void writeSummary(std::ostream& out, const RunSummary& summary);

// Headless progress: one line per saved row, optional per-pixel lines, a final summary.
// Each callback writes its lines whole under one mutex, so concurrent pixel events never interleave.
// The starting, row and finish lines are flushed as they are written, so a redirected stdout logs
// each row when it is saved; the frequent verbose pixel lines are left to the stream's buffering.
// Callbacks do not throw in practice: lines are formatted into fixed buffers and short strings.
// They are not noexcept only because std::mutex::lock and std::string construction may formally
// throw. The stream must keep exceptions disabled; once it fails (closed stdout, EPIPE), later
// lines are silently dropped.
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
  // The last ten saved rows, for the rate: a ring that never allocates. Unused slots are zero.
  std::array<RowSample, 10> window_{};
  std::size_t next_ = 0;  // the slot the next saved row overwrites
  // For the ETA: the pixels the run set out to fetch and how many it has attempted. A run does not
  // retry its own failures, so a pixel that failed no longer counts as work left.
  std::uint64_t pixelsTodo_ = 0;
  std::uint64_t attempted_ = 0;
};

}  // namespace demdl
