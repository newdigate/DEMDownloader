#pragma once

#include "demdl/ProgressSink.h"

namespace demdl::cli {

// The process exit codes, spec section 10.
inline constexpr int kExitComplete = 0;        // the output has no NaN pixel left
inline constexpr int kExitUsage = 1;           // usage or configuration error before any request, or an
                                               // unexpected internal error during the run
inline constexpr int kExitStopped = 2;         // stopped early; the checkpoint is intact and a rerun resumes
inline constexpr int kExitIncomplete = 3;      // ran to the end with failed pixels; a rerun retries them
inline constexpr int kExitSecondSignal = 130;  // a second SIGINT or SIGTERM: immediate exit

// The exit code for a finished run. A failed checkpoint write decides first: the file lacks the row whose
// save failed, so whatever the counts say, the run stopped early.
[[nodiscard]] inline int exitCodeFor(const demdl::RunSummary& s) noexcept {
  if (s.checkpointWriteFailed) return kExitStopped;
  if (s.remainingNaN == 0) return kExitComplete;
  if (s.stopReason != demdl::StopReason::None) return kExitStopped;
  return kExitIncomplete;
}

}  // namespace demdl::cli
