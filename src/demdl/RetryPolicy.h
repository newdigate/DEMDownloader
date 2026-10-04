#pragma once

#include <chrono>

namespace demdl {

// Exponential backoff: min(base * 2^attempt, max), scaled by 1 + jitter * (2 * unitRandom - 1).
struct RetryPolicy {
  int maxRetries = 8;  // retries after the first attempt
  std::chrono::milliseconds baseDelay{500};
  // The cap is applied before jitter, so the longest sleep is maxDelay * (1 + jitter).
  std::chrono::milliseconds maxDelay{30000};
  double jitter = 0.2;

  // Delay before retry number `attempt` (0-based). Requires attempt >= 0, baseDelay > 0 and
  // 0 <= jitter <= 1; these are not checked. The caller supplies unitRandom in [0, 1], which keeps
  // the schedule testable. The interval is closed on purpose: std::uniform_real_distribution can
  // return exactly 1.0 on this standard library.
  [[nodiscard]] std::chrono::milliseconds delayFor(int attempt, double unitRandom) const;
};

}  // namespace demdl
