#pragma once

#include <chrono>

namespace demdl {

// Exponential backoff: min(base * 2^attempt, max), scaled by 1 + jitter * (2 * unitRandom - 1).
struct RetryPolicy {
  int maxRetries = 8;  // retries after the first attempt
  std::chrono::milliseconds baseDelay{500};
  std::chrono::milliseconds maxDelay{30000};
  double jitter = 0.2;

  // Delay before retry number `attempt` (0-based). unitRandom is in [0, 1]; passing it in keeps the schedule testable.
  [[nodiscard]] std::chrono::milliseconds delayFor(int attempt, double unitRandom) const;
};

}  // namespace demdl
