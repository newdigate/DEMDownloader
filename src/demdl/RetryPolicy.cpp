#include "demdl/RetryPolicy.h"

#include <algorithm>
#include <cmath>

namespace demdl {

std::chrono::milliseconds RetryPolicy::delayFor(int attempt, double unitRandom) const {
  const double capped = std::min(static_cast<double>(baseDelay.count()) * std::ldexp(1.0, attempt),
                                 static_cast<double>(maxDelay.count()));
  const double scaled = capped * (1.0 + jitter * (2.0 * unitRandom - 1.0));
  return std::chrono::milliseconds(std::llround(scaled));
}

}  // namespace demdl
