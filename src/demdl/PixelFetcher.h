#pragma once

#include <atomic>
#include <string>

namespace demdl {

struct FetchResult {
  enum class Kind { Value, NoData, Failed };
  Kind kind = Kind::Failed;
  float value = 0.0f;   // valid when kind == Value
  int attempts = 0;     // attempts made, including the first
  std::string error{};  // last error when kind == Failed
};

// Fetches one pixel. Implementations must not throw: fetch runs inside a pool job, so every
// failure is reported through the result. They may block for retries, but once `stop` is set they
// must neither start another attempt nor keep sleeping (a request already on the wire may run to
// its timeout). `value` is finite when `kind == Value`. One instance serves one worker thread;
// instances need not be thread-safe.
class PixelFetcher {
 public:
  virtual ~PixelFetcher() = default;
  [[nodiscard]] virtual FetchResult fetch(int x, int y, const std::atomic<bool>& stop) = 0;
};

}  // namespace demdl
