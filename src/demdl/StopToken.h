#pragma once

#include <atomic>
#include <cassert>

#include "demdl/ProgressSink.h"

namespace demdl {

// Shared "please stop" flag plus the first reason given. request() raises the flag and keeps the
// first reason: later reasons are ignored. Both members are lock-free atomics (see the
// static_asserts) and request() touches nothing else, so a signal handler may call it.
class StopToken {
 public:
  // Requires reason != StopReason::None (asserted in debug builds); None would raise the flag
  // without saying why.
  void request(StopReason reason) noexcept {
    assert(reason != StopReason::None);
    StopReason expected = StopReason::None;
    // Reason first, then flag; both seq_cst, so a reader that observes the flag observes a reason.
    reason_.compare_exchange_strong(expected, reason);
    stop_.store(true);
  }
  [[nodiscard]] bool requested() const noexcept { return stop_.load(); }
  [[nodiscard]] StopReason reason() const noexcept { return reason_.load(); }
  // The underlying flag, read-only, for handing to PixelFetcher::fetch.
  [[nodiscard]] const std::atomic<bool>& flag() const noexcept { return stop_; }

 private:
  std::atomic<bool> stop_{false};
  std::atomic<StopReason> reason_{StopReason::None};
  static_assert(std::atomic<bool>::is_always_lock_free);
  static_assert(std::atomic<StopReason>::is_always_lock_free);
};

}  // namespace demdl
