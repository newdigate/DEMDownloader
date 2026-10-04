#pragma once

#include <atomic>

#include "demdl/ProgressSink.h"

namespace demdl {

// Shared "please stop" flag plus the first reason given. request() always raises the flag but
// keeps the first reason: later reasons are ignored. Both members are lock-free atomics and
// request() touches nothing else, so a signal handler may call it.
class StopToken {
 public:
  std::atomic<bool> stop{false};

  void request(StopReason reason) noexcept {
    StopReason expected = StopReason::None;
    reason_.compare_exchange_strong(expected, reason);
    stop.store(true);
  }
  [[nodiscard]] StopReason reason() const noexcept { return reason_.load(); }

 private:
  std::atomic<StopReason> reason_{StopReason::None};
};

}  // namespace demdl
