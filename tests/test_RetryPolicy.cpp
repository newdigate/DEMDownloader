#include <catch2/catch_test_macros.hpp>

#include <chrono>
#include <limits>

#include "demdl/RetryPolicy.h"

using namespace std::chrono_literals;
using demdl::RetryPolicy;

TEST_CASE("the schedule doubles from the base and caps at the maximum", "[retrypolicy]") {
  const RetryPolicy p;  // defaults: 500 ms base, 30 s cap, 20 % jitter
  const std::chrono::milliseconds expected[] = {500ms, 1000ms, 2000ms, 4000ms, 8000ms, 16000ms, 30000ms, 30000ms};
  for (int attempt = 0; attempt < 8; ++attempt) {
    CAPTURE(attempt);
    CHECK(p.delayFor(attempt, 0.5) == expected[attempt]);
  }
}

TEST_CASE("jitter scales the delay between 1 - jitter and 1 + jitter", "[retrypolicy]") {
  const RetryPolicy p;
  CHECK(p.delayFor(0, 0.0) == 400ms);
  CHECK(p.delayFor(0, 1.0) == 600ms);
}

TEST_CASE("a custom policy is honoured", "[retrypolicy]") {
  RetryPolicy p;
  p.baseDelay = 1ms;
  p.maxDelay = 3ms;
  p.jitter = 0.0;
  CHECK(p.delayFor(0, 0.9) == 1ms);
  CHECK(p.delayFor(1, 0.9) == 2ms);
  CHECK(p.delayFor(2, 0.9) == 3ms);
  CHECK(p.delayFor(40, 0.9) == 3ms);
}

TEST_CASE("jitter applies after the cap, so the longest sleep is maxDelay * (1 + jitter)", "[retrypolicy]") {
  const RetryPolicy p;
  CHECK(p.delayFor(7, 0.0) == 24000ms);
  CHECK(p.delayFor(7, 1.0) == 36000ms);
}

TEST_CASE("an attempt beyond the range of double still yields the cap", "[retrypolicy]") {
  const RetryPolicy p;
  CHECK(p.delayFor(1024, 0.5) == 30000ms);
  CHECK(p.delayFor(std::numeric_limits<int>::max(), 0.5) == 30000ms);
}
