#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>

#include <csignal>
#include <cstdint>
#include <vector>

#include "app/ExitCodes.h"
#include "demdl/ProgressSink.h"

using demdl::RunSummary;
using demdl::StopReason;
using demdl::cli::exitCodeFor;
using demdl::cli::kExitComplete;
using demdl::cli::kExitError;
using demdl::cli::kExitIncomplete;
using demdl::cli::kExitStopped;

namespace {

// Every combination of the fields exitCodeFor reads, each with and without pixels left and failures.
std::vector<RunSummary> everySummaryShape() {
  std::vector<RunSummary> shapes;
  for (const StopReason reason : {StopReason::None, StopReason::Signal, StopReason::WindowClosed,
                                  StopReason::ConsecutiveFailures, StopReason::WriteError}) {
    for (const std::uint64_t remaining : {std::uint64_t{0}, std::uint64_t{3}}) {
      for (const std::uint64_t failed : {std::uint64_t{0}, std::uint64_t{3}}) {
        for (const bool writeFailed : {false, true}) {
          RunSummary s;
          s.remainingNaN = remaining;
          s.failed = failed;
          s.stopReason = reason;
          s.checkpointWriteFailed = writeFailed;
          shapes.push_back(s);
        }
      }
    }
  }
  return shapes;
}

}  // namespace

// One case per row of the exit-code table in spec section 10, then the checkpoint-write rule.

TEST_CASE("a run that leaves no NaN pixel exits 0", "[exitcodes]") {
  STATIC_REQUIRE(kExitComplete == 0);
  RunSummary s;
  s.pixelsFetched = 500;
  s.rowsSaved = 5;
  CHECK(exitCodeFor(s) == kExitComplete);
  // A stop requested after the last row was saved leaves the file complete all the same.
  s.stopReason = StopReason::Signal;
  CHECK(exitCodeFor(s) == kExitComplete);
}

TEST_CASE("a usage, configuration, file or internal error is exit code 1, which no run summary maps to",
          "[exitcodes]") {
  STATIC_REQUIRE(kExitError == 1);
  for (const RunSummary& s : everySummaryShape()) CHECK(exitCodeFor(s) != kExitError);
}

TEST_CASE("a run stopped early with pixels left exits 2 whatever stopped it", "[exitcodes]") {
  STATIC_REQUIRE(kExitStopped == 2);
  const StopReason reason = GENERATE(StopReason::Signal, StopReason::WindowClosed,
                                     StopReason::ConsecutiveFailures, StopReason::WriteError);
  RunSummary s;
  s.pixelsFetched = 300;
  s.rowsSaved = 3;
  s.remainingNaN = 200;
  s.stopReason = reason;
  s.checkpointWriteFailed = reason == StopReason::WriteError;  // as the Downloader reports a failed save
  CHECK(exitCodeFor(s) == kExitStopped);
}

TEST_CASE("a run that reaches the end with failed pixels exits 3", "[exitcodes]") {
  STATIC_REQUIRE(kExitIncomplete == 3);
  RunSummary s;
  s.pixelsFetched = 497;
  s.failed = 3;
  s.remainingNaN = 3;
  s.rowsSaved = 5;
  CHECK(exitCodeFor(s) == kExitIncomplete);
}

TEST_CASE("a second interrupt kills the process, so no run summary maps to 130 or 143", "[exitcodes]") {
  // A shell reports 128 plus the number of the signal that killed a process.
  for (const RunSummary& s : everySummaryShape()) {
    CHECK(exitCodeFor(s) != 128 + SIGINT);
    CHECK(exitCodeFor(s) != 128 + SIGTERM);
  }
}

TEST_CASE("a failed checkpoint write exits 2 even when no NaN pixel is counted", "[exitcodes]") {
  RunSummary s;
  s.pixelsFetched = 500;
  s.remainingNaN = 0;
  s.stopReason = StopReason::WriteError;
  s.checkpointWriteFailed = true;
  CHECK(exitCodeFor(s) == kExitStopped);
}

TEST_CASE("a failed checkpoint write after a signal exits 2", "[exitcodes]") {
  // The signal came first and stays the recorded reason. With pixels left the reason alone gives 2;
  // with none counted, only the write failure does.
  RunSummary s;
  s.remainingNaN = GENERATE(as<std::uint64_t>{}, 0, 200);
  s.stopReason = StopReason::Signal;
  s.checkpointWriteFailed = true;
  CHECK(exitCodeFor(s) == kExitStopped);
}
