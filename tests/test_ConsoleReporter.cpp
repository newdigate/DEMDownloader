#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <ostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "demdl/ConsoleReporter.h"

using Catch::Matchers::ContainsSubstring;
using demdl::ConsoleReporter;
using demdl::PixelEvent;
using demdl::PixelState;
using demdl::RowReport;
using demdl::RunPlan;
using demdl::RunSummary;
using demdl::StopReason;
using demdl::formatDuration;

namespace {

// The lines of `text`, without their newlines.
std::vector<std::string> linesOf(const std::string& text) {
  std::vector<std::string> lines;
  std::istringstream in(text);
  for (std::string line; std::getline(in, line);) lines.push_back(line);
  return lines;
}

// A string buffer that counts sync() calls, which is where std::ostream::flush() ends up, and
// remembers how many characters had been written at the last one.
class SyncCountingBuf : public std::stringbuf {
 public:
  int syncs = 0;
  std::size_t syncedSize = 0;

 protected:
  int sync() override {
    ++syncs;
    syncedSize = str().size();
    return std::stringbuf::sync();
  }
};

}  // namespace

TEST_CASE("formatDuration picks units by magnitude", "[consolereporter]") {
  CHECK(formatDuration(41.2) == "41.2s");
  CHECK(formatDuration(725.0) == "12m05s");
  CHECK(formatDuration(48667.0) == "13h31m");
  CHECK(formatDuration(50520.0) == "14h02m");
}

TEST_CASE("formatDuration rounds before it picks the unit and tames odd inputs", "[consolereporter]") {
  CHECK(formatDuration(59.94) == "59.9s");
  CHECK(formatDuration(59.95) == "1m00s");  // "%.1f" alone would print 60.0
  CHECK(formatDuration(60.0) == "1m00s");
  CHECK(formatDuration(3599.4) == "59m59s");
  CHECK(formatDuration(3599.6) == "1h00m");  // not 60m00s
  CHECK(formatDuration(-1.0) == "0.0s");
  CHECK(formatDuration(std::nan("")) == "0.0s");
  CHECK(formatDuration(1e300) == "1000000000h00m");  // capped at a billion hours
  CHECK(formatDuration(std::numeric_limits<double>::infinity()) == "1000000000h00m");
}

TEST_CASE("a saved row prints progress, rate and ETA", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRunStarted(RunPlan{3670016, 3670016, 1792, 1792});
  reporter.onRowSaved(RowReport{811, 1792, 2048, 0, 3, std::chrono::duration<double>(41.2), 1247296, 3670016});
  const std::string text = out.str();
  CHECK_THAT(text, ContainsSubstring("starting: 3670016 pixels in 1792 rows"));
  CHECK_THAT(text, ContainsSubstring("row 812/1792  done 1247296/3670016 (34.0%)  row: 2048 ok 0 nodata 3 failed "
                                     "41.2s  49.8 px/s  eta 13h31m"));
}

TEST_CASE("a row without elapsed time prints no ETA", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  reporter.onRowSaved(RowReport{0, 4, 10, 0, 0, std::chrono::duration<double>(0.0), 10, 40});
  CHECK(out.str() == "row 1/4  done 10/40 (25.0%)  row: 10 ok 0 nodata 0 failed 0.0s  0.0 px/s  eta --\n");
}

TEST_CASE("rate and ETA cover the last ten saved rows", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  // Row 0 attempts 100 pixels in an absurd 1e6 s. Rows 1 to 10 attempt 100 each (90 ok, 6 nodata,
  // 4 failed) in 2 s, so each of them adds 96 to the 2000 pixels' done count.
  reporter.onRowSaved(RowReport{0, 20, 100, 0, 0, std::chrono::duration<double>(1e6), 100, 2000});
  for (int row = 1; row <= 10; ++row) {
    reporter.onRowSaved(RowReport{row, 20, 90, 6, 4, std::chrono::duration<double>(2.0),
                                  100 + 96 * static_cast<std::uint64_t>(row), 2000});
  }
  const std::vector<std::string> lines = linesOf(out.str());
  REQUIRE(lines.size() == 11);
  // The tenth line still counts row 0: 1000 pixels in 1000018 s.
  CHECK_THAT(lines[9], ContainsSubstring("  0.0 px/s  "));
  // The eleventh drops it: 1000 pixels in 20 s is 50 px/s, and 940 pixels remain.
  CHECK(lines[10] == "row 11/20  done 1060/2000 (53.0%)  row: 90 ok 6 nodata 4 failed 2.0s  50.0 px/s  eta 18.8s");
}

TEST_CASE("the summary lists totals and the retry hint", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  RunSummary s;
  s.pixelsFetched = 3669980;
  s.noData = 12034;
  s.failed = 3;
  s.remainingNaN = 3;
  s.rowsSaved = 1792;
  s.elapsed = std::chrono::duration<double>(50520.0);
  reporter.onRunFinished(s);
  CHECK_THAT(out.str(),
             ContainsSubstring("done: 3669980 values, 12034 nodata, 3 failed, 1792 rows saved, 14h02m elapsed"));
  CHECK_THAT(out.str(), ContainsSubstring("rerun with the same arguments to retry the 3 failed pixels"));
  CHECK_THAT(out.str(), !ContainsSubstring("stopped:"));

  s.failed = 1;
  std::ostringstream one;
  ConsoleReporter oneFailure(one, false);
  oneFailure.onRunFinished(s);
  CHECK_THAT(one.str(), ContainsSubstring("rerun with the same arguments to retry the 1 failed pixel\n"));
}

TEST_CASE("a stopped run names the reason", "[consolereporter]") {
  std::ostringstream out;
  ConsoleReporter reporter(out, false);
  RunSummary s;
  s.stopReason = StopReason::Signal;
  s.stopMessage = "interrupted";
  reporter.onRunFinished(s);
  CHECK_THAT(out.str(), ContainsSubstring("stopped: interrupted"));
}

TEST_CASE("verbose mode prints one line per resolved pixel", "[consolereporter]") {
  std::ostringstream quiet;
  ConsoleReporter silent(quiet, false);
  silent.onPixelState(PixelEvent{12, 812, PixelState::Value, 123.4f, 1, {}});
  CHECK(quiet.str().empty());

  std::ostringstream out;
  ConsoleReporter verbose(out, true);
  verbose.onPixelState(PixelEvent{12, 812, PixelState::Value, 123.4f, 1, {}});
  verbose.onPixelState(PixelEvent{13, 812, PixelState::NoData, 0.0f, 1, {}});
  verbose.onPixelState(PixelEvent{14, 812, PixelState::Failed, 0.0f, 9, "http 500"});
  verbose.onPixelState(PixelEvent{15, 812, PixelState::Queued, 0.0f, 0, {}});
  verbose.onPixelState(PixelEvent{15, 812, PixelState::InFlight, 0.0f, 0, {}});
  verbose.onPixelState(PixelEvent{15, 812, PixelState::Failed, 0.0f, 1, "transport: SSL server verification failed"});
  verbose.onPixelState(PixelEvent{16, 812, PixelState::Value, 5.0f, 2, {}});
  // No attempt counted, as for an exception the Downloader caught: the line carries no count.
  verbose.onPixelState(PixelEvent{17, 812, PixelState::Failed, 0.0f, 0, "unknown exception"});
  verbose.onPixelState(PixelEvent{18, 812, PixelState::Failed, 0.0f, -1, "exception: boom"});
  CHECK(out.str() ==
        "x=12 y=812 123.4 m (1 attempt)\n"
        "x=13 y=812 nodata\n"
        "x=14 y=812 failed after 9 attempts: http 500\n"
        "x=15 y=812 failed after 1 attempt: transport: SSL server verification failed\n"
        "x=16 y=812 5.0 m (2 attempts)\n"
        "x=17 y=812 failed: unknown exception\n"
        "x=18 y=812 failed: exception: boom\n");
}

TEST_CASE("the starting, row and finish lines are flushed, pixel lines are not", "[consolereporter]") {
  SyncCountingBuf buf;
  std::ostream out(&buf);
  ConsoleReporter reporter(out, true);

  reporter.onRunStarted(RunPlan{40, 40, 4, 4});
  CHECK(buf.syncs >= 1);
  CHECK(buf.syncedSize == buf.str().size());

  const int beforePixels = buf.syncs;
  reporter.onPixelState(PixelEvent{0, 0, PixelState::Queued, 0.0f, 0, {}});
  reporter.onPixelState(PixelEvent{0, 0, PixelState::InFlight, 0.0f, 0, {}});
  reporter.onPixelState(PixelEvent{0, 0, PixelState::Value, 12.5f, 1, {}});
  reporter.onPixelState(PixelEvent{1, 0, PixelState::NoData, 0.0f, 1, {}});
  reporter.onPixelState(PixelEvent{2, 0, PixelState::Failed, 0.0f, 3, "http 500"});
  CHECK_THAT(buf.str(), ContainsSubstring("x=2 y=0 failed after 3 attempts: http 500\n"));
  CHECK(buf.syncs == beforePixels);

  reporter.onRowSaved(RowReport{0, 4, 1, 1, 1, std::chrono::duration<double>(1.0), 2, 40});
  CHECK(buf.syncs > beforePixels);
  CHECK(buf.syncedSize == buf.str().size());

  const int beforeFinish = buf.syncs;
  RunSummary s;
  s.failed = 1;
  s.stopReason = StopReason::Signal;
  s.stopMessage = "interrupted";
  reporter.onRunFinished(s);
  CHECK(buf.syncs > beforeFinish);
  CHECK(buf.syncedSize == buf.str().size());
}

TEST_CASE("concurrent pixel events print whole lines", "[consolereporter]") {
  constexpr int kThreads = 4;
  constexpr int kEvents = 1000;
  std::ostringstream out;
  ConsoleReporter reporter(out, true);
  std::atomic<bool> go{false};
  std::vector<std::thread> threads;
  for (int t = 0; t < kThreads; ++t) {
    threads.emplace_back([&reporter, &go, t] {
      while (!go.load()) std::this_thread::yield();
      for (int i = 0; i < kEvents; ++i) {
        if (i % 3 == 0) reporter.onPixelState(PixelEvent{i, t, PixelState::Value, static_cast<float>(i) + 0.5f, 1, {}});
        if (i % 3 == 1) reporter.onPixelState(PixelEvent{i, t, PixelState::NoData, 0.0f, 1, {}});
        if (i % 3 == 2) reporter.onPixelState(PixelEvent{i, t, PixelState::Failed, 0.0f, 3, "http 503"});
      }
    });
  }
  go = true;
  for (std::thread& thread : threads) thread.join();

  std::vector<std::string> expected;
  for (int t = 0; t < kThreads; ++t) {
    for (int i = 0; i < kEvents; ++i) {
      const std::string pixel = "x=" + std::to_string(i) + " y=" + std::to_string(t);
      if (i % 3 == 0) expected.push_back(pixel + " " + std::to_string(i) + ".5 m (1 attempt)");
      if (i % 3 == 1) expected.push_back(pixel + " nodata");
      if (i % 3 == 2) expected.push_back(pixel + " failed after 3 attempts: http 503");
    }
  }
  std::vector<std::string> lines = linesOf(out.str());
  CHECK(lines.size() == static_cast<std::size_t>(kThreads * kEvents));
  std::sort(lines.begin(), lines.end());
  std::sort(expected.begin(), expected.end());
  CHECK(lines == expected);
}
