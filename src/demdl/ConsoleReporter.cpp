#include "demdl/ConsoleReporter.h"

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <ostream>

namespace demdl {

std::string formatDuration(double seconds) {
  // NaN fails the comparison as well, so it joins the negatives at zero. The cap keeps llround in range.
  constexpr double kMaxSeconds = 3600.0 * 1e9;  // a billion hours
  if (!(seconds > 0.0)) seconds = 0.0;
  if (seconds > kMaxSeconds) seconds = kMaxSeconds;
  char buf[48];
  // Round first and choose the unit from the rounded value, which rules out "60.0s" and "60m00s".
  const long long tenths = std::llround(seconds * 10.0);
  if (tenths < 600) {
    std::snprintf(buf, sizeof buf, "%lld.%llds", tenths / 10, tenths % 10);
    return buf;
  }
  const long long total = std::llround(seconds);
  if (total < 3600) {
    std::snprintf(buf, sizeof buf, "%lldm%02llds", total / 60, total % 60);
    return buf;
  }
  std::snprintf(buf, sizeof buf, "%lldh%02lldm", total / 3600, (total % 3600) / 60);
  return buf;
}

void writeSummary(std::ostream& out, const RunSummary& s) {
  out << "done: " << s.pixelsFetched << " values, " << s.noData << " nodata, " << s.failed << " failed, "
      << s.rowsSaved << " rows saved, " << formatDuration(s.elapsed.count()) << " elapsed\n";
  if (s.stopReason != StopReason::None) out << "stopped: " << s.stopMessage << "\n";
  if (s.remainingNaN == 0) return;
  out << "rerun with the same arguments to ";
  if (s.failed > 0 && s.remainingNaN == s.failed) {
    out << "retry the " << s.failed << (s.failed == 1 ? " failed pixel" : " failed pixels");
  } else {
    out << "resume the remaining " << s.remainingNaN << (s.remainingNaN == 1 ? " pixel" : " pixels");
    if (s.failed > 0) out << " (" << s.failed << " failed this run)";
  }
  out << "\n";
}

ConsoleReporter::ConsoleReporter(std::ostream& out, bool verbose) : out_(out), verbose_(verbose) {}

void ConsoleReporter::onRunStarted(const RunPlan& plan) {
  std::lock_guard<std::mutex> lock(mutex_);
  window_.fill(RowSample{});
  next_ = 0;
  pixelsTodo_ = plan.pixelsTodo;
  attempted_ = 0;
  out_ << "starting: " << plan.pixelsTodo << " pixels in " << plan.rowsTodo << " rows remaining of "
       << plan.pixelsTotal << " pixels in " << plan.rowsTotal << " rows\n" << std::flush;
}

void ConsoleReporter::onPixelState(const PixelEvent& e) {
  // Queued and InFlight print nothing, so they return before taking the lock.
  if (!verbose_ || e.state == PixelState::Queued || e.state == PixelState::InFlight) return;
  std::lock_guard<std::mutex> lock(mutex_);
  switch (e.state) {
    case PixelState::Value: {
      char value[48];  // room for -FLT_MAX at one decimal
      std::snprintf(value, sizeof value, "%.1f", static_cast<double>(e.value));
      out_ << "x=" << e.x << " y=" << e.y << " " << value << " m (" << e.attempts
           << (e.attempts == 1 ? " attempt)" : " attempts)") << "\n";
      break;
    }
    case PixelState::NoData:
      out_ << "x=" << e.x << " y=" << e.y << " nodata\n";
      break;
    case PixelState::Failed:
      // An exception the Downloader caught leaves attempts at 0, so there is no count to print.
      out_ << "x=" << e.x << " y=" << e.y << " failed";
      if (e.attempts >= 1) out_ << " after " << e.attempts << (e.attempts == 1 ? " attempt" : " attempts");
      out_ << ": " << e.error << "\n";
      break;
    case PixelState::Queued:
    case PixelState::InFlight:
      break;
  }
}

void ConsoleReporter::onRowSaved(const RowReport& r) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::uint64_t rowAttempted =
      static_cast<std::uint64_t>(r.ok) + static_cast<std::uint64_t>(r.noData) + static_cast<std::uint64_t>(r.failed);
  attempted_ += rowAttempted;
  window_[next_] = RowSample{static_cast<double>(rowAttempted), r.elapsed.count()};
  next_ = (next_ + 1) % window_.size();
  double windowPixels = 0.0;
  double windowSeconds = 0.0;
  for (const RowSample& w : window_) {
    windowPixels += w.pixels;
    windowSeconds += w.seconds;
  }
  const double rate = windowSeconds > 0.0 ? windowPixels / windowSeconds : 0.0;
  // "done" and the percentage describe the file; the ETA counts only pixels the run has yet to attempt.
  const double percent =
      r.pixelsTotal > 0 ? 100.0 * static_cast<double>(r.pixelsDoneTotal) / static_cast<double>(r.pixelsTotal) : 100.0;
  const std::uint64_t remaining = pixelsTodo_ > attempted_ ? pixelsTodo_ - attempted_ : 0;
  const std::string eta = rate > 0.0 ? formatDuration(static_cast<double>(remaining) / rate) : std::string("--");
  const std::string rowTime = formatDuration(r.elapsed.count());
  char line[256];
  std::snprintf(line, sizeof line,
                "row %d/%d  done %llu/%llu (%.1f%%)  row: %d ok %d nodata %d failed %s  %.1f px/s  eta %s", r.row + 1,
                r.rowsTotal, static_cast<unsigned long long>(r.pixelsDoneTotal),
                static_cast<unsigned long long>(r.pixelsTotal), percent, r.ok, r.noData, r.failed, rowTime.c_str(),
                rate, eta.c_str());
  out_ << line << "\n" << std::flush;
}

void ConsoleReporter::onRunFinished(const RunSummary& s) {
  std::lock_guard<std::mutex> lock(mutex_);
  writeSummary(out_, s);
  out_.flush();
}

}  // namespace demdl
