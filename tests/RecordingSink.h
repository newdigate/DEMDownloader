#pragma once

#include <algorithm>
#include <mutex>
#include <optional>
#include <vector>

#include "demdl/ProgressSink.h"

// Records every event. Reading the vectors is safe once the run has returned or been joined.
struct RecordingSink : demdl::ProgressSink {
  std::mutex m;
  demdl::RunPlan plan;
  std::vector<demdl::PixelEvent> pixels;
  std::vector<demdl::RowReport> rows;
  std::optional<demdl::RunSummary> summary;

  void onRunStarted(const demdl::RunPlan& p) override {
    std::lock_guard<std::mutex> lock(m);
    plan = p;
  }
  void onPixelState(const demdl::PixelEvent& e) override {
    std::lock_guard<std::mutex> lock(m);
    pixels.push_back(e);
  }
  void onRowSaved(const demdl::RowReport& r) override {
    std::lock_guard<std::mutex> lock(m);
    rows.push_back(r);
  }
  void onRunFinished(const demdl::RunSummary& s) override {
    std::lock_guard<std::mutex> lock(m);
    summary = s;
  }

  [[nodiscard]] int count(demdl::PixelState state) {
    std::lock_guard<std::mutex> lock(m);
    return static_cast<int>(std::count_if(pixels.begin(), pixels.end(),
                                          [state](const demdl::PixelEvent& e) { return e.state == state; }));
  }
};
