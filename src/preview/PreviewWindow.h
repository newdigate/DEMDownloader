#pragma once

#include <cassert>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <vector>

#include "demdl/DemRaster.h"
#include "demdl/ProgressSink.h"
#include "demdl/StopToken.h"

struct GLFWwindow;

namespace demdl {

class Shader;

// A GLFW window showing the raster as a status map. Construct it and call pump() on the main
// thread; the sink methods may be called from any thread.
class PreviewWindow : public ProgressSink {
 public:
  // Throws std::runtime_error when GLFW cannot initialise or open a window, when the shaders do not
  // build, or when the raster is wider or taller than the GPU's maximum texture size. Before it
  // throws, it destroys the window, if it opened one, and terminates GLFW.
  PreviewWindow(const DemRaster& raster, StopToken& stop);
  ~PreviewWindow() override;
  PreviewWindow(const PreviewWindow&) = delete;
  PreviewWindow& operator=(const PreviewWindow&) = delete;

  // Polls events and redraws every 500 ms. The first pump that finds the window closed requests
  // StopReason::WindowClosed and hides the window at once, so the close shows while in-flight
  // requests drain; a hidden window is not drawn again.
  void pump();

  void onRunStarted(const RunPlan& plan) override;
  void onPixelState(const PixelEvent& event) override;
  void onRowSaved(const RowReport& report) override;
  void onRunFinished(const RunSummary& summary) override;

 private:
  enum Cell : std::uint8_t { kTodo = 0, kNoData = 1, kQueued = 2, kInFlight = 3, kValue = 4, kFailed = 5 };

  void initGl();
  void redraw();
  std::size_t index(int x, int y) const {
    assert(x >= 0 && x < width_ && y >= 0 && y < height_);
    return static_cast<std::size_t>(y) * static_cast<std::size_t>(width_) + static_cast<std::size_t>(x);
  }

  int width_;
  int height_;
  StopToken& stop_;

  // Written by the sink methods on any thread, under mutex_.
  std::mutex mutex_;
  std::vector<std::uint8_t> cells_;
  std::vector<float> values_;
  float minValue_;
  float maxValue_;
  double percent_ = 0.0;

  // The main thread's own.
  GLFWwindow* window_ = nullptr;
  unsigned int texture_ = 0;
  unsigned int vao_ = 0;
  unsigned int vbo_ = 0;
  unsigned int ebo_ = 0;
  std::unique_ptr<Shader> shader_;
  std::vector<std::uint8_t> cellsCopy_;  // redraw's snapshot of cells_, sized up front
  std::vector<float> valuesCopy_;        // and of values_
  std::vector<unsigned char> rgb_;
  std::chrono::steady_clock::time_point lastRedraw_;
  bool closed_ = false;  // the window was closed: the stop is requested and the window hidden
};

}  // namespace demdl
