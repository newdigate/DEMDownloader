#include "preview/PreviewWindow.h"

#ifdef __APPLE__
#include <OpenGL/gl3.h>
#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>
#else
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#endif

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

#include "preview/Palette.h"
#include "preview/Shader.h"

namespace demdl {

namespace {

const char* const kVertexShader = R"glsl(
#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec2 aTexCoord;
out vec2 TexCoord;
void main() {
  gl_Position = vec4(aPos, 1.0);
  TexCoord = aTexCoord;
}
)glsl";

const char* const kFragmentShader = R"glsl(
#version 330 core
out vec4 FragColor;
in vec2 TexCoord;
uniform sampler2D texture1;
void main() {
  FragColor = texture(texture1, TexCoord);
}
)glsl";

void onFramebufferResized(GLFWwindow*, int width, int height) { glViewport(0, 0, width, height); }

}  // namespace

PreviewWindow::PreviewWindow(const DemRaster& raster, StopToken& stop)
    : width_(raster.width()),
      height_(raster.height()),
      stop_(stop),
      cells_(raster.pixelCount(), kTodo),
      values_(raster.pixelCount(), 0.0f),
      minValue_(std::numeric_limits<float>::infinity()),
      maxValue_(-std::numeric_limits<float>::infinity()),
      rgb_(raster.pixelCount() * 3, 0) {
  for (int y = 0; y < height_; ++y) {
    for (int x = 0; x < width_; ++x) {
      const float v = raster.at(x, y);
      const std::size_t i = index(x, y);
      if (std::isnan(v)) continue;
      if (v == DemRaster::kNoData) {
        cells_[i] = kNoData;
        continue;
      }
      cells_[i] = kValue;
      values_[i] = v;
      minValue_ = std::min(minValue_, v);
      maxValue_ = std::max(maxValue_, v);
    }
  }
  const std::uint64_t done = raster.pixelCount() - raster.countNaN();
  percent_ = 100.0 * static_cast<double>(done) / static_cast<double>(raster.pixelCount());
  if (!glfwInit()) throw std::runtime_error("GLFW initialisation failed");
  try {
    initGl();
  } catch (...) {
    // The destructor does not run for a constructor that throws, so undo the setup here: the shader
    // while its context is still current, then the window, then GLFW.
    shader_.reset();
    if (window_ != nullptr) glfwDestroyWindow(window_);
    glfwTerminate();
    throw;
  }
  lastRedraw_ = std::chrono::steady_clock::now() - std::chrono::seconds(1);  // draw on the first pump
}

PreviewWindow::~PreviewWindow() {
  if (window_ != nullptr) {
    glfwMakeContextCurrent(window_);
    shader_.reset();
    glDeleteTextures(1, &texture_);
    glDeleteBuffers(1, &ebo_);
    glDeleteBuffers(1, &vbo_);
    glDeleteVertexArrays(1, &vao_);
    glfwDestroyWindow(window_);
  }
  glfwTerminate();
}

// Opens the window and creates the GL objects. GLFW is initialised; if this throws, the constructor
// cleans up.
void PreviewWindow::initGl() {
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  window_ = glfwCreateWindow(640, 480, "dl_elevation", nullptr, nullptr);
  if (window_ == nullptr) throw std::runtime_error("GLFW could not create a window; is a display available?");
  glfwMakeContextCurrent(window_);
  glfwSetFramebufferSizeCallback(window_, onFramebufferResized);
#ifndef __APPLE__
  glewExperimental = GL_TRUE;
  if (glewInit() != GLEW_OK) throw std::runtime_error("GLEW initialisation failed");
#endif

  // The whole raster is uploaded as one texture.
  int maxTextureSize = 0;
  glGetIntegerv(GL_MAX_TEXTURE_SIZE, &maxTextureSize);
  if (width_ > maxTextureSize || height_ > maxTextureSize) {
    throw std::runtime_error("the raster is " + std::to_string(width_) + " x " + std::to_string(height_) +
                             ", larger than this GPU's maximum texture size " + std::to_string(maxTextureSize) +
                             "; run without --preview");
  }

  shader_ = std::make_unique<Shader>(kVertexShader, kFragmentShader);

  const float vertices[] = {
      //  x      y     z     u     v
      1.0f,  1.0f, 0.0f, 1.0f, 0.0f,  // top right
      1.0f,  -1.0f, 0.0f, 1.0f, 1.0f,  // bottom right
      -1.0f, -1.0f, 0.0f, 0.0f, 1.0f,  // bottom left
      -1.0f, 1.0f,  0.0f, 0.0f, 0.0f,  // top left
  };
  const unsigned int indices[] = {0, 1, 3, 1, 2, 3};
  glGenVertexArrays(1, &vao_);
  glGenBuffers(1, &vbo_);
  glGenBuffers(1, &ebo_);
  glBindVertexArray(vao_);
  glBindBuffer(GL_ARRAY_BUFFER, vbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof vertices, vertices, GL_STATIC_DRAW);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ebo_);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof indices, indices, GL_STATIC_DRAW);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(0));
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, 5 * sizeof(float), reinterpret_cast<void*>(3 * sizeof(float)));
  glEnableVertexAttribArray(1);

  glGenTextures(1, &texture_);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  // Rows are width*3 bytes, which need not be a multiple of the default 4-byte alignment.
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
}

void PreviewWindow::pump() {
  glfwPollEvents();
  if (closed_) return;
  if (glfwWindowShouldClose(window_)) {
    // The run still drains its in-flight requests and saves, so hide the window now rather than when
    // it ends. The flag makes this happen once.
    closed_ = true;
    stop_.request(StopReason::WindowClosed);
    glfwHideWindow(window_);
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  if (now - lastRedraw_ >= std::chrono::milliseconds(500)) {
    redraw();
    lastRedraw_ = now;
  }
}

void PreviewWindow::redraw() {
  char title[64];
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const float lo = minValue_;
    const float span = maxValue_ > lo ? maxValue_ - lo : 1.0f;
    for (std::size_t i = 0; i < cells_.size(); ++i) {
      Rgb c = kTodoColour;
      switch (cells_[i]) {
        case kTodo: break;
        case kNoData: c = kNoDataColour; break;
        case kQueued: c = kQueuedColour; break;
        case kInFlight: c = kInFlightColour; break;
        case kFailed: c = kFailedColour; break;
        case kValue: c = valueColour((values_[i] - lo) / span); break;
      }
      rgb_[3 * i] = c.r;
      rgb_[3 * i + 1] = c.g;
      rgb_[3 * i + 2] = c.b;
    }
    std::snprintf(title, sizeof title, "dl_elevation %.1f%%", percent_);
  }
  glfwSetWindowTitle(window_, title);
  glClearColor(0.1f, 0.1f, 0.1f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, texture_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width_, height_, 0, GL_RGB, GL_UNSIGNED_BYTE, rgb_.data());
  shader_->use();
  shader_->setInt("texture1", 0);
  glBindVertexArray(vao_);
  glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, nullptr);
  glfwSwapBuffers(window_);
}

void PreviewWindow::onRunStarted(const RunPlan&) {}

void PreviewWindow::onPixelState(const PixelEvent& e) {
  std::lock_guard<std::mutex> lock(mutex_);
  const std::size_t i = index(e.x, e.y);
  switch (e.state) {
    case PixelState::Queued: cells_[i] = kQueued; break;
    case PixelState::InFlight: cells_[i] = kInFlight; break;
    case PixelState::NoData: cells_[i] = kNoData; break;
    case PixelState::Failed: cells_[i] = kFailed; break;
    case PixelState::Value:
      cells_[i] = kValue;
      values_[i] = e.value;
      minValue_ = std::min(minValue_, e.value);
      maxValue_ = std::max(maxValue_, e.value);
      break;
  }
}

void PreviewWindow::onRowSaved(const RowReport& r) {
  std::lock_guard<std::mutex> lock(mutex_);
  percent_ = r.pixelsTotal > 0 ? 100.0 * static_cast<double>(r.pixelsDoneTotal) / static_cast<double>(r.pixelsTotal)
                               : 100.0;
}

void PreviewWindow::onRunFinished(const RunSummary&) {}

}  // namespace demdl
