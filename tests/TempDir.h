#pragma once

#include <filesystem>
#include <random>
#include <string>

// A fresh directory under the system temp dir, removed with everything in it on destruction.
struct TempDir {
  std::filesystem::path path;

  TempDir() {
    std::random_device rd;
    path = std::filesystem::temp_directory_path() / ("demdl-test-" + std::to_string(rd()));
    std::filesystem::create_directories(path);
  }
  ~TempDir() {
    std::error_code ignored;
    std::filesystem::remove_all(path, ignored);
  }
  TempDir(const TempDir&) = delete;
  TempDir& operator=(const TempDir&) = delete;
};
