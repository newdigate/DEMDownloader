#pragma once

// Runs an executable of this build as a child process for the end-to-end tests, and the small file and text helpers
// their checks share. Moved from test_EndToEnd.cpp, so that dl_elevation's and dem2mesh's end-to-end tests start,
// wait for and judge their programs the same way.

#include <catch2/catch_test_macros.hpp>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include "FakeFetcher.h"  // Gate

extern char** environ;

// How long a child may run, and how long a test waits for anything it expects of one.
inline constexpr std::chrono::seconds kDeadline{20};

inline std::string readFile(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

inline void writeFile(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream file(path, std::ios::binary);
  file << bytes;
  file.close();
  REQUIRE(file.good());
}

// True when the file holds exactly `bytes`. A bool, so that a failed check prints no binary data.
inline bool fileHolds(const std::filesystem::path& path, const std::string& bytes) { return readFile(path) == bytes; }

// How many times `needle` occurs in `text` without overlapping.
inline std::size_t occurrences(std::string_view text, std::string_view needle) {
  std::size_t count = 0;
  for (std::size_t at = text.find(needle); at != std::string_view::npos; at = text.find(needle, at + needle.size())) {
    ++count;
  }
  return count;
}

// "exit code 3" or "signal 2", from a wait status.
inline std::string describeStatus(int status) {
  return WIFEXITED(status) ? "exit code " + std::to_string(WEXITSTATUS(status))
                           : "signal " + std::to_string(WTERMSIG(status));
}

// What a child left behind once it ended.
struct Finished {
  int status = 0;  // as waitpid reported it
  std::string out;
  std::string err;

  // The exit code, or -1 when a signal ended the process.
  [[nodiscard]] int exitCode() const { return WIFEXITED(status) ? WEXITSTATUS(status) : -1; }
  // The signal that ended the process, or 0 when it exited.
  [[nodiscard]] int termSignal() const { return WIFSIGNALED(status) ? WTERMSIG(status) : 0; }
};

// The executable started with `args`, stdin from /dev/null, stdout and stderr to files in `dir`. SIGINT,
// SIGTERM, SIGHUP and SIGPIPE start at their default actions and unblocked, whatever this process has (the
// httplib::Server constructor ignores SIGPIPE here, and nohup would ignore SIGHUP), so main's own settings
// are what the tests see. A child still running when the object goes away is killed. Every wait fails at
// once, with the child's stderr, when the child has ended before what it waits for.
class Child {
 public:
  Child(const std::filesystem::path& executable, const std::vector<std::string>& args,
        const std::filesystem::path& dir)
      : name_(executable.filename().string()), outPath_(dir / "stdout.txt"), errPath_(dir / "stderr.txt") {
    std::vector<char*> argv{const_cast<char*>(executable.c_str())};
    for (const std::string& arg : args) argv.push_back(const_cast<char*>(arg.c_str()));
    argv.push_back(nullptr);

    posix_spawn_file_actions_t actions;
    REQUIRE(posix_spawn_file_actions_init(&actions) == 0);
    REQUIRE(posix_spawn_file_actions_addopen(&actions, STDIN_FILENO, "/dev/null", O_RDONLY, 0) == 0);
    REQUIRE(posix_spawn_file_actions_addopen(&actions, STDOUT_FILENO, outPath_.c_str(),
                                             O_WRONLY | O_CREAT | O_TRUNC, 0644) == 0);
    REQUIRE(posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, errPath_.c_str(),
                                             O_WRONLY | O_CREAT | O_TRUNC, 0644) == 0);
    posix_spawnattr_t attributes;
    REQUIRE(posix_spawnattr_init(&attributes) == 0);
    sigset_t defaults;
    sigemptyset(&defaults);
    sigaddset(&defaults, SIGINT);
    sigaddset(&defaults, SIGTERM);
    sigaddset(&defaults, SIGHUP);
    sigaddset(&defaults, SIGPIPE);
    sigset_t unblocked;
    sigemptyset(&unblocked);
    int flags = POSIX_SPAWN_SETSIGDEF | POSIX_SPAWN_SETSIGMASK;
#ifdef POSIX_SPAWN_CLOEXEC_DEFAULT
    // macOS: only the three descriptors above reach the child, not the server's accepted sockets.
    flags |= POSIX_SPAWN_CLOEXEC_DEFAULT;
#endif
    REQUIRE(posix_spawnattr_setsigdefault(&attributes, &defaults) == 0);
    REQUIRE(posix_spawnattr_setsigmask(&attributes, &unblocked) == 0);
    REQUIRE(posix_spawnattr_setflags(&attributes, static_cast<short>(flags)) == 0);
    const int spawned = posix_spawn(&pid_, executable.c_str(), &actions, &attributes, argv.data(), environ);
    posix_spawnattr_destroy(&attributes);
    posix_spawn_file_actions_destroy(&actions);
    REQUIRE(spawned == 0);
  }
  ~Child() {
    if (pid_ > 0) {
      kill(pid_, SIGKILL);
      waitpid(pid_, nullptr, 0);
    }
  }
  Child(const Child&) = delete;
  Child& operator=(const Child&) = delete;

  void sendSignal(int number) const {
    REQUIRE(pid_ > 0);  // once the child is reaped, pid_ is -1, and kill(-1, ...) signals every process we own
    REQUIRE(kill(pid_, number) == 0);
  }

  // Waits until the child's stderr contains `text`.
  void awaitStderr(std::string_view text) {
    awaitCondition([this, text] { return readFile(errPath_).find(text) != std::string::npos; });
  }

  // Waits until at least `count` requests are held at `gate`.
  void awaitHeld(const Gate& gate, int count) {
    awaitCondition([&gate, count] { return gate.waiting.load() >= count; });
  }

  // Waits for the child to end. Past kDeadline it kills the child and fails the test.
  Finished wait() {
    const auto deadline = std::chrono::steady_clock::now() + kDeadline;
    while (!ended()) {
      if (std::chrono::steady_clock::now() > deadline) {
        kill(pid_, SIGKILL);
        waitpid(pid_, nullptr, 0);
        pid_ = -1;
        INFO("stderr:\n" << readFile(errPath_));
        FAIL(name_ << " did not exit within " << kDeadline.count() << " s");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return {status_, readFile(outPath_), readFile(errPath_)};
  }

 private:
  // True once the child has ended; its wait status is then in status_.
  bool ended() {
    if (pid_ <= 0) return true;
    const pid_t done = waitpid(pid_, &status_, WNOHANG);
    const int waitError = errno;
    if (done == pid_) {
      pid_ = -1;
      return true;
    }
    if (done < 0 && waitError != EINTR) {
      pid_ = -1;  // the pid may belong to another process now, so the destructor must not kill it
      FAIL("waitpid failed: " << std::generic_category().message(waitError));
    }
    return false;
  }

  // Polls `satisfied` until it holds. Fails at once when the child ends first, and after kDeadline.
  template <typename Condition>
  void awaitCondition(Condition satisfied) {
    const auto deadline = std::chrono::steady_clock::now() + kDeadline;
    while (!satisfied()) {
      if (ended()) {
        if (satisfied()) return;  // it came true just before the child ended
        INFO("stderr:\n" << readFile(errPath_));
        FAIL(name_ << " ended first, with " << describeStatus(status_));
      }
      if (std::chrono::steady_clock::now() > deadline) {
        INFO("stderr:\n" << readFile(errPath_));
        FAIL("still waiting after " << kDeadline.count() << " s");
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }

  std::string name_;  // the executable's file name, for the failure messages
  std::filesystem::path outPath_;
  std::filesystem::path errPath_;
  pid_t pid_ = -1;
  int status_ = 0;  // as waitpid reported it, once the child has ended
};
