#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <httplib.h>

#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <vector>

#include "FakeFetcher.h"  // Gate
#include "TempDir.h"
#include "demdl/DemRaster.h"

extern char** environ;

using namespace std::chrono_literals;
using Catch::Matchers::ContainsSubstring;
using Catch::Matchers::StartsWith;
using demdl::DemRaster;

namespace {

// The dl_elevation built by this CMake tree. These tests point it at a loopback server only.
const char* const kExecutable = DL_ELEVATION_EXE;

// How long a child may run, and how long a test waits for anything it expects of one.
constexpr std::chrono::seconds kDeadline{20};

// Pixel (I, J) as one number, for the server's settings.
constexpr int pixelKey(int i, int j) { return i * 1000 + j; }

// The whole of `text` as a non-negative int, otherwise -1.
int parseIndex(const std::string& text) {
  int value = -1;
  const char* const end = text.data() + text.size();
  const auto [ptr, ec] = std::from_chars(text.data(), end, value);
  return ec == std::errc() && ptr == end && value >= 0 ? value : -1;
}

// A WMS on a loopback port. GetFeatureInfo for pixel (I, J) answers "BAND 0: <I*10+J>", except at the
// NoData pixel, which gets a plain body without a band, and at the failing pixel, which gets a 500.
// Requests are counted as they arrive. Every request after the first `holdAfter` waits at `gate` until
// the test releases it, which keeps requests in flight for as long as a test needs; once released, the
// gate stays open.
struct LoopbackWms {
  httplib::Server server;
  std::thread thread;
  int port = 0;
  std::atomic<int> requests{0};
  std::atomic<int> noDataPixel{-1};   // a pixelKey, or -1 for none
  std::atomic<int> failingPixel{-1};  // a pixelKey, or -1 for none
  std::atomic<int> holdAfter{-1};     // -1 holds nothing
  Gate gate;

  LoopbackWms() {
    server.Get("/wms", [this](const httplib::Request& req, httplib::Response& res) { respond(req, res); });
    port = server.bind_to_any_port("127.0.0.1");
    REQUIRE(port > 0);
    thread = std::thread([this] { server.listen_after_bind(); });
    server.wait_until_ready();
  }
  ~LoopbackWms() {
    gate.release();  // a held request would keep stop() waiting
    server.stop();
    thread.join();
  }
  LoopbackWms(const LoopbackWms&) = delete;
  LoopbackWms& operator=(const LoopbackWms&) = delete;

  void respond(const httplib::Request& req, httplib::Response& res) {
    const int arrived = ++requests;
    const int hold = holdAfter.load();
    if (hold >= 0 && arrived > hold) gate.wait();
    const int i = parseIndex(req.get_param_value("I"));
    const int j = parseIndex(req.get_param_value("J"));
    if (i < 0 || j < 0) {
      res.status = 400;
    } else if (pixelKey(i, j) == failingPixel.load()) {
      res.status = 500;
    } else if (pixelKey(i, j) == noDataPixel.load()) {
      res.set_content("GetFeatureInfo results:\n", "text/plain");
    } else {
      res.set_content("GetFeatureInfo results:\nBAND 0: " + std::to_string(i * 10 + j) + "\n", "text/plain");
    }
  }

  std::string url() const { return "http://127.0.0.1:" + std::to_string(port) + "/wms"; }
};

std::string readFile(const std::filesystem::path& path) {
  std::ifstream in(path, std::ios::binary);
  return {std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>()};
}

// What a child left behind once it exited.
struct Finished {
  int status = -1;  // the exit code, or minus the number of the signal that ended the process
  std::string out;
  std::string err;
};

// dl_elevation started with `args`, stdin from /dev/null, stdout and stderr to files in `dir`. SIGINT and
// SIGTERM start at their default actions and unblocked, whatever this process has, so main's handlers
// alone decide what a signal does. A child still running when the object goes away is killed.
class Child {
 public:
  Child(const std::vector<std::string>& args, const std::filesystem::path& dir)
      : outPath_(dir / "stdout.txt"), errPath_(dir / "stderr.txt") {
    std::vector<char*> argv{const_cast<char*>(kExecutable)};
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
    const int spawned = posix_spawn(&pid_, kExecutable, &actions, &attributes, argv.data(), environ);
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

  const std::filesystem::path& errPath() const { return errPath_; }

  void signal(int number) const { REQUIRE(kill(pid_, number) == 0); }

  // Waits for the child to exit. Past kDeadline it kills the child and fails the test.
  Finished wait() {
    const auto deadline = std::chrono::steady_clock::now() + kDeadline;
    int status = 0;
    for (;;) {
      const pid_t done = waitpid(pid_, &status, WNOHANG);
      const int waitError = errno;
      if (done == pid_) break;
      if (done < 0 && waitError != EINTR) FAIL("waitpid failed: " << std::generic_category().message(waitError));
      if (std::chrono::steady_clock::now() > deadline) {
        kill(pid_, SIGKILL);
        waitpid(pid_, &status, 0);
        pid_ = -1;
        FAIL("dl_elevation did not exit within " << kDeadline.count() << " s; stderr:\n" << readFile(errPath_));
      }
      std::this_thread::sleep_for(10ms);
    }
    pid_ = -1;
    return {WIFEXITED(status) ? WEXITSTATUS(status) : -WTERMSIG(status), readFile(outPath_), readFile(errPath_)};
  }

 private:
  std::filesystem::path outPath_;
  std::filesystem::path errPath_;
  pid_t pid_ = -1;
};

Finished runToExit(const std::vector<std::string>& args, const std::filesystem::path& dir) {
  Child child(args, dir);
  return child.wait();
}

// Polls `path` until it contains `text`; false once kDeadline has passed.
bool awaitText(const std::filesystem::path& path, std::string_view text) {
  const auto deadline = std::chrono::steady_clock::now() + kDeadline;
  while (readFile(path).find(text) == std::string::npos) {
    if (std::chrono::steady_clock::now() > deadline) return false;
    std::this_thread::sleep_for(10ms);
  }
  return true;
}

// The example box on a width x height grid, fetched from `wms` into `output`, followed by `extra`.
std::vector<std::string> commandLine(const LoopbackWms& wms, const std::filesystem::path& output, int width,
                                     int height, const std::vector<std::string>& extra) {
  std::vector<std::string> args{"--north", "-34.1845418", "--south", "-34.2163447", "--west", "18.4185791",
                                "--east", "18.4625244", "--width", std::to_string(width), "--height",
                                std::to_string(height), "--output", output.string(), "--wms-url", wms.url()};
  args.insert(args.end(), extra.begin(), extra.end());
  return args;
}

// Every pixel of the file holds I*10+J, except kNoData at `noData` and NaN at `failed` (pixelKeys, or -1).
void checkPixels(const std::filesystem::path& path, int width, int height, int noData, int failed) {
  const DemRaster raster = DemRaster::load(path);
  REQUIRE(raster.width() == width);
  REQUIRE(raster.height() == height);
  for (int y = 0; y < height; ++y) {
    for (int x = 0; x < width; ++x) {
      CAPTURE(x, y);
      const float value = raster.at(x, y);
      if (pixelKey(x, y) == noData) {
        CHECK(value == DemRaster::kNoData);
      } else if (pixelKey(x, y) == failed) {
        CHECK(std::isnan(value));
      } else {
        CHECK(value == static_cast<float>(x * 10 + y));
      }
    }
  }
}

}  // namespace

TEST_CASE("a full run fills the file, keeps NoData and leaves a failed pixel NaN", "[e2e]") {
  LoopbackWms wms;
  wms.noDataPixel = pixelKey(1, 2);
  wms.failingPixel = pixelKey(3, 1);
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const Finished result = runToExit(commandLine(wms, output, 4, 3, {"--threads", "2", "--max-retries", "0"}), dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.status == 3);
  for (const char* line : {"starting:", "row 1/3", "row 2/3", "row 3/3", "done:", "retry the 1 failed pixel"}) {
    CHECK_THAT(result.out, ContainsSubstring(line));
  }
  CHECK_THAT(result.err, ContainsSubstring("warning: first failed request: http 500 (not retried)"));
  CHECK(wms.requests.load() == 12);  // one attempt per pixel
  checkPixels(output, 4, 3, pixelKey(1, 2), pixelKey(3, 1));
}

TEST_CASE("a rerun requests only the failed pixel and completes the file", "[e2e]") {
  LoopbackWms wms;
  wms.noDataPixel = pixelKey(1, 2);
  wms.failingPixel = pixelKey(3, 1);
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const std::vector<std::string> args = commandLine(wms, output, 4, 3, {"--threads", "2", "--max-retries", "0"});
  REQUIRE(runToExit(args, dir.path).status == 3);

  wms.failingPixel = -1;
  wms.requests = 0;
  const Finished rerun = runToExit(args, dir.path);
  INFO("stdout:\n" << rerun.out << "stderr:\n" << rerun.err);
  CHECK(rerun.status == 0);
  CHECK(wms.requests.load() == 1);  // NoData is final, and every other pixel already has its value
  checkPixels(output, 4, 3, pixelKey(1, 2), -1);
}

TEST_CASE("a dry run prints the plan, makes no request and writes nothing", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const Finished result = runToExit(commandLine(wms, output, 4, 3, {"--dry-run"}), dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.status == 0);
  CHECK_THAT(result.out, StartsWith("dry run"));
  CHECK_THAT(result.out, ContainsSubstring("  first request: " + wms.url() + "?SERVICE=WMS&"));
  CHECK(wms.requests.load() == 0);
  CHECK_FALSE(std::filesystem::exists(output));
}

TEST_CASE("SIGINT stops the run after the requests in flight, and a rerun completes the file", "[e2e]") {
  LoopbackWms wms;
  wms.holdAfter = 10;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const std::vector<std::string> args = commandLine(wms, output, 16, 8, {"--threads", "2"});
  {
    Child child(args, dir.path);
    // Ten requests are answered and the eleventh is held, so the run cannot end before main, which polls
    // every 100 ms, has seen the stop and announced it. A fixed response delay would race that poll.
    REQUIRE(wms.gate.awaitArrivals(1, kDeadline));
    child.signal(SIGINT);
    REQUIRE(awaitText(child.errPath(), "stopping: interrupted; waiting for "));
    wms.gate.release();
    const Finished stopped = child.wait();
    INFO("stdout:\n" << stopped.out << "stderr:\n" << stopped.err);
    CHECK(stopped.status == 2);
    CHECK_THAT(stopped.out, ContainsSubstring("stopped: interrupted"));
    CHECK_THAT(stopped.out, ContainsSubstring("resume the remaining"));
  }
  const DemRaster partial = DemRaster::load(output);
  CHECK(partial.countNaN() > 0);
  CHECK(partial.countNaN() < partial.pixelCount());  // the partial first row was saved

  const Finished rerun = runToExit(args, dir.path);  // the gate stays open, so nothing is held now
  INFO("stdout:\n" << rerun.out << "stderr:\n" << rerun.err);
  CHECK(rerun.status == 0);
  checkPixels(output, 16, 8, -1, -1);
}

TEST_CASE("a second SIGINT exits at once with status 130", "[e2e]") {
  LoopbackWms wms;
  wms.holdAfter = 0;  // every request is held, so only the second signal can end the run
  const TempDir dir;
  Child child(commandLine(wms, dir.path / "dem.tif", 4, 3, {"--threads", "2"}), dir.path);
  REQUIRE(wms.gate.awaitArrivals(1, kDeadline));
  child.signal(SIGINT);
  REQUIRE(awaitText(child.errPath(), "stopping: interrupted"));  // the first signal has been handled
  child.signal(SIGINT);
  const Finished result = child.wait();
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.status == 130);
}

TEST_CASE("a usage error exits 1 before any request", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const Finished result = runToExit(commandLine(wms, output, 4, 3, {"--threads", "0"}), dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.status == 1);
  CHECK_THAT(result.err, ContainsSubstring("error: threads must be at least 1"));
  CHECK(wms.requests.load() == 0);
  CHECK_FALSE(std::filesystem::exists(output));
}
