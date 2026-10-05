#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <httplib.h>

#include <atomic>
#include <cerrno>
#include <charconv>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <string>
#include <string_view>
#include <system_error>
#include <thread>
#include <utility>
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

void writeFile(const std::filesystem::path& path, const std::string& bytes) {
  std::ofstream file(path, std::ios::binary);
  file << bytes;
  file.close();
  REQUIRE(file.good());
}

// True when the file holds exactly `bytes`. A bool, so that a failed check prints no binary data.
bool fileHolds(const std::filesystem::path& path, const std::string& bytes) { return readFile(path) == bytes; }

// Makes a directory read-only (0555) for as long as it lives and writable (0755) again afterwards, even when a
// check fails, so that the TempDir can remove it.
class ReadOnlyDirectory {
 public:
  explicit ReadOnlyDirectory(std::filesystem::path path) : path_(std::move(path)) {
    REQUIRE(chmod(path_.c_str(), 0555) == 0);
  }
  ~ReadOnlyDirectory() { chmod(path_.c_str(), 0755); }
  ReadOnlyDirectory(const ReadOnlyDirectory&) = delete;
  ReadOnlyDirectory& operator=(const ReadOnlyDirectory&) = delete;

 private:
  std::filesystem::path path_;
};

// How many times `needle` occurs in `text` without overlapping.
std::size_t occurrences(std::string_view text, std::string_view needle) {
  std::size_t count = 0;
  for (std::size_t at = text.find(needle); at != std::string_view::npos; at = text.find(needle, at + needle.size())) {
    ++count;
  }
  return count;
}

// "exit code 3" or "signal 2", from a wait status.
std::string describeStatus(int status) {
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

// dl_elevation started with `args`, stdin from /dev/null, stdout and stderr to files in `dir`. SIGINT,
// SIGTERM, SIGHUP and SIGPIPE start at their default actions and unblocked, whatever this process has (the
// httplib::Server constructor ignores SIGPIPE here, and nohup would ignore SIGHUP), so main's own settings
// are what the tests see. A child still running when the object goes away is killed. Every wait fails at
// once, with the child's stderr, when the child has ended before what it waits for.
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
        FAIL("dl_elevation did not exit within " << kDeadline.count() << " s");
      }
      std::this_thread::sleep_for(10ms);
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
        FAIL("dl_elevation ended first, with " << describeStatus(status_));
      }
      if (std::chrono::steady_clock::now() > deadline) {
        INFO("stderr:\n" << readFile(errPath_));
        FAIL("still waiting after " << kDeadline.count() << " s");
      }
      std::this_thread::sleep_for(10ms);
    }
  }

  std::filesystem::path outPath_;
  std::filesystem::path errPath_;
  pid_t pid_ = -1;
  int status_ = 0;  // as waitpid reported it, once the child has ended
};

Finished runToExit(const std::vector<std::string>& args, const std::filesystem::path& dir) {
  Child child(args, dir);
  return child.wait();
}

// The example box on a width x height grid, fetched from `wmsUrl` into `output`, followed by `extra`.
std::vector<std::string> commandLine(const std::string& wmsUrl, const std::filesystem::path& output, int width,
                                     int height, const std::vector<std::string>& extra) {
  std::vector<std::string> args{"--north", "-34.1845418", "--south", "-34.2163447", "--west", "18.4185791",
                                "--east", "18.4625244", "--width", std::to_string(width), "--height",
                                std::to_string(height), "--output", output.string(), "--wms-url", wmsUrl};
  args.insert(args.end(), extra.begin(), extra.end());
  return args;
}

std::vector<std::string> commandLine(const LoopbackWms& wms, const std::filesystem::path& output, int width,
                                     int height, const std::vector<std::string>& extra) {
  return commandLine(wms.url(), output, width, height, extra);
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
  CHECK(result.exitCode() == 3);
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
  REQUIRE(runToExit(args, dir.path).exitCode() == 3);

  wms.failingPixel = -1;
  wms.requests = 0;
  const Finished rerun = runToExit(args, dir.path);
  INFO("stdout:\n" << rerun.out << "stderr:\n" << rerun.err);
  CHECK(rerun.exitCode() == 0);
  CHECK(wms.requests.load() == 1);  // NoData is final, and every other pixel already has its value
  checkPixels(output, 4, 3, pixelKey(1, 2), -1);
}

TEST_CASE("a file made for other parameters is refused and left untouched", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  REQUIRE(runToExit(commandLine(wms, output, 4, 3, {"--threads", "2"}), dir.path).exitCode() == 0);
  const std::string before = readFile(output);

  wms.requests = 0;
  const Finished result = runToExit(commandLine(wms, output, 5, 3, {"--threads", "2"}), dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK_THAT(result.err, ContainsSubstring("was created for different parameters"));
  CHECK_THAT(result.err, ContainsSubstring("  width: file 4, requested 5"));
  CHECK_THAT(result.err, ContainsSubstring("use --overwrite"));
  CHECK(wms.requests.load() == 0);
  CHECK(fileHolds(output, before));
}

TEST_CASE("an output that is not a GeoTIFF is refused untouched, and --overwrite replaces it", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const std::string junk = "these bytes are not a GeoTIFF\n";
  writeFile(output, junk);
  std::vector<std::string> args = commandLine(wms, output, 4, 3, {"--threads", "2"});
  {
    const Finished refused = runToExit(args, dir.path);
    INFO("stdout:\n" << refused.out << "stderr:\n" << refused.err);
    CHECK(refused.exitCode() == 1);
    CHECK(occurrences(refused.err, "use --overwrite") == 1);
    CHECK(wms.requests.load() == 0);
    CHECK(fileHolds(output, junk));
  }
  args.push_back("--overwrite");
  {
    std::vector<std::string> dryRun = args;
    dryRun.push_back("--dry-run");
    const Finished plan = runToExit(dryRun, dir.path);
    INFO("stdout:\n" << plan.out << "stderr:\n" << plan.err);
    CHECK(plan.exitCode() == 0);
    CHECK_THAT(plan.out, ContainsSubstring("  output: " + output.string() + " (exists, will be overwritten)\n"));
    CHECK(fileHolds(output, junk));
  }
  const Finished replaced = runToExit(args, dir.path);
  INFO("stdout:\n" << replaced.out << "stderr:\n" << replaced.err);
  CHECK(replaced.exitCode() == 0);
  CHECK(wms.requests.load() == 12);
  checkPixels(output, 4, 3, -1, -1);
}

TEST_CASE("a GeoTIFF that dl_elevation did not write is refused with the load error's own hint", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const std::vector<std::string> args = commandLine(wms, output, 4, 3, {"--threads", "2"});
  REQUIRE(runToExit(args, dir.path).exitCode() == 0);
  // The same number of bytes, so the TIFF stays readable; only its description no longer names dl_elevation.
  std::string bytes = readFile(output);
  const std::string marker = "dl_elevation;layer=";
  const std::size_t at = bytes.find(marker);
  REQUIRE(at != std::string::npos);
  REQUIRE(bytes.find(marker, at + 1) == std::string::npos);
  bytes.replace(at, 2, "xx");
  writeFile(output, bytes);

  wms.requests = 0;
  const Finished result = runToExit(args, dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK_THAT(result.err, ContainsSubstring("was not written by dl_elevation"));
  CHECK(occurrences(result.err, "use --overwrite") == 1);  // the message names it, so main adds no hint
  CHECK(wms.requests.load() == 0);
  CHECK(fileHolds(output, bytes));
}

TEST_CASE("a complete file in a read-only directory is left alone, without an error", "[e2e]") {
  if (geteuid() == 0) SKIP("root ignores directory permissions");
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path sub = dir.path / "readonly";
  REQUIRE(std::filesystem::create_directory(sub));
  const std::filesystem::path output = sub / "dem.tif";
  const std::vector<std::string> args = commandLine(wms, output, 4, 3, {"--threads", "2"});
  REQUIRE(runToExit(args, dir.path).exitCode() == 0);
  const std::string before = readFile(output);

  wms.requests = 0;
  const ReadOnlyDirectory readOnly(sub);
  const Finished result = runToExit(args, dir.path);  // stdout and stderr go to the writable TempDir
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK(wms.requests.load() == 0);
  CHECK(fileHolds(output, before));
}

TEST_CASE("a dry run prints the plan, makes no request and writes nothing", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const Finished result = runToExit(commandLine(wms, output, 4, 3, {"--dry-run"}), dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK_THAT(result.out, StartsWith("dry run"));
  CHECK_THAT(result.out, ContainsSubstring("\n  wms: " + wms.url() + "\n  tls: none (plain http)\n"));
  CHECK_THAT(result.out, ContainsSubstring("  first request: " + wms.url() + "?SERVICE=WMS&"));
  CHECK(wms.requests.load() == 0);
  CHECK_FALSE(std::filesystem::exists(output));
}

TEST_CASE("the dry run names how https requests would be secured", "[e2e]") {
  const std::string mode = GENERATE(as<std::string>{}, "trust store", "--ca-bundle", "--insecure");
  CAPTURE(mode);
  // A dry run sends nothing, so this https URL on a closed loopback port is never contacted.
  const std::string url = "https://127.0.0.1:1/wms";
  const TempDir dir;
  const std::filesystem::path bundle = dir.path / "bundle.pem";
  writeFile(bundle, "");  // --ca-bundle only requires that the file exists
  std::vector<std::string> args = commandLine(url, dir.path / "dem.tif", 4, 3, {"--dry-run"});
  std::string tls = "verify with the system trust store";
  if (mode == "--ca-bundle") {
    args.insert(args.end(), {"--ca-bundle", bundle.string()});
    tls = "verify with " + bundle.string();
  } else if (mode == "--insecure") {
    args.push_back("--insecure");
    tls = "certificate verification disabled (--insecure)";
  }
  const Finished result = runToExit(args, dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 0);
  CHECK_THAT(result.out, ContainsSubstring("\n  wms: " + url + "\n  tls: " + tls + "\n"));
}

TEST_CASE("a dry run into a missing directory fails and creates nothing", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path missing = dir.path / "nodir";
  const Finished result = runToExit(commandLine(wms, missing / "dem.tif", 4, 3, {"--dry-run"}), dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK_THAT(result.err,
             ContainsSubstring("error: cannot write to " + missing.string() + ": No such file or directory"));
  CHECK(wms.requests.load() == 0);
  CHECK_FALSE(std::filesystem::exists(missing));
}

TEST_CASE("a first SIGINT, SIGTERM or SIGHUP stops after the request in flight, and a rerun completes the file",
          "[e2e]") {
  const int stopSignal = GENERATE(SIGINT, SIGTERM, SIGHUP);
  CAPTURE(stopSignal);
  LoopbackWms wms;
  wms.holdAfter = 10;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const std::vector<std::string> args = commandLine(wms, output, 16, 8, {"--threads", "1"});
  {
    Child child(args, dir.path);
    // Ten requests are answered and the eleventh is held, so the run cannot end before main, which polls
    // every 100 ms, has seen the stop and announced it. A fixed response delay would race that poll. With
    // one worker, exactly that request is in flight.
    child.awaitHeld(wms.gate, 1);
    child.sendSignal(stopSignal);
    child.awaitStderr("stopping: interrupted; waiting for 1 in-flight request\n");
    std::this_thread::sleep_for(250ms);  // at least two more polls with the token raised
    wms.gate.release();
    const Finished stopped = child.wait();
    INFO("stdout:\n" << stopped.out << "stderr:\n" << stopped.err);
    CHECK(stopped.exitCode() == 2);
    CHECK(occurrences(stopped.err, "stopping:") == 1);
    CHECK_THAT(stopped.out, ContainsSubstring("stopped: interrupted"));
    CHECK_THAT(stopped.out, ContainsSubstring("resume the remaining"));
  }
  const DemRaster partial = DemRaster::load(output);
  CHECK(partial.countNaN() > 0);
  CHECK(partial.countNaN() < partial.pixelCount());  // the partial first row was saved

  const Finished rerun = runToExit(args, dir.path);  // the gate stays open, so nothing is held now
  INFO("stdout:\n" << rerun.out << "stderr:\n" << rerun.err);
  CHECK(rerun.exitCode() == 0);
  checkPixels(output, 16, 8, -1, -1);
}

TEST_CASE("a second SIGINT ends the process by SIGINT, so a calling shell stops too", "[e2e]") {
  LoopbackWms wms;
  wms.holdAfter = 0;  // every request is held, so no row is saved and only a signal can end the run
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  Child child(commandLine(wms, output, 4, 3, {"--threads", "2"}), dir.path);
  child.awaitHeld(wms.gate, 2);  // both workers' first requests
  {
    // Only the save before the first request can have written the file.
    REQUIRE(std::filesystem::exists(output));
    const DemRaster early = DemRaster::load(output);
    CHECK(early.countNaN() == early.pixelCount());
  }
  child.sendSignal(SIGINT);
  child.awaitStderr("stopping: interrupted; waiting for 2 in-flight requests\n");  // the first was handled
  child.sendSignal(SIGINT);
  const Finished result = child.wait();
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.termSignal() == SIGINT);
}

TEST_CASE("a checkpoint that cannot be written stops the run, says so on stderr and keeps the last save", "[e2e]") {
  if (geteuid() == 0) SKIP("root ignores directory permissions");
  LoopbackWms wms;
  wms.holdAfter = 0;  // the first row's requests wait until the directory is read-only
  const TempDir dir;
  const std::filesystem::path sub = dir.path / "out";
  REQUIRE(std::filesystem::create_directory(sub));
  const std::filesystem::path output = sub / "dem.tif";
  Child child(commandLine(wms, output, 4, 3, {"--threads", "2"}), dir.path);
  child.awaitHeld(wms.gate, 2);
  const std::string early = readFile(output);
  REQUIRE_FALSE(early.empty());
  const ReadOnlyDirectory readOnly(sub);  // stdout and stderr go to the writable TempDir
  wms.gate.release();
  const Finished result = child.wait();
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 2);
  CHECK_THAT(result.err, ContainsSubstring("error: checkpoint write failed: "));
  CHECK(fileHolds(output, early));
}

TEST_CASE("a usage error exits 1 before any request", "[e2e]") {
  LoopbackWms wms;
  const TempDir dir;
  const std::filesystem::path output = dir.path / "dem.tif";
  const Finished result = runToExit(commandLine(wms, output, 4, 3, {"--threads", "0"}), dir.path);
  INFO("stdout:\n" << result.out << "stderr:\n" << result.err);
  CHECK(result.exitCode() == 1);
  CHECK_THAT(result.err, ContainsSubstring("error: threads must be at least 1"));
  CHECK(wms.requests.load() == 0);
  CHECK_FALSE(std::filesystem::exists(output));
}
