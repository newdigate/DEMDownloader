#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <exception>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <utility>
#include <vector>

#include "app/Cli.h"
#include "app/ExitCodes.h"
#include "demdl/Config.h"
#include "demdl/ConsoleReporter.h"
#include "demdl/DemRaster.h"
#include "demdl/Downloader.h"
#include "demdl/ElevationClient.h"
#include "demdl/PixelFetcher.h"
#include "demdl/ProgressSink.h"
#include "demdl/RetryPolicy.h"
#include "demdl/StopToken.h"
#include "demdl/WmsQuery.h"
#ifdef DL_ELEVATION_HAVE_PREVIEW
#include "preview/PreviewWindow.h"
#endif

namespace {

// The signal handler touches these two and nothing else: lock-free atomics only (the token holds two),
// constant-initialised so that they are ready before any signal can arrive.
constinit demdl::StopToken g_stopToken;
constinit std::atomic<int> g_signalCount{0};
static_assert(std::atomic<int>::is_always_lock_free);

constexpr const char* kOverwriteHint = "use --overwrite to start fresh or choose another --output\n";

// SIGINT and SIGTERM. The first requests a stop, which main announces when it sees the token raised:
// nothing may print here. A second ends the process by that signal's default action, so a calling shell
// sees the signal, reports 130 or 143 and stops a script or loop running this program. Every call here is
// async-signal-safe.
void onSignal(int number) {
  if (g_signalCount.fetch_add(1) == 0) {
    g_stopToken.request(demdl::StopReason::Signal);
    return;
  }
  struct sigaction defaultAction {};
  defaultAction.sa_handler = SIG_DFL;
  sigemptyset(&defaultAction.sa_mask);
  sigaction(number, &defaultAction, nullptr);
  // The signal is blocked while its own handler runs; unblocked, it is delivered before raise returns.
  sigset_t self;
  sigemptyset(&self);
  sigaddset(&self, number);
  pthread_sigmask(SIG_UNBLOCK, &self, nullptr);
  raise(number);
}

// SIGINT and SIGTERM go to onSignal, each blocking both while it runs, and interrupted system calls
// restart. SIGPIPE is ignored, which ElevationClient relies on: httplib's client does not suppress it.
void installSignalHandlers() {
  struct sigaction stop {};
  stop.sa_handler = onSignal;
  stop.sa_flags = SA_RESTART;
  sigemptyset(&stop.sa_mask);
  sigaddset(&stop.sa_mask, SIGINT);
  sigaddset(&stop.sa_mask, SIGTERM);
  sigaction(SIGINT, &stop, nullptr);
  sigaction(SIGTERM, &stop, nullptr);
  struct sigaction ignore {};
  ignore.sa_handler = SIG_IGN;
  sigemptyset(&ignore.sa_mask);
  sigaction(SIGPIPE, &ignore, nullptr);
}

// Joins the thread when it goes out of scope, so that no way out of main leaves it running.
class ThreadJoiner {
 public:
  explicit ThreadJoiner(std::thread& thread) : thread_(thread) {}
  ~ThreadJoiner() {
    if (thread_.joinable()) thread_.join();
  }
  ThreadJoiner(const ThreadJoiner&) = delete;
  ThreadJoiner& operator=(const ThreadJoiner&) = delete;

 private:
  std::thread& thread_;
};

// `outputPlan` says what the run would do with the file, such as "exists, will be resumed".
void printPlan(const demdl::Config& cfg, const demdl::DemRaster& raster, const char* outputPlan) {
  using demdl::formatCoordinate;
  std::cout << "dry run\n"
            << "  box: north " << formatCoordinate(cfg.box.north) << ", south " << formatCoordinate(cfg.box.south)
            << ", east " << formatCoordinate(cfg.box.east) << ", west " << formatCoordinate(cfg.box.west) << "\n"
            << "  grid: " << cfg.width << " x " << cfg.height << "\n"
            << "  output: " << cfg.output.string() << " (" << outputPlan << ")\n"
            << "  wms: " << cfg.wmsUrl << "\n"
            << "  layer: " << cfg.layer << "\n"
            << "  threads: " << cfg.threads << ", max retries: " << cfg.maxRetries
            << ", max consecutive failures: " << cfg.maxConsecutiveFailures << "\n";
  const std::vector<int> rows = raster.rowsWithWork();
  std::cout << "  rows remaining: " << rows.size() << " of " << raster.height() << "\n"
            << "  pixels remaining: " << raster.countNaN() << " of " << raster.pixelCount() << "\n";
  if (!rows.empty()) {
    const int row = rows.front();
    const int x = raster.todoInRow(row).front();
    std::cout << "  first request: " << demdl::parseWmsUrl(cfg.wmsUrl)->origin << demdl::featureInfoUrl(cfg, x, row)
              << "\n";
  }
}

// The existing output, loaded for resuming. Empty, with the reason printed, when it cannot be resumed.
std::optional<demdl::DemRaster> loadForResume(const demdl::Config& cfg) {
  std::optional<demdl::DemRaster> raster;
  try {
    raster = demdl::DemRaster::load(cfg.output);
  } catch (const std::exception& e) {
    const std::string what = e.what();
    std::cerr << "error: " << what << "\n";
    // DemRaster's message for a file that dl_elevation did not write names --overwrite already.
    if (what.find("--overwrite") == std::string::npos) std::cerr << kOverwriteHint;
    return std::nullopt;
  }
  std::string why;
  if (!raster->matches(cfg, why)) {
    std::cerr << "error: " << cfg.output.string() << " was created for different parameters:\n"
              << why << kOverwriteHint;
    return std::nullopt;
  }
  if (const std::string warning = raster->wmsUrlWarning(cfg); !warning.empty()) {
    std::cerr << "warning: " << warning << "\n";
  }
  return raster;
}

// Empty when the output's directory ("." when the path names none) exists and files can be created in it,
// otherwise the problem.
std::string outputDirectoryProblem(const std::filesystem::path& output) {
  std::filesystem::path dir = output.parent_path();
  if (dir.empty()) dir = ".";
  if (access(dir.c_str(), W_OK | X_OK) == 0) return {};
  const int error = errno;
  return "cannot write to " + dir.string() + ": " + std::generic_category().message(error);
}

}  // namespace

int main(int argc, char** argv) {
  using namespace std::chrono_literals;
  using namespace demdl;

  // Tied, every stderr write would first flush stdout, so a warning from a worker thread (ElevationClient
  // writes two, once per process) could push out half of a line the reporter is still writing.
  // sync_with_stdio(false) must never be called: the standard streams may be shared between threads only
  // while they are synchronised with stdio.
  std::cerr.tie(nullptr);

  Config cfg;
  const cli::ParseOutcome parsed = cli::parseArgs(argc, argv, cfg, std::cout, std::cerr);
  if (parsed.exitNow) return parsed.exitCode;  // 0 after --help or --version, cli::kExitError after an error
  if (const std::string err = cfg.validate(); !err.empty()) {
    std::cerr << "error: " << err << "\n";
    return cli::kExitError;
  }
#ifndef DL_ELEVATION_HAVE_PREVIEW
  if (cfg.preview) {
    std::cerr << "error: this build has no preview support; configure with -DDL_ELEVATION_PREVIEW=ON\n";
    return cli::kExitError;
  }
#endif

  RetryPolicy policy;
  policy.maxRetries = cfg.maxRetries;

  std::optional<DemRaster> raster;
  std::unique_ptr<ProgressSink> sink;
#ifdef DL_ELEVATION_HAVE_PREVIEW
  PreviewWindow* preview = nullptr;
#endif
  try {
    const bool outputExists = std::filesystem::exists(cfg.output);
    const char* outputPlan = "will be created";
    if (outputExists && !cfg.overwrite) {
      raster = loadForResume(cfg);
      if (!raster) return cli::kExitError;
      outputPlan = "exists, will be resumed";
    } else {
      raster = DemRaster::create(cfg.width, cfg.height, cfg.box, RasterMetadata{cfg.layer, cfg.wmsUrl});
      if (outputExists) outputPlan = "exists, will be overwritten";
    }
    // A file with work left, new or resumed, is written before the first request, so that an unusable
    // directory fails here, in a dry run as well, and a full disk at that first save; a new file can then be
    // opened while it fills. A complete file is not rewritten, so its directory need not be writable.
    const bool writesOutput = raster->countNaN() > 0;
    if (writesOutput) {
      if (const std::string problem = outputDirectoryProblem(cfg.output); !problem.empty()) {
        std::cerr << "error: " << problem << "\n";
        return cli::kExitError;
      }
    }
    {
      // A probe, discarded at once: the constructor throws for a WMS URL or an HTTP client it cannot use,
      // which would otherwise surface only through the run's future. It sends no request.
      const ElevationClient probe(cfg, policy);
    }
    if (cfg.dryRun) {
      printPlan(cfg, *raster, outputPlan);
      return cli::kExitComplete;
    }
    // The sink comes before the first write, so a preview window that cannot open (no display, a raster
    // larger than the GPU's textures) ends the run with nothing written, and --overwrite has not yet
    // replaced the existing file.
#ifdef DL_ELEVATION_HAVE_PREVIEW
    if (cfg.preview) {
      auto window = std::make_unique<PreviewWindow>(*raster, g_stopToken);
      preview = window.get();
      sink = std::move(window);
    }
#endif
    if (!sink) sink = std::make_unique<ConsoleReporter>(std::cout, cfg.verbose);
    if (writesOutput) raster->save(cfg.output);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return cli::kExitError;
  }

  installSignalHandlers();

  const FetcherFactory factory = [&cfg, policy](int) -> std::unique_ptr<PixelFetcher> {
    return std::make_unique<ElevationClient>(cfg, policy);
  };
  std::optional<Downloader> downloader;
  std::promise<RunSummary> promise;
  std::future<RunSummary> result = promise.get_future();
  std::thread runner;
  // Declared after everything the run thread uses, so the thread is joined, on every way out of main, before
  // any of that goes away. It must also have ended before exit() starts its cleanup: its end runs
  // thread-local destructors, such as OpenSSL's per-thread state from the clients the factory made, and
  // libc++'s std::async would detach it.
  // Any early return added once the runner has started must request a stop first, or this waits for the whole run.
  const ThreadJoiner joinRunner(runner);
  try {
    downloader.emplace(*raster, cfg.output, factory, *sink, g_stopToken,
                       DownloaderOptions{cfg.threads, cfg.maxConsecutiveFailures});
    // Off the main thread, which belongs to the preview window when there is one.
    runner = std::thread([&downloader, &promise] {
      try {
        promise.set_value(downloader->run());
      } catch (...) {
        promise.set_exception(std::current_exception());
      }
    });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return cli::kExitError;
  }

  bool announced = false;
  while (result.wait_for(100ms) != std::future_status::ready) {
#ifdef DL_ELEVATION_HAVE_PREVIEW
    if (preview != nullptr) preview->pump();
#endif
    if (!announced && g_stopToken.requested()) {
      announced = true;
      // One string in one insertion, on stderr: the reporter owns stdout and writes under its own mutex,
      // so a line written to stdout from here could land inside one of its lines.
      const std::string line = std::string("stopping: ") + describe(g_stopToken.reason()) + "; waiting for " +
                               std::to_string(downloader->inFlight()) + " in-flight requests\n";
      std::cerr << line;
    }
  }

  RunSummary summary;
  try {
    summary = result.get();
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return cli::kExitError;
  } catch (...) {
    std::cerr << "error: unknown exception\n";
    return cli::kExitError;
  }
#ifdef DL_ELEVATION_HAVE_PREVIEW
  if (preview != nullptr) {
    writeSummary(std::cout, summary);  // the window prints nothing, so the finish lines come from here
    std::cout.flush();
  }
#endif
  return cli::exitCodeFor(summary);
}
