#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <exception>
#include <filesystem>
#include <future>
#include <iostream>
#include <memory>
#include <optional>
#include <string>
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

// SIGINT and SIGTERM: the first requests a stop, the second exits at once. Nothing is printed here;
// main announces the stop when it sees the token raised.
void onSignal(int) {
  if (g_signalCount.fetch_add(1) >= 1) std::_Exit(demdl::cli::kExitSecondSignal);
  g_stopToken.request(demdl::StopReason::Signal);
}

void printPlan(const demdl::Config& cfg, const demdl::DemRaster& raster, bool created) {
  using demdl::formatCoordinate;
  std::cout << "dry run\n"
            << "  box: north " << formatCoordinate(cfg.box.north) << ", south " << formatCoordinate(cfg.box.south)
            << ", east " << formatCoordinate(cfg.box.east) << ", west " << formatCoordinate(cfg.box.west) << "\n"
            << "  grid: " << cfg.width << " x " << cfg.height << "\n"
            << "  output: " << cfg.output.string() << (created ? " (will be created)" : " (exists, will be resumed)")
            << "\n"
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
  if (parsed.exitNow) return parsed.exitCode;  // 0 after --help or --version, cli::kExitUsage after an error
  if (const std::string err = cfg.validate(); !err.empty()) {
    std::cerr << "error: " << err << "\n";
    return cli::kExitUsage;
  }
#ifndef DL_ELEVATION_HAVE_PREVIEW
  if (cfg.preview) {
    std::cerr << "error: this build has no preview support; configure with -DDL_ELEVATION_PREVIEW=ON\n";
    return cli::kExitUsage;
  }
#endif

  RetryPolicy policy;
  policy.maxRetries = cfg.maxRetries;

  std::optional<DemRaster> raster;
  bool created = false;
  try {
    if (!cfg.overwrite && std::filesystem::exists(cfg.output)) {
      raster = loadForResume(cfg);
      if (!raster) return cli::kExitUsage;
    } else {
      raster = DemRaster::create(cfg.width, cfg.height, cfg.box, RasterMetadata{cfg.layer, cfg.wmsUrl});
      created = true;
    }
    {
      // A probe, discarded at once: the constructor throws for a WMS URL or an HTTP client it cannot use,
      // which would otherwise surface only through the run's future. It sends no request.
      const ElevationClient probe(cfg, policy);
    }
    if (cfg.dryRun) {
      printPlan(cfg, *raster, created);
      return cli::kExitComplete;
    }
    // A new file, or one with work left, is written before the first request, so that an unwritable
    // directory or a full disk fails here and a new file can be opened while it fills. A complete file is
    // not rewritten.
    if (raster->countNaN() > 0) raster->save(cfg.output);
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return cli::kExitUsage;
  }

  std::signal(SIGINT, onSignal);
  std::signal(SIGTERM, onSignal);
  std::signal(SIGPIPE, SIG_IGN);  // ElevationClient relies on it: httplib's client does not suppress SIGPIPE

  std::unique_ptr<ProgressSink> sink;
#ifdef DL_ELEVATION_HAVE_PREVIEW
  PreviewWindow* preview = nullptr;
  if (cfg.preview) {
    try {
      auto window = std::make_unique<PreviewWindow>(*raster, g_stopToken);
      preview = window.get();
      sink = std::move(window);
    } catch (const std::exception& e) {
      std::cerr << "error: " << e.what() << "\n";
      return cli::kExitUsage;
    }
  }
#endif
  if (!sink) sink = std::make_unique<ConsoleReporter>(std::cout, cfg.verbose);

  const FetcherFactory factory = [&cfg, policy](int) -> std::unique_ptr<PixelFetcher> {
    return std::make_unique<ElevationClient>(cfg, policy);
  };
  std::optional<Downloader> downloader;
  std::future<RunSummary> result;
  try {
    downloader.emplace(*raster, cfg.output, factory, *sink, g_stopToken,
                       DownloaderOptions{cfg.threads, cfg.maxConsecutiveFailures});
    // Off the main thread, which belongs to the preview window when there is one.
    result = std::async(std::launch::async, [&downloader] { return downloader->run(); });
  } catch (const std::exception& e) {
    std::cerr << "error: " << e.what() << "\n";
    return cli::kExitUsage;
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
    return cli::kExitUsage;
  } catch (...) {
    std::cerr << "error: unknown exception\n";
    return cli::kExitUsage;
  }
#ifdef DL_ELEVATION_HAVE_PREVIEW
  if (preview != nullptr) {
    writeSummary(std::cout, summary);  // the window prints nothing, so the finish lines come from here
    std::cout.flush();
  }
#endif
  return cli::exitCodeFor(summary);
}
