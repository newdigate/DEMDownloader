# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`dl_elevation` scrapes a Digital Elevation Model (DEM) from a WMS server one pixel at a time, one `GetFeatureInfo` request per pixel, into a single-band float32 GeoTIFF, and resumes safely across runs. The default source is the City of Cape Town's public server. macOS is the primary platform; the headless build also targets Linux.

Design and plan: `docs/superpowers/specs/2026-10-04-cli-refactor-design.md` and `docs/superpowers/plans/2026-10-04-cli-refactor.md`.

## Build, test, run

CMake 3.27+ with Ninja. The first configure downloads the pinned dependencies and compiles Catch2, so allow a few minutes.

```bash
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B build
```

```bash
cmake --build build
```

```bash
ctest --test-dir build --output-on-failure
```

One unit's tests by Catch2 tag, for example `./build/demdl_tests "[downloader]"`. One case by name, for example `./build/demdl_tests "a stop request saves the partial row and goes no further"`.

Options: `-DDL_ELEVATION_PREVIEW=OFF` drops the GLFW window and every OpenGL dependency (default ON on macOS, OFF elsewhere). `-DDL_ELEVATION_TESTS=OFF` skips Catch2.

Homebrew packages: `pkg-config libtiff libgeotiff openssl@3`, plus `glfw` for the preview (Linux also needs `glew`). Homebrew's `cpp-httplib` is used when it is 0.46.1 or a later 0.46 release; otherwise FetchContent downloads the pinned version.

Running the tool makes real requests to a public server. Check a command line with `--dry-run`, and never start a live download just to see whether the build works:

```bash
./build/dl_elevation --north -34.1845418 --south -34.2163447 --west 18.4185791 --east 18.4625244 --width 2048 --height 1792 --output simonstown.tif --dry-run
```

The end-to-end tests and the loopback WMS script in the plan's Task 13 Step 9 run the whole program against 127.0.0.1 and need no network. Live requests to the City of Cape Town server are for the owner to start.

`./build/dl_elevation --help` lists every flag. Headless is the default; `--preview` opens the window.

## Layout

- `src/demdl/` is the static library holding all logic: `BoundingBox`; `Config` (every option plus `validate()`); `WmsQuery` (URL building and body parsing, pure functions); `RetryPolicy`; `DemRaster` (float32 grid, GeoTIFF I/O, resume checks); `PixelFetcher` (one-method interface); `ElevationClient` (httplib); `Downloader` (row loop, pool, checkpoints, stop handling); `ProgressSink` with `ConsoleReporter`; `StopToken` (the shared stop flag, which keeps the first reason given).
- `src/app/` is the executable: `Cli.cpp` turns argv into a `Config` with CLI11; `ExitCodes.h` maps a finished run to its exit code; `main.cpp` wires the components and handles signals.
- `src/preview/` holds the optional GLFW/OpenGL sink, `PreviewWindow`, and its `Shader.h`. `Palette.h` there holds the preview's colours and includes no OpenGL header, so its tests build in every configuration.
- `tests/` has one Catch2 file per unit, the doubles `FakeFetcher.h` and `RecordingSink.h`, and the helpers `TempDir.h` and `ExampleConfig.h`. Nothing touches the network except an in-process `httplib::Server` on loopback.
- `tests/test_EndToEnd.cpp` spawns the built `dl_elevation` against a loopback WMS, so the test target depends on the executable.
- Test names must not begin with `-`, because ctest passes each name to Catch2 as an argument.

## Behaviours to keep in mind

- **The raster is the resume state.** NaN means not fetched, or failed last time and due for retry. `-32768`, the declared NoData, means the server has no elevation there and is final. On start an existing output must match width, height, bounding box (within 1e-9 degrees) and layer, otherwise exit 1 with the differences; `--overwrite` starts fresh. Files from the old RGB16 format are rejected.
- **Checkpoints are atomic.** Before anything is written, the output directory is checked and the sink is created. The checkpoint is saved before the first request and after every row with work. Each save writes the whole raster to `<output>.tmp` and renames it over the output.
- **Stopping is one flag.** SIGINT, SIGTERM, closing the preview, a checkpoint write failure, or the consecutive-failure limit set the `StopToken`. Queued pixels are abandoned, in-flight ones finish, and the partial row is saved. `main` prints `stopping: ...` to stderr.
- **Exit codes:** 0 complete; 1 usage, configuration, file or internal error; 2 stopped early and resumable; 3 finished with failed pixels and resumable. A second Ctrl-C (or SIGTERM) kills the process by that signal's default action, so the shell reports 130 (143 for SIGTERM) and a calling script stops.
- **Failures:** with the default policy and 8 workers, an unreachable server or a wrong path takes about 20 minutes to reach the 100-failure limit (longer when each connection attempt times out). Certificate errors are not retried, so they fail at once. If the system trust store rejects the server's chain, `ca-bundle.crt` in the repository root, the CA bundle the old program used, is a candidate for `--ca-bundle`. The first failed request and the first unparseable body are each reported once per run on stderr. Progress lines go to stdout.
- **Threading:** worker threads write distinct raster cells and own one `ElevationClient` each; sinks guard their own state; nothing prints from a signal handler.
- The old single-file `main.cpp` and the `include/` LearnOpenGL boilerplate are gone. Do not resurrect them.

## CMake notes

- Link `httplib::httplib`, never bare `httplib`. Only the namespaced target exists when Homebrew's package is found, and the bare name silently drops `CPPHTTPLIB_OPENSSL_SUPPORT`.
- `CMAKE_FIND_FRAMEWORK` is `LAST` because `/Library/Frameworks/Mono.framework/Headers` ships a libtiff 4.0.9 `tiff.h` that otherwise wins over Homebrew's.
- Apple clang searches `/usr/local/include` ahead of every `-isystem` directory, so a Homebrew header shadows a same-named header from a FetchContent target. Harmless today, worth knowing when bumping versions.
- Policy CMP0156 is set to NEW so that `libdemdl.a` is linked once when a target names both `demdl` and `demdl_cli`, which links `demdl` itself. Without it, the Xcode 15 linker warns about a duplicate library. The block is guarded by `if(POLICY CMP0156)` because the minimum version, CMake 3.27, predates the policy, which arrived in 3.29.
- Dependencies are pinned: cpp-httplib v0.46.1 (matches the Homebrew package), ctpl 437e135, CLI11 v2.7.2, Catch2 v3.16.0. Bump cpp-httplib together with the Homebrew package.

## Repository hygiene

`*.tif` and `*.tif.tmp` are ignored. The 20 to 40 MB files in the repository root are download outputs, not sources. Never stage with `git add -A`.
