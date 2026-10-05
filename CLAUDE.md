# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

`dl_elevation` scrapes a Digital Elevation Model (DEM) from a WMS server one pixel at a time, one `GetFeatureInfo` request per pixel, into a single-band float32 GeoTIFF, and resumes safely across runs. The default source is the City of Cape Town's public server. macOS is the primary platform. The headless build is meant to work on Linux as well, but that is untested: it has never been compiled there.

Design and plan: `docs/superpowers/specs/2026-10-04-cli-refactor-design.md` and `docs/superpowers/plans/2026-10-04-cli-refactor.md`.

## Build, test, run

CMake 3.27+ with Ninja. The first configure downloads the pinned dependencies. The first build also compiles Catch2, so allow a few minutes.

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

Homebrew packages: `libtiff libgeotiff openssl@3`, plus `pkg-config` and `glfw` for the preview (and `glew` on Linux, untested). Homebrew's `cpp-httplib` is used when it is 0.46.1 or a later 0.46 release; otherwise FetchContent downloads the pinned version.

Running the tool makes real requests to a public server. Check a command line with `--dry-run`, which writes nothing and prints the plan, including a `tls:` line saying how requests would be secured, and never start a live download just to see whether the build works:

```bash
./build/dl_elevation --north -34.1845418 --south -34.2163447 --west 18.4185791 --east 18.4625244 --width 2048 --height 1792 --output "${TMPDIR:-/tmp}/simonstown.tif" --dry-run
```

The `.tif` files in the repository root are old RGB16 downloads that the new tool refuses to resume; `--overwrite` on them would destroy them, so choose another `--output` for new runs.

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

- **The raster is the resume state.** NaN means not fetched, or failed last time and due for retry. `-32768`, the declared NoData, means the server's mask band says there is no elevation there (`BAND 1: 0`, as at sea, where `BAND 0` reads 0) and is final; a body without `BAND 0:` is a parse error, retried, never NoData. The mask rule applies to every `--layer`: one whose `BAND 1` is a data band rather than a 0/255 mask would have every pixel where that band reads exactly 0 stored as NoData, so check a captured response before using another layer. On start an existing output must match width, height, bounding box (within 1e-9 degrees) and layer, otherwise exit 1 with the differences; `--overwrite` starts fresh. Files from the old RGB16 format are rejected.
- **What the server returns.** Checked live on 2026-10-05: the server samples the top-left corner of each requested pixel (its `Point:` line says so), and at 2048 x 1792 over the Simon's Town box, about 2 m per pixel, its heights equal the earlier download's exactly. A 4 x 3 test grid over the same box returned different heights at the same corner points, so coarse requests appear to be answered from lower-resolution overviews: use fine grids for real data. The files declare `RasterPixelIsArea`, so GIS shows each height half a pixel south-east of where it was sampled, about 1 m on the default grid and below the model's 5 m resolution.
- **Checkpoints are atomic.** Before anything is written, the output directory is checked and the sink is created. The checkpoint is saved before the first request and after every row with work. Each save writes the whole raster to `<output>.tmp`, syncs it and renames it over the output, so every row with work rewrites 4 x width x height bytes: about 26 GB over a 2048 x 1792 run, and about 50 TB for a square grid near the 2^29-pixel limit, enough to wear an SSD noticeably. A save after the first request that fails, on a full disk for instance, stops the run with exit 2 and the last good checkpoint intact, and is reported on stderr as `error: checkpoint write failed: ...` (after another stop reason, `error: interrupted; checkpoint write failed: ...`) as well as in the summary on stdout. If the save before the first request fails, the run exits 1 with `error: ...` and makes no request.
- **Stopping is one flag.** SIGINT, SIGTERM, SIGHUP (closing the terminal; left ignored when the tool is started under `nohup`), closing the preview, a checkpoint write failure, or the consecutive-failure limit set the `StopToken`. Queued pixels are abandoned, in-flight ones finish, and the partial row is saved. `main` prints `stopping: ...` to stderr.
- **Exit codes:** 0 complete; 1 usage, configuration, file or internal error; 2 stopped early and resumable; 3 finished with failed pixels and resumable. A second Ctrl-C kills the process by SIGINT, which the shell reports as 130, and because Ctrl-C reaches the whole foreground job, a calling shell script stops too. A second SIGTERM kills it by SIGTERM, which the shell reports as 143, and a second SIGHUP by SIGHUP, 129.
- **Failures:** with the default policy and 8 workers, an unreachable server or a wrong path takes about 20 minutes to reach the 100-failure limit (longer when each connection attempt times out). Certificate errors are not retried, so they fail at once. `ca-bundle.crt` in the repository root is Mozilla's CA snapshot of 2019-01-22, root certificates only. The old program passed it to httplib but turned verification off, so it never verified anything. `--ca-bundle` replaces the system trust store entirely rather than adding to it, so that file cannot supply an intermediate certificate the server fails to send, and it breaks chains whose root is newer than 2019. If verification fails, build a PEM file holding the server's intermediate certificate(s) and its root and pass that with `--ca-bundle`; never use `--insecure` for a long run. The first failed request and the first unparseable body are each reported once per run on stderr. Progress lines go to stdout.
- **Threading:** worker threads write distinct raster cells and own one `ElevationClient` each; sinks guard their own state; nothing prints from a signal handler.
- The old single-file `main.cpp` and the `include/` LearnOpenGL boilerplate are gone. Do not resurrect them.

## CMake notes

- Link `httplib::httplib`, never bare `httplib`. Only the namespaced target exists when Homebrew's package is found, and the bare name silently drops `CPPHTTPLIB_OPENSSL_SUPPORT`.
- `CMAKE_FIND_FRAMEWORK` is `LAST` because `/Library/Frameworks/Mono.framework/Headers` ships a libtiff 4.0.9 `tiff.h` that otherwise wins over Homebrew's.
- Apple clang searches `/usr/local/include` ahead of every `-isystem` directory, so it compiles `/usr/local/include/httplib.h` even when FetchContent supplies the pinned copy. After `brew upgrade cpp-httplib` moves past 0.46, the build silently compiles the newer header. Pin the formula with `brew pin cpp-httplib`, or bump the pin in `CMakeLists.txt` to match.
- Policy CMP0156 is set to NEW so that `libdemdl.a` is linked once when a target names both `demdl` and `demdl_cli`, which links `demdl` itself. Without it, the Xcode 15 linker warns about a duplicate library. The block is guarded by `if(POLICY CMP0156)` because the minimum version, CMake 3.27, predates the policy, which arrived in 3.29.
- Dependencies are pinned: cpp-httplib v0.46.1 (matches the Homebrew package), ctpl 437e135, CLI11 v2.7.2, Catch2 v3.16.0. Bump cpp-httplib together with the Homebrew package.
- After `brew upgrade` of `openssl@3`, `glfw` or `libtiff`, an existing build tree may still point at Cellar paths that the upgrade removed: delete `build/` and configure again. Never hard-code `/usr/local/Cellar/<formula>/<version>` paths in `CMakeLists.txt`.

## Repository hygiene

`*.tif` and `*.tif.tmp` are ignored. The 20 to 40 MB files in the repository root are download outputs, not sources. Each is RGB16 with the same value in every channel and no georeferencing, and reads as metres = `value * S / 65535`: S is 1100 for `capetown.tif` and its two byte-identical copies (grid and box in `e2bea3e:main.cpp`) and 700 for `simonstown.tif` (`fc7dcef:main.cpp`). In those versions of `main.cpp` the variable `north` holds the southern edge's latitude and `south` the northern edge's. A 0 in these files is not only a negative height: it is also what the old program wrote for a response without `BAND 0:`, for a pixel whose 500 attempts all failed, and for a pixel it never fetched. `new.tif` (1980 x 1645) has no recorded scale or box. Never stage with `git add -A`.
