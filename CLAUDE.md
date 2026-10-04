# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single-executable C++20 scraper that pulls a Digital Elevation Model (DEM) off the City of Cape Town's public WMS server one pixel at a time (one `GetFeatureInfo` request per pixel), accumulates the heights into a 16-bit RGB TIFF on disk, and shows live progress in an OpenGL window. All logic is in `main.cpp`. macOS/Homebrew only in practice.

## Build and run

CMake + Ninja. CMake project `DownloadElevation`, single target `dl_elevation`. No tests, no linter.

```bash
cmake -G Ninja -DCMAKE_BUILD_TYPE=Debug -S . -B cmake-build-debug
```

```bash
cmake --build cmake-build-debug --target dl_elevation
```

The build directory must sit exactly one level below the repo root, and the binary must be run from inside it: the input/output TIFF and the CA bundle are opened via `../simonstown.tif` and `../ca-bundle.crt`.

```bash
cd cmake-build-debug && ./dl_elevation
```

Running the binary immediately opens 50 concurrent HTTPS connections to `cityimg.capetown.gov.za` and rewrites `../simonstown.tif` after every row. Do not launch it just to confirm that a build works.

Homebrew packages needed: `pkg-config glfw glew glm assimp libtiff libgeotiff openssl@3`. `ctpl` is always fetched by `FetchContent`; `cpp-httplib` is taken from Homebrew's `cpp-httplib` package when installed and fetched otherwise, so the first configure needs network. assimp, GLEW and libgeotiff are demanded by the CMake files but referenced by no code; in particular the output is a plain RGB TIFF with no georeferencing tags.

No paths are hardcoded: libtiff comes from CMake's bundled `FindTIFF`, assimp from its own config, glfw and GLEW from pkg-config, OpenSSL transitively through `httplib::httplib`, and GLM and libgeotiff from `find_path`/`find_library`. Do not reintroduce `/usr/local/Cellar/<formula>/<version>` paths; they go stale on the next `brew upgrade`. Note that Apple clang puts `/usr/local/include` ahead of every `-isystem` directory, so a Homebrew header shadows a same-named header from an imported or FetchContent target. On a machine with the Homebrew `cpp-httplib` package, that package's `httplib.h` is what gets compiled even when FetchContent downloaded a copy.

Three things in the CMake files look odd but are deliberate:

- `dl_elevation` links `httplib::httplib`, not `httplib`. Only the namespaced name exists when the Homebrew package is found; the bare name silently degrades to `-lhttplib`, loses `CPPHTTPLIB_OPENSSL_SUPPORT`, and the compile fails with "no type named 'SSLClient'".
- GLM is located with `find_path`, not `find_package`. Homebrew GLM's config file declares a CMake minimum of 3.2, which CMake 4.x refuses to load.
- `CMAKE_FIND_FRAMEWORK` is set to `LAST`. With the macOS default of `FIRST`, CMake globs every `*.framework/Headers` before looking in `/usr/local`, and `/Library/Frameworks/Mono.framework/Headers` ships a libtiff 4.0.9 `tiff.h` that would otherwise be chosen over Homebrew's 4.7.1 headers.

## How main.cpp works

Global state plus free functions; read it top to bottom.

**All configuration is hardcoded.** Bounding box (`north/east/south/west` in `main()`, EPSG:4326), grid size (`gridSizeWidth`/`gridSizeHeight`, default 2048x1792), WMS layer name (`Elevation_CCT_GLM2019_5m_DEM`), host, elevation scale (`700.0` m maps to 65535), and the output filename, which appears twice (the `load_tiff` and `save_tiff` calls). Retargeting means editing all of these together.

**The output TIFF is the checkpoint.** On startup `load_tiff` reads the existing file, then `main()` scans for the last row containing any nonzero sample and resumes downloading from the row after it. If the file is absent, a zeroed raster is allocated and work starts at row 0. To restart from scratch, delete or rename the TIFF. `load_tiff` takes `gridSizeWidth`/`gridSizeHeight` by reference and overwrites them with the file's dimensions, so a grid-size change in code is silently ignored while an old TIFF exists.

**Download pipeline.** One `downloadThread` walks rows from `startY`. Per row it pushes one job per column onto a 50-thread `ctpl::thread_pool`, waits on all of that row's futures, then `save_tiff` rewrites the whole file. Each job constructs its own `httplib::SSLClient` (certificate verification disabled), issues a WMS `GetFeatureInfo` with `I=x&J=y`, retries up to 500 times until it gets HTTP 200, and parses the five characters following `"BAND 0:"` in the plain-text body as metres. Negative values clamp to 0.

**Pixel formats.** `raster` is 16 bits per sample, 3 samples per pixel, same scaled value in all three channels. `displayTexture` is a parallel RGB8 buffer that the main thread uploads to a GL texture every 500 ms. Its colours are a status map: blue = zero/no data, red = queued, green = request in flight, long-rainbow (`rainbow()`) = elevation.

**Threading.** `raster`, `displayTexture` and `maxDisplayValue` are written by pool threads and read by the main thread with no synchronisation. The sole mutex (`g_i_mutex`) only serialises `std::cout`. This works because each job touches a distinct pixel; don't add shared mutable state assuming it is protected.

**Rendering** is a fullscreen textured quad. The GLSL is embedded as raw string literals at the top of `main.cpp`. `include/shader.h` is the only header from `include/` in use; everything else there (mesh, model, animation, camera, the other shader variants, stb_image) is LearnOpenGL boilerplate carried over from another project and compiled by nothing.

## Repository hygiene

The `.tif` files in the repo root (`capetown.tif`, `simonstown.tif`, `new.tif` and copies) are 20-40 MB download outputs: untracked and not ignored. `.gitignore` only covers `cmake-build-debug*` and `output/new2.tif`. Never stage with `git add -A` or `git add .`.
