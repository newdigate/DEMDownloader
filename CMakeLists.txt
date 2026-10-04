cmake_minimum_required(VERSION 3.27)
project(DownloadElevation VERSION 0.2.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# macOS CMake searches frameworks before /usr/local by default and globs every
# *.framework/Headers, so /Library/Frameworks/Mono.framework/Headers (libtiff 4.0.9)
# would win over Homebrew's tiff.h. Search frameworks last.
set(CMAKE_FIND_FRAMEWORK LAST)

if(APPLE)
  set(_dl_preview_default ON)
else()
  set(_dl_preview_default OFF)
endif()
option(DL_ELEVATION_PREVIEW "Build the OpenGL preview window" ${_dl_preview_default})
option(DL_ELEVATION_TESTS "Build the Catch2 test suite" ON)

include(FetchContent)
# cpp-httplib is pinned to the Homebrew version in use so both dependency paths compile identical
# code. FIND_PACKAGE_ARGS prefers the system package when it is 0.46.x, otherwise the pinned copy is downloaded.
FetchContent_Declare(httplib
  GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
  GIT_TAG        v0.46.1
  FIND_PACKAGE_ARGS 0.46.1 NAMES httplib)
# ctpl's last commit (2015-06-12); its two upstream tags both point here, so pin the hash.
FetchContent_Declare(ctpl
  GIT_REPOSITORY https://github.com/vit-vit/ctpl.git
  GIT_TAG        437e135dbd94eb65b45533d9ce8ee28b5bd37b6d)
FetchContent_Declare(CLI11
  GIT_REPOSITORY https://github.com/CLIUtils/CLI11.git
  GIT_TAG        v2.7.2)
FetchContent_Declare(Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        v3.16.0)
# For the downloaded cpp-httplib only: match the Homebrew package's feature set (OpenSSL and
# zlib, no Brotli or zstd) and fail at configure time instead of silently losing HTTPS.
set(HTTPLIB_REQUIRE_OPENSSL ON)
set(HTTPLIB_USE_BROTLI_IF_AVAILABLE OFF)
set(HTTPLIB_USE_ZSTD_IF_AVAILABLE OFF)
set(HTTPLIB_INSTALL OFF)
FetchContent_MakeAvailable(httplib ctpl CLI11)
if(DL_ELEVATION_TESTS)
  FetchContent_MakeAvailable(Catch2)
endif()

# libtiff via CMake's bundled FindTIFF. libgeotiff ships no CMake package, so locate it directly.
find_package(TIFF REQUIRED)
find_path(GEOTIFF_INCLUDE_DIR geotiff.h PATH_SUFFIXES geotiff libgeotiff REQUIRED)
find_library(GEOTIFF_LIBRARY geotiff REQUIRED)

add_library(demdl STATIC
  src/demdl/BoundingBox.cpp
  src/demdl/Config.cpp
  src/demdl/WmsQuery.cpp)
target_include_directories(demdl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${GEOTIFF_INCLUDE_DIR})
target_include_directories(demdl SYSTEM PRIVATE ${ctpl_SOURCE_DIR})
# httplib::httplib exists on both paths: the Homebrew package only exports the
# namespaced name, and the FetchContent build adds it as an alias. The bare
# "httplib" name is not a target when the system package is found, so linking it
# silently drops CPPHTTPLIB_OPENSSL_SUPPORT and SSLClient disappears.
target_link_libraries(demdl PUBLIC httplib::httplib TIFF::TIFF ${GEOTIFF_LIBRARY})
target_compile_options(demdl PRIVATE -Wall -Wextra)
target_compile_definitions(demdl PUBLIC DL_ELEVATION_VERSION="${PROJECT_VERSION}")

if(DL_ELEVATION_TESTS)
  enable_testing()
  list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
  include(Catch)
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp)
  target_link_libraries(demdl_tests PRIVATE demdl Catch2::Catch2WithMain)
  target_compile_options(demdl_tests PRIVATE -Wall -Wextra)
  catch_discover_tests(demdl_tests)
endif()
