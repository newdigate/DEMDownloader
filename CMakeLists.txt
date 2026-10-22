cmake_minimum_required(VERSION 3.27)
# CMake 3.29+ de-duplicates static libraries on the link line (CMP0156); without it, naming demdl
# directly and again through demdl_cli's PUBLIC link makes the Xcode 15 linker warn about a duplicate.
if(POLICY CMP0156)
  cmake_policy(SET CMP0156 NEW)
endif()
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
# code. FIND_PACKAGE_ARGS prefers the system package when it is 0.46.1 or a later 0.46 release,
# otherwise the pinned copy is downloaded.
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
  src/demdl/WmsQuery.cpp
  src/demdl/RetryPolicy.cpp
  src/demdl/TiffSetup.cpp
  src/demdl/DemRaster.cpp
  src/demdl/Downloader.cpp
  src/demdl/ElevationClient.cpp
  src/demdl/ConsoleReporter.cpp)
target_include_directories(demdl PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/src ${GEOTIFF_INCLUDE_DIR})
target_include_directories(demdl SYSTEM PRIVATE ${ctpl_SOURCE_DIR})
# httplib::httplib exists on both paths: the Homebrew package only exports the
# namespaced name, and the FetchContent build adds it as an alias. The bare
# "httplib" name is not a target when the system package is found, so linking it
# silently drops CPPHTTPLIB_OPENSSL_SUPPORT and SSLClient disappears.
target_link_libraries(demdl PUBLIC httplib::httplib TIFF::TIFF ${GEOTIFF_LIBRARY})
target_compile_options(demdl PRIVATE -Wall -Wextra)
target_compile_definitions(demdl PUBLIC DL_ELEVATION_VERSION="${PROJECT_VERSION}")

# Cli.cpp is compiled once and shared by the executable and the tests; only it includes CLI11.
add_library(demdl_cli OBJECT src/app/Cli.cpp)
target_link_libraries(demdl_cli PUBLIC demdl PRIVATE CLI11::CLI11)
target_compile_options(demdl_cli PRIVATE -Wall -Wextra)

add_executable(dl_elevation src/app/main.cpp)
target_link_libraries(dl_elevation PRIVATE demdl demdl_cli)
target_compile_options(dl_elevation PRIVATE -Wall -Wextra)

if(DL_ELEVATION_PREVIEW)
  find_package(PkgConfig REQUIRED)
  pkg_search_module(glfw REQUIRED IMPORTED_TARGET glfw3)
  add_library(demdl_preview STATIC src/preview/PreviewWindow.cpp)
  # PreviewWindow.h includes no GLFW or OpenGL header, so those stay PRIVATE; a static library still hands
  # them on to whatever links it, for linking only.
  target_link_libraries(demdl_preview PUBLIC demdl PRIVATE PkgConfig::glfw)
  target_compile_definitions(demdl_preview PRIVATE GL_SILENCE_DEPRECATION)
  target_compile_options(demdl_preview PRIVATE -Wall -Wextra)
  if(APPLE)
    target_link_libraries(demdl_preview PRIVATE
      "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
  else()
    find_package(OpenGL REQUIRED)
    pkg_search_module(glew REQUIRED IMPORTED_TARGET glew)
    target_link_libraries(demdl_preview PRIVATE OpenGL::GL PkgConfig::glew)
  endif()
  target_link_libraries(dl_elevation PRIVATE demdl_preview)
  target_compile_definitions(dl_elevation PRIVATE DL_ELEVATION_HAVE_PREVIEW)
endif()

# The units behind dem2mesh (GeoTIFF to printable solid). They build on demdl, whose PUBLIC include path puts src/ in
# reach, so headers are included as "mesh/<Name>.h".
add_library(demmesh STATIC
  src/mesh/TransverseMercator.cpp
  src/mesh/GeoTiffGrid.cpp
  src/mesh/CropWindow.cpp
  src/mesh/NoDataFill.cpp
  src/mesh/Resampler.cpp
  src/mesh/Predicates.cpp
  src/mesh/PolygonTriangulation.cpp
  src/mesh/Footprint.cpp
  src/mesh/ProfileSimplifier.cpp
  src/mesh/Simplifier.cpp
  src/mesh/Tiles.cpp
  src/mesh/BasePlane.cpp
  src/mesh/SolidMesh.cpp
  src/mesh/FootprintSolid.cpp
  src/mesh/AtomicFileWriter.cpp
  src/mesh/StlWriter.cpp
  src/mesh/PlyWriter.cpp
  src/mesh/BlenderRunner.cpp)
target_link_libraries(demmesh PUBLIC demdl)
target_compile_options(demmesh PRIVATE -Wall -Wextra)
# The simplifier's choices hang on comparing errors worked out in double: Simplifier.cpp picks the sample to insert next
# and ProfileSimplifier.cpp the height to split a side at. Both are built without fused multiply-adds, which compilers
# form by default where the target has them (arm64 always does) and which round differently: the same grid then gives
# the same TIN on every platform (spec 4.3).
set_source_files_properties(src/mesh/Simplifier.cpp src/mesh/ProfileSimplifier.cpp PROPERTIES COMPILE_OPTIONS
                            -ffp-contract=off)
# The Blender scene script (spec 7.1) is compiled into demmesh as the raw string kBlenderScenePy, so that dem2mesh needs
# no file beside it: it writes the script out for Blender at run time. The configure step reads it, and runs again when
# it changes. Its text must not hold the raw string's closing delimiter, which would end the string early.
file(READ src/mesh/blender_scene.py BLENDER_SCENE_PY)
string(FIND "${BLENDER_SCENE_PY}" ")PYSRC\"" _blender_scene_delimiter)
if(NOT _blender_scene_delimiter EQUAL -1)
  message(FATAL_ERROR "src/mesh/blender_scene.py contains )PYSRC\", which would end the C++ raw string it is embedded in")
endif()
configure_file(src/mesh/BlenderScenePy.h.in ${CMAKE_BINARY_DIR}/generated/mesh/BlenderScenePy.h @ONLY)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS src/mesh/blender_scene.py)
target_include_directories(demmesh PRIVATE ${CMAKE_BINARY_DIR}/generated)

# MeshCli.cpp is compiled once and shared by the dem2mesh executable and the tests; only it includes CLI11.
add_library(demmesh_cli OBJECT src/app/MeshCli.cpp)
target_link_libraries(demmesh_cli PUBLIC demmesh PRIVATE CLI11::CLI11)
target_compile_options(demmesh_cli PRIVATE -Wall -Wextra)

add_executable(dem2mesh src/app/dem2mesh_main.cpp)
target_link_libraries(dem2mesh PRIVATE demmesh demmesh_cli)
target_compile_options(dem2mesh PRIVATE -Wall -Wextra)

if(DL_ELEVATION_TESTS)
  enable_testing()
  list(APPEND CMAKE_MODULE_PATH "${catch2_SOURCE_DIR}/extras")
  include(Catch)
  add_executable(demdl_tests
    tests/test_BoundingBox.cpp
    tests/test_Config.cpp
    tests/test_WmsQuery.cpp
    tests/test_RetryPolicy.cpp
    tests/test_DemRaster.cpp
    tests/test_StopToken.cpp
    tests/test_Downloader.cpp
    tests/test_ElevationClient.cpp
    tests/test_ConsoleReporter.cpp
    tests/test_Cli.cpp
    tests/test_CliChecks.cpp
    tests/test_ExitCodes.cpp
    tests/test_EndToEnd.cpp
    tests/test_Palette.cpp
    tests/test_TransverseMercator.cpp
    tests/test_GeoTiffGrid.cpp
    tests/test_CropWindow.cpp
    tests/test_NoDataFill.cpp
    tests/test_Resampler.cpp
    tests/test_Predicates.cpp
    tests/test_PolygonTriangulation.cpp
    tests/test_Footprint.cpp
    tests/test_ProfileSimplifier.cpp
    tests/test_Simplifier.cpp
    tests/test_Tiles.cpp
    tests/test_BasePlane.cpp
    tests/test_SolidMesh.cpp
    tests/test_FootprintSolid.cpp
    tests/test_MeshWriters.cpp
    tests/test_MeshCli.cpp
    tests/test_BlenderRunner.cpp
    tests/test_Dem2MeshEndToEnd.cpp)
  target_link_libraries(demdl_tests PRIVATE demdl demmesh demdl_cli demmesh_cli Catch2::Catch2WithMain)
  target_compile_options(demdl_tests PRIVATE -Wall -Wextra)
  # The runner's test compares what Blender is given with the embedded script itself.
  target_include_directories(demdl_tests PRIVATE ${CMAKE_BINARY_DIR}/generated)
  # The end-to-end tests run the executables themselves, so they are built first and their paths are compiled in.
  add_dependencies(demdl_tests dl_elevation dem2mesh)
  target_compile_definitions(demdl_tests PRIVATE DL_ELEVATION_EXE="$<TARGET_FILE:dl_elevation>"
                                                 DEM2MESH_EXE="$<TARGET_FILE:dem2mesh>")
  catch_discover_tests(demdl_tests)
endif()
