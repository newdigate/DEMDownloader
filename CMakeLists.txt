cmake_minimum_required(VERSION 3.27)
project(DownloadElevation)
set(CMAKE_CXX_STANDARD 20)


include(FetchContent)
FetchContent_Declare(
        httplib
        GIT_REPOSITORY https://github.com/yhirose/cpp-httplib.git
        GIT_TAG        master
        FIND_PACKAGE_ARGS NAMES httplib
)
FetchContent_Declare(
        ctpl
        GIT_REPOSITORY https://github.com/vit-vit/ctpl.git
        GIT_TAG        master
        FIND_PACKAGE_ARGS NAMES ctpl
)

# After the following call, the CMake targets defined by googletest and
# Catch2 will be available to the rest of the build
FetchContent_MakeAvailable(httplib)
FetchContent_MakeAvailable(ctpl)
include_directories(${ctpl_SOURCE_DIR})
# macOS CMake searches frameworks before /usr/local by default and globs every
# *.framework/Headers, so /Library/Frameworks/Mono.framework/Headers (libtiff 4.0.9)
# would win over Homebrew's tiff.h. Search frameworks last.
set(CMAKE_FIND_FRAMEWORK LAST)

# libtiff via CMake's bundled FindTIFF. libgeotiff ships no CMake package, so locate it directly.
find_package(TIFF REQUIRED)
find_path(GEOTIFF_INCLUDE_DIR geotiff.h REQUIRED)
find_library(GEOTIFF_LIBRARY geotiff REQUIRED)
include_directories(${GEOTIFF_INCLUDE_DIR})
add_executable(dl_elevation main.cpp)


include(opengl.cmake.in)
target_link_libraries(dl_elevation ${LIBS})
target_link_libraries(dl_elevation ${GEOTIFF_LIBRARY} TIFF::TIFF)
# OpenSSL is not linked here: httplib::httplib carries OpenSSL::SSL and OpenSSL::Crypto on both paths.
# httplib::httplib exists on both paths: the Homebrew package only exports the
# namespaced name, and the FetchContent build adds it as an alias. The bare
# "httplib" name is not a target when the system package is found, so linking it
# silently drops CPPHTTPLIB_OPENSSL_SUPPORT and SSLClient disappears.
target_link_libraries(dl_elevation httplib::httplib)

if  (APPLE)
    target_link_libraries(dl_elevation
            "-framework CoreServices"
            "-framework CoreAudio"
            "-framework Foundation"
            "-framework AudioUnit")
endif()