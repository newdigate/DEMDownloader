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
find_package(httplib)
find_package(ctpl)
include_directories(${ctpl_SOURCE_DIR})
include_directories(/usr/local/Cellar/libgeotiff/1.7.1_3/include)
include_directories(/usr/local/Cellar/libtiff/4.6.0/include/)
include_directories(/usr/local/include/)
include_directories(${HTTPLIB_HEADER_PATH})
add_executable(dl_elevation main.cpp)


include(opengl.cmake.in)
target_link_libraries(dl_elevation ${LIBS})
target_link_libraries(dl_elevation -L/usr/local/Cellar/libgeotiff/1.7.1_3/lib -lgeotiff)
target_link_libraries(dl_elevation -L/usr/local/Cellar/libtiff/4.6.0/lib -ltiff)
target_link_libraries(dl_elevation -L/usr/local/opt/openssl/lib -lssl -lcrypto)
target_link_libraries(dl_elevation httplib)

if  (APPLE)
    target_link_libraries(dl_elevation
            "-framework CoreServices"
            "-framework CoreAudio"
            "-framework Foundation"
            "-framework AudioUnit")
endif()