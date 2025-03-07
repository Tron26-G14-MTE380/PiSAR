cmake_minimum_required(VERSION 3.20)
project(EasyProfiler)

include(ExternalProject)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(extra_cmake_args)
else()
    set(extra_cmake_args -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})
endif()


ExternalProject_Add(
    easy_profiler
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/easy_profiler  # Use the actual submodule
    CMAKE_ARGS
        ${extra_cmake_args}
        -DCMAKE_CXX_STANDARD=20
        -DBUILD_SHARED_LIBS=OFF
    INSTALL_COMMAND cmake --install .
)
