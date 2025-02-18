cmake_minimum_required(VERSION 3.20)
project(EasyProfiler)

include(ExternalProject)

ExternalProject_Add(
    easy_profiler
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/easy_profiler  # Use the actual submodule
    CMAKE_ARGS 
        -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_INSTALL_DIR} 
        -DCMAKE_CXX_STANDARD=20
        -DBUILD_SHARED_LIBS=OFF
)
