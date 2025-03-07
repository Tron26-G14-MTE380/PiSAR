cmake_minimum_required(VERSION 3.20)
project(Eigen)

include(ExternalProject)

ExternalProject_Add(
    eigen
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen  # Use the actual submodule
    CMAKE_ARGS 
        -DCMAKE_CXX_STANDARD=20
    INSTALL_COMMAND cmake --install .
)
