cmake_minimum_required(VERSION 3.20)
project(Eigen)

include(ExternalProject)

ExternalProject_Add(
    eigen
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/eigen  # Use the actual submodule
    CMAKE_ARGS 
        -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_INSTALL_DIR} 
        -DCMAKE_CXX_STANDARD=20
)
