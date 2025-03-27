cmake_minimum_required(VERSION 3.20)
project(Eigen)

include(ExternalProject)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(extra_cmake_args -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})
else()
    set(extra_cmake_args -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})
endif()



ExternalProject_Add(
    sml
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/sml  # Use the actual submodule
    CMAKE_ARGS
        ${extra_cmake_args}
        -DCMAKE_CXX_STANDARD=20
    INSTALL_COMMAND cmake --install .
)
