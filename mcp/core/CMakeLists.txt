cmake_minimum_required(VERSION 3.20)

project(pisar_core CXX)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package(easy_profiler REQUIRED)

add_library(pisar_core STATIC)

target_include_directories(pisar_core
    PUBLIC
    include
)

target_sources(pisar_core
    PUBLIC
    src/vision/thinning/thinning_classical.cpp
    src/vision/thinning/thinning_ma_ren.cpp
    src/vision/path_extraction.cpp
    src/vision/path_simplification.cpp
)

target_link_libraries(pisar_core
    PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    easy_profiler
)

set_target_properties(pisar_core PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)
