cmake_minimum_required(VERSION 3.20)

include(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/import.cmake)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
find_package(easy_profiler REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

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
