cmake_minimum_required(VERSION 3.20)

include(${CMAKE_CURRENT_LIST_DIR}/../thirdparty/import.cmake)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
include_directories(${OpenCV_INCLUDE_DIRS})

add_library(pisar_core STATIC)

target_include_directories(pisar_core
    PUBLIC
    include
)

target_sources(pisar_core
    PUBLIC
    src/vision/thinning.cpp
    src/vision/line_tracker.cpp
)

target_link_libraries(pisar_core
    PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
)

set_target_properties(pisar_core PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)
