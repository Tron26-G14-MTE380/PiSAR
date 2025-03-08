cmake_minimum_required(VERSION 3.20)

project(pisar_core CXX)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package(easy_profiler REQUIRED)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../common common)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../driveunit_interface driveunit_interface)

# Detect if building on Raspberry Pi
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_NAME MATCHES "Linux")
    message(STATUS "Building for Raspberry Pi: Enabling wiringPi")
    find_library(WIRINGPI_LIB wiringPi)
    set(PISAR_CORE_BUILD_RASPBERRY_PI TRUE)
else()
    message(STATUS "Not on Raspberry Pi: Skipping wiringPi")
    set(PISAR_CORE_BUILD_RASPBERRY_PI FALSE)
endif()


add_library(pisar_core STATIC)

target_include_directories(pisar_core
    PUBLIC
    include
)

target_sources(pisar_core
    PUBLIC
    src/vision/thinning/thinning_classical.cpp
    src/vision/thinning/thinning_ma_ren.cpp
    src/vision/debug_visualization.cpp
    src/vision/path_extraction.cpp
    src/vision/path_simplification.cpp
    src/vision/utils.cpp
)

target_link_libraries(pisar_core
    PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        easy_profiler
        pisar_utilities
        pisar_driveunit_interface
)

# Link wiringPi only if on Raspberry Pi
if(PISAR_CORE_BUILD_RASPBERRY_PI)
    target_link_libraries(pisar_core PUBLIC ${WIRINGPI_LIB})
endif()

set_target_properties(pisar_core PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)
