cmake_minimum_required(VERSION 3.20)

project(pisar_core CXX)

find_package(OpenCV REQUIRED)
find_package (Eigen3 3.4 REQUIRED NO_MODULE)
find_package(easy_profiler REQUIRED)

add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../common common)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/../../driveunit_interface driveunit_interface)

# Detect if building on Raspberry Pi
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm" OR CMAKE_SYSTEM_NAME MATCHES "Linux")
    message(STATUS "Building for Raspberry Pi: Enabling wiringPi and libcamera")
    find_library(WIRINGPI_LIB wiringPi)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(Libcamera REQUIRED libcamera)
    pkg_check_modules(Pisp REQUIRED libpisp)
    set(PISAR_CORE_BUILD_RASPBERRY_PI TRUE)
else()
    message(STATUS "Not on Raspberry Pi: Skipping wiringPi and libcamera")
    set(PISAR_CORE_BUILD_RASPBERRY_PI FALSE)
endif()


add_library(pisar_core STATIC)

target_include_directories(pisar_core
    PUBLIC
    include
)

target_sources(pisar_core
    PUBLIC
    src/driveunit/async_driveunit_controller.cpp
    src/driveunit/driveunit_transport.cpp

    src/vision/thinning/thinning_classical.cpp
    src/vision/thinning/thinning_ma_ren.cpp
    src/vision/debug_visualization.cpp
    src/vision/path_extraction.cpp
    src/vision/utils.cpp
    src/vision/video_source.cpp
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
    target_include_directories(pisar_core PUBLIC ${Libcamera_INCLUDE_DIRS} ${Pisp_INCLUDE_DIRS})
    target_link_libraries(pisar_core PUBLIC ${WIRINGPI_LIB} ${Libcamera_LIBRARIES} ${Pisp_LIBRARIES})
    target_compile_options(pisar_core PUBLIC ${Libcamera_CFLAGS_OTHER} ${Pisp_CFLAGS_OTHER})
endif()

set_target_properties(pisar_core PROPERTIES
    CXX_STANDARD 20
    CXX_STANDARD_REQUIRED YES
)
