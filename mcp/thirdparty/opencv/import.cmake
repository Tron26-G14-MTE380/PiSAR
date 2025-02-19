cmake_minimum_required(VERSION 3.20)

if (UNIX)
    set(OpenCV_DIR ${CMAKE_CURRENT_LIST_DIR}/../install/lib/cmake/opencv4)
else()
    set(OpenCV_DIR ${CMAKE_CURRENT_LIST_DIR}/../install/lib)
endif()

set(OpenCV_BIN_DIR ${CMAKE_CURRENT_LIST_DIR}/../install/${OpenCV_ARCH}/${OpenCV_RUNTIME})
set(BUILD_SHARED_LIBS OFF)
