cmake_minimum_required(VERSION 3.20)

if (UNIX)
    set(OpenCV_DIR ${PISAR_THIRDPARTY_INSTALL_DIR}/lib/cmake/opencv4)
else()
    set(OpenCV_DIR ${PISAR_THIRDPARTY_INSTALL_DIR})
endif()

set(OpenCV_BIN_DIR ${PISAR_THIRDPARTY_INSTALL_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME})
set(BUILD_SHARED_LIBS OFF)
