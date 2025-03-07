cmake_minimum_required(VERSION 3.20)
project(OpenCV_Local)

include(ExternalProject)

ExternalProject_Add(
    opencv
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opencv  # Use the actual submodule
    CMAKE_ARGS
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_CXX_STANDARD=20
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_TESTS=OFF
        -DBUILD_DOCS=OFF
        -DBUILD_PERF_TESTS=OFF
        -DBUILD_opencv_objdetect=OFF
        -DBUILD_opencv_ml=OFF
        -DBUILD_opencv_photo=OFF
        -DBUILD_opencv_stiching=OFF
        -DBUILD_JAVA=OFF
        -DBUILD_opencv_python2=OFF
        -DBUILD_opencv_python3=OFF
    INSTALL_COMMAND cmake --install .
)