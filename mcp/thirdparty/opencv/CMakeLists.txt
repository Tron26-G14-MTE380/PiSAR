cmake_minimum_required(VERSION 3.20)
project(OpenCV_Local)

include(ExternalProject)

ExternalProject_Add(
    opencv
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opencv  # Use the actual submodule
    CMAKE_ARGS
        -DCMAKE_INSTALL_PREFIX=${THIRD_PARTY_INSTALL_DIR}
        -DCMAKE_BUILD_TYPE=Release
        -DCMAKE_CXX_STANDARD=20
        -DBUILD_SHARED_LIBS=OFF
        -DBUILD_EXAMPLES=OFF
        -DBUILD_TESTS=OFF
        -DBUILD_DOCS=OFF
        -DBUILD_PERF_TESTS=OFF
        -DOPENCV_WORLD=OFF
        -DBUILD_LIST=core,imgproc,features2d,video,videoio,calib3d,highgui,imgcodecs,flann
        -DWITH_GTK=ON
)