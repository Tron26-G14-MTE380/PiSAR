cmake_minimum_required(VERSION 3.20)
project(OpenCV_Local)

include(ExternalProject)

if(CMAKE_SYSTEM_NAME MATCHES "Linux")
    set(extra_cmake_args)
else()
    set(extra_cmake_args -DCMAKE_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX})
endif()



ExternalProject_Add(
    opencv
    SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/opencv  # Use the actual submodule
    CMAKE_ARGS
        ${extra_cmake_args}
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
        -DWITH_GSTREAMER=ON
        -DWITH_V4L=ON
        -DWITH_LIBV4L=ON
        -DWITH_OPENGL=OFF
        -DBUILD_opencv_videoio=ON
        -DWITH_OPENEXR=OFF
        -DWITH_FFMPEG=ON
    INSTALL_COMMAND cmake --install .
)
