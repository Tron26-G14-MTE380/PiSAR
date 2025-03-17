cmake_minimum_required(VERSION 3.20)

if (UNIX)
    set(OpenCV_DIR ${PISAR_THIRDPARTY_INSTALL_DIR}/lib/cmake/opencv4)
else()
    set(OpenCV_DIR ${PISAR_THIRDPARTY_INSTALL_DIR})
endif()

set(BUILD_SHARED_LIBS OFF)

# Ensure OpenCV is found
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)

set(OpenCV_BIN_DIR ${PISAR_THIRDPARTY_INSTALL_DIR}/${OpenCV_ARCH}/${OpenCV_RUNTIME}/bin)

# Select the correct file extensions for the platform
if(WIN32)
    file(GLOB OpenCV_LIBS "${OpenCV_BIN_DIR}/*.dll")  # Windows: Copy only .dll files
elseif(UNIX)
    file(GLOB OpenCV_LIBS "${OpenCV_BIN_DIR}/*.so*")  # Linux: Copy only .so files
endif()

# Copy each DLL or shared object file individually
foreach(lib ${OpenCV_LIBS})
    file(COPY "${lib}" DESTINATION "${CMAKE_BINARY_DIR}")
endforeach()

# Create a target that depends on copying all OpenCV libraries
add_custom_target(copy_opencv_libs ALL
    DEPENDS "${OpenCV_LIBS}"
    COMMENT "Ensuring all OpenCV shared libraries are copied"
)

