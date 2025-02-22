cmake_minimum_required(VERSION 3.20)

include(${CMAKE_CURRENT_LIST_DIR}/modules.cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

project(pisar_thirdparty CXX)

# Set local install directory
set(PISAR_THIRDPARTY_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/install/${CMAKE_BUILD_TYPE}")
file(MAKE_DIRECTORY ${PISAR_THIRDPARTY_INSTALL_DIR})

# Build & install dependencies
foreach(module_name IN LISTS PISAR_THIRDPARTY_MODULES)
    if (${pisar_build_${module_name}})
        message(STATUS "Building thirdparty module ${module_name}")
        add_subdirectory(${module_name})
    else()
        message(STATUS "Skipping thirdparty module ${module_name}")
    endif()
endforeach()

# Install all files to ${CMAKE_INSTALL_PREFIX}
install(DIRECTORY "${PISAR_THIRDPARTY_INSTALL_DIR}" DESTINATION "${CMAKE_INSTALL_PREFIX}")
