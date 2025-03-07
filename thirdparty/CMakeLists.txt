cmake_minimum_required(VERSION 3.20)

include(${CMAKE_CURRENT_LIST_DIR}/modules.cmake)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

project(pisar_thirdparty CXX)

# Build & install dependencies
foreach(module_name IN LISTS PISAR_THIRDPARTY_MODULES)
    if (${pisar_build_${module_name}})
        message(STATUS "Building thirdparty module ${module_name}")
        add_subdirectory(${module_name})
    else()
        message(STATUS "Skipping thirdparty module ${module_name}")
    endif()
endforeach()

