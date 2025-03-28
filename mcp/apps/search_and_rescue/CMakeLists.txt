cmake_minimum_required(VERSION 3.20)

project(search_and_rescue CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

set(pisar_build_opencv ON)
set(pisar_build_eigen ON)
set(pisar_build_easy_profiler ON)
set(pisar_build_zpp_bits ON)
set(pisar_build_sml ON)

include(${CMAKE_CURRENT_LIST_DIR}/../../../thirdparty/import.cmake)

add_subdirectory(../../core pisar_core)

add_executable(${PROJECT_NAME})

target_sources(${PROJECT_NAME}
    PRIVATE
    main.cpp
)

target_link_libraries(${PROJECT_NAME}
    PRIVATE
    pisar_core
)

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "main")