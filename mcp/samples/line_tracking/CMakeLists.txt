cmake_minimum_required(VERSION 3.20)

project(pisar_line_tracking CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

#add_subdirectory(../../thirdparty thirdparty)
add_subdirectory(../../core pisar_core)

add_executable(pisar_line_tracking)

target_sources(pisar_line_tracking
    PRIVATE
    main.cpp
)

target_link_libraries(pisar_line_tracking
    PRIVATE
    pisar_core
)