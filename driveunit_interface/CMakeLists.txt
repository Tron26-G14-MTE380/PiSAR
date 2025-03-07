cmake_minimum_required(VERSION 3.20)

find_package(zpp_bits REQUIRED)

set(LIBRARY_NAME pisar_driveunit_interface)
add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

target_link_libraries(${LIBRARY_NAME}
    INTERFACE
    zpp_bits::zpp_bits
)

install(TARGETS ${LIBRARY_NAME}
    EXPORT ${LIBRARY_NAME}Targets
    INCLUDES DESTINATION include)
