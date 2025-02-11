cmake_minimum_required(VERSION 3.30)

set(LIBRARY_NAME pisar_driveunit_interface)
add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/include)

install(TARGETS ${LIBRARY_NAME}
        EXPORT ${LIBRARY_NAME}Targets
        INCLUDES DESTINATION include)
