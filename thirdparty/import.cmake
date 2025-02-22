cmake_minimum_required(VERSION 3.20)

include(${CMAKE_CURRENT_LIST_DIR}/modules.cmake)

message(STATUS "Configuring mcp thirdparty libraries at ${CMAKE_CURRENT_LIST_DIR}.")

set(PISAR_THIRDPARTY_COPYOVER_CMAKE_FLAGS CMAKE_BUILD_TYPE)
foreach(module_name IN LISTS PISAR_THIRDPARTY_MODULES)
    list(APPEND PISAR_THIRDPARTY_COPYOVER_CMAKE_FLAGS pisar_build_${module_name})
endforeach()

# Create the command-line argument list
set(PISAR_THIRDPARTY_CMAKE_COMMAND_LINE_FLAGS)

# Iterate over key
foreach(flag_name IN LISTS PISAR_THIRDPARTY_COPYOVER_CMAKE_FLAGS)
    list(APPEND PISAR_THIRDPARTY_CMAKE_COMMAND_LINE_FLAGS "-D${flag_name}=${${flag_name}}")
endforeach()

set(PISAR_THIRDPARTY_DIR ${CMAKE_CURRENT_LIST_DIR})
set(PISAR_THIRDPARTY_BUILD_DIR ${PISAR_THIRDPARTY_DIR}/build/${CMAKE_BUILD_TYPE})
set(PISAR_THIRDPARTY_INSTALL_DIR ${PISAR_THIRDPARTY_DIR}/install/${CMAKE_BUILD_TYPE})

if (${pisar_thirdparty_local_build})
    set(PISAR_THIRDPARTY_BUILD_DIR ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/build/${CMAKE_BUILD_TYPE})
    set(PISAR_THIRDPARTY_INSTALL_DIR ${CMAKE_CURRENT_BINARY_DIR}/thirdparty/install/${CMAKE_BUILD_TYPE})
endif()

# Run cmake configure step
execute_process(
    COMMAND ${CMAKE_COMMAND} ${CMAKE_CURRENT_LIST_DIR} -B "${PISAR_THIRDPARTY_BUILD_DIR}" -G "${CMAKE_GENERATOR}" ${PISAR_THIRDPARTY_CMAKE_COMMAND_LINE_FLAGS}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    RESULT_VARIABLE CONFIGURE_RESULT
)

if(CONFIGURE_RESULT)
    message(FATAL_ERROR "CMake configuration failed for mcp thirdparty libraries!")
endif()

include(ProcessorCount)
ProcessorCount(CPU_COUNT)

message(STATUS "Building mcp thirdparty libraries at ${CMAKE_CURRENT_LIST_DIR}. Using ${CPU_COUNT} threads.")

# Run cmake build step
execute_process(
    COMMAND ${CMAKE_COMMAND} --build build/${CMAKE_BUILD_TYPE} -- -j ${CPU_COUNT}
    WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
    RESULT_VARIABLE BUILD_RESULT
)

if(BUILD_RESULT)
    message(FATAL_ERROR "CMake build failed for mcp thirdparty libraries!")
endif()

foreach(module_name IN LISTS PISAR_THIRDPARTY_MODULES)
    include(${PISAR_THIRDPARTY_DIR}/${module_name}/import.cmake)
endforeach()
