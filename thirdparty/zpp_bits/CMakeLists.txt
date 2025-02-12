cmake_minimum_required(VERSION 3.30)

set(LIBRARY_NAME zpp_bits)
add_library(${LIBRARY_NAME} INTERFACE)

target_include_directories(${LIBRARY_NAME} INTERFACE ${CMAKE_CURRENT_LIST_DIR}/zpp_bits)

install(TARGETS ${LIBRARY_NAME}
        EXPORT ${LIBRARY_NAME}Targets
        INCLUDES DESTINATION include)
