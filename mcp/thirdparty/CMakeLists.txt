cmake_minimum_required(VERSION 3.20)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED YES)

project(ThirdParty CXX)

# Set local install directory
set(THIRD_PARTY_INSTALL_DIR "${CMAKE_CURRENT_SOURCE_DIR}/install")
file(MAKE_DIRECTORY ${THIRD_PARTY_INSTALL_DIR})

# Build & install dependencies
add_subdirectory(eigen)
add_subdirectory(easy_profiler)
add_subdirectory(opencv)

# Install all files to ${CMAKE_INSTALL_PREFIX}
install(DIRECTORY "${THIRD_PARTY_INSTALL_DIR}/" DESTINATION "${CMAKE_INSTALL_PREFIX}")
