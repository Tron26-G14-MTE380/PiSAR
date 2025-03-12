cmake_minimum_required(VERSION 3.20)

include(GNUInstallDirs)

project(zpp_bits LANGUAGES CXX)

# Define the library target
add_library(zpp_bits INTERFACE)
target_include_directories(zpp_bits INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/zpp_bits>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(zpp_bits INTERFACE cxx_std_17)

# Exporting the package
include(CMakePackageConfigHelpers)

set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/cmake/zpp_bits)

# Install the library
install(TARGETS zpp_bits EXPORT zpp_bitsTargets)

# Install the headers
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/zpp_bits DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install the package configuration files
install(EXPORT zpp_bitsTargets
    FILE zpp_bitsTargets.cmake
    NAMESPACE zpp_bits::
    DESTINATION ${INSTALL_CONFIGDIR}
)

configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/zpp_bitsConfig.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/zpp_bitsConfig.cmake
    INSTALL_DESTINATION ${INSTALL_CONFIGDIR}
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/zpp_bitsConfig.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)

# Generate a package version file
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/zpp_bitsConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY AnyNewerVersion
)

install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/zpp_bitsConfigVersion.cmake
    DESTINATION ${INSTALL_CONFIGDIR}
)
