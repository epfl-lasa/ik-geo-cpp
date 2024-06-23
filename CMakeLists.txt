# Modified from https://gitlab.com/Fraser999/Rust-From-CXX/

project(ik_geo)
cmake_minimum_required(VERSION 3.0)

# Enable ExternalProject CMake module
include(ExternalProject)

#target_include_directories(ik_geo PUBLIC include)

# # Set default ExternalProject root directory
# set_directory_properties(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/Rust)

# Add rust_example as a CMake target
ExternalProject_Add(
    ik_geo_lib
    DOWNLOAD_COMMAND ""
    CONFIGURE_COMMAND ""
    BUILD_COMMAND cargo build COMMAND cargo build --release
    BINARY_DIR "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper"
    INSTALL_COMMAND ""
    LOG_BUILD ON)


# Specify link libraries

# if (WIN32)
#     target_link_libraries(ik_geo
#         debug "${CMAKE_SOURCE_DIR}/rust-wrapper/target/debug/libik_geo.lib"
#         optimized "${CMAKE_SOURCE_DIR}/rust-wrapper/target/release/libik_geo.lib"
#         ws2_32 userenv advapi32)
# else ()
#     target_link_libraries(ik_geo
#         debug "${CMAKE_SOURCE_DIR}/rust-wrapper/target/de bug/libik_geo.so"
#         optimized "${CMAKE_SOURCE_DIR}/rust-wrapper/target/release/libik_geo.so")
# endif()



#set_target_properties(ik_geo PROPERTIES CXX_STANDARD 11 CXX_STANDARD_REQUIRED ON)

# Build to the dist folder
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

# Custom command to copy libik_cpp.a to the dist folder
if (WIN32)
    add_custom_command(TARGET ik_geo_lib POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
            "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper/target/debug/libik_geo.lib"
            "${CMAKE_BINARY_DIR}/libik_geo.lib"
    )
else()
    add_custom_command(TARGET ik_geo_lib POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy
        "${CMAKE_CURRENT_LIST_DIR}/rust-wrapper/target/release/libik_geo.so"
        "${CMAKE_BINARY_DIR}/libik_geo.so"
    )
endif()


# Copy ik_geo.h to the dist folder
add_custom_command(TARGET ik_geo_lib POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy
    "${CMAKE_CURRENT_LIST_DIR}/include/ik_geo.h"
    "${CMAKE_BINARY_DIR}/ik_geo.h")

