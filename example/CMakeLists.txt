cmake_minimum_required(VERSION 3.21)
project(test)

set(CMAKE_CXX_STANDARD 17)

include(../CMakeLists.txt)

add_executable(test src/main.cpp)
target_include_directories(test PRIVATE ../include)

target_link_libraries(test PRIVATE ik_geo)
target_link_directories(test PRIVATE ${CMAKE_BINARY_DIR})

add_dependencies(test ik_geo_lib)