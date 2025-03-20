cmake_minimum_required(VERSION 3.21)

project(OpenCAMLib LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# 默认构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Relase" CACHE STRING "Build type" FORCE)
endif()

# Common 3rd party libraries
find_package(libigl CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)

add_subdirectory(refactor/src)

option(OCL_BUILD_EXAMPLES "build the examples to try the ocl" ON)
option(OCL_BUILD_TESTS "build the tests to test the ocl" OFF)

if(OCL_BUILD_EXAMPLES)
    add_subdirectory(refactor/examples)
endif()

if(OCL_BUILD_TESTS)
    enable_testing()
endif()

# TODO install rules
