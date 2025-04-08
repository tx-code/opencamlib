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

# 检测是否为多配置生成器
get_property(is_multi_config GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)

if(is_multi_config)
    # 多配置生成器（如 Visual Studio）
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

    # 为每种构建类型设置特定输出目录
    foreach(OUTPUT_CONFIG ${CMAKE_CONFIGURATION_TYPES})
        string(TOLOWER ${OUTPUT_CONFIG} OUTPUT_CONFIG_LOWER)
        string(TOUPPER ${OUTPUT_CONFIG} OUTPUT_CONFIG_UPPER)

        set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${OUTPUT_CONFIG_UPPER} ${CMAKE_BINARY_DIR}/bin/${OUTPUT_CONFIG_LOWER})
        set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${OUTPUT_CONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib/${OUTPUT_CONFIG_LOWER})
        set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${OUTPUT_CONFIG_UPPER} ${CMAKE_BINARY_DIR}/lib/${OUTPUT_CONFIG_LOWER})
    endforeach()

    message(STATUS "Configured for multi-config generator")
else()
    # 单配置生成器（如 Unix Makefiles）
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE "Debug" CACHE STRING "Choose the type of build." FORCE)
    endif()

    string(TOLOWER ${CMAKE_BUILD_TYPE} build_type_lower)

    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin/${build_type_lower})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${build_type_lower})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib/${build_type_lower})

    message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
    message(STATUS "Binary output directory: ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
endif()

# See https://ccache.dev/, you may need install ccache first
find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    set(CMAKE_CXX_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER "${CCACHE_PROGRAM}")
    message(STATUS "Using ccache: ${CCACHE_PROGRAM}")
endif()

# Clangd need this
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Common 3rd party libraries
find_package(libigl CONFIG REQUIRED)
find_package(CGAL REQUIRED) # Although the CGAL is linked with the libigl, we explicitly find it here to avoid the conflict.
find_package(spdlog CONFIG REQUIRED)
find_package(Boost REQUIRED COMPONENTS math graph)
find_package(OpenMP REQUIRED)
find_package(TBB REQUIRED)

add_subdirectory(refactor/src)

option(OCL_BUILD_SHARED "build the ocl to shared library" ON)
option(OCL_BUILD_EXAMPLES "build the examples to try the ocl" OFF)
option(OCL_BUILD_TESTS "build the tests to test the ocl" OFF)

if(OCL_BUILD_EXAMPLES)
    add_subdirectory(refactor/examples)
endif()

if(OCL_BUILD_TESTS)
    add_subdirectory(refactor/tests)
endif()

# TODO install rules
