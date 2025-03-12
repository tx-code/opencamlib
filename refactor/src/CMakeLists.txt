cmake_minimum_required(VERSION 3.14)
project(OpenCAMLib VERSION 2025.3 LANGUAGES CXX)

# 设置C++17标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 设置构建类型
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# 查找依赖
find_package(Eigen3 3.4 REQUIRED)
find_package(Libigl CONFIG REQUIRED)

# 添加子目录
add_subdirectory(common)
add_subdirectory(geo)

# 创建主库
add_library(opencamlib INTERFACE)
target_link_libraries(opencamlib INTERFACE
    ocl_geo
)

# 添加测试
option(OCL_BUILD_TESTS "Build tests" ON)

if(OCL_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif()