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
find_package(Boost REQUIRED COMPONENTS unit_test_framework)

# 添加子目录
add_subdirectory(common)
add_subdirectory(geo)

# 创建主库
add_library(opencamlib INTERFACE)
target_link_libraries(opencamlib INTERFACE
    ocl_geo
)

# 导出目标
install(EXPORT ocl-targets
    FILE opencamlib-targets.cmake
    NAMESPACE opencamlib::
    DESTINATION lib/cmake/opencamlib
)

# 创建配置文件
include(CMakePackageConfigHelpers)
configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/opencamlib-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/opencamlib-config.cmake
    INSTALL_DESTINATION lib/cmake/opencamlib
)

# 创建版本文件
write_basic_package_version_file(
    ${CMAKE_CURRENT_BINARY_DIR}/opencamlib-config-version.cmake
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# 安装配置文件
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/opencamlib-config.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/opencamlib-config-version.cmake
    DESTINATION lib/cmake/opencamlib
)

# 添加测试
option(OCL_BUILD_TESTS "Build tests" ON)

if(OCL_BUILD_TESTS)
    include(CTest)
    enable_testing()
    add_subdirectory(tests)
endif()