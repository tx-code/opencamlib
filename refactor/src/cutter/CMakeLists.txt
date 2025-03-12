cmake_minimum_required(VERSION 3.14)

# 设置头文件
set(HEADERS
    cutter.hpp
    cylindrical_cutter.hpp
    ball_cutter.hpp
    bull_cutter.hpp
    cone_cutter.hpp
    torus_cutter.hpp
)

# 创建库
add_library(ocl_cutter INTERFACE ${HEADERS})

# 设置包含目录
target_include_directories(ocl_cutter
    INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

# 链接依赖库
target_link_libraries(ocl_cutter
    INTERFACE
    ocl_common
    ocl_geo
    Eigen3::Eigen
)