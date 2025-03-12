# 几何模块的CMakeLists.txt

# 设置源文件
set(OCL_GEO_SRCS

    # 目前我们只有头文件，没有源文件
)

# 设置头文件
set(OCL_GEO_HDRS
    geometry.hpp
    mesh.hpp
    line.hpp
    arc.hpp
    path.hpp
    cutter_point.hpp
)

# 创建库
add_library(ocl_geo INTERFACE)

# 设置包含目录
target_include_directories(ocl_geo INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

# 链接依赖
target_link_libraries(ocl_geo INTERFACE
    ocl_common
    Eigen3::Eigen
    igl::igl_core
)