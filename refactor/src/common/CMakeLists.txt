# common模块的CMakeLists.txt

# 设置源文件
set(OCL_COMMON_SRCS

    # 目前我们只有头文件，没有源文件
)

# 设置头文件
set(OCL_COMMON_HDRS
    types.hpp
)

# 创建库
add_library(ocl_common INTERFACE)

# 设置包含目录
target_include_directories(ocl_common INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
    $<INSTALL_INTERFACE:include>
)

# 链接依赖
target_link_libraries(ocl_common INTERFACE
    Eigen3::Eigen
)

if(MSVC)
    target_compile_options(ocl_common INTERFACE /wd4819)
endif()

# 安装规则
install(TARGETS ocl_common
    EXPORT ocl-targets
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

install(FILES ${OCL_COMMON_HDRS}
    DESTINATION include/ocl/common
)