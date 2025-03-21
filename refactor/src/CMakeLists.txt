include(version_string.cmake)

# 添加子目录
message(STATUS "Building shared library")
add_library(ocl SHARED)

target_sources(ocl
    PRIVATE ocl.cpp
    PUBLIC ocl.hpp
)

include(GenerateExportHeader)

generate_export_header(ocl
    BASE_NAME ocl
    EXPORT_MACRO_NAME OCL_API
    EXPORT_FILE_NAME ${CMAKE_CURRENT_BINARY_DIR}/ocl_export.hpp
    STATIC_DEFINE OCL_STATIC
)

target_include_directories(ocl
    PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}> # include the version_string.hpp and ocl_export.hpp
)

target_link_libraries(ocl
    PUBLIC
    spdlog::spdlog
    igl::igl_core
    igl_copyleft::igl_copyleft_cgal
    Eigen3::Eigen
    Boost::math
    Boost::graph
)

add_subdirectory(common)
add_subdirectory(geo)
add_subdirectory(cutters)
add_subdirectory(dropcutter)
add_subdirectory(algo)
