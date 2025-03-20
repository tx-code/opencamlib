include(version_string.cmake)

# 添加子目录

# We now only have one shared library, ocl
add_library(ocl STATIC)

target_sources(ocl
    PRIVATE ocl.cpp
    PUBLIC ocl.hpp
)

target_include_directories(ocl
    PRIVATE
    ${CMAKE_CURRENT_BINARY_DIR}
)

target_link_libraries(ocl
    PRIVATE
    Boost::math
    Boost::graph
)

add_subdirectory(common)
add_subdirectory(geo)
add_subdirectory(cutters)
add_subdirectory(dropcutter)
add_subdirectory(algo)
