target_sources(ocl
    PRIVATE
    numeric.cpp
    lineclfilter.cpp
    PUBLIC
    brent_zero.hpp
    clfilter.hpp
    halfedgediagram.hpp
    kdtree.hpp
    kdnode.hpp
    numeric.hpp
    lineclfilter.hpp
)

target_include_directories(ocl
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
