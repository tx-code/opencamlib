target_sources(ocl
    PRIVATE
    ballcutter.cpp
    bullcutter.cpp
    compositecutter.cpp
    conecutter.cpp
    cylcutter.cpp
    ellipse.cpp
    ellipseposition.cpp
    millingcutter.cpp
)

target_sources(ocl
    PUBLIC
    ballcutter.hpp
    bullcutter.hpp
    compositecutter.hpp
    conecutter.hpp
    cylcutter.hpp
    ellipse.hpp
    ellipseposition.hpp
    millingcutter.hpp
)

target_include_directories(ocl
    PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}
)
