target_sources(ocl
    PRIVATE
    adaptivepathdropcutter.cpp
    batchdropcutter.cpp
    pathdropcutter.cpp
    pointdropcutter.cpp
)

target_sources(ocl
    PUBLIC
    adaptivepathdropcutter.hpp
    batchdropcutter.hpp
    pathdropcutter.hpp
    pointdropcutter.hpp
)

target_include_directories(ocl
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
