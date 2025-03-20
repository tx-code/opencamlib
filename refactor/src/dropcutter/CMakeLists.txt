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
