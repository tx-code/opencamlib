target_sources(ocl
  PRIVATE
  adaptivewaterline.cpp
  batchpushcutter.cpp
  fiber.cpp
  fiberpushcutter.cpp
  interval.cpp
  simple_weave.cpp
  smart_weave.cpp
  waterline.cpp
  weave.cpp
)

target_sources(ocl
  PUBLIC
  adaptivewaterline.hpp
  batchpushcutter.hpp
  clsurface.hpp
  fiber.hpp
  fiberpushcutter.hpp
  interval.hpp
  operation.hpp
  simple_weave.hpp
  smart_weave.hpp
  tsp.hpp
  waterline.hpp
  weave.hpp
  weave_typedef.hpp
  zigzag.hpp
)