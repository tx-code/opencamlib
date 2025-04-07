#pragma once

#include "geo/triangle.hpp"

namespace ocl
{
// Create random points in a triangle with options to exclude vertices and edges
std::vector<Point> createRandomPointsInTriangle(const Triangle& triangle,
                                                const size_t num_points,
                                                const bool no_vertex = false,
                                                const bool no_edge = false);
}  // namespace ocl
