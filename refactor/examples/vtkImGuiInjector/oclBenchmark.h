#pragma once

#include "dropcutter/batchdropcutter.hpp"
#include "oclUtils.h"

// Initialize a dedicated benchmark logger that outputs to a separate file
void init_benchmark_logger(const std::string& log_file_path = "benchmark.log");

// Fix the surface and cutter, and run the batchdropcutter with different number of points (w/o tbb,
// 10, 1e2, 1e3, 1e4, 1e5, 1e6, 1e7)
void run_batchdropcutter(const CAMModelManager& model, bool verbose = true);

// Fix the surface and cutter, and run the batchdropcutter with different bucket size (1~10 in
// kd-tree)
void run_BatchDropCutter_WithDifferentBucketSize(const CAMModelManager& model, bool verbose = true);

// Use Subdivision algorithm to subdivide the surface, Up to 1e7 facets
void run_SurfaceSubdivisionBatchDropCutter(const CAMModelManager& model, bool verbose = true);

// Run the AABBTree and KDTree with the same input (surface and cutter), and compare the performance
void run_AABBTree_VS_KDTree(const CAMModelManager& model, bool verbose = true);
