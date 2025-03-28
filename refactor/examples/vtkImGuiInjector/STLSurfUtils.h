#pragma once

#include "geo/stlsurf.hpp"
#include <Eigen/Dense>


void ExtractVF(const ocl::STLSurf& surf, Eigen::MatrixXd& V, Eigen::MatrixXi& F);

// Upsample the surface mesh...
void SubdivideSurface(ocl::STLSurf& surf, int level = 1);

void RandomPerturbation(ocl::STLSurf& surf,
                        double max_move_distance = 0.01,
                        bool do_project = true);

void CreateVoxelGrid(const ocl::STLSurf& surf,
                     const int size,
                     const int pad_count,
                     Eigen::MatrixXd& GV,
                     Eigen::RowVector3i& res);
