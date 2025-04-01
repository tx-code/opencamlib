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

// Sample the surface mesh for a point cloud (basic version from libigl)
void SampleMeshForPointCloud(const ocl::STLSurf& surf,
                             const int number_points,
                             Eigen::MatrixXd& P,
                             Eigen::MatrixXd& N);

// Use CGAL to read a polygon mesh from a file, not just STL file, but also other file formats
void ReadPolygonMesh(const std::string& filename, ocl::STLSurf& surf);
