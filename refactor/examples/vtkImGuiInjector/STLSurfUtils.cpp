#include "STLSurfUtils.h"
#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/random_perturbation.h>
#include <CGAL/Surface_mesh.h>
#include <igl/per_face_normals.h>
#include <igl/random_points_on_mesh.h>
#include <igl/remove_duplicate_vertices.h>
#include <igl/upsample.h>
#include <igl/voxel_grid.h>
#include <spdlog/spdlog.h>


#include "geo/point.hpp"
#include "geo/triangle.hpp"

using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using SurfaceMesh = CGAL::Surface_mesh<K::Point_3>;

namespace
{

void ToSurfaceMesh(const Eigen::MatrixXd& V, const Eigen::MatrixXi& F, SurfaceMesh& mesh)
{
    mesh.clear();
    for (int i = 0; i < V.rows(); i++) {
        mesh.add_vertex(K::Point_3(V(i, 0), V(i, 1), V(i, 2)));
    }
    for (int i = 0; i < F.rows(); i++) {
        mesh.add_face(CGAL::SM_Vertex_index(F(i, 0)),
                      CGAL::SM_Vertex_index(F(i, 1)),
                      CGAL::SM_Vertex_index(F(i, 2)));
    }
    assert(mesh.number_of_vertices() == V.rows());
    assert(mesh.number_of_faces() == F.rows());
}

}  // namespace

void ExtractVF(const ocl::STLSurf& surf, Eigen::MatrixXd& V, Eigen::MatrixXi& F)
{
    // first, extract all the vertices and faces from the surface
    Eigen::MatrixXd V_all;
    Eigen::MatrixXi F_all;
    V_all.resize(surf.size() * 3, 3);
    F_all.resize(surf.size(), 3);

    int v_idx = 0;
    int f_idx = 0;
    for (const auto& tri : surf.tris) {
        for (const auto& v : tri.p) {
            V_all(v_idx, 0) = v.x;
            V_all(v_idx, 1) = v.y;
            V_all(v_idx, 2) = v.z;
            v_idx++;
        }
        F_all.row(f_idx) << v_idx - 3, v_idx - 2, v_idx - 1;
        f_idx++;
    }

    // then, remove duplicate vertices
    Eigen::MatrixXd V_clean;
    Eigen::MatrixXi F_clean;
    Eigen::VectorXi I, J;  // I is the index of the original vertices, J is the
                           // index of the cleaned vertices
    igl::remove_duplicate_vertices(V_all, F_all, 1e-6, V_clean, I, J, F_clean);
    spdlog::info("Removed {} duplicate vertices", V_all.rows() - V_clean.rows());
    spdlog::info("#V: {}, #F: {}", V_clean.rows(), F_clean.rows());

    V.swap(V_clean);
    F.swap(F_clean);
}

void RandomPerturbation(ocl::STLSurf& surf, double max_move_distance, bool do_project)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    ExtractVF(surf, V, F);

    SurfaceMesh mesh;
    ToSurfaceMesh(V, F, mesh);

    CGAL::Polygon_mesh_processing::random_perturbation(mesh,
                                                       max_move_distance,
                                                       CGAL::parameters::do_project(do_project));

    // Convert back to STLSurf
    // FIXME: we do not need to clear the triangles, but we need to update the vertices
    surf.tris.clear();
    for (auto f : mesh.faces()) {
        auto h = mesh.halfedge(f);
        auto v0 = mesh.source(h);
        const auto& p0 = mesh.point(v0);
        auto v1 = mesh.target(h);
        const auto& p1 = mesh.point(v1);
        auto v2 = mesh.target(mesh.next(h));
        const auto& p2 = mesh.point(v2);

        ocl::Triangle tri(ocl::Point(p0[0], p0[1], p0[2]),
                          ocl::Point(p1[0], p1[1], p1[2]),
                          ocl::Point(p2[0], p2[1], p2[2]));

        surf.addTriangle(tri);
    }
}

void CreateVoxelGrid(const ocl::STLSurf& surf,
                     const int size,
                     const int pad_count,
                     Eigen::MatrixXd& GV,
                     Eigen::RowVector3i& res)
{
    spdlog::info("Extract VF");
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    ExtractVF(surf, V, F);

    spdlog::info("Create Voxel Grid");
    igl::voxel_grid(V, 0, size, pad_count, GV, res);

    spdlog::info("#GV: {}, #res: ({}, {}, {})", GV.rows(), res(0), res(1), res(2));
}

void SubdivideSurface(ocl::STLSurf& surf, int level)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    ExtractVF(surf, V, F);

    Eigen::MatrixXd NV;
    Eigen::MatrixXi NF;
    igl::upsample(V, F, NV, NF, level);

    spdlog::info("Upsampled from #V: {}, #F: {} to #NV: {}, #NF: {}",
                 V.rows(),
                 F.rows(),
                 NV.rows(),
                 NF.rows());

    // to STLSurf
    surf.tris.clear();
    for (int i = 0; i < NF.rows(); i++) {
        auto VF_0 = NV.row(NF(i, 0));
        auto VF_1 = NV.row(NF(i, 1));
        auto VF_2 = NV.row(NF(i, 2));
        surf.addTriangle(ocl::Triangle(ocl::Point(VF_0[0], VF_0[1], VF_0[2]),
                                       ocl::Point(VF_1[0], VF_1[1], VF_1[2]),
                                       ocl::Point(VF_2[0], VF_2[1], VF_2[2])));
    }
}

void SampleMeshForPointCloud(const ocl::STLSurf& surf,
                             const int number_points,
                             Eigen::MatrixXd& P,
                             Eigen::MatrixXd& N)
{
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
    ExtractVF(surf, V, F);

    Eigen::VectorXi I;
    Eigen::MatrixXd B;
    igl::random_points_on_mesh(number_points, V, F, B, I, P);
    Eigen::MatrixXd FN;
    igl::per_face_normals(V, F, FN);
    N.resize(P.rows(), 3);
    for (int i = 0; i < P.rows(); i++) {
        N.row(i) = FN.row(I(i));
    }
}
