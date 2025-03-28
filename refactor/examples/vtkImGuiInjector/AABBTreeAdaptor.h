#pragma once

/***********************************************************************
 * AABB Tree adaptor using libigl's AABB implementation for OpenCAMlib
 * This provides a compatible interface with OCL's native KDTree
 *************************************************************************/

#include "cutters/millingcutter.hpp"
#include "geo/bbox.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"
#include <Eigen/Core>
#include <igl/AABB.h>
#include <igl/point_mesh_squared_distance.h>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <spdlog/spdlog.h>
#include <vector>


namespace ocl
{

template<class BBObj>
class AABBTreeLibIGL
{
public:
    AABBTreeLibIGL()
        : search_dims(0)
    {}
    virtual ~AABBTreeLibIGL()
    {}

    // 设置桶大小（为了兼容OCL接口，实际不使用）
    void setBucketSize(int b)
    {
        // 忽略，libigl AABB不使用bucket size
    }

    // 设置XY平面搜索
    void setXYDimensions()
    {
        search_dims = 0;  // XY平面
    }

    // 设置YZ平面搜索
    void setYZDimensions()
    {
        search_dims = 1;  // YZ平面
    }

    // 设置XZ平面搜索
    void setXZDimensions()
    {
        search_dims = 2;  // XZ平面
    }

    // 从三角形列表构建树
    void build(const std::list<BBObj>& list)
    {
        // 将三角形转换为Eigen矩阵格式
        triangles.resize(list.size());
        vertices.resize(list.size() * 3, 3);
        faces.resize(list.size(), 3);

        int tri_idx = 0;
        for (const auto& tri : list) {
            triangles[tri_idx] = tri;

            // 提取顶点
            vertices.row(tri_idx * 3 + 0) << tri.p[0].x, tri.p[0].y, tri.p[0].z;
            vertices.row(tri_idx * 3 + 1) << tri.p[1].x, tri.p[1].y, tri.p[1].z;
            vertices.row(tri_idx * 3 + 2) << tri.p[2].x, tri.p[2].y, tri.p[2].z;

            // 面索引
            faces.row(tri_idx) << tri_idx * 3 + 0, tri_idx * 3 + 1, tri_idx * 3 + 2;

            tri_idx++;
        }

        // 构建AABB树
        tree.init(vertices, faces);
    }

    // 从STLSurf构建
    void setSTL(const STLSurf& s)
    {
        std::list<BBObj> list;
        for (const auto& tri : s.tris) {
            list.push_back(tri);
        }
        build(list);
    }

    // 搜索与边界盒重叠的对象
    // FIXME: unfinished...
    std::list<BBObj>* search(const Bbox& bb)
    {
        spdlog::stopwatch sw;
        std::list<BBObj>* result = new std::list<BBObj>();

        // 使用libigl AABB树进行高效搜索
        std::queue<igl::AABB<Eigen::MatrixXd, 3>*> queue;
        queue.push(&tree);

        while (!queue.empty()) {
            igl::AABB<Eigen::MatrixXd, 3>* node = queue.front();
            queue.pop();

            // 检查当前节点的边界盒是否与查询边界盒重叠
            Eigen::AlignedBox3d node_box = node->m_box;


            // 创建查询边界盒
            Eigen::AlignedBox3d query_box;
            query_box.min() = Eigen::Vector3d(bb[0], bb[2], bb[4]);
            query_box.max() = Eigen::Vector3d(bb[1], bb[3], bb[5]);

            // 检查是否重叠 - 根据search_dims值选择合适的测试方法
            bool overlaps = false;
            if (search_dims == 0) {  // XY平面
                overlaps = (node_box.min().x() <= query_box.max().x()
                            && node_box.max().x() >= query_box.min().x()
                            && node_box.min().y() <= query_box.max().y()
                            && node_box.max().y() >= query_box.min().y());
            }
            else if (search_dims == 1) {  // YZ平面
                overlaps = (node_box.min().y() <= query_box.max().y()
                            && node_box.max().y() >= query_box.min().y()
                            && node_box.min().z() <= query_box.max().z()
                            && node_box.max().z() >= query_box.min().z());
            }
            else if (search_dims == 2) {  // XZ平面
                overlaps = (node_box.min().x() <= query_box.max().x()
                            && node_box.max().x() >= query_box.min().x()
                            && node_box.min().z() <= query_box.max().z()
                            && node_box.max().z() >= query_box.min().z());
            }
            else {  // 完整3D重叠测试
                overlaps = query_box.intersects(node_box);
            }

            if (!overlaps) {
                continue;  // 如果不重叠，跳过这个节点及其所有子节点
            }

            // 如果是叶节点，测试包含的三角形
            if (node->is_leaf()) {
                // 获取该节点的三角形索引范围
                std::vector<int> leaf_triangles;
                for (int i = 0; i < faces.rows(); ++i) {
                    // 构建当前三角形的包围盒
                    Eigen::RowVector3d v1 = vertices.row(faces(i, 0));
                    Eigen::RowVector3d v2 = vertices.row(faces(i, 1));
                    Eigen::RowVector3d v3 = vertices.row(faces(i, 2));

                    Eigen::AlignedBox3d tri_box;
                    tri_box.extend(v1.transpose());
                    tri_box.extend(v2.transpose());
                    tri_box.extend(v3.transpose());

                    // 如果三角形与节点包围盒相交，可能属于该节点
                    if (node_box.intersects(tri_box)) {
                        int tri_idx = i;
                        if (tri_idx < triangles.size()) {
                            const BBObj& tri = triangles[tri_idx];

                            // 检查三角形的边界盒是否与查询边界盒重叠
                            bool triangle_overlaps = true;
                            if (search_dims == 0) {  // XY平面
                                triangle_overlaps =
                                    (tri.bb[1] >= bb[0] && tri.bb[0] <= bb[1] &&  // X重叠
                                     tri.bb[3] >= bb[2] && tri.bb[2] <= bb[3]);   // Y重叠
                            }
                            else if (search_dims == 1) {  // YZ平面
                                triangle_overlaps =
                                    (tri.bb[3] >= bb[2] && tri.bb[2] <= bb[3] &&  // Y重叠
                                     tri.bb[5] >= bb[4] && tri.bb[4] <= bb[5]);   // Z重叠
                            }
                            else if (search_dims == 2) {  // XZ平面
                                triangle_overlaps =
                                    (tri.bb[1] >= bb[0] && tri.bb[0] <= bb[1] &&  // X重叠
                                     tri.bb[5] >= bb[4] && tri.bb[4] <= bb[5]);   // Z重叠
                            }
                            else {  // 3D重叠
                                triangle_overlaps =
                                    (tri.bb[1] >= bb[0] && tri.bb[0] <= bb[1] &&  // X重叠
                                     tri.bb[3] >= bb[2] && tri.bb[2] <= bb[3] &&  // Y重叠
                                     tri.bb[5] >= bb[4] && tri.bb[4] <= bb[5]);   // Z重叠
                            }

                            if (triangle_overlaps) {
                                result->push_back(tri);
                            }
                        }
                    }
                }
            }
            else {
                // 非叶子节点，添加子节点到队列
                if (node->m_left)
                    queue.push(node->m_left);
                if (node->m_right)
                    queue.push(node->m_right);
            }
        }

        return result;
    }

    // 搜索与刀具重叠的对象
    std::list<BBObj>* search_cutter_overlap(const MillingCutter* c, CLPoint* cl)
    {
        double r = c->getRadius();
        // 构建刀具边界盒
        Bbox bb(cl->x - r, cl->x + r, cl->y - r, cl->y + r, cl->z, cl->z + c->getLength());
        return this->search(bb);
    }

    // 射线相交测试（可用于更精确的查询）
    bool intersect_ray(const Point& origin,
                       const Point& direction,
                       double& hit_t,
                       Point& hit_point,
                       int& hit_face_idx)
    {
        // 创建射线
        Eigen::RowVector3d ray_origin;
        ray_origin << origin.x, origin.y, origin.z;

        Eigen::RowVector3d ray_direction;
        ray_direction << direction.x, direction.y, direction.z;

        // 使用libigl的射线相交测试
        Eigen::VectorXi I;
        Eigen::VectorXd T;
        Eigen::MatrixXd UV;
        double min_t = std::numeric_limits<double>::infinity();

        tree.intersect_ray(vertices, faces, ray_origin, ray_direction, min_t, I, T, UV);

        if (I.size() > 0) {
            hit_t = T(0);
            hit_face_idx = I(0);

            // 计算交点位置
            hit_point.x = origin.x + direction.x * hit_t;
            hit_point.y = origin.y + direction.y * hit_t;
            hit_point.z = origin.z + direction.z * hit_t;

            return true;
        }

        return false;
    }

    // 查找包含点的所有三角形（基于libigl的find方法）
    std::list<BBObj>* find_triangles_containing_point(const Point& point)
    {
        std::list<BBObj>* result = new std::list<BBObj>();

        // 转换为Eigen格式
        Eigen::RowVector3d query_point;
        query_point << point.x, point.y, point.z;

        // 使用libigl的find方法
        std::vector<int> face_indices = tree.find(vertices, faces, query_point);

        // 添加找到的三角形
        for (int idx : face_indices) {
            if (idx >= 0 && idx < triangles.size()) {
                result->push_back(triangles[idx]);
            }
        }

        return result;
    }

private:
    int search_dims;  // 0: XY, 1: YZ, 2: XZ
    igl::AABB<Eigen::MatrixXd, 3> tree;
    Eigen::MatrixXd vertices;
    Eigen::MatrixXi faces;
    std::vector<BBObj> triangles;
};

}  // namespace ocl
