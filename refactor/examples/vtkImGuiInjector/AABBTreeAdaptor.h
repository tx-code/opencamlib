#pragma once

#include <list>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive_3.h>
#include <CGAL/Simple_cartesian.h>

#include "cutters/millingcutter.hpp"
#include "geo/bbox.hpp"
#include "geo/triangle.hpp"


namespace ocl {
    template<typename InputPrimitive>
    class AABBTreeAdaptor {
    public:
        // Prepare the types needed by the AABB tree
        using K = CGAL::Simple_cartesian<double>;
        using FT = K::FT;
        using Point = K::Point_3;
        using Triangle_3 = K::Triangle_3;
        using Iterator = std::list<Triangle_3>::const_iterator;
        using Primitive = CGAL::AABB_triangle_primitive_3<K, Iterator>;
        using AABB_triangle_traits = CGAL::AABB_traits_3<K, Primitive>;
        using Tree = CGAL::AABB_tree<AABB_triangle_traits>;
        using Node = CGAL::AABB_node<AABB_triangle_traits>;

        using InputTypes = std::list<InputPrimitive>;
        using QueryResult = std::vector<InputPrimitive>;

        // 跟原先的KDtree不同之处，这里的查询维度可以是XYZ
        enum class QueryDimensions {
            QD_XY, // XY
            QD_XZ, // XZ
            QD_YZ, // YZ
            QD_XYZ, // XYZ
        };

        // Most public members are similar to the ocl::KDTree<Triangle> class
    public:
        AABBTreeAdaptor() = default;

        void setXYDimensions() {
            query_dimensions = QueryDimensions::QD_XY;
        }

        void setXZDimensions() {
            query_dimensions = QueryDimensions::QD_XZ;
        }

        void setYZDimensions() {
            query_dimensions = QueryDimensions::QD_YZ;
        }

        /// build the AABB tree based on a list of input triangles
        void build(const InputTypes& list) {
            spdlog::stopwatch sw;

            // Clear any existing data
            triangles.clear();

            // Convert OCL triangles to CGAL triangles
            for (const auto& tri: list) {
                Point p1(tri.p[0].x, tri.p[0].y, tri.p[0].z);
                Point p2(tri.p[1].x, tri.p[1].y, tri.p[1].z);
                Point p3(tri.p[2].x, tri.p[2].y, tri.p[2].z);
                triangles.push_back(Triangle_3(p1, p2, p3));
            }

            // Build the AABB tree with the triangles
            tree.clear();
            tree.insert(triangles.begin(), triangles.end());
            tree.accelerate_distance_queries();

            spdlog::info("AABBTree::build() size:={} time:={} s", list.size(), sw);
        }

        /// search for overlap with input Bbox bb, return found objects
        [[nodiscard]] QueryResult search(const Bbox& bb) {
            QueryResult result;
            std::list<Primitive> primitives;

            // Create CGAL box for querying
            const auto& minpt = bb.minpt;
            const auto& maxpt = bb.maxpt;
            K::Iso_cuboid_3 query_box;
            if (query_dimensions == QueryDimensions::QD_XY) {
                // In Z Dimension, the range is from -infinity to +infinity
                query_box = K::Iso_cuboid_3(minpt.x,
                                            minpt.y,
                                            std::numeric_limits<double>::lowest(),
                                            maxpt.x,
                                            maxpt.y,
                                            std::numeric_limits<double>::max());
            } else if (query_dimensions == QueryDimensions::QD_XZ) {
                // In Y Dimension, the range is from -infinity to +infinity
                query_box = K::Iso_cuboid_3(minpt.x,
                                            std::numeric_limits<double>::lowest(),
                                            minpt.z,
                                            maxpt.x,
                                            std::numeric_limits<double>::max(),
                                            maxpt.z);
            } else if (query_dimensions == QueryDimensions::QD_YZ) {
                // In X Dimension, the range is from -infinity to +infinity
                query_box = K::Iso_cuboid_3(std::numeric_limits<double>::lowest(),
                                            minpt.y,
                                            minpt.z,
                                            std::numeric_limits<double>::max(),
                                            maxpt.y,
                                            maxpt.z);
            } else if (query_dimensions == QueryDimensions::QD_XYZ) {
                query_box = K::Iso_cuboid_3(minpt.x, minpt.y, minpt.z, maxpt.x, maxpt.y, maxpt.z);
            }
            // Find all triangles that intersect with the box
            tree.all_intersected_primitives(query_box, std::back_inserter(primitives));

            // Convert CGAL primitives back to OCL triangles
            for (const auto& primitive: primitives) {
                // TODO
            }
            return result;
        }

        /// search for overlap with a MillingCutter c positioned at cl, return found objects
        [[nodiscard]] QueryResult search_cutter_overlap(const MillingCutter* c, CLPoint* cl) {
            double r = c->getRadius();
            // build a bounding-box at the current CL
            Bbox bb(cl->x - r, cl->x + r, cl->y - r, cl->y + r, cl->z, cl->z + c->getLength());
            return this->search(bb);
        }

        /// string representation
        std::string str() const {
            return "AABBTreeAdaptor with " + std::to_string(triangles.size()) + " triangles";
        }

        /// 获取AABBTree
        const Tree& getTree() const {
            return tree;
        }

    private:
        Tree tree;
        std::list<Triangle_3> triangles;
        QueryDimensions query_dimensions = QueryDimensions::QD_XY;
    };

    using TriangleTree = AABBTreeAdaptor<ocl::Triangle>;
} // namespace ocl
