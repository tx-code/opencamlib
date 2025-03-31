#pragma once

#include <algorithm>
#include <list>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include <CGAL/AABB_traits_3.h>
#include <CGAL/AABB_tree.h>
#include <CGAL/AABB_triangle_primitive_3.h>
#include <CGAL/Simple_cartesian.h>

#include "geo/bbox.hpp"
#include "geo/clpoint.hpp"
#include "geo/triangle.hpp"

namespace ocl
{
// TEST ON 2025-03-31: Build time有较大的优势，但是查询似乎效率差不多？
// 可能原因是频繁的Point、Triangle转换导致效率低下；此外搜索的时候考虑的是3个维度，而kdtree只需要考虑2个维度
// 但值得使用该Adaptor的理由也是该查询结构可以支持3个维度，也支持更多的QueryType
class AABBTreeAdaptor
{
public:
    // Prepare the types needed by the AABB tree
    using K = CGAL::Simple_cartesian<double>;
    using FT = K::FT;
    using Triangle_3 = K::Triangle_3;
    using Point_3 = K::Point_3;

    using InputTypes = std::list<ocl::Triangle>;
    using Input_Iterator = InputTypes::const_iterator;
    using QueryResult = std::vector<ocl::Triangle>;

    // This primitive provides the conversion facilities between the custom triangle and point types
    // and the CGAL ones
    struct CustomTrianglePrimitive
    {
    public:
        // this is the type of data that the queries returns. For this example
        // we imagine that, for some reasons, we do not want to store the iterators
        // of the vector, but raw pointers. This is to show that the Id type
        // does not have to be the same as the one of the input parameter of the
        // constructor.
        typedef const ocl::Triangle* Id;

        // CGAL types returned
        typedef K::Point_3 Point;     // CGAL 3D point type
        typedef K::Triangle_3 Datum;  // CGAL 3D triangle type

    private:
        Id m_pt;  // this is what the AABB tree stores internally

    public:
        // Default constructor needed
        CustomTrianglePrimitive()
        {}

        // The Following constructor is the one that receives the iterators from iterator range
        // given as input to the AABB tree
        CustomTrianglePrimitive(Input_Iterator it)
            : m_pt(&*it)
        {}

        const Id& id() const
        {
            return m_pt;
        }

        // Utility function to convert a custom point
        // type to CGAL point type
        Point_3 convert(const ocl::Point& p) const
        {
            return Point_3(p.x, p.y, p.z);
        }

        // On the fly conversion from the internal data to the CGAL types
        Datum datum() const
        {
            return Datum(convert(m_pt->p[0]), convert(m_pt->p[1]), convert(m_pt->p[2]));
        }

        // returns a reference point which must be on the primitive
        Point reference_point() const
        {
            return convert(m_pt->p[0]);
        }
    };
    using AABB_triangle_traits = CGAL::AABB_traits_3<K, CustomTrianglePrimitive>;
    using Tree = CGAL::AABB_tree<AABB_triangle_traits>;
    using Node = CGAL::AABB_node<AABB_triangle_traits>;

    // 跟原先的KDtree不同之处，这里的查询维度可以是XYZ
    enum class QueryDimensions
    {
        QD_XY,   // XY
        QD_XZ,   // XZ
        QD_YZ,   // YZ
        QD_XYZ,  // XYZ
    };

    // Most public members are similar to the ocl::KDTree<Triangle> class
public:
    AABBTreeAdaptor() = default;

    void setXYDimensions()
    {
        query_dimensions = QueryDimensions::QD_XY;
    }

    void setXZDimensions()
    {
        query_dimensions = QueryDimensions::QD_XZ;
    }

    void setYZDimensions()
    {
        query_dimensions = QueryDimensions::QD_YZ;
    }

    /// build the AABB tree based on a list of input triangles
    void build(const InputTypes& list)
    {
        spdlog::stopwatch sw;

        // Build the AABB tree with the triangles
        tree.clear();
        tree.insert(list.begin(), list.end());
        tree.accelerate_distance_queries();

        spdlog::info("AABBTree::build() size:={} time:={} s", list.size(), sw);
    }

    /// search for overlap with input Bbox bb, return found objects
    [[nodiscard]] QueryResult search(const Bbox& bb) const
    {
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
        }
        else if (query_dimensions == QueryDimensions::QD_XZ) {
            // In Y Dimension, the range is from -infinity to +infinity
            query_box = K::Iso_cuboid_3(minpt.x,
                                        std::numeric_limits<double>::lowest(),
                                        minpt.z,
                                        maxpt.x,
                                        std::numeric_limits<double>::max(),
                                        maxpt.z);
        }
        else if (query_dimensions == QueryDimensions::QD_YZ) {
            // In X Dimension, the range is from -infinity to +infinity
            query_box = K::Iso_cuboid_3(std::numeric_limits<double>::lowest(),
                                        minpt.y,
                                        minpt.z,
                                        std::numeric_limits<double>::max(),
                                        maxpt.y,
                                        maxpt.z);
        }
        else if (query_dimensions == QueryDimensions::QD_XYZ) {
            query_box = K::Iso_cuboid_3(minpt.x, minpt.y, minpt.z, maxpt.x, maxpt.y, maxpt.z);
        }
        std::list<CustomTrianglePrimitive::Id> primitives;
        // Find all triangles that intersect with the box
        tree.all_intersected_primitives(query_box, std::back_inserter(primitives));
        if (primitives.empty()) {
            return {};
        }

        QueryResult result(primitives.size());
        // Convert CGAL primitives back to OCL triangles
        std::transform(primitives.begin(),
                       primitives.end(),
                       result.begin(),
                       [](const CustomTrianglePrimitive::Id& p) {
                           return *p;
                       });

        return result;
    }

    /// search for overlap with a MillingCutter c positioned at cl, return found objects
    template<typename CutterType>
    [[nodiscard]] QueryResult search_cutter_overlap(const CutterType* c, const CLPoint* cl)
    {
        double r = c->getRadius();
        // build a bounding-box at the current CL
        Bbox bb(cl->x - r, cl->x + r, cl->y - r, cl->y + r, cl->z, cl->z + c->getLength());
        return this->search(bb);
    }

    /// string representation
    std::string str() const
    {
        return "AABBTreeAdaptor with " + std::to_string(tree.size()) + " primitives";
    }

    /// 获取AABBTree
    const Tree& getTree() const
    {
        return tree;
    }

private:
    Tree tree;
    QueryDimensions query_dimensions = QueryDimensions::QD_XY;
};

}  // namespace ocl
