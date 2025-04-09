#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Polygon_mesh_processing/IO/polygon_mesh_io.h>
#include <CGAL/Polygon_mesh_processing/random_perturbation.h>
#include <CGAL/Surface_mesh.h>
#include <fstream>
#include <gtest/gtest.h>
#include <string>
#include <vector>


#include "algo/fiber.hpp"
#include "algo/fiberpushcutter.hpp"
#include "algo/interval.hpp"
#include "cutters/ballcutter.hpp"
#include "cutters/conecutter.hpp"
#include "cutters/cylcutter.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"


using namespace ocl;

// STL模型目录路径由CMake传入
#ifndef STL_MODELS_DIR
#define STL_MODELS_DIR "../../../stl"
#endif


using K = CGAL::Exact_predicates_inexact_constructions_kernel;
using SurfaceMesh = CGAL::Surface_mesh<K::Point_3>;
namespace PMP = CGAL::Polygon_mesh_processing;

namespace
{
void ReadPolygonMesh(const std::string& filename, ocl::STLSurf& surf)
{
    SurfaceMesh mesh;
    if (!PMP::IO::read_polygon_mesh(filename, mesh)) {
        spdlog::error("Failed to read polygon mesh from {}", filename);
        return;
    }

    surf.tris.clear();
    assert(CGAL::is_triangle_mesh(mesh));
    for (auto f : mesh.faces()) {
        auto h = mesh.halfedge(f);
        auto v0 = mesh.source(h);
        const auto& p0 = mesh.point(v0);
        auto v1 = mesh.target(h);
        const auto& p1 = mesh.point(v1);
        auto v2 = mesh.target(mesh.next(h));
        const auto& p2 = mesh.point(v2);
        surf.addTriangle(ocl::Triangle(ocl::Point(p0[0], p0[1], p0[2]),
                                       ocl::Point(p1[0], p1[1], p1[2]),
                                       ocl::Point(p2[0], p2[1], p2[2])));
    }

    spdlog::info("Loaded {} triangles from {}", surf.size(), filename);
}
}  // namespace

class FiberPushCutterSTLTest: public ::testing::Test
{
protected:
    void SetUp() override
    {
        // 创建铣刀
        ballCutter = new BallCutter(6.0, 20.0);
        cylCutter = new CylCutter(6.0, 20.0);
        coneCutter = new ConeCutter(6.0, 20.0, 45.0);

        // 创建FiberPushCutter
        fpc = new FiberPushCutter();
    }

    void TearDown() override
    {
        delete ballCutter;
        delete cylCutter;
        delete coneCutter;
        delete fpc;
    }

    // 辅助函数 - 加载STL文件
    STLSurf loadStlModel(const std::string& filename)
    {
        std::string filepath = std::string(STL_MODELS_DIR) + "/" + filename;
        STLSurf surface;

        ReadPolygonMesh(filepath, surface);

        bool hasTriangles = !surface.tris.empty();
        EXPECT_TRUE(hasTriangles) << "STL file has no triangles: " << filepath;

        return surface;
    }

    // 辅助函数 - 在一个模型上创建并测试一组fiber
    void testFibersOnModel(STLSurf& model, MillingCutter* cutter, const std::string& direction)
    {
        fpc->setCutter(cutter);

        // 根据方向设置FiberPushCutter
        if (direction == "X") {
            fpc->setXDirection();
        }
        else if (direction == "Y") {
            fpc->setYDirection();
        }
        else {
            FAIL() << "Invalid direction: " << direction;
        }

        // 设置STL模型
        fpc->setSTL(model);

#if 0
        // 确定模型边界
        double xmin = std::numeric_limits<double>::max();
        double xmax = std::numeric_limits<double>::lowest();
        double ymin = std::numeric_limits<double>::max();
        double ymax = std::numeric_limits<double>::lowest();
        double zmin = std::numeric_limits<double>::max();
        double zmax = std::numeric_limits<double>::lowest();

        for (auto& t : model.tris) {
            for (int j = 0; j < 3; j++) {
                Point p = t.p[j];
                xmin = std::min(xmin, p.x);
                xmax = std::max(xmax, p.x);
                ymin = std::min(ymin, p.y);
                ymax = std::max(ymax, p.y);
                zmin = std::min(zmin, p.z);
                zmax = std::max(zmax, p.z);
            }
        }

        // 添加余量确保fiber能穿过整个模型
        const double margin = 10.0;
        xmin -= margin;
        xmax += margin;
        ymin -= margin;
        ymax += margin;
        zmin -= margin;
        zmax += margin;
#else
        const auto& minPt = model.bb.minpt;
        const auto& maxPt = model.bb.maxpt;
        double xmin = minPt.x - 2 * cutter->getRadius();
        double xmax = maxPt.x + 2 * cutter->getRadius();
        double ymin = minPt.y - 2 * cutter->getRadius();
        double ymax = maxPt.y + 2 * cutter->getRadius();
        double zmin = minPt.z - 2 * cutter->getRadius();
        double zmax = maxPt.z + 2 * cutter->getRadius();
#endif

        // 创建fiber网格，根据指定间隔
        constexpr double fiberSpacing = 0.5;  // fiber间隔为0.5单位
        std::vector<Fiber> fibers;

        if (direction == "X") {
            // X方向的fiber (在YZ平面上创建网格)
            // 计算在Y和Z方向上需要多少根fiber
            int yCount = static_cast<int>((ymax - ymin) / fiberSpacing) + 1;
            int zCount = static_cast<int>((zmax - zmin) / fiberSpacing) + 1;

            for (int yi = 0; yi < yCount; yi++) {
                double y = ymin + yi * fiberSpacing;
                for (int zi = 0; zi < zCount; zi++) {
                    double z = zmin + zi * fiberSpacing;

                    Point startPoint(xmin, y, z);
                    Point endPoint(xmax, y, z);
                    fibers.push_back(Fiber(startPoint, endPoint));
                }
            }
        }
        else if (direction == "Y") {
            // Y方向的fiber (在XZ平面上创建网格)
            // 计算在X和Z方向上需要多少根fiber
            int xCount = static_cast<int>((xmax - xmin) / fiberSpacing) + 1;
            int zCount = static_cast<int>((zmax - zmin) / fiberSpacing) + 1;

            for (int xi = 0; xi < xCount; xi++) {
                double x = xmin + xi * fiberSpacing;
                for (int zi = 0; zi < zCount; zi++) {
                    double z = zmin + zi * fiberSpacing;

                    Point startPoint(x, ymin, z);
                    Point endPoint(x, ymax, z);
                    fibers.push_back(Fiber(startPoint, endPoint));
                }
            }
        }

        // 执行FiberPushCutter计算
        int hitCount = 0;
        for (auto& fiber : fibers) {
            fpc->run(fiber);

            if (!fiber.ints.empty()) {
                hitCount++;

                // 检查intervals是否有效
                for (auto& interval : fiber.ints) {
                    EXPECT_GE(interval.lower, 0.0 - 1e-6);
                    EXPECT_LE(interval.upper, 1.0 + 1e-6);
                    EXPECT_LE(interval.lower, interval.upper);
                }
            }
        }

        // 至少应该有一些fiber与模型相交
        EXPECT_GT(hitCount, 0) << "No fibers intersected the model in " << direction
                               << " direction";
        spdlog::info("Generated {} fibers in {} direction, {} hit the model",
                     fibers.size(),
                     direction,
                     hitCount);
    }

    // 成员变量
    BallCutter* ballCutter;
    CylCutter* cylCutter;
    ConeCutter* coneCutter;
    FiberPushCutter* fpc;
};

// Only test the issue model...

// pycam-textbox.stl
TEST_F(FiberPushCutterSTLTest, PycamTextboxTest)
{
    STLSurf model = loadStlModel("pycam-textbox.stl");
    testFibersOnModel(model, ballCutter, "X");
    testFibersOnModel(model, ballCutter, "Y");

    // testFibersOnModel(model, cylCutter, "X");
    // testFibersOnModel(model, cylCutter, "Y");

    // testFibersOnModel(model, coneCutter, "X");
    // testFibersOnModel(model, coneCutter, "Y");
}