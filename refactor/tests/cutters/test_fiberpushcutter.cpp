#include <gtest/gtest.h>

#include "algo/fiber.hpp"
#include "algo/fiberpushcutter.hpp"
#include "algo/interval.hpp"
#include "cutters/ballcutter.hpp"
#include "cutters/cylcutter.hpp"
#include "geo/ccpoint.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"

using namespace ocl;

// FiberPushCutter的Cube测试的Fixture类
class FiberPushCutterCubeTest: public ::testing::Test
{
protected:
    // 每个测试前运行的代码
    void SetUp() override
    {
        // 创建一个立方体 (0,0,0) 到 (10,10,10)
        // 每个面由两个三角形组成

        // 底面 (z=0)
        cube.addTriangle(Triangle(Point(0, 0, 0), Point(10, 0, 0), Point(0, 10, 0)));
        cube.addTriangle(Triangle(Point(10, 10, 0), Point(10, 0, 0), Point(0, 10, 0)));

        // 顶面 (z=10)
        cube.addTriangle(Triangle(Point(0, 0, 10), Point(0, 10, 10), Point(10, 0, 10)));
        cube.addTriangle(Triangle(Point(10, 10, 10), Point(0, 10, 10), Point(10, 0, 10)));

        // 前面 (y=0)
        cube.addTriangle(Triangle(Point(0, 0, 0), Point(0, 0, 10), Point(10, 0, 0)));
        cube.addTriangle(Triangle(Point(10, 0, 10), Point(0, 0, 10), Point(10, 0, 0)));

        // 后面 (y=10)
        cube.addTriangle(Triangle(Point(0, 10, 0), Point(10, 10, 0), Point(0, 10, 10)));
        cube.addTriangle(Triangle(Point(10, 10, 10), Point(0, 10, 10), Point(10, 10, 0)));

        // 左面 (x=0)
        cube.addTriangle(Triangle(Point(0, 0, 0), Point(0, 10, 0), Point(0, 0, 10)));
        cube.addTriangle(Triangle(Point(0, 10, 10), Point(0, 0, 10), Point(0, 10, 0)));

        // 右面 (x=10)
        cube.addTriangle(Triangle(Point(10, 0, 0), Point(10, 0, 10), Point(10, 10, 0)));
        cube.addTriangle(Triangle(Point(10, 10, 10), Point(10, 0, 10), Point(10, 10, 0)));

        // 创建圆柱形铣刀
        diameter = 6.0;
        length = 20.0;
        radius = diameter / 2.0;
        cutter = new CylCutter(diameter, length);

        // 创建FiberPushCutter
        fpc = new FiberPushCutter();
        fpc->setCutter(cutter);
        fpc->setXDirection();
        fpc->setSTL(cube);
    }

    void TearDown() override
    {
        delete cutter;
        delete fpc;
    }

    // 共享数据成员
    STLSurf cube;
    double diameter;
    double length;
    double radius;
    CylCutter* cutter;
    FiberPushCutter* fpc;
};

// 使用Fixture的测试
TEST_F(FiberPushCutterCubeTest, FiberAlongX)
{
    // 设置X方向
    fpc->setXDirection();

    // X方向的fiber
    Point startPoint(-5, 5, 5);
    Point endPoint(15, 5, 5);
    Fiber fiberX(startPoint, endPoint);

    // 运行FiberPushCutter
    fpc->run(fiberX);

    // 验证interval不为空
    ASSERT_FALSE(fiberX.ints.empty()) << "FiberX should have intervals after FiberPushCutter";
    EXPECT_EQ(fiberX.ints.size(), 1);

    const Interval& interval = fiberX.ints.front();

    // 验证interval的范围
    double expected_lower = (-startPoint.x - radius) / (endPoint.x - startPoint.x);
    double expected_upper = (10 - startPoint.x + radius) / (endPoint.x - startPoint.x);
    EXPECT_NEAR(interval.lower, expected_lower, 1e-5)
        << "X direction fiber should have correct interval.lower";
    EXPECT_NEAR(interval.upper, expected_upper, 1e-5)
        << "X direction fiber should have correct interval.upper";

    // 验证接触点坐标
    Point lowerPoint = fiberX.point(interval.lower);
    Point upperPoint = fiberX.point(interval.upper);

    EXPECT_NEAR(lowerPoint.x, 0 - radius, 1e-5);
    EXPECT_NEAR(upperPoint.x, 10 + radius, 1e-5);
}

TEST_F(FiberPushCutterCubeTest, FiberAlongY)
{
    // 设置Y方向
    fpc->setYDirection();

    // Y方向的fiber
    Point startPoint(5, -5, 5);
    Point endPoint(5, 15, 5);
    Fiber fiberY(startPoint, endPoint);

    // 运行FiberPushCutter
    fpc->run(fiberY);

    // 验证interval不为空
    ASSERT_FALSE(fiberY.ints.empty()) << "FiberY should have intervals after FiberPushCutter";
    EXPECT_EQ(fiberY.ints.size(), 1);

    const Interval& interval = fiberY.ints.front();

    // 验证interval的范围
    double expected_lower = (-startPoint.y - radius) / (endPoint.y - startPoint.y);
    double expected_upper = (10 - startPoint.y + radius) / (endPoint.y - startPoint.y);
    EXPECT_NEAR(interval.lower, expected_lower, 1e-5)
        << "Y direction fiber should have correct interval.lower";
    EXPECT_NEAR(interval.upper, expected_upper, 1e-5)
        << "Y direction fiber should have correct interval.upper";

    // 验证接触点坐标
    Point lowerPoint = fiberY.point(interval.lower);
    Point upperPoint = fiberY.point(interval.upper);

    EXPECT_NEAR(lowerPoint.y, -radius, 1e-5);
    EXPECT_NEAR(upperPoint.y, 10 + radius, 1e-5);
}

TEST_F(FiberPushCutterCubeTest, MultipleFibersGrid)
{
    // 测试X方向的fiber网格
    fpc->setXDirection();

    const int gridSize = 5;
    const double gridStep = 2.5;

    // 在Y-Z平面上创建网格
    for (int y = 0; y < gridSize; y++) {
        for (int z = 0; z < gridSize; z++) {
            // 计算当前fiber的y和z坐标
            double yCoord = y * gridStep;
            double zCoord = z * gridStep;

            // 创建一个X方向的fiber
            Point startPoint(-5, yCoord, zCoord);
            Point endPoint(15, yCoord, zCoord);
            Fiber fiberX(startPoint, endPoint);

            // 运行FiberPushCutter
            fpc->run(fiberX);

            // 验证在立方体内部的fiber都有interval
            bool shouldHaveInterval =
                (yCoord >= 0 && yCoord <= 10 + radius && zCoord >= 0 && zCoord <= 10 + radius);

            if (shouldHaveInterval) {
                ASSERT_FALSE(fiberX.ints.empty()) << "Fiber at y=" << yCoord << ", z=" << zCoord
                                                  << " should have intervals after FiberPushCutter";

                if (!fiberX.ints.empty()) {
                    Interval& interval = fiberX.ints.front();

                    // 验证interval的范围
                    double expected_lower = (-startPoint.x - radius) / (endPoint.x - startPoint.x);
                    double expected_upper =
                        (10 - startPoint.x + radius) / (endPoint.x - startPoint.x);

                    EXPECT_NEAR(interval.lower, expected_lower, 1e-5)
                        << "Fiber at y=" << yCoord << ", z=" << zCoord
                        << " should have correct interval.lower";
                    EXPECT_NEAR(interval.upper, expected_upper, 1e-5)
                        << "Fiber at y=" << yCoord << ", z=" << zCoord
                        << " should have correct interval.upper";
                }
            }
        }
    }

    // 测试Y方向的fiber网格
    fpc->setYDirection();

    // 在X-Z平面上创建网格
    for (int x = 0; x < gridSize; x++) {
        for (int z = 0; z < gridSize; z++) {
            // 计算当前fiber的x和z坐标
            double xCoord = x * gridStep;
            double zCoord = z * gridStep;

            // 创建一个Y方向的fiber
            Point startPoint(xCoord, -5, zCoord);
            Point endPoint(xCoord, 15, zCoord);
            Fiber fiberY(startPoint, endPoint);

            // 运行FiberPushCutter
            fpc->run(fiberY);

            // 验证在立方体内部的fiber都有interval
            bool shouldHaveInterval =
                (xCoord >= 0 && xCoord <= 10 + radius && zCoord >= 0 && zCoord <= 10 + radius);

            if (shouldHaveInterval) {
                ASSERT_FALSE(fiberY.ints.empty()) << "Fiber at x=" << xCoord << ", z=" << zCoord
                                                  << " should have intervals after FiberPushCutter";

                if (!fiberY.ints.empty()) {
                    Interval& interval = fiberY.ints.front();

                    // 验证interval的范围
                    double expected_lower = (-startPoint.y - radius) / (endPoint.y - startPoint.y);
                    double expected_upper =
                        (10 - startPoint.y + radius) / (endPoint.y - startPoint.y);
                    EXPECT_NEAR(interval.lower, expected_lower, 1e-5)
                        << "Fiber at x=" << xCoord << ", z=" << zCoord
                        << " should have correct interval.lower";
                    EXPECT_NEAR(interval.upper, expected_upper, 1e-5)
                        << "Fiber at x=" << xCoord << ", z=" << zCoord
                        << " should have correct interval.upper";
                }
            }
        }
    }
}

TEST_F(FiberPushCutterCubeTest, FiberNearCube)
{
    // 设置X方向
    fpc->setXDirection();

    // 靠近但不相交的fiber (在立方体外)
    Point startPoint(-5, -radius-0.1, 0);
    Point endPoint(15, -radius-0.1, 0);
    Fiber fiberOutside(startPoint, endPoint);

    // 运行FiberPushCutter
    fpc->run(fiberOutside);

    // 应该没有接触
    EXPECT_TRUE(fiberOutside.ints.empty())
        << "Fiber outside should not have intervals after FiberPushCutter";

    auto* offseted_cutter = cutter->offsetCutter(0.1);
    fpc->setCutter(offseted_cutter);

    fpc->run(fiberOutside);
    // 当好接触
    EXPECT_FALSE(fiberOutside.ints.empty());
    EXPECT_EQ(fiberOutside.ints.size(), 1);


    delete offseted_cutter;
}