#include <gtest/gtest.h>

#include "../utils/triangles_utils.h"
#include "algo/fiber.hpp"
#include "algo/interval.hpp"
#include "cutters/ballcutter.hpp"
#include "cutters/cylcutter.hpp"
#include "geo/ccpoint.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"

using namespace ocl;

TEST(PushCutterTests, HorizontalTriangle)
{
    // 测试水平放置三角形的情况
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(0, 10, 0);
    Triangle triangle(p1, p2, p3);

    // 创建球形铣刀
    double diameter = 6.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    BallCutter cutter(diameter, length);

    // 创建多根平行的fiber，从p1到p3开始，然后在x方向平移
    const int numFibers = 12;  // 生成12根fiber，x从0到11

    for (int i = 0; i < numFibers; i++) {
        // 当前fiber的起点和终点
        Point startPoint(i, 0, 0);
        Point endPoint(i, 10, 0);
        Fiber fiber(startPoint, endPoint);

        // 测试push cutter
        Interval interval;
        bool hit = cutter.pushCutter(fiber, interval, triangle);

        // 前面的fiber应该与三角形相交，后面的可能不相交
        if (i < 10) {  // 在三角形内部或边界上的fiber
            EXPECT_TRUE(hit) << "Fiber at x=" << i << " should hit the triangle";
            EXPECT_FALSE(interval.empty()) << "Fiber at x=" << i << " should have interval";

            // 检查接触范围
            if (i == 0) {
                // x=0时，fiber应该从p1到p3完全在三角形上
                EXPECT_NEAR(interval.lower, 0.0, 1e-6);
                EXPECT_NEAR(interval.upper, 1.0, 1e-6);

                // 验证接触点坐标
                Point contactPoint1 = fiber.point(interval.lower);
                Point contactPoint2 = fiber.point(interval.upper);
                EXPECT_NEAR(contactPoint1.x, p1.x, 1e-6);
                EXPECT_NEAR(contactPoint1.y, p1.y, 1e-6);
                EXPECT_NEAR(contactPoint1.z, p1.z, 1e-6);
                EXPECT_NEAR(contactPoint2.x, p3.x, 1e-6);
                EXPECT_NEAR(contactPoint2.y, p3.y, 1e-6);
                EXPECT_NEAR(contactPoint2.z, p3.z, 1e-6);
            }

            else {
                // 对于其他情况，fiber与三角形相交一段线段
                // 相交的y范围应为：[0, 10-i]
                Point contactPoint1 = fiber.point(interval.lower);
                Point contactPoint2 = fiber.point(interval.upper);
                EXPECT_NEAR(contactPoint1.y, 0.0, 1e-6);
                EXPECT_NEAR(contactPoint2.y, 10.0 - i, 1e-6);
            }
        }
        else if (i == 10) {
            // x=10时，fiber只与三角形在p2处相交
            EXPECT_TRUE(hit) << "Fiber at x=" << i << " should hit the triangle";
            EXPECT_TRUE(interval.empty()) << "Fiber at x=" << i << " should not have interval";
        }
        else {  // x=11时，fiber应该完全在三角形外
            // 注意：由于刀具有半径，即使fiber轴在三角形外，
            // 也可能有部分刀具与三角形接触
            if (!hit) {
                EXPECT_TRUE(interval.empty());
            }
            else {
                // 如果检测到碰撞，接触点应该在三角形边缘附近
                Point contactPoint = fiber.point(interval.upper);
                double distToEdge = std::min(
                    (contactPoint - p2).norm(),
                    std::min((contactPoint - p3).norm(), (contactPoint - 0.5 * (p2 + p3)).norm()));
                EXPECT_LE(distToEdge, radius + 1e-6);
            }
        }
    }
}

TEST(PushCutterTests, VerticalTriangleIntersect)
{
    // 两个相隔10的竖直三角形
    Point p1(0, 0, 0);
    Point p2(0, 0, 10);
    Point p3(10, 0, 0);
    Triangle triangle1(p1, p2, p3);
    // Y方向平移10，创建第二个三角形
    Point p4(p1.x, p1.y + 10, p1.z);
    Point p5(p3.x, p3.y + 10, p3.z);
    Point p6(p2.x, p2.y + 10, p2.z);
    Triangle triangle2(p4, p5, p6);

    // 创建圆柱形铣刀
    double diameter = 6.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    CylCutter cutter(diameter, length);

    // 创建多个fiber的网格，用于测试与三角形相交的情况
    const int minCoord = -1;
    const int maxCoord = 11;

    // 先测试原始的基准用例
    {
        // 创建一根fiber，与三角形相交 (y方向与三角形垂直)
        // fiber的长度为20，从y=-5到y=15
        Point startPoint(0, -5, 0);
        Point endPoint(0, 15, 0);
        Fiber fiber(startPoint, endPoint);

        // 测试push cutter, 先测试triangle1
        Interval interval;
        bool hit1 = cutter.pushCutter(fiber, interval, triangle1);

        // 应该检测到碰撞
        EXPECT_TRUE(hit1);
        EXPECT_FALSE(interval.empty());

        // 验证interval (允许一定误差)
        double expected_lower = (5 - radius) / (fiber.p2.y - fiber.p1.y);
        EXPECT_NEAR(interval.lower, expected_lower, 1e-6);
        double expected_upper = (5 + radius) / (fiber.p2.y - fiber.p1.y);
        EXPECT_NEAR(interval.upper, expected_upper, 1e-6);

        // 检查接触点坐标是否合理
        Point contactPoint = fiber.point(interval.upper);
        EXPECT_NEAR(contactPoint.y, radius, 1e-6);

        // 验证接触位置是否在三角形的合理范围内
        EXPECT_GE(contactPoint.x, 0.0);
        EXPECT_LE(contactPoint.x, 10.0);
        EXPECT_GE(contactPoint.z, 0.0);
        EXPECT_LE(contactPoint.z, 10.0);

        // 测试push cutter, 再测试triangle2
        bool hit2 = cutter.pushCutter(fiber, interval, triangle2);
        EXPECT_TRUE(hit2);
        EXPECT_FALSE(interval.empty());
        // expected_lower 不变
        EXPECT_NEAR(interval.lower, expected_lower, 1e-6);
        expected_upper = (15 + radius) / (fiber.p2.y - fiber.p1.y);
        EXPECT_NEAR(interval.upper, expected_upper, 1e-6);
    }

    // 创建x-z平面的fiber网格，遍历不同x和z坐标
    // 这里会连续pushCutter两次之后再检查
    for (int x = minCoord; x <= maxCoord; x += 2) {      // 间隔2减少测试数量
        for (int z = minCoord; z <= maxCoord; z += 2) {  // 间隔2减少测试数量
            // 创建从y=-5到y=15的fiber
            Point startPoint(x, -5, z);
            Point endPoint(x, 15, z);
            Fiber fiber(startPoint, endPoint);

            std::string fiberDesc = "Fiber at x=" + std::to_string(x) + ", z=" + std::to_string(z);

            // fiber在三角形1和三角形2的范围内
            if (x >= 0 && x <= 10 && z >= 0 && z <= 10 && (x / 10.0 + z / 10.0 <= 1.0)) {
                Interval interval;
                bool hit1 = cutter.pushCutter(fiber, interval, triangle1);
                bool hit2 = cutter.pushCutter(fiber, interval, triangle2);

                EXPECT_TRUE(hit1) << fiberDesc << " should hit triangle1";
                EXPECT_FALSE(interval.empty())
                    << fiberDesc << " should have interval with triangle1";
                EXPECT_TRUE(hit2) << fiberDesc << " should hit triangle2";
                EXPECT_FALSE(interval.empty())
                    << fiberDesc << " should have interval with triangle2";

                // Lower
                double expected_lower = (5 - radius) / (fiber.p2.y - fiber.p1.y);
                EXPECT_NEAR(interval.lower, expected_lower, 1e-6);

                // Upper
                double expected_upper = (15 + radius) / (fiber.p2.y - fiber.p1.y);
                EXPECT_NEAR(interval.upper, expected_upper, 1e-6);

                // Lower CCpoint, must be on the triangle1
                auto lowerPoint = interval.lower_cc;
                EXPECT_NEAR(lowerPoint.x, x, 1e-6);
                EXPECT_NEAR(lowerPoint.y, 0.0, 1e-6);

                // Upper CCpoint, must be on the triangle2
                auto upperPoint = interval.upper_cc;
                EXPECT_NEAR(upperPoint.x, x, 1e-6);
                EXPECT_NEAR(upperPoint.y, 10.0, 1e-6);
            }
        }
    }
}

TEST(PushCutterTests, VerticalTriangleParallel)
{
    // 测试竖直放置的三角形，且fiber与三角形平行
    Point p1(0, 0, 0);
    Point p2(0, 0, 10);
    Point p3(10, 0, 0);
    Triangle triangle(p1, p2, p3);

    // 创建球形铣刀
    double diameter = 6.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    CylCutter cutter(diameter, length);

    // 创建很多根fiber，与三角形平行 (沿着X轴方向)
    // 从 y = -4 到 y = 4
    for (int y = -4; y <= 4; y += 1) {
        Point startPoint(-5, y, 0);
        Point endPoint(15, y, 0);
        Fiber fiber(startPoint, endPoint);

        // 测试push cutter
        Interval interval;
        bool hit = cutter.pushCutter(fiber, interval, triangle);

        if (y == -4 || y == 4) {
            // Far away from the triangle
            EXPECT_FALSE(hit);
            EXPECT_TRUE(interval.empty());
        }
        else {
            // 应该检测到碰撞
            EXPECT_TRUE(hit);
            // 检查interval不为空
            EXPECT_FALSE(interval.empty());

            EXPECT_GT(interval.lower, 0.0);
            EXPECT_LT(interval.upper, 1.0);

            // lower_cc 始终是P1
            // upper_cc 始终是P3
            EXPECT_NEAR(interval.lower_cc.x, p1.x, 1e-6);
            EXPECT_NEAR(interval.lower_cc.y, p1.y, 1e-6);
            EXPECT_NEAR(interval.lower_cc.z, p1.z, 1e-6);
            EXPECT_NEAR(interval.upper_cc.x, p3.x, 1e-6);
            EXPECT_NEAR(interval.upper_cc.y, p3.y, 1e-6);
            EXPECT_NEAR(interval.upper_cc.z, p3.z, 1e-6);
        }
    }
}