#include <gtest/gtest.h>

#include "cutters/bullcutter.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"


using namespace ocl;

TEST(CuttersTests, BullCutterProperties)
{
    // 测试牛鼻刀的基本属性
    double diameter = 10.0;
    double corner_radius = 2.0;
    double length = 20.0;
    BullCutter cutter(diameter, corner_radius, length);

    EXPECT_DOUBLE_EQ(cutter.getDiameter(), diameter);
    EXPECT_DOUBLE_EQ(cutter.getRadius(), diameter / 2);
    EXPECT_DOUBLE_EQ(cutter.getRadius2(), corner_radius);
    EXPECT_DOUBLE_EQ(cutter.getLength(), length);
}

TEST(CuttersTests, BullCutterHorizontalTriangle)
{
    // 创建水平三角形用于测试
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(0, 10, 0);
    Triangle triangle(p1, p2, p3);

    // 创建牛鼻刀
    double diameter = 6.0;
    double corner_radius = 1.0;
    double length = 20.0;
    BullCutter cutter(diameter, corner_radius, length);

    // 测试在不同位置的drop cutter

    // 位置1：直接在三角形上方
    CLPoint cl1(5, 5, -10);
    bool hit1 = cutter.dropCutter(cl1, triangle);
    EXPECT_TRUE(hit1);
    EXPECT_DOUBLE_EQ(cl1.z, 0);          // 底部圆盘接触
    EXPECT_EQ(cl1.getCC().type, EDGE);  // 接触类型应为EDGE

    // 位置2：三角形外但在刀具半径内
    CLPoint cl2(-1, -1, -10);
    bool hit2 = cutter.dropCutter(cl2, triangle);
    EXPECT_TRUE(hit2);
    EXPECT_DOUBLE_EQ(cl2.z, 0);
    // 接触类型取决于接触点的位置，可能是VERTEX或FACET
    auto cc2 = cl2.getCC();
    EXPECT_TRUE(cc2.type == VERTEX || cc2.type == FACET);

    // 位置3：远离三角形（不应该碰撞）
    CLPoint cl3(-10, -10, -10);
    bool hit3 = cutter.dropCutter(cl3, triangle);
    EXPECT_FALSE(hit3);
    EXPECT_DOUBLE_EQ(cl3.z, -10.0);     // z应保持不变
    EXPECT_EQ(cl3.getCC().type, NONE);  // 没有接触点
}

TEST(CuttersTests, BullCutterVerticalTriangle)
{
    // 测试牛鼻刀与垂直三角形的接触
    Point p1(0, 0, 0);
    Point p2(0, 0, 10);
    Point p3(10, 0, 0);
    Triangle triangle(p1, p2, p3);

    // 创建牛鼻刀
    double diameter = 6.0;
    double corner_radius = 1.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    BullCutter cutter(diameter, corner_radius, length);

    // 测试drop cutter - 平行于垂直三角形放置
    CLPoint cl(5, radius, -10);
    bool hit = cutter.dropCutter(cl, triangle);
    EXPECT_TRUE(hit);

    EXPECT_DOUBLE_EQ(cl.z, 5 - corner_radius);

    // 测试接触点
    EXPECT_EQ(cl.getCC().type, EDGE);  // 应该是边缘接触
}

TEST(CuttersTests, BullCutterEdgeCase)
{
    // 测试牛鼻刀与边缘的接触
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(0, 10, 0);
    Triangle triangle(p1, p2, p3);

    // 创建牛鼻刀
    double diameter = 6.0;
    double corner_radius = 1.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    BullCutter cutter(diameter, corner_radius, length);

    // 刀具位于边缘p1-p2上方
    CLPoint cl(5, -(radius - corner_radius), -10);
    bool hit = cutter.dropCutter(cl, triangle);
    EXPECT_TRUE(hit);

    // 刀具此时应接触边缘，高度应该是0
    EXPECT_NEAR(cl.z, 0.0, 1e-10);

    // 检查接触点类型是否为EDGE
    // 注意：根据接触点的具体位置，类型可能是EDGE或EDGE_SHAFT
    // 这里假设接触发生在圆角部分
    EXPECT_TRUE(cl.getCC().type == EDGE || cl.getCC().type == EDGE_SHAFT);  // 应该是边缘接触
}

TEST(CuttersTests, BullCutterCubeModel)
{
    // 创建简单立方体模型 (0,0,0) 到 (10,10,10)
    STLSurf cube;

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

    // 创建牛鼻刀
    double diameter = 6.0;
    double corner_radius = 1.0;
    double length = 20.0;
    BullCutter cutter(diameter, corner_radius, length);

    // 测试点

    // 1. 立方体中心上方
    CLPoint cl1(5, 5, -20);
    bool hit1 = cutter.dropCutterSTL(cl1, cube);
    EXPECT_TRUE(hit1);
    EXPECT_DOUBLE_EQ(cl1.z, 10.0);
    EXPECT_EQ(cl1.getCC().type, EDGE);  // 接触类型应为EDGE

    // 2. 立方体边缘上方
    CLPoint cl2(0, 5, -20);
    bool hit2 = cutter.dropCutterSTL(cl2, cube);
    EXPECT_TRUE(hit2);
    // 应该接触顶面，但因为是边缘，所以高度可能会受到影响
    EXPECT_LE(cl2.z, 10.0);
    EXPECT_TRUE(cl2.getCC().type == FACET || cl2.getCC().type == EDGE);  // 接触类型应为FACET或EDGE

    // 3. 立方体外部
    CLPoint cl3(-10, -10, -20);
    bool hit3 = cutter.dropCutterSTL(cl3, cube);
    EXPECT_FALSE(hit3);
    EXPECT_DOUBLE_EQ(cl3.z, -20.0);     // 不变
    EXPECT_EQ(cl3.getCC().type, NONE);  // 没有接触点
}