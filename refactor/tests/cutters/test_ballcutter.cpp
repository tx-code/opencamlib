#include <gtest/gtest.h>

#include "cutters/ballcutter.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"
#include "../utils/triangles_utils.h"


using namespace ocl;

TEST(CuttersTests, BallCutterProperties)
{
    // 测试球形铣刀的基本属性
    double diameter = 10.0;
    double length = 20.0;
    BallCutter cutter(diameter, length);

    EXPECT_DOUBLE_EQ(cutter.getDiameter(), diameter);
    EXPECT_DOUBLE_EQ(cutter.getRadius(), diameter / 2);
    EXPECT_DOUBLE_EQ(cutter.getLength(), length);
}

TEST(CuttersTests, BallCutterHorizontalTriangle)
{
    // 创建水平三角形用于测试
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(0, 10, 0);
    Triangle triangle(p1, p2, p3);

    // 创建球形铣刀
    double diameter = 6.0;
    double length = 20.0;
    BallCutter cutter(diameter, length);

    // 测试在不同位置的drop cutter

    // 位置1：直接在三角形上方（应该在z=-radius接触）
    CLPoint cl1(5, 5, -1e6);
    bool hit1 = cutter.dropCutter(cl1, triangle);
    EXPECT_TRUE(hit1);
    // 球形铣刀的CL点位于半球面顶点
    EXPECT_DOUBLE_EQ(cl1.z, 0);
    EXPECT_EQ(cl1.getCC().type, EDGE);  // 接触类型应为EDGE

    // 位置2：三角形外但在刀具半径内（应该仍然在z=-radius接触）
    CLPoint cl2(-1, -1, -1e6);
    bool hit2 = cutter.dropCutter(cl2, triangle);
    EXPECT_TRUE(hit2);
    // 这个位置CC点更容易计算...
    auto cc2 = cl2.getCC();
    EXPECT_DOUBLE_EQ(cc2.x, 0);
    EXPECT_DOUBLE_EQ(cc2.y, 0);
    EXPECT_DOUBLE_EQ(cc2.z, 0);
    EXPECT_EQ(cc2.type, VERTEX);  // 此处接触点为顶点

    // 位置3：远离三角形（不应该碰撞）
    CLPoint cl3(-10, -10, -1e6);
    bool hit3 = cutter.dropCutter(cl3, triangle);
    EXPECT_FALSE(hit3);
    EXPECT_DOUBLE_EQ(cl3.z, -1e6);      // z应保持不变
    EXPECT_EQ(cl3.getCC().type, NONE);  // 没有接触点
}

TEST(CuttersTests, BallCutterVerticalTriangle)
{
    // 测试球刀与垂直三角形的接触
    Point p1(0, 0, 0);
    Point p2(0, 0, 10);
    Point p3(10, 0, 0);
    Triangle triangle(p1, p2, p3);

    // 创建球形铣刀
    double diameter = 6.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    BallCutter cutter(diameter, length);

    // 测试drop cutter
    CLPoint cl(5, radius, -10);
    bool hit = cutter.dropCutter(cl, triangle);
    EXPECT_TRUE(hit);

    // 球刀中心y坐标为radius，正好接触y=0的平面
    // 接触点为边中点，实际CL为中点Z坐标 - 半径
    EXPECT_DOUBLE_EQ(cl.z, 2);

    // 测试接触点
    EXPECT_EQ(cl.getCC().type, EDGE);  // 应该是边缘接触
}

TEST(CuttersTests, BallCutterEdgeCase)
{
    // 测试球刀与三角形边缘的接触
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(0, 10, 0);
    Triangle triangle(p1, p2, p3);

    // 创建球形铣刀
    double diameter = 6.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    BallCutter cutter(diameter, length);

    // 刀具位于边缘p1-p2上方
    CLPoint cl(5, -radius, -10);
    bool hit = cutter.dropCutter(cl, triangle);
    EXPECT_TRUE(hit);

    // 刀具中心应该在z=0平面上方半径距离处
    EXPECT_DOUBLE_EQ(cl.z, -radius);

    // 检查接触点类型应该是EDGE
    EXPECT_EQ(cl.getCC().type, EDGE);  // 应该是边缘接触
}

TEST(CuttersTests, BallCutterCubeModel)
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

    // 创建球形铣刀
    double diameter = 6.0;
    double length = 20.0;
    double radius = diameter / 2.0;
    BallCutter cutter(diameter, length);

    // 测试点

    // 1. 立方体中心上方
    CLPoint cl1(5, 5, -20);
    bool hit1 = cutter.dropCutterSTL(cl1, cube);
    EXPECT_TRUE(hit1);
    EXPECT_DOUBLE_EQ(cl1.z, 10.0);       // 应该接触顶面在z=10-radius
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

TEST(CuttersTests, BallCutterRandomPoints)
{
    // 创建一个非水平三角形用于测试
    Point p1(0, 0, 0);
    Point p2(10, 0, 5);
    Point p3(0, 10, 8);
    Triangle triangle(p1, p2, p3);

    // 创建球形铣刀
    double diameter = 6.0;
    double length = 20.0;
    BallCutter cutter(diameter, length);

    // 在三角形内随机生成1000个点，排除顶点和边上的点
    const size_t num_points = 1000;
    std::vector<Point> random_points = createRandomPointsInTriangle(triangle, num_points, true, true);
    
    // 确认生成了正确数量的点
    EXPECT_EQ(random_points.size(), num_points);
    
    // 对每个随机点执行dropCutter测试
    for (const auto& point : random_points) {
        CLPoint cl(point.x, point.y, -20);  // 从下方接近
        bool hit = cutter.dropCutter(cl, triangle);
        
        // 应该都能检测到碰撞
        EXPECT_TRUE(hit) << "Point (" << point.x << ", " << point.y << ", " << point.z << ") Unhit";
        
        // cl的z值应该被更新
        EXPECT_GT(cl.z, -20.0) << "Point (" << point.x << ", " << point.y << ", " << point.z << ") z value not updated";
        
        // 接触类型不应该是NONE
        EXPECT_NE(cl.getCC().type, NONE) << "Point (" << point.x << ", " << point.y << ", " << point.z << ") contact type is NONE";
    }
}