#include <boost/math/constants/constants.hpp>
#include <gtest/gtest.h>


#include "cutters/conecutter.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"
#include "../utils/triangles_utils.h"

using namespace ocl;

TEST(CuttersTests, ConeCutterProperties)
{
    // 测试锥形铣刀的基本属性
    double diameter = 10.0;
    double angle = boost::math::constants::pi<double>() / 4;  // 45度（90度的半角）
    double length = 20.0;
    ConeCutter cutter(diameter, angle, length);

    EXPECT_DOUBLE_EQ(cutter.getDiameter(), diameter);
    EXPECT_DOUBLE_EQ(cutter.getRadius(), diameter / 2);
    EXPECT_DOUBLE_EQ(cutter.getAngle(), angle);
    EXPECT_DOUBLE_EQ(cutter.getLength(), length + diameter / 2.0);
}

TEST(CuttersTests, ConeCutterHorizontalTriangle)
{
    // 创建水平三角形用于测试
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(0, 10, 0);
    Triangle triangle(p1, p2, p3);

    // 创建锥形铣刀
    double diameter = 6.0;
    double angle = boost::math::constants::pi<double>() / 4;  // 45度（90度的半角）
    double length = 20.0;
    ConeCutter cutter(diameter, angle, length);

    // 测试在不同位置的drop cutter

    // 位置1：直接在三角形上方（应该接触于尖端）
    CLPoint cl1(5, 5, -1);
    bool hit1 = cutter.dropCutter(cl1, triangle);
    EXPECT_TRUE(hit1);
    EXPECT_DOUBLE_EQ(cl1.z, 0.0);            // 尖端接触平面
    EXPECT_EQ(cl1.getCC().type, EDGE);

    // 位置2：三角形外但在刀具半径内（接触可能是在圆周边缘）
    CLPoint cl2(-1, -1, -10);
    bool hit2 = cutter.dropCutter(cl2, triangle);
    EXPECT_TRUE(hit2);
    EXPECT_TRUE(cl2.getCC().type == VERTEX
                || cl2.getCC().type == EDGE_CONE);  // 接触类型取决于具体位置

    // 位置3：远离三角形（不应该碰撞）
    CLPoint cl3(-10, -10, -1);
    bool hit3 = cutter.dropCutter(cl3, triangle);
    EXPECT_FALSE(hit3);
    EXPECT_DOUBLE_EQ(cl3.z, -1);        // z应保持不变
    EXPECT_EQ(cl3.getCC().type, NONE);  // 没有接触点
}

TEST(CuttersTests, ConeCutterVerticalTriangle)
{
    // 测试锥形铣刀与垂直三角形的接触
    Point p1(0, 0, 0);
    Point p2(0, 0, 10);
    Point p3(10, 0, 0);
    Triangle triangle(p1, p2, p3);

    // 创建锥形铣刀
    double diameter = 6.0;
    double angle = boost::math::constants::pi<double>() / 4;  // 45度（90度的半角）
    double length = 20.0;
    double radius = diameter / 2.0;
    ConeCutter cutter(diameter, angle, length);

    // 测试drop cutter
    CLPoint cl(5, radius, -1);
    bool hit = cutter.dropCutter(cl, triangle);
    EXPECT_TRUE(hit);
    EXPECT_DOUBLE_EQ(cl.z, 2);

    // 测试接触点
    EXPECT_EQ(cl.getCC().type, EDGE);  // 与边缘接触
}

TEST(CuttersTests, ConeCutterTipCase)
{
    // 测试锥刀尖端与三角形的接触
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(5, 5, 5);
    Triangle triangle(p1, p2, p3);

    // 创建锥形铣刀
    double diameter = 6.0;
    double angle = boost::math::constants::pi<double>() / 4;  // 45度（90度的半角）
    double length = 20.0;
    ConeCutter cutter(diameter, angle, length);

    // 刀具正好在顶点p3上方
    CLPoint cl(5, 5, -10);
    bool hit = cutter.dropCutter(cl, triangle);
    EXPECT_TRUE(hit);

    // 刀具尖端应接触p3点
    EXPECT_DOUBLE_EQ(cl.z, 5.0);

    // 检查接触点类型应该是VERTEX
    EXPECT_EQ(cl.getCC().type, VERTEX);  // 与顶点接触
}

TEST(CuttersTests, ConeCutterCubeModel)
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

    // 创建锥形铣刀
    double diameter = 6.0;
    double angle = boost::math::constants::pi<double>() / 4;  // 45度（90度的半角）
    double length = 20.0;
    ConeCutter cutter(diameter, angle, length);

    // 测试点

    // 1. 立方体中心上方
    CLPoint cl1(5, 5, -20);
    bool hit1 = cutter.dropCutterSTL(cl1, cube);
    EXPECT_TRUE(hit1);
    EXPECT_DOUBLE_EQ(cl1.z, 10.0);           // 应该尖端接触顶面在z=10
    EXPECT_EQ(cl1.getCC().type, EDGE);  // 接触类型应为FACET_TIP

    // 2. 立方体顶点上方
    CLPoint cl2(0, 0, -20);
    bool hit2 = cutter.dropCutterSTL(cl2, cube);
    EXPECT_TRUE(hit2);
    EXPECT_DOUBLE_EQ(cl2.z, 10.0);        // 应该尖端接触顶点
    EXPECT_EQ(cl2.getCC().type, VERTEX);  // 接触类型应为VERTEX

    // 3. 立方体外部
    CLPoint cl3(-10, -10, -20);
    bool hit3 = cutter.dropCutterSTL(cl3, cube);
    EXPECT_FALSE(hit3);
    EXPECT_DOUBLE_EQ(cl3.z, -20.0);     // 不变
    EXPECT_EQ(cl3.getCC().type, NONE);  // 没有接触点
}

TEST(CuttersTests, ConeCutterRandomPoints)
{
    // 创建一个非水平三角形用于测试
    Point p1(0, 0, 0);
    Point p2(10, 0, 5);
    Point p3(0, 10, 8);
    Triangle triangle(p1, p2, p3);

    // 创建锥形铣刀
    double diameter = 6.0;
    double angle = boost::math::constants::pi<double>() / 4;  // 45度（90度的半角）
    double length = 20.0;
    ConeCutter cutter(diameter, angle, length);

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
        EXPECT_TRUE(hit) << "Point (" << point.x << ", " << point.y << ", " << point.z << ") not hit";
        
        // cl的z值应该被更新
        EXPECT_GT(cl.z, -20.0) << "Point (" << point.x << ", " << point.y << ", " << point.z << ") z value not updated";
        
        // 接触类型不应该是NONE
        EXPECT_NE(cl.getCC().type, NONE) << "Point (" << point.x << ", " << point.y << ", " << point.z << ") contact type is NONE";
    }
}