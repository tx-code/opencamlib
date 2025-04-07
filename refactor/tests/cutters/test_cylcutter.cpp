#include <gtest/gtest.h>

#include "cutters/cylcutter.hpp"
#include "geo/clpoint.hpp"
#include "geo/stlsurf.hpp"
#include "geo/triangle.hpp"
#include "../utils/triangles_utils.h"


using namespace ocl;

TEST(CuttersTests, CylindricalCutterProperties)
{
    // 测试圆柱形铣刀的基本属性
    double diameter = 10.0;
    double length = 20.0;
    CylCutter cutter(diameter, length);

    // 验证基本属性值是否正确
    EXPECT_DOUBLE_EQ(cutter.getDiameter(), diameter);
    EXPECT_DOUBLE_EQ(cutter.getRadius(), diameter / 2);
    EXPECT_DOUBLE_EQ(cutter.getLength(), length);
}

TEST(CuttersTests, CylindricalCutterHorizontalTriangle)
{
    // 创建一个水平三角形用于测试
    Point p1(0, 0, 0);
    Point p2(10, 0, 0);
    Point p3(0, 10, 0);
    Triangle triangle(p1, p2, p3);

    // 创建圆柱形铣刀
    double diameter = 6.0;
    double length = 20.0;
    CylCutter cutter(diameter, length);

    // 测试铣刀在不同位置的接触情况

    // 位置1: 铣刀正在三角形上方，应该在z=0处与平面接触
    CLPoint cl1(5, 5, -10);
    bool hit1 = cutter.dropCutter(cl1, triangle);
    EXPECT_TRUE(hit1);                   // 应该检测到碰撞
    EXPECT_DOUBLE_EQ(cl1.z, 0.0);        // 底部接触平面z=0
    EXPECT_EQ(cl1.getCC().type, EDGE);  // 接触类型应为EDGE

    // 位置2: 铣刀在三角形外但仍在其半径范围内，仍应该在z=0处接触
    CLPoint cl2(-1, -1, -10);
    bool hit2 = cutter.dropCutter(cl2, triangle);
    EXPECT_TRUE(hit2);                   // 应该检测到碰撞
    EXPECT_DOUBLE_EQ(cl2.z, 0.0);        // 底部接触平面z=0
    EXPECT_EQ(cl2.getCC().type, VERTEX);  // 接触类型应为VERTEX

    // 位置3: 铣刀远离三角形，超出半径范围，不应该有碰撞
    CLPoint cl3(-10, -10, -10);
    bool hit3 = cutter.dropCutter(cl3, triangle);
    EXPECT_FALSE(hit3);                 // 应该没有碰撞
    EXPECT_DOUBLE_EQ(cl3.z, -10.0);     // 位置不变，仍为初始z值
    EXPECT_EQ(cl3.getCC().type, NONE);  // 没有接触点
}

TEST(CuttersTests, CylindricalCutterVericalTriangle)
{
    // 测试圆柱铣刀与垂直三角形的接触情况
    Point p1(0, 0, 0);
    Point p2(0, 0, 10);  // 垂直方向上升
    Point p3(10, 0, 0);
    Triangle triangle(p1, p2, p3);

    // 创建铣刀
    double diameter = 6.0;
    double length = 20.0;
    CylCutter cutter(diameter, length);

    // 测试铣刀的垂直面接触
    CLPoint cl(5, 3, -10);
    bool hit = cutter.dropCutter(cl, triangle);
    EXPECT_TRUE(hit);  // 应该检测到碰撞
    EXPECT_DOUBLE_EQ(cl.z, 5.0);

    // 检查接触点类型，应该是边缘接触
    EXPECT_EQ(cl.getCC().type, EDGE);  // 接触类型应为EDGE
}

TEST(CuttersTests, CylindricalCutterCubeModel)
{
    // 创建一个简单的立方体模型，范围从(0,0,0)到(10,10,10)
    STLSurf cube;

    // 添加底面 (z=0)
    cube.addTriangle(Triangle(Point(0, 0, 0), Point(10, 0, 0), Point(0, 10, 0)));
    cube.addTriangle(Triangle(Point(10, 10, 0), Point(10, 0, 0), Point(0, 10, 0)));

    // 添加顶面 (z=10)
    cube.addTriangle(Triangle(Point(0, 0, 10), Point(0, 10, 10), Point(10, 0, 10)));
    cube.addTriangle(Triangle(Point(10, 10, 10), Point(0, 10, 10), Point(10, 0, 10)));

    // 添加前面 (y=0)
    cube.addTriangle(Triangle(Point(0, 0, 0), Point(0, 0, 10), Point(10, 0, 0)));
    cube.addTriangle(Triangle(Point(10, 0, 10), Point(0, 0, 10), Point(10, 0, 0)));

    // 添加后面 (y=10)
    cube.addTriangle(Triangle(Point(0, 10, 0), Point(10, 10, 0), Point(0, 10, 10)));
    cube.addTriangle(Triangle(Point(10, 10, 10), Point(0, 10, 10), Point(10, 10, 0)));

    // 添加左面 (x=0)
    cube.addTriangle(Triangle(Point(0, 0, 0), Point(0, 10, 0), Point(0, 0, 10)));
    cube.addTriangle(Triangle(Point(0, 10, 10), Point(0, 0, 10), Point(0, 10, 0)));

    // 添加右面 (x=10)
    cube.addTriangle(Triangle(Point(10, 0, 0), Point(10, 0, 10), Point(10, 10, 0)));
    cube.addTriangle(Triangle(Point(10, 10, 10), Point(10, 0, 10), Point(10, 10, 0)));

    // 创建圆柱铣刀
    double diameter = 6.0;
    double length = 20.0;
    CylCutter cutter(diameter, length);

    // 在不同位置测试铣刀

    // 1. 立方体中心上方
    CLPoint cl1(5, 5, -20);
    bool hit1 = cutter.dropCutterSTL(cl1, cube);
    EXPECT_TRUE(hit1);                   // 应该有碰撞
    EXPECT_DOUBLE_EQ(cl1.z, 10.0);       // 铣刀底部应该接触顶面z=10
    EXPECT_EQ(cl1.getCC().type, EDGE);  // 接触类型应为EDGE

    // 2. 立方体边缘上方
    CLPoint cl2(0, 5, -20);
    bool hit2 = cutter.dropCutterSTL(cl2, cube);
    EXPECT_TRUE(hit2);                   // 应该有碰撞
    EXPECT_DOUBLE_EQ(cl2.z, 10.0);       // 铣刀底部应该接触顶面z=10
    EXPECT_EQ(cl2.getCC().type, EDGE);  // 接触类型应为EDGE

    // 3. 在立方体外部，超出铣刀半径范围
    CLPoint cl3(-10, -10, -20);
    bool hit3 = cutter.dropCutterSTL(cl3, cube);
    EXPECT_FALSE(hit3);                 // 应该没有碰撞
    EXPECT_DOUBLE_EQ(cl3.z, -20.0);     // 位置不变，仍为初始z值
    EXPECT_EQ(cl3.getCC().type, NONE);  // 没有接触点
}

TEST(CuttersTests, CylindricalCutterRandomPoints)
{
    // 创建一个非水平三角形用于测试
    Point p1(0, 0, 0);
    Point p2(10, 0, 5);
    Point p3(0, 10, 8);
    Triangle triangle(p1, p2, p3);

    // 创建圆柱形铣刀
    double diameter = 6.0;
    double length = 20.0;
    CylCutter cutter(diameter, length);

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