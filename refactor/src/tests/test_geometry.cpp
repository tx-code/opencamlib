#define BOOST_TEST_MODULE GeometryTests
#include <boost/test/unit_test.hpp>

#include <iomanip>
#include <iostream>
#include <vector>

#include "geo/arc.hpp"
#include "geo/geometry.hpp"
#include "geo/line.hpp"
#include "geo/mesh.hpp"
#include "geo/path.hpp"

using namespace ocl;

// 测试点和向量
BOOST_AUTO_TEST_CASE(TestPoint) {
  Point p1(1, 2, 3);
  Point p2(4, 5, 6);

  // 测试基本运算
  Point sum = p1 + p2;
  BOOST_CHECK_CLOSE(sum.x(), 5, 1e-8);
  BOOST_CHECK_CLOSE(sum.y(), 7, 1e-8);
  BOOST_CHECK_CLOSE(sum.z(), 9, 1e-8);

  Point diff = p2 - p1;
  BOOST_CHECK_CLOSE(diff.x(), 3, 1e-8);
  BOOST_CHECK_CLOSE(diff.y(), 3, 1e-8);
  BOOST_CHECK_CLOSE(diff.z(), 3, 1e-8);

  // 测试点积
  double dot = p1.dot(p2);
  BOOST_CHECK_CLOSE(dot, 1 * 4 + 2 * 5 + 3 * 6, 1e-8);

  // 测试叉积
  Vector3 cross = p1.cross(p2);
  BOOST_CHECK_CLOSE(cross.x(), 2 * 6 - 3 * 5, 1e-8);
  BOOST_CHECK_CLOSE(cross.y(), 3 * 4 - 1 * 6, 1e-8);
  BOOST_CHECK_CLOSE(cross.z(), 1 * 5 - 2 * 4, 1e-8);

  // 测试范数
  double norm = p1.norm();
  BOOST_CHECK_CLOSE(norm, std::sqrt(1 * 1 + 2 * 2 + 3 * 3), 1e-8);
}

// 测试线段
BOOST_AUTO_TEST_CASE(TestLine) {
  Point p1(0, 0, 0);
  Point p2(10, 0, 0);
  Line line(p1, p2);

  // 测试长度
  BOOST_CHECK_CLOSE(line.length(), 10, 1e-8);

  // 测试方向
  Vector3 dir = line.direction();
  BOOST_CHECK_CLOSE(dir.x(), 1, 1e-8);
  BOOST_CHECK_SMALL(dir.y(), 1e-8);
  BOOST_CHECK_SMALL(dir.z(), 1e-8);

  // 测试点到线段的距离
  Point p3(5, 1, 0);
  BOOST_CHECK_CLOSE(line.distanceTo(p3), 1, 1e-8);

  Point p4(5, 0, 0);
  BOOST_CHECK_SMALL(line.distanceTo(p4), 1e-8);

  Point p5(-1, 0, 0);
  BOOST_CHECK_CLOSE(line.distanceTo(p5), 1, 1e-8);

  Point p6(11, 0, 0);
  BOOST_CHECK_CLOSE(line.distanceTo(p6), 1, 1e-8);

  // 测试线段上的点
  Point pt = line.pointAt(0.5);
  BOOST_CHECK_CLOSE(pt.x(), 5, 1e-8);
  BOOST_CHECK_SMALL(pt.y(), 1e-8);
  BOOST_CHECK_SMALL(pt.z(), 1e-8);

  // 测试射线相交
  Ray ray(Point(5, 0, 0), Vector3(0, -1, 0));
  auto intersection = line.intersectWith(ray);
  BOOST_REQUIRE(intersection.has_value());
  BOOST_CHECK_CLOSE(intersection->x(), 5, 1e-8);
  BOOST_CHECK_SMALL(intersection->y(), 1e-8);
  BOOST_CHECK_SMALL(intersection->z(), 1e-8);

  // 测试不相交的情况
  Ray ray2(Point(5, 5, 0), Vector3(1, 0, 0));
  auto intersection2 = line.intersectWith(ray2);
  BOOST_CHECK(!intersection2.has_value());
}

// 测试圆弧
BOOST_AUTO_TEST_CASE(TestArc) {
  Point center(0, 0, 0);
  double radius = 5;
  double startAngle = 0;
  double endAngle = PI / 2;

  Arc arc(center, radius, startAngle, endAngle);

  // 测试起点和终点
  Point startPoint = arc.startPoint();
  BOOST_CHECK_CLOSE(startPoint.x(), radius, 1e-8);
  BOOST_CHECK_SMALL(startPoint.y(), 1e-8);
  BOOST_CHECK_SMALL(startPoint.z(), 1e-8);

  Point endPoint = arc.endPoint();
  BOOST_CHECK_SMALL(endPoint.x(), 1e-8);
  BOOST_CHECK_CLOSE(endPoint.y(), radius, 1e-8);
  BOOST_CHECK_SMALL(endPoint.z(), 1e-8);

  // 测试长度
  BOOST_CHECK_CLOSE(arc.length(), radius * PI / 2, 1e-8);

  // 测试圆弧上的点
  Point midPoint = arc.pointAt(0.5);
  BOOST_CHECK_CLOSE(midPoint.x(), radius * std::cos(PI / 4), 1e-8);
  BOOST_CHECK_CLOSE(midPoint.y(), radius * std::sin(PI / 4), 1e-8);
  BOOST_CHECK_SMALL(midPoint.z(), 1e-8);

  // 测试点到圆弧的距离
  Point p1(0, 0, 0); // 圆心
  BOOST_CHECK_CLOSE(arc.distanceTo(p1), radius, 1e-8);

  Point p2(radius, 0, 0); // 起点
  BOOST_CHECK_SMALL(arc.distanceTo(p2), 1e-8);

  Point p3(0, radius, 0); // 终点
  BOOST_CHECK_SMALL(arc.distanceTo(p3), 1e-8);

  // 测试圆弧外的点
  Point p4(radius * 2, 0, 0);
  BOOST_CHECK_CLOSE(arc.distanceTo(p4), radius, 1e-8);
}

// 测试路径
BOOST_AUTO_TEST_CASE(TestPath) {
  Path path;

  // 添加线段
  path.addLine(Point(0, 0, 0), Point(10, 0, 0));

  // 添加圆弧
  path.addArc(Point(10, 5, 0), 5, PI, 0);

  // 添加另一条线段
  path.addLine(Point(10, 10, 0), Point(0, 10, 0));

  // 闭合路径
  path.addLine(Point(0, 10, 0), Point(0, 0, 0));

  // 测试路径是否闭合
  BOOST_CHECK(path.isClosed());

  // 测试路径长度
  double expectedLength = 10 + 5 * PI + 10 + 10;
  BOOST_CHECK_CLOSE(path.length(), expectedLength, 1e-8);

  // 测试路径上的点
  Point midPoint = path.pointAt(0.5);

  // 测试点到路径的距离
  Point p1(5, 5, 0); // 路径内部的点
  double dist = path.distanceTo(p1);
  BOOST_CHECK_LT(dist, 5);
}

// 测试三角形
BOOST_AUTO_TEST_CASE(TestTriangle) {
  Point p1(0, 0, 0);
  Point p2(10, 0, 0);
  Point p3(0, 10, 0);

  Triangle triangle(p1, p2, p3);

  // 测试法向量
  Vector3 normal = triangle.normal();
  BOOST_CHECK_SMALL(normal.x(), 1e-8);
  BOOST_CHECK_SMALL(normal.y(), 1e-8);
  BOOST_CHECK_CLOSE(normal.z(), 1, 1e-8);

  // 测试面积
  BOOST_CHECK_CLOSE(triangle.area(), 50, 1e-8);

  // 测试点是否在三角形内
  Point p4(1, 1, 0); // 三角形内部
  BOOST_CHECK(triangle.contains(p4));

  Point p5(5, 6, 0); // 三角形外部
  BOOST_CHECK(!triangle.contains(p5));

  // 测试点到三角形的距离
  BOOST_CHECK_SMALL(triangle.distanceTo(p4), 1e-8);

  Point p6(1, 1, 1); // 三角形上方1个单位
  BOOST_CHECK_CLOSE(triangle.distanceTo(p6), 1, 1e-8);

  // 测试射线相交
  Ray ray(Point(1, 1, 5), Vector3(0, 0, -1));
  auto intersection = triangle.intersectWith(ray);
  BOOST_REQUIRE(intersection.has_value());
  BOOST_CHECK_CLOSE(intersection->x(), 1, 1e-8);
  BOOST_CHECK_CLOSE(intersection->y(), 1, 1e-8);
  BOOST_CHECK_SMALL(intersection->z(), 1e-8);

  // 测试不相交的情况
  Ray ray2(Point(20, 20, 5), Vector3(0, 0, -1));
  auto intersection2 = triangle.intersectWith(ray2);
  BOOST_CHECK(!intersection2.has_value());
}

// 测试网格
BOOST_AUTO_TEST_CASE(TestMesh) {
  // 创建一个简单的立方体网格
  VertexMatrix V(8, 3);
  V << 0, 0, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 1, 1, 0, 1, 1, 1, 1, 0, 1, 1;

  FaceMatrix F(12, 3);
  F << 0, 1, 2,         // 底面
      0, 2, 3, 4, 5, 6, // 顶面
      4, 6, 7, 0, 1, 5, // 侧面
      0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7;

  auto mesh = MeshFactory::createFromData(V, F);

  // 测试三角形数量
  BOOST_CHECK_EQUAL(mesh->triangleCount(), 12);

  // 测试获取三角形
  Triangle tri = mesh->getTriangle(0);
  BOOST_CHECK_SMALL(tri.v0().x(), 1e-8);
  BOOST_CHECK_SMALL(tri.v0().y(), 1e-8);
  BOOST_CHECK_SMALL(tri.v0().z(), 1e-8);

  // 测试点到网格的距离
  Point p1(0.5, 0.5, 0.5); // 立方体内部
  double dist = mesh->distanceTo(p1);
  BOOST_CHECK_CLOSE(dist, 0.5, 1e-8);

  // 测试射线相交
  Ray ray(Point(0.5, 0.5, 2), Vector3(0, 0, -1));
  auto intersection = mesh->intersectWith(ray);
  BOOST_REQUIRE(intersection.has_value());
  BOOST_CHECK_CLOSE(intersection->z(), 1, 1e-8);
}