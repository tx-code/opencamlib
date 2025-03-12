#define BOOST_TEST_MODULE CutterTests
#include <boost/test/unit_test.hpp>

#include "../cutter/cutter_factory.hpp"
#include "../geo/mesh.hpp"

using namespace ocl;

// 测试刀具工厂
BOOST_AUTO_TEST_CASE(test_cutter_factory) {
  // 测试创建圆柱形刀具
  auto cylindrical =
      CutterFactory::createCutter(CutterType::Cylindrical, 10.0, 30.0);
  BOOST_CHECK_EQUAL(cylindrical->getType(), CutterType::Cylindrical);
  BOOST_CHECK_EQUAL(cylindrical->getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(cylindrical->getLength(), 30.0);

  // 测试创建球形刀具
  auto ball = CutterFactory::createCutter(CutterType::Ball, 10.0, 30.0);
  BOOST_CHECK_EQUAL(ball->getType(), CutterType::Ball);
  BOOST_CHECK_EQUAL(ball->getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(ball->getLength(), 30.0);

  // 测试创建牛鼻刀具
  auto bull = CutterFactory::createCutter(CutterType::Bull, 10.0, 30.0, 2.0);
  BOOST_CHECK_EQUAL(bull->getType(), CutterType::Bull);
  BOOST_CHECK_EQUAL(bull->getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(bull->getLength(), 30.0);

  // 测试创建锥形刀具
  auto cone = CutterFactory::createCutter(CutterType::Cone, 10.0, 30.0, 0.5);
  BOOST_CHECK_EQUAL(cone->getType(), CutterType::Cone);
  BOOST_CHECK_EQUAL(cone->getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(cone->getLength(), 30.0);

  // 测试创建环形刀具
  auto torus = CutterFactory::createCutter(CutterType::Torus, 10.0, 30.0, 2.0);
  BOOST_CHECK_EQUAL(torus->getType(), CutterType::Torus);
  BOOST_CHECK_EQUAL(torus->getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(torus->getLength(), 30.0);
}

// 测试圆柱形刀具
BOOST_AUTO_TEST_CASE(test_cylindrical_cutter) {
  CylindricalCutter cutter(10.0, 30.0);

  // 测试基本属性
  BOOST_CHECK_EQUAL(cutter.getType(), CutterType::Cylindrical);
  BOOST_CHECK_EQUAL(cutter.getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(cutter.getLength(), 30.0);

  // 测试高度和宽度计算
  BOOST_CHECK_EQUAL(cutter.height(0.0), 0.0);
  BOOST_CHECK_EQUAL(cutter.height(5.0), 0.0);
  BOOST_CHECK_EQUAL(cutter.height(10.0), -1.0); // 超出范围

  BOOST_CHECK_EQUAL(cutter.width(0.0), 5.0);
  BOOST_CHECK_EQUAL(cutter.width(15.0), 5.0);
  BOOST_CHECK_EQUAL(cutter.width(30.0), 5.0);
  BOOST_CHECK_EQUAL(cutter.width(35.0), -1.0); // 超出范围
}

// 测试球形刀具
BOOST_AUTO_TEST_CASE(test_ball_cutter) {
  BallCutter cutter(10.0, 30.0);

  // 测试基本属性
  BOOST_CHECK_EQUAL(cutter.getType(), CutterType::Ball);
  BOOST_CHECK_EQUAL(cutter.getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(cutter.getLength(), 30.0);

  // 测试高度和宽度计算
  BOOST_CHECK_CLOSE(cutter.height(0.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.height(3.0), 5.0 - std::sqrt(25.0 - 9.0), 0.001);
  BOOST_CHECK_EQUAL(cutter.height(10.0), -1.0); // 超出范围

  BOOST_CHECK_CLOSE(cutter.width(0.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.width(2.5), std::sqrt(25.0 - 2.5 * 2.5), 0.001);
  BOOST_CHECK_CLOSE(cutter.width(5.0), 0.0, 0.001);
  BOOST_CHECK_EQUAL(cutter.width(35.0), -1.0); // 超出范围
}

// 测试牛鼻刀具
BOOST_AUTO_TEST_CASE(test_bull_cutter) {
  BullCutter cutter(10.0, 2.0, 30.0);

  // 测试基本属性
  BOOST_CHECK_EQUAL(cutter.getType(), CutterType::Bull);
  BOOST_CHECK_EQUAL(cutter.getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(cutter.getLength(), 30.0);
  BOOST_CHECK_EQUAL(cutter.getCornerRadius(), 2.0);

  // 测试高度和宽度计算
  BOOST_CHECK_CLOSE(cutter.height(0.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.height(3.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.height(4.0), 2.0 - std::sqrt(4.0 - 1.0), 0.001);
  BOOST_CHECK_EQUAL(cutter.height(10.0), -1.0); // 超出范围

  BOOST_CHECK_CLOSE(cutter.width(0.0), 3.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.width(1.0), 3.0 + std::sqrt(4.0 - 1.0), 0.001);
  BOOST_CHECK_CLOSE(cutter.width(2.0), 5.0, 0.001);
  BOOST_CHECK_EQUAL(cutter.width(35.0), -1.0); // 超出范围
}

// 测试锥形刀具
BOOST_AUTO_TEST_CASE(test_cone_cutter) {
  ConeCutter cutter(10.0, 0.5, 30.0);

  // 测试基本属性
  BOOST_CHECK_EQUAL(cutter.getType(), CutterType::Cone);
  BOOST_CHECK_EQUAL(cutter.getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(cutter.getLength(), 30.0);
  BOOST_CHECK_EQUAL(cutter.getAngle(), 0.5);

  // 测试高度和宽度计算
  double tanAngle = std::tan(0.5 / 2.0);

  BOOST_CHECK_CLOSE(cutter.height(0.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.height(3.0), 3.0 / tanAngle, 0.001);
  BOOST_CHECK_EQUAL(cutter.height(10.0), -1.0); // 超出范围

  BOOST_CHECK_CLOSE(cutter.width(0.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.width(10.0), 10.0 * tanAngle, 0.001);
  BOOST_CHECK_EQUAL(cutter.width(35.0), -1.0); // 超出范围
}

// 测试环形刀具
BOOST_AUTO_TEST_CASE(test_torus_cutter) {
  TorusCutter cutter(10.0, 2.0, 30.0);

  // 测试基本属性
  BOOST_CHECK_EQUAL(cutter.getType(), CutterType::Torus);
  BOOST_CHECK_EQUAL(cutter.getDiameter(), 10.0);
  BOOST_CHECK_EQUAL(cutter.getLength(), 30.0);
  BOOST_CHECK_EQUAL(cutter.getTorusRadius(), 2.0);

  // 测试高度和宽度计算
  BOOST_CHECK_CLOSE(cutter.height(0.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.height(3.0), 0.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.height(4.0), 2.0 - std::sqrt(4.0 - 1.0), 0.001);
  BOOST_CHECK_EQUAL(cutter.height(10.0), -1.0); // 超出范围

  BOOST_CHECK_CLOSE(cutter.width(0.0), 3.0, 0.001);
  BOOST_CHECK_CLOSE(cutter.width(1.0), 3.0 + std::sqrt(4.0 - 1.0), 0.001);
  BOOST_CHECK_CLOSE(cutter.width(2.0), 5.0, 0.001);
  BOOST_CHECK_EQUAL(cutter.width(35.0), -1.0); // 超出范围
}

// 测试刀具与三角形的接触
BOOST_AUTO_TEST_CASE(test_cutter_triangle_contact) {
  // 创建一个水平三角形
  Triangle triangle(Point(0.0, 0.0, 0.0), Point(10.0, 0.0, 0.0),
                    Point(0.0, 10.0, 0.0));

  // 测试圆柱形刀具
  {
    CylindricalCutter cutter(10.0, 30.0);

    // 刀具在三角形上方
    CutterPoint cp1 = cutter.dropCutter(Point(5.0, 5.0, 10.0), triangle);
    BOOST_CHECK_CLOSE(cp1.z(), 0.0, 0.001);
    BOOST_CHECK_EQUAL(cp1.getCCType(), CCType::FACET_CYL);

    // 刀具在三角形外部
    CutterPoint cp2 = cutter.dropCutter(Point(15.0, 15.0, 10.0), triangle);
    BOOST_CHECK(cp2.z() > 0.0);
  }

  // 测试球形刀具
  {
    BallCutter cutter(10.0, 30.0);

    // 刀具在三角形上方
    CutterPoint cp1 = cutter.dropCutter(Point(5.0, 5.0, 10.0), triangle);
    BOOST_CHECK_CLOSE(cp1.z(), 5.0, 0.001);
    BOOST_CHECK_EQUAL(cp1.getCCType(), CCType::FACET_BALL);

    // 刀具在三角形外部
    CutterPoint cp2 = cutter.dropCutter(Point(15.0, 15.0, 10.0), triangle);
    BOOST_CHECK(cp2.z() > 5.0);
  }
}

// 测试刀具网格创建
BOOST_AUTO_TEST_CASE(test_cutter_mesh_creation) {
  // 测试圆柱形刀具
  {
    auto cutter =
        CutterFactory::createCutter(CutterType::Cylindrical, 10.0, 30.0);
    auto mesh = cutter->createMesh(12);
    BOOST_CHECK(mesh != nullptr);
    BOOST_CHECK(mesh->vertexCount() > 0);
    BOOST_CHECK(mesh->triangleCount() > 0);
  }

  // 测试球形刀具
  {
    auto cutter = CutterFactory::createCutter(CutterType::Ball, 10.0, 30.0);
    auto mesh = cutter->createMesh(12);
    BOOST_CHECK(mesh != nullptr);
    BOOST_CHECK(mesh->vertexCount() > 0);
    BOOST_CHECK(mesh->triangleCount() > 0);
  }

  // 测试牛鼻刀具
  {
    auto cutter =
        CutterFactory::createCutter(CutterType::Bull, 10.0, 2.0, 30.0);
    auto mesh = cutter->createMesh(12);
    BOOST_CHECK(mesh != nullptr);
    BOOST_CHECK(mesh->vertexCount() > 0);
    BOOST_CHECK(mesh->triangleCount() > 0);
  }

  // 测试锥形刀具
  {
    auto cutter =
        CutterFactory::createCutter(CutterType::Cone, 10.0, 0.5, 30.0);
    auto mesh = cutter->createMesh(12);
    BOOST_CHECK(mesh != nullptr);
    BOOST_CHECK(mesh->vertexCount() > 0);
    BOOST_CHECK(mesh->triangleCount() > 0);
  }

  // 测试环形刀具
  {
    auto cutter =
        CutterFactory::createCutter(CutterType::Torus, 10.0, 2.0, 30.0);
    auto mesh = cutter->createMesh(12);
    BOOST_CHECK(mesh != nullptr);
    BOOST_CHECK(mesh->vertexCount() > 0);
    BOOST_CHECK(mesh->triangleCount() > 0);
  }
}