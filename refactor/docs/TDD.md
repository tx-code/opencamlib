# OpenCAMLib (OCL) 测试驱动开发 (TDD) 实施计划

本文档详细描述了OpenCAMLib重构项目中采用测试驱动开发(TDD)的实施计划，以确保代码质量和功能正确性。

## 1. TDD在OCL中的价值

在几何计算库这类对精确性要求高的系统中，TDD能带来显著优势：

1. **确保功能一致性** - 重构的主要挑战是保持行为不变。通过先写测试，可以确保新实现与原有功能完全一致。

2. **防止算法退化** - CAM操作对精度和性能都有高要求，测试可以防止优化过程中引入误差或性能退化。

3. **模块化设计** - TDD自然促使代码更模块化，这与重构目标一致。先编写测试会推动更好的接口设计。

4. **简化重构过程** - 大型重构容易失控，而测试提供了"安全网"，可以较小单元进行重构。

5. **文档化算法行为** - 测试实际上是算法行为的活文档，特别适合记录边缘情况和预期结果。

## 2. 测试框架与配置

### 2.1 测试框架选择

采用Google Test (gtest)作为主要测试框架，结合CMake构建系统：

```cpp
// tests/main.cpp
#include <gtest/gtest.h>

int main(int argc, char **argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
```

### 2.2 基本测试结构

```
tests/
├── main.cpp                 # 测试主入口
├── geo/                     # 几何模块测试
│   ├── point_test.cpp
│   ├── triangle_test.cpp
│   └── mesh_test.cpp
├── cutters/                 # 刀具模块测试
│   ├── cylcutter_test.cpp
│   ├── ballcutter_test.cpp
│   └── bullcutter_test.cpp
├── operations/              # 操作模块测试
│   ├── dropcutter_test.cpp
│   ├── pushcutter_test.cpp
│   └── waterline_test.cpp
└── fixtures/                # 测试夹具
    ├── mesh_fixture.hpp
    └── cutter_fixture.hpp
```

### 2.3 CMake集成

```cmake
# tests/CMakeLists.txt
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG release-1.12.1
)
FetchContent_MakeAvailable(googletest)

add_executable(ocl_tests
    main.cpp
    geo/point_test.cpp
    # 其他测试文件...
)

target_link_libraries(ocl_tests
    PRIVATE
        ocl
        gtest
        gtest_main
)

add_test(NAME ocl_tests COMMAND ocl_tests)
```

## 3. 分层测试策略

从基础组件到高层模块，按依赖关系构建测试：

### 3.1 几何原语测试

```cpp
// tests/geo/point_test.cpp
#include <gtest/gtest.h>
#include "geo/point.h"

TEST(GeoTests, PointOperations) {
    Point p1(1.0, 2.0, 3.0);
    Point p2(4.0, 5.0, 6.0);
    
    // 向量运算测试
    auto v = p2 - p1;
    EXPECT_NEAR(v.norm(), 5.196, 0.001);
    
    // 点积测试
    EXPECT_NEAR(v.dot(v), 27.0, 0.001);
}
```

### 3.2 刀具模型测试

```cpp
// tests/cutters/cylcutter_test.cpp
#include <gtest/gtest.h>
#include "cutters/cylcutter.h"

TEST(CutterTests, CylCutterGeometry) {
    CylCutter cutter(10.0, 20.0);  // 直径10mm，长度20mm
    
    // 高度函数测试
    EXPECT_DOUBLE_EQ(cutter.height(0.0), 0.0);    // 中心点高度
    EXPECT_DOUBLE_EQ(cutter.height(4.9), 0.0);    // 半径内高度
    EXPECT_DOUBLE_EQ(cutter.height(5.1), -1.0);   // 超出半径
    
    // 宽度函数测试
    EXPECT_DOUBLE_EQ(cutter.width(0.0), 5.0);     // 水平面宽度
    EXPECT_DOUBLE_EQ(cutter.width(-1.0), -1.0);   // 无效高度
}
```

### 3.3 算法单元测试

```cpp
// tests/dropcutter/dropcutter_test.cpp
#include <gtest/gtest.h>
#include "operations/dropcutter.h"

TEST(DropCutterTests, TriangleDropCutter) {
    // 创建简单平面三角形
    Triangle t({0,0,0}, {10,0,0}, {0,10,0});
    CylCutter cutter(2.0, 10.0);
    
    // 测试平面上的点
    Point sample(5, 5, 10);
    auto result = dropCutter(cutter, t, sample);
    
    EXPECT_NEAR(result.z, 0.0, 0.001);
    
    // 测试边缘情况
    Point edge(10, 10, 10);
    auto edgeResult = dropCutter(cutter, t, edge);
    EXPECT_EQ(edgeResult.type, CLPoint::EDGE);
}
```

## 4. 测试技术与模式

### 4.1 参数化测试

使用Google Test的参数化测试功能测试多个输入：

```cpp
struct HeightTestCase {
    double r;
    double expectedHeight;
};

class BallCutterTest : public ::testing::TestWithParam<HeightTestCase> {};

TEST_P(BallCutterTest, HeightFunction) {
    auto testCase = GetParam();
    BallCutter cutter(10.0, 30.0);  // 直径10mm球刀
    
    EXPECT_NEAR(cutter.height(testCase.r), testCase.expectedHeight, 0.001);
}

INSTANTIATE_TEST_SUITE_P(
    HeightFunctionTests,
    BallCutterTest,
    ::testing::Values(
        HeightTestCase{0.0, 0.0},         // 中心点
        HeightTestCase{2.5, 0.3371},      // 1/4半径
        HeightTestCase{5.0, 5.0},         // 刀具半径
        HeightTestCase{6.0, -1.0}         // 超出半径
    )
);
```

### 4.2 测试夹具

使用测试夹具创建复用的测试环境：

```cpp
// tests/fixtures/mesh_fixture.hpp
#include <gtest/gtest.h>
#include "mesh/libigl_mesh.h"

class MeshFixture : public ::testing::Test {
protected:
    void SetUp() override {
        // 创建测试网格
        mesh = std::make_shared<LibiglMesh>();
        // 添加网格数据
        V.resize(4, 3);
        F.resize(2, 3);
        
        // 简单四边形网格 (两个三角形)
        V << 0, 0, 0,
             10, 0, 0,
             10, 10, 0,
             0, 10, 0;
        
        F << 0, 1, 2,
             0, 2, 3;
             
        mesh->setMesh(V, F);
    }
    
    std::shared_ptr<LibiglMesh> mesh;
    Eigen::MatrixXd V;
    Eigen::MatrixXi F;
};
```

### 4.3 性能测试

结合功能测试和性能测试：

```cpp
TEST(PerformanceTests, DropCutterPerformance) {
    // 创建较大的网格
    auto mesh = createLargeMesh(10000);
    CylCutter cutter(10.0, 20.0);
    
    // 创建采样点
    std::vector<Point> samples = createSampleGrid(100, 100);
    
    // 测量性能
    auto start = std::chrono::high_resolution_clock::now();
    
    for (const auto& sample : samples) {
        auto result = cutter.dropCutter(sample, *mesh);
    }
    
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    
    std::cout << "Processing time: " << duration.count() << "ms" << std::endl;
    
    // 基准性能检查
    EXPECT_LT(duration.count(), 5000); // 应该在5秒内完成
}
```

### 4.4 数值误差处理

浮点数比较采用相对误差：

```cpp
// 使用EXPECT_NEAR进行浮点数比较
EXPECT_NEAR(actual, expected, 0.001); // 允许0.001的绝对误差

// 或者自定义比较函数
bool almostEqual(double a, double b, double tolerance = 1e-10) {
    return std::abs(a - b) <= tolerance * std::max(1.0, std::max(std::abs(a), std::abs(b)));
}

// 在测试中使用
EXPECT_TRUE(almostEqual(actual, expected));
```

## 5. TDD开发流程

### 5.1 测试先行原则

1. **编写失败测试**：为新功能或修复编写测试，确保测试失败
2. **实现功能**：最小实现使测试通过
3. **重构代码**：优化实现，保持测试通过

示例工作流：

```cpp
// Step 1: 写测试
TEST(CutterTests, CalculateContactPoint) {
    CylCutter cutter(10.0, 20.0);
    Triangle tri({0,0,0}, {10,0,0}, {0,10,0});
    Point samplePoint(5, 5, 10);
    
    auto contact = cutter.calculateContactPoint(tri, samplePoint);
    
    ASSERT_TRUE(contact.has_value());
    EXPECT_NEAR(contact->x, 5.0, 0.001);
    EXPECT_NEAR(contact->y, 5.0, 0.001);
    EXPECT_NEAR(contact->z, 0.0, 0.001);
}

// Step 2: 实现功能
std::optional<Point> CylCutter::calculateContactPoint(
    const Triangle& tri, const Point& samplePoint) const {
    // 初步实现
    return Point(5, 5, 0); // 暂时硬编码
}

// Step 3: 完善实现
std::optional<Point> CylCutter::calculateContactPoint(
    const Triangle& tri, const Point& samplePoint) const {
    // 完整实现逻辑...
}
```

### 5.2 回归测试策略

1. 基于原始代码行为创建参考测试
2. 确保每次重构后所有测试通过
3. 定期运行完整测试套件

### 5.3 边界条件测试

特别注意边界情况，如：

1. 刀具与网格边缘、顶点的接触
2. 特殊几何形状（共面三角形、退化图元）
3. 数值极限情况（非常大或非常小的值）

```cpp
TEST(DropCutterTests, EdgeCases) {
    // 测试刀具与三角形边缘的接触
    Triangle t({0,0,0}, {10,0,0}, {0,10,0});
    CylCutter cutter(2.0, 10.0);
    
    // 测试边缘上的点
    Point edgePoint(5, 0, 10);  // 位于三角形边缘上方
    auto result = dropCutter(cutter, t, edgePoint);
    
    EXPECT_EQ(result.type, CLPoint::EDGE);
    EXPECT_NEAR(result.z, 0.0, 0.001);
}
```

## 6. CI/CD集成

### 6.1 自动化测试配置

设置GitHub Actions自动化测试：

```yaml
# .github/workflows/tests.yml
name: C++ Tests
on: [push, pull_request]
jobs:
  build_and_test:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v2
      - name: Install dependencies
        run: sudo apt-get install -y libeigen3-dev
      - name: Build
        run: |
          mkdir build && cd build
          cmake ..
          make
      - name: Test
        run: |
          cd build
          ctest --output-on-failure
```

### 6.2 测试覆盖率监控

使用gcov和lcov监控测试覆盖率：

```cmake
# 启用覆盖率
if(CODE_COVERAGE)
    target_compile_options(ocl PUBLIC -g -O0 --coverage)
    target_link_options(ocl PUBLIC --coverage)
endif()
```

## 7. 测试优先级与实施计划

### 7.1 测试优先级

按以下优先级实施测试：

1. **核心几何模块**：点、向量、三角形等基础组件
2. **刀具模型**：各种刀具形状及其特性
3. **操作算法**：DropCutter、PushCutter等核心算法
4. **集成功能**：组合使用不同组件的功能

### 7.2 实施时间表

| 阶段 | 内容 | 时间估计 |
|------|------|----------|
| 1 | 搭建测试框架与基础设施 | 1周 |
| 2 | 几何模块测试 | 2周 |
| 3 | 刀具模块测试 | 2周 |
| 4 | 算法模块测试 | 3周 |
| 5 | 集成测试 | 2周 |

## 8. 结论

采用TDD方法进行OCL重构将显著提高代码质量和可维护性。虽然在短期内可能会增加开发时间，但从长远来看，将节省大量调试和修复问题的时间，特别是在处理复杂的几何算法时。TDD也将作为活文档，帮助开发者理解系统的预期行为和边界条件。
