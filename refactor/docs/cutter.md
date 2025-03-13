# OpenCAMlib 切削刀具 (Cutters) 模块

本文档提供了 OpenCAMlib 中 `/src/cutters` 目录的详细概述，包括各种切削刀具类型、核心算法和数学计算。

## 目录结构

`/src/cutters` 目录包含了用于 CNC 加工仿真的各种铣刀实现。这些刀具用于计算刀具路径并执行诸如 drop-cutter 和 push-cutter 等算法。

## 基础类

### `MillingCutter` (millingcutter.hpp/cpp)

- 所有铣刀的抽象基类
- 定义了常见属性如直径、半径和长度
- 实现了核心算法：
  - `dropCutter`：沿 z 轴降下刀具直到与三角形接触
  - `pushCutter`：沿纤维方向推动刀具直到与三角形接触
  - `vertexDrop`、`facetDrop`、`edgeDrop`：专门的降刀操作

## 刀具类型

### 1. `CylCutter` (cylcutter.hpp/cpp)

- 圆柱形平底铣刀
- 由直径和长度定义
- 最简单的刀具类型，具有平底

### 2. `BallCutter` (ballcutter.hpp/cpp)

- 球头或球形铣刀
- 由直径和长度定义
- 具有半球形切削端

### 3. `BullCutter` (bullcutter.hpp/cpp)

- 牛鼻刀或环形铣刀（圆角端铣刀）
- 由直径、圆角半径和长度定义
- 结合了平底和圆角

### 4. `ConeCutter` (conecutter.hpp/cpp)

- 锥形铣刀
- 由直径、锥半角和长度定义
- 用于 V 形雕刻和刻字

## 复合刀具

### `CompositeCutter` (compositecutter.hpp/cpp)

- 可以将多种刀具类型组合成一个
- 刀具在不同的径向区域有效
- 专门的复合类型包括：
  - `CompCylCutter`：复合圆柱刀具
  - `CompBallCutter`：复合球刀
  - `CylConeCutter`：圆柱 + 锥形
  - `BallConeCutter`：球形 + 锥形
  - `BullConeCutter`：牛鼻 + 锥形
  - `ConeConeCutter`：锥形 + 锥形

## 支持类

### `Ellipse` (ellipse.hpp/cpp)

- 椭圆的数学表示
- 用于某些刀具-三角形相交计算

### `EllipsePosition` (ellipseposition.hpp/cpp)

- 帮助在 3D 空间中定位椭圆
- 用于复杂的刀具-三角形相交计算

## 核心算法

所有刀具都实现了以下关键算法：

### 1. 高度和宽度函数

这些是定义每个刀具形状的基本几何函数：

#### 高度函数

```cpp
virtual double height(double r) const
```

- **目的**：返回给定半径 r 处刀具表面的高度（z 坐标）
- **各刀具实现**：
  - **CylCutter**：如果 r ≤ radius 返回 0（平底），否则返回 -1（无效）
  - **BallCutter**：返回 radius - sqrt(radius² - r²)（球面方程）
  - **BullCutter**：涉及平底部分和环形角的复杂计算
  - **ConeCutter**：返回 r * tan(angle)（锥面方程）

#### 宽度函数

```cpp
virtual double width(double h) const
```

- **目的**：返回给定高度 h 处刀具的宽度（半径）
- **各刀具实现**：
  - **CylCutter**：始终返回 radius（恒定宽度）
  - **BallCutter**：如果 h < radius，返回 sqrt(radius² - (radius-h)²)，否则返回 radius
  - **BullCutter**：基于环形几何的复杂计算
  - **ConeCutter**：返回 h / tan(angle)（高度函数的逆）

### 2. Drop Cutter 算法

Drop cutter 算法确定刀具可以定位的最低点，而不会切入工件。

#### 主函数

```cpp
bool dropCutter(CLPoint &cl, const Triangle &t) const
```

- **目的**：在位置 (cl.x, cl.y) 降下刀具直到接触三角形 t
- **算法**：
  1. 依次调用 `vertexDrop`、`facetDrop` 和 `edgeDrop`
  2. 将 cl.z 更新为找到的最高 z 值（以避免切入）
  3. 设置刀具接触点 (CC point)，即刀具接触三角形的位置

#### 顶点降刀

```cpp
bool vertexDrop(CLPoint &cl, const Triangle &t) const
```

- **目的**：检查三角形的任何顶点是否在刀具下方
- **算法**：
  1. 对于三角形 t 的每个顶点 p：
     - 计算 cl 到 p 的 xy 平面距离 q
     - 如果 q ≤ radius（顶点在刀具下方）：
       - 计算 z = p.z - height(q)（使用高度函数）
       - 如果 z > cl.z，更新 cl.z 并将 CC 点设置为该顶点

#### 面降刀

```cpp
bool facetDrop(CLPoint &cl, const Triangle &t) const
```

- **目的**：检查三角形的面（平面）是否与刀具相交
- **算法**：
  1. 获取三角形的法向量
  2. 如果 normal.z = 0（垂直面），返回 false（无法对垂直面降刀）
  3. 如果 normal.x = 0 且 normal.y = 0（水平面）：
     - 将 CC 点设置为 (cl.x, cl.y, facet_z)
  4. 否则（一般情况）：
     - 计算平面方程：a*x + b*y + c*z + d = 0
     - 计算从 CC 点到刀具中心的半径向量
     - 在平面上找到 CC 点
     - 计算刀具尖端 z 坐标
     - 必要时更新 cl.z

#### 边降刀

```cpp
bool edgeDrop(CLPoint &cl, const Triangle &t) const
```

- **目的**：检查三角形的任何边是否与刀具相交
- **算法**：
  1. 对于三角形 t 的每条边 (p1, p2)：
     - 计算 cl 到边的 xy 平面距离 d
     - 如果 d ≤ radius（边可能与刀具相交）：
       - 调用 `singleEdgeDrop(cl, p1, p2, d)`
       - 这将问题转换为规范形式并调用 `singleEdgeDropCanonical()`
       - 每种刀具类型都实现自己的 `singleEdgeDropCanonical()` 方法

#### 规范边降刀

```cpp
virtual CC_CLZ_Pair singleEdgeDropCanonical(const Point& u1, const Point& u2) const
```

- **目的**：在简化的坐标系中解决边降刀问题
- **各刀具实现**：
  - **CylCutter**：找到圆柱与线的交点，选择较高的点
  - **BallCutter**：找到球体与线的交点
  - **BullCutter**：涉及环形几何的复杂计算
  - **ConeCutter**：找到锥体与线的交点

### 3. Push Cutter 算法

Push cutter 算法确定刀具沿纤维（线）移动时会切入工件的位置。

#### 主函数

```cpp
bool pushCutter(const Fiber& f, Interval& i, const Triangle& t) const
```

- **目的**：沿纤维 f 推动刀具并找到它会切入三角形 t 的区间 i
- **算法**：
  1. 依次调用 `vertexPush`、`facetPush` 和 `edgePush`
  2. 用所有切入区间的并集更新区间 i

#### 顶点推刀

```cpp
bool vertexPush(const Fiber& f, Interval& i, const Triangle& t) const
```

- **目的**：找到刀具会切入三角形 t 的任何顶点的区间
- **算法**：
  1. 对于三角形 t 的每个顶点 p：
     - 调用 `singleVertexPush(f, i, p, VERTEX)`
     - 这计算刀具会切入顶点的区间

#### 面推刀

```cpp
bool facetPush(const Fiber& f, Interval& i, const Triangle& t) const
```

- **目的**：找到刀具会切入三角形 t 的面的区间
- **算法**：
  1. 计算刀具的 normal_length、center_height 和 xy_normal_length
  2. 用这些参数调用 `generalFacetPush()`
  3. 这计算刀具会切入面的区间

#### 边推刀

```cpp
bool edgePush(const Fiber& f, Interval& i, const Triangle& t) const
```

- **目的**：找到刀具会切入三角形 t 的任何边的区间
- **算法**：
  1. 对于三角形 t 的每条边 (p1, p2)：
     - 调用 `singleEdgePush(f, i, p1, p2)`
     - 这处理特殊情况（水平、轴）并调用 `generalEdgePush()`
     - 每种刀具类型都实现自己的 `generalEdgePush()` 方法

#### 一般边推刀

```cpp
virtual bool generalEdgePush(const Fiber& f, Interval& i, const Point& p1, const Point& p2) const
```

- **目的**：解决一般情况下的边推刀问题
- **各刀具实现**：
  - **BallCutter**：找到球体与 3D 中线的交点
  - **BullCutter**：涉及环形几何的复杂计算
  - **ConeCutter**：找到锥体与 3D 中线的交点

### 4. 复合刀具算法

`CompositeCutter` 类组合多个刀具，需要特殊处理：

#### 复合降刀

```cpp
bool facetDrop(CLPoint &cl, const Triangle &t) const
```

- **目的**：将复合刀具降到三角形上
- **算法**：
  1. 对于每个组件刀具：
     - 调用其 `facetDrop()` 方法
     - 如果有效且高于当前 cl.z，更新 cl.z 和 CC 点
  2. 根据半径确定哪个组件刀具接触

#### 复合推刀

```cpp
bool vertexPush(const Fiber& f, Interval& i, const Triangle& t) const
```

- **目的**：沿纤维推动复合刀具
- **算法**：
  1. 对于每个组件刀具：
     - 调用其 `vertexPush()` 方法
     - 用所有切入区间的并集更新区间 i
  2. 根据半径确定哪个组件刀具接触

## 数学计算

OpenCAMlib 的切削刀具模块使用了多种复杂的数学计算来模拟 CNC 加工过程：

### 1. 几何形状表示

#### 基本几何函数

- **高度函数 (height)**：计算给定半径处的刀具高度

  ```cpp
  // 球刀高度函数
  double height(double r) const {return radius - sqrt(square(radius) - square(r));}
  
  // 圆锥刀高度函数
  double height(double r) const {return r/tan(angle);}
  
  // 牛鼻刀高度函数
  double height(double r) const {
      if (r <= radius1)
          return 0.0; // 圆柱部分
      else
          return radius2 - sqrt(square(radius2) - square(r-radius1)); // 环面部分
  }
  ```

- **宽度函数 (width)**：计算给定高度处的刀具宽度

  ```cpp
  // 球刀宽度函数
  double width(double h) const {return (h >= radius) ? radius : sqrt(square(radius) - square(radius-h));}
  
  // 圆锥刀宽度函数
  double width(double h) const {return (h<center_height) ? h*tan(angle) : radius;}
  ```

### 2. 三角函数计算

- **角度计算**：特别是在圆锥刀中使用

  ```cpp
  double angle = a; // 圆锥半角，以弧度表示
  double center_height = radius/tan(angle); // 圆锥高度
  ```

- **正弦/余弦/正切**：用于计算切削刀具的几何形状

  ```cpp
  double theta = atan((p2.z - p1.z) / (p2-p1).xyNorm()); // 计算边缘的斜率
  double major_length = fabs(radius2/sin(theta)); // 椭圆长轴长度
  ```

### 3. 椭圆计算

- **椭圆表示**：特别是在牛鼻刀(BullCutter)的边缘接触计算中

  ```cpp
  // 创建椭圆
  Ellipse(Point& centerin, double a, double b, double offset);
  
  // 椭圆上的点
  Point ePoint(const EllipsePosition& position) const;
  ```

- **椭圆求解器**：使用数值方法（如Brent方法）求解椭圆相关方程

  ```cpp
  int solver_brent(); // 使用Brent方法求解偏移椭圆
  ```

### 4. 线性代数计算

- **向量运算**：点积、叉积、归一化等

  ```cpp
  Point ao_x_ab = ao.cross(ab); // 叉积
  double ab2 = ab.dot(ab); // 点积
  xy_tang.xyNormalize(); // 归一化
  ```

- **坐标变换**：旋转和平移

  ```cpp
  // 将几何体平移使得在XY平面上cl = (0,0)
  // 旋转p1-p2边缘使得新边缘u1-u2沿x轴
  ```

### 5. 相交计算

#### 线-线相交

```cpp
bool xy_line_line_intersection(p1, p2, u, f.p1, f.p2, v); // 计算XY平面上两条线的相交
```

#### 线-圆相交

```cpp
// 圆-线相交公式
// x = det*dy +/- sign(dy) * dx * sqrt(r^2 dr^2 - det^2) / dr^2
// y = -det*dx +/- abs(dy) * sqrt(r^2 dr^2 - det^2) / dr^2
```

#### 线-圆柱相交

```cpp
// 射线：P(t) = O + t*V
// 圆柱 [A, B, r]
// 点P在无限圆柱上的条件：((P - A) x (B - A))^2 = r^2 * (B - A)^2
// 展开为二次方程：a*t^2 + b*t + c = 0
```

#### 线-椭圆相交

```cpp
// 在牛鼻刀中使用对齐椭圆计算相交
AlignedEllipse e(ell_center, major_length, minor_length, radius1, major_dir, minor_dir);
```

### 6. 数值方法

- **二次方程求解**：用于计算线与曲面的相交

  ```cpp
  double discr = b*b-4*a*c;
  double t1 = (-b + sqrt(discr)) / (2*a);
  double t2 = (-b - sqrt(discr)) / (2*a);
  ```

- **Brent方法**：用于求解非线性方程

  ```cpp
  double dia_sln = brent_zero(apos.diangle, bpos.diangle, 3E-16, OE_ERROR_TOLERANCE, this);
  ```

### 7. 复合几何计算

- **复合刀具几何**：组合多个基本刀具形状

  ```cpp
  // 球锥刀计算
  double rcontact = radius1*cos(angle);
  double height1 = radius1 - sqrt(square(radius1)-square(rcontact));
  double cone_offset = -((rcontact)/tan(angle) - height1);
  ```

### 8. 距离计算

- **点到线距离**：

  ```cpp
  double d = cl.xyDistanceToLine(p1,p2); // XY平面上点到线的距离
  ```

- **点到点距离**：

  ```cpp
  double q = (p-pq).xyNorm(); // XY平面上两点间的距离
  ```

### 9. 区间计算

- **区间更新**：用于确定刀具与工件接触的区间

  ```cpp
  i.updateUpper(f.tval(stop), cc_tmp);
  i.updateLower(f.tval(start), cc_tmp);
  ```

这些数学计算共同构成了OpenCAMlib切削刀具模块的核心，使其能够准确模拟不同类型刀具与工件表面的接触，从而实现高精度的CNC加工仿真和刀具路径生成。

## Drop vs Push

Drop Cutter和Push Cutter是OpenCAMlib中两种核心算法，虽然它们有相似之处，但解决的问题和实现方式有本质区别。

### 相似点

1. **接触检测对象**：两种算法都需要检测刀具与三角形的三种元素（顶点、边、面）的接触
2. **数学基础**：都使用相似的几何计算（如高度函数、宽度函数、相交计算）
3. **分解策略**：都将复杂问题分解为顶点检测、边检测和面检测三部分

### 本质区别

1. **运动方向不同**：
   - **Drop Cutter**：刀具沿Z轴（垂直）方向下降，直到接触工件
   - **Push Cutter**：刀具沿XY平面内的纤维(Fiber)方向移动，寻找接触区间

2. **输出结果不同**：
   - **Drop Cutter**：输出单个接触点的Z高度（CLPoint的z值）
   - **Push Cutter**：输出刀具沿纤维移动时会与工件接触的区间（Interval）

3. **算法目标不同**：
   - **Drop Cutter**：寻找刀具可以下降到的最低点（不切入工件）
   - **Push Cutter**：寻找刀具沿纤维移动时会切入工件的区间

4. **数学处理不同**：
   - **Drop Cutter**：主要求解点到面的距离问题
   - **Push Cutter**：主要求解线与面的相交问题

### 为什么Push Cutter需要计算区间而非单个值

Push Cutter需要求解完整的接触参数区间，而不仅仅是单个值，原因如下：

1. **工件切削的本质需求**：
   - 需要知道刀具在哪些位置会与工件接触（或切入工件）
   - 确保刀具不会意外切入不应该切削的区域
   - 生成有效的刀具路径，确保完整加工所有需要切削的区域

2. **参数区间的实际意义**：
   - **区间下界**：刀具开始接触工件的位置
   - **区间上界**：刀具结束接触工件的位置
   - **区间内部**：刀具持续与工件接触的所有位置

3. **多个接触区域的情况**：
   在复杂工件上，刀具沿一条纤维移动时可能会有多个分离的接触区域，例如：

   ```
   工件横截面：    ___      ___      ___
                  |   |    |   |    |   |
   纤维方向：→→→→→→→→→→→→→→→→→→→→→→→→→→→→→→
   接触区间：      [--]    [--]    [--]
   ```

   这种情况下，需要知道所有的接触区间，而不仅仅是第一个接触点。

4. **刀具路径生成的需求**：
   - 需要知道刀具何时进入材料（区间下界）
   - 需要知道刀具何时离开材料（区间上界）
   - 可能需要在接触区间内调整进给速度
   - 需要在多个接触区间之间规划快速移动

### 算法组合与优化

虽然Drop Cutter和Push Cutter不能简单合并为单一通用算法，但可以通过以下方式优化：

1. **共享基础几何计算**：将共同的几何计算（如高度函数、宽度函数）抽象为通用工具
2. **统一接口设计**：为两种算法设计统一的接口，使调用更一致
3. **算法组合**：在特定应用中组合使用两种算法，例如使用Drop Cutter生成初始高度图，然后使用Push Cutter优化水平切削路径

总之，Drop Cutter和Push Cutter解决的是CNC加工中两种不同的基本问题，它们的核心逻辑和目标有本质区别，因此需要分别实现，但可以共享许多基础计算和数据结构。
