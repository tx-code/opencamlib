# OpenCAMlib中的Weave算法

## 简介

Weave（编织）算法是OpenCAMlib中的一个核心几何算法，主要用于创建和处理曲面路径规划的拓扑结构。该算法将离散的扫描线信息转换为有组织的刀具路径，是实现高效加工策略的关键组件。

## 基础概念

1. **Fiber（纤维）**：代表空间中的一条线段，可以是X平行或Y平行的
2. **Interval（间隔）**：每条Fiber上的一段，由上下边界定义
3. **顶点类型**：
   - **CL（接触点）**：位于Interval的端点，代表刀具与工件表面的接触点
   - **INT（内部点）**：代表两个Fiber的交点，用于构建拓扑结构

4. **VertexPair**：顶点与位置值的配对，用于在Interval中存储和检索顶点

## 算法演化

OpenCAMlib实现了三个版本的Weave算法，逐步改进了性能和内存使用：

### 1. 基础Weave类

基础类提供了管理功能和接口：

- 管理X和Y方向纤维的容器
- 添加纤维的功能
- 遍历图结构生成循环的方法
- 获取点列表的接口

```cpp
void Weave::addFiber(Fiber& f) {
    if (f.dir.xParallel() && !f.empty()) {
        xfibers.push_back(f);
    } else if (f.dir.yParallel() && !f.empty()) {
        yfibers.push_back(f);
    }
}
```

### 2. SimpleWeave实现

这是一个基本实现，易于理解但内存效率较低：

- 为每个交点创建顶点
- 内存消耗与图的面积成正比（顶点数量约为N*N）
- 随着问题规模增大，内存占用迅速增加

```cpp
void SimpleWeave::build() {
    // 为所有X纤维和Y纤维的交点创建顶点
    BOOST_FOREACH(Fiber& xf, xfibers) {
        BOOST_FOREACH(Interval& xi, xf.ints) {
            // 添加X区间端点
            // 处理与所有Y纤维的交点
        }
    }
}
```

### 3. SmartWeave实现

改进版实现，优化了内存使用：

- 更智能地管理内存
- 只保留对算法结果有贡献的顶点和边
- 内存消耗与图的周长成正比（约为N+N）
- 对大规模问题有更好的性能

```cpp
void SmartWeave::build() {
    add_vertices_x();  // 添加X方向顶点和必要的交点
    add_vertices_y();  // 添加Y方向顶点和必要的交点
    
    // 只处理必要的交点
    BOOST_FOREACH(Fiber& xf, xfibers) {
        // 智能处理交点
    }
    
    add_all_edges();  // 添加必要的边
}
```

## 循环生成过程

所有Weave实现使用相同的循环生成算法：

```cpp
void Weave::face_traverse() { 
    while (!clVertexSet.empty()) {
        std::vector<Vertex> loop;
        Vertex current = *(clVertexSet.begin());
        Vertex first = current;
        
        do {
            loop.push_back(current);
            clVertexSet.erase(current);
            std::vector<Edge> outEdges = g.out_edges(current);
            Edge currentEdge = outEdges[0]; 
            do {
                current = g.target(currentEdge); 
                currentEdge = g[currentEdge].next;
            } while (g[current].type != CL);
        } while (current!=first);
        
        loops.push_back(loop);
    }
}
```

循环生成过程：

1. 从未处理的CL顶点开始
2. 遵循出边和next指针
3. 直到找到下一个CL顶点
4. 重复直到返回起始点，完成一个循环
5. 将所有CL顶点分配到不同的循环中

## next/prev指针系统

Weave算法的核心是建立一个半边数据结构，通过next和prev指针连接顶点：

```cpp
// SimpleWeave中设置next/prev边的示例
g[xe_lu_prev].next = xl_v;  g[xl_v].prev = xe_lu_prev;
g[xl_v].next = v_yl;        g[v_yl].prev = xl_v;
```

这种连接方式确保：

- 每个CL点恰好有一条入边和一条出边
- 遵循next指针会形成闭合循环
- 所有路径都正确构建

## 应用场景

在OpenCAMlib中，Weave算法应用于多种CAM策略：

### 1. 等高线加工（Waterline Machining）

```cpp
void Waterline::weave_process() {
    weave::SimpleWeave weave;
    // 添加X和Y方向的纤维
    weave.build(); 
    weave.face_traverse();
    loops = weave.getLoops();
}
```

- 在固定Z高度平面生成切削轮廓
- 使用BatchPushCutter生成X和Y方向的Fiber
- 通过Weave连接成闭合循环
- 适用于模具制造和复杂曲面加工

### 2. 自适应等高线加工（Adaptive Waterline）

- 继承自Waterline类，增加自适应采样功能
- 根据曲面复杂度动态调整采样密度
- 在复杂区域增加采样点，平坦区域减少采样点
- 提高加工精度和效率

### 3. 其他相关策略

- **PathDropCutter**：沿预定义路径进行下刀运算
- **BatchPushCutter**：为Weave算法提供纤维和交点信息
- **ZigZag加工**：可与Weave结合处理复杂内部区域

## 算法优势

1. **拓扑正确性**：保证生成的刀具路径是拓扑正确的闭合循环
2. **内存效率**：SmartWeave版本优化了内存使用，适用于大型复杂模型
3. **灵活性**：可应用于多种加工策略
4. **可扩展性**：算法框架允许进一步优化和定制

## 总结

Weave算法是OpenCAMlib中连接离散几何数据和生成连续刀具路径的关键桥梁。通过其半边数据结构和循环生成机制，它能够将X和Y方向的扫描线信息转换为拓扑正确的闭合路径，为各种CAM策略提供基础支持。
