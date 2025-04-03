# 自适应算法 (Adaptive Algorithms)

## 1. 概述

自适应算法是指能够根据模型表面的几何复杂度动态调整采样密度的算法。OpenCAMLib 中实现了两种主要的自适应算法：

- **AdaptiveWaterline**: 生成固定 Z 高度的水平切割路径
- **AdaptivePathDropCutter**: 沿指定路径生成刀具位置点

这些算法的核心思想是在曲率大或几何变化复杂的区域使用更密集的采样点，而在平坦区域使用较少的采样点，从而在保证加工精度的同时减少计算量和输出点数，提高效率。

## 2. 自适应算法原理

### 2.1 关键概念

#### 平坦度判断 (Flatness Predicate)

自适应算法的核心是"平坦度判断"机制：

```cpp
// AdaptiveWaterline 中的实现
bool flat(Point start_cl, Point mid_cl, Point stop_cl) const;

// AdaptivePathDropCutter 中的实现
bool flat(CLPoint &start_cl, CLPoint &mid_cl, CLPoint &stop_cl);
```

这个判断通过检查三个点（起点、中点、终点）形成的角度来决定区域是否"平坦"。当三点几乎共线时（角度接近 180 度），该区域被判断为平坦，不需要更多采样点。

#### 余弦限制 (Cosine Limit)

平坦度判断使用余弦值来进行角度比较：

```cpp
void setCosLimit(double lim) { cosLimit = lim; }
```

`cosLimit` 参数（默认为 0.999）控制平坦度判断的严格程度，直接影响采样的密度和精度。

#### 最小采样间隔 (Minimum Sampling)

为防止过度细分，设定了最小采样间隔：

```cpp
void setMinSampling(double s) { min_sampling = s; }
```

即使区域不平坦，当采样间隔达到 `min_sampling` 时也会停止细分。

### 2.2 递归细分算法

自适应算法的核心是递归细分过程：

```
函数 adaptive_sample(span, start_t, stop_t, start_cl, stop_cl):
    mid_t = (start_t + stop_t) / 2.0
    mid_cl = 在 mid_t 处采样
    
    // 平坦度判断 - 关键步骤
    if flat(start_cl, mid_cl, stop_cl) 或 (stop_t - start_t) <= min_sampling:
        返回  // 足够平坦或达到最小采样间隔，停止细分
    else:
        // 递归细分
        adaptive_sample(span, start_t, mid_t, start_cl, mid_cl)
        adaptive_sample(span, mid_t, stop_t, mid_cl, stop_cl)
```

在 `AdaptiveWaterline` 中，该过程分别在 X 和 Y 方向上实现为 `xfiber_adaptive_sample` 和 `yfiber_adaptive_sample`。

## 3. 与非自适应算法的比较

### 3.1 主要区别

| 特性 | 非自适应算法 | 自适应算法 |
|------|--------------|------------|
| 采样间隔 | 固定均匀间隔 | 可变间隔（根据几何复杂度调整） |
| 采样决策 | 一次性生成所有点 | 递归细分，按需生成 |
| 控制参数 | 单一参数 `sampling` | 多参数 (`sampling`, `min_sampling`, `cosLimit`) |
| 点数分布 | 均匀分布 | 复杂区域密集，平坦区域稀疏 |
| 计算量 | 可能较大（尤其对复杂模型） | 优化的，集中在需要的区域 |

### 3.2 优缺点分析

**自适应算法优点**：

- 减少总采样点数（特别是在大部分平坦的模型上）
- 提高计算效率和存储效率
- 在关键区域保持高精度

**自适应算法缺点**：

- 实现复杂度更高
- 需要调整更多参数
- 算法复杂度更高，可能引入额外开销

## 4. 实现细节

### 4.1 AdaptiveWaterline 类

```cpp
class AdaptiveWaterline : public Waterline {
public:
    void setMinSampling(double s) { min_sampling = s; }
    void setCosLimit(double lim) { cosLimit = lim; }
    void run();
    
protected:
    void adaptive_sampling_run();
    void xfiber_adaptive_sample(const Span *span, double start_t, double stop_t,
                              Fiber start_f, Fiber stop_f);
    void yfiber_adaptive_sample(const Span *span, double start_t, double stop_t,
                              Fiber start_f, Fiber stop_f);
    bool flat(Fiber &start, Fiber &mid, Fiber &stop) const;
    bool flat(Point start_cl, Point mid_cl, Point stop_cl) const;
    
    double min_sampling;
    double cosLimit;
};
```

### 4.2 AdaptivePathDropCutter 类

```cpp
class AdaptivePathDropCutter : public Operation {
public:
    void setMinSampling(double s) { min_sampling = s; }
    void setCosLimit(double lim) { cosLimit = lim; }
    void run();
    
protected:
    void adaptive_sample(const Span *span, double start_t, double stop_t,
                        CLPoint start_cl, CLPoint stop_cl);
    bool flat(CLPoint &start_cl, CLPoint &mid_cl, CLPoint &stop_cl);
    void adaptive_sampling_run();
    
    double min_sampling;
    double cosLimit;
};
```

### 4.3 平坦度判断实现

平坦度判断通常基于向量计算：

```cpp
bool AdaptiveWaterline::flat(Point start_cl, Point mid_cl, Point stop_cl) const {
    // 计算向量
    Point v1 = mid_cl - start_cl;
    Point v2 = stop_cl - mid_cl;
    v1.normalize();
    v2.normalize();
    
    // 计算角度（使用点积）
    double dotProduct = v1.dot(v2);
    
    // 如果点积接近 1.0，则向量几乎平行，区域平坦
    return (dotProduct >= cosLimit);
}
```

## 5. 应用场景

自适应算法特别适用于以下场景：

1. **复杂模型加工**：包含平坦区域和细节丰富区域的混合模型
2. **大型模型加工**：大型模型需要减少点数，提高效率
3. **精加工操作**：需要在关键区域保持高精度
4. **预览和规划**：快速生成路径预览和时间估计

## 6. 最佳实践

使用自适应算法时的一些建议：

1. 通常将 `min_sampling` 设置为标准 `sampling` 的 1/5 到 1/10
2. `cosLimit` 建议值为 0.99 到 0.9999，值越高要求越平坦
3. 对于粗加工，可以使用较低的 `cosLimit` 值
4. 对于精加工，使用较高的 `cosLimit` 值
5. 根据模型复杂度和精度要求调整参数
