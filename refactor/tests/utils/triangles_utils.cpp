#include "triangles_utils.h"
#include <random>

namespace ocl {

std::vector<Point> createRandomPointsInTriangle(const Triangle& triangle,
                                                const size_t num_points,
                                                const bool no_vertex,
                                                const bool no_edge) {
    std::vector<Point> points;
    points.reserve(num_points);

    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    size_t count = 0;
    while (count < num_points) {
        // 生成重心坐标 (r1, r2, r3) 其中 r1 + r2 + r3 = 1
        // 方法1：使用两个随机数
        double r1 = dist(gen);
        double r2 = dist(gen);
        
        // 如果r1+r2>1，则通过变换保证点在三角形内
        if (r1 + r2 > 1.0) {
            r1 = 1.0 - r1;
            r2 = 1.0 - r2;
        }
        
        double r3 = 1.0 - r1 - r2;

        // 检查是否需要排除顶点
        if (no_vertex) {
            // 如果任一坐标接近1，则该点接近顶点
            if (r1 > 0.99 || r2 > 0.99 || r3 > 0.99) {
                continue;
            }
        }

        // 检查是否需要排除边
        if (no_edge) {
            // 如果任一坐标接近0，则该点接近边
            if (r1 < 0.01 || r2 < 0.01 || r3 < 0.01) {
                continue;
            }
        }

        // 使用重心坐标计算点的位置
        Point point = triangle.p[0] * r1 + triangle.p[1] * r2 + triangle.p[2] * r3;
        points.push_back(point);
        count++;
    }
    
    return points;
}

}  // namespace ocl
