/*
 *  Copyright (c) 2023 OpenCAMLib contributors
 *
 *  This file is part of OpenCAMLib
 *  (see https://github.com/aewallin/opencamlib).
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef OCL_TORUS_CUTTER_HPP
#define OCL_TORUS_CUTTER_HPP

#include "cutter.hpp"
#include <cmath>

namespace ocl {

// 环形刀具类
class TorusCutter : public ICutter {
public:
  TorusCutter(double diameter, double length, double torusRadius)
      : diameter_(diameter), radius_(diameter / 2.0), length_(length),
        torusRadius_(torusRadius) {
    // 确保环形半径不大于刀具半径
    if (torusRadius_ > radius_) {
      torusRadius_ = radius_;
    }

    // 计算环形中心轨迹的半径
    centerRadius_ = radius_ - torusRadius_;
  }

  // 获取刀具类型
  CutterType getType() const override { return CutterType::Torus; }

  // 获取刀具直径
  double getDiameter() const override { return diameter_; }

  // 获取刀具长度
  double getLength() const override { return length_; }

  // 获取环形半径
  double getTorusRadius() const { return torusRadius_; }

  // 计算给定半径处的刀具高度
  double height(double r) const override {
    if (r > radius_) {
      return -1.0; // 超出刀具范围
    }

    if (r <= centerRadius_) {
      // 中心区域（平坦部分）
      return torusRadius_;
    }

    // 环形部分
    double dr = r - centerRadius_;
    return torusRadius_ - std::sqrt(torusRadius_ * torusRadius_ - dr * dr);
  }

  // 计算给定高度处的刀具宽度
  double width(double h) const override {
    if (h < 0.0 || h > length_) {
      return -1.0; // 超出刀具范围
    }

    if (h > torusRadius_) {
      return radius_; // 圆柱部分
    }

    if (h <= 0.0) {
      return radius_; // 底部
    }

    // 环形部分
    double dh = torusRadius_ - h;
    return centerRadius_ + std::sqrt(torusRadius_ * torusRadius_ - dh * dh);
  }

  // 计算刀具与三角形的接触点
  CutterPoint dropCutter(const Point &point,
                         const Triangle &triangle) const override {
    // 初始化结果为点本身，z坐标设为无穷大
    CutterPoint result(
        Point(point.x(), point.y(), std::numeric_limits<double>::max()));

    // 获取三角形的三个顶点
    const Point &v0 = triangle.v0();
    const Point &v1 = triangle.v1();
    const Point &v2 = triangle.v2();

    // 检查与三角形顶点的接触
    checkVertex(point, v0, result);
    checkVertex(point, v1, result);
    checkVertex(point, v2, result);

    // 检查与三角形边的接触
    checkEdge(point, v0, v1, result);
    checkEdge(point, v1, v2, result);
    checkEdge(point, v2, v0, result);

    // 检查与三角形面的接触
    checkFacet(point, triangle, result);

    return result;
  }

  // 计算刀具与网格的接触点
  CutterPoint dropCutter(const Point &point, const IMesh &mesh) const override {
    // 初始化结果为点本身，z坐标设为无穷大
    CutterPoint result(
        Point(point.x(), point.y(), std::numeric_limits<double>::max()));

    // 遍历网格中的所有三角形
    for (size_t i = 0; i < mesh.triangleCount(); ++i) {
      Triangle triangle = mesh.getTriangle(i);

      // 计算与当前三角形的接触点
      CutterPoint triangleResult = dropCutter(point, triangle);

      // 如果新的接触点更高，则更新结果
      if (triangleResult.z() < result.z()) {
        result = triangleResult;
      }
    }

    return result;
  }

  // 批量计算刀具与网格的接触点
  std::vector<CutterPoint> dropCutter(const std::vector<Point> &points,
                                      const IMesh &mesh) const override {
    std::vector<CutterPoint> results;
    results.reserve(points.size());

    for (const auto &point : points) {
      results.push_back(dropCutter(point, mesh));
    }

    return results;
  }

  // 创建刀具的几何表示
  std::shared_ptr<IMesh> createMesh(double resolution = 20) const override {
    // 计算圆周上的点数和环形截面上的点数
    int numCirclePoints = static_cast<int>(resolution);
    int numTorusPoints = static_cast<int>(resolution / 2);

    if (numCirclePoints < 8)
      numCirclePoints = 8;
    if (numTorusPoints < 4)
      numTorusPoints = 4;

    // 计算顶点总数
    int numVertices = 1 +                                // 顶部中心点
                      numCirclePoints +                  // 顶部圆周点
                      numCirclePoints * numTorusPoints + // 环形点
                      1;                                 // 底部中心点

    // 创建顶点矩阵
    VertexMatrix vertices(numVertices, 3);

    // 添加顶部中心点
    vertices.row(0) = Eigen::Vector3d(0, 0, length_);

    // 添加顶部圆周点
    for (int i = 0; i < numCirclePoints; ++i) {
      double angle = 2.0 * PI * i / numCirclePoints;
      double x = radius_ * std::cos(angle);
      double y = radius_ * std::sin(angle);

      vertices.row(i + 1) = Eigen::Vector3d(x, y, length_ - torusRadius_);
    }

    // 添加环形点
    for (int j = 0; j < numTorusPoints; ++j) {
      double phi = PI / 2 + PI / 2 * j / (numTorusPoints - 1);

      for (int i = 0; i < numCirclePoints; ++i) {
        double angle = 2.0 * PI * i / numCirclePoints;

        // 计算环形中心点
        double cx = centerRadius_ * std::cos(angle);
        double cy = centerRadius_ * std::sin(angle);

        // 计算环形上的点
        double x = cx + torusRadius_ * std::cos(phi) * std::cos(angle);
        double y = cy + torusRadius_ * std::cos(phi) * std::sin(angle);
        double z = torusRadius_ * std::sin(phi);

        int index = 1 + numCirclePoints + j * numCirclePoints + i;
        vertices.row(index) = Eigen::Vector3d(x, y, z);
      }
    }

    // 添加底部中心点
    vertices.row(numVertices - 1) = Eigen::Vector3d(0, 0, 0);

    // 计算面的数量
    int numFaces = numCirclePoints + // 顶部圆锥面
                   numCirclePoints * (numTorusPoints - 1) *
                       2 + // 环形四边形（每个分成两个三角形）
                   numCirclePoints; // 底部平面

    // 创建面矩阵
    FaceMatrix faces(numFaces, 3);

    // 添加顶部圆锥面
    for (int i = 0; i < numCirclePoints; ++i) {
      int next = (i + 1) % numCirclePoints;
      faces.row(i) = Eigen::Vector3i(0, i + 1, next + 1);
    }

    // 添加环形四边形
    int faceIndex = numCirclePoints;
    for (int j = 0; j < numTorusPoints - 1; ++j) {
      for (int i = 0; i < numCirclePoints; ++i) {
        int next = (i + 1) % numCirclePoints;

        int v1 = 1 + numCirclePoints + j * numCirclePoints + i;
        int v2 = 1 + numCirclePoints + j * numCirclePoints + next;
        int v3 = 1 + numCirclePoints + (j + 1) * numCirclePoints + i;
        int v4 = 1 + numCirclePoints + (j + 1) * numCirclePoints + next;

        // 每个四边形分成两个三角形
        faces.row(faceIndex++) = Eigen::Vector3i(v1, v2, v4);
        faces.row(faceIndex++) = Eigen::Vector3i(v1, v4, v3);
      }
    }

    // 添加底部平面（连接到最后一圈环形点）
    int lastTorusCircleStart =
        1 + numCirclePoints + (numTorusPoints - 1) * numCirclePoints;
    for (int i = 0; i < numCirclePoints; ++i) {
      int next = (i + 1) % numCirclePoints;
      faces.row(faceIndex++) =
          Eigen::Vector3i(numVertices - 1, lastTorusCircleStart + i,
                          lastTorusCircleStart + next);
    }

    // 创建网格
    return MeshFactory::createFromData(vertices, faces);
  }

private:
  double diameter_;     // 刀具直径
  double radius_;       // 刀具半径
  double length_;       // 刀具长度
  double torusRadius_;  // 环形半径
  double centerRadius_; // 环形中心轨迹半径

  // 检查刀具与顶点的接触
  void checkVertex(const Point &point, const Point &vertex,
                   CutterPoint &result) const {
    // 计算点到顶点的水平距离
    double dx = point.x() - vertex.x();
    double dy = point.y() - vertex.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算接触点的z坐标
    double z;

    if (distance <= centerRadius_) {
      // 中心区域接触
      z = vertex.z() + torusRadius_;
    } else {
      // 环形部分接触
      double dr = distance - centerRadius_;
      z = vertex.z() + torusRadius_ -
          std::sqrt(torusRadius_ * torusRadius_ - dr * dr);
    }

    // 如果新的z坐标更小，则更新结果
    if (z < result.z()) {
      // 计算法向量
      Vector3 normal;

      if (distance <= centerRadius_) {
        // 中心区域法向量垂直向上
        normal = Vector3(0, 0, 1);
      } else {
        // 环形部分法向量
        double dr = distance - centerRadius_;
        double dz =
            torusRadius_ - std::sqrt(torusRadius_ * torusRadius_ - dr * dr);

        // 计算环形上的接触点到环形中心的向量
        double ratio = distance > EPSILON ? 1.0 / distance : 0.0;
        double nx = dx * ratio;
        double ny = dy * ratio;

        // 环形中心到接触点的向量
        Vector3 torusVector(dr * nx, dr * ny, dz);
        torusVector.normalize();

        normal = torusVector;
      }

      result.setCL(Point(point.x(), point.y(), z));
      result.setNormal(normal);
      result.setCC(vertex);
    }
  }

  // 检查刀具与边的接触
  void checkEdge(const Point &point, const Point &v1, const Point &v2,
                 CutterPoint &result) const {
    // 计算边的方向向量
    Vector3 edge = v2 - v1;
    double edgeLength = edge.norm();

    if (edgeLength < EPSILON) {
      return; // 边长度太小，忽略
    }

    // 归一化边向量
    Vector3 edgeDir = edge / edgeLength;

    // 计算点到边的垂直向量
    Vector3 toPoint = Point(point.x(), point.y(), 0) - Point(v1.x(), v1.y(), 0);
    Vector3 perpendicular = toPoint - toPoint.dot(edgeDir) * edgeDir;
    double perpDistance = perpendicular.norm();

    // 计算点在边上的投影参数
    double t = toPoint.dot(edgeDir);

    // 检查投影是否在边的范围内
    if (t < 0 || t > edgeLength) {
      return; // 投影在边的延长线上，不考虑
    }

    // 计算边上的最近点
    Point closestPoint = v1 + t * edgeDir;

    // 计算接触点的z坐标
    double z;
    Vector3 normal;

    if (perpDistance <= centerRadius_) {
      // 中心区域接触
      z = closestPoint.z() + torusRadius_;
      normal = Vector3(0, 0, 1);
    } else {
      // 环形部分接触
      double dr = perpDistance - centerRadius_;
      z = closestPoint.z() + torusRadius_ -
          std::sqrt(torusRadius_ * torusRadius_ - dr * dr);

      // 计算法向量
      if (perpDistance > centerRadius_ + EPSILON) {
        // 环形部分法向量
        double dz =
            torusRadius_ - std::sqrt(torusRadius_ * torusRadius_ - dr * dr);

        // 计算环形上的接触点到环形中心的向量
        double ratio = perpDistance > EPSILON ? 1.0 / perpDistance : 0.0;
        double nx = perpendicular.x() * ratio;
        double ny = perpendicular.y() * ratio;

        // 环形中心到接触点的向量
        normal = Vector3(dr * nx, dr * ny, dz);
        normal.normalize();
      } else {
        // 如果在环形中心轨迹上，法向量垂直于边和z轴
        normal = edgeDir.cross(Vector3(0, 0, 1)).normalized();
      }
    }

    // 如果新的z坐标更小，则更新结果
    if (z < result.z()) {
      result.setCL(Point(point.x(), point.y(), z));
      result.setNormal(normal);
      result.setCC(closestPoint);
    }
  }

  // 检查刀具与面的接触
  void checkFacet(const Point &point, const Triangle &triangle,
                  CutterPoint &result) const {
    // 获取三角形法向量
    Vector3 normal = triangle.normal();

    // 如果法向量几乎水平，则忽略
    if (std::abs(normal.z()) < EPSILON) {
      return;
    }

    // 计算点在三角形平面上的投影
    double d = triangle.v0().dot(normal);
    double t = (d - Point(point.x(), point.y(), 0).dot(normal)) / normal.z();
    Point projectedPoint(point.x(), point.y(), t);

    // 检查投影点是否在三角形内
    if (triangle.contains(projectedPoint)) {
      // 计算接触点的z坐标
      double z;

      if (normal.z() > 1.0 - EPSILON) {
        // 如果面几乎水平，则使用中心区域
        z = t + torusRadius_;
      } else {
        // 否则考虑环形部分
        // 这是一个近似计算，对于大多数情况足够准确
        double nx = normal.x();
        double ny = normal.y();
        double nz = normal.z();

        // 计算环形与平面的接触点
        z = t + torusRadius_ * (1.0 - std::sqrt(nx * nx + ny * ny) / nz);
      }

      // 如果新的z坐标更小，则更新结果
      if (z < result.z()) {
        result.setCL(Point(point.x(), point.y(), z));
        result.setNormal(normal);
        result.setCC(projectedPoint);
      }
    }
  }
};

} // namespace ocl

#endif // OCL_TORUS_CUTTER_HPP