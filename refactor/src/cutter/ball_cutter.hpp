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

#ifndef OCL_BALL_CUTTER_HPP
#define OCL_BALL_CUTTER_HPP

#include "cutter.hpp"
#include <cmath>

namespace ocl {

// 球形刀具类
class BallCutter : public ICutter {
public:
  BallCutter(double diameter, double length)
      : diameter_(diameter), radius_(diameter / 2.0), length_(length) {}

  // 获取刀具类型
  CutterType getType() const override { return CutterType::Ball; }

  // 获取刀具直径
  double getDiameter() const override { return diameter_; }

  // 获取刀具长度
  double getLength() const override { return length_; }

  // 计算给定半径处的刀具高度
  double height(double r) const override {
    if (r > radius_) {
      return -1.0; // 超出刀具范围
    }
    return radius_ - std::sqrt(radius_ * radius_ - r * r);
  }

  // 计算给定高度处的刀具宽度
  double width(double h) const override {
    if (h < 0.0 || h > radius_ || h > length_) {
      return -1.0; // 超出刀具范围
    }
    return std::sqrt(radius_ * radius_ - (radius_ - h) * (radius_ - h));
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
    // 计算圆周上的点数
    int numPoints = static_cast<int>(resolution);
    if (numPoints < 8)
      numPoints = 8;

    // 计算球体上的点数
    int numRings = numPoints / 2;
    if (numRings < 4)
      numRings = 4;

    // 创建顶点矩阵
    int totalVertices = numPoints * numRings + 2;
    VertexMatrix vertices(totalVertices, 3);

    // 添加顶部和底部点
    vertices.row(0) = Eigen::Vector3d(0, 0, length_);
    vertices.row(1) = Eigen::Vector3d(0, 0, length_ - radius_);

    // 添加球体部分的点
    int vertexIndex = 2;
    for (int ring = 1; ring < numRings; ++ring) {
      double phi = PI * ring / numRings;
      double z = length_ - radius_ + radius_ * std::cos(phi);
      double ringRadius = radius_ * std::sin(phi);

      for (int i = 0; i < numPoints; ++i) {
        double theta = 2.0 * PI * i / numPoints;
        double x = ringRadius * std::cos(theta);
        double y = ringRadius * std::sin(theta);

        vertices.row(vertexIndex++) = Eigen::Vector3d(x, y, z);
      }
    }

    // 添加底部圆周点
    for (int i = 0; i < numPoints; ++i) {
      double theta = 2.0 * PI * i / numPoints;
      double x = radius_ * std::cos(theta);
      double y = radius_ * std::sin(theta);

      vertices.row(vertexIndex++) = Eigen::Vector3d(x, y, length_ - radius_);
    }

    // 创建面矩阵
    int totalFaces = numPoints * (numRings - 1) + numPoints;
    FaceMatrix faces(totalFaces, 3);

    // 添加顶部三角形
    for (int i = 0; i < numPoints; ++i) {
      int next = (i + 1) % numPoints;
      faces.row(i) = Eigen::Vector3i(1, i + 2, next + 2);
    }

    // 添加球体部分的三角形
    int faceIndex = numPoints;
    for (int ring = 1; ring < numRings - 1; ++ring) {
      int ringStart = 2 + (ring - 1) * numPoints;
      int nextRingStart = 2 + ring * numPoints;

      for (int i = 0; i < numPoints; ++i) {
        int next = (i + 1) % numPoints;

        // 每个矩形分成两个三角形
        faces.row(faceIndex++) = Eigen::Vector3i(
            ringStart + i, nextRingStart + i, nextRingStart + next);
        faces.row(faceIndex++) = Eigen::Vector3i(
            ringStart + i, nextRingStart + next, ringStart + next);
      }
    }

    // 创建网格
    return MeshFactory::createFromData(vertices, faces);
  }

private:
  double diameter_; // 刀具直径
  double radius_;   // 刀具半径
  double length_;   // 刀具长度

  // 检查刀具与顶点的接触
  void checkVertex(const Point &point, const Point &vertex,
                   CutterPoint &result) const {
    // 计算点到顶点的距离
    double dx = point.x() - vertex.x();
    double dy = point.y() - vertex.y();
    double dz = 0; // 我们只关心水平距离
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算接触点的z坐标
    double z = vertex.z() + radius_;

    // 如果距离小于刀具半径，则需要调整z坐标
    if (distance < radius_) {
      // 使用球面方程计算z坐标
      z -= std::sqrt(radius_ * radius_ - distance * distance);
    }

    // 如果新的z坐标更小，则更新结果
    if (z < result.z()) {
      // 计算法向量（从接触点指向刀具中心）
      Vector3 normal;
      if (distance > EPSILON) {
        normal = Vector3(dx, dy,
                         -std::sqrt(radius_ * radius_ - distance * distance));
      } else {
        normal = Vector3(0, 0, -radius_);
      }
      normal.normalize();

      // 计算接触点
      Point contactPoint = vertex;

      // 设置刀具位置点（CL点）
      result.setCL(Point(point.x(), point.y(), z));
      // 设置法向量
      result.setNormal(normal);
      // 设置刀具接触点（CC点）
      result.setCC(contactPoint);
      // 设置接触类型
      result.setCCType(CCType::VERTEX_BALL);
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
    Vector3 toPoint = Point(point.x(), point.y(), v1.z()) - v1;
    double dot = toPoint.dot(edgeDir);

    // 检查点是否在边的范围内
    if (dot < 0 || dot > edgeLength) {
      return; // 点在边的延长线上，不考虑
    }

    // 计算边上的最近点
    Point closestPoint = v1 + dot * edgeDir;

    // 计算点到最近点的水平距离
    double dx = point.x() - closestPoint.x();
    double dy = point.y() - closestPoint.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算接触点的z坐标
    double z = closestPoint.z() + radius_;

    // 如果距离小于刀具半径，则需要调整z坐标
    if (distance < radius_) {
      // 使用球面方程计算z坐标
      z -= std::sqrt(radius_ * radius_ - distance * distance);
    }

    // 如果新的z坐标更小，则更新结果
    if (z < result.z()) {
      // 计算法向量
      Vector3 normal;
      if (distance > EPSILON) {
        normal = Vector3(dx, dy,
                         -std::sqrt(radius_ * radius_ - distance * distance));
        normal.normalize();
      } else {
        // 如果水平距离为0，则法向量为边的垂直向量
        normal = edgeDir.cross(Vector3(0, 0, 1)).normalized();
      }

      // 计算接触点
      Point contactPoint = closestPoint;

      // 设置刀具位置点（CL点）
      result.setCL(Point(point.x(), point.y(), z));
      // 设置法向量
      result.setNormal(normal);
      // 设置刀具接触点（CC点）
      result.setCC(contactPoint);
      // 设置接触类型
      result.setCCType(CCType::EDGE_BALL);
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
    double t = (triangle.v0().dot(normal) -
                Point(point.x(), point.y(), 0).dot(normal)) /
               normal.z();
    Point projectedPoint(point.x(), point.y(), t);

    // 检查投影点是否在三角形内
    if (triangle.contains(projectedPoint)) {
      // 计算接触点的z坐标
      double z = t + radius_ * (1.0 - normal.z());

      // 如果新的z坐标更小，则更新结果
      if (z < result.z()) {
        // 计算接触点
        Point contactPoint = projectedPoint - radius_ * normal;

        // 设置刀具位置点（CL点）
        result.setCL(Point(point.x(), point.y(), z));
        // 设置法向量
        result.setNormal(normal);
        // 设置刀具接触点（CC点）
        result.setCC(contactPoint);
        // 设置接触类型
        result.setCCType(CCType::FACET_BALL);
      }
    }
  }
};

} // namespace ocl

#endif // OCL_BALL_CUTTER_HPP