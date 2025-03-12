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

#ifndef OCL_CYLINDRICAL_CUTTER_HPP
#define OCL_CYLINDRICAL_CUTTER_HPP

#include "cutter.hpp"
#include <cmath>

namespace ocl {

// 圆柱形刀具类
class CylindricalCutter : public ICutter {
public:
  CylindricalCutter(double diameter, double length)
      : diameter_(diameter), radius_(diameter / 2.0), length_(length) {}

  // 获取刀具类型
  CutterType getType() const override { return CutterType::Cylindrical; }

  // 获取刀具直径
  double getDiameter() const override { return diameter_; }

  // 获取刀具长度
  double getLength() const override { return length_; }

  // 计算给定半径处的刀具高度
  double height(double r) const override {
    if (r > radius_) {
      return -1.0; // 超出刀具范围
    }
    return 0.0; // 圆柱形刀具底部是平的
  }

  // 计算给定高度处的刀具宽度
  double width(double h) const override {
    if (h < 0.0 || h > length_) {
      return -1.0; // 超出刀具范围
    }
    return radius_; // 圆柱形刀具宽度恒定
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

    // 创建顶点矩阵
    VertexMatrix vertices(numPoints * 2 + 2, 3);

    // 添加顶部和底部中心点
    vertices.row(0) = Eigen::Vector3d(0, 0, length_);
    vertices.row(numPoints * 2 + 1) = Eigen::Vector3d(0, 0, 0);

    // 添加圆周上的点
    for (int i = 0; i < numPoints; ++i) {
      double angle = 2.0 * PI * i / numPoints;
      double x = radius_ * std::cos(angle);
      double y = radius_ * std::sin(angle);

      // 顶部圆周点
      vertices.row(i + 1) = Eigen::Vector3d(x, y, length_);

      // 底部圆周点
      vertices.row(i + numPoints + 1) = Eigen::Vector3d(x, y, 0);
    }

    // 创建面矩阵
    // 顶部面 + 底部面 + 侧面
    FaceMatrix faces(numPoints * 2, 3);

    // 添加顶部面
    for (int i = 0; i < numPoints; ++i) {
      int next = (i + 1) % numPoints;
      faces.row(i) = Eigen::Vector3i(0, i + 1, next + 1);
    }

    // 添加底部面
    for (int i = 0; i < numPoints; ++i) {
      int next = (i + 1) % numPoints;
      faces.row(i + numPoints) = Eigen::Vector3i(
          numPoints * 2 + 1, next + numPoints + 1, i + numPoints + 1);
    }

    // 添加侧面
    FaceMatrix sideFaces(numPoints * 2, 3);
    for (int i = 0; i < numPoints; ++i) {
      int next = (i + 1) % numPoints;

      // 每个矩形分成两个三角形
      sideFaces.row(i * 2) =
          Eigen::Vector3i(i + 1, i + numPoints + 1, next + numPoints + 1);
      sideFaces.row(i * 2 + 1) =
          Eigen::Vector3i(i + 1, next + numPoints + 1, next + 1);
    }

    // 合并所有面
    FaceMatrix allFaces(faces.rows() + sideFaces.rows(), 3);
    allFaces.topRows(faces.rows()) = faces;
    allFaces.bottomRows(sideFaces.rows()) = sideFaces;

    // 创建网格
    return MeshFactory::createFromData(vertices, allFaces);
  }

private:
  double diameter_; // 刀具直径
  double radius_;   // 刀具半径
  double length_;   // 刀具长度

  // 检查刀具与顶点的接触
  void checkVertex(const Point &point, const Point &vertex,
                   CutterPoint &result) const {
    // 计算点到顶点的水平距离
    double dx = point.x() - vertex.x();
    double dy = point.y() - vertex.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    // 如果距离小于刀具半径，则可能接触
    if (distance <= radius_) {
      // 计算接触点的z坐标
      double z =
          vertex.z() + std::sqrt(radius_ * radius_ - distance * distance);

      // 如果新的z坐标更小，则更新结果
      if (z < result.z()) {
        // 计算法向量（从接触点指向刀具中心）
        Vector3 normal(dx, dy, 0);
        if (normal.norm() > EPSILON) {
          normal.normalize();
        } else {
          normal = Vector3(0, 0, 1);
        }

        // 设置刀具位置点（CL点）
        result.setCL(Point(point.x(), point.y(), z));
        // 设置法向量
        result.setNormal(normal);
        // 设置刀具接触点（CC点）
        result.setCC(vertex);
        // 设置接触类型
        result.setCCType(CCType::VERTEX_CYL);
      }
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

    // 如果距离小于刀具半径，则可能接触
    if (distance <= radius_) {
      // 计算接触点的z坐标
      double z =
          closestPoint.z() + std::sqrt(radius_ * radius_ - distance * distance);

      // 如果新的z坐标更小，则更新结果
      if (z < result.z()) {
        // 计算法向量
        Vector3 normal;
        CCType contactType;

        if (distance > EPSILON) {
          normal = Vector3(dx, dy, 0);
          normal.normalize();
          contactType = CCType::EDGE_CYL;
        } else {
          // 如果水平距离为0，则法向量为边的垂直向量
          normal = edgeDir.cross(Vector3(0, 0, 1)).normalized();
          contactType = CCType::EDGE_SHAFT;
        }

        // 设置刀具位置点（CL点）
        result.setCL(Point(point.x(), point.y(), z));
        // 设置法向量
        result.setNormal(normal);
        // 设置刀具接触点（CC点）
        result.setCC(closestPoint);
        // 设置接触类型
        result.setCCType(contactType);
      }
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
      // 如果新的z坐标更小，则更新结果
      if (t < result.z()) {
        // 设置刀具位置点（CL点）
        result.setCL(Point(point.x(), point.y(), t));
        // 设置法向量
        result.setNormal(normal);
        // 设置刀具接触点（CC点）
        result.setCC(projectedPoint);
        // 设置接触类型
        result.setCCType(CCType::FACET_CYL);
      }
    }
  }
};

} // namespace ocl

#endif // OCL_CYLINDRICAL_CUTTER_HPP