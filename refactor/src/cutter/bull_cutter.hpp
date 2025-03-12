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

#ifndef OCL_BULL_CUTTER_HPP
#define OCL_BULL_CUTTER_HPP

#include "cutter.hpp"
#include <cmath>

namespace ocl {

// 牛鼻刀具类（圆角端铣刀）
class BullCutter : public ICutter {
public:
  BullCutter(double diameter, double cornerRadius, double length)
      : diameter_(diameter), radius_(diameter / 2.0),
        cornerRadius_(cornerRadius), length_(length) {
    // 确保圆角半径不大于刀具半径
    if (cornerRadius_ > radius_) {
      cornerRadius_ = radius_;
    }

    // 计算平坦部分的半径
    flatRadius_ = radius_ - cornerRadius_;
  }

  // 获取刀具类型
  CutterType getType() const override { return CutterType::Bull; }

  // 获取刀具直径
  double getDiameter() const override { return diameter_; }

  // 获取刀具长度
  double getLength() const override { return length_; }

  // 获取圆角半径
  double getCornerRadius() const { return cornerRadius_; }

  // 计算给定半径处的刀具高度
  double height(double r) const override {
    if (r > radius_) {
      return -1.0; // 超出刀具范围
    }

    if (r <= flatRadius_) {
      return 0.0; // 平坦部分
    }

    // 圆角部分
    double dr = r - flatRadius_;
    return cornerRadius_ - std::sqrt(cornerRadius_ * cornerRadius_ - dr * dr);
  }

  // 计算给定高度处的刀具宽度
  double width(double h) const override {
    if (h < 0.0 || h > length_) {
      return -1.0; // 超出刀具范围
    }

    if (h > cornerRadius_) {
      return radius_; // 圆柱部分
    }

    // 圆角部分
    return flatRadius_ + std::sqrt(cornerRadius_ * cornerRadius_ -
                                   (cornerRadius_ - h) * (cornerRadius_ - h));
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

    // 计算圆角部分的环数
    int numRings = numPoints / 4;
    if (numRings < 3)
      numRings = 3;

    // 创建顶点矩阵
    int totalVertices = 1 + numPoints + numPoints * numRings + numPoints + 1;
    VertexMatrix vertices(totalVertices, 3);

    // 添加顶部中心点
    vertices.row(0) = Eigen::Vector3d(0, 0, length_);

    // 添加顶部圆周点
    for (int i = 0; i < numPoints; ++i) {
      double angle = 2.0 * PI * i / numPoints;
      double x = radius_ * std::cos(angle);
      double y = radius_ * std::sin(angle);

      vertices.row(i + 1) = Eigen::Vector3d(x, y, length_ - cornerRadius_);
    }

    // 添加圆角部分的点
    int vertexIndex = numPoints + 1;
    for (int ring = 1; ring <= numRings; ++ring) {
      double phi = PI / 2 * ring / numRings;
      double z = cornerRadius_ * (1 - std::sin(phi));
      double r = flatRadius_ + cornerRadius_ * std::cos(phi);

      for (int i = 0; i < numPoints; ++i) {
        double angle = 2.0 * PI * i / numPoints;
        double x = r * std::cos(angle);
        double y = r * std::sin(angle);

        vertices.row(vertexIndex++) = Eigen::Vector3d(x, y, z);
      }
    }

    // 添加底部圆周点
    for (int i = 0; i < numPoints; ++i) {
      double angle = 2.0 * PI * i / numPoints;
      double x = flatRadius_ * std::cos(angle);
      double y = flatRadius_ * std::sin(angle);

      vertices.row(vertexIndex++) = Eigen::Vector3d(x, y, 0);
    }

    // 添加底部中心点
    vertices.row(totalVertices - 1) = Eigen::Vector3d(0, 0, 0);

    // 创建面矩阵
    int totalFaces = numPoints + numPoints * numRings * 2 + numPoints;
    FaceMatrix faces(totalFaces, 3);

    // 添加顶部面
    for (int i = 0; i < numPoints; ++i) {
      int next = (i + 1) % numPoints;
      faces.row(i) = Eigen::Vector3i(0, i + 1, next + 1);
    }

    // 添加圆角部分的面
    int faceIndex = numPoints;
    for (int ring = 0; ring < numRings; ++ring) {
      int ringStart = numPoints + 1 + ring * numPoints;
      int nextRingStart = ringStart + numPoints;

      for (int i = 0; i < numPoints; ++i) {
        int next = (i + 1) % numPoints;

        // 每个矩形分成两个三角形
        faces.row(faceIndex++) = Eigen::Vector3i(
            ringStart + i, nextRingStart + i, nextRingStart + next);
        faces.row(faceIndex++) = Eigen::Vector3i(
            ringStart + i, nextRingStart + next, ringStart + next);
      }
    }

    // 添加底部面
    int bottomStart = numPoints + 1 + numRings * numPoints;
    for (int i = 0; i < numPoints; ++i) {
      int next = (i + 1) % numPoints;
      faces.row(faceIndex++) = Eigen::Vector3i(
          totalVertices - 1, bottomStart + next, bottomStart + i);
    }

    // 创建网格
    return MeshFactory::createFromData(vertices, faces);
  }

private:
  double diameter_;     // 刀具直径
  double radius_;       // 刀具半径
  double cornerRadius_; // 圆角半径
  double flatRadius_;   // 平坦部分半径
  double length_;       // 刀具长度

  // 检查刀具与顶点的接触
  void checkVertex(const Point &point, const Point &vertex,
                   CutterPoint &result) const {
    // 计算点到顶点的水平距离
    double dx = point.x() - vertex.x();
    double dy = point.y() - vertex.y();
    double distance = std::sqrt(dx * dx + dy * dy);

    // 计算接触点的z坐标
    double z;
    CCType contactType;

    if (distance <= flatRadius_) {
      // 平坦部分接触
      z = vertex.z();
      contactType = CCType::VERTEX_FLAT;
    } else if (distance <= radius_) {
      // 圆角部分接触
      double dr = distance - flatRadius_;
      z = vertex.z() + cornerRadius_ -
          std::sqrt(cornerRadius_ * cornerRadius_ - dr * dr);
      contactType = CCType::VERTEX_BULL;
    } else {
      // 超出刀具范围
      return;
    }

    // 如果新的z坐标更小，则更新结果
    if (z < result.z()) {
      // 计算法向量
      Vector3 normal;

      if (distance <= flatRadius_ || distance <= EPSILON) {
        // 平坦部分或距离很小，法向量垂直向上
        normal = Vector3(0, 0, 1);
      } else {
        // 圆角部分，法向量从接触点指向圆心
        double dr = distance - flatRadius_;
        double dz = std::sqrt(cornerRadius_ * cornerRadius_ - dr * dr);
        normal = Vector3(dx, dy, dz);
        normal.normalize();
      }

      // 设置刀具位置点（CL点）
      result.setCL(Point(point.x(), point.y(), z));
      // 设置法向量
      result.setNormal(normal);
      // 设置刀具接触点（CC点）
      result.setCC(vertex);
      // 设置接触类型
      result.setCCType(contactType);
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
    double z;
    CCType contactType;

    if (distance <= flatRadius_) {
      // 平坦部分接触
      z = closestPoint.z();
      contactType = CCType::EDGE_FLAT;
    } else if (distance <= radius_) {
      // 圆角部分接触
      double dr = distance - flatRadius_;
      z = closestPoint.z() + cornerRadius_ -
          std::sqrt(cornerRadius_ * cornerRadius_ - dr * dr);
      contactType = CCType::EDGE_BULL;
    } else {
      // 超出刀具范围
      return;
    }

    // 如果新的z坐标更小，则更新结果
    if (z < result.z()) {
      // 计算法向量
      Vector3 normal;

      if (distance <= flatRadius_) {
        // 平坦部分，法向量垂直向上
        normal = Vector3(0, 0, 1);
      } else if (distance <= EPSILON) {
        // 距离很小，法向量垂直于边和z轴
        normal = edgeDir.cross(Vector3(0, 0, 1)).normalized();
      } else {
        // 圆角部分，法向量从接触点指向圆心
        double dr = distance - flatRadius_;
        double dz = std::sqrt(cornerRadius_ * cornerRadius_ - dr * dr);
        normal = Vector3(dx, dy, dz);
        normal.normalize();
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
      double z;
      CCType contactType;

      if (normal.z() > 1.0 - EPSILON) {
        // 平面几乎水平，使用平坦部分
        z = t;
        contactType = CCType::FACET_FLAT;
      } else {
        // 倾斜平面，考虑圆角
        z = t + cornerRadius_ * (1.0 - normal.z());
        contactType = CCType::FACET_BULL;
      }

      // 如果新的z坐标更小，则更新结果
      if (z < result.z()) {
        // 计算接触点
        Point contactPoint;

        if (normal.z() > 1.0 - EPSILON) {
          // 平面几乎水平，接触点就是投影点
          contactPoint = projectedPoint;
        } else {
          // 倾斜平面，接触点需要考虑圆角
          contactPoint = projectedPoint - cornerRadius_ * normal;
        }

        // 设置刀具位置点（CL点）
        result.setCL(Point(point.x(), point.y(), z));
        // 设置法向量
        result.setNormal(normal);
        // 设置刀具接触点（CC点）
        result.setCC(contactPoint);
        // 设置接触类型
        result.setCCType(contactType);
      }
    }
  }
};

} // namespace ocl

#endif // OCL_BULL_CUTTER_HPP