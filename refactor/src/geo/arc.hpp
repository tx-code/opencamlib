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

#ifndef OCL_ARC_HPP
#define OCL_ARC_HPP

#include "common/types.hpp"
#include "geometry.hpp"
#include <cmath>
#include <optional>

namespace ocl {

// 圆弧类
class Arc : public IGeometry {
public:
  // 通过圆心、半径、起始角度和终止角度构造
  Arc(const Point &center, double radius, double startAngle, double endAngle,
      const Vector3 &normal = Vector3(0, 0, 1))
      : center_(center), radius_(radius), startAngle_(startAngle),
        endAngle_(endAngle), normal_(normal.normalized()) {

    // 确保角度在[0, 2π]范围内
    while (startAngle_ < 0)
      startAngle_ += 2 * PI;
    while (startAngle_ >= 2 * PI)
      startAngle_ -= 2 * PI;
    while (endAngle_ < 0)
      endAngle_ += 2 * PI;
    while (endAngle_ >= 2 * PI)
      endAngle_ -= 2 * PI;

    // 计算起点和终点
    updatePoints();
  }

  // 通过起点、终点和圆心构造
  static Arc fromPoints(const Point &start, const Point &end,
                        const Point &center) {
    Vector3 v1 = start - center;
    Vector3 v2 = end - center;

    double radius = v1.norm();
    if (std::abs(v2.norm() - radius) > EPSILON) {
      throw std::invalid_argument(
          "Start and end points must be equidistant from center");
    }

    // 计算法向量
    Vector3 normal = v1.cross(v2);
    if (normal.norm() < EPSILON) {
      throw std::invalid_argument(
          "Start and end points cannot be collinear with center");
    }
    normal.normalize();

    // 计算角度
    double startAngle = std::atan2(v1.y(), v1.x());
    double endAngle = std::atan2(v2.y(), v2.x());

    return Arc(center, radius, startAngle, endAngle, normal);
  }

  const Point &center() const { return center_; }
  double radius() const { return radius_; }
  double startAngle() const { return startAngle_; }
  double endAngle() const { return endAngle_; }
  const Vector3 &normal() const { return normal_; }
  const Point &startPoint() const { return startPoint_; }
  const Point &endPoint() const { return endPoint_; }

  // 计算圆弧长度
  double length() const {
    double angle = angleSpan();
    return radius_ * angle;
  }

  // 计算圆弧角度跨度
  double angleSpan() const {
    double span = endAngle_ - startAngle_;
    if (span < 0) {
      span += 2 * PI;
    }
    return span;
  }

  // 计算圆弧上的点
  Point pointAt(double t) const {
    // t in [0, 1]
    double angle = startAngle_ + t * angleSpan();

    // 计算在XY平面上的点
    Point p = center_ + radius_ * Vector3(std::cos(angle), std::sin(angle), 0);

    // 如果法向量不是Z轴，需要旋转点
    if (normal_ != Vector3(0, 0, 1)) {
      // 计算从Z轴到normal_的旋转
      Vector3 rotAxis = Vector3(0, 0, 1).cross(normal_);
      double rotAngle = std::acos(Vector3(0, 0, 1).dot(normal_));

      if (rotAxis.norm() > EPSILON) {
        Quaternion q(Eigen::AngleAxisd(rotAngle, rotAxis.normalized()));
        p = center_ + q * (p - center_);
      }
    }

    return p;
  }

  // 计算点在圆弧上的参数t
  // 如果点不在圆弧上，返回最近点的参数
  double parameterAt(const Point &p) const {
    // 将点投影到圆弧平面
    Vector3 v = p - center_;
    double dist = v.dot(normal_);
    Point projectedPoint = p - dist * normal_;

    // 计算点到圆心的向量
    Vector3 radialVector = projectedPoint - center_;

    // 检查点是否在圆上
    if (std::abs(radialVector.norm() - radius_) > EPSILON) {
      // 点不在圆上，找最近点
      radialVector.normalize();
      projectedPoint = center_ + radius_ * radialVector;
    }

    // 计算角度
    double angle;
    if (normal_ == Vector3(0, 0, 1)) {
      // 如果法向量是Z轴，直接计算角度
      angle = std::atan2(radialVector.y(), radialVector.x());
    } else {
      // 否则需要将向量投影到法向量定义的平面
      // 创建一个局部坐标系
      Vector3 xAxis = Vector3(1, 0, 0);
      if (std::abs(normal_.dot(xAxis)) > 0.9) {
        xAxis = Vector3(0, 1, 0);
      }

      Vector3 yAxis = normal_.cross(xAxis).normalized();
      xAxis = yAxis.cross(normal_).normalized();

      // 计算在局部坐标系中的角度
      double x = radialVector.dot(xAxis);
      double y = radialVector.dot(yAxis);
      angle = std::atan2(y, x);
    }

    // 确保角度在[0, 2π]范围内
    while (angle < 0)
      angle += 2 * PI;
    while (angle >= 2 * PI)
      angle -= 2 * PI;

    // 计算参数t
    double span = angleSpan();
    double relativeAngle;

    if (endAngle_ >= startAngle_) {
      relativeAngle = angle - startAngle_;
      if (relativeAngle < 0)
        relativeAngle += 2 * PI;
    } else {
      relativeAngle = angle - startAngle_;
      if (relativeAngle < 0)
        relativeAngle += 2 * PI;
    }

    double t = relativeAngle / span;
    return std::clamp(t, 0.0, 1.0);
  }

  // 计算点到圆弧的最近点
  Point closestPoint(const Point &p) const {
    double t = parameterAt(p);
    return pointAt(t);
  }

  // IGeometry接口实现
  double distanceTo(const Point &p) const override {
    Point closest = closestPoint(p);
    return (p - closest).norm();
  }

  std::optional<Point> intersectWith(const Ray &ray) const override {
    // 将射线投影到圆弧平面
    double denom = ray.direction().dot(normal_);

    // 如果射线与平面平行
    if (std::abs(denom) < EPSILON) {
      return std::nullopt;
    }

    // 计算射线与平面的交点
    double t = (center_.dot(normal_) - ray.origin().dot(normal_)) / denom;

    // 如果交点在射线的负方向
    if (t < 0) {
      return std::nullopt;
    }

    // 计算交点
    Point intersection = ray.pointAt(t);

    // 检查交点是否在圆上
    double distToCenter = (intersection - center_).norm();
    if (std::abs(distToCenter - radius_) > EPSILON) {
      return std::nullopt;
    }

    // 检查交点是否在圆弧上
    double param = parameterAt(intersection);
    if (param < 0 || param > 1) {
      return std::nullopt;
    }

    return intersection;
  }

  BoundingBox getBoundingBox() const override {
    BoundingBox box;

    // 添加起点和终点
    box.extend(startPoint_);
    box.extend(endPoint_);

    // 检查圆弧是否跨越坐标轴
    double span = angleSpan();
    double angle = startAngle_;

    // 检查每个90度点
    for (int i = 0; i < 4; ++i) {
      double checkAngle = i * PI / 2;

      // 确保角度在[0, 2π]范围内
      while (checkAngle < 0)
        checkAngle += 2 * PI;
      while (checkAngle >= 2 * PI)
        checkAngle -= 2 * PI;

      // 检查这个角度是否在圆弧范围内
      bool inArc = false;
      if (endAngle_ >= startAngle_) {
        inArc = (checkAngle >= startAngle_ && checkAngle <= endAngle_);
      } else {
        inArc = (checkAngle >= startAngle_ || checkAngle <= endAngle_);
      }

      if (inArc) {
        box.extend(pointAt((checkAngle - startAngle_) / span));
      }
    }

    return box;
  }

  void transform(const Transform &t) override {
    center_ = t * center_;

    // 变换法向量（只考虑旋转部分）
    Matrix3 rotation = t.linear();
    normal_ = (rotation * normal_).normalized();

    // 重新计算起点和终点
    updatePoints();
  }

private:
  Point center_;
  double radius_;
  double startAngle_;
  double endAngle_;
  Vector3 normal_;
  Point startPoint_;
  Point endPoint_;

  void updatePoints() {
    startPoint_ = pointAt(0.0);
    endPoint_ = pointAt(1.0);
  }
};

} // namespace ocl

#endif // OCL_ARC_HPP