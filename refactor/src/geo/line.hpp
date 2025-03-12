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

#ifndef OCL_LINE_HPP
#define OCL_LINE_HPP

#include "../common/types.hpp"
#include "geometry.hpp"
#include <optional>

namespace ocl {

// 线段类
class Line : public IGeometry {
public:
  Line(const Point &p1, const Point &p2) : p1_(p1), p2_(p2) {}

  const Point &p1() const { return p1_; }
  const Point &p2() const { return p2_; }

  // 计算线段长度
  double length() const { return (p2_ - p1_).norm(); }

  // 计算线段方向向量
  Vector3 direction() const { return (p2_ - p1_).normalized(); }

  // 计算线段上的点
  Point pointAt(double t) const {
    // t in [0, 1]
    return p1_ + t * (p2_ - p1_);
  }

  // 计算点在线段上的参数t
  // 如果点不在线段上，返回最近点的参数
  double parameterAt(const Point &p) const {
    Vector3 v = p2_ - p1_;
    double l2 = v.squaredNorm();

    if (l2 < EPSILON) {
      return 0.0; // 线段退化为点
    }

    double t = (p - p1_).dot(v) / l2;
    return std::clamp(t, 0.0, 1.0);
  }

  // 计算点到线段的最近点
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
    // 线段可以看作参数方程 p(s) = p1 + s * (p2 - p1), s in [0, 1]
    // 射线可以看作参数方程 q(t) = origin + t * direction, t >= 0
    // 求解 p(s) = q(t)

    Vector3 v = p2_ - p1_;
    Vector3 w = ray.origin() - p1_;
    Vector3 u = ray.direction();

    double a = v.dot(v);
    double b = v.dot(u);
    double c = u.dot(u);
    double d = v.dot(w);
    double e = u.dot(w);

    double denom = a * c - b * b;

    // 如果线段和射线几乎平行
    if (std::abs(denom) < EPSILON) {
      return std::nullopt;
    }

    double s = (b * e - c * d) / denom;
    double t = (a * e - b * d) / denom;

    // 检查s和t是否在有效范围内
    if (s < 0.0 || s > 1.0 || t < 0.0) {
      return std::nullopt;
    }

    // 计算交点
    Point intersection = p1_ + s * v;

    // 检查交点是否真的在线段和射线上
    double distToRay = (intersection - ray.pointAt(t)).norm();
    double distToLine = (intersection - pointAt(s)).norm();

    if (distToRay < EPSILON && distToLine < EPSILON) {
      return intersection;
    }

    return std::nullopt;
  }

  BoundingBox getBoundingBox() const override {
    BoundingBox box;
    box.extend(p1_);
    box.extend(p2_);
    return box;
  }

  void transform(const Transform &t) override {
    p1_ = t * p1_;
    p2_ = t * p2_;
  }

private:
  Point p1_;
  Point p2_;
};

} // namespace ocl

#endif // OCL_LINE_HPP