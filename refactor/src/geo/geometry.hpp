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

#ifndef OCL_GEOMETRY_HPP
#define OCL_GEOMETRY_HPP

#include "../common/types.hpp"
#include <optional>

namespace ocl {

// 射线类
class Ray {
public:
  Ray(const Point &origin, const Vector3 &direction)
      : origin_(origin), direction_(direction.normalized()) {}

  const Point &origin() const { return origin_; }
  const Vector3 &direction() const { return direction_; }

  // 计算射线上的点
  Point pointAt(double t) const { return origin_ + t * direction_; }

private:
  Point origin_;
  Vector3 direction_;
};

// 包围盒类
class BoundingBox {
public:
  BoundingBox()
      : min_(Point::Constant(std::numeric_limits<double>::max())),
        max_(Point::Constant(std::numeric_limits<double>::lowest())) {}

  BoundingBox(const Point &min, const Point &max) : min_(min), max_(max) {}

  const Point &min() const { return min_; }
  const Point &max() const { return max_; }

  // 扩展包围盒以包含点p
  void extend(const Point &p) {
    min_ = min_.cwiseMin(p);
    max_ = max_.cwiseMax(p);
  }

  // 扩展包围盒以包含另一个包围盒
  void extend(const BoundingBox &box) {
    min_ = min_.cwiseMin(box.min_);
    max_ = max_.cwiseMax(box.max_);
  }

  // 检查点是否在包围盒内
  bool contains(const Point &p) const {
    return (p.array() >= min_.array()).all() &&
           (p.array() <= max_.array()).all();
  }

  // 检查包围盒是否与射线相交
  bool intersects(const Ray &ray, double &tMin, double &tMax) const {
    // 实现射线-包围盒相交测试
    Vector3 invDir(1.0 / ray.direction().x(), 1.0 / ray.direction().y(),
                   1.0 / ray.direction().z());
    Vector3 t0 = (min_ - ray.origin()).cwiseProduct(invDir);
    Vector3 t1 = (max_ - ray.origin()).cwiseProduct(invDir);

    Vector3 tMinVec = t0.cwiseMin(t1);
    Vector3 tMaxVec = t0.cwiseMax(t1);

    tMin = std::max(std::max(tMinVec.x(), tMinVec.y()), tMinVec.z());
    tMax = std::min(std::min(tMaxVec.x(), tMaxVec.y()), tMaxVec.z());

    return tMax >= tMin && tMax >= 0.0;
  }

  // 计算包围盒的中心点
  Point center() const { return (min_ + max_) * 0.5; }

  // 计算包围盒的尺寸
  Vector3 size() const { return max_ - min_; }

  // 检查包围盒是否为空
  bool isEmpty() const { return (max_.array() < min_.array()).any(); }

private:
  Point min_;
  Point max_;
};

// 几何对象接口
class IGeometry {
public:
  virtual ~IGeometry() = default;

  // 计算到点p的距离
  virtual double distanceTo(const Point &p) const = 0;

  // 与射线相交测试
  virtual std::optional<Point> intersectWith(const Ray &ray) const = 0;

  // 获取包围盒
  virtual BoundingBox getBoundingBox() const = 0;

  // 变换几何对象
  virtual void transform(const Transform &t) = 0;
};

// 三角形类
class Triangle : public IGeometry {
public:
  Triangle(const Point &v0, const Point &v1, const Point &v2)
      : v0_(v0), v1_(v1), v2_(v2) {
    updateNormal();
  }

  const Point &v0() const { return v0_; }
  const Point &v1() const { return v1_; }
  const Point &v2() const { return v2_; }
  const Vector3 &normal() const { return normal_; }

  // 计算三角形面积
  double area() const { return 0.5 * (v1_ - v0_).cross(v2_ - v0_).norm(); }

  // 检查点是否在三角形内
  bool contains(const Point &p) const {
    // 重心坐标法
    Vector3 v0v1 = v1_ - v0_;
    Vector3 v0v2 = v2_ - v0_;
    Vector3 v0p = p - v0_;

    double d00 = v0v1.dot(v0v1);
    double d01 = v0v1.dot(v0v2);
    double d11 = v0v2.dot(v0v2);
    double d20 = v0p.dot(v0v1);
    double d21 = v0p.dot(v0v2);

    double denom = d00 * d11 - d01 * d01;
    double v = (d11 * d20 - d01 * d21) / denom;
    double w = (d00 * d21 - d01 * d20) / denom;
    double u = 1.0 - v - w;

    return v >= 0.0 && w >= 0.0 && u >= 0.0;
  }

  // IGeometry接口实现
  double distanceTo(const Point &p) const override {
    // 计算点到三角形的距离
    Vector3 v0v1 = v1_ - v0_;
    Vector3 v0v2 = v2_ - v0_;
    Vector3 v0p = p - v0_;

    // 检查点在三角形平面上的投影是否在三角形内
    double a = v0v1.dot(v0v1);
    double b = v0v1.dot(v0v2);
    double c = v0v2.dot(v0v2);
    double d = v0v1.dot(v0p);
    double e = v0v2.dot(v0p);

    double det = a * c - b * b;
    double s = b * e - c * d;
    double t = b * d - a * e;

    if (s + t <= det) {
      if (s < 0) {
        if (t < 0) {
          // 区域4
          if (d < 0) {
            t = 0;
            s = -d >= a ? 1 : -d / a;
          } else {
            s = 0;
            t = e >= 0 ? 0 : (e >= -c ? -e / c : 1);
          }
        } else {
          // 区域3
          s = 0;
          t = e >= 0 ? 0 : (e >= -c ? -e / c : 1);
        }
      } else if (t < 0) {
        // 区域5
        t = 0;
        s = d >= 0 ? 0 : (-d >= a ? 1 : -d / a);
      } else {
        // 区域0
        double invDet = 1.0 / det;
        s *= invDet;
        t *= invDet;
      }
    } else {
      if (s < 0) {
        // 区域2
        double tmp0 = b + d;
        double tmp1 = c + e;
        if (tmp1 > tmp0) {
          double numer = tmp1 - tmp0;
          double denom = a - 2 * b + c;
          s = numer >= denom ? 1 : numer / denom;
          t = 1 - s;
        } else {
          s = 0;
          t = tmp1 <= 0 ? 1 : (e >= 0 ? 0 : -e / c);
        }
      } else if (t < 0) {
        // 区域6
        double tmp0 = b + e;
        double tmp1 = a + d;
        if (tmp1 > tmp0) {
          double numer = tmp1 - tmp0;
          double denom = a - 2 * b + c;
          t = numer >= denom ? 1 : numer / denom;
          s = 1 - t;
        } else {
          t = 0;
          s = tmp1 <= 0 ? 1 : (d >= 0 ? 0 : -d / a);
        }
      } else {
        // 区域1
        double numer = c + e - b - d;
        if (numer <= 0) {
          s = 0;
        } else {
          double denom = a - 2 * b + c;
          s = numer >= denom ? 1 : numer / denom;
        }
        t = 1 - s;
      }
    }

    // 计算最近点
    Point closest = v0_ + s * v0v1 + t * v0v2;

    // 返回距离
    return (p - closest).norm();
  }

  std::optional<Point> intersectWith(const Ray &ray) const override {
    // Möller–Trumbore算法
    Vector3 edge1 = v1_ - v0_;
    Vector3 edge2 = v2_ - v0_;
    Vector3 h = ray.direction().cross(edge2);
    double a = edge1.dot(h);

    // 如果射线与三角形平行
    if (std::abs(a) < EPSILON) {
      return std::nullopt;
    }

    double f = 1.0 / a;
    Vector3 s = ray.origin() - v0_;
    double u = f * s.dot(h);

    if (u < 0.0 || u > 1.0) {
      return std::nullopt;
    }

    Vector3 q = s.cross(edge1);
    double v = f * ray.direction().dot(q);

    if (v < 0.0 || u + v > 1.0) {
      return std::nullopt;
    }

    // 计算交点
    double t = f * edge2.dot(q);

    if (t > EPSILON) {
      return ray.pointAt(t);
    }

    return std::nullopt;
  }

  BoundingBox getBoundingBox() const override {
    BoundingBox box;
    box.extend(v0_);
    box.extend(v1_);
    box.extend(v2_);
    return box;
  }

  void transform(const Transform &t) override {
    v0_ = t * v0_;
    v1_ = t * v1_;
    v2_ = t * v2_;
    updateNormal();
  }

private:
  Point v0_, v1_, v2_;
  Vector3 normal_;

  void updateNormal() { normal_ = (v1_ - v0_).cross(v2_ - v0_).normalized(); }
};

} // namespace ocl

#endif // OCL_GEOMETRY_HPP