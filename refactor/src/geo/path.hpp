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

#ifndef OCL_PATH_HPP
#define OCL_PATH_HPP

#include "../common/types.hpp"
#include "arc.hpp"
#include "geometry.hpp"
#include "line.hpp"
#include <memory>
#include <optional>
#include <variant>
#include <vector>

namespace ocl {

// 路径段类型
using PathSegment = std::variant<Line, Arc>;

// 路径类
class Path : public IGeometry {
public:
  // 默认构造函数
  Path() = default;

  // 添加线段
  void addLine(const Point &p1, const Point &p2) {
    if (segments_.empty()) {
      segments_.push_back(Line(p1, p2));
    } else {
      // 确保与上一段连接
      const Point &lastPoint = getLastPoint();
      if ((lastPoint - p1).norm() > EPSILON) {
        throw std::invalid_argument(
            "Line does not connect to the last segment");
      }
      segments_.push_back(Line(p1, p2));
    }
  }

  // 添加圆弧
  void addArc(const Point &center, double radius, double startAngle,
              double endAngle, const Vector3 &normal = Vector3(0, 0, 1)) {
    Arc arc(center, radius, startAngle, endAngle, normal);

    if (segments_.empty()) {
      segments_.push_back(arc);
    } else {
      // 确保与上一段连接
      const Point &lastPoint = getLastPoint();
      if ((lastPoint - arc.startPoint()).norm() > EPSILON) {
        throw std::invalid_argument("Arc does not connect to the last segment");
      }
      segments_.push_back(arc);
    }
  }

  // 添加圆弧（通过起点、终点和圆心）
  void addArc(const Point &start, const Point &end, const Point &center) {
    Arc arc = Arc::fromPoints(start, end, center);

    if (segments_.empty()) {
      segments_.push_back(arc);
    } else {
      // 确保与上一段连接
      const Point &lastPoint = getLastPoint();
      if ((lastPoint - start).norm() > EPSILON) {
        throw std::invalid_argument("Arc does not connect to the last segment");
      }
      segments_.push_back(arc);
    }
  }

  // 获取路径段数量
  size_t size() const { return segments_.size(); }

  // 获取指定索引的路径段
  const PathSegment &getSegment(size_t index) const {
    if (index >= segments_.size()) {
      throw std::out_of_range("Segment index out of range");
    }
    return segments_[index];
  }

  // 获取所有路径段
  const std::vector<PathSegment> &getSegments() const { return segments_; }

  // 获取路径的起点
  Point getStartPoint() const {
    if (segments_.empty()) {
      throw std::runtime_error("Path is empty");
    }

    return getSegmentStartPoint(segments_.front());
  }

  // 获取路径的终点
  Point getEndPoint() const {
    if (segments_.empty()) {
      throw std::runtime_error("Path is empty");
    }

    return getSegmentEndPoint(segments_.back());
  }

  // 计算路径长度
  double length() const {
    double totalLength = 0.0;

    for (const auto &segment : segments_) {
      std::visit([&totalLength](const auto &s) { totalLength += s.length(); },
                 segment);
    }

    return totalLength;
  }

  // 计算路径上的点
  Point pointAt(double t) const {
    if (segments_.empty()) {
      throw std::runtime_error("Path is empty");
    }

    if (t <= 0.0) {
      return getSegmentStartPoint(segments_.front());
    }

    if (t >= 1.0) {
      return getSegmentEndPoint(segments_.back());
    }

    // 计算每个段的长度和总长度
    std::vector<double> segmentLengths;
    double totalLength = 0.0;

    for (const auto &segment : segments_) {
      double length =
          std::visit([](const auto &s) { return s.length(); }, segment);
      segmentLengths.push_back(length);
      totalLength += length;
    }

    // 计算目标距离
    double targetDistance = t * totalLength;

    // 找到对应的段
    double accumulatedLength = 0.0;
    for (size_t i = 0; i < segments_.size(); ++i) {
      double nextLength = accumulatedLength + segmentLengths[i];

      if (targetDistance <= nextLength || i == segments_.size() - 1) {
        // 计算段内的参数
        double segmentT =
            segmentLengths[i] > EPSILON
                ? (targetDistance - accumulatedLength) / segmentLengths[i]
                : 0.0;

        // 计算点
        return std::visit(
            [segmentT](const auto &s) { return s.pointAt(segmentT); },
            segments_[i]);
      }

      accumulatedLength = nextLength;
    }

    // 不应该到达这里
    return getSegmentEndPoint(segments_.back());
  }

  // 计算点到路径的最近点和距离
  std::tuple<Point, size_t, double> closestPoint(const Point &p) const {
    if (segments_.empty()) {
      throw std::runtime_error("Path is empty");
    }

    Point closestPoint;
    size_t closestSegmentIndex = 0;
    double minDistance = std::numeric_limits<double>::max();

    for (size_t i = 0; i < segments_.size(); ++i) {
      Point segmentClosest = std::visit(
          [&p](const auto &s) { return s.closestPoint(p); }, segments_[i]);
      double distance = (p - segmentClosest).norm();

      if (distance < minDistance) {
        minDistance = distance;
        closestPoint = segmentClosest;
        closestSegmentIndex = i;
      }
    }

    return {closestPoint, closestSegmentIndex, minDistance};
  }

  // IGeometry接口实现
  double distanceTo(const Point &p) const override {
    auto [_, __, dist] = closestPoint(p);
    return dist;
  }

  std::optional<Point> intersectWith(const Ray &ray) const override {
    std::optional<Point> closestIntersection;
    double minDistance = std::numeric_limits<double>::max();

    for (const auto &segment : segments_) {
      std::optional<Point> intersection = std::visit(
          [&ray](const auto &s) { return s.intersectWith(ray); }, segment);

      if (intersection) {
        double distance = (ray.origin() - *intersection).norm();
        if (!closestIntersection || distance < minDistance) {
          closestIntersection = intersection;
          minDistance = distance;
        }
      }
    }

    return closestIntersection;
  }

  BoundingBox getBoundingBox() const override {
    if (segments_.empty()) {
      return BoundingBox();
    }

    BoundingBox box;
    for (const auto &segment : segments_) {
      std::visit([&box](const auto &s) { box.extend(s.getBoundingBox()); },
                 segment);
    }
    return box;
  }

  void transform(const Transform &t) override {
    for (auto &segment : segments_) {
      std::visit([&t](auto &s) { s.transform(t); }, segment);
    }
  }

  // 检查路径是否闭合
  bool isClosed() const {
    if (segments_.size() < 2) {
      return false;
    }

    const Point &start = getStartPoint();
    const Point &end = getEndPoint();

    return (start - end).norm() < EPSILON;
  }

  // 闭合路径（如果尚未闭合）
  void close() {
    if (segments_.empty()) {
      return;
    }

    if (!isClosed()) {
      const Point &start = getStartPoint();
      const Point &end = getEndPoint();
      addLine(end, start);
    }
  }

private:
  std::vector<PathSegment> segments_;

  // 获取路径段的起点
  static Point getSegmentStartPoint(const PathSegment &segment) {
    return std::visit(
        [](const auto &s) -> Point {
          if constexpr (std::is_same_v<std::decay_t<decltype(s)>, Line>) {
            return s.p1();
          } else if constexpr (std::is_same_v<std::decay_t<decltype(s)>, Arc>) {
            return s.startPoint();
          }
        },
        segment);
  }

  // 获取路径段的终点
  static Point getSegmentEndPoint(const PathSegment &segment) {
    return std::visit(
        [](const auto &s) -> Point {
          if constexpr (std::is_same_v<std::decay_t<decltype(s)>, Line>) {
            return s.p2();
          } else if constexpr (std::is_same_v<std::decay_t<decltype(s)>, Arc>) {
            return s.endPoint();
          }
        },
        segment);
  }

  // 获取最后一个路径段的终点
  Point getLastPoint() const {
    if (segments_.empty()) {
      throw std::runtime_error("Path is empty");
    }
    return getSegmentEndPoint(segments_.back());
  }
};

} // namespace ocl

#endif // OCL_PATH_HPP