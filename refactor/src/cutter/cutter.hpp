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

#ifndef OCL_CUTTER_HPP
#define OCL_CUTTER_HPP

#include "common/types.hpp"
#include "geo/cutter_point.hpp"
#include "geo/mesh.hpp"
#include <memory>
#include <vector>

namespace ocl {

// 刀具类型枚举
enum class CutterType {
  Cylindrical, // 圆柱形刀具
  Ball,        // 球形刀具
  Bull,        // 牛鼻刀具（圆角端铣刀）
  Cone,        // 锥形刀具
  Torus        // 环形刀具
};

// to ostream
inline std::ostream &operator<<(std::ostream &os, const CutterType &type) {
  switch (type) {
  case CutterType::Cylindrical:
    os << "Cylindrical";
    break;
  case CutterType::Ball:
    os << "Ball";
    break;
  case CutterType::Bull:
    os << "Bull";
    break;
  case CutterType::Cone:
    os << "Cone";
    break;
  case CutterType::Torus:
    os << "Torus";
    break;
  }
  return os;
}

// 刀具接口
class ICutter {
public:
  virtual ~ICutter() = default;

  // 获取刀具类型
  virtual CutterType getType() const = 0;

  // 获取刀具直径
  virtual double getDiameter() const = 0;

  // 获取刀具长度
  virtual double getLength() const = 0;

  // 计算给定半径处的刀具高度
  virtual double height(double r) const = 0;

  // 计算给定高度处的刀具宽度
  virtual double width(double h) const = 0;

  // 计算刀具与三角形的接触点
  virtual CutterPoint dropCutter(const Point &point,
                                 const Triangle &triangle) const = 0;

  // 计算刀具与网格的接触点
  virtual CutterPoint dropCutter(const Point &point,
                                 const IMesh &mesh) const = 0;

  // 批量计算刀具与网格的接触点
  virtual std::vector<CutterPoint> dropCutter(const std::vector<Point> &points,
                                              const IMesh &mesh) const = 0;

  // 创建刀具的几何表示
  virtual std::shared_ptr<IMesh> createMesh(double resolution) const = 0;
};

} // namespace ocl

#endif // OCL_CUTTER_HPP