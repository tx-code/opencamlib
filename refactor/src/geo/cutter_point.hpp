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

#ifndef OCL_CUTTER_POINT_HPP
#define OCL_CUTTER_POINT_HPP

#include "common/types.hpp"

namespace ocl {

// 刀具接触类型枚举
enum class CCType {
  NONE, // 无接触

  // 圆柱形刀具接触类型
  VERTEX_CYL, // 顶点接触圆柱形刀具
  EDGE_CYL,   // 边接触圆柱形刀具
  EDGE_SHAFT, // 边接触圆柱形刀具轴
  FACET_CYL,  // 面接触圆柱形刀具

  // 球形刀具接触类型
  VERTEX_BALL, // 顶点接触球形刀具
  EDGE_BALL,   // 边接触球形刀具
  FACET_BALL,  // 面接触球形刀具

  // 牛鼻刀具接触类型
  VERTEX_FLAT, // 顶点接触牛鼻刀具平坦部分
  VERTEX_BULL, // 顶点接触牛鼻刀具圆角部分
  EDGE_FLAT,   // 边接触牛鼻刀具平坦部分
  EDGE_BULL,   // 边接触牛鼻刀具圆角部分
  FACET_FLAT,  // 面接触牛鼻刀具平坦部分
  FACET_BULL,  // 面接触牛鼻刀具圆角部分

  // 锥形刀具接触类型
  VERTEX_CONE, // 顶点接触锥形刀具
  EDGE_CONE,   // 边接触锥形刀具
  FACET_CONE,  // 面接触锥形刀具

  // 环形刀具接触类型
  VERTEX_TORUS, // 顶点接触环形刀具
  EDGE_TORUS,   // 边接触环形刀具
  FACET_TORUS   // 面接触环形刀具
};

// CCTyte to ostream
inline std::ostream &operator<<(std::ostream &os, const CCType &type) {
  switch (type) {
  case CCType::NONE:
    os << "NONE";
    break;
  case CCType::VERTEX_CYL:
    os << "VERTEX_CYL";
    break;
  case CCType::EDGE_CYL:
    os << "EDGE_CYL";
    break;
  case CCType::EDGE_SHAFT:
    os << "EDGE_SHAFT";
    break;
  case CCType::FACET_CYL:
    os << "FACET_CYL";
    break;
  case CCType::VERTEX_BALL:
    os << "VERTEX_BALL";
    break;
  case CCType::EDGE_BALL:
    os << "EDGE_BALL";
    break;
  case CCType::FACET_BALL:
    os << "FACET_BALL";
    break;
  case CCType::VERTEX_FLAT:
    os << "VERTEX_FLAT";
    break;
  case CCType::VERTEX_BULL:
    os << "VERTEX_BULL";
    break;
  case CCType::EDGE_FLAT:
    os << "EDGE_FLAT";
    break;
  case CCType::EDGE_BULL:
    os << "EDGE_BULL";
    break;
  case CCType::FACET_FLAT:
    os << "FACET_FLAT";
    break;
  case CCType::FACET_BULL:
    os << "FACET_BULL";
    break;
  case CCType::VERTEX_CONE:
    os << "VERTEX_CONE";
    break;
  case CCType::EDGE_CONE:
    os << "EDGE_CONE";
    break;
  case CCType::FACET_CONE:
    os << "FACET_CONE";
    break;
  case CCType::VERTEX_TORUS:
    os << "VERTEX_TORUS";
    break;
  case CCType::EDGE_TORUS:
    os << "EDGE_TORUS";
    break;
  case CCType::FACET_TORUS:
    os << "FACET_TORUS";
    break;
  }
  return os;
}

/**
 * @class CutterPoint
 * @brief 表示刀具与工件接触点的类
 *
 * 这个类包含了两个重要的点：
 * 1. 刀具位置点（CL点，Cutter Location
 * Point）：刀具中心轴上的点，通常是刀具最低点在Z轴方向的投影
 * 2. 刀具接触点（CC点，Cutter Contact
 * Point）：刀具与工件实际接触的点，位于刀具表面
 *
 * 这两个点对于CNC路径规划非常重要，CL点用于控制刀具的位置，而CC点提供了刀具与工件接触的精确位置。
 */
class CutterPoint {
public:
  /**
   * @brief 构造函数
   * @param clPoint 刀具位置点（CL点）
   */
  CutterPoint(const Point &clPoint)
      : clPoint_(clPoint), ccPoint_(clPoint), normal_(0, 0, 1),
        ccType_(CCType::NONE) {}

  /**
   * @brief 获取刀具位置点（CL点）
   * @return 刀具位置点
   */
  const Point &getCL() const { return clPoint_; }

  /**
   * @brief 设置刀具位置点（CL点）
   * @param point 新的刀具位置点
   */
  void setCL(const Point &point) { clPoint_ = point; }

  /**
   * @brief 获取刀具接触点（CC点）
   * @return 刀具接触点
   */
  const Point &getCC() const { return ccPoint_; }

  /**
   * @brief 设置刀具接触点（CC点）
   * @param point 新的刀具接触点
   */
  void setCC(const Point &point) { ccPoint_ = point; }

  /**
   * @brief 获取法向量
   * @return 法向量
   */
  const Vector3 &getNormal() const { return normal_; }

  /**
   * @brief 设置法向量
   * @param normal 新的法向量
   */
  void setNormal(const Vector3 &normal) { normal_ = normal; }

  /**
   * @brief 获取接触类型
   * @return 接触类型
   */
  CCType getCCType() const { return ccType_; }

  /**
   * @brief 设置接触类型
   * @param type 新的接触类型
   */
  void setCCType(CCType type) { ccType_ = type; }

  /**
   * @brief 获取CL点的x坐标
   * @return x坐标
   */
  double x() const { return clPoint_.x(); }

  /**
   * @brief 获取CL点的y坐标
   * @return y坐标
   */
  double y() const { return clPoint_.y(); }

  /**
   * @brief 获取CL点的z坐标
   * @return z坐标
   */
  double z() const { return clPoint_.z(); }

private:
  Point clPoint_;  // 刀具位置点（CL点）
  Point ccPoint_;  // 刀具接触点（CC点）
  Vector3 normal_; // 法向量
  CCType ccType_;  // 接触类型
};

} // namespace ocl

#endif // OCL_CUTTER_POINT_HPP