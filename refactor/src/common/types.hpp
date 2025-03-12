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

#ifndef OCL_TYPES_HPP
#define OCL_TYPES_HPP

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <optional>
#include <vector>

namespace ocl {

// 基本几何类型定义
using Point = Eigen::Vector3d;
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;
using Matrix4 = Eigen::Matrix4d;
using Quaternion = Eigen::Quaterniond;
using Transform = Eigen::Affine3d;
using AlignedBox3 = Eigen::AlignedBox3d;

// 网格数据类型
using VertexMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, 3,
                  Eigen::RowMajor>; // n x 3 矩阵，每行是一个顶点
using FaceMatrix =
    Eigen::Matrix<int, Eigen::Dynamic, 3,
                  Eigen::RowMajor>; // m x 3 矩阵，每行是一个面的三个顶点索引
using NormalMatrix =
    Eigen::Matrix<double, Eigen::Dynamic, 3,
                  Eigen::RowMajor>; // n x 3 矩阵，每行是一个法向量

// 常用常量
constexpr double PI = 3.14159265358979323846;
constexpr double EPSILON = 1e-10;

// 实用函数
inline bool isZero(double value) { return std::abs(value) < EPSILON; }

inline bool isEqual(double a, double b) { return std::abs(a - b) < EPSILON; }

// 类型转换辅助函数
template <typename T> inline Point toPoint(const T &p) {
  return Point(p.x(), p.y(), p.z());
}

template <typename T> inline Vector3 toVector3(const T &v) {
  return Vector3(v.x(), v.y(), v.z());
}
} // namespace ocl

#endif // OCL_TYPES_HPP