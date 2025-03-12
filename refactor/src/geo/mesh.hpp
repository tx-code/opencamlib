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

#ifndef OCL_MESH_HPP
#define OCL_MESH_HPP

#include <fstream>
#include <igl/AABB.h>
#include <igl/point_mesh_squared_distance.h>
#include <igl/ray_mesh_intersect.h>
#include <igl/readStl.h>
#include <igl/writeStl.h>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "../common/types.hpp"
#include "geometry.hpp"

namespace ocl {

// 网格接口
class IMesh : public IGeometry {
public:
  virtual ~IMesh() = default;

  // 获取顶点和面数据
  virtual const VertexMatrix &vertices() const = 0;
  virtual const FaceMatrix &faces() const = 0;

  // 获取三角形数量
  virtual size_t triangleCount() const = 0;

  // 获取指定索引的三角形
  virtual Triangle getTriangle(size_t index) const = 0;

  // 保存到STL文件
  virtual bool saveToSTL(const std::string &filename) const = 0;
};

// 基于libigl的网格实现
class LibiglMesh : public IMesh {
public:
  // 默认构造函数
  LibiglMesh() { initializeAABBTree(); }

  // 从顶点和面构造
  LibiglMesh(const VertexMatrix &vertices, const FaceMatrix &faces)
      : vertices_(vertices), faces_(faces) {
    initializeAABBTree();
  }

  // 从STL文件加载
  static std::shared_ptr<LibiglMesh> fromSTL(const std::string &filename) {
    auto mesh = std::make_shared<LibiglMesh>();
    if (mesh->loadFromSTL(filename)) {
      return mesh;
    }
    return nullptr;
  }

  // 从STL文件加载
  bool loadFromSTL(const std::string &filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
      return false;
    }
    bool success = igl::readSTL(file, vertices_, faces_, normals_);
    if (success) {
      initializeAABBTree();
    }
    return success;
  }

  // 保存到STL文件
  bool saveToSTL(const std::string &filename) const override {
    return igl::writeSTL(filename, vertices_, faces_, normals_);
  }

  // 获取顶点和面数据
  const VertexMatrix &vertices() const override { return vertices_; }
  const FaceMatrix &faces() const override { return faces_; }

  // 获取三角形数量
  size_t triangleCount() const override { return faces_.rows(); }

  // 获取指定索引的三角形
  Triangle getTriangle(size_t index) const override {
    if (index >= triangleCount()) {
      throw std::out_of_range("Triangle index out of range");
    }

    const auto &face = faces_.row(index);
    return Triangle(vertices_.row(face(0)).transpose(),
                    vertices_.row(face(1)).transpose(),
                    vertices_.row(face(2)).transpose());
  }

  // IGeometry接口实现
  double distanceTo(const Point &p) const override {
    Eigen::VectorXd sqrDist;
    Eigen::VectorXi I;
    Eigen::MatrixXd C;

    igl::point_mesh_squared_distance(p.transpose(), vertices_, faces_, sqrDist,
                                     I, C);

    return std::sqrt(sqrDist(0));
  }

  std::optional<Point> intersectWith(const Ray &ray) const override {
    std::vector<igl::Hit> hits;

    if (igl::ray_mesh_intersect(ray.origin().transpose(),
                                ray.direction().transpose(), vertices_, faces_,
                                hits)) {

      if (!hits.empty()) {
        // 获取最近的交点
        const auto &hit = hits[0];
        double t = hit.t;
        return ray.pointAt(t);
      }
    }

    return std::nullopt;
  }

  BoundingBox getBoundingBox() const override {
    if (vertices_.rows() == 0) {
      return BoundingBox();
    }

    Point min = vertices_.colwise().minCoeff().transpose();
    Point max = vertices_.colwise().maxCoeff().transpose();

    return BoundingBox(min, max);
  }

  void transform(const Transform &t) override {
    assert(vertices_.cols() == 3);
    // 应用变换到所有顶点
    for (int i = 0; i < vertices_.rows(); ++i) {
      // 确保我们处理的是3D点
      Eigen::Vector3d vertex = vertices_.row(i).transpose();

      // 应用变换
      Eigen::Vector3d transformed = t * vertex;

      // 更新顶点
      vertices_.row(i) = transformed.transpose();
    }

    // 重新构建AABB树
    initializeAABBTree();
  }

private:
  VertexMatrix vertices_;
  FaceMatrix faces_;
  NormalMatrix normals_;
  igl::AABB<VertexMatrix, 3> aabbTree_;

  void initializeAABBTree() {
    if (vertices_.rows() > 0 && faces_.rows() > 0) {
      aabbTree_.init(vertices_, faces_);
    }
  }
};

// 工厂函数，用于创建网格
class MeshFactory {
public:
  // 从STL文件创建网格
  static std::shared_ptr<IMesh> createFromSTL(const std::string &filename) {
    return LibiglMesh::fromSTL(filename);
  }

  // 从顶点和面创建网格
  static std::shared_ptr<IMesh> createFromData(const VertexMatrix &vertices,
                                               const FaceMatrix &faces) {
    return std::make_shared<LibiglMesh>(vertices, faces);
  }

  // 创建空网格
  static std::shared_ptr<IMesh> createEmpty() {
    return std::make_shared<LibiglMesh>();
  }
};

} // namespace ocl

#endif // OCL_MESH_HPP