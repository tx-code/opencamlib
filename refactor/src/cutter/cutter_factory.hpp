#pragma once

#include "ball_cutter.hpp"
#include "bull_cutter.hpp"
#include "cone_cutter.hpp"
#include "cutter.hpp"
#include "cylindrical_cutter.hpp"
#include "torus_cutter.hpp"

namespace ocl {
// 刀具工厂类
class CutterFactory {
public:
  // 创建刀具的工厂方法
  static std::shared_ptr<ICutter> createCutter(CutterType type, double diameter,
                                               double length,
                                               double param = 0.0) {
    switch (type) {
    case CutterType::Cylindrical:

      return std::make_shared<CylindricalCutter>(diameter, length);
    case CutterType::Ball:
      return std::make_shared<BallCutter>(diameter, length);

    case CutterType::Bull:
      // 对于牛鼻刀具，param是圆角半径
      return std::make_shared<BullCutter>(diameter, param, length);

    case CutterType::Cone:
      // 对于锥形刀具，param是锥角（弧度）
      return std::make_shared<ConeCutter>(diameter, param, length);

    case CutterType::Torus:
      // 对于环形刀具，param是环形半径
      return std::make_shared<TorusCutter>(diameter, param, length);

    default:
      // 默认返回圆柱形刀具
      return std::make_shared<CylindricalCutter>(diameter, length);
    }
  }
};

} // namespace ocl
