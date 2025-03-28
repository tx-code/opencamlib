#pragma once

#include <memory>
#include <string>
#include <vector>
#include <vtkActor.h>
#include <vtkSmartPointer.h>

#include "cutters/ballcutter.hpp"
#include "cutters/bullcutter.hpp"
#include "cutters/conecutter.hpp"
#include "cutters/cylcutter.hpp"
#include "cutters/millingcutter.hpp"
#include "geo/clpoint.hpp"

// 通用绘制铣刀函数，根据铣刀类型动态选择绘制方法
void UpdateCutterActor(vtkSmartPointer<vtkActor>& actor,
                       const ocl::MillingCutter& cutter,
                       const ocl::Point& p);

// 绘制圆柱铣刀(Cylindrical Cutter)
void UpdateCylCutter(vtkSmartPointer<vtkActor>& actor,
                     const ocl::CylCutter& cutter,
                     const ocl::Point& p);

// 绘制球头铣刀(Ball Cutter)
void UpdateBallCutter(vtkSmartPointer<vtkActor>& actor,
                      const ocl::BallCutter& cutter,
                      const ocl::Point& p);

// 绘制牛头铣刀(Bull Cutter)
void UpdateBullCutter(vtkSmartPointer<vtkActor>& actor,
                      const ocl::BullCutter& cutter,
                      const ocl::Point& p);

// 绘制锥形铣刀(Cone Cutter)
void UpdateConeCutter(vtkSmartPointer<vtkActor>& actor,
                      const ocl::ConeCutter& cutter,
                      const ocl::Point& p);
