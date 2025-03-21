#pragma once

#include <codecvt>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "ocl_utils.h"

#include "algo/adaptivewaterline.hpp"
#include "algo/waterline.hpp"
#include "cutters/ballcutter.hpp"
#include "cutters/bullcutter.hpp"
#include "cutters/conecutter.hpp"
#include "cutters/cylcutter.hpp"
#include "dropcutter/adaptivepathdropcutter.hpp"
#include "dropcutter/pathdropcutter.hpp"
#include "geo/line.hpp"
#include "geo/path.hpp"
#include "geo/point.hpp"
#include "geo/stlreader.hpp"
#include "geo/stlsurf.hpp"
#include "ocl.hpp"

void hello_ocl() {
  spdlog::info("ocl version: {}", ocl::version());
  spdlog::info("max threads: {}", ocl::max_threads());
}

void printXYZ(ocl::Point point) {
  // printf("X%g ", round(point.x * 100000.0) / 100000.0);
  // printf("Y%g ", round(point.y * 100000.0) / 100000.0);
  // printf("Z%g", round(point.z * 100000.0) / 100000.0);
}

void linear(ocl::Point point) {
  // printf("G01 ");
  // printXYZ(point);
  // printf("\n");
}

void moveSafely(ocl::Point point) {
  // printf("G00 Z10\n");
  // printf("G00 ");
  // printf("X%g ", round(point.x * 100000.0) / 100000.0);
  // printf("Y%g\n", round(point.y * 100000.0) / 100000.0);
  // printf("G01 ");
  // printf("Z%g", round(point.z * 100000.0) / 100000.0);
  // printf(" F50\n");
}

void printPoints(std::vector<ocl::Point> points) {
  for (auto j = 0; j < points.size(); j++) {
    auto point = points[j];
    if (j == 0)
      moveSafely(point);
    else
      linear(point);
  }
}

void printPoints(std::vector<ocl::CLPoint> points) {
  for (auto j = 0; j < points.size(); j++) {
    auto point = points[j];
    if (j == 0)
      moveSafely(point);
    else
      linear(point);
  }
}

void printLoops(std::vector<std::vector<ocl::Point>> loops) {
  for (auto i = 0; i < loops.size(); i++) {
    printPoints(loops[i]);
  }
}

void drawLoops(VtkViewer &viewer,
               const std::vector<std::vector<ocl::Point>> &loops) {
  for (int loop_idx = 0; loop_idx < loops.size(); loop_idx++) {
    int nloop = 0;
    for (const auto &lop : loops) {
      int n = 0;
      int N = lop.size();
      ocl::Point first_point(-1, -1, 5);
      ocl::Point previous(-1, -1, 5);
      for (const auto &p : lop) {
        if (n == 0) { // don't draw anything on the first iteration
          previous = p;
          first_point = p;
        } else if (n == (N - 1)) {                          // the last point
          viewer.addActor(CreateLine(previous, p, yellow)); // the normal line
          // and a line from p to the first point
          viewer.addActor(CreateLine(p, first_point, yellow));
        } else {
          viewer.addActor(CreateLine(previous, p, yellow));
          previous = p;
        }
        n++;
      }
      std::cout << "rendered loop " << nloop << " with " << lop.size()
                << " points" << std::endl;
      nloop++;
    }
  }
}

void waterline(ocl::STLSurf surface, ocl::MillingCutter *cutter, double z,
               double sampling, VtkViewer *viewer = nullptr) {
  ocl::Waterline wl = ocl::Waterline();
  wl.setSTL(surface);
  wl.setCutter(cutter);
  wl.setSampling(sampling);
  for (double h = 0; h < z; h = h + 0.1) {
    wl.reset();
    wl.setZ(h);
    wl.run();
    auto loops = wl.getLoops();
    // printLoops(loops);
    if (viewer) {
      drawLoops(*viewer, loops);
    }
  }
}

void adaptiveWaterline(ocl::STLSurf surface, ocl::MillingCutter *cutter,
                       double z, double sampling, double minSampling) {
  ocl::AdaptiveWaterline awl = ocl::AdaptiveWaterline();
  awl.setSTL(surface);
  awl.setCutter(cutter);
  awl.setSampling(sampling);
  awl.setMinSampling(minSampling);
  for (double h = 0; h < z; h = h + 0.1) {
    awl.reset();
    awl.setZ(h);
    awl.run();
    auto loops = awl.getLoops();
    printLoops(loops);
  }
}

void pathDropCutter(ocl::STLSurf surface, ocl::MillingCutter *cutter,
                    double sampling, ocl::Path *path,
                    VtkViewer *viewer = nullptr) {
  ocl::PathDropCutter pdc = ocl::PathDropCutter();
  pdc.setSTL(surface);
  pdc.setCutter(cutter);
  pdc.setPath(path);
  pdc.setSampling(sampling);
  pdc.reset();
  pdc.setZ(0);
  pdc.run();
  auto points = pdc.getPoints();
  printPoints(points);

  if (viewer) {
    DrawCLPoints(*viewer, points);
  }
}

void adaptivePathDropCutter(ocl::STLSurf surface, ocl::MillingCutter *cutter,
                            double sampling, double minSampling,
                            ocl::Path *path) {
  ocl::AdaptivePathDropCutter apdc = ocl::AdaptivePathDropCutter();
  apdc.setSTL(surface);
  apdc.setCutter(cutter);
  apdc.setPath(path);
  apdc.setSampling(sampling);
  apdc.setMinSampling(minSampling);
  apdc.reset();
  apdc.setZ(0);
  apdc.run();
  auto points = apdc.getPoints();
  printPoints(points);
}

void ocl_all_algos_demo(VtkViewer &viewer) {
  spdlog::stopwatch sw;

  ocl::STLSurf surface = ocl::STLSurf();
  std::wstring stlPath = L"./stl/gnu_tux_mod.stl";
  ocl::STLReader(stlPath, surface);
  spdlog::info("surface size: {} in {} ms", surface.size(), sw);
  viewer.addActor(CreateStlActor(surface));

  ocl::CylCutter cylCutter = ocl::CylCutter(0.4, 10);
  ocl::BallCutter ballCutter = ocl::BallCutter(4, 20);
  ocl::BullCutter bullCutter = ocl::BullCutter(4, 0.05, 20);
  ocl::ConeCutter coneCutter = ocl::ConeCutter(4, 0.05, 20);
  std::vector<ocl::MillingCutter *> cutters;
  cutters.push_back(&cylCutter);
  cutters.push_back(&ballCutter);
  cutters.push_back(&bullCutter);
  cutters.push_back(&coneCutter);
  double z = 0.5;
  double sampling = 0.1;
  for (auto cutter : cutters) {
    spdlog::info("WL + Cutter: {}", cutter->str());
    waterline(surface, cutter, z, sampling);
    spdlog::info("WL done in {} ms", sw);
  }
  double minSampling = 0.01;
  for (auto cutter : cutters) {
    spdlog::info("AWL + Cutter: {}", cutter->str());
    adaptiveWaterline(surface, cutter, z, sampling, minSampling);
    spdlog::info("AWL done in {} ms", sw);
  }
  ocl::Path path = ocl::Path();
  int i = 0;
  for (double y = 0; y <= 0.2; y = y + 0.1) {
    bool ltr = ((int)i % 2) == 0;
    ocl::Point p1 = ocl::Point(ltr ? -2 : 11, y, 0);
    ocl::Point p2 = ocl::Point(ltr ? 11 : -2, y, 0);
    ocl::Line l = ocl::Line(p1, p2);
    path.append(l);
    ocl::Point p3 = ocl::Point(ltr ? 11 : -2, y + 1, 0);
    ocl::Line l2 = ocl::Line(p2, p3);
    path.append(l2);
    i++;
  }
  for (auto cutter : cutters) {
    spdlog::info("PDC + Cutter: {}", cutter->str());
    pathDropCutter(surface, cutter, sampling, &path);
    spdlog::info("PDC done in {} ms", sw);
  }
  for (auto cutter : cutters) {
    spdlog::info("APDC: {}", cutter->str());
    adaptivePathDropCutter(surface, cutter, sampling, minSampling, &path);
    spdlog::info("APDC done in {} ms", sw);
  }
}

// 导入STL模型并创建VTK可视化表面
ocl::STLSurf loadSTLModel(VtkViewer &viewer, const std::wstring &stlPath) {
  spdlog::stopwatch sw;
  ocl::STLSurf surface = ocl::STLSurf();
  ocl::STLReader(stlPath, surface);
  spdlog::info(
      "Loading STL model: {} Triangle count: {} Time: {} ms",
      std::wstring_convert<std::codecvt_utf8<wchar_t>>().to_bytes(stlPath),
      surface.size(), sw);
  viewer.addActor(CreateStlActor(surface));
  return surface;
}

// 创建标准测试路径
ocl::Path createTestPath() {
  ocl::Path path = ocl::Path();
  int i = 0;
  for (double y = 0; y <= 0.2; y = y + 0.1) {
    bool ltr = ((int)i % 2) == 0;
    ocl::Point p1 = ocl::Point(ltr ? -2 : 11, y, 0);
    ocl::Point p2 = ocl::Point(ltr ? 11 : -2, y, 0);
    ocl::Line l = ocl::Line(p1, p2);
    path.append(l);
    ocl::Point p3 = ocl::Point(ltr ? 11 : -2, y + 1, 0);
    ocl::Line l2 = ocl::Line(p2, p3);
    path.append(l2);
    i++;
  }
  return path;
}

// 圆柱铣刀水平等高线切削演示
void cylCutter_waterline_demo(VtkViewer &viewer) {
  spdlog::stopwatch sw;
  std::wstring stlPath = L"./stl/gnu_tux_mod.stl";
  ocl::STLSurf surface = loadSTLModel(viewer, stlPath);
  auto height = surface.bb.maxpt.z - surface.bb.minpt.z;

  ocl::CylCutter cylCutter = ocl::CylCutter(0.4, 10);
  DrawCylCutter(viewer, cylCutter, ocl::Point(0, 0, 0));
  // double z = 0.5;
  double z = height;
  double sampling = 0.1;

  spdlog::info("Cylindrical Cutter Waterline: {}", cylCutter.str());
  waterline(surface, &cylCutter, z, sampling, &viewer);
  spdlog::info("Waterline operation completed in {} ms", sw);
}

// 球头铣刀水平等高线切削演示
void ballCutter_waterline_demo(VtkViewer &viewer) {
  spdlog::stopwatch sw;
  std::wstring stlPath = L"./stl/gnu_tux_mod.stl";
  ocl::STLSurf surface = loadSTLModel(viewer, stlPath);

  ocl::BallCutter ballCutter = ocl::BallCutter(4, 20);
  double z = 0.5;
  double sampling = 0.1;

  spdlog::info("Ball Cutter Waterline: {}", ballCutter.str());
  waterline(surface, &ballCutter, z, sampling);
  spdlog::info("Waterline operation completed in {} ms", sw);
}

// 牛头铣刀自适应水平等高线切削演示
void bullCutter_adaptiveWaterline_demo(VtkViewer &viewer) {
  spdlog::stopwatch sw;
  std::wstring stlPath = L"./stl/gnu_tux_mod.stl";
  ocl::STLSurf surface = loadSTLModel(viewer, stlPath);

  ocl::BullCutter bullCutter = ocl::BullCutter(4, 0.05, 20);
  double z = 0.5;
  double sampling = 0.1;
  double minSampling = 0.01;

  spdlog::info("Bull Cutter Adaptive Waterline: {}", bullCutter.str());
  adaptiveWaterline(surface, &bullCutter, z, sampling, minSampling);
  spdlog::info("Adaptive waterline operation completed in {} ms", sw);
}

// 圆锥铣刀路径降刀演示
void coneCutter_pathDropCutter_demo(VtkViewer &viewer) {
  spdlog::stopwatch sw;
  std::wstring stlPath = L"./stl/gnu_tux_mod.stl";
  ocl::STLSurf surface = loadSTLModel(viewer, stlPath);

  ocl::ConeCutter coneCutter = ocl::ConeCutter(4, 0.05, 20);
  double sampling = 0.1;
  ocl::Path path = createTestPath();

  spdlog::info("Cone Cutter PathDropCutter: {}", coneCutter.str());
  pathDropCutter(surface, &coneCutter, sampling, &path, &viewer);
  spdlog::info("PathDropCutter operation completed in {} ms", sw);
}

// 球头铣刀自适应路径降刀演示
void ballCutter_adaptivePathDropCutter_demo(VtkViewer &viewer) {
  spdlog::stopwatch sw;
  std::wstring stlPath = L"./stl/gnu_tux_mod.stl";
  ocl::STLSurf surface = loadSTLModel(viewer, stlPath);

  ocl::BallCutter ballCutter = ocl::BallCutter(4, 20);
  double sampling = 0.1;
  double minSampling = 0.01;
  ocl::Path path = createTestPath();

  spdlog::info("Ball Cutter Adaptive PathDropCutter: {}", ballCutter.str());
  adaptivePathDropCutter(surface, &ballCutter, sampling, minSampling, &path);
  spdlog::info("Adaptive PathDropCutter operation completed in {} ms", sw);
}