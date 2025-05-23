﻿#include "oclUtils.h"

#include "algo/batchpushcutter.hpp"
#include "algo/fiberpushcutter.hpp"


// 添加随机数生成器相关头文件
#include <boost/math/constants/constants.hpp>
#include <functional>
#include <random>
#include <tbb/global_control.h>
#include <tbb/info.h>


namespace
{
void printStats(const std::vector<ocl::CLPoint>& points)
{
    spdlog::info("Statistics of the points:");
    std::array<double, ocl::CCType::CCTYPE_ERROR + 1> stats = {0};
    for (const auto& point : points) {
        const auto& cc = point.cc.load();
        stats[static_cast<int>(cc->type)]++;
    }
    for (int i = 0; i <= ocl::CCType::CCTYPE_ERROR; i++) {
        if (stats[i] > 0) {
            spdlog::info("{}: {}/{}, {:.2f}%",
                         ocl::CCType2String(static_cast<ocl::CCType>(i)),
                         stats[i],
                         points.size(),
                         stats[i] / points.size() * 100);
        }
    }
}


}  // namespace

void CAMModelManager::createCube(float length, float width, float height)
{
    stlFilePath.clear();
    surface = std::make_unique<ocl::STLSurf>();

    // Calculate half dimensions
    float hx = length / 2.0f;
    float hy = width / 2.0f;
    float hz = height / 2.0f;

    // Create triangles for a cube based on BuildRegularGeoms.py implementation

    // Bottom face
    surface->addTriangle(ocl::Point(-hx, -hy, -hz),
                         ocl::Point(hx, -hy, -hz),
                         ocl::Point(hx, -hy, hz));
    surface->addTriangle(ocl::Point(-hx, -hy, -hz),
                         ocl::Point(hx, -hy, hz),
                         ocl::Point(-hx, -hy, hz));

    // Top face
    surface->addTriangle(ocl::Point(-hx, hy, -hz), ocl::Point(hx, hy, hz), ocl::Point(hx, hy, -hz));
    surface->addTriangle(ocl::Point(-hx, hy, -hz), ocl::Point(-hx, hy, hz), ocl::Point(hx, hy, hz));

    // Left face
    surface->addTriangle(ocl::Point(-hx, -hy, -hz),
                         ocl::Point(-hx, hy, hz),
                         ocl::Point(-hx, hy, -hz));
    surface->addTriangle(ocl::Point(-hx, -hy, -hz),
                         ocl::Point(-hx, -hy, hz),
                         ocl::Point(-hx, hy, hz));

    // Right face
    surface->addTriangle(ocl::Point(hx, -hy, -hz), ocl::Point(hx, hy, -hz), ocl::Point(hx, hy, hz));
    surface->addTriangle(ocl::Point(hx, -hy, -hz), ocl::Point(hx, hy, hz), ocl::Point(hx, -hy, hz));

    // Front face
    surface->addTriangle(ocl::Point(-hx, -hy, -hz),
                         ocl::Point(-hx, hy, -hz),
                         ocl::Point(hx, hy, -hz));
    surface->addTriangle(ocl::Point(-hx, -hy, -hz),
                         ocl::Point(hx, hy, -hz),
                         ocl::Point(hx, -hy, -hz));

    // Back face
    surface->addTriangle(ocl::Point(-hx, -hy, hz), ocl::Point(hx, hy, hz), ocl::Point(-hx, hy, hz));
    surface->addTriangle(ocl::Point(-hx, -hy, hz), ocl::Point(hx, -hy, hz), ocl::Point(hx, hy, hz));

    // 重建AABB树
    rebuildAABBTree();

    spdlog::info("Created cube with dimensions: {} x {} x {}", length, width, height);
}

void CAMModelManager::createSphere(float radius, int count)
{
    stlFilePath.clear();
    surface = std::make_unique<ocl::STLSurf>();

    // Create sphere triangles
    for (int i = 0; i < count; i++) {
        double theta1 = boost::math::constants::pi<double>() * static_cast<double>(i) / count;
        double theta2 = boost::math::constants::pi<double>() * static_cast<double>(i + 1) / count;

        for (int j = 0; j < 2 * count; j++) {
            double phi1 =
                2.0 * boost::math::constants::pi<double>() * static_cast<double>(j) / (2 * count);
            double phi2 = 2.0 * boost::math::constants::pi<double>() * static_cast<double>(j + 1)
                / (2 * count);

            // Calculate points for two triangles
            double x1 = radius * sin(theta1) * cos(phi1);
            double y1 = radius * sin(theta1) * sin(phi1);
            double z1 = radius * cos(theta1);

            double x2 = radius * sin(theta2) * cos(phi1);
            double y2 = radius * sin(theta2) * sin(phi1);
            double z2 = radius * cos(theta2);

            double x3 = radius * sin(theta1) * cos(phi2);
            double y3 = radius * sin(theta1) * sin(phi2);
            double z3 = radius * cos(theta1);

            double x4 = radius * sin(theta2) * cos(phi2);
            double y4 = radius * sin(theta2) * sin(phi2);
            double z4 = radius * cos(theta2);

            // Add triangles, avoiding degenerate cases at the poles
            if (sin(theta1) != 0.0) {
                surface->addTriangle(ocl::Point(x1, y1, z1),
                                     ocl::Point(x2, y2, z2),
                                     ocl::Point(x3, y3, z3));
            }

            if (sin(theta2) != 0.0) {
                surface->addTriangle(ocl::Point(x2, y2, z2),
                                     ocl::Point(x4, y4, z4),
                                     ocl::Point(x3, y3, z3));
            }
        }
    }

    // 重建AABB树
    rebuildAABBTree();

    spdlog::info("Created sphere with radius: {}, resolution: {}", radius, count);
}

void CAMModelManager::createEllipsoid(float radius1, float radius2, int count)
{
    stlFilePath.clear();
    surface = std::make_unique<ocl::STLSurf>();

    // Create ellipsoid triangles using spherical coordinates
    for (int i = 0; i < count; i++) {
        double theta1 = boost::math::constants::pi<double>() * static_cast<double>(i) / count;
        double theta2 = boost::math::constants::pi<double>() * static_cast<double>(i + 1) / count;

        for (int j = 0; j < 2 * count; j++) {
            double phi1 =
                2.0 * boost::math::constants::pi<double>() * static_cast<double>(j) / (2 * count);
            double phi2 = 2.0 * boost::math::constants::pi<double>() * static_cast<double>(j + 1)
                / (2 * count);

            // Calculate points for two triangles, scaling x and y by radius1, z by radius2
            double x1 = radius1 * sin(theta1) * cos(phi1);
            double y1 = radius1 * sin(theta1) * sin(phi1);
            double z1 = radius2 * cos(theta1);

            double x2 = radius1 * sin(theta2) * cos(phi1);
            double y2 = radius1 * sin(theta2) * sin(phi1);
            double z2 = radius2 * cos(theta2);

            double x3 = radius1 * sin(theta1) * cos(phi2);
            double y3 = radius1 * sin(theta1) * sin(phi2);
            double z3 = radius2 * cos(theta1);

            double x4 = radius1 * sin(theta2) * cos(phi2);
            double y4 = radius1 * sin(theta2) * sin(phi2);
            double z4 = radius2 * cos(theta2);

            // Add triangles, avoiding degenerate cases at the poles
            if (sin(theta1) != 0.0) {
                surface->addTriangle(ocl::Point(x1, y1, z1),
                                     ocl::Point(x2, y2, z2),
                                     ocl::Point(x3, y3, z3));
            }

            if (sin(theta2) != 0.0) {
                surface->addTriangle(ocl::Point(x2, y2, z2),
                                     ocl::Point(x4, y4, z4),
                                     ocl::Point(x3, y3, z3));
            }
        }
    }

    // 重建AABB树
    rebuildAABBTree();

    spdlog::info("Created ellipsoid with radius1: {}, radius2: {}, resolution: {}",
                 radius1,
                 radius2,
                 count);
}

void CAMModelManager::createCylinder(float diameter, float height, int count, bool closed)
{
    stlFilePath.clear();
    surface = std::make_unique<ocl::STLSurf>();

    float radius = diameter / 2.0f;

    // Center points of top and bottom faces
    ocl::Point bottomCenter(0, 0, -height / 2);
    ocl::Point topCenter(0, 0, height / 2);

    for (int i = 0; i < count; i++) {
        double angle = 2.0 * boost::math::constants::pi<double>() * static_cast<double>(i) / count;
        double nextAngle = 2.0 * boost::math::constants::pi<double>()
            * static_cast<double>((i + 1) % count) / count;

        double x1 = radius * cos(angle);
        double y1 = radius * sin(angle);
        double x2 = radius * cos(nextAngle);
        double y2 = radius * sin(nextAngle);

        // Points for bottom and top faces
        ocl::Point bottomPt1(x1, y1, -height / 2);
        ocl::Point bottomPt2(x2, y2, -height / 2);
        ocl::Point topPt1(x1, y1, height / 2);
        ocl::Point topPt2(x2, y2, height / 2);

        // Add triangles for bottom and top faces if closed
        if (closed) {
            surface->addTriangle(bottomCenter, bottomPt1, bottomPt2);
            surface->addTriangle(topCenter, topPt2, topPt1);
        }

        // Add triangles for side face (two triangles per side segment)
        surface->addTriangle(bottomPt1, topPt1, bottomPt2);
        surface->addTriangle(bottomPt2, topPt1, topPt2);
    }

    // 重建AABB树
    rebuildAABBTree();

    spdlog::info("Created cylinder with diameter: {}, height: {}, resolution: {}, closed: {}",
                 diameter,
                 height,
                 count,
                 closed);
}

void CAMModelManager::createCone(float diameter1,
                                 float diameter2,
                                 float height,
                                 float edgeLength,
                                 int count,
                                 bool closed)
{
    stlFilePath.clear();
    surface = std::make_unique<ocl::STLSurf>();

    float radius1 = diameter1 / 2.0f;  // Bottom radius
    float radius2 = diameter2 / 2.0f;  // Top radius

    // Center points of top and bottom faces
    ocl::Point bottomCenter(0, 0, -height / 2);
    ocl::Point topCenter(0, 0, height / 2);

    // Calculate number of segments based on the edge length and perimeter
    if (edgeLength > 0) {
        float maxRadius = std::max(radius1, radius2);
        float maxPerimeter = 2 * boost::math::constants::pi<float>() * maxRadius;
        int calculated_count = static_cast<int>(maxPerimeter / edgeLength);
        if (calculated_count > count) {
            count = calculated_count;
        }
    }

    for (int i = 0; i < count; i++) {
        double angle = 2.0 * boost::math::constants::pi<double>() * static_cast<double>(i) / count;
        double nextAngle = 2.0 * boost::math::constants::pi<double>()
            * static_cast<double>((i + 1) % count) / count;

        double x1 = radius1 * cos(angle);
        double y1 = radius1 * sin(angle);
        double x2 = radius1 * cos(nextAngle);
        double y2 = radius1 * sin(nextAngle);

        double xt1 = radius2 * cos(angle);
        double yt1 = radius2 * sin(angle);
        double xt2 = radius2 * cos(nextAngle);
        double yt2 = radius2 * sin(nextAngle);

        // Points for bottom and top faces
        ocl::Point bottomPt1(x1, y1, -height / 2);
        ocl::Point bottomPt2(x2, y2, -height / 2);
        ocl::Point topPt1(xt1, yt1, height / 2);
        ocl::Point topPt2(xt2, yt2, height / 2);

        // Add triangles for bottom and top faces if closed
        if (closed) {
            if (radius1 > 0) {
                surface->addTriangle(bottomCenter, bottomPt1, bottomPt2);
            }
            if (radius2 > 0) {
                surface->addTriangle(topCenter, topPt2, topPt1);
            }
        }

        // Add triangles for side face (two triangles per side segment)
        surface->addTriangle(bottomPt1, topPt1, bottomPt2);
        surface->addTriangle(bottomPt2, topPt1, topPt2);
    }

    // 重建AABB树
    rebuildAABBTree();

    spdlog::info(
        "Created cone with diameter1: {}, diameter2: {}, height: {}, resolution: {}, closed: {}",
        diameter1,
        diameter2,
        height,
        count,
        closed);
}

void CAMModelManager::createTorus(float radius1, float radius2, int count)
{
    stlFilePath.clear();
    surface = std::make_unique<ocl::STLSurf>();

    // radius1 - major radius (center of tube to center of torus)
    // radius2 - minor radius (radius of the tube)

    for (int i = 0; i < count; i++) {
        double theta1 = 2.0 * boost::math::constants::pi<double>() * static_cast<double>(i) / count;
        double theta2 = 2.0 * boost::math::constants::pi<double>()
            * static_cast<double>((i + 1) % count) / count;

        for (int j = 0; j < count; j++) {
            double phi1 =
                2.0 * boost::math::constants::pi<double>() * static_cast<double>(j) / count;
            double phi2 = 2.0 * boost::math::constants::pi<double>()
                * static_cast<double>((j + 1) % count) / count;

            // Calculate the four points that form two triangles
            // Point on the torus at (theta, phi)
            auto torusPoint = [radius1, radius2](double theta, double phi) {
                double x = (radius1 + radius2 * cos(phi)) * cos(theta);
                double y = (radius1 + radius2 * cos(phi)) * sin(theta);
                double z = radius2 * sin(phi);
                return ocl::Point(x, y, z);
            };

            ocl::Point p1 = torusPoint(theta1, phi1);
            ocl::Point p2 = torusPoint(theta2, phi1);
            ocl::Point p3 = torusPoint(theta1, phi2);
            ocl::Point p4 = torusPoint(theta2, phi2);

            // Add two triangles for this grid cell
            surface->addTriangle(p1, p2, p4);
            surface->addTriangle(p1, p4, p3);
        }
    }

    // 重建AABB树
    rebuildAABBTree();

    spdlog::info("Created torus with major radius: {}, minor radius: {}, resolution: {}",
                 radius1,
                 radius2,
                 count);
}

void CAMModelManager::createCustomTriangles(const std::list<ocl::Triangle>& triangles)
{
    stlFilePath.clear();
    surface = std::make_unique<ocl::STLSurf>();

    for (const auto& tri : triangles) {
        surface->addTriangle(tri.p[0], tri.p[1], tri.p[2]);
    }

    rebuildAABBTree();
    spdlog::info("Created custom triangles with {} triangles", triangles.size());
}


ocl::Path createGuidePath(const ocl::STLSurf& surface)
{
    // Enlarge 5%
    double x_len = surface.bb.maxpt.x - surface.bb.minpt.x;
    double x_min = surface.bb.minpt.x;
    x_min -= 0.05 * x_len;
    double x_max = surface.bb.maxpt.x;
    x_max += 0.05 * x_len;

    double y_len = surface.bb.maxpt.y - surface.bb.minpt.y;
    double y_min = surface.bb.minpt.y;
    y_min -= 0.05 * y_len;
    double y_max = surface.bb.maxpt.y;
    y_max += 0.05 * y_len;

    double z_len = surface.bb.maxpt.z - surface.bb.minpt.z;
    // sometimes，the surface is too small, so we add a small offset
    double z_min = surface.bb.minpt.z - std::max(0.01, 0.05 * z_len);

    constexpr int NY = 40;
    const double dy = (y_max - y_min) / NY;
    ocl::Path path;
    for (int n = 0; n < NY; n++) {
        double y = y_min + n * dy;
        ocl::Point p1(x_min, y, z_min);
        ocl::Point p2(x_max, y, z_min);
        ocl::Line l(p1, p2);
        path.append(l);
    }

    spdlog::info("Guide path created with {} segments", path.span_list.size());
    return path;
}

void waterline(CAMModelManager& model,
               vtkActorManager& actorManager,
               double sampling,
               double lift_to,
               double lift_step,
               double lift_from,
               bool verbose)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }

    model.operation = std::make_unique<ocl::Waterline>();
    auto& wl = *dynamic_cast<ocl::Waterline*>(model.operation.get());
    wl.setSTL(*model.surface);
    wl.setCutter(model.cutter.get());
    wl.setSampling(sampling);

    spdlog::info("Waterline lifting from {} to {} with step {}", lift_from, lift_to, lift_step);

    using loop_type = decltype(wl.getLoops());
    std::vector<loop_type> all_loops;
    spdlog::stopwatch sw;

    for (double h = lift_from; h <= lift_to;) {
        wl.reset();
        wl.setZ(h);
        wl.run();
        auto loops = wl.getLoops();
        if (verbose) {
            spdlog::info("Got {} loops at height {:.3f}", loops.size(), h);
        }
        all_loops.emplace_back(std::move(loops));
        h += lift_step;
    }

    if (verbose) {
        spdlog::info("Generated {} layers of loops in {:.2f} s", all_loops.size(), sw);
    }

    UpdateLoopsActor(actorManager.operationActor, all_loops);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Waterline");
        actorManager.legendActor->VisibilityOff();
    }
}

void singleWaterline(CAMModelManager& model,
                     vtkActorManager& actorManager,
                     double sampling,
                     double z,
                     bool verbose)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }

    model.operation = std::make_unique<ocl::Waterline>();
    auto& wl = *dynamic_cast<ocl::Waterline*>(model.operation.get());
    wl.setSTL(*model.surface);
    wl.setCutter(model.cutter.get());
    wl.setSampling(sampling);

    spdlog::info("Single Waterline at {}", z);

    using loop_type = decltype(wl.getLoops());
    std::vector<loop_type> all_loops;
    spdlog::stopwatch sw;

    wl.reset();
    wl.setZ(z);
    wl.run();
    auto loops = wl.getLoops();
    if (verbose) {
        spdlog::info("Got {} loops at height {:.3f} in {} s", loops.size(), z, sw);
    }
    all_loops.emplace_back(std::move(loops));

    UpdateLoopsActor(actorManager.operationActor, all_loops);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Waterline");
        actorManager.legendActor->VisibilityOff();
    }
}


void adaptiveWaterline(CAMModelManager& model,
                       vtkActorManager& actorManager,
                       double sampling,
                       double minSampling,
                       double lift_to,
                       double lift_step,
                       double lift_from,
                       bool verbose)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }
    model.operation = std::make_unique<ocl::AdaptiveWaterline>();
    auto& awl = *dynamic_cast<ocl::AdaptiveWaterline*>(model.operation.get());
    awl.setSTL(*model.surface);
    awl.setCutter(model.cutter.get());
    awl.setSampling(sampling);
    awl.setMinSampling(minSampling);

    spdlog::info("Adaptive Waterline lifting from {} to {} with step {}",
                 lift_from,
                 lift_to,
                 lift_step);

    using loop_type = decltype(awl.getLoops());
    std::vector<loop_type> all_loops;
    spdlog::stopwatch sw;

    for (double h = lift_from; h <= lift_to;) {
        awl.reset();
        awl.setZ(h);
        awl.run();
        auto loops = awl.getLoops();
        if (verbose) {
            spdlog::info("Got {} adaptive loops at height {:.3f}", loops.size(), h);
        }
        all_loops.emplace_back(std::move(loops));
        h += lift_step;
    }

    if (verbose) {
        spdlog::info("Generated {} layers of adaptive loops in {:.2f} s", all_loops.size(), sw);
    }

    UpdateLoopsActor(actorManager.operationActor, all_loops);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Adaptive Waterline");
        actorManager.legendActor->VisibilityOff();
    }
}

void pathDropCutter(CAMModelManager& model, vtkActorManager& actorManager, double sampling)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }
    model.operation = std::make_unique<ocl::PathDropCutter>();
    auto& pdc = *dynamic_cast<ocl::PathDropCutter*>(model.operation.get());
    spdlog::stopwatch sw;
    pdc.setSTL(*model.surface);
    pdc.setCutter(model.cutter.get());

    auto guidePath = createGuidePath(*model.surface);
    pdc.setPath(&guidePath);
    pdc.setSampling(sampling);
    pdc.reset();
    // 设置z值为表面最小z值减去 max(0.01, 0.05 * (surface.bb.maxpt.z - surface.bb.minpt.z))
    pdc.setZ(model.surface->bb.minpt.z
             - max(0.01, 0.05 * (model.surface->bb.maxpt.z - model.surface->bb.minpt.z)));
    pdc.run();
    auto points = pdc.getPoints();
    spdlog::info("PDC done in {} s and got {} points", sw, points.size());
    // print the statistics of the points
    printStats(points);

    UpdateCLPointCloudActor(actorManager.operationActor, actorManager.legendActor, points);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Path Drop Cutter");
        actorManager.legendActor->VisibilityOn();
    }
}

void randomBatchDropCutter(CAMModelManager& model,
                           vtkActorManager& actorManager,
                           double sampling,
                           int randomPoints)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }
    spdlog::stopwatch sw;
    model.operation = std::make_unique<ocl::BatchDropCutter>();
    auto& bdc = *dynamic_cast<ocl::BatchDropCutter*>(model.operation.get());
    bdc.setSTL(*model.surface);
    bdc.setCutter(model.cutter.get());
    bdc.setSampling(sampling);

    // Generate random points using modern C++ random generators
    const auto& minp = model.surface->bb.minpt;
    const auto& maxp = model.surface->bb.maxpt;

    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    // 为x, y, z坐标创建均匀分布
    std::uniform_real_distribution<double> dist_x(minp.x, maxp.x);
    std::uniform_real_distribution<double> dist_y(minp.y, maxp.y);
    std::uniform_real_distribution<double> dist_z(minp.z, maxp.z);

    for (int i = 0; i < randomPoints; i++) {
        ocl::CLPoint p(dist_x(gen), dist_y(gen), dist_z(gen));
        bdc.appendPoint(p);
    }

    bdc.run();
    auto points = bdc.getCLPoints();
    spdlog::info("RBD done in {} s and got {} points", sw, points.size());

    // print the statistics of the points
    printStats(points);

    UpdateCLPointCloudActor(actorManager.operationActor, actorManager.legendActor, points);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Random Batch Drop Cutter");
        actorManager.legendActor->VisibilityOn();
    }
}


void adaptivePathDropCutter(CAMModelManager& model,
                            vtkActorManager& actorManager,
                            double sampling,
                            double minSampling)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }
    spdlog::stopwatch sw;
    model.operation = std::make_unique<ocl::AdaptivePathDropCutter>();
    auto& apdc = *dynamic_cast<ocl::AdaptivePathDropCutter*>(model.operation.get());
    apdc.setSTL(*model.surface);
    apdc.setCutter(model.cutter.get());

    auto guidePath = createGuidePath(*model.surface);
    apdc.setPath(&guidePath);
    apdc.setSampling(sampling);
    apdc.setMinSampling(minSampling);
    apdc.reset();
    // FIXME: APDC实际上并没有考虑这个minZ值
    // 设置z值为表面最小z值减去 max(0.01, 0.05 * (surface.bb.maxpt.z - surface.bb.minpt.z))
    apdc.setZ(model.surface->bb.minpt.z
              - max(0.01, 0.05 * (model.surface->bb.maxpt.z - model.surface->bb.minpt.z)));
    apdc.run();
    auto points = apdc.getPoints();
    spdlog::info("APDC done in {} s and got {} points", sw, points.size());

    // print the statistics of the points
    printStats(points);

    UpdateCLPointCloudActor(actorManager.operationActor, actorManager.legendActor, points);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Adaptive Path Drop Cutter");
        actorManager.operationActor->VisibilityOn();
    }
}

void hello_ocl()
{
    spdlog::info("=============================");
    spdlog::info("ocl version: {}", ocl::version());
    spdlog::info("max threads: {}", ocl::max_threads());

    spdlog::info("===== TBB Global Control info =====");
    int max_parallelism =
        tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
    spdlog::info("max parallelism: {}", max_parallelism);

    size_t stack_size = tbb::global_control::active_value(tbb::global_control::thread_stack_size);
    spdlog::info("thread stack size: {} bytes", stack_size);

    spdlog::info("=============================");
}

std::vector<ocl::CLPoint> debugPointDropCutter(CAMModelManager& model, const ocl::CLPoint& inputCL)
{
    int calls = 0;
    if (!model.surface || !model.cutter || !model.aabbTree) {
        spdlog::error("No surface or cutter or aabbTree");
        return {};
    }

    auto tris = model.aabbTree->search_cutter_overlap(model.cutter.get(), &inputCL);
    std::vector<ocl::CLPoint> res;
    ocl::CLPoint cl = inputCL;
    spdlog::info("The initial point is at {}", cl.str());
    // Loop over found triangles
    for (const auto& tri : tris) {
        // Cutter overlap triangle? check
        if (model.cutter->overlaps(cl, tri)) {
            if (cl.below(tri)) {
                if (model.cutter->dropCutter(cl, tri)) {
                    spdlog::info("The point is at {}", cl.str());
                    res.emplace_back(cl);
                }
                calls++;
            }
        }
    }
    spdlog::info("DropCutter done in {} calls and got {} points", calls, res.size());
    return res;
}

void fiberPushCutter(CAMModelManager& model,
                     vtkActorManager& actorManager,
                     const Eigen::Vector3d& start,
                     const Eigen::Vector3d& end,
                     bool verbose)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }

    Eigen::Vector3d dir = end - start;
    dir.normalize();
    int direction = 0;  // 0 for x, 1 for y
    if (dir.isApprox(Eigen::Vector3d::UnitY()) || dir.isApprox(Eigen::Vector3d::UnitY() * -1)) {
        direction = 1;
    }
    ocl::Fiber fiber(ocl::Point(start[0], start[1], start[2]), ocl::Point(end[0], end[1], end[2]));
    spdlog::info("Create a fiber: {}", fiber.str());

    model.operation = std::make_unique<ocl::FiberPushCutter>();
    auto& fpc = *dynamic_cast<ocl::FiberPushCutter*>(model.operation.get());
    if (direction == 0) {
        fpc.setXDirection();
    }
    else {
        fpc.setYDirection();
    }
    fpc.setSTL(*model.surface);
    fpc.setCutter(model.cutter.get());
    fpc.run(fiber);

    spdlog::info("After running the fpc, fiber: {}", fiber.str());
    UpdateFiberActor(actorManager.operationActor, actorManager.legendActor, {fiber});
}

void batchFiberPushCutter(CAMModelManager& model,
                          vtkActorManager& actorManager,
                          double sampling,
                          double lift_to,
                          double lift_step,
                          double lift_from,
                          bool verbose)
{
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }

    model.operation = std::make_unique<ocl::BatchPushCutter>();
    auto& bpc = *dynamic_cast<ocl::BatchPushCutter*>(model.operation.get());
    bpc.setSampling(sampling);
    bpc.setCutter(model.cutter.get());

    // 计算XY平面的边界，并扩展2个刀具半径
    double minx = model.surface->bb.minpt.x - 2 * model.cutter->getRadius();
    double maxx = model.surface->bb.maxpt.x + 2 * model.cutter->getRadius();
    double miny = model.surface->bb.minpt.y - 2 * model.cutter->getRadius();
    double maxy = model.surface->bb.maxpt.y + 2 * model.cutter->getRadius();

    // 使用匿名函数生成fibers
    auto generateFibers = [&](double minx,
                              double maxx,
                              double miny,
                              double maxy,
                              double sampling,
                              double lift_from,
                              double lift_to,
                              double lift_step) {
        // 计算X和Y方向上需要的fiber数量
        int Nx = static_cast<int>((maxx - minx) / sampling);
        int Ny = static_cast<int>((maxy - miny) / sampling);

        // 生成X和Y方向的采样点
        auto generateRange = [](double start, double end, int N) {
            std::vector<double> output;
            double d = (end - start) / static_cast<double>(N);
            double v = start;
            for (int n = 0; n < (N + 1); ++n) {
                output.push_back(v);
                v = v + d;
            }
            return output;
        };

        std::vector<double> xvals = generateRange(minx, maxx, Nx);
        std::vector<double> yvals = generateRange(miny, maxy, Ny);

        // 收集所有生成的X和Y方向的fibers
        std::vector<ocl::Fiber> xfibers;
        std::vector<ocl::Fiber> yfibers;

        // 对每个高度生成fiber
        for (double z = lift_from; z <= lift_to; z += lift_step) {
            // 为每个y值生成X方向的fiber
            for (double y : yvals) {
                ocl::Point p1(minx, y, z);
                ocl::Point p2(maxx, y, z);
                ocl::Fiber f(p1, p2);
                xfibers.push_back(f);
            }

            // 为每个x值生成Y方向的fiber
            for (double x : xvals) {
                ocl::Point p1(x, miny, z);
                ocl::Point p2(x, maxy, z);
                ocl::Fiber f(p1, p2);
                yfibers.push_back(f);
            }
        }

        return std::make_pair(xfibers, yfibers);
    };

    // 生成fibers
    auto [xfibers, yfibers] =
        generateFibers(minx, maxx, miny, maxy, sampling, lift_from, lift_to, lift_step);

    if (verbose) {
        spdlog::info("Generated {} X-direction fibers", xfibers.size());
        spdlog::info("Generated {} Y-direction fibers", yfibers.size());
    }

    // 设置X方向的fibers并运行
    bpc.setXDirection();
    bpc.setSTL(*model.surface);
    for (auto& fiber : xfibers) {
        bpc.appendFiber(fiber);
    }
    bpc.run();
    auto xfibers_processed = *bpc.getFibers();

    // 重置并设置Y方向的fibers并运行
    bpc.reset();
    bpc.setYDirection();
    bpc.setSTL(*model.surface);
    for (auto& fiber : yfibers) {
        bpc.appendFiber(fiber);
    }
    bpc.run();
    auto yfibers_processed = *bpc.getFibers();

    // 更新fiber显示
    std::vector<ocl::Fiber> all_fibers;
    all_fibers.insert(all_fibers.end(), xfibers_processed.begin(), xfibers_processed.end());
    all_fibers.insert(all_fibers.end(), yfibers_processed.begin(), yfibers_processed.end());

    UpdateFiberActor(actorManager.operationActor, actorManager.legendActor, all_fibers);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Batch Fiber PushCutter");
        actorManager.legendActor->VisibilityOn();
    }
}
