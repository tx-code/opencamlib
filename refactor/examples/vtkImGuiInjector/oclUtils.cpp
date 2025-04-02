#include "oclUtils.h"

// 添加随机数生成器相关头文件
#include <functional>
#include <random>
#include <tbb/global_control.h>
#include <tbb/info.h>


ocl::Path createGuidePath(const ocl::STLSurf& surface)
{
    // Enlarge 5%
    double x_min = surface.bb.minpt.x;
    x_min -= 0.05 * x_min;
    double x_max = surface.bb.maxpt.x;
    x_max += 0.05 * x_max;
    double y_min = surface.bb.minpt.y;
    y_min -= 0.05 * y_min;
    double y_max = surface.bb.maxpt.y;
    y_max += 0.05 * y_max;
    constexpr int NY = 40;
    const double dy = (y_max - y_min) / NY;
    ocl::Path path;
    for (int n = 0; n < NY; n++) {
        double y = y_min + n * dy;
        ocl::Point p1(x_min, y, 0);
        ocl::Point p2(x_max, y, 0);
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
    pdc.setZ(model.surface->bb.minpt.z);
    pdc.run();
    auto points = pdc.getPoints();
    spdlog::info("PDC done in {} s and got {} points", sw, points.size());

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
    apdc.setZ(model.surface->bb.minpt.z);
    apdc.run();
    auto points = apdc.getPoints();
    spdlog::info("APDC done in {} s and got {} points", sw, points.size());

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
