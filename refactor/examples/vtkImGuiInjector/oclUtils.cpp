#include "oclUtils.h"

ocl::Path createGuidePath(const ocl::STLSurf& surface) {
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
               bool verbose) {
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }

    model.operation = std::make_unique<ocl::Waterline>();
    auto& wl = *dynamic_cast<ocl::Waterline *>(model.operation.get());
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
        spdlog::info("Generated {} layers of loops in {:.2f} ms", all_loops.size(), sw);
    }

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
                       bool verbose) {
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }
    model.operation = std::make_unique<ocl::AdaptiveWaterline>();
    auto& awl = *dynamic_cast<ocl::AdaptiveWaterline *>(model.operation.get());
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
        spdlog::info("Generated {} layers of adaptive loops in {:.2f} ms", all_loops.size(), sw);
    }

    UpdateLoopsActor(actorManager.operationActor, all_loops);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Adaptive Waterline");
        actorManager.legendActor->VisibilityOff();
    }
}

void pathDropCutter(CAMModelManager& model, vtkActorManager& actorManager, double sampling) {
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }
    model.operation = std::make_unique<ocl::PathDropCutter>();
    auto& pdc = *dynamic_cast<ocl::PathDropCutter *>(model.operation.get());
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
    spdlog::info("PDC done in {} ms and got {} points", sw, points.size());

    UpdateCLPointCloudActor(actorManager.operationActor, actorManager.legendActor, points);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Path Drop Cutter");
        actorManager.legendActor->VisibilityOn();
    }
}

void adaptivePathDropCutter(CAMModelManager& model,
                            vtkActorManager& actorManager,
                            double sampling,
                            double minSampling) {
    if (!model.cutter || !model.surface) {
        spdlog::error("No cutter or surface");
        return;
    }
    spdlog::stopwatch sw;
    model.operation = std::make_unique<ocl::AdaptivePathDropCutter>();
    auto& apdc = *dynamic_cast<ocl::AdaptivePathDropCutter *>(model.operation.get());
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
    spdlog::info("APDC done in {} ms and got {} points", sw, points.size());

    UpdateCLPointCloudActor(actorManager.operationActor, actorManager.legendActor, points);
    if (actorManager.operationActor) {
        actorManager.operationActor->SetObjectName("Adaptive Path Drop Cutter");
        actorManager.operationActor->VisibilityOn();
    }
}

void hello_ocl() {
    spdlog::info("ocl version: {}", ocl::version());
    spdlog::info("max threads: {}", ocl::max_threads());
}
