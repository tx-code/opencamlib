#pragma once

#include <codecvt>
#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>

#include "vtkUtils.h"

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

struct CAM_DataModel {
    std::unique_ptr<ocl::STLSurf> surface;
    std::unique_ptr<ocl::MillingCutter> cutter;
    std::unique_ptr<ocl::Operation> operation;
};

void hello_ocl();

// Y-direction zigzag pattern
// TODO more patterns
ocl::Path createGuidePath(const ocl::STLSurf& surface);

void waterline(CAM_DataModel& model, vtkSmartPointer<vtkActor>& opActor, double sampling,
               double lift_to = 1, double lift_step = 0.1,
               double lift_from = 0.0, bool verbose = true);

void adaptiveWaterline(CAM_DataModel& model, vtkSmartPointer<vtkActor>& opActor, double sampling,
                       double minSampling, double lift_to = 1,
                       double lift_step = 0.1, double lift_from = 0.0,
                       bool verbose = true);

void pathDropCutter(CAM_DataModel& model, vtkSmartPointer<vtkActor>& opActor,
                    double sampling);

void adaptivePathDropCutter(CAM_DataModel& model, vtkSmartPointer<vtkActor>& opActor,
                            double sampling, double minSampling);
