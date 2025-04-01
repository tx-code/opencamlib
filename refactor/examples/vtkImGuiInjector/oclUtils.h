#pragma once

#include <codecvt>
#include <memory>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <string>
#include <vector>


#include "vtkActorManager.h"
#include "vtkUtils.h"


// OCL Stuff
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

struct CAMModelManager
{
    std::unique_ptr<ocl::STLSurf> surface;
    std::unique_ptr<ocl::MillingCutter> cutter;
    std::unique_ptr<ocl::Operation> operation;
    std::string stlFilePath;  // 当前打开的STL文件路径

    // 缓存的AABBTree，避免重复构建
    std::unique_ptr<ocl::AABBTreeAdaptor> aabbTree;

    // 重建AABB树
    void rebuildAABBTree()
    {
        if (surface && !surface->tris.empty()) {
            if (!aabbTree) {
                aabbTree = std::make_unique<ocl::AABBTreeAdaptor>();
            }
            aabbTree->build(surface->tris);
            spdlog::info("AABBTree rebuilt with {} triangles", surface->tris.size());
        }
        else {
            aabbTree.reset();
        }
    }
};

void hello_ocl();

// Y-direction zigzag pattern
// TODO more patterns
ocl::Path createGuidePath(const ocl::STLSurf& surface);

void waterline(CAMModelManager& model,
               vtkActorManager& actorManager,
               double sampling,
               double lift_to = 1,
               double lift_step = 0.1,
               double lift_from = 0.0,
               bool verbose = true);

void adaptiveWaterline(CAMModelManager& model,
                       vtkActorManager& actorManager,
                       double sampling,
                       double minSampling,
                       double lift_to = 1,
                       double lift_step = 0.1,
                       double lift_from = 0.0,
                       bool verbose = true);

void pathDropCutter(CAMModelManager& model, vtkActorManager& actorManager, double sampling);

void randomBatchDropCutter(CAMModelManager& model,
                           vtkActorManager& actorManager,
                           double sampling,
                           int randomPoints);

void adaptivePathDropCutter(CAMModelManager& model,
                            vtkActorManager& actorManager,
                            double sampling,
                            double minSampling);

// Debug the DropCutter from a single input CLPoint
// Return the result of the DropCutter as a vector of CLPoints
// And you can visualize the result by updating the actor's position step by step
std::vector<ocl::CLPoint> debugPointDropCutter(CAMModelManager& model, const ocl::CLPoint& inputCL);
