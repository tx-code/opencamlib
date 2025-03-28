#include "oclBenchmark.h"
#include <random>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <tbb/parallel_for.h>

namespace
{
void generate_points(const ocl::STLSurf& surface, int max_points, std::vector<ocl::CLPoint>& points)
{
    // Generate random points using modern C++ random generators
    const auto& minp = surface.bb.minpt;
    const auto& maxp = surface.bb.maxpt;

    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    // 为x, y, z坐标创建均匀分布
    std::uniform_real_distribution<double> dist_x(minp.x, maxp.x);
    std::uniform_real_distribution<double> dist_y(minp.y, maxp.y);
    std::uniform_real_distribution<double> dist_z(minp.z, maxp.z);

    points.resize(max_points);
    for (int i = 0; i < max_points; i++) {
        points[i] = std::move(ocl::CLPoint(dist_x(gen), dist_y(gen), dist_z(gen)));
    }
}

void warmup_tbb()
{
    // 方法1：执行一个简单的parallel_for
    tbb::parallel_for(0, 1000, [](int) {
        // 空操作
    });
}
}  // namespace

void run_batchdropcutter(const ocl::STLSurf& surface,
                         const ocl::MillingCutter& cutter,
                         bool verbose)
{
    // warmup_tbb first
    warmup_tbb();

    int max_points = 1;
    std::vector<ocl::CLPoint> points;
    for (int i = 0; i <= 6; i++) {
        // Prepare points
        generate_points(surface, max_points, points);
        max_points *= 10;

        // Same Points, Same Cutter, Same Surface
        for (int j = 0; j < 2; j++) {
            if (verbose) {
                if (j == 0) {
                    spdlog::info("Running OpenMP Version with {} points", max_points);
                }
                else {
                    spdlog::info("Running TBB Version with {} points", max_points);
                }
            }
            // Prepare batchdropcutter
            ocl::BatchDropCutter bdc;
            bdc.setSTL(surface);
            bdc.setCutter(&cutter);

            for (auto& p : points) {
                bdc.appendPoint(p);
            }

            // Run batchdropcutter
            if (j == 0) {
                bdc.setForceUseTBB(false);
            }
            else {
                bdc.setForceUseTBB(true);
            }
            spdlog::stopwatch sw;
            bdc.run();
            if (j == 0) {
                spdlog::info(
                    "##OpenMP Vertion: Batchdropcutter with {} points took {} ms: {} calls",
                    max_points,
                    sw,
                    bdc.getCalls());
            }
            else {
                spdlog::info("##TBB Vertion: Batchdropcutter with {} points took {} ms: {} calls",
                             max_points,
                             sw,
                             bdc.getCalls());
            }
        }
    }
}
