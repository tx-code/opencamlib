#include "oclBenchmark.h"
#include <random>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <tbb/parallel_for.h>


#include "STLSurfUtils.h"

namespace
{
// 定义全局的benchmark logger
std::shared_ptr<spdlog::logger> benchmark_logger;

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
    // 执行一个简单的parallel_for
    tbb::parallel_for(0, 1000, [](int) {
        // 空操作
    });
}
}  // namespace

void init_benchmark_logger(const std::string& log_file_path)
{
    try {
        // 创建文件sink和控制台sink
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file_path, true);
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        // 创建sinks数组
        std::vector<spdlog::sink_ptr> sinks {file_sink, console_sink};

        // 创建logger，同时输出到文件和控制台
        benchmark_logger =
            std::make_shared<spdlog::logger>("benchmark", sinks.begin(), sinks.end());

        // 设置日志格式
        benchmark_logger->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] %v");

        // 设置日志级别
        benchmark_logger->set_level(spdlog::level::info);

        // 刷新策略
        benchmark_logger->flush_on(spdlog::level::info);

        benchmark_logger->info("Benchmark logger initialized");
    }
    catch (const spdlog::spdlog_ex& ex) {
        spdlog::error("Benchmark logger initialization failed: {}", ex.what());
    }
}

void run_batchdropcutter(const CAMModelManager& model, bool verbose)
{
    // 如果logger未初始化，则初始化它
    if (!benchmark_logger) {
        init_benchmark_logger();
    }

    benchmark_logger->info("=====Begin Benchmark=====");
    benchmark_logger->info("Use Cutter {} and Surface {} (#F: {})",
                           model.cutter->str(),
                           model.stlFilePath,
                           model.surface->tris.size());

    // warmup_tbb first
    warmup_tbb();

    int max_points = 1;
    std::vector<ocl::CLPoint> points;
    for (int i = 0; i <= 6; i++) {
        // Prepare points
        generate_points(*model.surface, max_points, points);
        max_points *= 10;

        // Same Points, Same Cutter, Same Surface
        for (int j = 0; j < 2; j++) {
            if (verbose) {
                if (j == 0) {
                    benchmark_logger->info("Running OpenMP Version with {} points", max_points);
                }
                else {
                    benchmark_logger->info("Running TBB Version with {} points", max_points);
                }
            }
            // Prepare batchdropcutter
            ocl::BatchDropCutter bdc;
            bdc.setSTL(*model.surface);
            bdc.setCutter(model.cutter.get());

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
                benchmark_logger->info(
                    "##OpenMP Version: Batchdropcutter with {} points took {} ms: {} calls",
                    max_points,
                    sw,
                    bdc.getCalls());
            }
            else {
                benchmark_logger->info(
                    "##TBB Version: Batchdropcutter with {} points took {} ms: {} calls",
                    max_points,
                    sw,
                    bdc.getCalls());
            }
        }
    }

    benchmark_logger->info("=====End Benchmark=====");
}

void run_SurfaceSubdivisionBatchDropCutter(const CAMModelManager& model, bool verbose)
{
    // 如果logger未初始化，则初始化它
    if (!benchmark_logger) {
        init_benchmark_logger();
    }

    benchmark_logger->info("=====Begin Benchmark=====");
    benchmark_logger->info("Use Cutter {} and Surface {} (#F: {})",
                           model.cutter->str(),
                           model.stlFilePath,
                           model.surface->tris.size());

    // warmup_tbb first
    warmup_tbb();

    // prepare 1e5 points
    int max_points = 100000;
    std::vector<ocl::CLPoint> points;
    generate_points(*model.surface, max_points, points);

    // copy the surf
    ocl::STLSurf surface_copy = *model.surface;

    while (surface_copy.tris.size() < 1e7) {
        if (verbose) {
            benchmark_logger->info(
                "Running Surface Subdivision Batchdropcutter with {} triangles and {} drop points",
                surface_copy.tris.size(),
                max_points);
        }
        // Prepare batchdropcutter
        ocl::BatchDropCutter bdc;
        bdc.setSTL(surface_copy);
        bdc.setCutter(model.cutter.get());

        for (auto& p : points) {
            bdc.appendPoint(p);
        }

        // Run batchdropcutter
        spdlog::stopwatch sw;
        bdc.run();

        benchmark_logger->info("Run batchdropcutter with {} triangles took {} ms: {} calls",
                               surface_copy.tris.size(),
                               sw,
                               bdc.getCalls());

        // update the surface
        SubdivideSurface(surface_copy);
    }
    benchmark_logger->info("=====End Benchmark=====");
}