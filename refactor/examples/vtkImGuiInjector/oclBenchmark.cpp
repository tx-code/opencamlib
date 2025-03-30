#include "oclBenchmark.h"
#include <random>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <tbb/parallel_for.h>


#include "STLSurfUtils.h"
#include "algo/waterline.hpp"

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
                    "##OpenMP Version: Batchdropcutter with {} points took {} s: {} calls",
                    max_points,
                    sw,
                    bdc.getCalls());
            }
            else {
                benchmark_logger->info(
                    "##TBB Version: Batchdropcutter with {} points took {} s: {} calls",
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

        benchmark_logger->info("Run batchdropcutter with {} triangles took {} s: {} calls",
                               surface_copy.tris.size(),
                               sw,
                               bdc.getCalls());

        // update the surface
        SubdivideSurface(surface_copy);
    }
    benchmark_logger->info("=====End Benchmark=====");
}

void run_BatchDropCutter_WithDifferentBucketSize(const CAMModelManager& model, bool verbose)
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

    for (int bucket_size = 1; bucket_size <= 10; bucket_size++) {
        if (verbose) {
            benchmark_logger->info("Running Batchdropcutter with bucket size {}", bucket_size);
        }
        // Prepare batchdropcutter
        ocl::BatchDropCutter bdc;
        bdc.setSTL(*model.surface);
        bdc.setCutter(model.cutter.get());
        bdc.setBucketSize(bucket_size);

        for (auto& p : points) {
            bdc.appendPoint(p);
        }

        // Run batchdropcutter
        spdlog::stopwatch sw;
        bdc.run();

        benchmark_logger->info("Run batchdropcutter with bucket size {} took {} s: {} calls",
                               bucket_size,
                               sw,
                               bdc.getCalls());
    }

    benchmark_logger->info("=====End Benchmark=====");
}

void run_WaterlineBenchmark(const CAMModelManager& model, bool verbose)
{
    // 如果logger未初始化，则初始化它
    if (!benchmark_logger) {
        init_benchmark_logger();
    }

    benchmark_logger->info("=====Begin Waterline Benchmark=====");
    benchmark_logger->info("Use Cutter {} and Surface {} (#F: {})",
                           model.cutter->str(),
                           model.stlFilePath,
                           model.surface->tris.size());

    // warmup_tbb first
    warmup_tbb();

#if 0
    /// BENCHMARK 1: 不同数量的z值
    // 测试不同数量的z值
    std::vector<int> z_counts = {1, 2, 4, 8, 16, 32, 64, 128, 256};
    
    for (int z_count : z_counts) {
        std::vector<double> z_values;
        
        // 生成均匀分布的z值
        double min_z = model.surface->bb.minpt.z;
        double max_z = model.surface->bb.maxpt.z;
        double step = (max_z - min_z) / (z_count + 1);
        double old_total_time = 0.0;
        double new_total_time = 0.0;
        
        for (int i = 1; i <= z_count; i++) {
            z_values.push_back(min_z + i * step);
        }
        
        // 测试run2 (OpenMP)
        {
            ocl::Waterline waterline;
            waterline.setSTL(*model.surface);
            waterline.setCutter(model.cutter.get());
            waterline.setSampling(0.1); // 使用合适的采样率
            
            for (double z : z_values) {
                waterline.setZ(z);
                waterline.reset();
                
                spdlog::stopwatch sw;
                waterline.run2();
                double elapsed = sw.elapsed().count();
                old_total_time += elapsed;
            }
            
            benchmark_logger->info("##OpenMP Version (run2): Total time for {} z-values: {} s, avg: {} s", 
                                 z_count, old_total_time, old_total_time / z_count);
        }
        
        // 测试run3 (TBB)
        {
            ocl::Waterline waterline;
            waterline.setSTL(*model.surface);
            waterline.setCutter(model.cutter.get());
            waterline.setSampling(0.1); // 使用合适的采样率
            waterline.setZValues(z_values);
            waterline.setForceUseTBB(true);
            
            spdlog::stopwatch sw;
            waterline.run3();
            new_total_time = sw.elapsed().count();
            
            benchmark_logger->info("##TBB Version (run3): Total time for {} z-values: {} s", 
                                 z_count, new_total_time);
            
            if (verbose) {
                benchmark_logger->info("run3 generated {} loops for {} z-values", 
                                     waterline.getLoops().size(), z_count);
            }
        }
        
        // 计算加速比
        double acceleration = old_total_time / new_total_time;
        benchmark_logger->info("Acceleration: {}%", acceleration * 100.0);
    }
#endif

    // 新增测试：比较force_use_tbb对Waterline性能的影响，使用固定的64个z值
    benchmark_logger->info("==== Testing force_use_tbb Impact with 64 fixed z-values ====");

    // 生成32个均匀分布的z值
    std::vector<double> z_values_32;
    double min_z = model.surface->bb.minpt.z;
    double max_z = model.surface->bb.maxpt.z;
    double step = (max_z - min_z) / 65.0;

    for (int i = 1; i <= 64; i++) {
        z_values_32.push_back(min_z + i * step);
    }

    // 使用不同的sampling率测试
    std::vector<double> sampling_rates = {0.05, 0.1, 0.2, 0.3};

    for (double sampling : sampling_rates) {
        benchmark_logger->info("== Sampling rate: {} ==", sampling);

        double total_time_openmp = 0.0;
        double total_time_tbb = 0.0;
        int total_loops_openmp = 0;
        int total_loops_tbb = 0;

        ocl::Waterline waterline;
        waterline.setSTL(*model.surface);
        waterline.setCutter(model.cutter.get());
        waterline.setSampling(sampling);
        waterline.setForceUseTBB(false);

        ocl::Waterline tbb_waterline;
        tbb_waterline.setSTL(*model.surface);
        tbb_waterline.setCutter(model.cutter.get());
        tbb_waterline.setSampling(sampling);
        tbb_waterline.setForceUseTBB(true);
        for (double z : z_values_32) {
            // 测试不开启force_use_tbb (使用OpenMP)
            {
                waterline.setZ(z);
                waterline.reset();
                spdlog::stopwatch sw;
                waterline.run2();
                double elapsed = sw.elapsed().count();
                total_time_openmp += elapsed;
                total_loops_openmp += waterline.getLoops().size();
            }

            // 测试开启force_use_tbb
            {
                tbb_waterline.setZ(z);
                tbb_waterline.reset();
                spdlog::stopwatch sw;
                tbb_waterline.run2();
                double elapsed = sw.elapsed().count();
                total_time_tbb += elapsed;
                total_loops_tbb += tbb_waterline.getLoops().size();
            }
        }

        // 输出总时间和平均时间
        benchmark_logger->info(
            "OpenMP Version: 64 z-values, Total time: {} s, Avg time: {} s, {} loops generated",
            total_time_openmp,
            total_time_openmp / 64.0,
            total_loops_openmp);

        benchmark_logger->info(
                "TBB Version: 64 z-values, Total time: {} s, Avg time: {} s, {} loops generated",
            total_time_tbb,
            total_time_tbb / 64.0,
            total_loops_tbb);

        // 计算加速比
        double acceleration = total_time_openmp / total_time_tbb;
        benchmark_logger->info("Acceleration: {}%", acceleration * 100.0);
    }

    benchmark_logger->info("=====End Waterline Benchmark=====");
}
