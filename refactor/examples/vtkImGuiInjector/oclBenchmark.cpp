#include "oclBenchmark.h"
#include <random>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>
#include <spdlog/stopwatch.h>
#include <tbb/parallel_for.h>


#include "AABBTreeAdaptor.h"
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

void generate_boxes(const ocl::STLSurf& surface, int max_boxes, std::vector<ocl::Bbox>& boxes)
{
    // Generate random boxes using modern C++ random generators
    const auto& minp = surface.bb.minpt;
    const auto& maxp = surface.bb.maxpt;

    double min_size = min(maxp.x - minp.x, maxp.y - minp.y);
    min_size = min(min_size, maxp.z - minp.z);
    double max_size = max(maxp.x - minp.x, maxp.y - minp.y);
    max_size = max(max_size, maxp.z - minp.z);

    // 创建随机数生成器
    std::random_device rd;
    std::mt19937 gen(rd());

    // 为x, y, z坐标创建均匀分布
    // A random point + a random size
    std::uniform_real_distribution<double> dist_x(minp.x, maxp.x);
    std::uniform_real_distribution<double> dist_y(minp.y, maxp.y);
    std::uniform_real_distribution<double> dist_z(minp.z, maxp.z);
    std::uniform_real_distribution<double> dist_size(min_size, max_size);

    boxes.resize(max_boxes);
    for (int i = 0; i < max_boxes; i++) {
        boxes[i] = std::move(ocl::Bbox(dist_x(gen),
                                       dist_x(gen) + dist_size(gen),
                                       dist_y(gen),
                                       dist_y(gen) + dist_size(gen),
                                       dist_z(gen),
                                       dist_z(gen) + dist_size(gen)));
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

void run_AABBTree_VS_KDTree(const CAMModelManager& model, bool verbose)
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

    benchmark_logger->info("Compare the build time of KDTree and AABBTree");

    // Raw KDTree
    spdlog::stopwatch sw;
    ocl::KDTree<ocl::Triangle> kd_tree;
    kd_tree.build(model.surface->tris);
    double kd_tree_build_time = sw.elapsed().count();
    benchmark_logger->info("\tRaw KDTree build with {} triangles took {} s",
                           model.surface->tris.size(),
                           kd_tree_build_time);

    sw.reset();
    // AABBTree
    ocl::AABBTreeAdaptor aabb_tree;
    aabb_tree.build(model.surface->tris);
    double aabb_tree_build_time = sw.elapsed().count();
    benchmark_logger->info("\tAABBTree build with {} triangles took {} s",
                           model.surface->tris.size(),
                           aabb_tree_build_time);

    // Acceleration
    benchmark_logger->info("\tAcceleration of the BUILD time: {}%",
                           kd_tree_build_time / aabb_tree_build_time * 100);

    //------------------------//
    // Search Time....        //
    //------------------------//
    benchmark_logger->info("Compare the search time of KDTree and AABBTree");

    // Generate 1e2 boxes
    int max_boxes = 100;
    std::vector<ocl::Bbox> boxes;
    generate_boxes(*model.surface, max_boxes, boxes);

    // Search
    sw.reset();
    std::array search_results {0, 0};
    for (auto& box : boxes) {
        auto res = kd_tree.search(box);
        search_results[0] += res->size();
        delete res;
    }
    double kd_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tKDTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           kd_tree_search_time,
                           search_results[0]);

    sw.reset();
    for (auto& box : boxes) {
        auto res = aabb_tree.search(box);
        search_results[1] += res.size();
    }
    double aabb_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tAABBTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           aabb_tree_search_time,
                           search_results[1]);

    if (search_results[0] != search_results[1]) {
        spdlog::warn("Search results are not equal");
    }
    search_results.fill(0);

    // Generate 1e3 boxes
    max_boxes = 1000;
    generate_boxes(*model.surface, max_boxes, boxes);
    sw.reset();
    for (auto& box : boxes) {
        auto res = kd_tree.search(box);
        search_results[0] += res->size();
        delete res;
    }
    kd_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tKDTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           kd_tree_search_time,
                           search_results[0]);

    sw.reset();
    for (auto& box : boxes) {
        auto res = aabb_tree.search(box);
        search_results[1] += res.size();
    }
    aabb_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tAABBTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           aabb_tree_search_time,
                           search_results[1]);

    if (search_results[0] != search_results[1]) {
        spdlog::warn("Search results are not equal");
    }
    search_results.fill(0);

    // Generate 1e4 boxes
    max_boxes = 10000;
    generate_boxes(*model.surface, max_boxes, boxes);
    sw.reset();
    for (auto& box : boxes) {
        auto res = kd_tree.search(box);
        search_results[0] += res->size();
        delete res;
    }
    kd_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tKDTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           kd_tree_search_time,
                           search_results[0]);

    sw.reset();
    for (auto& box : boxes) {
        auto res = aabb_tree.search(box);
        search_results[1] += res.size();
    }
    aabb_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tAABBTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           aabb_tree_search_time,
                           search_results[1]);

    if (search_results[0] != search_results[1]) {
        spdlog::warn("Search results are not equal");
    }
    search_results.fill(0);

    // Generate 1e5 boxes
    max_boxes = 100000;
    generate_boxes(*model.surface, max_boxes, boxes);
    sw.reset();
    for (auto& box : boxes) {
        auto res = kd_tree.search(box);
        search_results[0] += res->size();
        delete res;
    }
    kd_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tKDTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           kd_tree_search_time,
                           search_results[0]);

    sw.reset();
    for (auto& box : boxes) {
        auto res = aabb_tree.search(box);
        search_results[1] += res.size();
    }
    aabb_tree_search_time = sw.elapsed().count();
    benchmark_logger->info("\tAABBTree search with {} boxes took {} s and find {} results",
                           max_boxes,
                           aabb_tree_search_time,
                           search_results[1]);
    if (search_results[0] != search_results[1]) {
        spdlog::warn("Search results are not equal");
    }

    benchmark_logger->info("=====End Benchmark=====");
}
