/*  $Id$
 *
 *  Copyright (c) 2010-2011 Anders Wallin (anders.e.e.wallin "at" gmail.com).
 *
 *  This file is part of OpenCAMlib
 *  (see https://github.com/aewallin/opencamlib).
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 2.1 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Lesser General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <boost/foreach.hpp>

#ifdef _OPENMP
#include <omp.h>
#endif

#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/tbb.h>

#include "batchpushcutter.hpp"
#include "cutters/millingcutter.hpp"
#include "geo/point.hpp"
#include "geo/triangle.hpp"
#include "waterline.hpp"

// #include "weave.hpp"
#include "simple_weave.hpp"
#include "smart_weave.hpp"

namespace ocl
{

//********   ********************** */

Waterline::Waterline()
{
    subOp.clear();
    subOp.push_back(new BatchPushCutter());
    subOp.push_back(new BatchPushCutter());
    subOp[0]->setXDirection();
    subOp[1]->setYDirection();
    nthreads = 1;
#ifdef _OPENMP
    nthreads = omp_get_num_procs();
    // omp_set_dynamic(0);
    // omp_set_nested(1);
#endif
}

Waterline::~Waterline()
{
    // std::cout << "~Waterline(): subOp.size()= " << subOp.size() <<"\n";
    delete subOp[1];
    delete subOp[0];
    subOp.clear();
}


// run the batchpuschutter sub-operations to get x- and y-fibers
// pass the fibers to weave, and process the weave to get waterline-loops
void Waterline::run2()
{
    init_fibers();
    if (force_use_tbb) {
        // these two are independent, so could/should run in parallel
        subOp[0]->setForceUseTBB(force_use_tbb);
        subOp[1]->setForceUseTBB(force_use_tbb);
        tbb::parallel_invoke(
            [&]() {
                subOp[0]->run();
            },
            [&]() {
                subOp[1]->run();
            });
    }
    else {
        subOp[0]->run();
        subOp[1]->run();
    }

    xfibers = *(subOp[0]->getFibers());
    yfibers = *(subOp[1]->getFibers());

    weave_process2();
}

void Waterline::run()
{
    init_fibers();
    subOp[0]->run();  // these two are independent, so could/should run in parallel
    subOp[1]->run();

    xfibers = *(subOp[0]->getFibers());
    yfibers = *(subOp[1]->getFibers());

    weave_process();
}


void Waterline::reset()
{
    xfibers.clear();
    yfibers.clear();
    zhs.clear();
    subOp[0]->reset();
    subOp[1]->reset();
}

void Waterline::weave_process()
{
    // std::cout << "Weave...\n" << std::flush;
    weave::SimpleWeave weave;
    BOOST_FOREACH (Fiber f, xfibers) {
        weave.addFiber(f);
    }
    BOOST_FOREACH (Fiber f, yfibers) {
        weave.addFiber(f);
    }

    // std::cout << "Weave::build()..." << std::flush;
    weave.build();
    // std::cout << "done.\n";

    // std::cout << "Weave::face traverse()...";
    weave.face_traverse();
    // std::cout << "done.\n";

    // std::cout << "Weave::get_loops()...";
    loops = weave.getLoops();
    // std::cout << "done.\n";
}

void Waterline::weave_process2()
{
    // std::cout << "Weave...\n" << std::flush;
    weave::SmartWeave weave;
    BOOST_FOREACH (Fiber f, xfibers) {
        weave.addFiber(f);
    }
    BOOST_FOREACH (Fiber f, yfibers) {
        weave.addFiber(f);
    }

    // std::cout << "Weave::build2()..." << std::flush;
    weave.build();
    // std::cout << "done.\n";

    // std::cout << "Weave::face traverse()...";
    weave.face_traverse();
    // std::cout << "done.\n";

    // std::cout << "Weave::get_loops()...";
    loops = weave.getLoops();
    // std::cout << "done.\n";
}

void Waterline::init_fibers()
{
    // std::cout << " Waterline::init_fibers()\n";
    double minx = surf->bb.minpt.x - 2 * cutter->getRadius();
    double maxx = surf->bb.maxpt.x + 2 * cutter->getRadius();
    double miny = surf->bb.minpt.y - 2 * cutter->getRadius();
    double maxy = surf->bb.maxpt.y + 2 * cutter->getRadius();
    int Nx = (int)((maxx - minx) / sampling);
    int Ny = (int)((maxy - miny) / sampling);
    std::vector<double> xvals = generate_range(minx, maxx, Nx);
    std::vector<double> yvals = generate_range(miny, maxy, Ny);
    BOOST_FOREACH (double y, yvals) {
        Point p1 = Point(minx, y, zh);
        Point p2 = Point(maxx, y, zh);
        Fiber f = Fiber(p1, p2);
        subOp[0]->appendFiber(f);
    }
    BOOST_FOREACH (double x, xvals) {
        Point p1 = Point(x, miny, zh);
        Point p2 = Point(x, maxy, zh);
        Fiber f = Fiber(p1, p2);
        subOp[1]->appendFiber(f);
    }
}

// return a double-vector [ start , ... , end ] with N elements
// for generating fibers.
std::vector<double> Waterline::generate_range(double start, double end, int N) const
{
    std::vector<double> output;
    double d = (end - start) / (double)N;
    double v = start;
    for (int n = 0; n < (N + 1); ++n) {
        output.push_back(v);
        v = v + d;
    }
    return output;
}

void Waterline::init_fibers_at_z(double z_height)
{
    double minx = surf->bb.minpt.x - 2 * cutter->getRadius();
    double maxx = surf->bb.maxpt.x + 2 * cutter->getRadius();
    double miny = surf->bb.minpt.y - 2 * cutter->getRadius();
    double maxy = surf->bb.maxpt.y + 2 * cutter->getRadius();
    int Nx = (int)((maxx - minx) / sampling);
    int Ny = (int)((maxy - miny) / sampling);
    std::vector<double> xvals = generate_range(minx, maxx, Nx);
    std::vector<double> yvals = generate_range(miny, maxy, Ny);

    xfibers.clear();
    yfibers.clear();
    subOp[0]->reset();
    subOp[1]->reset();

    BOOST_FOREACH (double y, yvals) {
        Point p1 = Point(minx, y, z_height);
        Point p2 = Point(maxx, y, z_height);
        Fiber f = Fiber(p1, p2);
        subOp[0]->appendFiber(f);
    }
    BOOST_FOREACH (double x, xvals) {
        Point p1 = Point(x, miny, z_height);
        Point p2 = Point(x, maxy, z_height);
        Fiber f = Fiber(p1, p2);
        subOp[1]->appendFiber(f);
    }
}

// run the waterline algorithm for multiple z-heights using TBB acceleration
void Waterline::run3()
{
    if (zhs.empty()) {
        // 如果没有设置z高度序列，则使用单一z高度
        zhs.push_back(zh);
    }
    
    // 保存原始循环结果，以便附加多个z高度的结果
    loops.clear();
    
    // 如果只有一个Z高度，不使用parallel_for
    if (zhs.size() == 1) {
        double z = zhs[0];
        
        // 重置并为当前z高度初始化fibers
        init_fibers_at_z(z);
        
        // 两个子操作并行执行，使用TBB
        tbb::parallel_invoke(
            [&]() { 
                subOp[0]->setForceUseTBB(force_use_tbb); 
                subOp[0]->run(); 
            },
            [&]() { 
                subOp[1]->setForceUseTBB(force_use_tbb); 
                subOp[1]->run(); 
            });
        
        xfibers = *(subOp[0]->getFibers());
        yfibers = *(subOp[1]->getFibers());
        
        // 使用SmartWeave处理
        weave::SmartWeave weave;
        BOOST_FOREACH(Fiber f, xfibers) {
            weave.addFiber(f);
        }
        BOOST_FOREACH(Fiber f, yfibers) {
            weave.addFiber(f);
        }
        
        weave.build();
        weave.face_traverse();
        
        // 保存结果
        loops = weave.getLoops();
        return;
    }
    
    // 创建单独的BatchPushCutter实例来处理多个Z高度
    // 每个z高度的fiber都有唯一标识，以便后续区分
    std::vector<double> minx_per_z(zhs.size());
    std::vector<double> maxx_per_z(zhs.size());
    std::vector<double> miny_per_z(zhs.size());
    std::vector<double> maxy_per_z(zhs.size());
    std::vector<int> Nx_per_z(zhs.size());
    std::vector<int> Ny_per_z(zhs.size());
    
    std::vector<std::vector<Fiber>> x_fibers_per_z(zhs.size());
    std::vector<std::vector<Fiber>> y_fibers_per_z(zhs.size());
    
    // 为所有Z高度准备fibers
    // 重置当前的subOp操作对象
    subOp[0]->reset();
    subOp[1]->reset();
    
    // 为每个Z高度创建fibers
    for (size_t i = 0; i < zhs.size(); i++) {
        double z = zhs[i];
        
        // 计算边界
        minx_per_z[i] = surf->bb.minpt.x - 2*cutter->getRadius();
        maxx_per_z[i] = surf->bb.maxpt.x + 2*cutter->getRadius();
        miny_per_z[i] = surf->bb.minpt.y - 2*cutter->getRadius();
        maxy_per_z[i] = surf->bb.maxpt.y + 2*cutter->getRadius();
        Nx_per_z[i] = (int)((maxx_per_z[i] - minx_per_z[i])/sampling);
        Ny_per_z[i] = (int)((maxy_per_z[i] - miny_per_z[i])/sampling);
        
        std::vector<double> xvals = generate_range(minx_per_z[i], maxx_per_z[i], Nx_per_z[i]);
        std::vector<double> yvals = generate_range(miny_per_z[i], maxy_per_z[i], Ny_per_z[i]);
        
        // 为当前Z高度创建fibers并添加到subOp
        BOOST_FOREACH(double y, yvals) {
            Point p1 = Point(minx_per_z[i], y, z);
            Point p2 = Point(maxx_per_z[i], y, z);
            Fiber f = Fiber(p1, p2);
            subOp[0]->appendFiber(f);
            x_fibers_per_z[i].push_back(f);
        }
        
        BOOST_FOREACH(double x, xvals) {
            Point p1 = Point(x, miny_per_z[i], z);
            Point p2 = Point(x, maxy_per_z[i], z);
            Fiber f = Fiber(p1, p2);
            subOp[1]->appendFiber(f);
            y_fibers_per_z[i].push_back(f);
        }
    }
    
    // 配置并运行subOp - 同时处理所有Z高度的fibers
    subOp[0]->setForceUseTBB(force_use_tbb);
    subOp[1]->setForceUseTBB(force_use_tbb);
    
    // 并行执行X和Y方向的处理
    tbb::parallel_invoke(
        [&]() { subOp[0]->run(); },
        [&]() { subOp[1]->run(); }
    );
    
    // 获取处理后的所有fibers
    std::vector<Fiber>* all_x_fibers = subOp[0]->getFibers();
    std::vector<Fiber>* all_y_fibers = subOp[1]->getFibers();
    
    // 为每个Z高度创建weave并生成loops
    std::vector<std::vector<std::vector<Point>>> all_loops(zhs.size());
    
    // 并行处理每个Z高度的weave
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, zhs.size()),
        [&](const tbb::blocked_range<size_t>& range) {
            for (size_t i = range.begin(); i != range.end(); ++i) {
                // 计算每个Z高度fiber的起始和结束索引
                size_t x_start = 0;
                size_t y_start = 0;
                
                for (size_t j = 0; j < i; j++) {
                    x_start += x_fibers_per_z[j].size();
                    y_start += y_fibers_per_z[j].size();
                }
                
                size_t x_count = x_fibers_per_z[i].size();
                size_t y_count = y_fibers_per_z[i].size();
                
                // 创建当前Z高度的weave
                weave::SmartWeave weave;
                
                // 添加已处理的x-fibers到weave
                for (size_t j = 0; j < x_count; j++) {
                    weave.addFiber((*all_x_fibers)[x_start + j]);
                }
                
                // 添加已处理的y-fibers到weave
                for (size_t j = 0; j < y_count; j++) {
                    weave.addFiber((*all_y_fibers)[y_start + j]);
                }
                
                // 构建并处理weave
                weave.build();
                weave.face_traverse();
                
                // 保存当前z高度的结果
                all_loops[i] = weave.getLoops();
            }
        },
        tbb::auto_partitioner()
    );
    
    // 合并所有z高度的结果
    for (size_t i = 0; i < all_loops.size(); ++i) {
        const auto& z_loops = all_loops[i];
        loops.insert(loops.end(), z_loops.begin(), z_loops.end());
    }
}

}  // namespace ocl
// end file waterline.cpp
