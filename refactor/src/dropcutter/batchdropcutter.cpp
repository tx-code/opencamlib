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

#include <atomic>
#include <boost/foreach.hpp>
#include <tbb/blocked_range.h>
#include <tbb/global_control.h>
#include <tbb/parallel_for.h>
#include <tbb/tbb.h>


#ifdef _OPENMP  // this should really not be a check for Windows, but a check for
                // OpenMP
#include <omp.h>
#endif

#include "batchdropcutter.hpp"
#include "geo/point.hpp"
#include "geo/triangle.hpp"

namespace ocl
{

//********   ********************** */

BatchDropCutter::BatchDropCutter()
{
    clpoints = new std::vector<CLPoint>();
    nCalls = 0;
#ifdef _OPENMP
    nthreads = omp_get_num_procs();  // figure out how many cores we have
#endif
    cutter = NULL;
    bucketSize = 1;
    root = new KDTree<Triangle>();
}

BatchDropCutter::~BatchDropCutter()
{
    clpoints->clear();
    delete clpoints;
    delete root;
}

void BatchDropCutter::run()
{
    if (force_use_tbb) {
        dropCutter6();
    }
    else {
        dropCutter5();
    }
}
void BatchDropCutter::setSTL(const STLSurf& s)
{
    // std::cout << "bdc::setSTL()\n";
    surf = &s;
    root->setXYDimensions();  // we search for triangles in the XY plane, don't
                              // care about Z-coordinate
    root->setBucketSize(bucketSize);
    root->build(s.tris);
    // std::cout << "bdc::setSTL() done.\n";
}

void BatchDropCutter::appendPoint(CLPoint& p)
{
    clpoints->push_back(p);
}

// drop cutter against all triangles in surface
void BatchDropCutter::dropCutter1()
{
    // std::cout << "dropCutterSTL1 " << clpoints->size() <<
    //           " cl-points and " << surf->tris.size() << " triangles...";
    nCalls = 0;
    BOOST_FOREACH (CLPoint& cl, *clpoints) {
        BOOST_FOREACH (const Triangle& t,
                       surf->tris) {  // test against all triangles in s
            cutter->dropCutter(cl, t);
            ++nCalls;
        }
    }
    // std::cout << "done.\n";
    return;
}

// first search for triangles under the cutter
// then only drop cutter against found triangles
void BatchDropCutter::dropCutter2()
{
    // std::cout << "dropCutterSTL2 " << clpoints->size() <<
    //         " cl-points and " << surf->tris.size() << " triangles.\n";
    std::cout.flush();
    nCalls = 0;
    std::list<Triangle>* triangles_under_cutter;
    BOOST_FOREACH (CLPoint& cl, *clpoints) {  // loop through each CL-point
        triangles_under_cutter = root->search_cutter_overlap(cutter, &cl);
        BOOST_FOREACH (const Triangle& t, *triangles_under_cutter) {
            cutter->dropCutter(cl, t);
            ++nCalls;
        }
        delete triangles_under_cutter;
    }

    // std::cout << "done. " << nCalls << " dropCutter() calls.\n";
    std::cout.flush();
    return;
}

// compared to dropCutter2, add an additional explicit overlap-test before
// testing triangle
void BatchDropCutter::dropCutter3()
{
    // std::cout << "dropCutterSTL3 " << clpoints->size() <<
    //         " cl-points and " << surf->tris.size() << " triangles.\n";
    nCalls = 0;
    std::list<Triangle>* triangles_under_cutter;
    BOOST_FOREACH (CLPoint& cl, *clpoints) {  // loop through each CL-point
        triangles_under_cutter = root->search_cutter_overlap(cutter, &cl);
        BOOST_FOREACH (const Triangle& t, *triangles_under_cutter) {
            if (cutter->overlaps(cl, t)) {
                if (cl.below(t)) {
                    cutter->dropCutter(cl, t);
                    ++nCalls;
                }
            }
        }
        delete triangles_under_cutter;
    }

    // std::cout << "done. " << nCalls << " dropCutter() calls.\n";
    return;
}

// use OpenMP to share work between threads
void BatchDropCutter::dropCutter4()
{
    // std::cout << "dropCutterSTL4 " << clpoints->size() <<
    //         " cl-points and " << surf->tris.size() << " triangles.\n";
    nCalls = 0;
    int calls = 0;
    std::list<Triangle>* tris;
#ifdef _WIN32  // OpenMP version 2 of VS2013 OpenMP need signed loop variable
    int n;  // loop variable
    int Nmax = static_cast<int>(clpoints->size());
#else
    unsigned int n;  // loop variable
    unsigned int Nmax = clpoints->size();
#endif
    std::vector<CLPoint>& clref = *clpoints;
#ifdef _OPENMP
    omp_set_num_threads(nthreads);  // the constructor sets number of threads right
                                    // or the user can explicitly specify something else
#endif
    std::list<Triangle>::iterator it;
#pragma omp parallel for shared(calls, clref) private(n, tris, it)
    for (n = 0; n < Nmax; n++) {  // PARALLEL OpenMP loop!
                                  // #ifdef _OPENMP
                                  //             if ( n== 0 ) { // first iteration
        //                 if (omp_get_thread_num() == 0 )
        //                     std::cout << "Number of OpenMP threads = "<<
        //                     omp_get_num_threads() << "\n";// print out how many
        //                     threads we are using
        //             }
        // #endif
        tris = root->search_cutter_overlap(cutter, &clref[n]);
        // assert( tris->size() <= ntriangles ); // can't possibly find more
        // triangles than in the STLSurf
        for (it = tris->begin(); it != tris->end(); ++it) {  // loop over found triangles
            if (cutter->overlaps(clref[n], *it)) {           // cutter overlap triangle? check
                if (clref[n].below(*it)) {
                    cutter->vertexDrop(clref[n], *it);
                    ++calls;
                }
            }
        }
        for (it = tris->begin(); it != tris->end(); ++it) {  // loop over found triangles
            if (cutter->overlaps(clref[n], *it)) {           // cutter overlap triangle? check
                if (clref[n].below(*it))
                    cutter->facetDrop(clref[n], *it);
            }
        }
        for (it = tris->begin(); it != tris->end(); ++it) {  // loop over found triangles
            if (cutter->overlaps(clref[n], *it)) {           // cutter overlap triangle? check
                if (clref[n].below(*it))
                    cutter->edgeDrop(clref[n], *it);
            }
        }
        delete (tris);
    }  // end OpenMP PARALLEL for
    nCalls = calls;
    // std::cout << " " << nCalls << " dropCutter() calls.\n";
    return;
}

// use OpenMP to share work between threads
void BatchDropCutter::dropCutter5()
{
    // std::cout << "dropCutterSTL5 " << clpoints->size() <<
    //         " cl-points and " << surf->tris.size() << " triangles.\n";
    nCalls = 0;
    int calls = 0;
    std::list<Triangle>* tris;
#ifdef _WIN32  // OpenMP version 2 of VS2013 OpenMP need signed loop variable
    int Nmax = static_cast<int>(clpoints->size());
    int n;  // loop variable
#else
    unsigned int Nmax = clpoints->size();
    unsigned int n;  // loop variable
#endif
    std::vector<CLPoint>& clref = *clpoints;

#ifdef _OPENMP
    omp_set_num_threads(nthreads);  // the constructor sets number of threads right
                                    // or the user can explicitly specify something else
#endif
    std::list<Triangle>::iterator it;
#pragma omp parallel for schedule(dynamic) shared(calls, clref) private(n, tris, it)
    for (n = 0; n < Nmax; ++n) {  // PARALLEL OpenMP loop!
                                  // #ifdef _OPENMP
                                  //             if ( n== 0 ) { // first iteration
        //                 if (omp_get_thread_num() == 0 )
        //                     std::cout << "Number of OpenMP threads = "<<
        //                     omp_get_num_threads() << "\n";
        //             }
        // #endif
        tris = root->search_cutter_overlap(cutter, &clref[n]);
        assert(tris);
        // assert( tris->size() <= ntriangles ); // can't possibly find more
        // triangles than in the STLSurf
        for (it = tris->begin(); it != tris->end(); ++it) {  // loop over found triangles
            if (cutter->overlaps(clref[n], *it)) {           // cutter overlap triangle? check
                if (clref[n].below(*it)) {
                    cutter->dropCutter(clref[n], *it);
                    ++calls;
                }
            }
        }
        delete (tris);
    }  // end OpenMP PARALLEL for
    nCalls = calls;
    // std::cout << "\n " << nCalls << " dropCutter() calls.\n";
    return;
}

void BatchDropCutter::dropCutter6()
{
    nCalls = 0;
    std::vector<CLPoint>& clref = *clpoints;
    unsigned int Nmax = clpoints->size();

    // 使用较大的粒度，减少任务创建的开销
    size_t grain_size = std::max(
        size_t(100),
        Nmax
            / (4
               * tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism)));

    // 使用combinable来收集每个线程本地的调用计数，避免频繁的原子操作
    tbb::combinable<int> local_calls([]() {
        return 0;
    });

    // 使用单层并行和auto_partitioner自动调整工作分配
    tbb::parallel_for(
        tbb::blocked_range<size_t>(0, Nmax, grain_size),
        [&](const tbb::blocked_range<size_t>& range) {
            int thread_local_calls = 0;
            std::list<Triangle>::iterator it;

            // 处理当前线程分配到的点
            for (size_t n = range.begin(); n != range.end(); ++n) {
                std::list<Triangle>* tris = root->search_cutter_overlap(cutter, &clref[n]);
                assert(tris);

                // 直接处理所有三角形，不进行二次并行
                for (it = tris->begin(); it != tris->end(); ++it) {
                    if (cutter->overlaps(clref[n], *it)) {
                        if (clref[n].below(*it)) {
                            cutter->dropCutter(clref[n], *it);
                            ++thread_local_calls;
                        }
                    }
                }

                delete tris;
            }

            // 更新线程本地计数
            local_calls.local() += thread_local_calls;
        },
        tbb::auto_partitioner());  // 使用auto_partitioner而不是固定粒度

    // 合并所有线程的结果
    nCalls = local_calls.combine([](int x, int y) {
        return x + y;
    });
    return;
}

}  // namespace ocl
// end file batchdropcutter.cpp
