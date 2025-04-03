//
// Created by user on 2025/4/3.
//
#include <gtest/gtest.h>

#include "geo/point.hpp"
#include "geo/clpoint.hpp"

using namespace ocl;

TEST(GeoTests, PointOperations)
{
    Point p1(1.0, 2.0, 3.0);
    Point p2(4.0, 5.0, 6.0);

    // 向量运算测试
    auto v = p2 - p1;
    EXPECT_NEAR(v.norm(), 5.196, 0.001);

    EXPECT_NEAR(v.dot(v), 27.0, 0.001);
}