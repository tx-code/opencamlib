enable_testing()

find_package(GTest CONFIG REQUIRED)

include(GoogleTest)

add_executable(OCL_Tests
        utils/triangles_utils.cpp
        main.cpp
        geo/test_point.cpp
        cutters/test_cylcutter.cpp
        cutters/test_ballcutter.cpp
        cutters/test_bullcutter.cpp
        cutters/test_conecutter.cpp
        cutters/test_pushcutter.cpp)

target_link_libraries(OCL_Tests
        ocl
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        GTest::gmock_main
)

add_test(AllTestsInMain OCL_Tests)
