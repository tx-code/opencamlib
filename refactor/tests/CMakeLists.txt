enable_testing()

find_package(GTest CONFIG REQUIRED)

include(GoogleTest)

# 设置STL模型文件的路径
set(STL_MODELS_DIR "${CMAKE_SOURCE_DIR}/stl" CACHE PATH "Path to STL model files for testing")

add_executable(OCL_Tests
        utils/triangles_utils.cpp
        main.cpp
        geo/test_point.cpp
        cutters/test_cylcutter.cpp
        cutters/test_ballcutter.cpp
        cutters/test_bullcutter.cpp
        cutters/test_conecutter.cpp
        cutters/test_pushcutter.cpp
        cutters/test_fiberpushcutter.cpp
        cutters/test_stl_fiberpushcutter.cpp)

# 将STL目录路径定义为预处理宏，使测试代码能够访问
target_compile_definitions(OCL_Tests PRIVATE
        STL_MODELS_DIR="${STL_MODELS_DIR}")

target_link_libraries(OCL_Tests
        ocl
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        GTest::gmock_main
)

add_test(AllTestsInMain OCL_Tests)
