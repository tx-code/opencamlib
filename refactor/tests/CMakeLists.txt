enable_testing()

find_package(GTest CONFIG REQUIRED)

include(GoogleTest)

add_executable(OCL_Tests
        main.cpp
        geo/test_point.cpp)

target_link_libraries(OCL_Tests
        ocl
        GTest::gtest
        GTest::gtest_main
        GTest::gmock
        GTest::gmock_main
)

add_test(AllTestsInMain OCL_Tests)
