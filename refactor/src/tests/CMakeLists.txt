# 测试模块的CMakeLists.txt

find_package(Boost REQUIRED COMPONENTS unit_test_framework)

# Define test data directory
set(TEST_DATA_DIR "${CMAKE_CURRENT_SOURCE_DIR}/data")
file(MAKE_DIRECTORY ${TEST_DATA_DIR})

# Create a function to add tests easily
function(add_boost_test TEST_NAME TEST_SOURCE)
    add_executable(${TEST_NAME} ${TEST_SOURCE})
    target_include_directories(${TEST_NAME}
        PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
    )
    target_link_libraries(${TEST_NAME}
        PRIVATE
        Boost::unit_test_framework
        opencamlib

        # Add other dependencies needed by your tests
    )

    # Define test target
    add_test(NAME ${TEST_NAME} COMMAND ${TEST_NAME})

    # Tell Boost.Test to use dynamic linking
    target_compile_definitions(${TEST_NAME} PRIVATE
        BOOST_TEST_DYN_LINK
        TEST_DATA_DIR="${TEST_DATA_DIR}"
    )
endfunction()

# 添加测试文件
add_boost_test(test_geometry test_geometry.cpp)
add_boost_test(test_cutter test_cutter.cpp)
