cmake_minimum_required(VERSION 3.10)
project(e2e_pnc_tests)

# # 继承主项目的配置
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 启用测试
enable_testing()

find_package(messages REQUIRED)
find_package(google_protobuf REQUIRED)
find_package(eigen3 REQUIRED)
find_package(zlog REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
find_package(tros REQUIRED)
find_package(fmt REQUIRED)
find_package(boost REQUIRED)
find_package(osqp REQUIRED)
find_package(myabsl REQUIRED)
find_package(gtest REQUIRED)
find_package(Git QUIET)

# 链接主项目的库和头文件
include_directories(
    ${messages_INCLUDE_DIRS}
    ${Protobuf_INCLUDE_DIRS}
    ${eigen3_INCLUDE_DIRS}
    ${zlog_INCLUDE_DIRS}
    ${glog_INCLUDE_DIRS}
    ${gflags_INCLUDE_DIRS}
    ${tros_INCLUDE_DIRS}
    ${boost_INCLUDE_DIRS}
    ${osqp_INCLUDE_DIRS}/osqp
    ${myabsl_INCLUDE_DIRS}
    ${gtest_INCLUDE_DIRS}
    ${fmt_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/../src
    ${CMAKE_BINARY_DIR}/pnc_interface/proto
)
add_dependencies(e2e_common proto_target)
# 添加 GTest 依赖
# find_package(gtest REQUIRED)
# include_directories(${gtest_INCLUDE_DIRS})

file(GLOB TEST_SOURCES "unit_tests/*.cc")

# 创建测试可执行文件
add_executable(run_unit_tests 
    ${TEST_SOURCES}
    ../src/e2e_pnc_component.cc
    ../src/nodes/e2e_planning_core.cpp
    ../src/nodes/planner/city/city_planner.cpp
    ../src/nodes/planner/city/msg_proxy.cpp
    ../src/plan/main_planner.cc
    ../src/plan/fallback_planner.cc
    ../src/plan/st_planner.cc
    ../src/plan/st_path_planner.cc
    )

add_dependencies(run_unit_tests proto_target)
target_compile_options(run_unit_tests PRIVATE -g -O0) # 禁用优化并添加调试符号
target_link_libraries(run_unit_tests PRIVATE -fsanitize=address)
# 链接主项目的库和 GTest
# target_link_libraries(run_unit_tests
#     e2e_common
#     dispatcher_selector
#     pnp_decision
#     trajectory_optimizer
#     ${COMMON_LIBS}  # 从主项目继承
# )

target_link_libraries(run_unit_tests PRIVATE
    e2e_common
    e2e_planning_protos
    dispatcher_selector
    pnp_decision
    interactive_speedplan
    trajectory_optimizer
    ${COMMON_LIBS}
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "${MY_PLATFORM}_64")
    set_target_properties(run_unit_tests PROPERTIES COMPILE_FLAGS "-march=${MY_PLATFORM}-64")
endif()

# 添加测试用例
add_test(NAME all_unit_tests COMMAND run_unit_tests)