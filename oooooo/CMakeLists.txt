cmake_minimum_required(VERSION 3.10)
project(e2e_pnc_interface)

# 开启 compile_commands.json 生成
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

# 全局抑制三方库、proto等 warning
add_compile_options(-w)

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
)

if(GIT_FOUND)
    execute_process(
        COMMAND ${GIT_EXECUTABLE} rev-parse --short HEAD
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/pnc_interface
        OUTPUT_VARIABLE GIT_COMMIT_HASH
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    execute_process(
        COMMAND ${GIT_EXECUTABLE} log -1 --format=%ci
        WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/pnc_interface
        OUTPUT_VARIABLE GIT_COMMIT_TIMESTAMP
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE GIT_EXEC_RESULT
    )
    message(STATUS "Found Git commit hash: ${GIT_COMMIT_HASH}")
else()
    set(GIT_COMMIT_HASH "unknown")
    message(WARNING "Git not found. Commit hash will be set to 'unknown'.")
endif()

set(COMMON_LIBS
    ${messages_LIBRARIES}
    ${zlog_LIBRARIES}
    ${fmt_LIBRARIES}
    ${myabsl_LIBRARIES}
    ${gtest_LIBRARIES}
    ${osqp_LIBRARIES}
    ${tros_LIBRARIES}
    ${eigen3_LIBRARIES}
    ${boost_LIBRARIES}
    ${Protobuf_LIBRARIES}
    ${glog_LIBRARIES}
)

set(E2E_ALGO_LIST
    dispatcher_selector
    pnp_decision
    interactive_speedplan
    trajectory_optimizer
)

set(E2E_ALGO_API_HDS_LIST)
foreach(E2E_ALGO ${E2E_ALGO_LIST})
    file(GLOB_RECURSE ${E2E_ALGO}_HDRS
    "../${E2E_ALGO}/*.h"
    "../${E2E_ALGO}/*.hpp"
    )
    file(GLOB_RECURSE ${E2E_ALGO}_SRCS
        "../${E2E_ALGO}/*.cc"
        "../${E2E_ALGO}/*.cpp"
    )
    list(FILTER ${E2E_ALGO}_SRCS EXCLUDE REGEX ".*_test.cc")
    list(FILTER ${E2E_ALGO}_SRCS EXCLUDE REGEX ".*/example.*")

    if(${E2E_ALGO}_SRCS)
        set(${E2E_ALGO}_LIB_HDRS
            ${CMAKE_CURRENT_SOURCE_DIR}/../${E2E_ALGO}/include
        )
    else()
        add_library(${E2E_ALGO} SHARED IMPORTED)
        set_target_properties(${E2E_ALGO} PROPERTIES
            IMPORTED_LOCATION "${CMAKE_CURRENT_SOURCE_DIR}/deps/${E2E_ALGO}/${MY_PLATFORM}/lib${E2E_ALGO}.so"
            INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/deps/${E2E_ALGO}/include"
        )
        set(${E2E_ALGO}_LIB_HDRS
            ${CMAKE_CURRENT_SOURCE_DIR}/deps/${E2E_ALGO}/include
        )
    endif()

    list(APPEND E2E_ALGO_API_HDS_LIST
        ${${E2E_ALGO}_LIB_HDRS}
    )
    if (E2E_ALGO STREQUAL "trajectory_optimizer")
        list(APPEND E2E_ALGO_API_HDS_LIST
            ${${E2E_ALGO}_LIB_HDRS}/optimization
        )
    endif()
endforeach()

list(APPEND E2E_BASE_HEARDER_DIRS
     deps/cereal/include
     deps/LBFGSpp/include
     src/base_algorithm/interactive_speedplan
     src
     ${e2e_planning_protos_INCLUDE_DIRS}

     ${E2E_ALGO_API_HDS_LIST}
)

include_directories(${E2E_BASE_HEARDER_DIRS})

# libe2e_common.so 库
file(GLOB_RECURSE E2E_COMMON_SRCS
    RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}
    "*.cc"
    "*.cpp"
)
list(REMOVE_ITEM E2E_COMMON_SRCS
    src/e2e_pnc_component.cc
    src/plan/decision_exploration.cc
    src/plan/fallback_planner.cc
    src/plan/st_planner.cc
    src/plan/st_path_planner.cc
    src/nodes/**/*.cc
    src/nodes/**/*.cpp
    src/plan/spatio_temporal_planner.cc
    src/spatio_temporal_planner/src/ilqr_solver.cc
    src/spatio_temporal_planner/src/spatio_temporal_cost.cc
    src/spatio_temporal_planner/src/spatio_temporal_ilqr.cc
)
list(FILTER E2E_COMMON_SRCS EXCLUDE REGEX ".*_test.cc")
list(FILTER E2E_COMMON_SRCS EXCLUDE REGEX ".*/example.*")

add_library(e2e_common SHARED ${E2E_COMMON_SRCS})
target_link_libraries(e2e_common PRIVATE
    e2e_planning_protos
    ${gflags_LIBRARIES}
    ${COMMON_LIBS}
)
add_dependencies(e2e_common proto_target)
target_compile_options(e2e_common PRIVATE -w)

if(dispatcher_selector_SRCS)
    list(APPEND DISPATCHER_SELECTOR_HEADER_DIRS
         ../dispatcher_selector/
    )

    add_library(dispatcher_selector SHARED ${dispatcher_selector_SRCS})
    target_link_libraries(dispatcher_selector PRIVATE
            e2e_planning_protos
            e2e_common
            ${COMMON_LIBS}
    )
    target_include_directories(dispatcher_selector PRIVATE
            ${E2E_BASE_HEADER_DIRS}
            ${DISPATCHER_SELECTOR_HEADER_DIRS}
    )
    target_compile_options(dispatcher_selector PRIVATE -w)
endif()

if(pnp_decision_SRCS)
    list(APPEND PNP_DECISION_HEADER_DIRS
        ../pnp_decision/
    )
    add_library(pnp_decision SHARED ${pnp_decision_SRCS})
    target_link_libraries(pnp_decision PRIVATE
        e2e_common
        e2e_planning_protos
        dispatcher_selector
        ${COMMON_LIBS}
    )
    target_include_directories(pnp_decision PRIVATE
        ${E2E_BASE_HEADER_DIRS}
        ${PNP_DECISION_HEADER_DIRS})
    target_compile_options(pnp_decision PRIVATE -w)
endif()

if(interactive_speedplan_SRCS)
    list(APPEND INTERACTIVE_SPEEDPLAN_HEADER_DIRS
        ../interactive_speedplan/
    )
    add_library(interactive_speedplan SHARED ${interactive_speedplan_SRCS})
    target_link_libraries(interactive_speedplan PRIVATE
        dispatcher_selector
        pnp_decision
        e2e_common
        e2e_planning_protos
        ${COMMON_LIBS}
    )
    target_include_directories(interactive_speedplan PRIVATE
        ${E2E_BASE_HEADER_DIRS}
        ${INTERACTIVE_SPEEDPLAN_HEADER_DIRS})
    target_compile_options(interactive_speedplan PRIVATE -w)
endif()

if(trajectory_optimizer_SRCS)
    list(APPEND OPT_HEADER_DIRS
        ../trajectory_optimizer/
        ../trajectory_optimizer/optimization
    )
    add_library(trajectory_optimizer SHARED ${trajectory_optimizer_SRCS})
    target_link_libraries(trajectory_optimizer PRIVATE
        e2e_common
        e2e_planning_protos
        dispatcher_selector
        ${COMMON_LIBS}
    )
    target_include_directories(trajectory_optimizer PRIVATE
        ${E2E_BASE_HEADER_DIRS}
        ${OPT_HEADER_DIRS})
    target_compile_options(trajectory_optimizer PRIVATE -w)
endif()

# libe2e_pnc.so 库
add_library(e2e_pnc SHARED
    src/e2e_pnc_component.cc

    src/nodes/e2e_planning_core.cpp
    src/nodes/planner/city/city_planner.cpp
    src/nodes/planner/city/msg_proxy.cpp
    src/plan/main_planner.cc
    src/plan/fallback_planner.cc
    src/plan/st_planner.cc
    src/plan/st_path_planner.cc
)
target_link_libraries(e2e_pnc PRIVATE
    e2e_common
    e2e_planning_protos

    dispatcher_selector
    pnp_decision
    interactive_speedplan
    trajectory_optimizer
    ${COMMON_LIBS}
)
target_compile_options(e2e_pnc PRIVATE -w)
target_compile_definitions(e2e_pnc PRIVATE 
    "GIT_COMMIT_TIMESTAMP=\"${GIT_COMMIT_TIMESTAMP}\""
    "GIT_COMMIT_HASH=\"${GIT_COMMIT_HASH}\""
)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "${MY_PLATFORM}_64")
    set_target_properties(e2e_common PROPERTIES COMPILE_FLAGS "-march=${MY_PLATFORM}-64")
    set_target_properties(e2e_pnc PROPERTIES COMPILE_FLAGS "-march=${MY_PLATFORM}-64")
endif()

option(BUILD_TESTS "Build unit tests" ON)
if(BUILD_TESTS)
    add_subdirectory(spt_test)
endif()