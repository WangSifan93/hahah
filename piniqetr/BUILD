load("@rules_pkg//pkg:pkg.bzl", "pkg_tar")
load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "e2e_common_hdrs",
    hdrs = glob(["**/*.h",
                 "**/*.hpp",
                 ],
                exclude = ["src/e2e_pnc_component.h",
                           "src/plan/main_planner.h",
                           "src/plan/multi_tasks_cruise_planner_input.h",
                           "src/plan/st_planner.h",
                           "src/plan/st_path_planner.h",
                           "**/*_test.h",
                           "**/example*.h"]),
    includes = [
        "deps/cereal/include",
        "deps/LBFGSpp/include",

        "../trajectory_optimization/include",
        "../trajectory_optimization/include/optimization",

        "../pnp_decision/include",

        "../dispatcher_selector/include",
        "../interactive_speedplan/include",

        "src/base_algorithm/interactive_speedplan",
        "src",

        "proto",
    ],
    deps = [
        "//apps/e2e_pnc/pnc_interface/proto:e2e_cc_proto",
        "//cheryos/zos/zlog",
        "//cheryos/ap_services/em:execution_client",
        "//cheryos/system_services/system_monitor/include:report_cli_hdrs",
        "//messages/common:heartbeat_cc_proto",
        "//messages/localization:localization_cc_proto",
        "//messages/map_service:zdrive_map_cc_proto",
        "//messages/map_fusion:map_fusion_cc_proto",
        "//messages/prediction:prediction_cc_proto",
        "//messages/cheryos/vehicle:vehicle_chassis_pt_cc_proto",
        "//messages/planning/driving:path_and_reference_line_cc_proto",
        "//messages/planning/driving:ads_pln_cc_proto",
        "//messages/planning/driving:ads_fct_cc_proto",
        "//messages/planning/driving:noa_debug_info_cc_proto",

        "@com_google_protobuf//:protobuf",
        "//third_party:googletest",

        "//third_party:boost",
        "//third_party:myabsl",
        "//third_party:osqp",
        "//third_party:eigen3",
    ],
    visibility = ["//visibility:public"],
)

cc_binary(
    name = "libe2e_common.so",
    linkshared = True,
    linkstatic = False,
    includes = [
        "deps/cereal/include",
        "deps/LBFGSpp/include",

        "../trajectory_optimization/include",
        "../trajectory_optimization/include/optimization",

        "../pnp_decision/include",

        "../dispatcher_selector/include",
        "../interactive_speedplan/include",

        "src/base_algorithm/interactive_speedplan",
        "src",

        "proto",
    ],
    srcs = glob(["**/*.cc",
                 "**/*.cpp",
                 ],
                 exclude = ["src/e2e_pnc_component.cc",
                            "src/plan/multi_tasks_cruise_planner.cc",
                            "src/plan/fallback_planner.cc",
                            "src/plan/st_planner.cc",
                            "src/plan/st_path_planner.cc",
                            "src/nodes/**/*.cc",
                            "src/nodes/**/*.cpp",
                            "**/*_test.cc","**/example*"]),
    deps = [
        ":e2e_common_hdrs",
        "//apps/e2e_pnc/pnc_interface/proto:e2e_cc_proto",
        "//cheryos/zos/zlog",
        "//cheryos/ap_services/em:execution_client",
        "//cheryos/system_services/system_monitor/include:report_cli_hdrs",
        "//messages/common:heartbeat_cc_proto",
        "//messages/localization:localization_cc_proto",
        "//messages/map_service:zdrive_map_cc_proto",
        "//messages/map_fusion:map_fusion_cc_proto",
        "//messages/prediction:prediction_cc_proto",
        "//messages/cheryos/vehicle:vehicle_chassis_pt_cc_proto",
        "//messages/planning/driving:path_and_reference_line_cc_proto",
        "//messages/planning/driving:ads_pln_cc_proto",
        "//messages/planning/driving:ads_fct_cc_proto",
        "//messages/planning/driving:noa_debug_info_cc_proto",

        "@com_google_protobuf//:protobuf",
        "//third_party:googletest",

        "//third_party:boost",
        "//third_party:myabsl",
        "//third_party:osqp",
        "//third_party:eigen3",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//board:board_j6": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    visibility = ["//visibility:public"],
    copts = ["-std=c++17",
        "-Wno-error=unused-parameter",
        "-Wno-error=unused-variable",
        "-Wno-error=unused-but-set-variable",
        "-Wno-error=unused-function",
        "-Wno-error=switch",]
)

cc_library(
    name = "mylibe2e_common",
    srcs = [
        ":libe2e_common.so",
    ],
    hdrs = glob(["**/*.h",
                 "**/*.hpp",
                 ],
                exclude = ["src/e2e_pnc_component.h",
                           "src/plan/main_planner.h",
                           "src/plan/multi_tasks_cruise_planner_input.h",
                           "src/plan/st_planner.h",
                           "src/plan/st_path_planner.h",
                           "**/*_test.h",
                           "**/example*.h"]),
    includes = [
        "deps/cereal/include",
        "deps/LBFGSpp/include",

        "../trajectory_optimization/include",
        "../trajectory_optimization/include/optimization",

        "../pnp_decision/include",

        "../dispatcher_selector/include",
        "../interactive_speedplan/include",

        "src/base_algorithm/interactive_speedplan",
        "src",

        "proto",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "e2e_api_hdrs",
    hdrs = ["src/e2e_pnc_component.h",
            "src/plan/main_planner.h",
            "src/plan/multi_tasks_cruise_planner_input.h",
            "src/plan/st_planner.h",
            "src/plan/st_path_planner.h",],
    includes = [
        "deps/cereal/include",
        "deps/LBFGSpp/include",

        "../trajectory_optimization/include",
        "../trajectory_optimization/include/optimization",

        "../pnp_decision/include",

        "../dispatcher_selector/include",
        "../interactive_speedplan/include",
        "src/base_algorithm/interactive_speedplan",
        "src",

        "proto",
    ],
)

cc_binary(
    name = "libe2e_pnc.so",
    srcs = ["src/nodes/e2e_planning_core.cpp",
            "src/nodes/planner/city/city_planner.cpp",
            "src/nodes/planner/city/msg_proxy.cpp",
            "src/e2e_pnc_component.cc",
            "src/plan/multi_tasks_cruise_planner.cc",
            "src/plan/fallback_planner.cc",
            "src/plan/st_planner.cc",
            "src/plan/st_path_planner.cc",
           ],
    includes = [
        "deps/cereal/include",
        "deps/LBFGSpp/include",

        "../trajectory_optimization/include",
        "../trajectory_optimization/include/optimization",

        "../pnp_decision/include",

        "../dispatcher_selector/include",
        "../interactive_speedplan/include",
        "src/base_algorithm/interactive_speedplan",
        "src",

        "proto",
    ],
    deps = [
        ":e2e_api_hdrs",
        ":e2e_common_hdrs",
        ":mylibe2e_common",
        "//apps/e2e_pnc/dispatcher_selector:mylibdispatcher_selector",
        "//apps/e2e_pnc/pnp_decision:mylibpnp_decision",
        "//apps/e2e_pnc/interactive_speedplan:mylibinteractive_speedplan",
        "//apps/e2e_pnc/trajectory_optimization:mylibtrajectory_optimization",
        "//apps/e2e_pnc/pnc_interface/proto:e2e_cc_proto",

        "//cheryos/zos/zlog",
        "//messages/planning/driving:ads_fct_cc_proto",
        "//cheryos/ap_services/em:execution_client",
        "//cheryos/system_services/system_monitor/include:report_cli_hdrs",
        "//messages/common:heartbeat_cc_proto",
        "//messages/localization:localization_cc_proto",
        "//messages/map_service:zdrive_map_cc_proto",
        "//messages/map_fusion:map_fusion_cc_proto",
        "//messages/prediction:prediction_cc_proto",
        "//messages/cheryos/vehicle:vehicle_chassis_pt_cc_proto",
        "//messages/planning/driving:path_and_reference_line_cc_proto",
        "//messages/planning/driving:ads_pln_cc_proto",

        "@com_google_protobuf//:protobuf",
        "//third_party:googletest",

        "@myabsl_x86//:myabsl",
        "@eigen3_x86//:eigen3",
        "@boost_x86//:boost",
        "@osqp_x86//:osqp",
    ],
    linkshared = True,
    linkstatic = False,
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//board:board_j6": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    visibility = ["//visibility:public"],
    copts = ["-std=c++17",
            "-Wno-error=unused-parameter",
            "-Wno-error=unused-variable",
            "-Wno-error=unused-but-set-variable",
            "-Wno-error=unused-function",
            "-Wno-error=switch",]
)

