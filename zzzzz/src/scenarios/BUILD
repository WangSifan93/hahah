load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "scenario_manager",
    srcs = ["scenario_manager.cc"],
    hdrs = ["scenario_manager.h"],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common",
        "//apps/planning/src/common/configs",
        "//apps/planning/src/common/util",
        "//apps/planning/src/common/vehicle_state",
        "//apps/planning/src/config",
        "//apps/planning/src/planning_msgs",
        "//apps/planning/src/reference_line_provider/pnc_map",
        "//apps/planning/src/scenarios/lane_follow",
        "//messages/mapfusion:all_map_cc_proto",
        "@common_status//:status",
        "@common_utils//:util",
    ],
)

cc_library(
    name = "scenario",
    srcs = ["scenario.cc"],
    hdrs = ["scenario.h"],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        ":stage",
        "//apps/planning/src/common",
        "//apps/planning/src/config",
        "//apps/planning/src/planning_msgs",
        "//apps/planning/src/tasks:task",
        "@common_status//:status",
        "@common_utils//:util",
    ],
)

cc_library(
    name = "stage",
    srcs = ["stage.cc"],
    hdrs = ["stage.h"],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common",
        "//apps/planning/src/common/trajectory",
        "//apps/planning/src/planning_msgs",
        "//apps/planning/src/tasks:task",
        "//apps/planning/src/tasks:task_factory",
        "@common_status//:status",
        "@common_utils//:util",
    ],
)
