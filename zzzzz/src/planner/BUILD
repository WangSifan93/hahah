load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planner",
    srcs = glob([
        "*.cc",
        "public_road/*.cc"
    ]),
    hdrs = glob([
        "*.h",
        "public_road/*.h"
    ]),
    deps = [
        "//apps/planning/src/planning_msgs:planning_msgs",
        "@common_basic_msgs//:basic_msgs",
        "@common_utils//:util",
        "@common_status//:status",
        "//apps/planning/src/common:common",
        "//apps/planning/src/tasks:task",
        "//apps/planning/src/scenarios:scenario",
        "//apps/planning/src/scenarios:scenario_manager"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

