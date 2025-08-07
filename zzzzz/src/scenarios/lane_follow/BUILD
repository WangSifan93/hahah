load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lane_follow",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common",
        "//apps/planning/src/common:base",
        "//apps/planning/src/common/util",
        "//apps/planning/src/common/vehicle_state",
        "//apps/planning/src/planning_msgs",
        "//apps/planning/src/scenarios:scenario",
        "//apps/planning/src/scenarios:stage",
        "//apps/planning/src/tasks:task",
        "@common_basic_msgs//:basic_msgs",
        "@common_status//:status",
        "@common_utils//:util",
    ],
)
