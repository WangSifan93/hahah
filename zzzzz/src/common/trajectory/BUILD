load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "discretized_trajectory",
    hdrs = glob(["discretized_trajectory.h"]),
    srcs = glob(["discretized_trajectory.cc"]),
    deps = [
        "//apps/planning/src/common:base",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:vec2d",
        "@common_math//:linear_interpolation",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

cc_library(
    name = "trajectory",
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = [
        "//apps/planning/src/common:base",
        "//apps/planning/src/planning_msgs:planning_msgs",
        "@common_math//:math",
        "//apps/planning/src/common:common",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
