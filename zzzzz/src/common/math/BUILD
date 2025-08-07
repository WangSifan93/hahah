load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "math",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common:base",
        "//cheryos/zos/zlog",
        "//third_party:fmt",
    ],
)

cc_library(
    name = "curve1d",
    srcs = glob(["curve1d/*.cc"]),
    hdrs = glob(["curve1d/*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common:base",
        "@common_math//:math",
        "@common_utils//:util",
    ],
)

cc_library(
    name = "discretized_points_smoothing",
    srcs = glob(["discretized_points_smoothing/*.cc"]),
    hdrs = glob(["discretized_points_smoothing/*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common:base",
        "//apps/planning/src/planning_msgs",
        "//cheryos/zos/zlog",
        "//third_party:fmt",
        "//third_party:osqp",
    ],
)

cc_library(
    name = "mpc",
    srcs = glob(["mpc/*.cc"]),
    hdrs = glob(["mpc/*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common",
        "//apps/planning/src/common:base",
        "//apps/planning/src/config",
        "//third_party:eigen3",
        "//third_party:osqp",
        "//third_party:osqp-eigen",
        "@common_math//:linear_quadratic_regulator",
    ],
)
