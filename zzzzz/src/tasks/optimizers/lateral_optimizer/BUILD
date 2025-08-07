load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])


cc_library(
    name = "lateral_optimizer",
    hdrs = [
        "lateral_optimizer.h"
    ],
    srcs = [
        "lateral_optimizer.cc"
    ],
    deps = [
        "//apps/planning/src/motion/lateral:lateral",
        "//apps/planning/src/motion/lateral:data_type",
        "//apps/planning/src/motion/longitudinal:longitudinal",
        "//apps/planning/src/common:common",
        "//apps/planning/src/tasks:task",
        "@common_math//:math",
        "@common_utils//:util",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
