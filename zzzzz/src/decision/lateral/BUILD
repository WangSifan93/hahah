load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lateral", 
    srcs =  glob(["*.cc"]),
    hdrs =  glob(["*.h"]),
    includes = [
        ".",
    ],
    deps = [
        "@common_math//:math",
        "//apps/planning/src/common/math:math",
        "//apps/planning/src/common/math:curve1d",
        "//apps/planning/src/common:common",
        "//apps/planning/src/planning_msgs:planning_msgs",
        "//apps/planning/src/config:config"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

