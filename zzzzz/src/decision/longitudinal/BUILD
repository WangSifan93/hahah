load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_type", 
    hdrs = ["data_type.h"],
    includes = [
        ".",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)



cc_library(
    name = "longitudinal", 
    srcs =  glob(["*.cc"]),
    hdrs =  glob(["*.h"]),
    includes = [
        ".",
    ],
    deps = [
        "@common_math//:math",
        "//apps/planning/src/common/configs:configs",
        "@common_status//:status",
        "//apps/planning/src/planning_msgs:planning_msgs",
        "//apps/planning/src/common:common",
        "//apps/planning/src/common/speed:speed",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

