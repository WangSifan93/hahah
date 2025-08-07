load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "configs",  
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = [
        "//apps/planning/src/common:base",
        "//apps/planning/src/config:config",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:math"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
