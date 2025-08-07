load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "speed",  
    hdrs = glob([
        "*.h",     
    ]),
    srcs = glob([
        "*.cc",
    ]),

    deps = [
        "//third_party:googletest",
        "@common_basic_msgs//:basic_msgs",
        "//apps/planning/src/common:base",
        "//apps/planning/src/config:config",
        "//apps/planning/src/planning_msgs:planning_msgs",
        "@common_math//:math",
        "//apps/planning/src/common/util:util"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
