load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "proto_convertor",  
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = [
      "@common_basic_msgs//:basic_msgs",
      "//apps/planning/src/planning_msgs:planning_msgs",
      "//apps/planning/src/common:base",
      "//apps/planning/src/common:common",
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
