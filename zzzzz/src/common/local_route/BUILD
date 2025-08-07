load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "local_route",  
    hdrs = glob(["*.h"]),
    srcs = glob(["*.cc"]),
    deps = [
      "@common_basic_msgs//:basic_msgs",
      "//apps/planning/src/planning_msgs:planning_msgs",
      "//apps/planning/src/config:config",
      "//apps/planning/src/common:base",
      "//apps/planning/src/common:common",
      "//apps/planning/src/common/configs:configs",
      "@common_math//:math",
      "@common_utils//:util",
      "//apps/planning/src/reference_line_provider/pnc_map:pnc_map"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
