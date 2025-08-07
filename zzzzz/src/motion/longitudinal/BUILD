load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])


cc_library(
    name = "longitudinal", 
    srcs =  glob(["*.cc"]),
    hdrs =  glob(["*.h"]),
    includes = [
        ".",
    ],
    deps = [
      "//apps/planning/src/common:common",
      "//apps/planning/src/planning_msgs:planning_msgs",
      "@common_math//:math",
      "//apps/planning/src/common/math:mpc"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
