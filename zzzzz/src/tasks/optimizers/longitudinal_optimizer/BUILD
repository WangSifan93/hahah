load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])


cc_library(
    name = "longitudinal_optimizer",
    hdrs = [
        "longitudinal_optimizer.h"
    ],
    srcs = [
        "longitudinal_optimizer.cc"
    ],
    deps = [
        "//apps/planning/src/motion/longitudinal:longitudinal",
        "//apps/planning/src/common:common",
        "//apps/planning/src/tasks:task",
        "@common_math//:math",
        "@common_utils//:util",
        "//apps/planning/src/common/math:mpc"
    ],
)
