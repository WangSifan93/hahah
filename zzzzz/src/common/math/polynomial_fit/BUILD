load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "polynomial_fit",
    srcs = glob(["polynomial_fit.cc"]),
    hdrs = glob(["polynomial_fit.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common/vehicle_state",
        "//apps/planning/src/common/trajectory",
        "//third_party:eigen3",
        "//third_party:googletest",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:vec2d",
    ],
)
