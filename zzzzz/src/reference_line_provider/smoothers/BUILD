load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "local_route_smoother",
    hdrs = ["local_route_smoother.h"],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

cc_library(
    name = "discrete_points_local_route_smoother",
    srcs = ["discrete_points_local_route_smoother.cc"],
    hdrs = ["discrete_points_local_route_smoother.h"],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
      ":local_route_smoother",
      "//apps/planning/src/common/local_route",
    ],
)
