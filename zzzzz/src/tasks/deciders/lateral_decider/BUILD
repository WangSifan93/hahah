load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])


cc_library(
    name = "lateral_decider",
    hdrs = ["lateral_decider.h"],
    srcs = ["lateral_decider.cc"],
    deps = [
      "//apps/planning/src/tasks/deciders:decider",
      "//apps/planning/src/decision/lateral:lateral",
      "//apps/planning/src/common:common",
      "//apps/planning/src/decision/longitudinal:longitudinal"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

