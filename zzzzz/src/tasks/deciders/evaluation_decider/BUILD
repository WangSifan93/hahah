load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "evaluation_decider",
    hdrs = ["evaluation_decider.h"],
    srcs = ["evaluation_decider.cc"],
    deps = [
      "//apps/planning/src/evaluation",
      "//apps/planning/src/planning_msgs",
      "//apps/planning/src/tasks/deciders:decider"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
