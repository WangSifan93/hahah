load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])


cc_library(
    name = "longitudinal_decider",
    hdrs = ["longitudinal_decider.h"],
    srcs = ["longitudinal_decider.cc"],
    deps = [
      "//apps/planning/src/tasks/deciders:decider",
      "//apps/planning/src/tasks:task",
      "//apps/planning/src/decision/longitudinal:longitudinal"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

