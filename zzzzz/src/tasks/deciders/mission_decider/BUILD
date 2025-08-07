load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "mission_decider",
    hdrs = ["mission_decider.h"],
    srcs = ["mission_decider.cc"],
    deps = [
      "//apps/planning/src/tasks/deciders:decider",
      "//apps/planning/src/common:common",
      "//apps/planning/src/mission:env_info_builder",
      "//apps/planning/src/mission:lc_request",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
