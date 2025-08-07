load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "evaluation",
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/evaluation:comfort_evaluator",
        "//apps/planning/src/evaluation:lane_change_evaluator",
        "//apps/planning/src/evaluation:progress_evaluator",
        "//apps/planning/src/evaluation:safety_evaluator",
        "//apps/planning/src/evaluation:similarity_evaluator",
    ],
)

cc_library(
    name = "const_values",
    hdrs = ["const_values.h"],
)

cc_library(
    name = "safety_evaluator",
    srcs = ["safety_evaluator.cc"],
    hdrs = ["safety_evaluator.h"],
    deps = [
        ":const_values",
        "//apps/planning/src/common",
        "//apps/planning/src/common/configs",
    ],
)

cc_library(
    name = "comfort_evaluator",
    srcs = ["comfort_evaluator.cc"],
    hdrs = ["comfort_evaluator.h"],
    deps = [
        ":const_values",
        "//apps/planning/src/common",
        "//apps/planning/src/common/configs",
    ],
)

cc_library(
    name = "progress_evaluator",
    srcs = ["progress_evaluator.cc"],
    hdrs = ["progress_evaluator.h"],
    deps = [
        ":const_values",
        "//apps/planning/src/common",
        "//apps/planning/src/common/configs",
    ],
)

cc_library(
    name = "similarity_evaluator",
    srcs = ["similarity_evaluator.cc"],
    hdrs = ["similarity_evaluator.h"],
    deps = [
        ":const_values",
        "//apps/planning/src/common",
        "//apps/planning/src/common/configs",
    ],
)

cc_library(
    name = "lane_change_evaluator",
    srcs = ["lane_change_evaluator.cc"],
    hdrs = ["lane_change_evaluator.h"],
    deps = [
        ":const_values",
        "//apps/planning/src/common",
        "//apps/planning/src/common/configs",
    ],
)
