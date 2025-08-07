load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_type", 
    hdrs = ["data_type.h"],
    includes = [
        ".",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

cc_library(
    name = "lateral", 
    srcs =  glob(["*.cc"]),
    hdrs =  glob(["*.h"]),
    includes = [
        ".",
    ],
    deps = [
        "//third_party:googletest",
        "//third_party:eigen3",
        "//apps/planning/src/config:config",
        "//apps/planning/src/common:common",
        "@common_math//:math",
        "//apps/planning/src/planning_msgs:planning_msgs"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

cc_library(
    name = "lateral_constraint",
    srcs = ["lateral_constraint.cc"],
    hdrs = ["lateral_constraint.h"],
    deps = [
        "//apps/planning/src/common:common",
        "//third_party:eigen3",
        "//third_party:googletest"
    ],
)

cc_library(
    name = "lateral_padding",
    srcs = ["lateral_padding.cc"],
    hdrs = ["lateral_padding.h"],
    deps = [
        "//apps/planning/src/common:common",
        "//third_party:googletest"
    ],
)

cc_library(
    name = "lateral_lookup_tables",
    srcs = ["lateral_lookup_tables.cc"],
    hdrs = ["lateral_lookup_tables.h"],
    deps = [
        "//apps/planning/src/common:common",
        "//apps/planning/src/planning_msgs:planning_msgs",
        "//third_party:googletest"
    ],
)

cc_library(
    name = "lateral_tube",
    srcs = ["lateral_tube.cc"],
    hdrs = ["lateral_tube.h"],
    deps = [
        "//apps/planning/src/common:common",
        "//third_party:eigen3",
        "//third_party:googletest"
    ],
)

cc_library(
    name = "lateral_stop_hold",
    srcs = ["lateral_stop_hold.cc"],
    hdrs = ["lateral_stop_hold.h"],
    deps = [
        "//apps/planning/src/common:common",
        "//third_party:eigen3",
        "//third_party:googletest"
    ],
)

cc_library(
    name = "lateral_mpc_data_mocker",
    srcs = ["lateral_mpc_data_mocker.cc"],
    hdrs = ["lateral_mpc_data_mocker.h"],
    deps = [
        "//apps/planning/src/common:common",
        "//third_party:eigen3",
        "//third_party:googletest"
    ],
)