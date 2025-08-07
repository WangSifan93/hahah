load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])


cc_library(
    name = "task",
    hdrs = ["task.h"],
    srcs = ["task.cc"],
    deps = [
        "//apps/planning/src/common:common",
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
    name = "task_factory",
    hdrs = ["task_factory.h"],
    srcs = ["task_factory.cc"],
    deps = [
        "@common_utils//:util",
        "//apps/planning/src/planning_msgs:planning_msgs",
        "//apps/planning/src/tasks:task",
        "@common_status//:status",
        "//apps/planning/src/tasks/deciders/traffic_rule_decider:traffic_rule_decider",
        "//apps/planning/src/tasks/deciders/mission_decider:mission_decider",
        "//apps/planning/src/tasks/deciders/lateral_decider:lateral_decider",
        "//apps/planning/src/tasks/deciders/longitudinal_decider:longitudinal_decider",
        "//apps/planning/src/tasks/optimizers/longitudinal_optimizer:longitudinal_optimizer",
        "//apps/planning/src/tasks/optimizers/lateral_optimizer:lateral_optimizer",
        "//apps/planning/src/tasks/deciders/evaluation_decider:evaluation_decider"
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
