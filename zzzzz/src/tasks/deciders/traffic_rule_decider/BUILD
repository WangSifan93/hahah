load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])


cc_library(
    name = "traffic_rule_decider",
    hdrs = [
      "traffic_rule_decider.h",
      "rule_crosswalk.h",
      "rule_destination.h",
      "rule_traffic_light.h"
    ],
    srcs = [
      "traffic_rule_decider.cc",
      "rule_destination.cc"
    ],
    deps = [
      "//apps/planning/src/common:base",
      "//apps/planning/src/planning_msgs:planning_msgs",
      "//apps/planning/src/tasks:task"
    ],
)

