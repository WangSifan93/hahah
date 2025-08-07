load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "lateral_decider_test", 
    srcs = ["lateral_decider_test.cc"],
    linkopts = ["-lpthread","-ldl"], 
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/decision/longitudinal:longitudinal",
        "//apps/planning/src/tasks/deciders/lateral_decider:lateral_decider",
    ],
)

