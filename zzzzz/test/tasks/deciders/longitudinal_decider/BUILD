load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "longitudinal_decider_test", 
    srcs = ["longitudinal_decider_test.cc"],
    linkopts = ["-lpthread","-ldl"], 
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/tasks/deciders/longitudinal_decider:longitudinal_decider",
        "//apps/planning/src/decision/lateral:lateral",
    ],
)

