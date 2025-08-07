load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "evaluation_decider_test",
    srcs = [
        "evaluation_decider_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
    ]
)
