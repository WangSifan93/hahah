load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "safety_evaluator_test",
    srcs = [
        "safety_evaluator_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/evaluation:safety_evaluator",
    ]
)

cc_test(
    name = "comfort_evaluator_test",
    srcs = [
        "comfort_evaluator_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/evaluation:comfort_evaluator",
    ]
)

cc_test(
    name = "progress_evaluator_test",
    srcs = [
        "progress_evaluator_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/evaluation:progress_evaluator",
    ]
)

cc_test(
    name = "similarity_evaluator_test",
    srcs = [
        "similarity_evaluator_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/evaluation:similarity_evaluator",
    ]
)

cc_test(
    name = "lane_change_evaluator_test",
    srcs = [
        "lane_change_evaluator_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/evaluation:lane_change_evaluator",
    ]
)
