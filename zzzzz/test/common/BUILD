load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "lateral_optimizer_config_test",
    srcs = [
        "lateral_optimizer_config_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
    ]
)

cc_test(
    name = "lookup_table_test",
    srcs = [
        "lookup_table_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
    ]
)

cc_test(
    name = "longitudinal_optimizer_config_test",
    srcs = [
        "longitudinal_optimizer_config_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
    ]
)

cc_test(
    name = "frame_test",
    srcs = [
        "frame_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
    ]
)

cc_test(
    name = "hysteresis_test",
    srcs = [
        "hysteresis_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
    ]
)

