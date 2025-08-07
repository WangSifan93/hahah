load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "mpc_problem_test",
    srcs = [
        "mpc_problem_test.cc",
    ],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    deps = [
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/common/math:mpc",
        "//third_party:googletest",
    ],
)

cc_test(
    name = "longitudinal_mpc_problem_test",
    srcs = [
        "longitudinal_mpc_problem_test.cc",
    ],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    deps = [
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/common/math:mpc",
        "//third_party:googletest",
    ],
)

cc_test(
    name = "lateral_mpc_problem_test",
    srcs = ["lateral_mpc_problem_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/common/math:mpc",
        "//third_party:eigen3",
        "//third_party:googletest",
    ],
    linkopts = ["-lpthread","-ldl"],
)

cc_test(
    name = "utility_test",
    srcs = [
        "utility_test.cc",
    ],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    deps = [
        "//apps/planning/src:planning_lib",
    ]
)
