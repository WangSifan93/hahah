load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_pkg//pkg:pkg.bzl", "pkg_tar", "pkg_deb")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "longitudinal_blocker_test",
    srcs = [
        "longitudinal_blocker_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/motion/longitudinal"
    ]
)

cc_test(
    name = "longitudinal_reference_test",
    srcs = [
        "longitudinal_reference_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/motion/longitudinal"
    ]
)


cc_test(
    name = "longitudinal_constraint_test",
    srcs = [
        "longitudinal_constraint_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/motion/longitudinal"
    ]
)


cc_test(
    name = "longitudinal_padding_test",
    srcs = [
        "longitudinal_padding_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/motion/longitudinal"
    ]
)

cc_test(
    name = "longitudinal_stop_hold_test",
    srcs = [
        "longitudinal_stop_hold_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/motion/longitudinal"
    ]
)

