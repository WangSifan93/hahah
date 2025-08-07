load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")
load("@rules_pkg//pkg:pkg.bzl", "pkg_tar", "pkg_deb")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "lateral_optimizer_test",
    srcs = [
        "lateral_optimizer_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/tasks/optimizers/lateral_optimizer:lateral_optimizer"
    ]
)
