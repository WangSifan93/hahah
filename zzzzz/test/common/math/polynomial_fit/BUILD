load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "polynomial_fit_test",
    srcs = ["polynomial_fit_test.cc"],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    deps = [
        "//apps/planning/src/common/local_route",
        "//apps/planning/src/common/util",
        "//apps/planning/src/common/math/polynomial_fit",
        "//third_party:googletest",
    ],
)
