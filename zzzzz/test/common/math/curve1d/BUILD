load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "quartic_polynomial_curve1d_test",
    srcs = ["quartic_polynomial_curve1d_test.cc"],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    deps = [
        "//apps/planning/src/common/math:curve1d",
        "//third_party:googletest",
    ],
)

cc_test(
    name = "quintic_polynomial_curve1d_test",
    srcs = ["quintic_polynomial_curve1d_test.cc"],
    linkopts = [
        "-lpthread",
        "-ldl",
    ],
    deps = [
        "//apps/planning/src/common/math:curve1d",
        "//third_party:googletest",
    ],
)
