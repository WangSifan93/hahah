load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "proto_convertor_test",
    srcs = [
        "proto_convertor_test.cc"
    ],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
    ]
)
