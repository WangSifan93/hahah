load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "util_test", 
    srcs = ["util_test.cc"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src/common/util:util",
    ],
    linkopts = ["-lpthread","-ldl"], 
)