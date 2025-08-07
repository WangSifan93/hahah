load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "mpc_example_test", 
    srcs = ["mpc_example_test.cc"],
    deps = [
        "//third_party:osqp",
        "//third_party:osqp-eigen",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)
