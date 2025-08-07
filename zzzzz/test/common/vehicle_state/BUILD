load("@rules_cc//cc:defs.bzl", "cc_test", "cc_binary")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "vehicle_state_provider_test", 
    srcs = ["vehicle_state_provider_test.cc"],
    deps = [
        "//third_party:googletest",
        "//third_party:protobuf",
        "//apps/planning/src/common/vehicle_state:vehicle_state",
    ],
    linkopts = ["-lpthread","-ldl"], 
)
