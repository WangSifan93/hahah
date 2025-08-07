load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_binary")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "e2e_proto",
    srcs = glob(["*.proto"]),
    deps = ["@com_google_protobuf//:timestamp_proto"],
)

cc_proto_library(
    name = "e2e_cc_proto",
    deps = [":e2e_proto",],
)
