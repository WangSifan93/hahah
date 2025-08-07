load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "on_lane_planning_test",
    srcs = ["on_lane_planning_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/reference_line_provider:local_route_provider",
        "//third_party:googletest",
    ],
    linkopts = ["-lpthread","-ldl"],
)
