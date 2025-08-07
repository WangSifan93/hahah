load("@rules_cc//cc:defs.bzl", "cc_test")
package(default_visibility = ["//visibility:public"])

cc_test(
  name = "frenet_trajectory_sampler_test",
  srcs = ["frenet_trajectory_sampler_test.cc"],
  linkopts = ["-lpthread","-ldl"],
  deps = [
    "//third_party:googletest",
    "//apps/planning/src/decision/lateral:lateral",
    "//apps/planning/src:planning_lib",
  ],
)

cc_test(
  name = "frenet_trajectory_map_builder_test",
  srcs = ["frenet_trajectory_map_builder_test.cc"],
  linkopts = ["-lpthread","-ldl"],
  deps = [
    "//third_party:googletest",
    "//apps/planning/src/decision/lateral:lateral",
    "//apps/planning/src:planning_lib",
  ],
)
