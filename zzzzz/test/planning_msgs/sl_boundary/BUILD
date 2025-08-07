load("@rules_cc//cc:defs.bzl", "cc_test")
package(default_visibility = ["//visibility:public"])

cc_test(
  name = "sl_boundary_test",
  srcs = ["sl_boundary_test.cc"],
  deps = [
    "//third_party:googletest",
    "//apps/planning/src/planning_msgs",
    "//third_party:fmt",
  ],
  linkopts = ["-lpthread","-ldl"],
)
