load("@rules_cc//cc:defs.bzl", "cc_test")
package(default_visibility = ["//visibility:public"])

cc_test(
  name = "env_info_builder_test",
  srcs = ["env_info_builder_test.cc"],
  linkopts = ["-lpthread","-ldl"],
  deps = [
    "//third_party:googletest",
    "//apps/planning/src/mission:env_info_builder",
    "//apps/planning/src:planning_lib",
  ],
)

cc_test(
  name = "lc_request_test",
  srcs = ["lc_request_test.cc"],
  linkopts = ["-lpthread","-ldl"],
  deps = [
    "//third_party:googletest",
    "//apps/planning/src/mission:lc_request",
    "//apps/planning/src:planning_lib",
  ],
)
