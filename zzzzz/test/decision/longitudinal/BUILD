load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "st_topology_builder_test", 
    srcs = ["st_topology_builder_test.cc"],
    linkopts = ["-lpthread", "-ldl"],
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/decision/longitudinal:longitudinal",
    ],
)
cc_test(
    name = "st_proposal_builder_test", 
    srcs = ["st_proposal_builder_test.cc"],
    linkopts = ["-lpthread","-ldl"], 
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/decision/longitudinal:longitudinal",
    ],
)
cc_test(
    name = "st_graph_builder_test", 
    srcs = ["st_graph_builder_test.cc"],
    linkopts = ["-lpthread","-ldl"], 
    deps = [
        "//third_party:googletest",
        "//apps/planning/src:planning_lib",
        "//apps/planning/src/decision/longitudinal:longitudinal",
        "//apps/planning/src/decision/lateral:lateral",
        "//apps/planning/src/planning_msgs:planning_msgs",
    ],
)
