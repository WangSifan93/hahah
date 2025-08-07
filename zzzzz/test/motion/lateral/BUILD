load("@rules_cc//cc:defs.bzl", "cc_test")

package(default_visibility = ["//visibility:public"])

cc_test(
    name = "lateral_constraint_test", 
    srcs = ["lateral_constraint_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)

cc_test(
    name = "lateral_padding_test", 
    srcs = ["lateral_padding_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)

cc_test(
    name = "lateral_reference_test", 
    srcs = ["lateral_reference_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)

cc_test(
    name = "lateral_tube_test", 
    srcs = ["lateral_tube_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)

cc_test(
    name = "lateral_nudger_test", 
    srcs = ["lateral_nudger_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)

cc_test(
    name = "lateral_stop_hold_test", 
    srcs = ["lateral_stop_hold_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)

cc_test(
    name = "lateral_mpc_data_mocker_test", 
    srcs = ["lateral_mpc_data_mocker_test.cc"],
    deps = [
        "//apps/planning/src:planning_lib",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:nlohmann_json"
    ],
    linkopts = ["-lpthread","-ldl"], 
)