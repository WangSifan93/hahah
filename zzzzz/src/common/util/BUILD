load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "util",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common:base",
        "//apps/planning/src/config",
        "//apps/planning/src/decision/longitudinal:data_type",
        "//apps/planning/src/motion/lateral:data_type",
        "//apps/planning/src/planning_msgs",
        "//apps/planning/src/reference_line_provider/pnc_map",
        "//cheryos/zos/zlog",
        "//messages/canbus:chassis_cc_proto",
        "//messages/common:heartbeat_cc_proto",
        "//messages/localization:localization_cc_proto",
        "//messages/mapfusion:all_map_cc_proto",
        "//messages/mapfusion:map_fusion_cc_proto",
        "//messages/mapfusion:routing_cc_proto",
        "//messages/perception:obstacle_cc_proto",
        "//messages/planning:planning_cc_proto",
        "//messages/planning:reference_line_cc_proto",
        "//messages/prediction:prediction_cc_proto",
        "//third_party:boost",
        "//third_party:eigen3",
        "//third_party:fmt",
        "//third_party:googletest",
        "//third_party:nlohmann_json",
        "//third_party:protobuf",
        "//third_party:tros",
        "@common_basic_msgs//:basic_msgs",
    ],
)
