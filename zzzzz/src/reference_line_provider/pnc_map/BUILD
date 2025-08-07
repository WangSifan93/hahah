load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "pnc_map",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/map_service/map_info:map_service_map_info_lib",
        "//apps/planning/src/common:base",
        "//apps/planning/src/common/configs",
        "//apps/planning/src/common/vehicle_state",
        "//apps/planning/src/config",
        "//apps/planning/src/common/data_reader",
        "//apps/planning/src/planning_msgs",
        "//messages/map_service:all_map_new_cc_proto",
        "//messages/mapfusion:map_fusion_cc_proto",
        "//messages/mapfusion:routing_cc_proto",
        "//messages/planning:reference_line_cc_proto",
        "//messages/planning:ads_adptrin_cc_proto",
        "//messages/planning:ads_fct_cc_proto",
        "//third_party:googletest",
        "//third_party:protobuf",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:math",
        "@common_utils//:util",
    ],
)
