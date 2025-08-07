load("@rules_cc//cc:defs.bzl", "cc_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "env_info_builder",
    hdrs = ["env_info_builder.h"],
    srcs = ["env_info_builder.cc"],
    deps = [
        "//apps/planning/src/common:common",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

cc_library(
    name = "lc_request",
    hdrs = ["lc_request.h"],
    srcs = ["lc_request.cc"],
    deps = [
        "//apps/planning/src/common:common",
        "//messages/planning:ads_decision_cc_proto",
        "//messages/state_management:state_management_cc_proto",
        ":force_lane_change_decision",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

cc_library(
    name = "force_lane_change_decision",
    hdrs = ["force_lane_change_decision.h"],
    srcs = ["force_lane_change_decision.cc"],
    deps = [
        "//apps/planning/src/common:common",
        "//messages/planning:ads_decision_cc_proto",
        "//messages/map_service:all_map_new_cc_proto",
        "//messages/state_management:state_management_cc_proto",
        "//apps/noa_decision/src/common/data_reader",
        "//apps/noa_decision/src/config",
        "//messages/planning:decision_measure_cc_proto",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:math",
        "@common_utils//:util",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
