load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "local_route_provider",
    srcs = [
        "local_route_provider.cc",
    ],
    hdrs = [
        "local_route_provider.h",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common:base",
        "//apps/planning/src/common/configs",
        "//apps/planning/src/common/math:discretized_points_smoothing",
        "//apps/planning/src/config",
        "//apps/planning/src/planning_msgs",
        "//apps/planning/src/reference_line_provider/pnc_map",
        "//apps/planning/src/reference_line_provider/map_polynomial_fit:map_polynomial_fit",
        "//apps/planning/src/reference_line_provider/smoothers:discrete_points_local_route_smoother",
        "//apps/planning/src/reference_line_provider/acc:acc_trajectory_builder",
        "//messages/canbus:chassis_cc_proto",
        "//messages/common:heartbeat_cc_proto",
        "//messages/localization:localization_cc_proto",
        "//messages/map_service:all_map_new_cc_proto",
        "//messages/mapfusion:all_map_cc_proto",
        "//messages/mapfusion:map_fusion_cc_proto",
        "//messages/mapfusion:routing_cc_proto",
        "//messages/perception:obstacle_cc_proto",
        "//messages/planning:ads_adptrin_cc_proto",
        "//messages/planning:planning_cc_proto",
        "//messages/planning:planning_debug_cc_proto",
        "//messages/planning:reference_line_cc_proto",
        "//messages/prediction:prediction_cc_proto",
        "//messages/sensors:vehicle_upstream_cc_proto",
        "@common_math//:math",
        "@common_utils//:util",
    ],
)
