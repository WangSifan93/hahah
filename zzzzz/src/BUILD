load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libplanning.so",
    linkshared = True,
    linkstatic = False,
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [":planning_lib"],
)

cc_library(
    name = "planning_lib",
    srcs = [
        "on_lane_planning.cc",
        "planning_base.cc",
        "planning_component.cc",
    ],
    hdrs = [
        "on_lane_planning.h",
        "planning_base.h",
        "planning_component.h",
    ],
    deps = [
        "//apps/planning/src/common",
        "//apps/planning/src/common:base",
        "//apps/planning/src/common/configs",
        "//apps/planning/src/common/local_route",
        "//apps/planning/src/reference_line_provider:local_route_provider",
        "//apps/planning/src/common/math/polynomial_fit",
        "//apps/planning/src/common/proto_convertor",
        "//apps/planning/src/common/trajectory",
        "//apps/planning/src/common/util",
        "//apps/planning/src/common/vehicle_state",
        "//apps/planning/src/config",
        "//apps/planning/src/planner",
        "//apps/planning/src/planning_msgs",
        "//apps/planning/src/tasks:task_factory",
        "//messages/common:heartbeat_cc_proto",
        "//messages/localization:localization_cc_proto",
        "//messages/map_service:all_map_new_cc_proto",
        "//messages/mapfusion:map_fusion_cc_proto",
        "//messages/mapfusion:routing_cc_proto",
        "//messages/planning:ads_com_cc_proto",
        "//messages/planning:path_and_reference_line_cc_proto",
        "//messages/planning:planning_cc_proto",
        "//messages/planning:planning_debug_cc_proto",
        "//messages/sensors:vehicle_upstream_cc_proto",
        "//third_party:fmt",
        "//third_party:googletest",
        "//third_party:tros",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:math",
        "@common_utils//:util",
    ],
)
