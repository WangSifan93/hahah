load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "planning_msgs",
    srcs = ["sl_boundary.cc"],
    hdrs = glob(["*.h"]),
    includes = [
        ".",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/planning_msgs/math",
        "//messages/localization:localization_cc_proto",
        "//messages/mapfusion:all_map_cc_proto",
        "//messages/mapfusion:routing_cc_proto",
        "//messages/perception:obstacle_cc_proto",
        "//third_party:protobuf",
        "//third_party:tros",
        "@common_math//:math",
        "@common_basic_msgs//:basic_msgs",
    ],
)
