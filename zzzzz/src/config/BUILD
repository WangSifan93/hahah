load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "config",
    srcs = glob([
        "*.cpp",
        "*.cc",
    ]),
    hdrs = glob([
        "*.h",
        "*hpp",
    ]),
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
        "//apps/planning/src/common:base",
        "//apps/planning/src/planning_msgs",
        "//messages/localization:localization_cc_proto",
        "//messages/mapfusion:all_map_cc_proto",
        "//messages/mapfusion:routing_cc_proto",
        "//messages/perception:obstacle_cc_proto",
        "//third_party:nlohmann_json",
        "@common_basic_msgs//:basic_msgs",
    ],
)
