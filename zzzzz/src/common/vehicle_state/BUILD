load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "vehicle_state",
    srcs = glob(["*.cc"]),
    hdrs = glob([
        "*.h",
        "proto/*.h",
    ]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//apps/planning/src/common:base",
        "//apps/planning/src/common/configs",
        "//apps/planning/src/config",
        "//cheryos/zos/zlog",
        "//messages/canbus:chassis_cc_proto",
        "//messages/localization:localization_cc_proto",
        "//messages/planning:ads_adptrin_cc_proto",
        "//messages/sensors:vehicle_upstream_cc_proto",
        "//third_party:eigen3",
        "//third_party:fmt",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:math",
        "@common_status//:status",
    ],
)
