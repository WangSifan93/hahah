load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "data_reader",
    srcs = glob(["*.cc"]),
    hdrs = glob(["*.h"]),
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
    deps = [
        "//messages/localization:localization_cc_proto",
        "//messages/map_service:all_map_new_cc_proto",
        "//messages/mapfusion:map_fusion_cc_proto",
        "//messages/planning:decision_measure_cc_proto",
        "//messages/planning:ads_decision_cc_proto",
        "//messages/planning:reference_line_cc_proto",
        "//apps/planning/src/config",
    ],
)

