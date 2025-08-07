load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "map_polynomial_fit",
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
        "//apps/planning/src/common",
        "//apps/planning/src/common/math",
        "//apps/planning/src/common/local_route",
        "//apps/planning/src/config",
        "//messages/map_service:all_map_new_cc_proto",
        "//messages/mapfusion:map_fusion_cc_proto",
        "//third_party:eigen3",
        "//third_party:googletest",
        "//third_party:protobuf",
        "@common_basic_msgs//:basic_msgs",
        "@common_math//:math",
        "@common_utils//:util",
    ],
)
