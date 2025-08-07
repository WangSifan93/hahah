load("@rules_cc//cc:defs.bzl", "cc_binary")
load("@rules_pkg//pkg:pkg.bzl", "pkg_tar", "pkg_deb")

package(default_visibility = ["//visibility:public"])


pkg_tar(
    name = "planning_libs",
    include_runfiles=True,
    srcs = [
        "//apps/planning/src:libplanning.so",
    ],
    mode = "0644",
    package_dir = "/zark/lib",
    stamp = 1,
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

pkg_tar(
    name = "planning_config",
    srcs = [
        ":config",
    ],
    mode = "0755",
    package_dir = "/zark/apps/planning/",
    stamp = 1,
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)

pkg_tar(
    name = "planning_zark_tarball",
    extension = "tar.gz",
    deps = [
        "//:zark_sh_script",
        "//:zark_version",
        "//scripts:scripts_files",
        "//cheryos/zos/zos_launch:zoslaunch",
        ":planning_libs",
        ":planning_config",
    ],
    target_compatible_with = select({
        "@platforms//cpu:x86_64": [],
        "@platforms//cpu:arm64e": [],
        "@platforms//cpu:arm64_32": [],
        "//conditions:default": ["@platforms//:incompatible"],
    }),
)
