load("@rules_cc//cc:cc_library.bzl", "cc_library")

cc_library(
    name = "roo_comms",
    srcs = glob(
        [
            "src/**/*.cpp",
            "src/**/*.c",
            "src/**/*.h",
        ],
        exclude = ["test/**"],
    ),
    includes = [
        "src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@nanopb",
        "@roo_blink",
        "@roo_collections",
        "@roo_control",
        "@roo_io",
        "@roo_logging",
        "@roo_scheduler",
        "@roo_testing//:arduino",
        "@roo_testing//roo_testing/frameworks/arduino-esp32-2.0.4/libraries/WiFi",
        "@roo_threads",
        "@roo_time",
        "@roo_transceivers",
    ],
)
