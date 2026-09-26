load("@rules_cc//cc:cc_library.bzl", "cc_library")
load("@rules_cc//cc:cc_test.bzl", "cc_test")

cc_library(
    name = "roo_comms",
    srcs = glob(
        [
            "src/**/*.cpp",
            "src/**/*.h",
        ],
        exclude = ["test/**"],
    ),
    includes = [
        "src",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "@roo_pb",
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

cc_test(
    name = "protocol_test",
    size = "small",
    srcs = ["test/protocol_test.cpp"],
    deps = [
        ":roo_comms",
        "@googletest//:gtest_main",
    ],
)
