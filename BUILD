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
#        "//lib/roo_collections",
#        "//lib/roo_logging",
#        "//lib/roo_prefs",
#        "//roo_testing:arduino",
        "//lib/roo_control",
        "//lib/roo_led",
        "//lib/roo_scheduler",
        "//lib/roo_io",
        "//lib/roo_transceivers",
        "@nanopb//:nanopb",
        "//roo_testing/frameworks/arduino-esp32-2.0.4/cores/esp32",
    ],
)
