load("@roo_testing//roo_testing/emulation:arduino.bzl", "roo_arduino_example")

roo_arduino_example(
    name = "relay_device",
    sketch = "relay_device.ino",
    visibility = ["//visibility:public"],
    deps = [
        "//:roo_comms",
        "@roo_testing//roo_testing/buses/esp_now",
        "@roo_testing//roo_testing/microcontrollers/esp32",
    ],
)
