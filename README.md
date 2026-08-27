# roo_comms

Communication abstractions and helpers for Roo applications.

## Examples

`relay_controller` and `relay_device` form an ESP-NOW request-response pair.
The controller reads and toggles a relay, while the device applies each request
and returns its current state. To run either side with its compile-guarded fake
peer in the host emulator:

    bazel run //examples/relay_controller
    bazel run //examples/relay_device

Both examples continue running until interrupted with Ctrl-C. For physical
hardware, flash the two sketches to separate ESP32 boards, keep their
`kWiFiChannel` values equal, and copy the MAC printed by `relay_device` into
`kRelayAddress` in `relay_controller`.

## Host emulation

Host builds use the roo_testing 2.0 Arduino ESP32 profile. With Bazelisk 1.21
or newer, a plain command defaults to that profile and prints a notice:

    bazel test ...
    bazel test ... --config=asan
    bazel test ... --config=roo_testing_arduino_esp32

The files under .roo_testing/bazelrc/esp32 are vendored from roo_testing;
follow their canonical-source headers when refreshing them.
