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

## Protocol buffers

Messages use `roo_pb` 0.1.0 and require C++17 and `roo_io` 2.4.0 or newer.
Hub adapters use the `roo_pb` descriptor API from `roo_transceivers` 1.3.0.
The generated `src/comms.pb.h` is checked in for Arduino and PlatformIO builds.
Regenerate it from any working directory with:

```sh
ROO_PB_ROOT=/path/to/roo_pb bash proto/generate.sh
```

The default generator path is a sibling `roo_pb` checkout. Generation requires
Python 3.11+ and clang-format. Field storage limits live in
`proto/comms.roo_pb.toml`; device descriptor payloads remain bounded to 128 bytes.

This migration changes the C++ message API. Types now live in `roo::comms`
(for example, `roo::comms::DataMessage`). Use getters, setters, and mutable
submessages instead of nanopb fields and union discriminators:

```cpp
roo::comms::DataMessage message;
message.mutable_relay_request()->set_mask(1);
message.mutable_relay_request()->set_write(1);
auto packet = roo_comms::SerializeHomeAutomationDataMessage(message);
```

Read oneof selection with `contents_case()` and the generated `ContentsCase`
enum. Optional scalar presence uses methods such as `has_temperature_celsius()`.
The protobuf field numbers, ESP-NOW prefixes, and persisted descriptor encoding
are unchanged, so existing devices and stored pairings remain wire compatible.

Run the protocol regression tests and compile the examples against a local
`roo_pb` checkout with:

```sh
bazel test --override_module=roo_pb=/path/to/roo_pb \
  --override_module=roo_transceivers=/path/to/roo_transceivers //...
```
