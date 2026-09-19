# roo_comms 1.1.6

- Upgrade Bazel and PlatformIO dependencies, including roo_time 2.0.0, roo_io 2.3.0, roo_scheduler 2.2.0, roo_transceivers 1.2.0, and roo_blink 1.1.0.
- Update Bazel tooling and test dependencies: rules_cc 0.2.25, GoogleTest 1.18.0.bcr.1, and nanopb 0.4.9.1.bcr.3.
- Update shared CI workflows to roo_testing 2.1.2.
- Add consolidated release notes for previous releases.

---

# [roo_comms 1.1.5](https://github.com/dejwk/roo_comms/releases/tag/1.1.5)

Published 2026-08-30.

### Highlights

- Added runnable ESP-NOW relay examples:
  - `relay_controller`
  - `relay_device`
- Examples run on the host emulator with Bazel and include guidance for deploying to physical ESP32 hardware.

### Build and CI

- Adopted `roo_testing` 2.0 host profiles for Arduino ESP32 emulation.
- Modernized GitHub Actions CI with pull-request and manual-dispatch triggers.
- Centralized AddressSanitizer configuration.
- Updated Bazel/Bazelisk configuration and moved vendored Bazel patches under `.roo_testing`.

### Upgrade notes

This release contains no intended public API changes. It primarily improves examples, host emulation, and project tooling.

**Full Changelog:** https://github.com/dejwk/roo_comms/compare/1.1.4...1.1.5

---

# [roo_comms 1.1.4](https://github.com/dejwk/roo_comms/releases/tag/1.1.4)

Published 2026-02-26.

* Fixing CI, patching nanopb and updating BUILD files, after Bazel behavior changed.
* Doxygen documentation.
* Updated dependencies. Builds without warnings now.

---

# [roo_comms 1.1.3](https://github.com/dejwk/roo_comms/releases/tag/1.1.3)

Published 2026-01-06.

Updated dependencies.

**Full Changelog**: https://github.com/dejwk/roo_comms/compare/1.1.2...1.1.3

---

# [roo_comms 1.1.2](https://github.com/dejwk/roo_comms/releases/tag/1.1.2)

Published 2025-11-12.

* Bug fixes.

**Full Changelog**: https://github.com/dejwk/roo_comms/compare/1.1.1...1.1.2

---

# [roo_comms 1.1.1](https://github.com/dejwk/roo_comms/releases/tag/1.1.1)

Published 2025-10-31.

* Bugfix: hub crashing when registering a previously known device.
* Updated deps, CI, .gitignore.

**Full Changelog**: https://github.com/dejwk/roo_comms/compare/1.0.0...1.1.1

---

# [roo_comms 1.1.0](https://github.com/dejwk/roo_comms/releases/tag/1.0.0)

Published 2025-10-19.

Initial release.

---

