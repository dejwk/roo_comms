#pragma once

#include <memory>

#include "roo_comms/hub/hub_device.h"

namespace roo_comms {

/// Factory for hub device adapters.
class HubDeviceFactory {
 public:
  virtual ~HubDeviceFactory() = default;

  /// Returns true if the device descriptor is supported.
  virtual bool isDeviceSupported(
      const roo_comms_DeviceDescriptor& descriptor) const = 0;

  /// Creates a hub device adapter for the descriptor.
  virtual std::unique_ptr<HubDevice> createDevice(
      EspNowTransport& transport, const roo_io::MacAddress& destination,
      const roo_comms_DeviceDescriptor& descriptor) const = 0;
};

}  // namespace roo_comms