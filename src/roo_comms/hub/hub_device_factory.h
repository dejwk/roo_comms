#pragma once

#include <memory>

#include "roo_comms/hub/hub_device.h"

namespace roo_comms {

class HubDeviceFactory {
 public:
  virtual ~HubDeviceFactory() = default;

  virtual std::unique_ptr<HubDevice> createDevice(
      EspNowTransport& transport, const roo_io::MacAddress& destination,
      const roo_comms_DeviceDescriptor& descriptor) const = 0;
};

}  // namespace roo_comms