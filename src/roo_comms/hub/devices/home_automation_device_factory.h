#pragma once

#include "roo_comms/hub/hub_device_factory.h"

namespace roo_comms {
/// Factory for home-automation hub devices.
class HomeAutomationDeviceFactory : public HubDeviceFactory {
 public:
  bool isDeviceSupported(
      const roo::comms::DeviceDescriptor& descriptor) const override;

  std::unique_ptr<HubDevice> createDevice(
      EspNowTransport& transport, const roo_io::MacAddress& destination,
      const roo::comms::DeviceDescriptor& descriptor) const override;
};

}  // namespace roo_comms