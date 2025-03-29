#include "roo_comms/hub/devices/home_automation_device_factory.h"

#include <memory>

#include "roo_comms/home_automation.h"
#include "roo_comms/hub/devices/hub_device_environmental_sensor.h"
#include "roo_comms/hub/devices/hub_device_relay.h"

namespace roo_comms {

std::unique_ptr<HubDevice> HomeAutomationDeviceFactory::createDevice(
    EspNowTransport& transport, const roo_io::MacAddress& destination,
    const roo_comms_DeviceDescriptor& descriptor) const {
  if (descriptor.realm_id != roo_comms_RealmId_kHomeAutomation) {
    return nullptr;
  }
  roo_comms_HomeAutomationDeviceDescriptor home_automation_descriptor;
  if (!TryParseHomeAutomationDescriptor(descriptor,
                                        home_automation_descriptor)) {
    return nullptr;
  }
  switch (home_automation_descriptor.which_kind) {
    case roo_comms_HomeAutomationDeviceDescriptor_environmental_sensor_tag: {
      return std::unique_ptr<HubDevice>(new HubDeviceEnvironmentalSensor(
          transport, destination,
          home_automation_descriptor.kind.environmental_sensor));
    }
    case roo_comms_HomeAutomationDeviceDescriptor_relay_tag: {
      return std::unique_ptr<HubDevice>(
          new HubDeviceRelay(transport, destination,
                             home_automation_descriptor.kind.relay.port_count));
    }
    default: {
      return nullptr;
    }
  }
}

}  // namespace roo_comms