#include "roo_comms/hub/devices/home_automation_device_factory.h"

#include <memory>

#include "roo_comms/home_automation.h"
#include "roo_comms/hub/devices/hub_device_environmental_sensor.h"
#include "roo_comms/hub/devices/hub_device_relay.h"

namespace roo_comms {

bool HomeAutomationDeviceFactory::isDeviceSupported(
    const roo::comms::DeviceDescriptor& descriptor) const {
  return (descriptor.realm_id() ==
          static_cast<int64_t>(roo::comms::RealmId::kHomeAutomation));
}

std::unique_ptr<HubDevice> HomeAutomationDeviceFactory::createDevice(
    EspNowTransport& transport, const roo_io::MacAddress& destination,
    const roo::comms::DeviceDescriptor& descriptor) const {
  if (descriptor.realm_id() !=
      static_cast<int64_t>(roo::comms::RealmId::kHomeAutomation)) {
    return nullptr;
  }
  roo::comms::HomeAutomationDeviceDescriptor home_automation_descriptor;
  if (!TryParseHomeAutomationDescriptor(descriptor,
                                        home_automation_descriptor)) {
    return nullptr;
  }
  switch (home_automation_descriptor.kind_case()) {
    case roo::comms::HomeAutomationDeviceDescriptor::KindCase::
        kEnvironmentalSensor: {
      return std::unique_ptr<HubDevice>(new HubDeviceEnvironmentalSensor(
          transport, destination,
          home_automation_descriptor.environmental_sensor()));
    }
    case roo::comms::HomeAutomationDeviceDescriptor::KindCase::kRelay: {
      return std::unique_ptr<HubDevice>(
          new HubDeviceRelay(transport, destination,
                             home_automation_descriptor.relay().port_count()));
    }
    default: {
      return nullptr;
    }
  }
}

}  // namespace roo_comms