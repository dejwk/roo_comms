#ifndef ROO_COMMS_HUB_HUB_DEVICE_ENVIRONMENTAL_SENSOR_H
#define ROO_COMMS_HUB_HUB_DEVICE_ENVIRONMENTAL_SENSOR_H

#include "roo_comms/hub/hub_device.h"
#include "roo_time.h"

namespace roo_comms {

class HubDeviceEnvironmentalSensor : public HubDevice {
 public:
  HubDeviceEnvironmentalSensor(
      EspNowTransport& transport, const roo_io::MacAddress& destination,
      const roo_comms_HomeAutomationDeviceDescriptor_EnvironmentalSensor&
          descriptor);

  void getDescriptor(roo_transceivers_Descriptor& result) const override;

  roo_transceivers::Measurement read(
      const roo_transceivers::SensorId& sensor_id) const override;

  void updateState(const roo::byte* data, size_t len) override;

 private:
  roo_comms_HomeAutomationDeviceDescriptor_EnvironmentalSensor descriptor_;
  roo_comms_DataMessage_EnvironmentalSensorReadings state_;
  roo_time::Uptime last_reading_;
};

}  // namespace roo_comms

#endif  // ROO_COMMS_HUB_HUB_DEVICE_ENVIRONMENTAL_SENSOR_H