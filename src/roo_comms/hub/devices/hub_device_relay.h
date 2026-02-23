#ifndef ROO_COMMS_HUB_HUB_DEVICE_RELAY_H
#define ROO_COMMS_HUB_HUB_DEVICE_RELAY_H

#include "roo_comms/hub/hub_device.h"
#include "roo_time.h"

namespace roo_comms {

class HubDeviceRelay : public HubDevice {
 public:
  HubDeviceRelay(EspNowTransport& transport,
                 const roo_io::MacAddress& destination, size_t relay_count);

  void getDescriptor(roo_transceivers_Descriptor& result) const override;

  roo_transceivers::Measurement read(
      const roo_transceivers::SensorId& sensor_id) const override;

  bool write(const roo_transceivers::ActuatorId& actuator_id,
             float value) const override;

  void requestUpdate() const override;

  void updateState(const uint8_t* data, size_t len) override;

 private:
  size_t relay_count_;
  uint32_t state_;
  roo_time::Uptime last_reading_;
};

}  // namespace roo_comms

#endif  // ROO_COMMS_HUB_HUB_DEVICE_RELAY_H