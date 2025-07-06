#ifndef ROO_COMMS_HUB_DEVICE_H
#define ROO_COMMS_HUB_DEVICE_H

#include "roo_comms.h"
#include "roo_comms/transport/esp_now_transport.h"
#include "roo_io/net/mac_address.h"
#include "roo_transceivers.pb.h"
#include "roo_transceivers/id.h"
#include "roo_transceivers/measurement.h"

namespace roo_comms {

class HubDevice {
 public:
  HubDevice(const roo_io::MacAddress& destination)
      : HubDevice(Transport(), destination) {}

  // For testing.
  HubDevice(EspNowTransport& transport, const roo_io::MacAddress& destination)
      : transport_(transport), destination_(destination) {}

  virtual ~HubDevice() = default;

  virtual void getDescriptor(roo_transceivers_Descriptor& result) const = 0;

  virtual roo_transceivers::Measurement read(
      const roo_transceivers::SensorId& sensor_id) const = 0;

  virtual bool write(const roo_transceivers::ActuatorId& actuator_id,
                     float value) const {
    return false;
  }

  virtual void requestUpdate() const {}

  virtual void updateState(const uint8_t* data, size_t len) = 0;

 protected:
  EspNowTransport& transport() const { return transport_; }
  const roo_io::MacAddress& destination() const { return destination_; }

 private:
  EspNowTransport& transport_;
  roo_io::MacAddress destination_;
};

}  // namespace roo_comms

#endif  // ROO_COMMS_HUB_DEVICE_H