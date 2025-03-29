#include "roo_comms/hub/devices/hub_device_relay.h"

#include <cstring>

#include "roo_comms/home_automation.h"

namespace roo_comms {

HubDeviceRelay::HubDeviceRelay(EspNowTransport& transport,
                               const roo_io::MacAddress& destination,
                               int relay_count)
    : HubDevice(transport, destination),
      relay_count_(relay_count),
      state_(0),
      last_reading_(roo_time::Uptime::Start()) {}

void HubDeviceRelay::getDescriptor(
    roo_transceivers_Descriptor& descriptor) const {
  descriptor.actuators_count = relay_count_;
  descriptor.sensors_count = relay_count_;
  for (size_t i = 0; i < relay_count_; ++i) {
    snprintf(descriptor.sensors[i].id, 24, "relay_%d", i + 1);
    descriptor.sensors[i].quantity = roo_transceivers_Quantity_kBinaryState;
    snprintf(descriptor.actuators[i].id, 24, "relay_%d", i + 1);
    descriptor.actuators[i].quantity = roo_transceivers_Quantity_kBinaryState;
  }

  strcpy(descriptor.actuators[0].id, "relay");
  descriptor.actuators[0].quantity = roo_transceivers_Quantity_kBinaryState;
}

namespace {

int extractRelayId(const char* actuator_id) {
  if (strlen(actuator_id) == 7 && strncmp(actuator_id, "relay_", 6) == 0 &&
      actuator_id[6] >= '0' && actuator_id[6] <= '8') {
    return actuator_id[6] - '0';
  }
  return -1;
}

}  // namespace

roo_transceivers::Measurement HubDeviceRelay::read(
    const roo_transceivers::SensorId& sensor_id) const {
  int d = extractRelayId(sensor_id.c_str());
  if (d >= 0) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kBinaryState, last_reading_,
        last_reading_ > roo_time::Uptime::Start()
            ? ((state_ & (1 << d)) == 0 ? 0.0f : 1.0f)
            : nanf(""));
  }
  return roo_transceivers::Measurement();
}

bool HubDeviceRelay::write(const roo_transceivers::ActuatorId& actuator_id,
                           float value) const {
  if (value != 0.0f && value != 1.0f) return false;
  int d = extractRelayId(actuator_id.c_str());
  if (d >= 0) {
    return WriteRelay(transport(), destination(), d, value == 1.0f);
  }
  return false;
}

void HubDeviceRelay::requestUpdate() const {
  RequestRelayState(transport(), destination());
}

void HubDeviceRelay::updateState(const roo::byte* data, size_t len) {
  roo_comms_DataMessage data_message;
  if (!TryParsingAsHomeAutomationDataMessage(data, len, data_message)) {
    LOG(WARNING) << "Failed to parse data message";
    return;
  }
  if (data_message.which_contents != roo_comms_DataMessage_relay_response_tag) {
    LOG(WARNING) << "Ignoring non-relay response message";
    return;
  }
  state_ = data_message.contents.relay_response.state;
  last_reading_ = roo_time::Uptime::Now();
}

}  // namespace roo_comms
