#include "roo_comms/hub/devices/hub_device_relay.h"

#include <cstring>

#include "roo_comms/home_automation.h"

namespace roo_comms {

HubDeviceRelay::HubDeviceRelay(EspNowTransport& transport,
                               const roo_io::MacAddress& destination,
                               size_t relay_count)
    : HubDevice(transport, destination),
      relay_count_(relay_count),
      state_(0),
      last_reading_(roo_time::Uptime::Start()) {}

void HubDeviceRelay::getDescriptor(roo_transceivers::Descriptor& result) const {
  result.Clear();
  for (size_t i = 0; i < relay_count_; ++i) {
    char id[32];
    snprintf(id, sizeof(id), "relay_%zu", i + 1);
    auto* sensor = result.add_sensors();
    sensor->set_id(id);
    sensor->set_quantity(roo_transceivers::Quantity::kBinaryState);
    auto* actuator = result.add_actuators();
    actuator->set_id(id);
    actuator->set_quantity(roo_transceivers::Quantity::kBinaryState);
  }
}

namespace {

int extractRelayId(const char* actuator_id) {
  if (strlen(actuator_id) == 7 && strncmp(actuator_id, "relay_", 6) == 0 &&
      actuator_id[6] >= '1' && actuator_id[6] <= '8') {
    return actuator_id[6] - '1';
  }
  return -1;
}

}  // namespace

roo_transceivers::Measurement HubDeviceRelay::read(
    const roo_transceivers::SensorId& sensor_id) const {
  int d = extractRelayId(sensor_id.c_str());
  if (d >= 0) {
    return roo_transceivers::Measurement(
        roo_transceivers::Quantity::kBinaryState, last_reading_,
        last_reading_ > roo_time::Uptime::Start()
            ? ((state_ & (1 << d)) == 0 ? 0.0f : 1.0f)
            : nanf(""));
  }
  return roo_transceivers::Measurement();
}

bool HubDeviceRelay::write(const roo_transceivers::ActuatorId& actuator_id,
                           float value) const {
  if (value != 0.0f && value != 1.0f) {
    LOG(WARNING) << "Received bogus write value " << value;
    return false;
  }
  int d = extractRelayId(actuator_id.c_str());
  if (d >= 0) {
    return WriteRelay(transport(), destination(), d, value == 1.0f);
  }
  LOG(WARNING) << "Received bogus relay ID " << actuator_id.c_str();
  return false;
}

void HubDeviceRelay::requestUpdate() const {
  RequestRelayState(transport(), destination());
}

void HubDeviceRelay::updateState(const uint8_t* data, size_t len) {
  roo::comms::DataMessage data_message;
  if (!TryParsingAsHomeAutomationDataMessage(data, len, data_message)) {
    LOG(WARNING) << "Failed to parse data message";
    return;
  }
  if (data_message.contents_case() !=
      roo::comms::DataMessage::ContentsCase::kRelayResponse) {
    LOG(WARNING) << "Ignoring non-relay response message";
    return;
  }
  state_ = data_message.relay_response().state();
  last_reading_ = roo_time::Uptime::Now();
}

}  // namespace roo_comms
