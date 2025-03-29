#include "roo_comms/hub/hub.h"

#include <algorithm>

#include "roo_backport.h"
#include "roo_io/text/base64.h"
#include "roo_io/text/string_printf.h"

namespace roo_comms {

static const char *kMacsKey = "macs";
static constexpr size_t kMaxPairedDevices = 1000;

static roo_transceivers::DeviceSchema kEspNowSchema =
    roo_transceivers::DeviceSchema("roo");

void Hub::processDiscoveryRequest(
    const roo_io::MacAddress &origin,
    const roo_comms_DeviceDescriptor &descriptor) {
  if (!checkSupportedType(descriptor)) return;
  SendDiscoveryResponse(transport_, origin);
}

void Hub::processPairingRequest(const roo_io::MacAddress &origin,
                                const roo_comms_DeviceDescriptor &descriptor) {
  if (!checkSupportedType(descriptor)) return;
  if (!addTransceiver(origin, descriptor)) {
    LOG(WARNING) << "Failed to add transceiver " << origin;
    return;
  }
  SendPairingResponse(transport_, origin);
}

bool Hub::checkSupportedType(const roo_comms_DeviceDescriptor &descriptor) {
  if (descriptor.realm_id != roo_comms_RealmId_kHomeAutomation) return false;
  roo_comms_HomeAutomationDeviceDescriptor home_automation_descriptor;
  if (!TryParseHomeAutomationDescriptor(descriptor,
                                        home_automation_descriptor)) {
    return false;
  }
  auto kind = home_automation_descriptor.which_kind;
  if (kind !=
          roo_comms_HomeAutomationDeviceDescriptor_environmental_sensor_tag &&
      kind != roo_comms_HomeAutomationDeviceDescriptor_relay_tag) {
    LOG(INFO) << "Ignoring pairing request from unsupported transceiver type "
              << kind;
    return false;
  }
  return true;
}

void Hub::processMessage(const roo_comms::Receiver::Message &received) {
  {
    roo_comms_ControlMessage msg;
    if (TryParsingAsControlMessage(received.data.get(), received.size, msg)) {
      switch (msg.which_contents) {
        case roo_comms_ControlMessage_hub_discovery_request_tag: {
          processDiscoveryRequest(
              received.source,
              msg.contents.hub_discovery_request.device_descriptor);
          break;
        }
        case roo_comms_ControlMessage_hub_pairing_request_tag: {
          LOG(INFO) << "Processing pairing request from " << received.source;
          processPairingRequest(
              received.source,
              msg.contents.hub_pairing_request.device_descriptor);
          break;
        }
        case roo_comms_ControlMessage_hub_discovery_response_tag:
        case roo_comms_ControlMessage_hub_pairing_response_tag: {
          LOG(WARNING) << "Unexpected message type; ignoring.";
        }
      }
      return;
    }
  }
  if (payload_cb_ != nullptr) {
    payload_cb_(received);
  }
}

Hub::Hub(EspNowTransport &transport, roo_scheduler::Scheduler &scheduler)
    : store_("hub"),
      transport_(transport),
      receiver_(
          scheduler,
          [this](const roo_comms::Receiver::Message &received) {
            processMessage(received);
          },
          100, 8, 256, nullptr),
      payload_cb_(
          [this](const Receiver::Message &msg) { processDataMessage(msg); }),
      transceiver_changed_cb_([this]() { notifyTransceiversChanged(); }) {}

void Hub::init(uint8_t channel) {
  transport_.setChannel(channel);
  transceiver_addresses_.clear();
  transceiver_details_.clear();
  std::unique_ptr<roo_io::byte[]> data = nullptr;
  size_t device_count = 0;
  roo_prefs::Transaction t(store_);
  size_t len = 0;
  roo_prefs::ReadResult status;
  status = t.store().readBytesLength(kMacsKey, &len);
  if (status == roo_prefs::READ_NOT_FOUND) {
    return;
  }
  CHECK(status == roo_prefs::READ_OK);
  if (len % 6 != 0) {
    CHECK(t.store().clear(kMacsKey) == roo_prefs::CLEAR_OK);
    len = 0;
  }
  if (len > 6 * kMaxPairedDevices) {
    LOG(WARNING) << "Max number of paired devices exceeded";
    len = 6 * kMaxPairedDevices;
  }
  if (len > 0) {
    data.reset(new roo_io::byte[len]);
    CHECK(t.store().readBytes(kMacsKey, data.get(), len, nullptr) ==
          roo_prefs::READ_OK);
    device_count = len / 6;
  }
  transceiver_addresses_.reserve(device_count);
  for (size_t i = 0; i < device_count; ++i) {
    roo_io::MacAddress addr(&data[i * 6]);
    char addr_key[13];
    snprintf(addr_key, 13, "%012" PRIX64, addr.asU64());
    size_t details_length;
    CHECK_EQ(t.store().readBytesLength(addr_key, &details_length),
             roo_prefs::READ_OK);
    roo::byte buf[details_length];
    CHECK_EQ(t.store().readBytes(addr_key, buf, details_length, nullptr),
             roo_prefs::READ_OK);
    pb_istream_t istream =
        pb_istream_from_buffer((const pb_byte_t *)buf, details_length);
    roo_comms_DeviceDescriptor descriptor;
    bool status =
        pb_decode(&istream, roo_comms_DeviceDescriptor_fields, &descriptor);
    if (!status) {
      LOG(WARNING) << "Ignoring " << addr.asString();
    }
    transceiver_addresses_.push_back(addr);
    transceiver_details_[addr] = descriptor;
    LOG(INFO) << "Loaded info for paired transceiver " << addr;
  }
}

bool Hub::addTransceiver(const roo_io::MacAddress &addr,
                         const roo_comms_DeviceDescriptor &descriptor) {
  if (!std::binary_search(transceiver_addresses_.begin(),
                          transceiver_addresses_.end(), addr)) {
    transceiver_addresses_.push_back(addr);
    std::sort(transceiver_addresses_.begin(), transceiver_addresses_.end());
    roo_prefs::Transaction t(store_);
    writeTransceiverAddresses(t);
    LOG(INFO) << "Registering transceiver " << addr;
  } else {
    LOG(INFO) << "Overwriting previous registration of " << addr;
  }
  transceiver_details_[addr] = descriptor;
  pb_byte_t buf[roo_comms_DeviceDescriptor_size];
  pb_ostream_t ostream = pb_ostream_from_buffer(buf, sizeof(buf));
  CHECK(pb_encode(&ostream, roo_comms_DeviceDescriptor_fields, &descriptor))
      << PB_GET_ERROR(&ostream);
  char addr_key[13];
  snprintf(addr_key, 13, "%012" PRIX64, addr.asU64());
  {
    roo_prefs::Transaction t(store_);
    CHECK_EQ(t.store().writeBytes(addr_key, buf, ostream.bytes_written),
             roo_prefs::WRITE_OK);
  }
  if (transceiver_changed_cb_ != nullptr) {
    transceiver_changed_cb_();
  }
  return true;
}

bool Hub::removeTransceiver(const roo_io::MacAddress &addr) {
  auto itr = std::lower_bound(transceiver_addresses_.begin(),
                              transceiver_addresses_.end(), addr);
  if (itr == transceiver_addresses_.end() || *itr != addr) {
    // Not found.
    return false;
  }
  if (itr - transceiver_addresses_.begin() ==
      transceiver_addresses_.size() - 1) {
    // Itr is already the last element. Just remove it.
    transceiver_addresses_.pop_back();
  } else {
    // Let's swap itr with the last element, remove last, and and re-sort.
    std::swap(*itr, transceiver_addresses_.back());
    transceiver_addresses_.pop_back();
    std::sort(transceiver_addresses_.begin(), transceiver_addresses_.end());
  }
  roo_prefs::Transaction t(store_);
  writeTransceiverAddresses(t);
  char addr_key[13];
  snprintf(addr_key, 13, "%012" PRIX64, addr.asU64());
  CHECK_EQ(t.store().clear(addr_key), roo_prefs::CLEAR_OK);

  if (transceiver_changed_cb_ != nullptr) {
    transceiver_changed_cb_();
  }
  return true;
}

void Hub::writeTransceiverAddresses(roo_prefs::Transaction &t) {
  size_t len = transceiver_addresses_.size() * 6;
  std::unique_ptr<roo_io::byte[]> encoded(new roo_io::byte[len]);
  for (size_t i = 0; i < transceiver_addresses_.size(); ++i) {
    transceiver_addresses_[i].writeTo(&encoded[i * 6]);
  }
  CHECK_EQ(t.store().writeBytes(kMacsKey, encoded.get(), len),
           roo_prefs::WRITE_OK);
}

bool Hub::hasTransceiver(const roo_io::MacAddress &addr) {
  return std::binary_search(transceiver_addresses_.begin(),
                            transceiver_addresses_.end(), addr);
}

void Hub::onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
  receiver_.handle(mac, incomingData, len);
}

const roo_comms_DeviceDescriptor *Hub::lookupDescriptor(
    const roo_io::MacAddress &addr) const {
  const auto &itr = transceiver_details_.find(addr);
  return (itr == transceiver_details_.end()) ? nullptr : &itr->second;
}

roo_comms_DeviceDescriptor *Hub::lookupDescriptor(
    const roo_io::MacAddress &addr) {
  auto itr = transceiver_details_.find(addr);
  return (itr != transceiver_details_.end()) ? nullptr : &itr->second;
}

void Hub::processDataMessage(const Receiver::Message &msg) {
  // Cache the payload so that we can report values from it when asked.
  // For now, all devices send their full state in a single message, so no
  // merging is necessary.
  DeviceState &state = states_[msg.source];
  state.last_reading = roo_time::Uptime::Now();
  if (!TryParsingAsHomeAutomationDataMessage(msg.data.get(), msg.size,
                                             state.last_payload)) {
    LOG(WARNING) << "Failed to parse data message";
    return;
  }

  notifyNewReadingsAvailable();
}

namespace {
bool ParseMac(const roo_transceivers::DeviceLocator &loc,
              roo_io::MacAddress &result) {
  if (loc.schema() != kEspNowSchema) {
    LOG(WARNING) << "Wrong schema";
    return false;
  }
  if (!result.parseFrom(loc.device_id().c_str())) {
    LOG(WARNING) << "Failed to parse " << loc.device_id().c_str();
    return false;
  }
  return true;
}
}  // namespace

bool Hub::forEachDevice(
    std::function<bool(const roo_transceivers::DeviceLocator &)> callback)
    const {
  for (size_t i = 0; i < deviceCount(); ++i) {
    if (!callback(device_locator(i))) {
      return false;
    }
  }
  return true;
}

roo_transceivers::DeviceLocator Hub::device_locator(size_t device_idx) const {
  char buf[18];
  device(device_idx).writeStringTo(buf);
  return roo_transceivers::DeviceLocator(kEspNowSchema, buf);
}

namespace {

static const char *kAht20Temperature = "aht20_temperature";
static const char *kAht20Humidity = "aht20_humidity";
static const char *kBmp280Temperature = "bmp280_temperature";
static const char *kBmp280Pressure = "bmp280_pressure";

void BuildEnvironmentalSensorDescriptor(
    const roo_comms_HomeAutomationDeviceDescriptor &input,
    roo_transceivers_Descriptor &result) {
  result.actuators_count = 0;
  size_t sensor_idx = 0;
  if (input.kind.environmental_sensor.has_aht20) {
    strcpy(result.sensors[sensor_idx].id, kAht20Temperature);
    result.sensors[sensor_idx].quantity =
        roo_transceivers_Quantity_kTemperature;
    sensor_idx++;
    strcpy(result.sensors[sensor_idx].id, kAht20Humidity);
    result.sensors[sensor_idx].quantity =
        roo_transceivers_Quantity_kAirHumidity;
    sensor_idx++;
  }
  if (input.kind.environmental_sensor.has_bmp280) {
    strcpy(result.sensors[sensor_idx].id, kBmp280Temperature);
    result.sensors[sensor_idx].quantity =
        roo_transceivers_Quantity_kTemperature;
    sensor_idx++;
    strcpy(result.sensors[sensor_idx].id, kBmp280Pressure);
    result.sensors[sensor_idx].quantity = roo_transceivers_Quantity_kPressure;
    sensor_idx++;
  }
  result.sensors_count = sensor_idx;
}

roo_transceivers::Measurement ReadEnvironmentalSensor(
    const DeviceState &state, const roo_transceivers::SensorId &sensor_id) {
  const auto &d = state.last_payload.contents.environmental_sensor_readings;
  if (sensor_id == kAht20Temperature) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kTemperature, state.last_reading,
        d.has_aht20 && d.aht20.has_temperature_celsius
            ? d.aht20.temperature_celsius
            : nanf(""));
  }
  if (sensor_id == kAht20Humidity) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kAirHumidity, state.last_reading,
        d.has_aht20 && d.aht20.has_humidity_percent ? d.aht20.humidity_percent
                                                    : nanf(""));
  }
  if (sensor_id == kBmp280Temperature) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kTemperature, state.last_reading,
        d.has_bmp280 && d.bmp280.has_temperature_celsius
            ? d.bmp280.temperature_celsius
            : nanf(""));
  }
  if (sensor_id == kBmp280Pressure) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kPressure, state.last_reading,
        d.has_bmp280 && d.bmp280.has_pressure_pa ? d.bmp280.pressure_pa
                                                 : nanf(""));
  }
  return roo_transceivers::Measurement();
}

void BuildRelayDescriptor(const roo_comms_HomeAutomationDeviceDescriptor &input,
                          roo_transceivers_Descriptor &result) {
  size_t relay_count = input.kind.relay.port_count;
  result.sensors_count = relay_count;
  result.actuators_count = relay_count;
  for (size_t i = 0; i < relay_count; ++i) {
    snprintf(result.sensors[i].id, 24, "relay_%d", i + 1);
    result.sensors[i].quantity = roo_transceivers_Quantity_kBinaryState;
    snprintf(result.actuators[i].id, 24, "relay_%d", i + 1);
    result.actuators[i].quantity = roo_transceivers_Quantity_kBinaryState;
  }
}

int extractRelayId(const char *locator) {
  if (strlen(locator) == 7 && strncmp(locator, "relay_", 6) == 0 &&
      locator[6] >= '0' && locator[6] <= '8') {
    return locator[6] - '0';
  }
  return -1;
}

roo_transceivers::Measurement ReadRelay(
    const DeviceState &state, const roo_transceivers::SensorId &sensor_id) {
  uint8_t relay_state = state.last_payload.contents.relay_response.state;
  int d = extractRelayId(sensor_id.c_str());
  if (d >= 0) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kBinaryState, state.last_reading,
        (relay_state & (1 << d)) == 0 ? 0.0f : 1.0f);
  }
  return roo_transceivers::Measurement();
}

bool WriteRelayUniversal(EspNowTransport &transport,
                         const roo_io::MacAddress &device,
                         const roo_transceivers::ActuatorId &actuator_id,
                         float value) {
  if (value != 0.0f && value != 1.0f) return false;
  int d = extractRelayId(actuator_id.c_str());
  if (d >= 0) {
    return WriteRelay(transport, device, d, value == 1.0f);
  }
  return false;
}

}  // namespace

bool Hub::getDeviceDescriptor(const roo_transceivers::DeviceLocator &locator,
                              roo_transceivers_Descriptor &descriptor) const {
  roo_io::MacAddress addr;
  if (!ParseMac(locator, addr)) return false;
  LOG(INFO) << "Parsed mac: " << addr;
  const roo_comms_DeviceDescriptor *raw = lookupDescriptor(addr);
  if (raw == nullptr) {
    LOG(WARNING) << "Did not find a descriptor for " << addr;
    return false;
  }
  roo_comms_HomeAutomationDeviceDescriptor home_automation_descriptor;
  CHECK(TryParseHomeAutomationDescriptor(*raw, home_automation_descriptor));
  switch (home_automation_descriptor.which_kind) {
    case roo_comms_HomeAutomationDeviceDescriptor_environmental_sensor_tag: {
      BuildEnvironmentalSensorDescriptor(home_automation_descriptor,
                                         descriptor);
      return true;
    }
    case roo_comms_HomeAutomationDeviceDescriptor_relay_tag: {
      BuildRelayDescriptor(home_automation_descriptor, descriptor);
      return true;
    }
    default: {
      return false;
    }
  }
}

roo_transceivers::Measurement Hub::read(
    const roo_transceivers::SensorLocator &locator) const {
  roo_io::MacAddress addr;
  if (!ParseMac(locator.device_locator(), addr)) {
    return roo_transceivers::Measurement();
  }
  const roo_comms_DeviceDescriptor *descriptor = lookupDescriptor(addr);
  if (descriptor == nullptr) return roo_transceivers::Measurement();
  const auto &itr = states_.find(addr);
  if (itr == states_.end()) {
    return roo_transceivers::Measurement();
  }
  const DeviceState &state = itr->second;
  roo_comms_HomeAutomationDeviceDescriptor home_automation_descriptor;
  CHECK(TryParseHomeAutomationDescriptor(*descriptor,
                                         home_automation_descriptor));
  switch (home_automation_descriptor.which_kind) {
    case roo_comms_HomeAutomationDeviceDescriptor_environmental_sensor_tag: {
      return ReadEnvironmentalSensor(state, locator.sensor_id());
    }
    case roo_comms_HomeAutomationDeviceDescriptor_relay_tag: {
      return ReadRelay(state, locator.sensor_id());
    }
    default: {
      return roo_transceivers::Measurement();
    }
  }
}

bool Hub::write(const roo_transceivers::ActuatorLocator &locator,
                float value) const {
  roo_io::MacAddress addr;
  if (!ParseMac(locator.device_locator(), addr)) {
    return false;
  }
  const roo_comms_DeviceDescriptor *descriptor = lookupDescriptor(addr);
  if (descriptor == nullptr) return false;
  roo_comms_HomeAutomationDeviceDescriptor home_automation_descriptor;
  CHECK(TryParseHomeAutomationDescriptor(*descriptor,
                                         home_automation_descriptor));
  switch (home_automation_descriptor.which_kind) {
    case roo_comms_HomeAutomationDeviceDescriptor_environmental_sensor_tag: {
      // These have no actuators.
      return false;
    }
    case roo_comms_HomeAutomationDeviceDescriptor_relay_tag: {
      WriteRelayUniversal(transport_, addr, locator.actuator_id(), value);
      return true;
    }
    default: {
      return false;
    }
  }

  return false;
}

void Hub::requestUpdate() {
  size_t count = deviceCount();
  for (size_t i = 0; i < count; ++i) {
    const roo_io::MacAddress &addr = device(i);
    const roo_comms_DeviceDescriptor *descriptor = lookupDescriptor(addr);
    if (descriptor == nullptr) continue;
    roo_comms_HomeAutomationDeviceDescriptor home_automation_descriptor;
    CHECK(TryParseHomeAutomationDescriptor(*descriptor,
                                           home_automation_descriptor));
    switch (home_automation_descriptor.which_kind) {
      case roo_comms_HomeAutomationDeviceDescriptor_relay_tag: {
        RequestRelayState(transport_, addr);
        break;
      }
      default: {
        break;
      }
    }
  }
}

void Hub::addEventListener(roo_transceivers::EventListener *listener) {
  listeners_.insert(listener);
}

void Hub::removeEventListener(roo_transceivers::EventListener *listener) {
  listeners_.erase(listener);
}

void Hub::notifyTransceiversChanged() {
  for (auto *listener : listeners_) {
    listener->devicesChanged();
  }
}

void Hub::notifyNewReadingsAvailable() {
  for (auto *listener : listeners_) {
    listener->newReadingsAvailable();
  }
}

}  // namespace roo_comms