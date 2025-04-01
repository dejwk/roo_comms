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
  return device_factory_.isDeviceSupported(descriptor);
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
  processDataMessage(received);
}

Hub::Hub(EspNowTransport &transport, roo_scheduler::Scheduler &scheduler,
         HubDeviceFactory &device_factory)
    : store_("hub"),
      transport_(transport),
      receiver_(
          scheduler,
          [this](const roo_comms::Receiver::Message &received) {
            processMessage(received);
          },
          100, 8, 256, nullptr),
      device_factory_(device_factory) {}

void Hub::init(uint8_t channel) {
  transport_.setChannel(channel);
  transceiver_addresses_.clear();
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
    std::unique_ptr<HubDevice> device =
        device_factory_.createDevice(transport_, addr, descriptor);
    if (device == nullptr) {
      LOG(WARNING) << "Failed to create device for " << addr;
    } else {
      devices_[addr] = std::move(device);
    }
    LOG(INFO) << "Loaded info for paired transceiver " << addr;
  }
}

bool Hub::addTransceiver(const roo_io::MacAddress &addr,
                         const roo_comms_DeviceDescriptor &descriptor) {
  std::unique_ptr<HubDevice> device =
      device_factory_.createDevice(transport_, addr, descriptor);
  if (device == nullptr) {
    LOG(WARNING) << "Failed to create device for " << addr;
    return false;
  }
  devices_[addr] = std::move(device);
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
  notifyTransceiversChanged();
  return true;
}

bool Hub::removeTransceiver(const roo_io::MacAddress &addr) {
  auto itr = std::lower_bound(transceiver_addresses_.begin(),
                              transceiver_addresses_.end(), addr);
  if (itr == transceiver_addresses_.end() || *itr != addr) {
    // Not found.
    return false;
  }
  devices_.erase(addr);
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
  notifyTransceiversChanged();
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

void Hub::processDataMessage(const Receiver::Message &msg) {
  HubDevice *device = lookupDevice(msg.source);
  if (device == nullptr) {
    LOG(WARNING) << "Received data message from unknown device "
                 << msg.source.asString();
    return;
  }
  device->updateState(msg.data.get(), msg.size);

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

bool Hub::getDeviceDescriptor(const roo_transceivers::DeviceLocator &locator,
                              roo_transceivers_Descriptor &descriptor) const {
  roo_io::MacAddress addr;
  if (!ParseMac(locator, addr)) return false;
  LOG(INFO) << "Parsed mac: " << addr;
  HubDevice *device = lookupDevice(addr);
  if (device == nullptr) {
    LOG(WARNING) << "Received data message from unknown device " << addr;
    return false;
  }
  device->getDescriptor(descriptor);
  return true;
}

roo_transceivers::Measurement Hub::read(
    const roo_transceivers::SensorLocator &locator) const {
  roo_io::MacAddress addr;
  if (!ParseMac(locator.device_locator(), addr)) {
    return roo_transceivers::Measurement();
  }
  HubDevice *device = lookupDevice(addr);
  if (device == nullptr) {
    LOG(WARNING) << "Received data message from unknown device " << addr;
    return roo_transceivers::Measurement();
  }
  return device->read(locator.sensor_id());
}

bool Hub::write(const roo_transceivers::ActuatorLocator &locator,
                float value) const {
  roo_io::MacAddress addr;
  if (!ParseMac(locator.device_locator(), addr)) {
    return false;
  }
  HubDevice *device = lookupDevice(addr);
  if (device == nullptr) {
    LOG(WARNING) << "Received write request targetet at an unknown device "
                 << addr;
    return false;
  }
  return device->write(locator.actuator_id(), value);
}

void Hub::requestUpdate() {
  size_t count = deviceCount();
  for (size_t i = 0; i < count; ++i) {
    const roo_io::MacAddress &addr = device(i);
    HubDevice *device = lookupDevice(addr);
    if (device == nullptr) continue;
    device->requestUpdate();
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