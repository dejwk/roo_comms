#include "roo_comms/home_automation.h"

namespace roo_comms {

// Payload identifier for 'home automation' device universe, using
// roo::comms::DataMessage payload.
static constexpr roo_io::byte kDataMagicHomeAutomation[8] = {
    roo_io::byte{'r'},  roo_io::byte{'o'},  roo_io::byte{'o'},
    roo_io::byte{0},    roo_io::byte{0x5E}, roo_io::byte{0x0C},
    roo_io::byte{0x15}, roo_io::byte{0x03}};

bool TryParsingAsHomeAutomationDataMessage(const uint8_t* incoming_data,
                                           size_t len,
                                           roo::comms::DataMessage& msg) {
  if (len < 8 || memcmp(incoming_data, kDataMagicHomeAutomation, 8) != 0)
    return false;
  auto status = roo_pb::Parse(incoming_data + 8, len - 8, msg);
  if (status != roo_pb::Status::kOk) {
    LOG(ERROR) << "Received a malformed message " << static_cast<int>(status);
    return false;
  }
  return true;
}

SerializedHomeAutomationDataMessage SerializeHomeAutomationDataMessage(
    const roo::comms::DataMessage& msg) {
  SerializedHomeAutomationDataMessage result;
  memcpy(result.data, kDataMagicHomeAutomation, 8);
  size_t written = 0;
  auto status =
      roo_pb::Serialize(msg, result.data + 8, sizeof(result.data) - 8, written);
  if (status == roo_pb::Status::kOk) {
    result.size = written + 8;
  } else {
    LOG(ERROR) << "Encoding failed: " << static_cast<int>(status);
    result.size = 0;
  }
  return result;
}

bool RequestRelayState(EspNowTransport& transport,
                       const roo_io::MacAddress& device) {
  roo::comms::DataMessage msg = {};

  msg.mutable_relay_request()->set_mask(0);
  msg.mutable_relay_request()->set_write(0);
  auto serialized = SerializeHomeAutomationDataMessage(msg);
  return transport.sendOnce(device, serialized.data, serialized.size);
}

bool WriteRelay(EspNowTransport& transport, const roo_io::MacAddress& device,
                int relay_idx, bool is_enabled) {
  roo::comms::DataMessage msg = {};

  msg.mutable_relay_request()->set_mask((1 << relay_idx));
  msg.mutable_relay_request()->set_write(is_enabled ? (1 << relay_idx) : 0);
  auto serialized = SerializeHomeAutomationDataMessage(msg);
  return transport.sendOnce(device, serialized.data, serialized.size);
}

roo::comms::DeviceDescriptor BuildHomeAutomationDescriptor(
    const roo::comms::HomeAutomationDeviceDescriptor& input) {
  roo::comms::DeviceDescriptor result = {};
  result.set_realm_id(
      static_cast<int64_t>(roo::comms::RealmId::kHomeAutomation));
  uint8_t buffer[roo::comms::HomeAutomationDeviceDescriptor::kMaxEncodedSize];
  size_t written = 0;
  CHECK(roo_pb::Serialize(input, buffer, sizeof(buffer), written) ==
        roo_pb::Status::kOk);
  CHECK(result.try_set_payload(reinterpret_cast<const char*>(buffer), written));
  return result;
}

bool TryParseHomeAutomationDescriptor(
    const roo::comms::DeviceDescriptor& input,
    roo::comms::HomeAutomationDeviceDescriptor& result) {
  return result.ParseFromArray(input.payload().data(), input.payload().size());
}

}  // namespace roo_comms
