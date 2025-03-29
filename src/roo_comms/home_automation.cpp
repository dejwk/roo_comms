#include "roo_comms/home_automation.h"

namespace roo_comms {

// Payload identifier for 'home automation' device universe, using
// roo_comms_DataMessage payload.
static constexpr roo_io::byte kDataMagicHomeAutomation[8] = {
    roo_io::byte{'r'},  roo_io::byte{'o'},  roo_io::byte{'o'},
    roo_io::byte{0},    roo_io::byte{0x5E}, roo_io::byte{0x0C},
    roo_io::byte{0x15}, roo_io::byte{0x03}};

bool TryParsingAsHomeAutomationDataMessage(const uint8_t* incoming_data,
                                           size_t len,
                                           roo_comms_DataMessage& msg) {
  if (memcmp(incoming_data, kDataMagicHomeAutomation, 8) != 0) return false;
  msg = roo_comms_DataMessage_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(incoming_data + 8, len - 8);
  bool status = pb_decode(&stream, roo_comms_DataMessage_fields, &msg);
  if (!status) {
    LOG(ERROR) << "Received a malformed message " << PB_GET_ERROR(&stream);
    return false;
  }
  return true;
}

SerializedHomeAutomationDataMessage SerializeHomeAutomationDataMessage(
    const roo_comms_DataMessage& msg) {
  SerializedHomeAutomationDataMessage result;
  memcpy(result.data, kDataMagicHomeAutomation, 8);
  pb_ostream_t stream =
      pb_ostream_from_buffer(result.data + 8, sizeof(result.data) - 8);
  bool status = pb_encode(&stream, roo_comms_DataMessage_fields, &msg);
  if (status) {
    result.size = stream.bytes_written;
  } else {
    LOG(ERROR) << "Encoding failed: " << PB_GET_ERROR(&stream);
    result.size = 0;
  }
  return result;
}

bool RequestRelayState(EspNowTransport& transport,
                       const roo_io::MacAddress& device) {
  roo_comms_DataMessage msg = roo_comms_DataMessage_init_zero;
  msg.which_contents = roo_comms_DataMessage_relay_request_tag;
  msg.contents.relay_request.mask = 0;
  msg.contents.relay_request.write = 0;
  auto serialized = SerializeHomeAutomationDataMessage(msg);
  return transport.sendOnce(device, serialized.data, serialized.size);
}

bool WriteRelay(EspNowTransport& transport, const roo_io::MacAddress& device,
                int relay_idx, bool is_enabled) {
  roo_comms_DataMessage msg = roo_comms_DataMessage_init_zero;
  msg.which_contents = roo_comms_DataMessage_relay_request_tag;
  msg.contents.relay_request.mask = (1 << relay_idx);
  msg.contents.relay_request.write = is_enabled ? (1 << relay_idx) : 0;
  auto serialized = SerializeHomeAutomationDataMessage(msg);
  return transport.sendOnce(device, serialized.data, serialized.size);
}

roo_comms_DeviceDescriptor BuildHomeAutomationDescriptor(
    const roo_comms_HomeAutomationDeviceDescriptor& input) {
  roo_comms_DeviceDescriptor result = roo_comms_DeviceDescriptor_init_zero;
  result.realm_id = roo_comms_RealmId_kHomeAutomation;
  pb_ostream_t stream = pb_ostream_from_buffer(result.payload.bytes, 128);
  CHECK(pb_encode(&stream, roo_comms_HomeAutomationDeviceDescriptor_fields,
                  &input));
  result.payload.size = stream.bytes_written;
  return result;
}

bool TryParseHomeAutomationDescriptor(
    const roo_comms_DeviceDescriptor& input,
    roo_comms_HomeAutomationDeviceDescriptor& result) {
  result = roo_comms_HomeAutomationDeviceDescriptor_init_zero;
  pb_istream_t stream =
      pb_istream_from_buffer(input.payload.bytes, input.payload.size);
  return pb_decode(&stream, roo_comms_HomeAutomationDeviceDescriptor_fields,
                   &result);
}

}  // namespace roo_comms
