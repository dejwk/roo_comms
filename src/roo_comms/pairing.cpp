#include "roo_comms/pairing.h"

namespace roo_comms {

static constexpr roo_io::byte kControlMagic[8] = {
    roo_io::byte{'r'},  roo_io::byte{'o'},  roo_io::byte{'o'},
    roo_io::byte{0},    roo_io::byte{0xE1}, roo_io::byte{0xB2},
    roo_io::byte{0x88}, roo_io::byte{0x99}};

namespace {

struct SerializedControlMessage {
  pb_byte_t data[8 + roo_comms_ControlMessage_size];
  size_t size;
};

SerializedControlMessage SerializeControlMessage(
    const roo_comms_ControlMessage& msg) {
  SerializedControlMessage result;
  memcpy(result.data, kControlMagic, 8);
  pb_ostream_t stream =
      pb_ostream_from_buffer(result.data + 8, sizeof(result.data) - 8);
  bool status = pb_encode(&stream, roo_comms_ControlMessage_fields, &msg);
  if (status) {
    result.size = stream.bytes_written + 8;
  } else {
    LOG(ERROR) << "Encoding failed: " << PB_GET_ERROR(&stream);
    result.size = 0;
  }
  return result;
}

}  // namespace

bool TryParsingAsControlMessage(const uint8_t* incoming_data, size_t len,
                                roo_comms_ControlMessage& msg) {
  if (memcmp(incoming_data, kControlMagic, 8) != 0) {
    return false;
  }
  msg = roo_comms_ControlMessage_init_zero;
  pb_istream_t stream = pb_istream_from_buffer(incoming_data + 8, len - 8);
  bool status = pb_decode(&stream, roo_comms_ControlMessage_fields, &msg);
  if (!status) {
    LOG(ERROR) << "Received a malformed message: " << PB_GET_ERROR(&stream);
    return false;
  }
  return true;
}

void SendDiscoveryRequest(EspNowTransport& transport,
                          const roo_comms_DeviceDescriptor& descriptor) {
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_discovery_request_tag;
  msg.contents.hub_pairing_request.has_device_descriptor = true;
  msg.contents.hub_discovery_request.device_descriptor = descriptor;

  auto serialized = SerializeControlMessage(msg);
  transport.broadcastAsync(serialized.data, serialized.size);
}

void SendDiscoveryResponse(EspNowTransport& transport,
                           const roo_io::MacAddress& origin) {
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_discovery_response_tag;
  msg.contents.hub_discovery_response.hub_channel = transport.channel();

  auto serialized = SerializeControlMessage(msg);
  transport.sendOnceAsync(origin, serialized.data, serialized.size);
}

// Sends a an ack to a pairing request.
void SendPairingResponse(EspNowTransport& transport,
                         const roo_io::MacAddress& origin) {
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_pairing_response_tag;
  msg.contents.hub_pairing_response.status =
      roo_comms_ControlMessage_HubPairingResponse_Status_kOk;
  auto serialized = SerializeControlMessage(msg);
  transport.sendOnceAsync(origin, serialized.data, serialized.size);
}

void SendPairingRequest(EspNowPeer& peer,
                        const roo_comms_DeviceDescriptor& descriptor) {
  LOG(INFO) << "Sending pairing request message";
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_pairing_request_tag;
  msg.contents.hub_pairing_request.has_device_descriptor = true;
  msg.contents.hub_pairing_request.device_descriptor = descriptor;
  auto result = SerializeControlMessage(msg);
  peer.sendAsync(result.data, result.size);
}

}  // namespace roo_comms