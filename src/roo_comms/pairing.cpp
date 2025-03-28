#include "roo_comms/pairing.h"

namespace roo_comms {

SerializedControlMessage SerializeControlMessage(
    const roo_comms_ControlMessage& msg, const Magic& magic) {
  SerializedControlMessage result;
  memcpy(result.data, magic, 8);
  pb_ostream_t stream =
      pb_ostream_from_buffer(result.data + 8, sizeof(result.data) - 8);
  bool status = pb_encode(&stream, roo_comms_ControlMessage_fields, &msg);
  if (status) {
    result.size = stream.bytes_written;
  } else {
    LOG(ERROR) << "Encoding failed: " << PB_GET_ERROR(&stream);
    result.size = 0;
  }
  return result;
}

void SendDiscoveryRequest(EspNowTransport& transport,
                          const roo_comms_DeviceDescriptor& descriptor) {
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_discovery_request_tag;
  msg.contents.hub_pairing_request.has_device_descriptor = true;
  msg.contents.hub_discovery_request.device_descriptor = descriptor;

  auto serialized = SerializeControlMessage(msg, roo_comms::kControlMagic);
  transport.broadcastAsync(serialized.data, serialized.size);
}

void SendDiscoveryResponse(EspNowTransport& transport,
                           const roo_io::MacAddress& origin) {
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_discovery_response_tag;
  msg.contents.hub_discovery_response.hub_channel = transport.channel();

  auto serialized = SerializeControlMessage(msg, roo_comms::kControlMagic);
  transport.sendOnceAsync(origin, serialized.data, serialized.size);
}

// Sends a an ack to a pairing request.
void SendPairingResponse(EspNowTransport& transport,
                         const roo_io::MacAddress& origin) {
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_pairing_response_tag;
  msg.contents.hub_pairing_response.status =
      roo_comms_ControlMessage_HubPairingResponse_Status_kOk;
  auto serialized = SerializeControlMessage(msg, roo_comms::kControlMagic);
  transport.sendOnceAsync(origin, serialized.data, serialized.size);
}

void SendPairingRequest(EspNowPeer& peer,
                        const roo_comms_DeviceDescriptor& descriptor) {
  LOG(INFO) << "Sending pairing request message";
  roo_comms_ControlMessage msg = roo_comms_ControlMessage_init_zero;
  msg.which_contents = roo_comms_ControlMessage_hub_pairing_request_tag;
  msg.contents.hub_pairing_request.has_device_descriptor = true;
  msg.contents.hub_pairing_request.device_descriptor = descriptor;
  auto result = SerializeControlMessage(msg, roo_comms::kControlMagic);
  peer.sendAsync(result.data, result.size);
}

}  // namespace roo_comms