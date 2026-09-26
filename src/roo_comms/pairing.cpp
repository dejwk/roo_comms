#include "roo_comms/pairing.h"

namespace roo_comms {

static constexpr roo_io::byte kControlMagic[8] = {
    roo_io::byte{'r'},  roo_io::byte{'o'},  roo_io::byte{'o'},
    roo_io::byte{0},    roo_io::byte{0xE1}, roo_io::byte{0xB2},
    roo_io::byte{0x88}, roo_io::byte{0x99}};

namespace {

struct SerializedControlMessage {
  uint8_t data[8 + roo::comms::ControlMessage::kMaxEncodedSize];
  size_t size;
};

SerializedControlMessage SerializeControlMessage(
    const roo::comms::ControlMessage& msg) {
  SerializedControlMessage result;
  memcpy(result.data, kControlMagic, 8);
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

}  // namespace

bool TryParsingAsControlMessage(const uint8_t* incoming_data, size_t len,
                                roo::comms::ControlMessage& msg) {
  if (len < 8 || memcmp(incoming_data, kControlMagic, 8) != 0) {
    return false;
  }
  auto status = roo_pb::Parse(incoming_data + 8, len - 8, msg);
  if (status != roo_pb::Status::kOk) {
    LOG(ERROR) << "Received a malformed message: " << static_cast<int>(status);
    return false;
  }
  return true;
}

void SendDiscoveryRequest(EspNowTransport& transport,
                          const roo::comms::DeviceDescriptor& descriptor) {
  roo::comms::ControlMessage msg = {};

  *msg.mutable_hub_discovery_request()->mutable_device_descriptor() =
      descriptor;

  auto serialized = SerializeControlMessage(msg);
  transport.broadcastAsync(serialized.data, serialized.size);
}

void SendDiscoveryResponse(EspNowTransport& transport,
                           const roo_io::MacAddress& origin) {
  roo::comms::ControlMessage msg = {};

  msg.mutable_hub_discovery_response()->set_hub_channel(transport.channel());

  auto serialized = SerializeControlMessage(msg);
  transport.sendOnceAsync(origin, serialized.data, serialized.size);
}

// Sends a an ack to a pairing request.
void SendPairingResponse(EspNowTransport& transport,
                         const roo_io::MacAddress& origin) {
  roo::comms::ControlMessage msg = {};

  msg.mutable_hub_pairing_response()->set_status(
      roo::comms::ControlMessage::HubPairingResponse::Status::kOk);
  auto serialized = SerializeControlMessage(msg);
  transport.sendOnceAsync(origin, serialized.data, serialized.size);
}

void SendPairingRequest(EspNowPeer& peer,
                        const roo::comms::DeviceDescriptor& descriptor) {
  LOG(INFO) << "Sending pairing request message";
  roo::comms::ControlMessage msg = {};

  *msg.mutable_hub_pairing_request()->mutable_device_descriptor() = descriptor;
  auto result = SerializeControlMessage(msg);
  peer.sendAsync(result.data, result.size);
}

}  // namespace roo_comms