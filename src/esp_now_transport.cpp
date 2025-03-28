#include "esp_now_transport.h"

#include "esp_err.h"

namespace roo_comms {

EspNowPeer::EspNowPeer(const EspNowTransport& transport,
                       const roo_io::MacAddress& addr)
    : peer_{} {
  addr.writeTo(peer_.peer_addr);
  peer_.channel = transport.channel_;
  peer_.encrypt = 0;  // no encryption
  ESP_ERROR_CHECK(esp_now_add_peer(&peer_));
}

EspNowPeer::~EspNowPeer() {
  ESP_ERROR_CHECK(esp_now_del_peer(peer_.peer_addr));
}

bool EspNowPeer::send(const void* data, size_t len) const {
  esp_err_t result = esp_now_send(peer_.peer_addr, (const uint8_t*)data, len);
  if (result == ESP_OK) {
    return true;
  } else {
    LOG(ERROR) << "ESP-NOW sending data message failed with code "
               << (int)result;
    return false;
  }
}

void EspNowTransport::sendOnce(const roo_io::MacAddress& addr, const void* data,
                               size_t len) const {
  EspNowPeer peer(*this, addr);
  peer.send(data, len);
}

void EspNowTransport::broadcast(const void* data, size_t len) const {
  EspNowPeer peer(*this, roo_io::MacAddress::Broadcast());
  peer.send(data, len);
}

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

SerializedDataMessage SerializeDataMessage(const roo_comms_DataMessage& msg,
                                           const Magic& magic) {
  SerializedDataMessage result;
  memcpy(result.data, magic, 8);
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

void SendEspNowControlMessage(const EspNowPeer& peer,
                              const Magic& magic,
                              const roo_comms_ControlMessage& msg) {
  auto result = SerializeControlMessage(msg, magic);
  peer.send(result.data, result.size);
}

}  // namespace roo_comms