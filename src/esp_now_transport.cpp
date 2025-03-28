#include "esp_now_transport.h"

#include "esp_err.h"
#include "esp_wifi.h"

namespace roo_comms {

void EspNowTransport::setChannel(uint8_t channel) {
  channel_ = channel;
  ESP_ERROR_CHECK(esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE));
}

EspNowPeer::EspNowPeer(EspNowTransport& transport,
                       const roo_io::MacAddress& addr)
    : transport_(transport), peer_{} {
  addr.writeTo(peer_.peer_addr);
  peer_.channel = transport.channel_;
  peer_.encrypt = 0;  // no encryption
  ESP_ERROR_CHECK(esp_now_add_peer(&peer_));
}

EspNowPeer::~EspNowPeer() {
  ESP_ERROR_CHECK(esp_now_del_peer(peer_.peer_addr));
}

bool EspNowPeer::send(const void* data, size_t len) {
  return transport_.send(*this, data, len);
}

bool EspNowTransport::send(const EspNowPeer& peer, const void* data,
                           size_t len) {
  esp_err_t result = esp_now_send(peer.peer_.peer_addr, (const uint8_t*)data, len);
  if (result == ESP_OK) {
    LOG(INFO) << "Packet seding: " << roo_io::MacAddress(peer.peer_.peer_addr);
    return true;
  } else {
    LOG(ERROR) << "ESP-NOW sending data message failed with code "
               << (int)result;
    return false;
  }
}

bool EspNowTransport::sendOnce(const roo_io::MacAddress& addr, const void* data,
                               size_t len) {
  return send(EspNowPeer(*this, addr), data, len);
}

void EspNowTransport::broadcast(const void* data, size_t len) {
  send(EspNowPeer(*this, roo_io::MacAddress::Broadcast()), data, len);
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

void SendEspNowControlMessage(EspNowPeer& peer, const Magic& magic,
                              const roo_comms_ControlMessage& msg) {
  auto result = SerializeControlMessage(msg, magic);
  peer.send(result.data, result.size);
}

}  // namespace roo_comms