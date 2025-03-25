#include "esp_now_transport.h"

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

void SendEspNowControlMessage(const esp_now_peer_info_t& peer,
                              const Magic& magic,
                              const roo_comms_ControlMessage& msg) {
  auto result = SerializeControlMessage(msg, magic);
  SendEspNowMessage(peer, result.data, result.size);
}

void SendEspNowMessage(const esp_now_peer_info_t& peer, const void* buf,
                       size_t len) {
  esp_err_t result = esp_now_send(peer.peer_addr, (const uint8_t*)buf, len);
  if (result != ESP_OK) {
    LOG(ERROR) << "ESP-NOW sending data message failed with code "
               << (int)result;
  }
}

}  // namespace roo_comms