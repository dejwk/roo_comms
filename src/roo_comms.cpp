#include "roo_comms.h"

#include "roo_comms/transport/esp_now_transport.h"

namespace roo_comms {

EspNowTransport& Transport() {
  static EspNowTransport transport;
  return transport;
}

namespace {

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Transport().ackSent(roo_io::MacAddress(mac_addr),
                      status == ESP_NOW_SEND_SUCCESS);
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int len) {
  Transport().onDataRecv(roo_io::MacAddress(mac_addr), data, len);
}

}  // namespace

void Begin(Mode mode) {
  Transport().begin(mode);
  esp_now_register_send_cb(OnDataSent);
  esp_now_register_recv_cb(OnDataRecv);
}

void SetReceiverFn(ReceiverFn receiver_fn) {
  Transport().setReceiverFn(std::move(receiver_fn));
}

void End() {
  esp_now_unregister_send_cb();
  esp_now_unregister_recv_cb();
  Transport().end();
}

uint8_t GetWiFiChannel() { return Transport().channel(); }

void SetWiFiChannel(uint8_t channel) { Transport().setChannel(channel); }

}  // namespace roo_comms
