#include "roo_comms.h"

#include "roo_comms/transport/esp_now_transport.h"

namespace roo_comms {

EspNowTransport& Transport() {
  static EspNowTransport transport;
  return transport;
}

namespace {

#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 1, 0)

void OnDataSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
  Transport().ackSent(roo_io::MacAddress(tx_info->des_addr),
                      status == ESP_NOW_SEND_SUCCESS);
}

void OnDataRecv(const esp_now_recv_info* recv_info, const uint8_t* data,
                int len) {
  Transport().onDataRecv(roo_io::MacAddress(recv_info->src_addr), data, len);
}

#else

// Legacy API.

void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
  Transport().ackSent(roo_io::MacAddress(mac_addr),
                      status == ESP_NOW_SEND_SUCCESS);
}

void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int len) {
  Transport().onDataRecv(roo_io::MacAddress(mac_addr), data, len);
}

#endif

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

bool Send(const roo_io::MacAddress& addr, const void* data, size_t len) {
  return Transport().sendOnce(addr, data, len);
}

bool SendAsync(const roo_io::MacAddress& addr, const void* data, size_t len) {
  return Transport().sendOnceAsync(addr, data, len);
}

void BroadcastAsync(const void* data, size_t len) {
  return Transport().broadcastAsync(data, len);
}

uint8_t GetWiFiChannel() { return Transport().channel(); }

void SetWiFiChannel(uint8_t channel) { Transport().setChannel(channel); }

}  // namespace roo_comms
