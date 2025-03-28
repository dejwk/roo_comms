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

bool EspNowPeer::sendAsync(const void* data, size_t len) {
  return transport_.sendAsync(*this, data, len);
}

bool EspNowTransport::send(const EspNowPeer& peer, const void* data,
                           size_t len) {
  while (true) {
    std::unique_lock<std::mutex> lock(pending_send_mutex_);
    Counter& pending = pending_[roo_io::MacAddress(peer.peer_.peer_addr)];
    if (pending.count == 0) {
      ++pending.count;
      break;
    }
    pending_emptied_.wait(lock);
  }
  esp_err_t result =
      esp_now_send(peer.peer_.peer_addr, (const uint8_t*)data, len);
  if (result == ESP_OK) {
    LOG(INFO) << "Packet seding: " << roo_io::MacAddress(peer.peer_.peer_addr);
    return true;
  } else {
    LOG(ERROR) << "ESP-NOW sending data message failed with code "
               << (int)result;
    return false;
  }
}

bool EspNowTransport::sendAsync(const EspNowPeer& peer, const void* data,
                                size_t len) {
  {
    std::lock_guard<std::mutex> lock(pending_send_mutex_);
    Counter& pending = pending_[roo_io::MacAddress(peer.peer_.peer_addr)];
    pending.count++;
  }
  esp_err_t result =
      esp_now_send(peer.peer_.peer_addr, (const uint8_t*)data, len);
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

bool EspNowTransport::sendOnceAsync(const roo_io::MacAddress& addr,
                                    const void* data, size_t len) {
  return sendAsync(EspNowPeer(*this, addr), data, len);
}

void EspNowTransport::broadcastAsync(const void* data, size_t len) {
  sendAsync(EspNowPeer(*this, roo_io::MacAddress::Broadcast()), data, len);
}

void EspNowTransport::ackSent(const roo_io::MacAddress& addr, bool success) {
  LOG(INFO) << "Delivery status: " << success;
  std::lock_guard<std::mutex> lock(pending_send_mutex_);
  Counter& pending = pending_[addr];
  CHECK_GT(pending.count, 0);
  if (--pending.count == 0) {
    pending_.erase(addr);
    pending_emptied_.notify_all();
    LOG(INFO) << "No more calls pending per " << addr;
  };
}

}  // namespace roo_comms