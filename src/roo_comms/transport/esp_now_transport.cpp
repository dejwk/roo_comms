#include "esp_now_transport.h"

#include "WiFi.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "roo_logging.h"

#if !defined(MLOG_roo_esp_now_transport)
#define MLOG_roo_esp_now_transport 0
#endif

namespace roo_comms {

void EspNowTransport::begin(Mode mode) {
  WiFi.mode(WIFI_STA);
  esp_wifi_set_protocol(
      WIFI_IF_STA, mode == kNormalMode ? WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G |
                                             WIFI_PROTOCOL_11N
                                       : WIFI_PROTOCOL_LR);
  ESP_ERROR_CHECK(esp_now_init());
}

void EspNowTransport::end() { esp_now_deinit(); }

void EspNowTransport::setReceiverFn(ReceiverFn receiver_fn) {
  receiver_fn_ = std::move(receiver_fn);
}

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
  roo_io::MacAddress addr(peer.peer_.peer_addr);
  {
    roo::unique_lock<roo::mutex> lock(pending_send_mutex_);
    while (true) {
      auto itr = pending_.find(addr);
      if (itr == pending_.end()) {
        pending_[addr] = Outbox(Outbox::kSyncPending);
        break;
      }
      pending_emptied_.wait(lock);
    }
  }
  esp_err_t result =
      esp_now_send(peer.peer_.peer_addr, (const uint8_t*)data, len);
  if (result != ESP_OK) {
    LOG(ERROR) << "ESP-NOW sending data message failed with code "
               << (int)result;
    roo::unique_lock<roo::mutex> lock(pending_send_mutex_);
    pending_.erase(addr);
    pending_emptied_.notify_all();
    return false;
  }
  // Wait for the send to be acknowledged.
  // Note: since we released the mutex above, the ack may have already been
  // delivered before we reacquire it. Therefore we check the state before
  // waiting.
  {
    roo::unique_lock<roo::mutex> lock(pending_send_mutex_);
    bool is_success = false;
    while (true) {
      Outbox& pending = pending_[addr];
      if (pending.status == Outbox::kSyncSuccessful) {
        is_success = true;
        break;
      } else if (pending.status == Outbox::kSyncFailed) {
        break;
      } else if (pending.count == 0) {
        LOG(ERROR) << "Unexpected status: " << pending.status;
        break;
      }
      CHECK_EQ(pending.status, Outbox::kSyncPending);
      pending_emptied_.wait(lock);
    }
    pending_.erase(addr);
    pending_emptied_.notify_all();
    return is_success;
  }
}

bool EspNowTransport::sendAsync(const EspNowPeer& peer, const void* data,
                                size_t len) {
  roo_io::MacAddress addr(peer.peer_.peer_addr);
  while (true) {
    roo::unique_lock<roo::mutex> lock(pending_send_mutex_);
    auto itr = pending_.find(addr);
    if (itr == pending_.end()) {
      pending_[addr] = Outbox(Outbox::kAsyncPending);
      break;
    } else if (itr->second.status == Outbox::kAsyncPending) {
      itr->second.count++;
      break;
    }

    // If there are any synchronous requests, wait for them to finish before
    // sending new async requests. This is so that we can attribute delivery
    // failures properly.
    pending_emptied_.wait(lock);
  }
  esp_err_t result =
      esp_now_send(peer.peer_.peer_addr, (const uint8_t*)data, len);
  if (result == ESP_OK) {
    MLOG(roo_esp_now_transport)
        << "Sending packet of " << len << " bytes to: " << addr;
    return true;
  } else {
    LOG(ERROR) << "ESP-NOW sending data message failed with code "
               << (int)result;
    roo::unique_lock<roo::mutex> lock(pending_send_mutex_);
    Outbox& pending = pending_[addr];
    CHECK_EQ(Outbox::kAsyncPending, pending.status);
    pending.count--;
    if (pending.count == 0) {
      pending_.erase(addr);
      pending_emptied_.wait(lock);
    }
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
  MLOG(roo_esp_now_transport)
      << "Delivery status for " << addr << ": " << success;
  roo::lock_guard<roo::mutex> lock(pending_send_mutex_);
  Outbox& pending = pending_[addr];
  CHECK_GT(pending.count, 0);
  if (--pending.count == 0) {
    switch (pending.status) {
      case Outbox::kAsyncPending: {
        pending_.erase(addr);
        break;
      }
      case Outbox::kSyncPending: {
        pending.status =
            success ? Outbox::kSyncSuccessful : Outbox::kSyncFailed;
        break;
      }
      default: {
        LOG(ERROR) << "Unexpected status: " << (int)pending.status;
        pending_.erase(addr);
      }
    }
    pending_emptied_.notify_all();
    MLOG(roo_esp_now_transport) << "No more calls pending for " << addr;
  };
}

void EspNowTransport::onDataRecv(const roo_io::MacAddress& addr,
                                 const void* data, size_t len) {
  if (receiver_fn_ == nullptr) {
    LOG(WARNING) << "Received data from " << addr
                << " but no receiver function is set. Ignoring.";
    return;
  }
  Source source{.addr = addr};
  receiver_fn_(source, data, len);
}

}  // namespace roo_comms