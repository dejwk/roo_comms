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
    std::unique_lock<std::mutex> lock(pending_send_mutex_);
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
    std::unique_lock<std::mutex> lock(pending_send_mutex_);
    pending_.erase(addr);
    pending_emptied_.notify_all();
    return false;
  }
  // Wait for the send to be acknowledged.
  // Note: since we released the mutex above, the ack may have already been
  // delivered before we reacquire it. Therefore we check the state before
  // waiting.
  {
    std::unique_lock<std::mutex> lock(pending_send_mutex_);
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
    std::unique_lock<std::mutex> lock(pending_send_mutex_);
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
    std::unique_lock<std::mutex> lock(pending_send_mutex_);
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
  std::lock_guard<std::mutex> lock(pending_send_mutex_);
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

Receiver::Receiver(roo_scheduler::Scheduler& scheduler,
                   ProcessorFn processor_fn, size_t max_queue_size,
                   size_t min_msg_size, size_t max_msg_size,
                   ValidatorFn validator_fn)
    : queue_([this]() { processor_.scheduleNow(); }),
      processor_fn_(std::move(processor_fn)),
      validator_fn_(std::move(validator_fn)),
      processor_(scheduler, [this]() { processMessages(); }),
      max_quqeue_size_(max_queue_size),
      min_msg_size_(min_msg_size),
      max_msg_size_(max_msg_size) {}

void Receiver::handle(const uint8_t* mac_addr, const uint8_t* incoming_data,
                      size_t len) {
  MLOG(roo_esp_now_transport) << "Received message of size " << len << " from "
                              << roo_io::MacAddress(mac_addr) << "; queue size: " << queue_.size();
  if (len < min_msg_size_) {
    LOG(WARNING) << "Received bogus message (too short: " << len
                 << " bytes); ignoring";
    return;
  }
  if (len > max_msg_size_) {
    LOG(WARNING) << "Received bogus message (too large: " << len
                 << " bytes); ignoring";
    return;
  }
  if (queue_.size() >= max_quqeue_size_) {
    LOG(WARNING) << "Received message, but queue is full; ignoring";
    return;
  }
  if (validator_fn_ != nullptr &&
      !validator_fn_((const roo::byte*)incoming_data, len)) {
    LOG(WARNING) << "Received message, but validation failed; ignoring";
    return;
  }
  std::unique_ptr<roo_io::byte[]> data(new roo::byte[len]);
  memcpy(data.get(), incoming_data, len);
  queue_.push(Message{.source = roo_io::MacAddress(mac_addr),
                      .size = len,
                      .data = std::move(data)});
}

void Receiver::processMessages() {
  Message msg;
  while (queue_.pop(msg)) {
    processor_fn_(msg);
  }
}

}  // namespace roo_comms