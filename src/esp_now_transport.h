#pragma once

#include <thread>

#include "comms.pb.h"
#include "esp_now.h"
#include "message_queue.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "roo_collections/flat_small_hash_map.h"
#include "roo_collections/flat_small_hash_set.h"
#include "roo_io/base/byte.h"
#include "roo_io/memory/memory_output_iterator.h"
#include "roo_io/net/mac_address.h"
#include "roo_logging.h"
#include "roo_scheduler.h"

namespace roo_comms {

bool TryParsingAsControlMessage(const uint8_t* incoming_data, size_t len,
                                roo_comms_ControlMessage& msg);

class EspNowPeer;

class EspNowTransport {
 public:
  EspNowTransport() : channel_(0) {}

  uint8_t channel() const { return channel_; }

  // Must be called after Wifi is initialized.
  void setChannel(uint8_t channel);

  bool sendOnce(const roo_io::MacAddress& addr, const void* data, size_t len);

  bool sendOnceAsync(const roo_io::MacAddress& addr, const void* data,
                     size_t len);

  // Sends a message and waits until it gets delivered to the recipient ESP
  // service. Usually takes a few ms. Returns true if delivery was successful;
  // false otherwise.
  //
  // FIFO delivery order is guaranteed per recipient.
  bool send(const EspNowPeer& peer, const void* data, size_t len);

  // Sends a message without waiting for delivery. Returns true if the message
  // was successfully enqueued; false otherwise.
  bool sendAsync(const EspNowPeer& peer, const void* data, size_t len);

  void broadcastAsync(const void* data, size_t len);

  // Must be called by the registered ESP-NOW callback. Otherwise, memory leaks
  // and deadlocks may occur.
  void ackSent(const roo_io::MacAddress& addr, bool success);

 private:
  struct Outbox {
    enum Status {
      kDone,
      kAsyncPending,
      kSyncPending,
      kSyncSuccessful,
      kSyncFailed
    };
    Outbox() : status(kDone), count(0) {}
    Status status;
    int count;
  };
  friend class EspNowPeer;
  std::mutex pending_send_mutex_;
  std::condition_variable pending_emptied_;
  roo_collections::FlatSmallHashMap<roo_io::MacAddress, Outbox> pending_;

  uint8_t channel_;
};

class EspNowPeer {
 public:
  EspNowPeer(EspNowTransport& transport, const roo_io::MacAddress& addr);

  ~EspNowPeer();
  // Sends a message and waits until it gets delivered to the peer ESP
  // service. Usually takes a few ms. Returns true if delivery was successful;
  // false otherwise.
  //
  // FIFO delivery order is guaranteed per peer.
  bool send(const void* data, size_t len);

  // Sends a message without waiting for delivery. Returns true if the message
  // was successfully enqueued; false otherwise.
  bool sendAsync(const void* data, size_t len);

 private:
  friend class EspNowTransport;

  EspNowTransport& transport_;
  esp_now_peer_info_t peer_;
};

// using Magic = roo_io::byte[8];

class Receiver {
 public:
  struct Message {
    roo_io::MacAddress source;
    size_t size;
    std::unique_ptr<roo_io::byte[]> data;
  };

  using ProcessorFn = std::function<void(const Message&)>;
  using ValidatorFn = std::function<bool(const roo_io::byte* data, size_t len)>;

  Receiver(roo_scheduler::Scheduler& scheduler, ProcessorFn processor_fn,
           size_t max_queue_size, size_t min_msg_size, size_t max_msg_size,
           ValidatorFn validator_fn)
      : queue_([this]() { processor_.scheduleNow(); }),
        processor_fn_(std::move(processor_fn)),
        validator_fn_(std::move(validator_fn)),
        processor_(scheduler, [this]() { processMessages(); }),
        max_quqeue_size_(max_queue_size),
        min_msg_size_(min_msg_size),
        max_msg_size_(max_msg_size) {}

  void handle(const uint8_t* mac_addr, const uint8_t* incoming_data,
              size_t len) {
    LOG(INFO) << "Received message of size " << len << " from "
              << roo_io::MacAddress(mac_addr);
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
    if (validator_fn_ != nullptr && !validator_fn_(incoming_data, len)) {
      LOG(WARNING) << "Received message, but validation failed; ignoring";
      return;
    }
    std::unique_ptr<roo_io::byte[]> data(new roo_io::byte[len]);
    memcpy(data.get(), incoming_data, len);
    queue_.push(Message{.source = roo_io::MacAddress(mac_addr),
                        .size = len,
                        .data = std::move(data)});
  }

  void processMessages() {
    Message msg;
    while (queue_.pop(msg)) {
      processor_fn_(msg);
    }
  }

 private:
  MessageQueue<Message> queue_;
  ProcessorFn processor_fn_;
  ValidatorFn validator_fn_;
  size_t max_quqeue_size_;
  size_t min_msg_size_;
  size_t max_msg_size_;
  roo_scheduler::SingletonTask processor_;
};

}  // namespace roo_comms
