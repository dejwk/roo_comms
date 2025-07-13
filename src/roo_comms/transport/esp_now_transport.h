#pragma once

#include "WiFi.h"
#include "comms.pb.h"
#include "esp_now.h"
#include "pb_decode.h"
#include "pb_encode.h"
#include "roo_backport/byte.h"
#include "roo_collections.h"
#include "roo_collections/flat_small_hash_map.h"
#include "roo_collections/flat_small_hash_set.h"
#include "roo_comms.h"
#include "roo_io.h"
#include "roo_io/memory/memory_output_iterator.h"
#include "roo_io/net/mac_address.h"
#include "roo_logging.h"
#include "roo_scheduler.h"
#include "roo_threads.h"
#include "roo_threads/condition_variable.h"
#include "roo_threads/mutex.h"

// This file abstracts away ESP-NOW so that it can be more easily unit-tested.
// Additionally, it adds some convenience features, such as synchronous send,
// simple broadcast, etc.

namespace roo_comms {

bool TryParsingAsControlMessage(const uint8_t* incoming_data, size_t len,
                                roo_comms_ControlMessage& msg);

class EspNowPeer;

class EspNowTransport {
 public:
  EspNowTransport() : channel_(0) {}

  void begin(Mode mode);
  void end();

  uint8_t channel() const { return channel_; }

  // Must be called after Wifi is initialized.
  void setChannel(uint8_t channel);

  void setReceiverFn(ReceiverFn receiver_fn);

  // Sends a message and waits until it gets delivered to the recipient ESP
  // service. Usually takes a few ms. Returns true if delivery was successful;
  // false otherwise.
  //
  // FIFO delivery order is guaranteed per recipient.
  bool send(const EspNowPeer& peer, const void* data, size_t len);

  // Sends a message without waiting for delivery. Returns true if the message
  // was successfully enqueued; false otherwise.
  bool sendAsync(const EspNowPeer& peer, const void* data, size_t len);

  // Similar to send, but doesn't require a preconfigured peer; just a MAC
  // address.
  bool sendOnce(const roo_io::MacAddress& addr, const void* data, size_t len);

  // Similar to sendAsync, but doesn't require a preconfigured peer; just a MAC
  // address.
  bool sendOnceAsync(const roo_io::MacAddress& addr, const void* data,
                     size_t len);

  void broadcastAsync(const void* data, size_t len);
  
  // Must be called by the registered ESP-NOW callback. Otherwise, memory leaks
  // and deadlocks may occur.
  void ackSent(const roo_io::MacAddress& addr, bool success);

  void onDataRecv(const roo_io::MacAddress& addr, const void* data, size_t len);

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
    Outbox(Status status) : status(status), count(1) {}
    Status status;
    int count;
  };

  ReceiverFn receiver_fn_;
  friend class EspNowPeer;
  roo::mutex pending_send_mutex_;
  roo::condition_variable pending_emptied_;
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

}  // namespace roo_comms
