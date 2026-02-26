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

/// ESP-NOW transport abstraction for easier testing and convenience helpers.

namespace roo_comms {

/// Parses a payload as a control message.
bool TryParsingAsControlMessage(const uint8_t* incoming_data, size_t len,
                                roo_comms_ControlMessage& msg);

class EspNowPeer;

class EspNowTransport {
 public:
  EspNowTransport() : channel_(0) {}

  /// Initializes ESP-NOW and optional protocol mode.
  void begin(Mode mode);
  /// Shuts down ESP-NOW transport.
  void end();

  /// Returns the current Wi-Fi channel.
  uint8_t channel() const { return channel_; }

  /// Sets the Wi-Fi channel (call after Wi-Fi initialization).
  void setChannel(uint8_t channel);

  /// Registers receiver callback.
  void setReceiverFn(ReceiverFn receiver_fn);

  /// Sends and waits for delivery to the recipient ESP service.
  ///
  /// Usually takes a few ms. Returns true if delivery was successful; false
  /// otherwise. FIFO delivery order is guaranteed per recipient.
  bool send(const EspNowPeer& peer, const void* data, size_t len);

  /// Sends without waiting for delivery.
  ///
  /// Returns true if the message was enqueued; false otherwise.
  bool sendAsync(const EspNowPeer& peer, const void* data, size_t len);

  /// Like send(), but for a raw MAC address without a configured peer.
  bool sendOnce(const roo_io::MacAddress& addr, const void* data, size_t len);

  /// Like sendAsync(), but for a raw MAC address without a configured peer.
  bool sendOnceAsync(const roo_io::MacAddress& addr, const void* data,
                     size_t len);

  /// Broadcasts a message without waiting for delivery.
  void broadcastAsync(const void* data, size_t len);

  /// Must be called by the ESP-NOW send callback to release pending sends.
  void ackSent(const roo_io::MacAddress& addr, bool success);

  /// Must be called by the ESP-NOW receive callback.
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
    // Count of pending requests.
    int count;
  };

  struct AutoPeer {
    int refcount;
    esp_now_peer_info_t peer;
  };

  // Guarded by the pending_send_mutex_.
  esp_now_peer_info_t* getOrCreateAutoPeer(const roo_io::MacAddress& addr);

  // Guarded by the pending_send_mutex_.
  void releaseAutoPeer(const roo_io::MacAddress& addr);

  bool sendImpl(const roo_io::MacAddress& addr, const esp_now_peer_info_t* peer,
                const void* data, size_t len);

  bool sendAsyncImpl(const roo_io::MacAddress& addr,
                     const esp_now_peer_info_t* peer, const void* data,
                     size_t len);

  ReceiverFn receiver_fn_;
  friend class EspNowPeer;
  roo::mutex pending_send_mutex_;
  roo::condition_variable pending_emptied_;

  // Map of pending outboxes (unacknowledged requests).
  roo_collections::FlatSmallHashMap<roo_io::MacAddress, Outbox> pending_;

  // Map of auto-created peers, used by sendOnce/sendOnceAsync.
  roo_collections::FlatSmallHashMap<roo_io::MacAddress, AutoPeer*> auto_peers_;

  uint8_t channel_;
};

class EspNowPeer {
 public:
  EspNowPeer(EspNowTransport& transport, const roo_io::MacAddress& addr);

  ~EspNowPeer();
  /// Sends and waits for delivery to the peer ESP service.
  ///
  /// Usually takes a few ms. Returns true if delivery was successful; false
  /// otherwise. FIFO delivery order is guaranteed per peer.
  bool send(const void* data, size_t len);

  /// Sends without waiting for delivery.
  ///
  /// Returns true if the message was enqueued; false otherwise.
  bool sendAsync(const void* data, size_t len);

 private:
  friend class EspNowTransport;

  EspNowTransport& transport_;
  esp_now_peer_info_t peer_;
};

// using Magic = roo_io::byte[8];

}  // namespace roo_comms
