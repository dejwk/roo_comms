#pragma once

#include "roo_comms/transport/esp_now_transport.h"

namespace roo_comms {

class Peer : public EspNowPeer {
 public:
  Peer(const roo_io::MacAddress& addr);

  /// Sends a message and waits for delivery to the peer ESP service.
  ///
  /// Usually takes a few ms. Returns true if delivery was successful; false
  /// otherwise. FIFO delivery order is guaranteed per peer.
  bool send(const void* data, size_t len);

  /// Sends a message without waiting for delivery.
  ///
  /// Returns true if the message was successfully enqueued; false otherwise.
  bool sendAsync(const void* data, size_t len);
};

}  // namespace roo_comms