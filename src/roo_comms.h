#pragma once

#include <functional>

#include "comms.pb.h"
#include "roo_io/net/mac_address.h"

namespace roo_comms {

enum Mode {
  kNormalMode = 0,
  kLongRangeMode = 1  // Activates WIFI_PROTOCOL_LR.
};

struct Source {
  roo_io::MacAddress addr;
};

using ReceiverFn =
    std::function<void(const Source&, const void* data, size_t len)>;

void Begin(Mode mode);

void SetReceiverFn(ReceiverFn receiver_fn);

void End();

uint8_t GetWiFiChannel();

// Must be called after Wifi is initialized.
void SetWiFiChannel(uint8_t channel);

// Sends the packet to the specified destination and waits until it gets
// delivered to the recipient ESP service. Usually takes a few ms. Returns true
// if delivery was successful; false otherwise.
bool Send(const roo_io::MacAddress& addr, const void* data, size_t len);

// Sends the packet to the specified destination without waiting until it gets
// delivered to the recipient ESP service. Returns true if message was
// successfully enqueued for send; false otherwise.
bool SendAsync(const roo_io::MacAddress& addr, const void* data, size_t len);

// Broadcasts the packet.
void BroadcastAsync(const void* data, size_t len);

class EspNowTransport;

EspNowTransport& Transport();

}  // namespace roo_comms
