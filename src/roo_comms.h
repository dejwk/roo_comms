#pragma once

/// Umbrella header for the roo_comms module.
///
/// Defines communication mode and callback registration helpers.

#include <functional>

#include "comms.pb.h"
#include "roo_io/net/mac_address.h"

namespace roo_comms {

/// Operating mode for the communication stack.
enum Mode {
  kNormalMode = 0,
  kLongRangeMode = 1  // Activates WIFI_PROTOCOL_LR.
};

/// Packet source information.
struct Source {
  roo_io::MacAddress addr;
};

/// Callback type for received payloads.
using ReceiverFn =
    std::function<void(const Source&, const void* data, size_t len)>;

/// Initializes communication subsystem.
void Begin(Mode mode);

/// Registers receive callback.
void SetReceiverFn(ReceiverFn receiver_fn);

/// Shuts down the communication subsystem.
void End();

/// Returns current Wi-Fi channel.
uint8_t GetWiFiChannel();

/// Sets the Wi-Fi channel (call after Wi-Fi initialization).
void SetWiFiChannel(uint8_t channel);

/// Sends the packet to the specified destination and waits until it gets
/// delivered to the recipient ESP service. Usually takes a few ms. Returns true
/// if delivery was successful; false otherwise.
bool Send(const roo_io::MacAddress& addr, const void* data, size_t len);

/// Sends the packet to the specified destination without waiting until it gets
/// delivered to the recipient ESP service. Returns true if message was
/// successfully enqueued for send; false otherwise.
bool SendAsync(const roo_io::MacAddress& addr, const void* data, size_t len);

/// Broadcasts the packet.
void BroadcastAsync(const void* data, size_t len);

class EspNowTransport;

/// Returns the global ESP-NOW transport instance.
EspNowTransport& Transport();

}  // namespace roo_comms
