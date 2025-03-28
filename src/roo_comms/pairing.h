#pragma once

#include "esp_now_transport.h"

namespace roo_comms {

static constexpr roo_io::byte kControlMagic[8] = {
    roo_io::byte{'r'},  roo_io::byte{'o'},  roo_io::byte{'o'},
    roo_io::byte{0},    roo_io::byte{0xE1}, roo_io::byte{0xB2},
    roo_io::byte{0x88}, roo_io::byte{0x99}};

struct SerializedControlMessage {
  pb_byte_t data[8 + roo_comms_ControlMessage_size];
  size_t size;
};

SerializedControlMessage SerializeControlMessage(
    const roo_comms_ControlMessage& msg, const Magic& magic);

// Sends a broadcast discovery request.
void SendDiscoveryRequest(EspNowTransport& transport,
                          const roo_comms_DeviceDescriptor& device_descriptor);

// Sends a pairing request.
void SendPairingRequest(EspNowPeer& peer,
                        const roo_comms_DeviceDescriptor& device_descriptor);

// Sends a an ack to a discovery request.
void SendDiscoveryResponse(EspNowTransport& transport,
                           const roo_io::MacAddress& origin);

// Sends a an ack to a pairing request.
void SendPairingResponse(EspNowTransport& transport,
                         const roo_io::MacAddress& origin);

}  // namespace roo_comms