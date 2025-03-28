#pragma once

#include "esp_now_transport.h"

namespace roo_comms {

bool TryParsingAsControlMessage(const uint8_t* incoming_data, size_t len,
                                roo_comms_ControlMessage& msg);

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