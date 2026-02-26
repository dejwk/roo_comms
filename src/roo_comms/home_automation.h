#pragma once

#include "roo_comms/transport/esp_now_transport.h"

namespace roo_comms {

/// Parses a payload as a home-automation data message.
bool TryParsingAsHomeAutomationDataMessage(const uint8_t* incoming_data,
                                           size_t len,
                                           roo_comms_DataMessage& msg);

struct SerializedHomeAutomationDataMessage {
  pb_byte_t data[8 + roo_comms_DataMessage_size];
  size_t size;
};

/// Serializes a home-automation data message into a raw buffer.
SerializedHomeAutomationDataMessage SerializeHomeAutomationDataMessage(
    const roo_comms_DataMessage& msg);

/// Requests relay state from a device.
bool RequestRelayState(EspNowTransport& transport,
                       const roo_io::MacAddress& device);

/// Writes a relay state on a device.
bool WriteRelay(EspNowTransport& transport, const roo_io::MacAddress& device,
                int relay_idx, bool is_enabled);

/// Builds a generic device descriptor from home-automation specifics.
roo_comms_DeviceDescriptor BuildHomeAutomationDescriptor(
    const roo_comms_HomeAutomationDeviceDescriptor& input);

/// Parses a home-automation descriptor from a generic descriptor.
bool TryParseHomeAutomationDescriptor(
    const roo_comms_DeviceDescriptor& input,
    roo_comms_HomeAutomationDeviceDescriptor& result);

}  // namespace roo_comms