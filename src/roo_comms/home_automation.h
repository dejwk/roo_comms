#pragma once

#include "roo_comms/transport/esp_now_transport.h"

namespace roo_comms {

/// Parses a payload as a home-automation data message.
bool TryParsingAsHomeAutomationDataMessage(const uint8_t* incoming_data,
                                           size_t len,
                                           roo::comms::DataMessage& msg);

struct SerializedHomeAutomationDataMessage {
  uint8_t data[8 + roo::comms::DataMessage::kMaxEncodedSize];
  size_t size;
};

/// Serializes a home-automation data message into a raw buffer.
SerializedHomeAutomationDataMessage SerializeHomeAutomationDataMessage(
    const roo::comms::DataMessage& msg);

/// Requests relay state from a device.
bool RequestRelayState(EspNowTransport& transport,
                       const roo_io::MacAddress& device);

/// Writes a relay state on a device.
bool WriteRelay(EspNowTransport& transport, const roo_io::MacAddress& device,
                int relay_idx, bool is_enabled);

/// Builds a generic device descriptor from home-automation specifics.
roo::comms::DeviceDescriptor BuildHomeAutomationDescriptor(
    const roo::comms::HomeAutomationDeviceDescriptor& input);

/// Parses a home-automation descriptor from a generic descriptor.
bool TryParseHomeAutomationDescriptor(
    const roo::comms::DeviceDescriptor& input,
    roo::comms::HomeAutomationDeviceDescriptor& result);

}  // namespace roo_comms