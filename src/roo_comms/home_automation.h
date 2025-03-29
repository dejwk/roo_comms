#pragma once

#include "esp_now_transport.h"

namespace roo_comms {

bool TryParsingAsHomeAutomationDataMessage(const uint8_t* incoming_data,
                                           size_t len,
                                           roo_comms_DataMessage& msg);

struct SerializedHomeAutomationDataMessage {
  pb_byte_t data[8 + roo_comms_DataMessage_size];
  size_t size;
};

SerializedHomeAutomationDataMessage SerializeHomeAutomationDataMessage(
    const roo_comms_DataMessage& msg);

bool RequestRelayState(EspNowTransport& transport,
                       const roo_io::MacAddress& device);

bool WriteRelay(EspNowTransport& transport, const roo_io::MacAddress& device,
                int relay_idx, bool is_enabled);

roo_comms_DeviceDescriptor BuildHomeAutomationDescriptor(
    const roo_comms_HomeAutomationDeviceDescriptor& input);

bool TryParseHomeAutomationDescriptor(
    const roo_comms_DeviceDescriptor& input,
    roo_comms_HomeAutomationDeviceDescriptor& result);

}  // namespace roo_comms