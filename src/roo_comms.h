#pragma once

#include <functional>

#include "roo_io/net/mac_address.h"
#include "comms.pb.h"

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

class EspNowTransport;

EspNowTransport& Transport();

}  // namespace roo_comms
