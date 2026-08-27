#include "Arduino.h"
#include "roo_comms.h"
#include "roo_comms/home_automation.h"
#include "roo_io/net/mac_address.h"

#ifdef ROO_TESTING
#include "roo_testing/buses/esp_now/fake_esp_now.h"
#include "roo_testing/microcontrollers/esp32/fake_esp32.h"
#endif

namespace {

// Both physical boards must use the same Wi-Fi channel.
constexpr uint8_t kWiFiChannel = 1;
constexpr int kRelayIndex = 0;

// On real hardware, replace this with the address printed by relay_device.
// The default is the address of the fake relay used by the host emulator.
const roo_io::MacAddress kRelayAddress(0x02, 0x00, 0x00, 0x00, 0x00, 0x02);

// The parser expects at least the eight-byte roo_comms protocol identifier.
constexpr size_t kProtocolPrefixSize = 8;

#ifdef ROO_TESTING

// -- EMULATOR SETUP
//
// roo_testing emulates one ESP32, not two independent ESP-NOW stacks. A
// FakeEspNowDevice represents the relay at kRelayAddress. The production code
// below still sends through roo_comms and receives through its regular
// callback; only the peer is simulated.

class FakeRelayDevice : public FakeEspNowDevice {
 public:
  FakeRelayDevice() : FakeEspNowDevice(0x020000000002ULL) {}

  void send(const void* data, size_t len) override {
    roo_comms_DataMessage request;
    if (len < kProtocolPrefixSize ||
        !roo_comms::TryParsingAsHomeAutomationDataMessage(
            static_cast<const uint8_t*>(data), len, request) ||
        request.which_contents != roo_comms_DataMessage_relay_request_tag) {
      Serial.println("[emulator relay] Ignoring an invalid relay request");
      return;
    }

    const uint32_t mask = request.contents.relay_request.mask;
    const uint32_t write = request.contents.relay_request.write;
    state_ = (state_ & ~mask) | (write & mask);

    Serial.printf("[emulator relay] State is now 0x%08lX\n",
                  static_cast<unsigned long>(state_));

    roo_comms_DataMessage response = roo_comms_DataMessage_init_zero;
    response.which_contents = roo_comms_DataMessage_relay_response_tag;
    response.contents.relay_response.state = state_;
    auto serialized = roo_comms::SerializeHomeAutomationDataMessage(response);
    if (serialized.size > 0) {
      respond(serialized.data, serialized.size);
    }
  }

 private:
  uint32_t state_ = 0;
};

struct Emulator {
  FakeRelayDevice relay;

  Emulator() { FakeEsp32().attachEspNowDevice(relay); }
} emulator;

// -- END EMULATOR SETUP

#endif  // ROO_TESTING

void OnMessageReceived(const roo_comms::Source& source, const void* data,
                       size_t len) {
  roo_comms_DataMessage response;
  if (len < kProtocolPrefixSize ||
      !roo_comms::TryParsingAsHomeAutomationDataMessage(
          static_cast<const uint8_t*>(data), len, response)) {
    Serial.println("Controller: received an invalid roo_comms message");
    return;
  }
  if (response.which_contents != roo_comms_DataMessage_relay_response_tag) {
    Serial.println(
        "Controller: received a message that is not a relay response");
    return;
  }

  const std::string address = source.addr.asString();
  Serial.printf(
      "Controller: relay %s reports state 0x%08lX\n", address.c_str(),
      static_cast<unsigned long>(response.contents.relay_response.state));
}

}  // namespace

void setup() {
  Serial.begin(115200);

  roo_comms::Begin(roo_comms::kNormalMode);
  roo_comms::SetWiFiChannel(kWiFiChannel);
  roo_comms::SetReceiverFn(OnMessageReceived);

  Serial.printf("Controller: requesting initial state from %s on channel %u\n",
                kRelayAddress.asString().c_str(), kWiFiChannel);
  const bool delivered =
      roo_comms::RequestRelayState(roo_comms::Transport(), kRelayAddress);
  Serial.printf("Controller: request delivery %s\n",
                delivered ? "succeeded" : "failed");

  // Delivery success above is the ESP-NOW acknowledgement. The relay's
  // application-level response arrives separately in OnMessageReceived().
  delay(1000);
}

void loop() {
  static bool enable_next = true;

  Serial.printf("Controller: asking relay %d to turn %s\n", kRelayIndex,
                enable_next ? "on" : "off");
  const bool delivered = roo_comms::WriteRelay(
      roo_comms::Transport(), kRelayAddress, kRelayIndex, enable_next);
  Serial.printf("Controller: write delivery %s\n",
                delivered ? "succeeded" : "failed");

  enable_next = !enable_next;
  delay(2000);
}
