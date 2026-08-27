#include "Arduino.h"
#include "WiFi.h"
#include "roo_comms.h"
#include "roo_comms/home_automation.h"

#ifdef ROO_TESTING
#include "roo_testing/buses/esp_now/fake_esp_now.h"
#include "roo_testing/microcontrollers/esp32/fake_esp32.h"
#endif

namespace {

// Must match kWiFiChannel in relay_controller.
constexpr uint8_t kWiFiChannel = 1;
constexpr int kRelayIndex = 0;

// The parser expects at least the eight-byte roo_comms protocol identifier.
constexpr size_t kProtocolPrefixSize = 8;

#ifdef ROO_TESTING

// -- EMULATOR SETUP
//
// This fake controller injects requests into the emulated ESP32 and receives
// the responses sent by the production relay code below.

class FakeRelayController : public FakeEspNowDevice {
 public:
  FakeRelayController() : FakeEspNowDevice(0x020000000001ULL) {}

  void requestState() {
    Serial.println("[emulator controller] Requesting relay state");
    request(0, 0);
  }

  void writeRelay(int relay_idx, bool enabled) {
    Serial.printf("[emulator controller] Asking relay %d to turn %s\n",
                  relay_idx, enabled ? "on" : "off");
    const uint32_t bit = uint32_t{1} << relay_idx;
    request(bit, enabled ? bit : 0);
  }

  void send(const void* data, size_t len) override {
    roo_comms_DataMessage response;
    if (len < kProtocolPrefixSize ||
        !roo_comms::TryParsingAsHomeAutomationDataMessage(
            static_cast<const uint8_t*>(data), len, response) ||
        response.which_contents != roo_comms_DataMessage_relay_response_tag) {
      Serial.println(
          "[emulator controller] Ignoring an invalid relay response");
      return;
    }

    Serial.printf(
        "[emulator controller] Relay reports state 0x%08lX\n",
        static_cast<unsigned long>(response.contents.relay_response.state));
  }

 private:
  void request(uint32_t mask, uint32_t write) {
    roo_comms_DataMessage request = roo_comms_DataMessage_init_zero;
    request.which_contents = roo_comms_DataMessage_relay_request_tag;
    request.contents.relay_request.mask = mask;
    request.contents.relay_request.write = write;
    auto serialized = roo_comms::SerializeHomeAutomationDataMessage(request);
    if (serialized.size > 0) {
      respond(serialized.data, serialized.size);
    }
  }
};

struct Emulator {
  FakeRelayController controller;

  Emulator() { FakeEsp32().attachEspNowDevice(controller); }
} emulator;

// -- END EMULATOR SETUP

#endif  // ROO_TESTING

uint32_t relay_state = 0;

void OnMessageReceived(const roo_comms::Source& source, const void* data,
                       size_t len) {
  roo_comms_DataMessage request;
  if (len < kProtocolPrefixSize ||
      !roo_comms::TryParsingAsHomeAutomationDataMessage(
          static_cast<const uint8_t*>(data), len, request)) {
    Serial.println("Relay: received an invalid roo_comms message");
    return;
  }
  if (request.which_contents != roo_comms_DataMessage_relay_request_tag) {
    Serial.println("Relay: received a message that is not a relay request");
    return;
  }

  const uint32_t mask = request.contents.relay_request.mask;
  const uint32_t write = request.contents.relay_request.write;
  relay_state = (relay_state & ~mask) | (write & mask);

  const std::string address = source.addr.asString();
  Serial.printf("Relay: request from %s; state is 0x%08lX\n", address.c_str(),
                static_cast<unsigned long>(relay_state));

  roo_comms_DataMessage response = roo_comms_DataMessage_init_zero;
  response.which_contents = roo_comms_DataMessage_relay_response_tag;
  response.contents.relay_response.state = relay_state;
  auto serialized = roo_comms::SerializeHomeAutomationDataMessage(response);
  if (serialized.size == 0 ||
      !roo_comms::SendAsync(source.addr, serialized.data, serialized.size)) {
    Serial.println("Relay: failed to enqueue the response");
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);

  roo_comms::Begin(roo_comms::kNormalMode);
  roo_comms::SetWiFiChannel(kWiFiChannel);
  roo_comms::SetReceiverFn(OnMessageReceived);

  Serial.printf("Relay: ready on channel %u; device MAC is %s\n", kWiFiChannel,
                WiFi.macAddress().c_str());
}

void loop() {
#ifdef ROO_TESTING
  // Exercise the real relay callback with a read, followed by alternating
  // writes. On hardware, relay_controller supplies these requests instead.
  static bool first_request = true;
  static bool enable_next = true;
  if (first_request) {
    emulator.controller.requestState();
    first_request = false;
  } else {
    emulator.controller.writeRelay(kRelayIndex, enable_next);
    enable_next = !enable_next;
  }
#endif

  delay(2000);
}
