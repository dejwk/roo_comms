#ifndef PAIRABLE_DEVICE_H_
#define PAIRABLE_DEVICE_H_

#include <inttypes.h>

#include <functional>
#include <memory>

#include "esp_err.h"
// #include "esp_now.h"
#include "roo_blink.h"
#include "roo_blink/monochrome/blinker.h"
#include "roo_blink/monochrome/led.h"
#include "roo_blink/rgb/blinker.h"
#include "roo_blink/rgb/led.h"
#include "roo_comms/pairing.h"
#include "roo_comms/transport/receiver.h"
#include "roo_control.h"
#include "roo_control/selector/push_button.h"
#include "roo_control/selector/selector.h"
#include "roo_io.h"
#include "roo_io/net/mac_address.h"
#include "roo_prefs.h"
#include "roo_scheduler.h"

using roo_time::Millis;

namespace roo_comms {

/// Push-button helper that invokes a callback on click/long-press.
class Button : public roo_control::PushButton {
 public:
  Button(roo_control::BinarySelector& selector,
         roo_scheduler::Scheduler& scheduler,
         std::function<void(bool is_long_pressed)> pressed);

  /// Starts periodic polling of the button.
  void start() { updater_.start(); }

 protected:
  void onClick() override { pressed_(false); }
  void onLongPress() override { pressed_(true); }

 private:
  roo_scheduler::RepetitiveTask updater_;
  std::function<void(bool)> pressed_;
};

/// Device-side pairing state machine and peer management.
class PairableDevice {
 public:
  /// Pairing state.
  enum State {
    kStartup = 0,
    kNotPaired = 1,
    kPairing = 2,
    kAwaitingPairingConfirmation = 3,
    kPaired = 4,
    kSleep = 5,
  };

  /// Initial state when starting up.
  enum InitialState {
    kColdStart = 0,  // Regular.
    kWakeup = 1,     // Coming back from deep sleep.
  };

  /// Interface for signaling pairing state to the user.
  class StateSignaler {
   public:
    virtual ~StateSignaler() = default;

    virtual void turnOff() = 0;

    virtual void signalPaired() = 0;

    virtual void signalUnpaired() = 0;

    virtual void signalPairing() = 0;
  };

  PairableDevice(
      const roo_comms_DeviceDescriptor* device_descriptor,
      roo_prefs::Collection& prefs, roo_control::BinarySelector& button,
      StateSignaler& signaler, roo_scheduler::Scheduler& scheduler,
      std::function<void(State prev_state, State new_state)> on_state_changed,
      std::function<void(const roo_comms::Receiver::Message&)>
          on_app_data_recv);

  // For testing.
  PairableDevice(
      EspNowTransport& transport,
      const roo_comms_DeviceDescriptor* device_descriptor,
      roo_prefs::Collection& prefs, roo_control::BinarySelector& button,
      StateSignaler& signaler, roo_scheduler::Scheduler& scheduler,
      std::function<void(State prev_state, State new_state)> on_state_changed,
      std::function<void(const roo_comms::Receiver::Message&)>
          on_app_data_recv);

  void begin(InitialState initial_state = kColdStart);

  /// Should be called instead of begin() when coming back from deep sleep.
  void wakeup();

  State state() const { return state_; }

  bool isPaired() const { return state() == kPaired; }

  EspNowPeer* peer() { return peer_.get(); }

  void onDataRecv(const Source& source, const void* data, size_t len);

 private:
  void buttonPressed(bool is_long_press);

  void setState(State new_state);

  // Sets the state to whatever was persisted (either paired or unpaired).
  void restoreState();

  void sendBroadcastAnnounceMessage();

  void initPeer(const roo_io::MacAddress& peer_addr);

  void sendPairingRequestMessage();

  void processMessage(const roo_comms::Receiver::Message& msg);

  void cycleChannelAndBroadcast();

  roo_io::MacAddress getPeerAddress();

  void clearPeerAddress();

  void setPeer(int channel, const roo_io::MacAddress& addr);

  EspNowTransport& transport_;

  const roo_comms_DeviceDescriptor* device_descriptor_;

  roo_prefs::Collection& prefs_;
  StateSignaler& signaler_;

  std::function<void(State prev_state, State new_state)> on_state_changed_;
  std::function<void(const roo_comms::Receiver::Message&)> on_app_data_recv_;

  roo_prefs::Uint64 peer_id_;
  roo_prefs::Uint8 peer_channel_;
  std::unique_ptr<EspNowPeer> peer_;

  roo_scheduler::SingletonTask broadcaster_;
  roo_scheduler::SingletonTask pairing_canceler_;
  roo_scheduler::SingletonTask unpairer_;

  Receiver receiver_;

  State state_;

  Button button_;
};

/// RGB LED-based signaler for pairing state.
class RgbLedSignaler : public PairableDevice::StateSignaler {
 public:
  RgbLedSignaler(roo_blink::RgbLed& led, roo_scheduler::Scheduler& scheduler);

  void turnOff() override;

  void signalPaired() override;

  void signalUnpaired() override;

  void signalPairing() override;

 private:
  roo_blink::RgbLed& led_;
  roo_blink::RgbBlinker blinker_;
};

/// Monochrome LED-based signaler for pairing state.
class MonochromeLedSignaler : public PairableDevice::StateSignaler {
 public:
  MonochromeLedSignaler(roo_blink::Led& led,
                        roo_scheduler::Scheduler& scheduler,
                        uint16_t normal_mode_intensity = 0);

  void turnOff() override;

  void signalPaired() override;

  void signalUnpaired() override;

  void signalPairing() override;

 private:
  roo_blink::Led& led_;
  roo_blink::Blinker blinker_;

  // The intensity of light in 'normal' mode (paired, active).
  uint16_t normal_mode_intensity_;
};

}  // namespace roo_comms

#endif  // PAIRABLE_DEVICE_H_