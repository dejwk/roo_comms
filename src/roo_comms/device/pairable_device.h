#ifndef PAIRABLE_DEVICE_H_
#define PAIRABLE_DEVICE_H_

#include <inttypes.h>

#include <functional>
#include <memory>

#include "esp_err.h"
// #include "esp_now.h"
#include "esp_now_transport.h"
#include "roo_comms/pairing.h"
#include "roo_control/selector/push_button.h"
#include "roo_control/selector/selector.h"
#include "roo_io/net/mac_address.h"
#include "roo_led/monochrome/blinker.h"
#include "roo_led/monochrome/led.h"
#include "roo_led/rgb/blinker.h"
#include "roo_led/rgb/led.h"
#include "roo_prefs.h"
#include "roo_scheduler.h"

using roo_time::Millis;

namespace roo_comms {

class Button : public roo_control::PushButton {
 public:
  Button(roo_control::BinarySelector& selector,
         roo_scheduler::Scheduler& scheduler,
         std::function<void()> long_pressed);

  void start() { updater_.start(); }

 protected:
  void onLongPress() override { long_pressed_(); }

 private:
  roo_scheduler::RepetitiveTask updater_;
  std::function<void()> long_pressed_;
};

class PairableDevice {
 public:
  enum State {
    kStartup = 0,
    kNotPaired = 1,
    kPairing = 2,
    kAwaitingPairingConfirmation = 3,
    kPaired = 4,
    kSleep = 5,
  };

  class StateSignaler {
   public:
    virtual ~StateSignaler() = default;

    virtual void turnOff() = 0;

    virtual void signalPaired() = 0;

    virtual void signalUnpaired() = 0;

    virtual void signalPairing() = 0;
  };

  PairableDevice(
      EspNowTransport& transport,
      const roo_comms_DeviceDescriptor* device_descriptor,
      roo_prefs::Collection& prefs, roo_control::BinarySelector& button,
      StateSignaler& signaler, roo_scheduler::Scheduler& scheduler,
      std::function<void(State prev_state, State new_state)> on_state_changed,
      std::function<void(const roo_comms::Receiver::Message&)>
          on_app_data_recv);

  void begin(bool wakeup);

  State state() const { return state_; }

  bool isPaired() const { return state() == kPaired; }

  EspNowPeer* peer() { return peer_.get(); }

  void onDataRecv(const uint8_t* source_mac_addr, const uint8_t* incomingData,
                  int len);

 private:
  void setState(State new_state);

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

class RgbLedSignaler : public PairableDevice::StateSignaler {
 public:
  RgbLedSignaler(roo_led::RgbLed& led, roo_scheduler::Scheduler& scheduler);

  void turnOff() override;

  void signalPaired() override;

  void signalUnpaired() override;

  void signalPairing() override;

 private:
  roo_led::RgbLed& led_;
  roo_led::RgbBlinker blinker_;
};

class MonochromeLedSignaler : public PairableDevice::StateSignaler {
 public:
  MonochromeLedSignaler(roo_led::MonochromeLed& led,
                        roo_scheduler::Scheduler& scheduler,
                        uint16_t normal_mode_intensity = 0);

  void turnOff() override;

  void signalPaired() override;

  void signalUnpaired() override;

  void signalPairing() override;

 private:
  roo_led::MonochromeLed& led_;
  roo_led::Blinker blinker_;

  // The intensity of light in 'normal' mode (paired, active).
  uint16_t normal_mode_intensity_;
};

}  // namespace roo_comms

#endif  // PAIRABLE_DEVICE_H_