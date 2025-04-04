#include "roo_comms/device/pairable_device.h"

namespace roo_comms {

Button::Button(roo_control::BinarySelector& selector,
               roo_scheduler::Scheduler& scheduler,
               std::function<void(bool)> pressed)
    : roo_control::PushButton(selector),
      updater_(
          scheduler, [this]() { tick(); }, roo_time::Millis(10)),
      pressed_(pressed) {}

RgbLedSignaler::RgbLedSignaler(roo_led::RgbLed& led,
                               roo_scheduler::Scheduler& scheduler)
    : led_(led), blinker_(led, scheduler) {}

void RgbLedSignaler::turnOff() { blinker_.turnOff(); }

void RgbLedSignaler::signalPaired() {
  blinker_.repeat(roo_led::RgbBlink(Millis(400), roo_led::Color(0, 255, 0), 50),
                  3);
}

void RgbLedSignaler::signalUnpaired() {
  blinker_.loop(
      roo_led::RgbBlink(Millis(500), roo_led::Color(255, 0, 255), 20));
}

void RgbLedSignaler::signalPairing() {
  blinker_.loop(roo_led::RgbBlink(Millis(200), roo_led::Color(0, 0, 255), 50));
}

MonochromeLedSignaler::MonochromeLedSignaler(
    roo_led::MonochromeLed& led, roo_scheduler::Scheduler& scheduler,
    uint16_t normal_mode_intensity)
    : led_(led),
      blinker_(led, scheduler),
      normal_mode_intensity_(normal_mode_intensity) {}

void MonochromeLedSignaler::turnOff() { blinker_.turnOff(); }

void MonochromeLedSignaler::signalPaired() {
  blinker_.repeat(roo_led::Blink(Millis(1000), 50, 30, 30), 2,
                  normal_mode_intensity_);
}

void MonochromeLedSignaler::signalUnpaired() {
  blinker_.loop(roo_led::Blink(Millis(2000), 35, 0, 0));
}

void MonochromeLedSignaler::signalPairing() {
  blinker_.loop(roo_led::Blink(Millis(500), 50, 0, 100));
}

PairableDevice::PairableDevice(
    EspNowTransport& transport,
    const roo_comms_DeviceDescriptor* device_descriptor,
    roo_prefs::Collection& prefs, roo_control::BinarySelector& button,
    StateSignaler& signaler, roo_scheduler::Scheduler& scheduler,
    std::function<void(State prev_state, State new_state)> on_state_changed,
    std::function<void(const roo_comms::Receiver::Message&)> on_app_data_recv)
    : transport_(transport),
      device_descriptor_(device_descriptor),
      prefs_(prefs),
      signaler_(signaler),
      on_state_changed_(on_state_changed),
      on_app_data_recv_(on_app_data_recv),
      peer_id_(prefs_, "peer", 0),
      peer_channel_(prefs_, "channel", 0),
      broadcaster_(scheduler, [this]() { cycleChannelAndBroadcast(); }),
      pairing_canceler_(scheduler, [this]() { setState(kNotPaired); }),
      unpairer_(scheduler,
                [this]() {
                  LOG(INFO) << "Unpairing";
                  setState(kNotPaired);
                }),
      receiver_(
          scheduler,
          [this](const roo_comms::Receiver::Message& msg) {
            processMessage(msg);
          },
          100, 8, 256, nullptr),
      state_(kStartup),
      button_(button, scheduler, [this](bool is_long_pressed) {
        buttonPressed(is_long_pressed);
      }) {}

void PairableDevice::begin(bool wakeup) {
  if (wakeup) {
    state_ = kSleep;
  }
  button_.start();
  restoreState();
}

void PairableDevice::restoreState() {
  roo_io::MacAddress peer_addr = getPeerAddress();
  if (peer_addr.asU64() != 0) {
    transport_.setChannel(peer_channel_.get());
    initPeer(peer_addr);
    setState(kPaired);
  } else {
    setState(kNotPaired);
  }
}

void PairableDevice::onDataRecv(const uint8_t* source_mac_addr,
                                const uint8_t* incomingData, int len) {
  receiver_.handle(source_mac_addr, incomingData, len);
}

void PairableDevice::buttonPressed(bool is_long_press) {
  LOG(INFO) << "Button pressed: " << is_long_press;
  switch (state_) {
    // case kStartup:
    case kNotPaired:
    case kPaired: {
      if (is_long_press) {
        setState(kPairing);
      }
      break;
    }
    case kPairing:
    case kAwaitingPairingConfirmation: {
      if (is_long_press) {
        clearPeerAddress();
      }
      restoreState();
      break;
    }
    default: {
      // Ignore.
    }
  }
}

void PairableDevice::setState(State new_state) {
  LOG(INFO) << "Setting state to " << new_state << " (currently is " << state_
            << ")";
  if (state_ == new_state) return;
  signaler_.turnOff();
  LOG(ERROR) << "Signaler turned off";
  switch (state_) {
    case kStartup:
    case kSleep: {
      break;
    }
    case kNotPaired: {
      break;
    }
    case kPairing: {
      broadcaster_.cancel();
      break;
    }
    case kAwaitingPairingConfirmation: {
      pairing_canceler_.cancel();
      break;
    }
    case kPaired: {
      peer_.reset();
      break;
    }
  }
  switch (new_state) {
    case kStartup: {
      break;
    }
    case kNotPaired: {
      LOG(INFO) << "Signaling unpaired";
      signaler_.signalUnpaired();
      LOG(INFO) << "Signaled unpaired";
      break;
    }
    case kPairing: {
      signaler_.signalPairing();
      broadcaster_.scheduleNow();
      break;
    }
    case kAwaitingPairingConfirmation: {
      signaler_.signalPairing();
      pairing_canceler_.scheduleAfter(roo_time::Seconds(10));
      break;
    }
    case kPaired: {
      if (state_ != kSleep) {
        signaler_.signalPaired();
      }
      LOG(INFO) << "Paired with " << getPeerAddress().asString();
      // unpairer.scheduleAfter(roo_time::Seconds(20));
      break;
    }
    case kSleep: {
      break;
    }
  }
  State prev_state = state_;
  state_ = new_state;
  LOG(INFO) << "Calling state change notifier";
  if (on_state_changed_ != nullptr) {
    on_state_changed_(prev_state, state_);
  }
}

void PairableDevice::sendBroadcastAnnounceMessage() {
  SendDiscoveryRequest(transport_, *device_descriptor_);
}

void PairableDevice::initPeer(const roo_io::MacAddress& peer_addr) {
  peer_.reset(new EspNowPeer(transport_, peer_addr));
}

void PairableDevice::sendPairingRequestMessage() {
  SendPairingRequest(*peer_, *device_descriptor_);
}

void PairableDevice::processMessage(const roo_comms::Receiver::Message& msg) {
  {
    roo_comms_ControlMessage control_msg;
    if (TryParsingAsControlMessage((const uint8_t*)msg.data.get(), msg.size,
                                   control_msg)) {
      switch (control_msg.which_contents) {
        case roo_comms_ControlMessage_hub_discovery_response_tag: {
          if (state_ != kPairing) {
            LOG(ERROR) << "Received pairing invitation, but we're not asking.";
            break;
          }
          int channel = control_msg.contents.hub_discovery_response.hub_channel;
          LOG(INFO) << "Received broadcast announce response with channel "
                    << channel;
          transport_.setChannel(channel);
          initPeer(msg.source);
          sendPairingRequestMessage();
          setState(kAwaitingPairingConfirmation);
          break;
        }
        case roo_comms_ControlMessage_hub_pairing_response_tag: {
          // TODO: verify that this came with the hub we asked.
          if (state_ != kAwaitingPairingConfirmation) {
            LOG(ERROR)
                << "Received paiting confirmation, but we're not pairing.";
            break;
          }
          setPeer(transport_.channel(), msg.source);
          setState(kPaired);
          break;
        }
        case roo_comms_ControlMessage_hub_discovery_request_tag:
        case roo_comms_ControlMessage_hub_pairing_request_tag:
        default: {
          // Ignoring.
          return;
        } break;
      }
      return;
    }
  }
  // Hand off all other messages to the application layer.
  if (on_app_data_recv_ != nullptr) {
    on_app_data_recv_(msg);
  }
}

void PairableDevice::cycleChannelAndBroadcast() {
  uint8_t channel = transport_.channel() + 1;
  if (channel >= 15) channel = 1;
  transport_.setChannel(channel);
  sendBroadcastAnnounceMessage();

  // // Now, wait 1s for pairing invitation.
  broadcaster_.scheduleAfter(roo_time::Seconds(1));
}

roo_io::MacAddress PairableDevice::getPeerAddress() {
  roo_io::MacAddress addr;
  addr.assignFromU64(peer_id_.get());
  return addr;
}

void PairableDevice::clearPeerAddress() { peer_id_.clear(); }

void PairableDevice::setPeer(int channel, const roo_io::MacAddress& addr) {
  CHECK(peer_channel_.set(channel));
  CHECK(peer_id_.set(addr.asU64()));
}

}  // namespace roo_comms