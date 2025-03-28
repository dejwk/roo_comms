#pragma once

#include <vector>

#include "esp_now_transport.h"
#include "roo_backport.h"
#include "roo_collections/flat_small_hash_map.h"
#include "roo_io/net/mac_address.h"
#include "roo_prefs.h"
#include "roo_transceivers/universe.h"

namespace roo_comms {

class Hub {
 public:
  using PayloadCb = std::function<void(const roo_io::MacAddress &,
                                       const roo_comms_DataMessage &)>;
  using TransceiverChangedCb = std::function<void()>;

  Hub(roo_scheduler::Scheduler &scheduler, PayloadCb payload_cb,
      TransceiverChangedCb transceiver_changed_cb);

  void init(uint8_t channel);

  void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

  size_t deviceCount() const { return transceiver_addresses_.size(); }

  const roo_io::MacAddress &device(size_t idx) const {
    return transceiver_addresses_[idx];
  }

  const roo_comms_DeviceDescriptor *lookupDescriptor(
      const roo_io::MacAddress &addr) const;

  roo_comms_DeviceDescriptor *lookupDescriptor(const roo_io::MacAddress &addr);

  const EspNowTransport& transport() const { return transport_; }

 private:
  friend class TransceiverHub;

  void processMessage(roo_comms::ReceivedMessage received);

  void processDiscoveryRequest(const roo_io::MacAddress &origin,
                               const roo_comms_DeviceDescriptor &descriptor);

  void processPairingRequest(const roo_io::MacAddress &origin,
                             const roo_comms_DeviceDescriptor &descriptor);

  bool checkSupportedType(pb_size_t which_kind);

  bool addTransceiver(const roo_io::MacAddress &addr,
                      const roo_comms_DeviceDescriptor &descriptor);

  bool removeTransceiver(const roo_io::MacAddress &addr);

  bool hasTransceiver(const roo_io::MacAddress &addr);

  void writeTransceiverAddresses(roo_prefs::Transaction &t);

  roo_prefs::Collection store_;
  EspNowTransport transport_;
  roo_comms::Receiver receiver_;

  PayloadCb payload_cb_;
  TransceiverChangedCb transceiver_changed_cb_;

  std::vector<roo_io::MacAddress> transceiver_addresses_;

  roo_collections::FlatSmallHashMap<roo_io::MacAddress,
                                    roo_comms_DeviceDescriptor>
      transceiver_details_;

  //   std::function<void(std::string)> send_notify_;
};

struct DeviceState {
  roo_time::Uptime last_reading;
  roo_comms_DataMessage last_payload;
};

class TransceiverHub : public roo_transceivers::Universe {
 public:
  TransceiverHub(roo_scheduler::Scheduler &scheduler);

  void init(uint8_t channel) { hub_.init(channel); }

  void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len) {
    hub_.onDataRecv(mac, incomingData, len);
  }

  size_t deviceCount() const override;

  bool forEachDevice(
      std::function<bool(const roo_transceivers::DeviceLocator &)> callback)
      const override;

  bool getDeviceDescriptor(
      const roo_transceivers::DeviceLocator &locator,
      roo_transceivers_Descriptor &descriptor) const override;

  roo_transceivers::Measurement read(
      const roo_transceivers::SensorLocator &locator) const override;

  bool write(const roo_transceivers::ActuatorLocator &locator,
             float value) const override;

  void requestUpdate() override;

  void addEventListener(roo_transceivers::EventListener *listener) override;

  void removeEventListener(roo_transceivers::EventListener *listener) override;

  void setRelay(int idx, bool is_enabled);

 private:
  roo_transceivers::DeviceLocator device(size_t device_idx) const;

  void processDataMessage(const roo_io::MacAddress &src,
                          const roo_comms_DataMessage &msg);

  void notifyTransceiversChanged();
  void notifyNewReadingsAvailable();

  Hub hub_;
  roo_collections::FlatSmallHashMap<roo_io::MacAddress, DeviceState> states_;

  roo_collections::FlatSmallHashSet<roo_transceivers::EventListener *>
      listeners_;
};

}