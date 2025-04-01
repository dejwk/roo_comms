#pragma once

#include <vector>

#include "esp_now_transport.h"
#include "roo_backport.h"
#include "roo_collections/flat_small_hash_map.h"
#include "roo_comms/hub/hub_device_factory.h"
#include "roo_comms/pairing.h"
#include "roo_io/net/mac_address.h"
#include "roo_prefs.h"
#include "roo_transceivers/universe.h"

namespace roo_comms {

class Hub : public roo_transceivers::Universe {
 public:
  using PayloadCb = std::function<void(const roo_comms::Receiver::Message &)>;
  using TransceiverChangedCb = std::function<void()>;

  Hub(EspNowTransport &transport, roo_scheduler::Scheduler &scheduler,
      HubDeviceFactory &device_factory);

  void init(uint8_t channel);

  void onDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len);

  size_t deviceCount() const { return transceiver_addresses_.size(); }

  const roo_io::MacAddress &device(size_t idx) const {
    return transceiver_addresses_[idx];
  }

  HubDevice *lookupDevice(const roo_io::MacAddress &addr) const {
    auto itr = devices_.find(addr);
    return (itr != devices_.end()) ? itr->second.get() : nullptr;
  }

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
  void processMessage(const roo_comms::Receiver::Message &received);

  void processDiscoveryRequest(const roo_io::MacAddress &origin,
                               const roo_comms_DeviceDescriptor &descriptor);

  void processPairingRequest(const roo_io::MacAddress &origin,
                             const roo_comms_DeviceDescriptor &descriptor);

  bool checkSupportedType(const roo_comms_DeviceDescriptor &descriptor);

  bool addTransceiver(const roo_io::MacAddress &addr,
                      const roo_comms_DeviceDescriptor &descriptor);

  bool removeTransceiver(const roo_io::MacAddress &addr);

  bool hasTransceiver(const roo_io::MacAddress &addr);

  void writeTransceiverAddresses(roo_prefs::Transaction &t);

  roo_transceivers::DeviceLocator device_locator(size_t device_idx) const;

  void processDataMessage(const Receiver::Message &msg);

  void notifyTransceiversChanged();
  void notifyNewReadingsAvailable();

  roo_collections::FlatSmallHashSet<roo_transceivers::EventListener *>
      listeners_;

  roo_prefs::Collection store_;
  EspNowTransport &transport_;
  roo_comms::Receiver receiver_;

  HubDeviceFactory &device_factory_;
  roo_collections::FlatSmallHashMap<roo_io::MacAddress,
                                    std::unique_ptr<HubDevice>>
      devices_;

  std::vector<roo_io::MacAddress> transceiver_addresses_;
};

}  // namespace roo_comms