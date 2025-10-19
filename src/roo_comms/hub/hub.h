#pragma once

#include <vector>

#include "roo_collections/flat_small_hash_map.h"
#include "roo_comms/hub/hub_device_factory.h"
#include "roo_comms/pairing.h"
#include "roo_comms/transport/receiver.h"
#include "roo_io/net/mac_address.h"
#include "roo_prefs.h"
#include "roo_transceivers/universe.h"

namespace roo_comms {

class Hub : public roo_transceivers::Universe {
 public:
  using PairingRequestCb =
      std::function<void(const roo_transceivers::DeviceLocator&,
                         const roo_transceivers_Descriptor&)>;

  Hub(roo_scheduler::Scheduler& scheduler, HubDeviceFactory& device_factory,
      PairingRequestCb pairing_request_cb);

  // For testing.
  Hub(EspNowTransport& transport, roo_scheduler::Scheduler& scheduler,
      HubDeviceFactory& device_factory, PairingRequestCb pairing_request_cb);

  void init(uint8_t channel);

  void onDataRecv(const Source& source, const void* data, size_t len);

  size_t deviceCount() const { return transceiver_addresses_.size(); }

  const roo_io::MacAddress& device(size_t idx) const {
    return transceiver_addresses_[idx];
  }

  HubDevice* lookupDevice(const roo_io::MacAddress& addr) const {
    auto itr = devices_.find(addr);
    return (itr != devices_.end()) ? itr->second.get() : nullptr;
  }

  bool forEachDevice(std::function<bool(const roo_transceivers::DeviceLocator&)>
                         callback) const override;

  bool getDeviceDescriptor(
      const roo_transceivers::DeviceLocator& locator,
      roo_transceivers_Descriptor& descriptor) const override;

  roo_transceivers::Measurement read(
      const roo_transceivers::SensorLocator& locator) const override;

  bool write(const roo_transceivers::ActuatorLocator& locator,
             float value) override;

  void requestUpdate() override;

  void addEventListener(roo_transceivers::EventListener* listener) override;

  void removeEventListener(roo_transceivers::EventListener* listener) override;

  void approvePairing(const roo_transceivers::DeviceLocator& locator);

 private:
  void processMessage(const roo_comms::Receiver::Message& received);

  void processDiscoveryRequest(const roo_io::MacAddress& origin,
                               const roo_comms_DeviceDescriptor& descriptor);

  void processPairingRequest(const roo_io::MacAddress& origin,
                             const roo_comms_DeviceDescriptor& descriptor);

  void pair(const roo_io::MacAddress& origin,
            const roo_comms_DeviceDescriptor& descriptor);

  bool checkSupportedType(const roo_comms_DeviceDescriptor& descriptor);

  bool addTransceiver(const roo_io::MacAddress& addr,
                      const roo_comms_DeviceDescriptor& descriptor);

  bool removeTransceiver(const roo_io::MacAddress& addr);

  bool hasTransceiver(const roo_io::MacAddress& addr);

  void writeTransceiverAddresses(roo_prefs::Transaction& t);

  void processDataMessage(const Receiver::Message& msg);

  void notifyTransceiversChanged();
  void notifyNewReadingsAvailable();

  roo_collections::FlatSmallHashSet<roo_transceivers::EventListener*>
      listeners_;

  roo_prefs::Collection store_;
  EspNowTransport& transport_;
  roo_comms::Receiver receiver_;

  HubDeviceFactory& device_factory_;
  roo_collections::FlatSmallHashMap<roo_io::MacAddress,
                                    std::unique_ptr<HubDevice>>
      devices_;

  std::vector<roo_io::MacAddress> transceiver_addresses_;

  struct PendingPairingRequest {
    enum State { kPending, kApproved };
    roo_transceivers_Descriptor descriptor;
    State state;
  };
  int32_t next_pairing_request_id_;

  // Pending pairing requests.
  roo_collections::FlatSmallHashMap<roo_transceivers::DeviceLocator,
                                    PendingPairingRequest>
      pending_pairings_;

  //   // Pending pairing requests that have been approved.
  //   roo_collections::FlatSmallHashMap<roo_transceivers::DeviceLocator,
  //                                     roo_transceivers_Descriptor>
  //       approved_pairings_;

  // Callback to be invoked when we receive a broadcast discovery request. It
  // can be used to start interaction with a human, to check if we should
  // respond to this request or ignore it. When the request gets approved, the
  // `approvePairing` method should be called to complete the pairing process.
  PairingRequestCb pairing_request_cb_;
};

}  // namespace roo_comms