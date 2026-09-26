#include <array>
#include <cstring>
#include <vector>

#include "gtest/gtest.h"
#include "roo_comms/home_automation.h"
#include "roo_comms/hub/devices/hub_device_environmental_sensor.h"
#include "roo_comms/hub/devices/hub_device_relay.h"
#include "roo_comms/pairing.h"

namespace roo_comms {
namespace {

using roo::comms::ControlMessage;
using roo::comms::DataMessage;
using roo::comms::DeviceDescriptor;
using roo::comms::HomeAutomationDeviceDescriptor;

// Golden protobuf bytes use the original schema's field numbers and wire types.
// The transport prefix remains unchanged for existing devices.
constexpr uint8_t kDataPrefix[] = {'r', 'o', 'o', 0, 0x5e, 0x0c, 0x15, 0x03};
constexpr uint8_t kControlPrefix[] = {'r', 'o', 'o', 0, 0xe1, 0xb2, 0x88, 0x99};

template <typename Message>
void ExpectWire(const Message& message,
                std::initializer_list<uint8_t> expected) {
  uint8_t buffer[Message::kMaxEncodedSize];
  size_t written = 0;
  ASSERT_EQ(roo_pb::Serialize(message, buffer, sizeof(buffer), written),
            roo_pb::Status::kOk);
  EXPECT_EQ(std::vector<uint8_t>(buffer, buffer + written),
            std::vector<uint8_t>(expected));
  Message parsed;
  ASSERT_TRUE(parsed.ParseFromArray(expected.begin(), expected.size()));
  ASSERT_EQ(roo_pb::Serialize(parsed, buffer, sizeof(buffer), written),
            roo_pb::Status::kOk);
  EXPECT_EQ(std::vector<uint8_t>(buffer, buffer + written),
            std::vector<uint8_t>(expected));
}

TEST(Protocol, RelayWireAndFraming) {
  DataMessage msg;
  msg.mutable_relay_request();
  ExpectWire(msg, {0x32, 0});  // Explicitly selected empty request.
  msg.mutable_relay_request()->set_mask(1);
  msg.mutable_relay_request()->set_write(1);
  ExpectWire(msg, {0x32, 10, 0x0d, 1, 0, 0, 0, 0x15, 1, 0, 0, 0});
  auto packet = SerializeHomeAutomationDataMessage(msg);
  ASSERT_EQ(packet.size, size_t{20});
  EXPECT_EQ(std::memcmp(packet.data, kDataPrefix, 8), 0);
  DataMessage parsed;
  ASSERT_TRUE(
      TryParsingAsHomeAutomationDataMessage(packet.data, packet.size, parsed));
  EXPECT_EQ(parsed.relay_request().write(), 1u);
  msg.mutable_relay_response()->set_state(0x80000001);
  ExpectWire(msg, {0x3a, 5, 0x0d, 1, 0, 0, 0x80});
}

TEST(Protocol, DiscoveryPairingAndStoredDescriptorWire) {
  HomeAutomationDeviceDescriptor home;
  home.mutable_relay()->set_port_count(2);
  ExpectWire(home, {0x12, 2, 8, 2});
  auto descriptor = BuildHomeAutomationDescriptor(home);
  ExpectWire(descriptor, {8, 1, 0x12, 4, 0x12, 2, 8, 2});
  HomeAutomationDeviceDescriptor parsed;
  ASSERT_TRUE(TryParseHomeAutomationDescriptor(descriptor, parsed));
  EXPECT_EQ(parsed.relay().port_count(), 2);
  ControlMessage msg;
  *msg.mutable_hub_discovery_request()->mutable_device_descriptor() =
      descriptor;
  ExpectWire(msg, {0x0a, 10, 0x0a, 8, 8, 1, 0x12, 4, 0x12, 2, 8, 2});
  *msg.mutable_hub_pairing_request()->mutable_device_descriptor() = descriptor;
  ExpectWire(msg, {0x1a, 10, 0x0a, 8, 8, 1, 0x12, 4, 0x12, 2, 8, 2});
  msg.mutable_hub_discovery_response()->set_hub_channel(6);
  ExpectWire(msg, {0x12, 2, 8, 6});
  msg.mutable_hub_pairing_response();
  ExpectWire(msg, {0x22, 0});
  msg.mutable_hub_pairing_response()->set_status(
      ControlMessage::HubPairingResponse::Status::kRejected);
  ExpectWire(msg, {0x22, 2, 8, 1});
  uint8_t packet[] = {'r', 'o', 'o', 0, 0xe1, 0xb2, 0x88, 0x99, 0x12, 2, 8, 6};
  ASSERT_TRUE(TryParsingAsControlMessage(packet, sizeof(packet), msg));
  EXPECT_EQ(msg.hub_discovery_response().hub_channel(), 6u);
}

TEST(Protocol, SensorPresenceAndZeroValues) {
  HomeAutomationDeviceDescriptor home;
  home.mutable_environmental_sensor()->set_has_aht20(true);
  home.mutable_environmental_sensor()->set_has_bmp280(true);
  ExpectWire(home, {0x0a, 4, 8, 1, 0x10, 1});
  DataMessage msg;
  msg.mutable_environmental_sensor_readings()
      ->mutable_aht20()
      ->set_temperature_celsius(0);
  ExpectWire(msg, {0x2a, 7, 0x12, 5, 0x0d, 0, 0, 0, 0});
  auto packet = SerializeHomeAutomationDataMessage(msg);
  DataMessage parsed;
  ASSERT_TRUE(
      TryParsingAsHomeAutomationDataMessage(packet.data, packet.size, parsed));
  const auto& readings = parsed.environmental_sensor_readings();
  EXPECT_TRUE(readings.has_aht20());
  EXPECT_TRUE(readings.aht20().has_temperature_celsius());
  EXPECT_FALSE(readings.aht20().has_humidity_percent());
  EXPECT_FALSE(readings.has_bmp280());
}

TEST(Protocol, DescriptorCapacity) {
  DeviceDescriptor descriptor;
  std::array<char, 129> payload{};
  EXPECT_TRUE(descriptor.try_set_payload(payload.data(), 128));
  EXPECT_FALSE(descriptor.try_set_payload(payload.data(), 129));
  uint8_t buffer[DeviceDescriptor::kMaxEncodedSize];
  size_t written;
  ASSERT_EQ(roo_pb::Serialize(descriptor, buffer, sizeof(buffer), written),
            roo_pb::Status::kOk);
  DeviceDescriptor parsed;
  ASSERT_TRUE(parsed.ParseFromArray(buffer, written));
  EXPECT_EQ(parsed.payload().size(), size_t{128});
  std::vector<uint8_t> oversized{0x12, 0x81, 1};
  oversized.resize(132);
  EXPECT_FALSE(parsed.ParseFromArray(oversized.data(), oversized.size()));
}

TEST(Protocol, HubDescriptorsUseRooPbAndReplaceExistingContents) {
  EspNowTransport transport;
  roo_io::MacAddress address(0x02, 0, 0, 0, 0, 1);
  HubDeviceRelay relay(transport, address, 2);
  roo_transceivers::Descriptor descriptor;
  relay.getDescriptor(descriptor);
  ASSERT_EQ(descriptor.sensors_size(), 2u);
  ASSERT_EQ(descriptor.actuators_size(), 2u);
  EXPECT_STREQ(descriptor.sensors(0).id().c_str(), "relay_1");
  EXPECT_STREQ(descriptor.actuators(1).id().c_str(), "relay_2");
  EXPECT_EQ(descriptor.sensors(0).quantity(),
            roo_transceivers::Quantity::kBinaryState);
  HomeAutomationDeviceDescriptor::EnvironmentalSensor sensor_descriptor;
  sensor_descriptor.set_has_aht20(true);
  sensor_descriptor.set_has_bmp280(true);
  HubDeviceEnvironmentalSensor sensor(transport, address, sensor_descriptor);
  sensor.getDescriptor(descriptor);
  ASSERT_EQ(descriptor.sensors_size(), 4u);
  EXPECT_EQ(descriptor.actuators_size(), 0u);
  EXPECT_STREQ(descriptor.sensors(0).id().c_str(), "aht20_temperature");
  EXPECT_STREQ(descriptor.sensors(3).id().c_str(), "bmp280_pressure");
  EXPECT_EQ(descriptor.sensors(3).quantity(),
            roo_transceivers::Quantity::kPressure);
  sensor.getDescriptor(descriptor);
  EXPECT_EQ(descriptor.sensors_size(), 4u);
}

TEST(Protocol, RejectsShortWrongAndMalformedPackets) {
  DataMessage data;
  ControlMessage control;
  EXPECT_FALSE(TryParsingAsHomeAutomationDataMessage(nullptr, 0, data));
  EXPECT_FALSE(TryParsingAsControlMessage(nullptr, 0, control));
  for (size_t length = 1; length < 8; ++length) {
    // Allocate only the supplied length so sanitizers catch prefix overreads.
    std::vector<uint8_t> packet(kDataPrefix, kDataPrefix + length);
    EXPECT_FALSE(
        TryParsingAsHomeAutomationDataMessage(packet.data(), length, data));
    packet.assign(kControlPrefix, kControlPrefix + length);
    EXPECT_FALSE(TryParsingAsControlMessage(packet.data(), length, control));
  }
  EXPECT_FALSE(TryParsingAsControlMessage(kDataPrefix, 8, control));
  EXPECT_FALSE(TryParsingAsHomeAutomationDataMessage(kControlPrefix, 8, data));
  std::vector<uint8_t> packet(kDataPrefix, kDataPrefix + 8);
  packet.insert(packet.end(), {0x32, 5, 0x0d});
  EXPECT_FALSE(TryParsingAsHomeAutomationDataMessage(packet.data(),
                                                     packet.size(), data));
  std::memcpy(packet.data(), kControlPrefix, 8);
  EXPECT_FALSE(
      TryParsingAsControlMessage(packet.data(), packet.size(), control));
}

}  // namespace
}  // namespace roo_comms
