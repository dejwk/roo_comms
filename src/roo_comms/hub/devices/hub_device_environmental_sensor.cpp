#include "roo_comms/hub/devices/hub_device_environmental_sensor.h"

#include <cstring>

#include "roo_comms/home_automation.h"

namespace roo_comms {

namespace {

static const char* kAht20Temperature = "aht20_temperature";
static const char* kAht20Humidity = "aht20_humidity";
static const char* kBmp280Temperature = "bmp280_temperature";
static const char* kBmp280Pressure = "bmp280_pressure";
}  // namespace

HubDeviceEnvironmentalSensor::HubDeviceEnvironmentalSensor(
    EspNowTransport& transport, const roo_io::MacAddress& destination,
    const roo::comms::HomeAutomationDeviceDescriptor_EnvironmentalSensor&
        descriptor)
    : HubDevice(transport, destination),
      descriptor_(descriptor),
      state_{},
      last_reading_(roo_time::Uptime::Start()) {}

void HubDeviceEnvironmentalSensor::getDescriptor(
    roo_transceivers::Descriptor& result) const {
  result.Clear();
  auto add_sensor = [&result](const char* id,
                              roo_transceivers::Quantity quantity) {
    auto* sensor = result.add_sensors();
    sensor->set_id(id);
    sensor->set_quantity(quantity);
  };
  if (descriptor_.has_aht20()) {
    add_sensor(kAht20Temperature, roo_transceivers::Quantity::kTemperature);
    add_sensor(kAht20Humidity, roo_transceivers::Quantity::kAirHumidity);
  }
  if (descriptor_.has_bmp280()) {
    add_sensor(kBmp280Temperature, roo_transceivers::Quantity::kTemperature);
    add_sensor(kBmp280Pressure, roo_transceivers::Quantity::kPressure);
  }
}

roo_transceivers::Measurement HubDeviceEnvironmentalSensor::read(
    const roo_transceivers::SensorId& sensor_id) const {
  if (sensor_id == kAht20Temperature) {
    return roo_transceivers::Measurement(
        roo_transceivers::Quantity::kTemperature, last_reading_,
        state_.has_aht20() && state_.aht20().has_temperature_celsius()
            ? state_.aht20().temperature_celsius()
            : nanf(""));
  } else if (sensor_id == kAht20Humidity) {
    return roo_transceivers::Measurement(
        roo_transceivers::Quantity::kAirHumidity, last_reading_,
        state_.has_aht20() && state_.aht20().has_humidity_percent()
            ? state_.aht20().humidity_percent()
            : nanf(""));
  } else if (sensor_id == kBmp280Temperature) {
    return roo_transceivers::Measurement(
        roo_transceivers::Quantity::kTemperature, last_reading_,
        state_.has_bmp280() && state_.bmp280().has_temperature_celsius()
            ? state_.bmp280().temperature_celsius()
            : nanf(""));
  } else if (sensor_id == kBmp280Pressure) {
    return roo_transceivers::Measurement(
        roo_transceivers::Quantity::kPressure, last_reading_,
        state_.has_bmp280() && state_.bmp280().has_pressure_pa()
            ? state_.bmp280().pressure_pa()
            : nanf(""));
  } else {
    return roo_transceivers::Measurement();
  }
}

void HubDeviceEnvironmentalSensor::updateState(const uint8_t* data,
                                               size_t len) {
  roo::comms::DataMessage data_message;
  if (!TryParsingAsHomeAutomationDataMessage(data, len, data_message)) {
    LOG(WARNING) << "Failed to parse data message";
    return;
  }
  if (data_message.contents_case() !=
      roo::comms::DataMessage::ContentsCase::kEnvironmentalSensorReadings) {
    LOG(WARNING) << "Ignoring non-relay response message";
    return;
  }
  state_ = data_message.environmental_sensor_readings();
  last_reading_ = roo_time::Uptime::Now();
}

}  // namespace roo_comms
