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
    const roo_comms_HomeAutomationDeviceDescriptor_EnvironmentalSensor&
        descriptor)
    : HubDevice(transport, destination),
      descriptor_(descriptor),
      state_{},
      last_reading_(roo_time::Uptime::Start()) {}

void HubDeviceEnvironmentalSensor::getDescriptor(
    roo_transceivers_Descriptor& result) const {
  result.actuators_count = 0;
  size_t sensor_idx = 0;
  if (descriptor_.has_aht20) {
    strcpy(result.sensors[sensor_idx].id, kAht20Temperature);
    result.sensors[sensor_idx].quantity =
        roo_transceivers_Quantity_kTemperature;
    sensor_idx++;
    strcpy(result.sensors[sensor_idx].id, kAht20Humidity);
    result.sensors[sensor_idx].quantity =
        roo_transceivers_Quantity_kAirHumidity;
    sensor_idx++;
  }
  if (descriptor_.has_bmp280) {
    strcpy(result.sensors[sensor_idx].id, kBmp280Temperature);
    result.sensors[sensor_idx].quantity =
        roo_transceivers_Quantity_kTemperature;
    sensor_idx++;
    strcpy(result.sensors[sensor_idx].id, kBmp280Pressure);
    result.sensors[sensor_idx].quantity = roo_transceivers_Quantity_kPressure;
    sensor_idx++;
  }
  result.sensors_count = sensor_idx;
}

roo_transceivers::Measurement HubDeviceEnvironmentalSensor::read(
    const roo_transceivers::SensorId& sensor_id) const {
  if (sensor_id == kAht20Temperature) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kTemperature, last_reading_,
        state_.has_aht20 && state_.aht20.has_temperature_celsius
            ? state_.aht20.temperature_celsius
            : nanf(""));
  } else if (sensor_id == kAht20Humidity) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kAirHumidity, last_reading_,
        state_.has_aht20 && state_.aht20.has_humidity_percent
            ? state_.aht20.humidity_percent
            : nanf(""));
  } else if (sensor_id == kBmp280Temperature) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kTemperature, last_reading_,
        state_.has_bmp280 && state_.bmp280.has_temperature_celsius
            ? state_.bmp280.temperature_celsius
            : nanf(""));
  } else if (sensor_id == kBmp280Pressure) {
    return roo_transceivers::Measurement(
        roo_transceivers_Quantity_kPressure, last_reading_,
        state_.has_bmp280 && state_.bmp280.has_pressure_pa
            ? state_.bmp280.pressure_pa
            : nanf(""));
  } else {
    return roo_transceivers::Measurement();
  }
}

void HubDeviceEnvironmentalSensor::updateState(const uint8_t* data,
                                               size_t len) {
  roo_comms_DataMessage data_message;
  if (!TryParsingAsHomeAutomationDataMessage(data, len, data_message)) {
    LOG(WARNING) << "Failed to parse data message";
    return;
  }
  if (data_message.which_contents !=
      roo_comms_DataMessage_environmental_sensor_readings_tag) {
    LOG(WARNING) << "Ignoring non-relay response message";
    return;
  }
  state_ = data_message.contents.environmental_sensor_readings;
  last_reading_ = roo_time::Uptime::Now();
}

}  // namespace roo_comms
