#include "ads1100.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

namespace esphome {
namespace ads1100 {

static const char *const TAG = "ads1100";

void ADS1100Component::setup() {
  ESP_LOGCONFIG(TAG, "Setting up ADS1100...");
  uint16_t value;
  if (!this->read_byte_16(0x00, &value)) {
    this->mark_failed();
    return;
  }
  uint8_t config = 0;
  // Clear single-shot bit
  //        0bxxxBxxxx
  config |= 0b00000000;

  // Setup Gain
  //        0bxxxxxxBB
  config |= ADS1100Gain;

  if (this->continuous_mode_) {
    // Set continuous mode
    //        0bxxxBxxxx
    config |= 0b00000000;
  } else {
    // Set singleshot mode
    //        0bxxxBxxxx
    config |= 0b00010000;
  }

  // Set data rate - 860 samples per second (we're in singleshot mode)
  //        0bxxxxBBxx
  config |= ADS1100Rate;

  if (!this->write_byte_8(0x00, config)) {
    this->mark_failed();
    return;
  }
  this->prev_config_ = config;
}
void ADS1100Component::dump_config() {
  ESP_LOGCONFIG(TAG, "Setting up ADS1100...");
  LOG_I2C_DEVICE(this);
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Communication with ADS1100 failed!");
  }

  for (auto *sensor : this->sensors_) {
    LOG_SENSOR("  ", "Sensor", sensor);
    ESP_LOGCONFIG(TAG, "    Rate: %u", sensor->get_rate());
    ESP_LOGCONFIG(TAG, "    Gain: %u", sensor->get_gain());
  }
}
float ADS1100Component::request_measurement(ADS1100Sensor *sensor) {
  uint16_t raw_conversion;
  if (!this->read_byte_16(0x00, &raw_conversion)) {
    this->status_set_warning();
    return NAN;
  }
  auto signed_conversion = static_cast<int16_t>(raw_conversion);

  float millivolts;
  switch (sensor->get_gain()) {
    case ADS1100_GAIN_1:
      millivolts = signed_conversion * 5.859375f;
      break;
    case ADS1100_GAIN_2:
      millivolts = signed_conversion * 2.9296875f;
      break;
    case ADS1100_GAIN_4:
      millivolts = signed_conversion * 1.46484375f;
      break;
    case ADS1100_GAIN_8:
      millivolts = signed_conversion * 0.732421875f;
      break;
    default:
      millivolts = NAN;
  }

  this->status_clear_warning();
  return millivolts;
}

float ADS1100Sensor::sample() { return this->parent_->request_measurement(this); }
void ADS1100Sensor::update() {
  float v = this->parent_->request_measurement(this);
  if (!std::isnan(v)) {
    ESP_LOGD(TAG, "'%s': Got Voltage=%fV", this->get_name().c_str(), v);
    this->publish_state(v);
  }
}

}  // namespace ads1100
}  // namespace esphome
