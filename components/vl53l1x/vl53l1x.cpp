#include "vl53l1x.h"

namespace esphome {
namespace vl53l1x {

bool VL53L1X::xshut_prepared_ = false;
bool VL53L1X::wire_initialized_ = false;
uint8_t VL53L1X::wire_sda_pin_ = 255;
uint8_t VL53L1X::wire_scl_pin_ = 255;
uint32_t VL53L1X::wire_i2c_frequency_ = 0;

std::vector<VL53L1X *> &VL53L1X::instances_() {
  static std::vector<VL53L1X *> instances;
  return instances;
}

VL53L1X::VL53L1X() { instances_().push_back(this); }

void VL53L1X::dump_config() {
  ESP_LOGCONFIG(TAG, "VL53L1X:");
  ESP_LOGCONFIG(TAG, "  Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  SDA Pin: %u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: %u", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  Bus Frequency: %u Hz", this->i2c_frequency_);
  if (this->ranging_mode != nullptr) {
    ESP_LOGCONFIG(TAG, "  Ranging: %s", this->ranging_mode->name);
  }
  if (offset.has_value()) {
    ESP_LOGCONFIG(TAG, "  Offset: %dmm", this->offset.value());
  }
  if (xtalk.has_value()) {
    ESP_LOGCONFIG(TAG, "  XTalk: %dcps", this->xtalk.value());
  }
  if (this->interrupt_pin.has_value()) {
    LOG_PIN("  Interrupt Pin: ", this->interrupt_pin.value());
  }
  if (this->xshut_pin.has_value()) {
    LOG_PIN("  XShut Pin: ", this->xshut_pin.value());
  }
}

bool VL53L1X::validate_multi_sensor_config_() {
  auto &instances = instances_();
  if (instances.size() <= 1) {
    return true;
  }

  for (auto *instance : instances) {
    if (!instance->xshut_pin.has_value()) {
      ESP_LOGE(TAG, "Multiple VL53L1X sensors require an xshut pin on every sensor");
      instance->mark_failed();
      return false;
    }
    if (instance->address_ == 0x29) {
      ESP_LOGE(TAG, "Multiple VL53L1X sensors require a unique non-default I2C address on every sensor");
      instance->mark_failed();
      return false;
    }
    if (instance->sda_pin_ != this->sda_pin_ || instance->scl_pin_ != this->scl_pin_) {
      ESP_LOGE(TAG, "Multiple VL53L1X sensors must share the same SDA/SCL pins");
      instance->mark_failed();
      return false;
    }
    if (instance->i2c_frequency_ != this->i2c_frequency_) {
      ESP_LOGE(TAG, "Multiple VL53L1X sensors must share the same I2C frequency");
      instance->mark_failed();
      return false;
    }
  }

  for (size_t i = 0; i < instances.size(); i++) {
    for (size_t j = i + 1; j < instances.size(); j++) {
      if (instances[i]->address_ == instances[j]->address_) {
        ESP_LOGE(TAG, "Duplicate VL53L1X I2C address 0x%02X configured", instances[i]->address_);
        instances[i]->mark_failed();
        instances[j]->mark_failed();
        return false;
      }
    }
  }

  return true;
}

void VL53L1X::prepare_xshut_pins_() {
  if (xshut_prepared_) {
    return;
  }

  for (auto *instance : instances_()) {
    if (!instance->xshut_pin.has_value()) {
      continue;
    }
    auto *pin = instance->xshut_pin.value();
    pin->setup();
    pin->pin_mode(gpio::FLAG_OUTPUT);
    pin->digital_write(false);
  }

  delay(10);
  xshut_prepared_ = true;
}

bool VL53L1X::initialize_wire_() {
  if (wire_initialized_) {
    if (wire_sda_pin_ != this->sda_pin_ || wire_scl_pin_ != this->scl_pin_ || wire_i2c_frequency_ != this->i2c_frequency_) {
      ESP_LOGE(TAG, "Wire bus already initialized with different SDA/SCL/frequency settings");
      this->mark_failed();
      return false;
    }
    return true;
  }

  // ESPHome's ESP32 build initializes an IDF I2C bus first. On this hybrid
  // Arduino+IDF runtime, calling Wire.begin() while the bus is already active
  // can skip buffer allocation, which later causes NULL TX/RX buffer errors.
  Wire.end();
  delay(1);
  if (!Wire.begin(this->sda_pin_, this->scl_pin_, this->i2c_frequency_)) {
    ESP_LOGE(TAG, "Failed to initialize Arduino Wire on SDA=%u SCL=%u", this->sda_pin_, this->scl_pin_);
    this->mark_failed();
    return false;
  }

  wire_sda_pin_ = this->sda_pin_;
  wire_scl_pin_ = this->scl_pin_;
  wire_i2c_frequency_ = this->i2c_frequency_;
  wire_initialized_ = true;
  ESP_LOGI(TAG, "Initialized Arduino Wire on SDA=%u SCL=%u @ %u Hz", this->sda_pin_, this->scl_pin_,
           this->i2c_frequency_);
  return true;
}

void VL53L1X::enable_sensor_() {
  if (!this->xshut_pin.has_value()) {
    return;
  }

  auto *pin = this->xshut_pin.value();
  pin->setup();
  pin->pin_mode(gpio::FLAG_OUTPUT);
  pin->digital_write(true);
  delay(10);
}

void VL53L1X::setup() {
  ESP_LOGD(TAG, "Beginning setup");

  if (this->sda_pin_ == 255 || this->scl_pin_ == 255) {
    ESP_LOGE(TAG, "I2C pins were not configured for Arduino Wire");
    this->mark_failed();
    return;
  }

  if (!this->validate_multi_sensor_config_()) {
    return;
  }

  this->prepare_xshut_pins_();
  if (!this->initialize_wire_()) {
    return;
  }
  this->enable_sensor_();

  auto status = this->init();
  if (status != VL53L1_ERROR_NONE) {
    this->mark_failed();
    return;
  }
  ESP_LOGD(TAG, "Device initialized");

  if (this->offset.has_value()) {
    ESP_LOGI(TAG, "Setting offset calibration to %d", this->offset.value());
    status = this->sensor.SetOffsetInMm(this->offset.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set offset calibration, error code: %d", status);
      this->mark_failed();
      return;
    }
  }

  if (this->xtalk.has_value()) {
    ESP_LOGI(TAG, "Setting crosstalk calibration to %d", this->xtalk.value());
    status = this->sensor.SetXTalk(this->xtalk.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set crosstalk calibration, error code: %d", status);
      this->mark_failed();
      return;
    }
  }

  ESP_LOGI(TAG, "Setup complete");
}

VL53L1_Error VL53L1X::init() {
  ESP_LOGD(TAG, "Trying to initialize");

  // Use the ULD wrapper's boot-and-init sequence first. This matches the
  // standalone examples in this repo and has proven more reliable on recent
  // ESPHome/ESP32 builds than manually polling boot state here.
  auto status = sensor.Begin();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not initialize device, error code: %d", status);
    return status;
  }

  // Multi-sensor setups still need each sensor moved away from the default
  // address after it has booted and initialized on its own.
  if (address_ != 0x29) {
    ESP_LOGD(TAG, "Setting different address");
    status = sensor.SetI2CAddress(address_ << 1);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Failed to change address. Error: %d", status);
      return status;
    }
    delay(5);
  }

  return status;
}

VL53L1_Error VL53L1X::wait_for_boot() {
  // Wait for firmware to copy NVM device_state into registers
  delayMicroseconds(1200);

  uint8_t device_state;
  VL53L1_Error status;
  auto start = millis();
  while ((millis() - start) < this->timeout) {
    status = get_device_state(&device_state);
    if (status != VL53L1_ERROR_NONE) {
      return status;
    }
    if ((device_state & 0x01) == 0x01) {
      ESP_LOGD(TAG, "Finished waiting for boot. Device state: %d", device_state);
      return VL53L1_ERROR_NONE;
    }
    App.feed_wdt();
  }

  ESP_LOGW(TAG, "Timed out waiting for boot. state: %d", device_state);
  return VL53L1_ERROR_TIME_OUT;
}

VL53L1_Error VL53L1X::get_device_state(uint8_t *device_state) {
  VL53L1_Error status = sensor.GetBootState(device_state);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to read device state. error: %d", status);
    return status;
  }

  // Our own logic...device_state is 255 when unable to complete read
  // Not sure why and why other libraries don't account for this.
  // Maybe somehow this is supposed to be 0, and it is getting messed up in I2C layer.
  if (*device_state == 255) {
    *device_state = 98;  // Unknown
  }

  ESP_LOGV(TAG, "Device state: %d", *device_state);

  return VL53L1_ERROR_NONE;
}

void VL53L1X::set_ranging_mode(const RangingMode *mode) {
  if (this->is_failed()) {
    ESP_LOGE(TAG, "Cannot set ranging mode while component is failed");
    return;
  }

  auto status = this->sensor.SetDistanceMode(mode->mode);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set distance mode: %d, error code: %d", mode->mode, status);
  }

  status = this->sensor.SetTimingBudgetInMs(mode->timing_budget);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set timing budget: %d ms, error code: %d", mode->timing_budget, status);
  }

  status = this->sensor.SetInterMeasurementInMs(mode->delay_between_measurements);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not set measurement delay: %d ms, error code: %d", mode->delay_between_measurements, status);
  }

  this->ranging_mode = mode;
  ESP_LOGI(TAG, "Set ranging mode: %s", mode->name);
}

optional<uint16_t> VL53L1X::read_distance(ROI *roi, VL53L1_Error &status) {
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Cannot read distance while component is failed");
    return {};
  }

  ESP_LOGVV(TAG, "Beginning distance read");

  if (last_roi == nullptr || *roi != *last_roi) {
    ESP_LOGVV(TAG, "Setting new ROI: { width: %d, height: %d, center: %d }", roi->width, roi->height, roi->center);

    status = this->sensor.SetROI(roi->width, roi->height);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI width/height, error code: %d", status);
      return {};
    }
    status = this->sensor.SetROICenter(roi->center);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set ROI center, error code: %d", status);
      return {};
    }
    last_roi = roi;
  }

  status = this->sensor.StartRanging();

  // Wait for the measurement to be ready
  // TODO use interrupt_pin, if given, to await data ready instead of polling
  uint8_t dataReady = false;
  while (!dataReady) {
    status = this->sensor.CheckForDataReady(&dataReady);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Failed to check if data is ready, error code: %d", status);
      return {};
    }
    delay(1);
    App.feed_wdt();
  }

  // Get the results
  uint16_t distance;
  status = this->sensor.GetDistanceInMm(&distance);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not get distance, error code: %d", status);
    return {};
  }

  // After reading the results reset the interrupt to be able to take another measurement
  status = this->sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not clear interrupt, error code: %d", status);
    return {};
  }
  status = this->sensor.StopRanging();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not stop ranging, error code: %d", status);
    return {};
  }

  ESP_LOGV(TAG, "Finished distance read: %d", distance);
  return {distance};
}

}  // namespace vl53l1x
}  // namespace esphome
