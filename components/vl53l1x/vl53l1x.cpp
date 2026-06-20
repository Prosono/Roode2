#include "vl53l1x.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace esphome {
namespace vl53l1x {

namespace {

SemaphoreHandle_t wire_mutex = nullptr;
constexpr uint8_t SENSOR_INIT_ATTEMPTS = 3;
constexpr uint32_t SENSOR_RECOVERY_SETTLE_MS = 25;

bool lock_wire_bus(uint32_t timeout_ms) {
  if (wire_mutex == nullptr) {
    wire_mutex = xSemaphoreCreateRecursiveMutex();
    if (wire_mutex == nullptr) {
      ESP_LOGE(TAG, "Failed to create Wire mutex");
      return false;
    }
  }

  if (xSemaphoreTakeRecursive(wire_mutex, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
    ESP_LOGW(TAG, "Timed out waiting for Wire mutex");
    return false;
  }

  return true;
}

void unlock_wire_bus() {
  if (wire_mutex != nullptr) {
    xSemaphoreGiveRecursive(wire_mutex);
  }
}

class ScopedWireLock {
 public:
  explicit ScopedWireLock(uint32_t timeout_ms) : locked_(lock_wire_bus(timeout_ms)) {}
  ~ScopedWireLock() {
    if (locked_) {
      unlock_wire_bus();
    }
  }
  bool locked() const { return locked_; }

 protected:
  bool locked_{false};
};

}  // namespace

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
  ScopedWireLock lock(2000);
  if (!lock.locked()) {
    this->mark_failed();
    return false;
  }

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

void VL53L1X::disable_sensor_() {
  if (!this->xshut_pin.has_value()) {
    return;
  }

  auto *pin = this->xshut_pin.value();
  pin->setup();
  pin->pin_mode(gpio::FLAG_OUTPUT);
  pin->digital_write(false);
  delay(10);
}

bool VL53L1X::apply_post_init_settings_() {
  if (this->offset.has_value()) {
    ScopedWireLock lock(2000);
    if (!lock.locked()) {
      return false;
    }
    ESP_LOGI(TAG, "Setting offset calibration to %d", this->offset.value());
    auto status = this->sensor.SetOffsetInMm(this->offset.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set offset calibration, error code: %d", status);
      return false;
    }
  }

  if (this->xtalk.has_value()) {
    ScopedWireLock lock(2000);
    if (!lock.locked()) {
      return false;
    }
    ESP_LOGI(TAG, "Setting crosstalk calibration to %d", this->xtalk.value());
    auto status = this->sensor.SetXTalk(this->xtalk.value());
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Could not set crosstalk calibration, error code: %d", status);
      return false;
    }
  }

  const RangingMode *mode = this->ranging_mode_override.has_value() ? this->ranging_mode_override.value() : this->ranging_mode;
  if (mode != nullptr) {
    this->set_ranging_mode(mode);
  }
  return true;
}

bool VL53L1X::recover_sensor_(const char *reason, bool power_cycle) {
  ESP_LOGW(TAG, "Recovering sensor 0x%02X after %s", this->address_, reason);

  this->setup_complete_ = false;
  this->last_roi = nullptr;

  if (power_cycle) {
    this->disable_sensor_();
    delay(SENSOR_RECOVERY_SETTLE_MS);
    this->enable_sensor_();
    delay(SENSOR_RECOVERY_SETTLE_MS);
  }

  if (!this->initialize_wire_()) {
    return false;
  }

  const auto status = this->init();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Sensor recovery failed during init, error code: %d", status);
    return false;
  }

  this->setup_complete_ = true;
  if (!this->apply_post_init_settings_()) {
    ESP_LOGE(TAG, "Sensor recovery failed while restoring settings");
    this->setup_complete_ = false;
    return false;
  }

  ESP_LOGI(TAG, "Sensor 0x%02X recovered successfully", this->address_);
  return true;
}

void VL53L1X::setup() {
  this->setup_complete_ = false;
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
  bool initialized = false;
  for (uint8_t attempt = 0; attempt < SENSOR_INIT_ATTEMPTS; attempt++) {
    if (attempt == 0) {
      this->enable_sensor_();
    } else {
      this->disable_sensor_();
      delay(SENSOR_RECOVERY_SETTLE_MS);
      this->enable_sensor_();
    }

    auto status = this->init();
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGW(TAG, "Init attempt %u/%u failed for sensor 0x%02X with error %d",
               static_cast<unsigned>(attempt + 1), static_cast<unsigned>(SENSOR_INIT_ATTEMPTS), this->address_, status);
      delay(SENSOR_RECOVERY_SETTLE_MS);
      continue;
    }

    this->setup_complete_ = true;
    if (!this->apply_post_init_settings_()) {
      ESP_LOGW(TAG, "Post-init setup attempt %u/%u failed for sensor 0x%02X",
               static_cast<unsigned>(attempt + 1), static_cast<unsigned>(SENSOR_INIT_ATTEMPTS), this->address_);
      this->setup_complete_ = false;
      delay(SENSOR_RECOVERY_SETTLE_MS);
      continue;
    }

    initialized = true;
    break;
  }

  if (!initialized) {
    this->mark_failed();
    return;
  }

  ESP_LOGI(TAG, "Setup complete");
}

VL53L1_Error VL53L1X::init() {
  ScopedWireLock lock(2000);
  if (!lock.locked()) {
    return VL53L1_ERROR_TIME_OUT;
  }

  ESP_LOGD(TAG, "Trying to initialize");

  VL53L1_Error status;

  // If address is non-default, set and try again.
  if (address_ != (sensor.GetI2CAddress() >> 1)) {
    ESP_LOGD(TAG, "Setting different address");
    status = sensor.SetI2CAddress(address_ << 1);
    if (status != VL53L1_ERROR_NONE) {
      ESP_LOGE(TAG, "Failed to change address. Error: %d", status);
      return status;
    }
  }

  status = wait_for_boot();
  if (status != VL53L1_ERROR_NONE) {
    return status;
  }

  ESP_LOGD(TAG, "Found device, initializing...");
  status = sensor.Init();
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Could not initialize device, error code: %d", status);
    return status;
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

  if (!this->setup_complete_) {
    ESP_LOGE(TAG, "Cannot set ranging mode before VL53L1X setup is complete");
    return;
  }

  ScopedWireLock lock(2000);
  if (!lock.locked()) {
    ESP_LOGE(TAG, "Cannot set ranging mode because Wire bus is busy");
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

optional<uint16_t> VL53L1X::read_distance_once_(ROI *roi, VL53L1_Error &status) {
  if (this->is_failed()) {
    ESP_LOGW(TAG, "Cannot read distance while component is failed");
    return {};
  }

  if (!this->setup_complete_) {
    ESP_LOGW(TAG, "Cannot read distance before VL53L1X setup is complete");
    status = VL53L1_ERROR_TIME_OUT;
    return {};
  }

  ScopedWireLock lock(2000);
  if (!lock.locked()) {
    status = VL53L1_ERROR_TIME_OUT;
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

optional<uint16_t> VL53L1X::read_distance(ROI *roi, VL53L1_Error &status) {
  auto result = this->read_distance_once_(roi, status);
  if (result.has_value()) {
    return result;
  }

  if (!this->recover_sensor_("read failure", true)) {
    this->mark_failed();
    return {};
  }

  return this->read_distance_once_(roi, status);
}

}  // namespace vl53l1x
}  // namespace esphome
