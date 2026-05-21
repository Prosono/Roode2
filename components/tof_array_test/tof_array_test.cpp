#include "tof_array_test.h"

#include <algorithm>
#include <cmath>
#include <sstream>

namespace esphome {
namespace tof_array_test {

static const char *const TAG = "tof_array_test";

void TofArrayTest::set_roi(uint8_t width, uint8_t height, uint8_t center) {
  this->roi_.width = width;
  this->roi_.height = height;
  this->roi_.center = center;
}

void TofArrayTest::setup() {
  this->prepare_xshut_pins_();
  if (!this->initialize_wire_()) {
    this->mark_failed();
    return;
  }
  this->rediscover();
}

void TofArrayTest::update() {
  if (this->channels_.empty()) {
    ESP_LOGW(TAG, "No active ToF sensors discovered yet");
    return;
  }

  const uint32_t started = millis();
  bool should_resync = false;
  for (auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    if (!this->read_channel_(channel)) {
      should_resync = true;
    }
    App.feed_wdt();
  }
  this->cycle_duration_ms_ = millis() - started;

  if (should_resync) {
    this->recover_wire_();
    this->start_all_ranging_();
  }
}

void TofArrayTest::dump_config() {
  ESP_LOGCONFIG(TAG, "ToF Array Test:");
  ESP_LOGCONFIG(TAG, "  SDA Pin: %u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: %u", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  I2C Frequency: %u Hz", this->i2c_frequency_);
  ESP_LOGCONFIG(TAG, "  Timeout: %u ms", this->timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Base Address: 0x%02X", this->base_address_);
  ESP_LOGCONFIG(TAG, "  Wake Delay: %u ms", this->wake_delay_ms_);
  ESP_LOGCONFIG(TAG, "  Post-Address Delay: %u ms", this->post_address_delay_ms_);
  ESP_LOGCONFIG(TAG, "  Distance Mode: %s", this->distance_mode_ == DISTANCE_MODE_SHORT ? "short" : "long");
  ESP_LOGCONFIG(TAG, "  Timing Budget: %u ms", this->timing_budget_ms_);
  ESP_LOGCONFIG(TAG, "  Intermeasurement: %u ms", this->intermeasurement_ms_);
  ESP_LOGCONFIG(TAG, "  Init Retries: %u", this->init_retries_);
  ESP_LOGCONFIG(TAG, "  ROI: width=%u height=%u center=%u", this->roi_.width, this->roi_.height, this->roi_.center);
  ESP_LOGCONFIG(TAG, "  XSHUT Candidates: %u", static_cast<unsigned>(this->xshut_pins_.size()));
  for (size_t i = 0; i < this->xshut_pins_.size(); i++) {
    ESP_LOGCONFIG(TAG, "    Candidate %u -> GPIO%u", static_cast<unsigned>(i + 1), this->xshut_pin_numbers_[i]);
  }
  ESP_LOGCONFIG(TAG, "  Discovered Sensors: %u", static_cast<unsigned>(this->channels_.size()));
  for (size_t i = 0; i < this->channels_.size(); i++) {
    const auto &channel = this->channels_[i];
    ESP_LOGCONFIG(TAG, "    Sensor %u -> %s at 0x%02X", static_cast<unsigned>(i + 1), channel.source_label.c_str(),
                  channel.address);
  }
}

bool TofArrayTest::initialize_wire_() {
  if (this->wire_initialized_) {
    return true;
  }

  Wire.end();
  delay(1);
  if (!Wire.begin(this->sda_pin_, this->scl_pin_)) {
    ESP_LOGE(TAG, "Failed to initialize Wire on SDA=%u SCL=%u", this->sda_pin_, this->scl_pin_);
    return false;
  }
  Wire.setClock(this->i2c_frequency_);

  this->wire_initialized_ = true;
  ESP_LOGI(TAG, "Initialized Wire on SDA=%u SCL=%u @ %u Hz", this->sda_pin_, this->scl_pin_, this->i2c_frequency_);
  return true;
}

bool TofArrayTest::recover_wire_() {
  this->wire_initialized_ = false;
  ESP_LOGD(TAG, "Recovering Wire bus");
  return this->initialize_wire_();
}

void TofArrayTest::prepare_xshut_pins_() {
  for (auto *pin : this->xshut_pins_) {
    pin->setup();
    pin->pin_mode(gpio::FLAG_OUTPUT);
  }
  this->set_all_xshut_(false);
  delay(this->wake_delay_ms_);
}

void TofArrayTest::set_all_xshut_(bool state) {
  for (auto *pin : this->xshut_pins_) {
    pin->digital_write(state);
  }
}

void TofArrayTest::set_xshut_(size_t index, bool state) {
  if (index >= this->xshut_pins_.size()) {
    return;
  }
  this->xshut_pins_[index]->digital_write(state);
}

bool TofArrayTest::probe_address_(uint8_t address) {
  Wire.beginTransmission(address);
  const uint8_t rc = Wire.endTransmission();
  if (rc == 0) {
    return true;
  }
  ESP_LOGD(TAG, "No ACK at 0x%02X (Wire rc=%u)", address, rc);
  return false;
}

bool TofArrayTest::probe_default_sensor_() {
  return this->probe_address_(0x29);
}

bool TofArrayTest::wait_for_boot_(VL53L1X_ULD &sensor) {
  delayMicroseconds(1200);
  uint8_t device_state = 0;
  auto start = millis();
  while ((millis() - start) < this->timeout_ms_) {
    auto status = sensor.GetBootState(&device_state);
    if (status == VL53L1_ERROR_NONE && (device_state & 0x01) == 0x01) {
      return true;
    }
    if (status != VL53L1_ERROR_NONE) {
      return false;
    }
    delay(1);
    App.feed_wdt();
  }
  return false;
}

bool TofArrayTest::set_temp_address_(VL53L1X_ULD &sensor, uint8_t address) {
  auto status = sensor.SetI2CAddress(address << 1);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to change sensor address to 0x%02X, error=%d", address, status);
    return false;
  }
  return true;
}

bool TofArrayTest::write_register_address_(uint8_t address, uint16_t register_address, uint8_t *wire_rc) {
  Wire.beginTransmission(address);
  Wire.write(static_cast<uint8_t>(register_address >> 8));
  Wire.write(static_cast<uint8_t>(register_address & 0xFF));
  const uint8_t rc = Wire.endTransmission(false);
  if (wire_rc != nullptr) {
    *wire_rc = rc;
  }
  return rc == 0;
}

bool TofArrayTest::read_register8_(uint8_t address, uint16_t register_address, uint8_t &value, uint8_t *wire_rc) {
  uint8_t rc = 0;
  if (!this->write_register_address_(address, register_address, &rc)) {
    if (wire_rc != nullptr) {
      *wire_rc = rc;
    }
    return false;
  }

  const auto count = Wire.requestFrom(static_cast<int>(address), 1);
  if (count != 1) {
    if (wire_rc != nullptr) {
      *wire_rc = rc;
    }
    return false;
  }

  value = Wire.read();
  if (wire_rc != nullptr) {
    *wire_rc = rc;
  }
  return true;
}

bool TofArrayTest::read_register16_(uint8_t address, uint16_t register_address, uint16_t &value, uint8_t *wire_rc) {
  uint8_t rc = 0;
  if (!this->write_register_address_(address, register_address, &rc)) {
    if (wire_rc != nullptr) {
      *wire_rc = rc;
    }
    return false;
  }

  const auto count = Wire.requestFrom(static_cast<int>(address), 2);
  if (count != 2) {
    if (wire_rc != nullptr) {
      *wire_rc = rc;
    }
    return false;
  }

  value = static_cast<uint16_t>(Wire.read()) << 8;
  value |= Wire.read();
  if (wire_rc != nullptr) {
    *wire_rc = rc;
  }
  return true;
}

bool TofArrayTest::capture_identity_(Channel &channel) {
  channel.last_stage = "identity";
  uint8_t rc = 0;
  uint16_t sensor_id = 0;
  uint8_t boot_state = 0;
  uint8_t address_register = 0;

  const bool id_ok = this->read_register16_(channel.address, VL53L1_IDENTIFICATION__MODEL_ID, sensor_id, &rc);
  const bool boot_ok = this->read_register8_(channel.address, VL53L1_FIRMWARE__SYSTEM_STATUS, boot_state, &rc);
  const bool address_ok =
      this->read_register8_(channel.address, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, address_register, &rc);

  if (id_ok) {
    channel.sensor_id = sensor_id;
  }
  if (boot_ok) {
    channel.boot_state = boot_state;
  }
  if (address_ok) {
    channel.address_register = address_register;
  }

  if (!id_ok || !boot_ok || !address_ok) {
    ESP_LOGW(TAG,
             "Identity capture incomplete for %s at 0x%02X (id_ok=%s, boot_ok=%s, addr_ok=%s, wire_rc=%u, id=0x%04X, "
             "fw=0x%02X, addr_reg=0x%02X)",
             channel.source_label.c_str(), channel.address, YESNO(id_ok), YESNO(boot_ok), YESNO(address_ok), rc,
             channel.sensor_id, channel.boot_state, channel.address_register);
    return false;
  }

  ESP_LOGD(TAG, "Identity for %s at 0x%02X -> id=0x%04X fw=0x%02X addr_reg=0x%02X", channel.source_label.c_str(),
           channel.address, channel.sensor_id, channel.boot_state, channel.address_register);
  return true;
}

bool TofArrayTest::configure_sensor_(Channel &channel) {
  auto &sensor = *channel.sensor;
  channel.default_timing_fallback = false;
  channel.last_stage = "boot_wait";
  if (!this->wait_for_boot_(sensor)) {
    channel.last_error = VL53L1_ERROR_TIME_OUT;
    return false;
  }
  this->capture_identity_(channel);

  VL53L1_Error status = VL53L1_ERROR_NONE;
  const uint8_t retries = std::max<uint8_t>(1, this->init_retries_);
  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    if (attempt > 0) {
      delay(this->post_address_delay_ms_);
      if (!this->wait_for_boot_(sensor)) {
        channel.last_error = VL53L1_ERROR_TIME_OUT;
        ESP_LOGW(TAG, "Boot wait retry %u/%u failed for %s at 0x%02X", static_cast<unsigned>(attempt + 1),
                 static_cast<unsigned>(retries), channel.source_label.c_str(), channel.address);
        continue;
      }
    }

    channel.last_stage = "init";
    status = sensor.Init();
    if (status == VL53L1_ERROR_NONE) {
      break;
    }

    channel.last_error = status;
    ESP_LOGW(TAG, "Init attempt %u/%u failed for %s at 0x%02X, error=%d", static_cast<unsigned>(attempt + 1),
             static_cast<unsigned>(retries), channel.source_label.c_str(), channel.address, status);
  }

  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    ESP_LOGE(TAG, "Init failed for %s at 0x%02X after %u attempt(s), error=%d", channel.source_label.c_str(),
             channel.address, static_cast<unsigned>(retries), status);
    return false;
  }
  channel.last_stage = "identity_post_init";
  this->capture_identity_(channel);

  channel.last_stage = "set_roi";
  status = sensor.SetROI(this->roi_.width, this->roi_.height);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    ESP_LOGE(TAG, "SetROI failed for %s at 0x%02X, error=%d", channel.source_label.c_str(), channel.address, status);
    return false;
  }
  channel.last_stage = "set_roi_center";
  status = sensor.SetROICenter(this->roi_.center);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    ESP_LOGE(TAG, "SetROICenter failed for %s at 0x%02X, error=%d", channel.source_label.c_str(), channel.address,
             status);
    return false;
  }

  if (this->distance_mode_ == DISTANCE_MODE_SHORT) {
    channel.last_stage = "set_distance_mode_short";
    status = sensor.SetDistanceMode(EDistanceMode::Short);
    if (status != VL53L1_ERROR_NONE) {
      channel.last_error = status;
      ESP_LOGE(TAG, "SetDistanceMode(short) failed for %s at 0x%02X, error=%d", channel.source_label.c_str(),
               channel.address, status);
      return false;
    }
  } else {
    channel.last_stage = "set_distance_mode_long_default";
    ESP_LOGD(TAG, "Keeping default long distance mode for %s at 0x%02X", channel.source_label.c_str(),
             channel.address);
  }
  const uint16_t min_timing_budget = this->distance_mode_ == DISTANCE_MODE_SHORT ? 20 : 33;
  const uint16_t timing_budget_ms = std::max<uint16_t>(this->timing_budget_ms_, min_timing_budget);
  const uint16_t intermeasurement_ms = std::max<uint16_t>(this->intermeasurement_ms_, timing_budget_ms + 4);

  channel.last_stage = "set_timing_budget";
  status = sensor.SetTimingBudgetInMs(timing_budget_ms);
  if (status != VL53L1_ERROR_NONE) {
    if (this->distance_mode_ == DISTANCE_MODE_LONG) {
      ESP_LOGW(TAG,
               "SetTimingBudgetInMs failed for %s at 0x%02X, error=%d. Falling back to default long-mode timing.",
               channel.source_label.c_str(), channel.address, status);
      channel.default_timing_fallback = true;
    } else {
      channel.last_error = status;
      ESP_LOGE(TAG, "SetTimingBudgetInMs failed for %s at 0x%02X, error=%d", channel.source_label.c_str(),
               channel.address, status);
      return false;
    }
  } else {
    channel.last_stage = "set_intermeasurement";
    status = sensor.SetInterMeasurementInMs(intermeasurement_ms);
    if (status != VL53L1_ERROR_NONE) {
      if (this->distance_mode_ == DISTANCE_MODE_LONG) {
        ESP_LOGW(TAG,
                 "SetInterMeasurementInMs failed for %s at 0x%02X, error=%d. Falling back to default long-mode timing.",
                 channel.source_label.c_str(), channel.address, status);
        channel.default_timing_fallback = true;
      } else {
        channel.last_error = status;
        ESP_LOGE(TAG, "SetInterMeasurementInMs failed for %s at 0x%02X, error=%d", channel.source_label.c_str(),
                 channel.address, status);
        return false;
      }
    }
  }

  channel.initialized = true;
  channel.ranging_started = false;
  channel.last_error = 0;
  channel.consecutive_errors = 0;
  channel.last_stage = channel.default_timing_fallback ? "configured_default_timing" : "configured";
  return true;
}

bool TofArrayTest::start_all_ranging_() {
  uint8_t started_count = 0;

  for (auto &channel : this->channels_) {
    if (!channel.initialized || !channel.sensor) {
      continue;
    }

    auto &sensor = *channel.sensor;
    if (channel.ranging_started) {
      sensor.StopRanging();
      delay(1);
    }

    const auto status = sensor.StartRanging();
    if (status != VL53L1_ERROR_NONE) {
      channel.last_error = status;
      channel.ranging_started = false;
      ESP_LOGW(TAG, "Failed to start ranging on %s (err %d)", channel.source_label.c_str(), status);
      continue;
    }

    channel.ranging_started = true;
    channel.last_error = 0;
    started_count++;
    delayMicroseconds(250);
    App.feed_wdt();
  }

  ESP_LOGI(TAG, "Started continuous ranging on %u sensors in a synchronized batch",
           static_cast<unsigned>(started_count));
  return started_count > 0;
}

bool TofArrayTest::read_channel_(Channel &channel) {
  auto &sensor = *channel.sensor;
  const uint32_t started = millis();

  if (!channel.ranging_started && !this->restart_ranging_(channel)) {
    return false;
  }

  uint8_t ready = 0;
  auto status = sensor.CheckForDataReady(&ready);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.consecutive_errors++;
    this->restart_ranging_(channel);
    return false;
  }

  if (!ready) {
    return true;
  }

  uint16_t distance = 0;
  status = sensor.GetDistanceInMm(&distance);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.consecutive_errors++;
    this->restart_ranging_(channel);
    return false;
  }

  status = sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.consecutive_errors++;
    this->restart_ranging_(channel);
    return false;
  }

  channel.last_distance = distance;
  channel.has_reading = true;
  channel.last_error = 0;
  channel.consecutive_errors = 0;
  channel.last_update_ms = millis();
  channel.last_good_read_ms = channel.last_update_ms;
  channel.last_read_duration_ms = millis() - started;
  return true;
}

bool TofArrayTest::restart_ranging_(Channel &channel) {
  if (!channel.sensor) {
    channel.ranging_started = false;
    return false;
  }

  auto &sensor = *channel.sensor;
  sensor.StopRanging();
  delay(2);
  const auto status = sensor.StartRanging();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.ranging_started = false;
    return false;
  }

  channel.ranging_started = true;
  return true;
}

void TofArrayTest::rediscover() {
  ESP_LOGI(TAG, "Starting sensor discovery");
  this->channels_.clear();
  this->candidate_trace_.clear();
  this->set_all_xshut_(false);
  delay(this->wake_delay_ms_);

  uint8_t next_address = this->base_address_;

  if (this->probe_default_sensor_()) {
    Channel channel;
    channel.xshut_index = -1;
    channel.address = next_address++;
    channel.source_label = "Always-on / no XSHUT";
    channel.sensor = std::make_unique<VL53L1X_ULD>();
    if (this->wait_for_boot_(*channel.sensor) && this->set_temp_address_(*channel.sensor, channel.address) &&
        this->configure_sensor_(channel)) {
      ESP_LOGI(TAG, "Discovered always-on sensor at 0x%02X", channel.address);
      char addr[8];
      snprintf(addr, sizeof(addr), "0x%02X", channel.address);
      this->candidate_trace_.push_back("Always-on: OK -> " + std::string(addr));
      this->channels_.push_back(std::move(channel));
    } else {
      ESP_LOGW(TAG, "Found default-address sensor but could not initialize it");
      this->candidate_trace_.push_back("Always-on: ACK at 0x29 but init failed");
      this->recover_wire_();
    }
  } else {
    this->candidate_trace_.push_back("Always-on: no ACK at 0x29");
    this->recover_wire_();
  }

  for (size_t index = 0; index < this->xshut_pins_.size(); index++) {
    this->set_xshut_(index, true);
    delay(this->wake_delay_ms_);
    const std::string label = this->sensor_label_for_pin_(this->xshut_pin_numbers_[index]) + " / GPIO" +
                              std::to_string(this->xshut_pin_numbers_[index]);

    if (!this->probe_default_sensor_()) {
      ESP_LOGD(TAG, "No sensor found on candidate GPIO%u", this->xshut_pin_numbers_[index]);
      this->candidate_trace_.push_back(label + ": no ACK at 0x29");
      this->set_xshut_(index, false);
      this->recover_wire_();
      continue;
    }

    Channel channel;
    channel.xshut_index = static_cast<int>(index);
    channel.address = next_address++;
    channel.source_label = label;
    channel.sensor = std::make_unique<VL53L1X_ULD>();
    if (!this->wait_for_boot_(*channel.sensor)) {
      ESP_LOGW(TAG, "Sensor on GPIO%u ACKed, but never reported boot-ready", this->xshut_pin_numbers_[index]);
      this->candidate_trace_.push_back(label + ": ACK at 0x29 but boot wait failed");
      this->set_xshut_(index, false);
      this->recover_wire_();
      continue;
    }

    if (!this->set_temp_address_(*channel.sensor, channel.address)) {
      ESP_LOGW(TAG, "Sensor on GPIO%u ACKed, but address change to 0x%02X failed", this->xshut_pin_numbers_[index],
               channel.address);
      this->candidate_trace_.push_back(label + ": boot OK but address set failed");
      this->set_xshut_(index, false);
      this->recover_wire_();
      continue;
    }

    delay(this->post_address_delay_ms_);
    this->capture_identity_(channel);

    if (this->configure_sensor_(channel)) {
      ESP_LOGI(TAG, "Discovered sensor on GPIO%u at 0x%02X", this->xshut_pin_numbers_[index], channel.address);
      char addr[16];
      char id[16];
      snprintf(addr, sizeof(addr), "0x%02X", channel.address);
      snprintf(id, sizeof(id), "0x%04X", channel.sensor_id);
      std::string trace = label + ": OK -> " + std::string(addr) + " id " + id;
      if (channel.default_timing_fallback) {
        trace += " (default timing)";
      }
      this->candidate_trace_.push_back(trace);
      this->channels_.push_back(std::move(channel));
    } else {
      ESP_LOGW(TAG, "Sensor on GPIO%u responded, but initialization failed", this->xshut_pin_numbers_[index]);
      char id[16];
      char addr_reg[16];
      char fw_status[16];
      snprintf(id, sizeof(id), "0x%04X", channel.sensor_id);
      snprintf(addr_reg, sizeof(addr_reg), "0x%02X", channel.address_register);
      snprintf(fw_status, sizeof(fw_status), "0x%02X", channel.boot_state);
      this->candidate_trace_.push_back(label + ": configure failed at " + channel.last_stage + " err " +
                                       std::to_string(channel.last_error) + " id " + id + " fw " + fw_status +
                                       " addr_reg " + addr_reg);
      this->set_xshut_(index, false);
      this->recover_wire_();
    }

    this->recover_wire_();
  }

  this->start_all_ranging_();
  this->last_discovery_ms_ = millis();
  ESP_LOGI(TAG, "Discovery complete: %u sensors active", static_cast<unsigned>(this->channels_.size()));
}

float TofArrayTest::get_discovered_sensor_count() const { return static_cast<float>(this->channels_.size()); }

float TofArrayTest::get_cycle_duration_ms() const { return static_cast<float>(this->cycle_duration_ms_); }

float TofArrayTest::get_update_skew_ms() const {
  if (this->channels_.size() < 2) {
    return 0.0f;
  }

  uint32_t min_ts = UINT32_MAX;
  uint32_t max_ts = 0;
  bool seen = false;
  for (const auto &channel : this->channels_) {
    if (!channel.has_reading || channel.last_update_ms == 0) {
      continue;
    }
    seen = true;
    min_ts = std::min(min_ts, channel.last_update_ms);
    max_ts = std::max(max_ts, channel.last_update_ms);
  }
  if (!seen) {
    return NAN;
  }
  return static_cast<float>(max_ts - min_ts);
}

float TofArrayTest::get_distance_span_mm() const {
  uint16_t min_distance = UINT16_MAX;
  uint16_t max_distance = 0;
  bool seen = false;
  for (const auto &channel : this->channels_) {
    if (!channel.has_reading) {
      continue;
    }
    seen = true;
    min_distance = std::min(min_distance, channel.last_distance);
    max_distance = std::max(max_distance, channel.last_distance);
  }
  if (!seen) {
    return NAN;
  }
  return static_cast<float>(max_distance - min_distance);
}

float TofArrayTest::get_distance_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].has_reading) {
    return NAN;
  }
  return static_cast<float>(this->channels_[index].last_distance);
}

float TofArrayTest::get_age_ms(size_t index) const {
  if (index >= this->channels_.size() || this->channels_[index].last_update_ms == 0) {
    return NAN;
  }
  return static_cast<float>(millis() - this->channels_[index].last_update_ms);
}

float TofArrayTest::get_read_duration_ms(size_t index) const {
  if (index >= this->channels_.size()) {
    return NAN;
  }
  return static_cast<float>(this->channels_[index].last_read_duration_ms);
}

float TofArrayTest::get_status_code(size_t index) const {
  if (index >= this->channels_.size()) {
    return NAN;
  }
  return static_cast<float>(this->channels_[index].last_error);
}

float TofArrayTest::get_address_decimal(size_t index) const {
  if (index >= this->channels_.size()) {
    return NAN;
  }
  return static_cast<float>(this->channels_[index].address);
}

std::string TofArrayTest::status_text_for_(const Channel &channel) const {
  if (!channel.initialized) {
    return "Init failed";
  }
  if (channel.last_error != 0) {
    return "Read error " + std::to_string(channel.last_error);
  }
  if (!channel.has_reading) {
    return "Waiting for first sample";
  }
  if (channel.default_timing_fallback) {
    return "OK (default timing)";
  }
  return "OK";
}

std::string TofArrayTest::sensor_label_for_pin_(uint8_t pin_number) const {
  switch (pin_number) {
    case 16:
      return "U3";
    case 17:
      return "U4";
    case 18:
      return "U5";
    case 19:
      return "U6";
    case 23:
      return "U7";
    case 25:
      return "U8";
    default:
      return "Unknown";
  }
}

std::string TofArrayTest::get_status_text(size_t index) const {
  if (index >= this->channels_.size()) {
    return "Not detected";
  }
  return this->status_text_for_(this->channels_[index]);
}

std::string TofArrayTest::get_address_hex(size_t index) const {
  if (index >= this->channels_.size()) {
    return "N/A";
  }
  char buffer[8];
  snprintf(buffer, sizeof(buffer), "0x%02X", this->channels_[index].address);
  return {buffer};
}

std::string TofArrayTest::get_source_label(size_t index) const {
  if (index >= this->channels_.size()) {
    return "Unused";
  }
  return this->channels_[index].source_label;
}

std::string TofArrayTest::get_summary() const {
  std::ostringstream oss;
  oss << this->channels_.size() << "/" << this->xshut_pins_.size()
      << " sensors discovered, cycle " << this->cycle_duration_ms_ << " ms";
  const auto skew = this->get_update_skew_ms();
  if (!std::isnan(skew)) {
    oss << ", skew " << static_cast<int>(skew) << " ms";
  }
  const auto span = this->get_distance_span_mm();
  if (!std::isnan(span)) {
    oss << ", span " << static_cast<int>(span) << " mm";
  }
  return oss.str();
}

std::string TofArrayTest::get_discovery_map() const {
  if (this->channels_.empty()) {
    return "No sensors discovered yet";
  }

  std::ostringstream oss;
  for (size_t i = 0; i < this->channels_.size(); i++) {
    if (i > 0) {
      oss << " | ";
    }
    oss << "S" << (i + 1) << "=" << this->channels_[i].source_label << "@" << this->get_address_hex(i);
  }
  return oss.str();
}

std::string TofArrayTest::get_candidate_trace() const {
  if (this->candidate_trace_.empty()) {
    return "Discovery has not run yet";
  }

  std::ostringstream oss;
  for (size_t i = 0; i < this->candidate_trace_.size(); i++) {
    if (i > 0) {
      oss << " | ";
    }
    oss << this->candidate_trace_[i];
  }
  return oss.str();
}

}  // namespace tof_array_test
}  // namespace esphome
