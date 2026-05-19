#include "tof_overdoor_counter.h"

#include <algorithm>
#include <sstream>

namespace esphome {
namespace tof_overdoor_counter {

static const char *const TAG = "tof_overdoor_counter";

static const char *pair_name(uint8_t row_index) { return row_index == 1 ? "U3/U4 pair" : "U7/U8 pair"; }

static const char *sensor_name_for_pin(uint8_t pin_number) {
  switch (pin_number) {
    case 16:
      return "U3";
    case 17:
      return "U4";
    case 23:
      return "U7";
    case 25:
      return "U8";
    default:
      return "Unknown";
  }
}

void TofOverdoorCounter::setup() {
  this->prepare_xshut_pins_();
  if (!this->initialize_wire_()) {
    this->mark_failed();
    return;
  }
  this->rediscover();
}

void TofOverdoorCounter::update() {
  if (this->channels_.empty()) {
    ESP_LOGW(TAG, "No ToF channels configured");
    return;
  }

  const uint32_t started = millis();
  bool any_failure = false;
  for (auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    if (!this->read_channel_(channel)) {
      any_failure = true;
    }
    App.feed_wdt();
  }
  this->cycle_duration_ms_ = millis() - started;

  if (any_failure) {
    this->recover_wire_();
  }

  if (this->mode_ == OperatingMode::COUNT) {
    this->update_occupancy_states_();
    this->update_baselines_();
    this->update_state_machine_();
  } else {
    for (auto &channel : this->channels_) {
      channel.occupied = false;
    }
    this->waiting_for_clear_ = false;
    this->cooldown_until_ms_ = 0;
    this->reset_sequence_();
    this->phase_text_ = this->all_reporting_() ? "Monitoring distances only" : "Waiting for first live readings";
    this->last_direction_ = "Not counting";
  }
}

void TofOverdoorCounter::dump_config() {
  ESP_LOGCONFIG(TAG, "ToF Over-Door Counter:");
  ESP_LOGCONFIG(TAG, "  SDA Pin: %u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: %u", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  I2C Frequency: %u Hz", this->i2c_frequency_);
  ESP_LOGCONFIG(TAG, "  Timeout: %u ms", this->timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Trigger Delta: %u mm", this->trigger_delta_mm_);
  ESP_LOGCONFIG(TAG, "  Release Delta: %u mm", this->release_delta_mm_);
  ESP_LOGCONFIG(TAG, "  Sequence Timeout: %u ms", this->sequence_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Cooldown: %u ms", this->cooldown_ms_);
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->mode_ == OperatingMode::COUNT ? "count" : "monitor");
  ESP_LOGCONFIG(TAG, "  Invert Direction: %s", this->invert_direction_ ? "YES" : "NO");
  for (size_t i = 0; i < this->channels_.size(); i++) {
    const auto &channel = this->channels_[i];
    ESP_LOGCONFIG(TAG, "  Slot %u -> %s, initialized=%s, address=0x%02X", static_cast<unsigned>(i + 1),
                  channel.source_label.c_str(), channel.initialized ? "true" : "false", channel.address);
  }
}

bool TofOverdoorCounter::initialize_wire_() {
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

bool TofOverdoorCounter::recover_wire_() {
  this->wire_initialized_ = false;
  ESP_LOGD(TAG, "Recovering Wire bus");
  return this->initialize_wire_();
}

void TofOverdoorCounter::prepare_xshut_pins_() {
  for (auto *pin : this->xshut_pins_) {
    pin->setup();
    pin->pin_mode(gpio::FLAG_OUTPUT);
  }
  this->set_all_xshut_(false);
  delay(this->wake_delay_ms_);
}

void TofOverdoorCounter::set_all_xshut_(bool state) {
  for (auto *pin : this->xshut_pins_) {
    pin->digital_write(state);
  }
}

void TofOverdoorCounter::set_xshut_(size_t index, bool state) {
  if (index >= this->xshut_pins_.size()) {
    return;
  }
  this->xshut_pins_[index]->digital_write(state);
}

bool TofOverdoorCounter::probe_address_(uint8_t address) {
  Wire.beginTransmission(address);
  const uint8_t rc = Wire.endTransmission();
  return rc == 0;
}

bool TofOverdoorCounter::wait_for_boot_(VL53L1X_ULD &sensor) {
  delayMicroseconds(1200);
  uint8_t device_state = 0;
  const uint32_t started = millis();
  while ((millis() - started) < this->timeout_ms_) {
    const auto status = sensor.GetBootState(&device_state);
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

bool TofOverdoorCounter::set_temp_address_(VL53L1X_ULD &sensor, uint8_t address) {
  const auto status = sensor.SetI2CAddress(address << 1);
  if (status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Failed to change sensor address to 0x%02X, error=%d", address, status);
    return false;
  }
  return true;
}

bool TofOverdoorCounter::configure_sensor_(Channel &channel) {
  auto &sensor = *channel.sensor;
  if (!this->wait_for_boot_(sensor)) {
    channel.last_error = VL53L1_ERROR_TIME_OUT;
    return false;
  }

  VL53L1_Error status = VL53L1_ERROR_NONE;
  const uint8_t retries = std::max<uint8_t>(1, this->init_retries_);
  for (uint8_t attempt = 0; attempt < retries; attempt++) {
    if (attempt > 0) {
      delay(this->post_address_delay_ms_);
      if (!this->wait_for_boot_(sensor)) {
        channel.last_error = VL53L1_ERROR_TIME_OUT;
        continue;
      }
    }
    status = sensor.Init();
    if (status == VL53L1_ERROR_NONE) {
      break;
    }
    channel.last_error = status;
  }

  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }

  status = sensor.SetROI(16, 16);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  status = sensor.SetROICenter(199);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  status = sensor.SetDistanceMode(EDistanceMode::Long);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  status = sensor.SetTimingBudgetInMs(20);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  status = sensor.SetInterMeasurementInMs(25);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }

  status = sensor.StartRanging();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.ranging_started = false;
    return false;
  }

  channel.initialized = true;
  channel.ranging_started = true;
  channel.last_error = 0;
  return true;
}

bool TofOverdoorCounter::read_channel_(Channel &channel) {
  auto &sensor = *channel.sensor;
  const uint32_t started = millis();

  if (!channel.ranging_started && !this->restart_ranging_(channel)) {
    channel.occupied = false;
    return false;
  }

  uint8_t ready = 0;
  auto status = sensor.CheckForDataReady(&ready);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.occupied = false;
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
    channel.occupied = false;
    this->restart_ranging_(channel);
    return false;
  }

  status = sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.occupied = false;
    this->restart_ranging_(channel);
    return false;
  }

  channel.last_distance = distance;
  channel.has_reading = true;
  channel.last_error = 0;
  channel.last_update_ms = millis();
  channel.last_read_duration_ms = millis() - started;
  return true;
}

bool TofOverdoorCounter::restart_ranging_(Channel &channel) {
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

void TofOverdoorCounter::rediscover() {
  ESP_LOGI(TAG, "Starting four-sensor discovery");
  this->channels_.clear();
  this->channels_.resize(this->xshut_pins_.size());
  this->set_all_xshut_(false);
  delay(this->wake_delay_ms_);

  uint8_t next_address = this->base_address_;
  for (size_t index = 0; index < this->xshut_pins_.size(); index++) {
    auto &channel = this->channels_[index];
    channel.pin_number = this->xshut_pin_numbers_[index];
    channel.source_label =
        std::string(sensor_name_for_pin(this->xshut_pin_numbers_[index])) + " / GPIO" +
        std::to_string(this->xshut_pin_numbers_[index]);
    channel.address = next_address;

    this->set_xshut_(index, true);
    delay(this->wake_delay_ms_);

    if (!this->probe_address_(0x29)) {
      ESP_LOGW(TAG, "No sensor ACK on %s", channel.source_label.c_str());
      this->set_xshut_(index, false);
      this->recover_wire_();
      continue;
    }

    channel.sensor = std::make_unique<VL53L1X_ULD>();
    if (!this->wait_for_boot_(*channel.sensor)) {
      ESP_LOGW(TAG, "%s ACKed but never reported boot-ready", channel.source_label.c_str());
      this->set_xshut_(index, false);
      this->recover_wire_();
      continue;
    }

    if (!this->set_temp_address_(*channel.sensor, channel.address)) {
      ESP_LOGW(TAG, "%s address change to 0x%02X failed", channel.source_label.c_str(), channel.address);
      this->set_xshut_(index, false);
      this->recover_wire_();
      continue;
    }

    delay(this->post_address_delay_ms_);

    if (!this->configure_sensor_(channel)) {
      ESP_LOGW(TAG, "%s configuration failed (err %d)", channel.source_label.c_str(), channel.last_error);
      this->set_xshut_(index, false);
      this->recover_wire_();
      continue;
    }

    ESP_LOGI(TAG, "Discovered sensor on %s at 0x%02X", channel.source_label.c_str(), channel.address);
    next_address++;
    this->recover_wire_();
  }

  this->last_discovery_ms_ = millis();
  this->recalibrate();
  ESP_LOGI(TAG, "Discovery complete: %u sensors active", static_cast<unsigned>(this->get_discovered_sensor_count()));
}

void TofOverdoorCounter::recalibrate() {
  for (auto &channel : this->channels_) {
    channel.baseline = NAN;
    channel.baseline_samples = 0;
    channel.occupied = false;
  }
  this->reset_sequence_();
  this->waiting_for_clear_ = false;
  this->cooldown_until_ms_ = 0;
  this->phase_text_ = "Recalibrating floor reference";
  ESP_LOGI(TAG, "Cleared baselines; waiting for empty doorway");
}

void TofOverdoorCounter::reset_counts() {
  this->entry_count_ = 0;
  this->exit_count_ = 0;
  this->people_count_ = 0;
  this->last_direction_ = "Reset";
  ESP_LOGI(TAG, "Counts reset to zero");
}

void TofOverdoorCounter::update_occupancy_states_() {
  for (auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading || std::isnan(channel.baseline)) {
      channel.occupied = false;
      continue;
    }

    const float active_limit = channel.baseline - static_cast<float>(this->trigger_delta_mm_);
    const float clear_limit = channel.baseline - static_cast<float>(this->release_delta_mm_);
    if (!channel.occupied) {
      channel.occupied = static_cast<float>(channel.last_distance) <= active_limit;
    } else {
      channel.occupied = static_cast<float>(channel.last_distance) <= clear_limit;
    }
  }
}

void TofOverdoorCounter::update_baselines_() {
  if (this->waiting_for_clear_ || this->row_is_active_(1) || this->row_is_active_(2) || this->start_row_ != 0) {
    return;
  }

  for (auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading) {
      continue;
    }

    if (std::isnan(channel.baseline)) {
      channel.baseline = static_cast<float>(channel.last_distance);
      channel.baseline_samples = 1;
      continue;
    }

    if (channel.baseline_samples < 8) {
      channel.baseline =
          ((channel.baseline * channel.baseline_samples) + static_cast<float>(channel.last_distance)) /
          static_cast<float>(channel.baseline_samples + 1);
      channel.baseline_samples++;
    } else {
      channel.baseline = (channel.baseline * 0.92f) + (static_cast<float>(channel.last_distance) * 0.08f);
    }
  }
}

void TofOverdoorCounter::update_state_machine_() {
  const bool row_a = this->row_is_active_(1);
  const bool row_b = this->row_is_active_(2);
  const uint32_t now = millis();

  if (!this->all_calibrated_()) {
    this->phase_text_ = "Calibrating floor reference";
    return;
  }

  if (this->waiting_for_clear_) {
    this->phase_text_ = "Waiting for doorway to clear";
    if (!row_a && !row_b) {
      this->waiting_for_clear_ = false;
      this->phase_text_ = "Ready";
    }
    return;
  }

  if (this->cooldown_until_ms_ != 0 && now < this->cooldown_until_ms_) {
    this->phase_text_ = "Settling after last event";
    return;
  }
  if (this->cooldown_until_ms_ != 0 && now >= this->cooldown_until_ms_) {
    this->cooldown_until_ms_ = 0;
  }

  const uint8_t single_row = row_a && !row_b ? 1 : (!row_a && row_b ? 2 : 0);

  if (this->start_row_ == 0) {
    if (single_row != 0) {
      this->start_row_ = single_row;
      this->last_single_row_ = single_row;
      this->saw_both_ = false;
      this->sequence_started_ms_ = now;
      this->phase_text_ = std::string(pair_name(single_row)) + " leading";
    } else if (row_a && row_b) {
      this->phase_text_ = "Both sensor pairs active - waiting for an edge";
    } else {
      this->phase_text_ = "Ready";
    }
    return;
  }

  if ((now - this->sequence_started_ms_) > this->sequence_timeout_ms_) {
    ESP_LOGW(TAG, "Sequence timed out");
    this->reset_sequence_();
    this->waiting_for_clear_ = true;
    this->phase_text_ = "Sequence timeout";
    return;
  }

  if (!row_a && !row_b) {
    this->finalize_sequence_();
    return;
  }

  if (row_a && row_b) {
    this->saw_both_ = true;
    this->phase_text_ = "Both sensor pairs active";
    return;
  }

  if (single_row != 0) {
    this->last_single_row_ = single_row;
    this->phase_text_ = std::string(pair_name(single_row)) + " active";
  }
}

void TofOverdoorCounter::finalize_sequence_() {
  const bool valid_transition = this->saw_both_ && this->start_row_ != 0 && this->last_single_row_ != 0 &&
                                this->start_row_ != this->last_single_row_;

  if (!valid_transition) {
    this->last_direction_ = "Cancelled";
    this->phase_text_ = "Sequence cancelled";
    this->reset_sequence_();
    this->cooldown_until_ms_ = millis() + this->cooldown_ms_;
    return;
  }

  const bool a_to_b = this->start_row_ == 1 && this->last_single_row_ == 2;
  const bool entry = this->invert_direction_ ? !a_to_b : a_to_b;

  if (entry) {
    this->entry_count_++;
    this->people_count_ = std::min(this->people_count_ + 1, 50);
    this->last_direction_ = "Entry";
  } else {
    this->exit_count_++;
    this->people_count_ = std::max(this->people_count_ - 1, 0);
    this->last_direction_ = "Exit";
  }

  ESP_LOGI(TAG, "Counted %s (people=%d, entry=%u, exit=%u)", this->last_direction_.c_str(), this->people_count_,
           static_cast<unsigned>(this->entry_count_), static_cast<unsigned>(this->exit_count_));

  this->phase_text_ = "Settling after count";
  this->reset_sequence_();
  this->cooldown_until_ms_ = millis() + this->cooldown_ms_;
}

void TofOverdoorCounter::reset_sequence_() {
  this->start_row_ = 0;
  this->last_single_row_ = 0;
  this->saw_both_ = false;
  this->sequence_started_ms_ = 0;
}

bool TofOverdoorCounter::all_calibrated_() const {
  if (this->mode_ != OperatingMode::COUNT) {
    return this->all_reporting_();
  }
  bool has_any = false;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    has_any = true;
    if (std::isnan(channel.baseline)) {
      return false;
    }
  }
  return has_any;
}

bool TofOverdoorCounter::all_reporting_() const {
  bool has_any = false;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    has_any = true;
    if (!channel.has_reading) {
      return false;
    }
  }
  return has_any;
}

bool TofOverdoorCounter::row_is_active_(uint8_t row_index) const {
  const size_t start = row_index == 1 ? 0 : 2;
  const size_t end = row_index == 1 ? 2 : 4;
  for (size_t i = start; i < end && i < this->channels_.size(); i++) {
    const auto &channel = this->channels_[i];
    if (channel.initialized && channel.occupied) {
      return true;
    }
  }
  return false;
}

float TofOverdoorCounter::row_distance_internal_(uint8_t row_index) const {
  const size_t start = row_index == 1 ? 0 : 2;
  const size_t end = row_index == 1 ? 2 : 4;
  uint16_t min_distance = UINT16_MAX;
  bool seen = false;
  for (size_t i = start; i < end && i < this->channels_.size(); i++) {
    const auto &channel = this->channels_[i];
    if (!channel.initialized || !channel.has_reading) {
      continue;
    }
    min_distance = std::min(min_distance, channel.last_distance);
    seen = true;
  }
  return seen ? static_cast<float>(min_distance) : NAN;
}

float TofOverdoorCounter::row_baseline_internal_(uint8_t row_index) const {
  const size_t start = row_index == 1 ? 0 : 2;
  const size_t end = row_index == 1 ? 2 : 4;
  float min_baseline = NAN;
  for (size_t i = start; i < end && i < this->channels_.size(); i++) {
    const auto &channel = this->channels_[i];
    if (!channel.initialized || std::isnan(channel.baseline)) {
      continue;
    }
    if (std::isnan(min_baseline) || channel.baseline < min_baseline) {
      min_baseline = channel.baseline;
    }
  }
  return min_baseline;
}

float TofOverdoorCounter::row_drop_internal_(uint8_t row_index) const {
  const size_t start = row_index == 1 ? 0 : 2;
  const size_t end = row_index == 1 ? 2 : 4;
  float max_drop = NAN;
  for (size_t i = start; i < end && i < this->channels_.size(); i++) {
    const auto &channel = this->channels_[i];
    if (!channel.initialized || !channel.has_reading || std::isnan(channel.baseline)) {
      continue;
    }
    const float drop = channel.baseline - static_cast<float>(channel.last_distance);
    if (std::isnan(max_drop) || drop > max_drop) {
      max_drop = drop;
    }
  }
  return max_drop;
}

float TofOverdoorCounter::get_discovered_sensor_count() const {
  uint32_t count = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized) {
      count++;
    }
  }
  return static_cast<float>(count);
}

float TofOverdoorCounter::get_reporting_sensor_count() const {
  uint32_t count = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.has_reading) {
      count++;
    }
  }
  return static_cast<float>(count);
}

float TofOverdoorCounter::get_cycle_duration_ms() const { return static_cast<float>(this->cycle_duration_ms_); }

float TofOverdoorCounter::get_update_skew_ms() const {
  uint32_t min_ts = UINT32_MAX;
  uint32_t max_ts = 0;
  bool seen = false;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading || channel.last_update_ms == 0) {
      continue;
    }
    min_ts = std::min(min_ts, channel.last_update_ms);
    max_ts = std::max(max_ts, channel.last_update_ms);
    seen = true;
  }
  return seen ? static_cast<float>(max_ts - min_ts) : NAN;
}

float TofOverdoorCounter::get_nearest_distance_mm() const {
  uint16_t min_distance = UINT16_MAX;
  bool seen = false;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading) {
      continue;
    }
    min_distance = std::min(min_distance, channel.last_distance);
    seen = true;
  }
  return seen ? static_cast<float>(min_distance) : NAN;
}

float TofOverdoorCounter::get_average_distance_mm() const {
  uint32_t total = 0;
  uint32_t count = 0;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading) {
      continue;
    }
    total += channel.last_distance;
    count++;
  }
  if (count == 0) {
    return NAN;
  }
  return static_cast<float>(total) / static_cast<float>(count);
}

float TofOverdoorCounter::get_distance_span_mm() const {
  uint16_t min_distance = UINT16_MAX;
  uint16_t max_distance = 0;
  bool seen = false;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading) {
      continue;
    }
    min_distance = std::min(min_distance, channel.last_distance);
    max_distance = std::max(max_distance, channel.last_distance);
    seen = true;
  }
  if (!seen) {
    return NAN;
  }
  return static_cast<float>(max_distance - min_distance);
}

float TofOverdoorCounter::get_distance_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized || !this->channels_[index].has_reading) {
    return NAN;
  }
  return static_cast<float>(this->channels_[index].last_distance);
}

float TofOverdoorCounter::get_baseline_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized || std::isnan(this->channels_[index].baseline)) {
    return NAN;
  }
  return this->channels_[index].baseline;
}

float TofOverdoorCounter::get_row_distance_mm(size_t row_index) const { return this->row_distance_internal_(row_index == 0 ? 1 : 2); }

float TofOverdoorCounter::get_row_baseline_mm(size_t row_index) const {
  return this->row_baseline_internal_(row_index == 0 ? 1 : 2);
}

float TofOverdoorCounter::get_row_drop_mm(size_t row_index) const { return this->row_drop_internal_(row_index == 0 ? 1 : 2); }

float TofOverdoorCounter::get_entry_count() const { return static_cast<float>(this->entry_count_); }

float TofOverdoorCounter::get_exit_count() const { return static_cast<float>(this->exit_count_); }

float TofOverdoorCounter::get_people_count() const { return static_cast<float>(this->people_count_); }

float TofOverdoorCounter::get_presence_state() const {
  if (this->mode_ != OperatingMode::COUNT) {
    return 0.0f;
  }
  return (this->row_is_active_(1) || this->row_is_active_(2) || this->waiting_for_clear_ || this->start_row_ != 0) ? 1.0f
                                                                                                                       : 0.0f;
}

float TofOverdoorCounter::get_ready_state() const { return this->all_calibrated_() ? 1.0f : 0.0f; }

float TofOverdoorCounter::get_row_active_state(size_t row_index) const {
  if (this->mode_ != OperatingMode::COUNT) {
    return 0.0f;
  }
  return this->row_is_active_(row_index == 0 ? 1 : 2) ? 1.0f : 0.0f;
}

std::string TofOverdoorCounter::status_text_for_(const Channel &channel) const {
  if (!channel.initialized) {
    return "Missing";
  }
  if (channel.last_error != 0) {
    return "Read error " + std::to_string(channel.last_error);
  }
  if (!channel.has_reading) {
    return "Waiting for first sample";
  }
  if (this->mode_ != OperatingMode::COUNT) {
    return "Online";
  }
  if (std::isnan(channel.baseline)) {
    return "Learning floor baseline";
  }
  if (channel.occupied) {
    return "Occupied";
  }
  return "Clear";
}

std::string TofOverdoorCounter::get_mode_text() const {
  return this->mode_ == OperatingMode::COUNT ? "Count" : "Monitor";
}

std::string TofOverdoorCounter::get_status_text(size_t index) const {
  if (index >= this->channels_.size()) {
    return "Unused";
  }
  return this->status_text_for_(this->channels_[index]);
}

std::string TofOverdoorCounter::get_source_label(size_t index) const {
  if (index >= this->channels_.size()) {
    return "Unused";
  }
  return this->channels_[index].source_label;
}

std::string TofOverdoorCounter::get_phase_text() const { return this->phase_text_; }

std::string TofOverdoorCounter::get_last_direction_text() const { return this->last_direction_; }

std::string TofOverdoorCounter::get_summary() const {
  std::ostringstream oss;
  if (this->mode_ != OperatingMode::COUNT) {
    oss << "Mode Monitor";
    const auto nearest = this->get_nearest_distance_mm();
    const auto average = this->get_average_distance_mm();
    const auto span = this->get_distance_span_mm();
    if (!std::isnan(nearest)) {
      oss << " | Nearest " << static_cast<int>(nearest) << " mm";
    }
    if (!std::isnan(average)) {
      oss << " | Average " << static_cast<int>(average) << " mm";
    }
    if (!std::isnan(span)) {
      oss << " | Spread " << static_cast<int>(span) << " mm";
    }
    return oss.str();
  }

  oss << "People " << this->people_count_ << " | Entry " << this->entry_count_ << " | Exit " << this->exit_count_
      << " | " << this->phase_text_;
  const auto row_a = this->row_distance_internal_(1);
  const auto row_b = this->row_distance_internal_(2);
  if (!std::isnan(row_a) && !std::isnan(row_b)) {
    oss << " | A " << static_cast<int>(row_a) << " mm, B " << static_cast<int>(row_b) << " mm";
  }
  return oss.str();
}

std::string TofOverdoorCounter::get_discovery_map() const {
  if (this->channels_.empty()) {
    return "No discovery data yet";
  }

  std::ostringstream oss;
  for (size_t i = 0; i < this->channels_.size(); i++) {
    if (i > 0) {
      oss << " | ";
    }
    const auto &channel = this->channels_[i];
    oss << "S" << (i + 1) << "=" << channel.source_label;
    if (channel.initialized) {
      char buffer[8];
      snprintf(buffer, sizeof(buffer), "0x%02X", channel.address);
      oss << "@" << buffer;
    } else {
      oss << "@missing";
    }
  }
  return oss.str();
}

}  // namespace tof_overdoor_counter
}  // namespace esphome
