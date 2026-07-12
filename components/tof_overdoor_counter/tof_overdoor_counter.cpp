#include "tof_overdoor_counter.h"

#include <Arduino.h>
#include <algorithm>
#include <cstdint>
#include <esp_system.h>
#include <sstream>

namespace esphome {
namespace tof_overdoor_counter {
namespace {

static const char *const TAG = "tof_overdoor_counter";
constexpr uint8_t PERSISTED_STATE_VERSION = 6;
constexpr uint32_t STALE_READING_MS = 450;
// Keep every sensor in XSHUT while the ESP32 rail, logger and Wi-Fi radio pass
// their cold-start/inrush phase. A warm reset does not reproduce that load.
constexpr uint32_t COLD_BOOT_SETTLE_MS = 5000;
constexpr uint32_t HEARTBEAT_INTERVAL_MS = 5000;
constexpr uint32_t I2C_TRANSACTION_TIMEOUT_MS = 50;
constexpr uint32_t CALIBRATION_CLEAR_SETTLE_MS = 120;
constexpr uint32_t BOOT_CLEAR_SETTLE_MS = 500;
constexpr uint32_t SENSOR_RECOVERY_BASE_MS = 1000;
constexpr uint32_t SENSOR_RECOVERY_MAX_MS = 30000;
constexpr uint8_t ERRORS_BEFORE_POWER_CYCLE = 3;
constexpr uint32_t TRACE_SAMPLE_INTERVAL_MS = 25;
// React strongly to a new range sample. The adaptive threshold, hysteresis and
// quorum provide the noise rejection; a slow EMA only delays zone edges.
constexpr float FILTER_ALPHA = 0.85f;
constexpr float BASELINE_TRACK_ALPHA = 0.015f;
constexpr float NOISE_TRACK_ALPHA = 0.08f;
constexpr uint16_t MIN_VALID_DISTANCE_MM = 30;
constexpr uint16_t MAX_VALID_DISTANCE_MM = 4000;
constexpr uint8_t ROODE_ZONE_WIDTH = 8;
constexpr uint8_t ROODE_ZONE_HEIGHT = 16;
constexpr uint8_t ROODE_OUT_ZONE_CENTER = 167;
constexpr uint8_t ROODE_IN_ZONE_CENTER = 231;
constexpr uint8_t ROODE_NOBODY = 0;
constexpr uint8_t ROODE_SOMEONE = 1;
constexpr uint8_t GROUP_STATE_NONE = 0;
constexpr uint8_t GROUP_STATE_OUT_ONLY = 1;
constexpr uint8_t GROUP_STATE_IN_ONLY = 2;
constexpr uint8_t GROUP_STATE_BOTH = 3;

const char *sensor_name_for_pin(uint8_t pin_number) {
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

SensorGroup group_for_pin(uint8_t pin_number) {
  // Legacy physical grouping kept for old row entities. Direction is now
  // decided per physical sensor from its two Roode ROI zones, then fused by quorum.
  switch (pin_number) {
    case 16:
    case 23:
      return GROUP_OUT;
    case 17:
    case 25:
      return GROUP_IN;
    default:
      return GROUP_NONE;
  }
}

const char *group_name(SensorGroup group) {
  switch (group) {
    case GROUP_OUT:
      return "OUT";
    case GROUP_IN:
      return "IN";
    default:
      return "Unknown";
  }
}

const char *group_debug_name(SensorGroup group) {
  return group == GROUP_NONE ? "none" : group_name(group);
}

const char *zone_name(uint8_t zone_index) {
  return zone_index == ZONE_OUT ? "OUT-zone" : "IN-zone";
}

uint8_t roi_center_for_zone(uint8_t zone_index) {
  return zone_index == ZONE_OUT ? ROODE_OUT_ZONE_CENTER : ROODE_IN_ZONE_CENTER;
}

SensorGroup group_from_state_code(uint8_t state_code) {
  if (state_code == GROUP_STATE_OUT_ONLY) {
    return GROUP_OUT;
  }
  if (state_code == GROUP_STATE_IN_ONLY) {
    return GROUP_IN;
  }
  return GROUP_NONE;
}

const char *group_state_name(uint8_t state_code) {
  switch (state_code) {
    case GROUP_STATE_OUT_ONLY:
      return "OUT";
    case GROUP_STATE_IN_ONLY:
      return "IN";
    case GROUP_STATE_BOTH:
      return "BOTH";
    case GROUP_STATE_NONE:
    default:
      return "CLEAR";
  }
}

const char *range_status_name(uint8_t status) {
  switch (status) {
    case RangeValid:
      return "valid";
    case SigmaFail:
      return "sigma_fail";
    case SignalFail:
      return "signal_fail";
    case MinRangeFail:
      return "min_range_fail";
    case PhaseOutOfLimit:
      return "phase_limit";
    case HardwareFail:
      return "hardware_fail";
    case RangeValidNoWrapCheck:
      return "valid_no_wrap";
    case WrapTargetFail:
      return "wrap_fail";
    default:
      return "unknown";
  }
}

const char *distance_mode_name(SensorDistanceMode mode) {
  switch (mode) {
    case DISTANCE_MODE_SHORT:
      return "short";
    case DISTANCE_MODE_LONG:
    default:
      return "long";
  }
}

uint8_t popcount_u8(uint8_t value) {
  uint8_t count = 0;
  while (value != 0) {
    count += value & 0x01;
    value >>= 1;
  }
  return count;
}

float clampf(float value, float min_value, float max_value) {
  return std::max(min_value, std::min(value, max_value));
}

template<typename T>
T sanitize_persisted(T value, T min_value, T max_value, T fallback) {
  if (value < min_value || value > max_value) {
    return fallback;
  }
  return value;
}

uint8_t clamp_quality(float value) {
  if (value < 0.0f) {
    return 0;
  }
  if (value > 100.0f) {
    return 100;
  }
  return static_cast<uint8_t>(value);
}

}  // namespace

void TofOverdoorCounter::setup() {
  this->apply_calibration_defaults_();
  // Assert every XSHUT line before doing anything on I2C. After an uncontrolled
  // power restoration all four VL53L1X devices otherwise wake at 0x29 and can
  // leave SDA low before ESPHome has even initialized its logger.
  this->prepare_xshut_pins_();
  this->init_preferences_();
  this->wire_initialized_ = false;
  this->system_status_ = STATUS_BOOTING;
  this->phase_text_ = "Core online; sensor discovery scheduled";
  this->next_rediscovery_ms_ = millis() + COLD_BOOT_SETTLE_MS;
}

void TofOverdoorCounter::update() {
  const uint32_t now = millis();

  if (!this->cold_boot_reset_evaluated_) {
    const esp_reset_reason_t reset_reason = esp_reset_reason();
    this->cold_boot_reset_evaluated_ = true;
    this->cold_boot_reset_pending_ =
        this->cold_boot_soft_reset_enabled_ &&
        (reset_reason == ESP_RST_POWERON || reset_reason == ESP_RST_BROWNOUT);
    if (this->cold_boot_reset_pending_) {
      this->cold_boot_reset_due_ms_ = now + this->cold_boot_soft_reset_delay_ms_;
      this->phase_text_ = "Cold-start stabilization before one-time software reset";
      ESP_LOGW(TAG, "Cold boot reset_reason=%d; one-time ESP.restart() scheduled in %u ms",
               static_cast<int>(reset_reason), static_cast<unsigned>(this->cold_boot_soft_reset_delay_ms_));
    }
  }

  if (this->cold_boot_reset_pending_ &&
      static_cast<int32_t>(now - this->cold_boot_reset_due_ms_) >= 0) {
    this->cold_boot_reset_pending_ = false;
    ESP_LOGW(TAG, "Power rail stabilization complete; performing one-time cold-boot software reset now");
    delay(20);
    ESP.restart();
    return;
  }

  if (!this->boot_diagnostics_logged_) {
    ESP_LOGI(TAG, "ESP32 core reached main loop; reset_reason=%d, ToF discovery deferred until %u ms uptime",
             static_cast<int>(esp_reset_reason()), static_cast<unsigned>(this->next_rediscovery_ms_));
    this->boot_diagnostics_logged_ = true;
    this->last_heartbeat_log_ms_ = now;
  } else if ((now - this->last_heartbeat_log_ms_) >= HEARTBEAT_INTERVAL_MS) {
    ESP_LOGI(TAG, "Heartbeat uptime=%u ms wire=%s discovered=%u phase=%s", static_cast<unsigned>(now),
             this->wire_initialized_ ? "ready" : "waiting",
             static_cast<unsigned>(this->get_discovered_sensor_count()), this->phase_text_.c_str());
    this->last_heartbeat_log_ms_ = now;
  }

  if (!this->wire_initialized_ || this->channels_.empty()) {
    if (this->next_rediscovery_ms_ == 0 || static_cast<int32_t>(now - this->next_rediscovery_ms_) >= 0) {
      if (this->initialize_wire_()) {
        this->rediscover();
      } else {
        this->system_status_ = STATUS_ERROR;
        this->phase_text_ = "I2C unavailable; retrying automatically";
        this->next_rediscovery_ms_ = now + SENSOR_RECOVERY_BASE_MS;
      }
    }
    return;
  }

  const uint32_t started = millis();
  for (auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    if (!this->read_channel_(channel)) {
      // Recovery is deliberately deferred and isolated to this channel. A
      // single marginal sensor must never restart the other three streams.
    }
    App.feed_wdt();
  }
  this->cycle_duration_ms_ = millis() - started;

  this->update_sensor_health_();
  this->service_recovery_(millis());

  if (this->calibration_active_) {
    this->process_calibration_();
    this->update_system_status_();
    return;
  }

  this->update_sensor_states_();
  this->record_history_snapshot_(millis());
  this->debug_log_sample_(millis());
  this->apply_idle_baseline_tracking_();
  this->update_blocked_state_();

  if (this->mode_ == OperatingMode::COUNT) {
    this->update_detection_state_machine_();
  } else {
    this->clear_event_tracking_();
    this->update_passage_state_(PASSAGE_IDLE);
    this->person_standing_in_door_ = false;
    this->phase_text_ = this->ready_for_counting_() ? "Monitoring live distances" : "Waiting for stable readings";
  }

  this->update_system_status_();

  if (this->state_dirty_ && this->auto_save_enabled_) {
    this->persist_runtime_state();
  }
}

void TofOverdoorCounter::dump_config() {
  ESP_LOGCONFIG(TAG, "ToF Over-Door Counter:");
  ESP_LOGCONFIG(TAG, "  Mode: %s", this->mode_ == OperatingMode::COUNT ? "count" : "monitor");
  ESP_LOGCONFIG(TAG, "  SDA Pin: %u", this->sda_pin_);
  ESP_LOGCONFIG(TAG, "  SCL Pin: %u", this->scl_pin_);
  ESP_LOGCONFIG(TAG, "  I2C Frequency: %u Hz", this->i2c_frequency_);
  ESP_LOGCONFIG(TAG, "  Distance Mode: %s", distance_mode_name(this->distance_mode_));
  ESP_LOGCONFIG(TAG, "  Timing Budget: %u ms", this->timing_budget_ms_);
  ESP_LOGCONFIG(TAG, "  Intermeasurement: %u ms", this->intermeasurement_ms_);
  ESP_LOGCONFIG(TAG, "  Sampling: %u", this->sampling_size_);
  ESP_LOGCONFIG(TAG, "  Trigger Threshold: %u mm", this->trigger_threshold_mm_);
  ESP_LOGCONFIG(TAG, "  Release Delta: %u mm", this->clear_threshold_mm_);
  ESP_LOGCONFIG(TAG, "  Baseline Tolerance: %u mm", this->baseline_tolerance_mm_);
  ESP_LOGCONFIG(TAG, "  Debounce: %u ms", this->debounce_ms_);
  ESP_LOGCONFIG(TAG, "  Detection Timeout: %u ms", this->detection_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Cooldown: %u ms", this->cooldown_ms_);
  ESP_LOGCONFIG(TAG, "  Blocked Timeout: %u ms", this->blocked_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Standing Timeout: %u ms", this->standing_timeout_ms_);
  ESP_LOGCONFIG(TAG, "  Min Event Sensors: %u", this->min_event_sensors_);
  ESP_LOGCONFIG(TAG, "  Min Active Duration: %u ms", this->min_active_duration_ms_);
  ESP_LOGCONFIG(TAG, "  Direction Window: %u ms", this->direction_window_ms_);
  ESP_LOGCONFIG(TAG, "  Calibration Samples: %u", this->calibration_samples_);
  ESP_LOGCONFIG(TAG, "  Min Valid Sensors: %u", this->min_valid_sensors_);
  ESP_LOGCONFIG(TAG, "  Max People Inside: %u", this->max_people_inside_);
  ESP_LOGCONFIG(TAG, "  Invert Direction: %s", this->invert_direction_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Debug Logging: %s", this->debug_logging_ ? "YES" : "NO");
  for (size_t i = 0; i < this->channels_.size(); i++) {
    const auto &channel = this->channels_[i];
    ESP_LOGCONFIG(TAG, "  Slot %u -> %s, group=%s, initialized=%s, address=0x%02X",
                  static_cast<unsigned>(i + 1), channel.source_label.c_str(), group_name(channel.group),
                  channel.initialized ? "true" : "false", channel.address);
  }
}

bool TofOverdoorCounter::initialize_wire_() {
  if (this->wire_initialized_) {
    return true;
  }
  Wire.end();
  delay(1);
  const bool bus_released = this->clear_i2c_bus_();
  if (!bus_released) {
    ESP_LOGW(TAG, "I2C bus is still held low after recovery pulses; Wire will retry with bounded transactions");
  }
  if (!Wire.begin(this->sda_pin_, this->scl_pin_)) {
    ESP_LOGE(TAG, "Failed to initialize Wire on SDA=%u SCL=%u", this->sda_pin_, this->scl_pin_);
    return false;
  }
  // Never let a marginal or half-powered sensor hold the ESP32 main loop
  // indefinitely. Higher-level boot/recovery deadlines handle retries.
  Wire.setTimeOut(I2C_TRANSACTION_TIMEOUT_MS);
  Wire.setClock(this->i2c_frequency_);
  this->wire_initialized_ = true;
  ESP_LOGI(TAG, "Initialized Wire on SDA=%u SCL=%u @ %u Hz", this->sda_pin_, this->scl_pin_, this->i2c_frequency_);
  return true;
}

bool TofOverdoorCounter::clear_i2c_bus_() {
  pinMode(this->sda_pin_, INPUT_PULLUP);
  pinMode(this->scl_pin_, INPUT_PULLUP);
  delayMicroseconds(10);

  // Give a slow slave a short opportunity to release SCL before clocking it.
  const uint32_t clock_wait_started = micros();
  while (digitalRead(this->scl_pin_) == LOW && (micros() - clock_wait_started) < 2000U) {
    delayMicroseconds(10);
  }

  pinMode(this->scl_pin_, OUTPUT_OPEN_DRAIN);
  digitalWrite(this->scl_pin_, HIGH);
  for (uint8_t pulse = 0; pulse < 18 && digitalRead(this->sda_pin_) == LOW; pulse++) {
    digitalWrite(this->scl_pin_, LOW);
    delayMicroseconds(5);
    digitalWrite(this->scl_pin_, HIGH);
    delayMicroseconds(5);
  }

  // Generate an explicit STOP (SDA low -> SCL high -> SDA high).
  pinMode(this->sda_pin_, OUTPUT_OPEN_DRAIN);
  digitalWrite(this->sda_pin_, LOW);
  delayMicroseconds(5);
  digitalWrite(this->scl_pin_, HIGH);
  delayMicroseconds(5);
  digitalWrite(this->sda_pin_, HIGH);
  delayMicroseconds(5);

  pinMode(this->sda_pin_, INPUT_PULLUP);
  pinMode(this->scl_pin_, INPUT_PULLUP);
  delayMicroseconds(10);
  return digitalRead(this->sda_pin_) == HIGH && digitalRead(this->scl_pin_) == HIGH;
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
    // Power-up can produce a few transient I2C failures. Keep trying for the
    // configured deadline instead of permanently dropping the sensor.
    delay(status == VL53L1_ERROR_NONE ? 1 : 4);
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

  channel.current_zone = ZONE_OUT;
  if (!this->set_channel_roi_(channel, channel.current_zone)) {
    return false;
  }
  const EDistanceMode sensor_distance_mode =
      this->distance_mode_ == DISTANCE_MODE_SHORT ? EDistanceMode::Short : EDistanceMode::Long;
  status = sensor.SetDistanceMode(sensor_distance_mode);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  const uint16_t min_timing_budget = this->distance_mode_ == DISTANCE_MODE_SHORT ? 20 : 33;
  const uint16_t timing_budget_ms = std::max<uint16_t>(this->timing_budget_ms_, min_timing_budget);
  const uint16_t intermeasurement_ms =
      std::max<uint16_t>(this->intermeasurement_ms_, static_cast<uint16_t>(timing_budget_ms + 4U));

  status = sensor.SetTimingBudgetInMs(timing_budget_ms);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  status = sensor.SetInterMeasurementInMs(intermeasurement_ms);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }

  channel.initialized = true;
  channel.ranging_started = false;
  channel.last_error = 0;
  return true;
}

bool TofOverdoorCounter::start_all_ranging_() {
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

    channel.current_zone = ZONE_OUT;
    if (!this->set_channel_roi_(channel, channel.current_zone)) {
      ESP_LOGW(TAG, "Failed to select initial ROI on %s (err %d)", channel.source_label.c_str(), channel.last_error);
      continue;
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

  ESP_LOGI(TAG, "Started continuous ranging on %u sensors in a synchronized batch", static_cast<unsigned>(started_count));
  return started_count >= this->min_valid_sensors_;
}

bool TofOverdoorCounter::read_channel_(Channel &channel) {
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
    return false;
  }

  if (!ready) {
    return true;
  }

  const uint8_t zone_index = channel.current_zone >= SENSOR_ZONE_COUNT ? ZONE_OUT : channel.current_zone;
  auto &zone = channel.zones[zone_index];

  VL53L1X_Result_t result{};
  status = sensor.GetResult(&result);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.consecutive_errors++;
    return false;
  }

  status = sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.consecutive_errors++;
    return false;
  }

  const uint32_t now = millis();
  zone.raw_distance = result.Distance;
  zone.range_status = result.Status;
  zone.last_update_ms = now;

  channel.raw_distance = result.Distance;
  channel.range_status = result.Status;
  channel.signal_per_spad = result.SigPerSPAD;
  channel.ambient_rate = result.Ambient;
  channel.spad_count = result.NumSPADs;
  channel.last_update_ms = now;

  if (!this->range_result_is_valid_(result)) {
    zone.valid_measurement = false;
    zone.sample_rejected = true;
    zone.consecutive_invalid++;
    channel.last_error = 0;
    channel.consecutive_errors = 0;
    channel.last_read_duration_ms = millis() - started;
    this->refresh_channel_aggregate_from_zones_(channel);
    if (!this->switch_channel_zone_(channel)) {
      return false;
    }
    if (this->debug_logging_) {
      ESP_LOGD(TAG, "%s %s rejected range sample distance=%u status=%s signal=%u ambient=%u spads=%u",
               channel.sensor_label.c_str(), zone_name(zone_index), result.Distance, range_status_name(result.Status),
               result.SigPerSPAD, result.Ambient, result.NumSPADs);
    }
    return true;
  }

  this->update_zone_sampling_(zone, result.Distance);
  const float sampled_distance = zone.has_sampled_distance ? static_cast<float>(zone.sampled_distance)
                                                           : static_cast<float>(result.Distance);
  zone.filtered_distance =
      std::isnan(zone.filtered_distance) ? sampled_distance
                                         : (zone.filtered_distance * (1.0f - FILTER_ALPHA)) +
                                               (sampled_distance * FILTER_ALPHA);
  zone.has_reading = true;
  zone.valid_measurement = true;
  zone.sample_rejected = false;
  zone.consecutive_invalid = 0;
  zone.last_good_read_ms = now;
  channel.last_error = 0;
  channel.consecutive_errors = 0;
  channel.recovery_attempts = 0;
  channel.next_recovery_ms = 0;
  channel.last_read_duration_ms = millis() - started;
  this->refresh_channel_aggregate_from_zones_(channel);
  if (!this->switch_channel_zone_(channel)) {
    return false;
  }
  return true;
}

bool TofOverdoorCounter::range_result_is_valid_(const VL53L1X_Result_t &result) const {
  if (result.Distance < MIN_VALID_DISTANCE_MM || result.Distance > MAX_VALID_DISTANCE_MM) {
    return false;
  }
  return result.Status == RangeValid || result.Status == RangeValidNoWrapCheck;
}

void TofOverdoorCounter::update_zone_sampling_(ZoneState &zone, uint16_t distance) {
  const uint8_t capacity = std::min<uint8_t>(this->sampling_size_, zone.samples.size());
  zone.samples[zone.sample_head] = distance;
  zone.sample_head = static_cast<uint8_t>((zone.sample_head + 1U) % capacity);
  zone.sample_count = std::min<uint8_t>(static_cast<uint8_t>(zone.sample_count + 1U), capacity);

  std::array<uint16_t, 8> sorted{};
  std::copy_n(zone.samples.begin(), zone.sample_count, sorted.begin());
  std::sort(sorted.begin(), sorted.begin() + zone.sample_count);
  zone.sampled_distance = sorted[zone.sample_count / 2U];
  zone.has_sampled_distance = true;
}

float TofOverdoorCounter::zone_logic_distance_(const ZoneState &zone) const {
  if (!std::isnan(zone.filtered_distance)) {
    return zone.filtered_distance;
  }
  if (zone.has_sampled_distance) {
    return static_cast<float>(zone.sampled_distance);
  }
  if (zone.has_reading) {
    return static_cast<float>(zone.raw_distance);
  }
  return NAN;
}

void TofOverdoorCounter::refresh_channel_aggregate_from_zones_(Channel &channel) {
  bool any_reading = false;
  bool any_valid = false;
  float nearest = NAN;
  uint16_t nearest_raw = 0;
  uint8_t nearest_status = channel.range_status;
  uint32_t last_update = 0;
  uint32_t last_good = 0;
  uint8_t worst_invalid = 0;

  for (const auto &zone : channel.zones) {
    if (zone.has_reading) {
      any_reading = true;
      last_update = std::max(last_update, zone.last_update_ms);
    }
    if (zone.valid_measurement) {
      any_valid = true;
      last_good = std::max(last_good, zone.last_good_read_ms);
    }
    worst_invalid = std::max(worst_invalid, zone.consecutive_invalid);

    const float distance = this->zone_logic_distance_(zone);
    if (!zone.has_reading || std::isnan(distance)) {
      continue;
    }
    if (std::isnan(nearest) || distance < nearest) {
      nearest = distance;
      nearest_raw = zone.raw_distance;
      nearest_status = zone.range_status;
    }
  }

  channel.has_reading = any_reading;
  channel.valid_measurement = any_valid;
  channel.sample_rejected = any_reading && !any_valid;
  channel.consecutive_invalid = any_valid ? 0 : worst_invalid;
  channel.last_update_ms = last_update == 0 ? channel.last_update_ms : last_update;
  channel.last_good_read_ms = last_good == 0 ? channel.last_good_read_ms : last_good;

  if (!std::isnan(nearest)) {
    channel.raw_distance = nearest_raw;
    channel.filtered_distance = nearest;
    channel.median_distance = nearest;
    channel.sampled_distance = static_cast<uint16_t>(nearest);
    channel.has_sampled_distance = true;
    channel.range_status = nearest_status;
  }
}

float TofOverdoorCounter::channel_logic_distance_(const Channel &channel) const {
  float nearest = NAN;
  for (const auto &zone : channel.zones) {
    const float distance = this->zone_logic_distance_(zone);
    if (!zone.has_reading || std::isnan(distance)) {
      continue;
    }
    nearest = std::isnan(nearest) ? distance : std::min(nearest, distance);
  }
  if (!std::isnan(nearest)) {
    return nearest;
  }
  if (!std::isnan(channel.filtered_distance)) {
    return channel.filtered_distance;
  }
  if (channel.has_sampled_distance) {
    return static_cast<float>(channel.sampled_distance);
  }
  if (channel.has_reading) {
    return static_cast<float>(channel.raw_distance);
  }
  return NAN;
}

float TofOverdoorCounter::adaptive_trigger_delta_(const ZoneState &zone) const {
  const float noise = std::isnan(zone.noise) ? 0.0f : zone.noise;
  return std::max<float>(this->trigger_threshold_mm_, 80.0f + noise * 6.0f);
}

float TofOverdoorCounter::adaptive_release_delta_(const ZoneState &zone) const {
  const float noise = std::isnan(zone.noise) ? 0.0f : zone.noise;
  const float noise_floor = 35.0f + noise * 3.0f;
  return std::min<float>(this->adaptive_trigger_delta_(zone) * 0.70f,
                         std::max<float>(this->clear_threshold_mm_, noise_floor));
}

bool TofOverdoorCounter::set_channel_roi_(Channel &channel, uint8_t zone_index) {
  if (!channel.sensor) {
    channel.last_error = VL53L1_ERROR_CONTROL_INTERFACE;
    return false;
  }
  auto &sensor = *channel.sensor;
  auto status = sensor.SetROI(ROODE_ZONE_WIDTH, ROODE_ZONE_HEIGHT);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  status = sensor.SetROICenter(roi_center_for_zone(zone_index));
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  channel.current_zone = zone_index;
  return true;
}

bool TofOverdoorCounter::switch_channel_zone_(Channel &channel) {
  if (!channel.sensor) {
    channel.ranging_started = false;
    return false;
  }

  auto &sensor = *channel.sensor;
  const uint8_t next_zone = channel.current_zone == ZONE_OUT ? ZONE_IN : ZONE_OUT;
  sensor.StopRanging();
  delayMicroseconds(250);
  if (!this->set_channel_roi_(channel, next_zone)) {
    channel.ranging_started = false;
    return false;
  }
  const auto status = sensor.StartRanging();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.ranging_started = false;
    return false;
  }
  channel.ranging_started = true;
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
  if (!this->set_channel_roi_(channel, channel.current_zone >= SENSOR_ZONE_COUNT ? ZONE_OUT : channel.current_zone)) {
    channel.ranging_started = false;
    return false;
  }
  const auto status = sensor.StartRanging();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.ranging_started = false;
    return false;
  }

  channel.ranging_started = true;
  return true;
}

bool TofOverdoorCounter::recover_channel_(size_t index, const char *reason) {
  if (index >= this->channels_.size() || index >= this->xshut_pins_.size()) {
    return false;
  }

  auto &channel = this->channels_[index];
  ESP_LOGW(TAG, "Recovering %s after %s", channel.sensor_label.c_str(), reason);
  if (channel.sensor && channel.ranging_started) {
    channel.sensor->StopRanging();
  }
  channel.ranging_started = false;
  channel.initialized = false;
  this->set_xshut_(index, false);
  delay(15);

  // A power-cycled VL53L1X always returns at 0x29. Recreate the ULD object so
  // its internal address cannot remain stuck at the old runtime address.
  channel.sensor = std::make_unique<VL53L1X_ULD>();
  this->set_xshut_(index, true);
  delay(this->wake_delay_ms_);

  bool default_address_ready = this->probe_address_(0x29);
  if (!default_address_ready && this->recover_wire_()) {
    default_address_ready = this->probe_address_(0x29);
  }
  bool recovered = default_address_ready && this->wait_for_boot_(*channel.sensor) &&
                   this->set_temp_address_(*channel.sensor, channel.address);
  if (recovered) {
    delay(this->post_address_delay_ms_);
    recovered = this->configure_sensor_(channel) && this->restart_ranging_(channel);
  }

  if (recovered) {
    channel.consecutive_errors = 0;
    channel.consecutive_invalid = 0;
    channel.recovery_attempts = 0;
    channel.last_good_read_ms = 0;
    channel.next_recovery_ms = millis() + 2000U;
    ESP_LOGI(TAG, "%s recovered at 0x%02X", channel.sensor_label.c_str(), channel.address);
    return true;
  }

  this->set_xshut_(index, false);
  channel.initialized = false;
  channel.ranging_started = false;
  channel.recovery_attempts = std::min<uint8_t>(static_cast<uint8_t>(channel.recovery_attempts + 1U), 6);
  const uint32_t backoff = std::min<uint32_t>(SENSOR_RECOVERY_MAX_MS,
                                              SENSOR_RECOVERY_BASE_MS << channel.recovery_attempts);
  channel.next_recovery_ms = millis() + backoff;
  ESP_LOGW(TAG, "%s recovery failed; retrying in %u ms", channel.sensor_label.c_str(),
           static_cast<unsigned>(backoff));
  return false;
}

void TofOverdoorCounter::service_recovery_(uint32_t now) {
  // Recover at most one channel per update so a bad cable cannot monopolize
  // the ESP32 loop or starve Wi-Fi/API processing.
  for (size_t index = 0; index < this->channels_.size(); index++) {
    auto &channel = this->channels_[index];
    const bool hard_error = channel.consecutive_errors >= ERRORS_BEFORE_POWER_CYCLE;
    const bool long_stale = channel.initialized &&
                            ((channel.last_good_read_ms == 0 && (now - this->last_discovery_ms_) > 2000U) ||
                             (channel.last_good_read_ms != 0 &&
                              (now - channel.last_good_read_ms) > (STALE_READING_MS * 4U)));
    const bool missing = !channel.initialized;
    if (!hard_error && !long_stale && !missing) {
      continue;
    }
    if (channel.next_recovery_ms != 0 && static_cast<int32_t>(now - channel.next_recovery_ms) < 0) {
      continue;
    }
    this->recover_channel_(index, missing ? "sensor unavailable" : (hard_error ? "I2C errors" : "stale data"));
    break;
  }
}

void TofOverdoorCounter::init_preferences_() {
  if (this->persisted_state_ready_ || global_preferences == nullptr) {
    return;
  }
  this->persisted_state_pref_ =
      global_preferences->make_preference<PersistedState>(this->preference_key_(), true);
  this->persisted_state_ready_ = true;
}

uint32_t TofOverdoorCounter::preference_key_() const {
  return 0x544F4634UL ^ (static_cast<uint32_t>(this->base_address_) << 8) ^ this->sda_pin_ ^ (this->scl_pin_ << 16);
}

void TofOverdoorCounter::restore_persisted_calibration_(Channel &channel, size_t zone_index,
                                                        const PersistedZoneCalibration &persisted) {
  if (zone_index >= SENSOR_ZONE_COUNT || persisted.valid == 0) {
    return;
  }
  auto &zone = channel.zones[zone_index];
  zone.baseline = static_cast<float>(persisted.baseline_mm);
  zone.noise = static_cast<float>(std::max<uint16_t>(persisted.noise_mm, 1));
  zone.calibration_quality = persisted.quality;
  zone.calibrated = true;
}

TofOverdoorCounter::PersistedZoneCalibration TofOverdoorCounter::build_persisted_calibration_(
    const ZoneState &zone) const {
  PersistedZoneCalibration calibration{};
  if (!zone.calibrated || std::isnan(zone.baseline)) {
    return calibration;
  }
  calibration.valid = 1;
  calibration.baseline_mm = static_cast<uint16_t>(std::max(0.0f, zone.baseline));
  calibration.noise_mm = static_cast<uint16_t>(std::max(1.0f, std::isnan(zone.noise) ? 1.0f : zone.noise));
  calibration.quality = zone.calibration_quality;
  return calibration;
}

void TofOverdoorCounter::load_persisted_state_() {
  if (!this->persisted_state_ready_) {
    return;
  }

  PersistedState state{};
  bool loaded = this->persisted_state_pref_.load(&state) &&
                (state.version == PERSISTED_STATE_VERSION || state.version == 5);

  // Version 6 keeps the same persisted layout as version 5. It migrates the
  // conservative timing values so already-installed counters receive the
  // low-latency profile without losing counts, calibration or direction.
  if (loaded && state.version == 5) {
    state.version = PERSISTED_STATE_VERSION;
    state.debounce_ms = std::min<uint16_t>(state.debounce_ms, 25);
    state.cooldown_ms = std::min<uint16_t>(state.cooldown_ms, 80);
    state.min_active_duration_ms = std::min<uint16_t>(state.min_active_duration_ms, 25);
    this->state_dirty_ = true;
    ESP_LOGI(TAG, "Migrated saved counter timing from version 5 to version %u",
             static_cast<unsigned>(PERSISTED_STATE_VERSION));
  }

  if (!loaded && global_preferences != nullptr) {
    PersistedStateV4 legacy{};
    auto legacy_pref = global_preferences->make_preference<PersistedStateV4>(this->preference_key_(), true);
    if (legacy_pref.load(&legacy) && legacy.version == 4) {
      state.version = PERSISTED_STATE_VERSION;
      state.people_inside = legacy.people_inside;
      state.confirmed_in = legacy.confirmed_in;
      state.confirmed_out = legacy.confirmed_out;
      state.unsure_in = legacy.unsure_in;
      state.unsure_out = legacy.unsure_out;
      state.trigger_threshold_mm = legacy.trigger_threshold_mm;
      // Version 4 stored an absolute clear distance (typically 500 mm).
      // Version 5 stores release hysteresis as a baseline drop.
      state.clear_threshold_mm = std::min<uint16_t>(160, legacy.trigger_threshold_mm / 2U);
      state.baseline_tolerance_mm = legacy.baseline_tolerance_mm;
      state.minimum_clear_distance_mm = std::max<uint16_t>(1200, legacy.minimum_clear_distance_mm);
      state.debounce_ms = legacy.debounce_ms;
      state.detection_timeout_ms = std::max<uint16_t>(3000, legacy.detection_timeout_ms);
      state.cooldown_ms = std::min<uint16_t>(legacy.cooldown_ms, 180);
      state.blocked_timeout_ms = legacy.blocked_timeout_ms;
      state.standing_timeout_ms = legacy.standing_timeout_ms;
      state.min_event_sensors = std::max<uint16_t>(3, legacy.min_event_sensors);
      state.min_active_duration_ms = legacy.min_active_duration_ms;
      state.direction_window_ms = legacy.direction_window_ms;
      state.calibration_samples = legacy.calibration_samples;
      state.max_people_inside = legacy.max_people_inside;
      state.min_valid_sensors = legacy.min_valid_sensors;
      state.auto_save_enabled = legacy.auto_save_enabled;
      state.invert_direction = legacy.invert_direction;
      state.debug_logging = legacy.debug_logging;
      for (size_t index = 0; index < SENSOR_COUNT; index++) {
        for (size_t zone_index = 0; zone_index < SENSOR_ZONE_COUNT; zone_index++) {
          // V4 stored one nearest-distance baseline per sensor. It cannot be
          // safely reused as two independent ROI baselines.
          state.calibrations[index][zone_index].valid = 0;
        }
      }
      loaded = true;
      this->state_dirty_ = true;
      ESP_LOGI(TAG, "Migrated saved counter state from version 4 to version %u",
               static_cast<unsigned>(PERSISTED_STATE_VERSION));
    }
  }

  if (!loaded && global_preferences != nullptr) {
    PersistedStateV3 legacy{};
    auto legacy_pref = global_preferences->make_preference<PersistedStateV3>(this->preference_key_(), true);
    if (legacy_pref.load(&legacy) && legacy.version == 3) {
      state.version = PERSISTED_STATE_VERSION;
      state.people_inside = legacy.people_inside;
      state.confirmed_in = legacy.confirmed_in;
      state.confirmed_out = legacy.confirmed_out;
      state.unsure_in = legacy.unsure_in;
      state.unsure_out = legacy.unsure_out;
      state.trigger_threshold_mm = legacy.trigger_threshold_mm;
      state.clear_threshold_mm = 160;
      state.baseline_tolerance_mm = legacy.baseline_tolerance_mm;
      state.minimum_clear_distance_mm = std::max<uint16_t>(1200, legacy.minimum_clear_distance_mm);
      state.debounce_ms = legacy.debounce_ms;
      state.detection_timeout_ms = std::max<uint16_t>(3000, legacy.detection_timeout_ms);
      state.cooldown_ms = std::min<uint16_t>(legacy.cooldown_ms, 180);
      state.blocked_timeout_ms = legacy.blocked_timeout_ms;
      state.standing_timeout_ms = legacy.standing_timeout_ms;
      state.calibration_samples = legacy.calibration_samples;
      state.max_people_inside = legacy.max_people_inside;
      state.min_valid_sensors = legacy.min_valid_sensors;
      state.auto_save_enabled = legacy.auto_save_enabled;
      state.invert_direction = legacy.invert_direction;
      for (size_t index = 0; index < SENSOR_COUNT; index++) {
        for (size_t zone_index = 0; zone_index < SENSOR_ZONE_COUNT; zone_index++) {
          state.calibrations[index][zone_index].valid = 0;
        }
      }
      loaded = true;
      this->state_dirty_ = true;
      ESP_LOGI(TAG, "Migrated saved counter state from version 3 to version %u",
               static_cast<unsigned>(PERSISTED_STATE_VERSION));
    }
  }

  if (!loaded) {
    this->apply_calibration_defaults_();
    return;
  }

  const uint16_t persisted_max_people = sanitize_persisted<uint16_t>(state.max_people_inside, 1, 5000, 50);
  this->people_inside_ = std::max<int32_t>(0, std::min<int32_t>(state.people_inside, persisted_max_people));
  this->confirmed_in_count_ = state.confirmed_in;
  this->confirmed_out_count_ = state.confirmed_out;
  this->unsure_in_count_ = state.unsure_in;
  this->unsure_out_count_ = state.unsure_out;
  this->rejected_count_ = state.rejected;
  this->trigger_threshold_mm_ = sanitize_persisted<uint16_t>(state.trigger_threshold_mm, 40, 3000, 320);
  this->clear_threshold_mm_ = sanitize_persisted<uint16_t>(state.clear_threshold_mm, 20, 1000, 160);
  this->baseline_tolerance_mm_ = sanitize_persisted<uint16_t>(state.baseline_tolerance_mm, 10, 500, 80);
  this->minimum_clear_distance_mm_ = sanitize_persisted<uint16_t>(state.minimum_clear_distance_mm, 100, 4000, 600);
  this->debounce_ms_ = sanitize_persisted<uint16_t>(state.debounce_ms, 5, 5000, 25);
  this->detection_timeout_ms_ = sanitize_persisted<uint16_t>(state.detection_timeout_ms, 200, 20000, 1600);
  this->cooldown_ms_ = sanitize_persisted<uint16_t>(state.cooldown_ms, 0, 20000, 80);
  this->blocked_timeout_ms_ = sanitize_persisted<uint16_t>(state.blocked_timeout_ms, 200, 60000, 1800);
  this->standing_timeout_ms_ = sanitize_persisted<uint16_t>(state.standing_timeout_ms, 200, 60000, 2200);
  this->min_event_sensors_ = sanitize_persisted<uint16_t>(state.min_event_sensors, 2, SENSOR_COUNT, 3);
  this->min_active_duration_ms_ = sanitize_persisted<uint16_t>(state.min_active_duration_ms, 0, 1000, 25);
  this->direction_window_ms_ = sanitize_persisted<uint16_t>(state.direction_window_ms, 10, 1000, 90);
  this->calibration_samples_ = sanitize_persisted<uint16_t>(state.calibration_samples, 4, 128, 24);
  this->max_people_inside_ = sanitize_persisted<uint16_t>(state.max_people_inside, 1, 5000, 50);
  this->min_valid_sensors_ = sanitize_persisted<uint8_t>(state.min_valid_sensors, 2, SENSOR_COUNT, 3);
  this->auto_save_enabled_ = state.auto_save_enabled != 0;
  this->invert_direction_ = state.invert_direction != 0;
  this->debug_logging_ = state.debug_logging != 0;

  if (this->clear_threshold_mm_ >= this->trigger_threshold_mm_) {
    this->clear_threshold_mm_ = static_cast<uint16_t>(this->trigger_threshold_mm_ / 2U);
  }
  this->apply_calibration_defaults_();

  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    for (size_t zone_index = 0; zone_index < SENSOR_ZONE_COUNT; zone_index++) {
      this->restore_persisted_calibration_(this->channels_[index], zone_index, state.calibrations[index][zone_index]);
    }
    auto &channel = this->channels_[index];
    if (channel.zones[ZONE_OUT].calibrated && channel.zones[ZONE_IN].calibrated) {
      channel.calibrated = true;
      channel.baseline = (channel.zones[ZONE_OUT].baseline + channel.zones[ZONE_IN].baseline) * 0.5f;
      channel.noise = std::max(channel.zones[ZONE_OUT].noise, channel.zones[ZONE_IN].noise);
      channel.calibration_quality = std::min(channel.zones[ZONE_OUT].calibration_quality,
                                             channel.zones[ZONE_IN].calibration_quality);
    }
  }

  ESP_LOGI(TAG,
           "Loaded saved counter state people=%d in=%u out=%u unsure_in=%u unsure_out=%u trigger=%u clear=%u min_valid=%u",
           this->people_inside_, static_cast<unsigned>(this->confirmed_in_count_),
           static_cast<unsigned>(this->confirmed_out_count_), static_cast<unsigned>(this->unsure_in_count_),
           static_cast<unsigned>(this->unsure_out_count_), static_cast<unsigned>(this->trigger_threshold_mm_),
           static_cast<unsigned>(this->clear_threshold_mm_), static_cast<unsigned>(this->min_valid_sensors_));
}

void TofOverdoorCounter::apply_calibration_defaults_() {
  this->i2c_frequency_ = sanitize_persisted<uint32_t>(this->i2c_frequency_, 100000, 400000, 400000);
  this->timing_budget_ms_ = sanitize_persisted<uint16_t>(this->timing_budget_ms_, 20, 1000, 33);
  this->intermeasurement_ms_ = sanitize_persisted<uint16_t>(this->intermeasurement_ms_, 20, 5000, 33);
  this->sampling_size_ = sanitize_persisted<uint8_t>(this->sampling_size_, 1, 8, 3);
  this->trigger_threshold_mm_ = sanitize_persisted<uint16_t>(this->trigger_threshold_mm_, 40, 3000, 320);
  this->clear_threshold_mm_ = sanitize_persisted<uint16_t>(this->clear_threshold_mm_, 20, 1000, 160);
  this->baseline_tolerance_mm_ = sanitize_persisted<uint16_t>(this->baseline_tolerance_mm_, 10, 500, 80);
  this->minimum_clear_distance_mm_ = sanitize_persisted<uint16_t>(this->minimum_clear_distance_mm_, 100, 4000, 600);
  this->debounce_ms_ = sanitize_persisted<uint32_t>(this->debounce_ms_, 5, 5000, 25);
  this->detection_timeout_ms_ = sanitize_persisted<uint32_t>(this->detection_timeout_ms_, 200, 20000, 1600);
  this->cooldown_ms_ = sanitize_persisted<uint32_t>(this->cooldown_ms_, 0, 20000, 80);
  this->blocked_timeout_ms_ = sanitize_persisted<uint32_t>(this->blocked_timeout_ms_, 200, 60000, 1800);
  this->standing_timeout_ms_ = sanitize_persisted<uint32_t>(this->standing_timeout_ms_, 200, 60000, 2200);
  this->min_event_sensors_ = sanitize_persisted<uint8_t>(this->min_event_sensors_, 2, SENSOR_COUNT, 3);
  this->min_active_duration_ms_ = sanitize_persisted<uint32_t>(this->min_active_duration_ms_, 0, 1000, 25);
  this->direction_window_ms_ = sanitize_persisted<uint32_t>(this->direction_window_ms_, 10, 1000, 90);
  this->calibration_samples_ = sanitize_persisted<uint16_t>(this->calibration_samples_, 4, 128, 24);
  this->max_people_inside_ = sanitize_persisted<uint16_t>(this->max_people_inside_, 1, 5000, 50);
  this->min_valid_sensors_ = sanitize_persisted<uint8_t>(this->min_valid_sensors_, 2, SENSOR_COUNT, 3);

  const uint16_t min_timing_budget = this->distance_mode_ == DISTANCE_MODE_SHORT ? 20 : 33;
  if (this->timing_budget_ms_ < min_timing_budget) {
    this->timing_budget_ms_ = min_timing_budget;
  }
  if (this->intermeasurement_ms_ < (this->timing_budget_ms_ + 4U)) {
    this->intermeasurement_ms_ = static_cast<uint16_t>(this->timing_budget_ms_ + 4U);
  }
  if (this->clear_threshold_mm_ >= this->trigger_threshold_mm_) {
    this->clear_threshold_mm_ = static_cast<uint16_t>(this->trigger_threshold_mm_ / 2U);
  }
}

void TofOverdoorCounter::persist_runtime_state() {
  if (!this->persisted_state_ready_) {
    return;
  }

  PersistedState state{};
  state.version = PERSISTED_STATE_VERSION;
  state.people_inside = this->people_inside_;
  state.confirmed_in = this->confirmed_in_count_;
  state.confirmed_out = this->confirmed_out_count_;
  state.unsure_in = this->unsure_in_count_;
  state.unsure_out = this->unsure_out_count_;
  state.rejected = this->rejected_count_;
  state.trigger_threshold_mm = this->trigger_threshold_mm_;
  state.clear_threshold_mm = this->clear_threshold_mm_;
  state.baseline_tolerance_mm = this->baseline_tolerance_mm_;
  state.minimum_clear_distance_mm = this->minimum_clear_distance_mm_;
  state.debounce_ms = static_cast<uint16_t>(this->debounce_ms_);
  state.detection_timeout_ms = static_cast<uint16_t>(this->detection_timeout_ms_);
  state.cooldown_ms = static_cast<uint16_t>(this->cooldown_ms_);
  state.blocked_timeout_ms = static_cast<uint16_t>(this->blocked_timeout_ms_);
  state.standing_timeout_ms = static_cast<uint16_t>(this->standing_timeout_ms_);
  state.min_event_sensors = this->min_event_sensors_;
  state.min_active_duration_ms = static_cast<uint16_t>(this->min_active_duration_ms_);
  state.direction_window_ms = static_cast<uint16_t>(this->direction_window_ms_);
  state.calibration_samples = this->calibration_samples_;
  state.max_people_inside = this->max_people_inside_;
  state.min_valid_sensors = this->min_valid_sensors_;
  state.auto_save_enabled = this->auto_save_enabled_ ? 1 : 0;
  state.invert_direction = this->invert_direction_ ? 1 : 0;
  state.debug_logging = this->debug_logging_ ? 1 : 0;
  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    for (size_t zone_index = 0; zone_index < SENSOR_ZONE_COUNT; zone_index++) {
      state.calibrations[index][zone_index] = this->build_persisted_calibration_(this->channels_[index].zones[zone_index]);
    }
  }

  this->persisted_state_pref_.save(&state);
  // Let ESPHome's flash_write_interval batch NVS commits. A forced sync per
  // passage blocks the loop and causes unnecessary flash wear.
  this->state_dirty_ = false;
}

void TofOverdoorCounter::rediscover() {
  ESP_LOGI(TAG, "Starting four-sensor discovery");
  this->channels_.clear();
  this->channels_.resize(this->xshut_pins_.size());
  this->clear_event_tracking_();
  this->startup_clear_validated_ = false;
  this->boot_clear_since_ms_ = 0;
  this->update_passage_state_(PASSAGE_IDLE);
  this->set_all_xshut_(false);
  delay(this->wake_delay_ms_);

  for (size_t index = 0; index < this->xshut_pins_.size(); index++) {
    auto &channel = this->channels_[index];
    channel.pin_number = this->xshut_pin_numbers_[index];
    // Address is tied to the physical slot, not discovery order. This makes
    // later isolated recovery safe even if an earlier sensor was absent.
    channel.address = static_cast<uint8_t>(this->base_address_ + index);
    channel.group = group_for_pin(channel.pin_number);
    channel.sensor_label = sensor_name_for_pin(channel.pin_number);
    channel.source_label = channel.sensor_label + std::string(" / fused two-zone track / GPIO") +
                           std::to_string(channel.pin_number);

    this->set_xshut_(index, true);
    delay(this->wake_delay_ms_);

    if (!this->probe_address_(0x29)) {
      ESP_LOGW(TAG, "No sensor ACK on %s", channel.source_label.c_str());
      this->set_xshut_(index, false);
      channel.next_recovery_ms = millis() + SENSOR_RECOVERY_BASE_MS;
      continue;
    }

    channel.sensor = std::make_unique<VL53L1X_ULD>();
    if (!this->wait_for_boot_(*channel.sensor)) {
      ESP_LOGW(TAG, "%s ACKed but never reported boot-ready", channel.source_label.c_str());
      this->set_xshut_(index, false);
      channel.next_recovery_ms = millis() + SENSOR_RECOVERY_BASE_MS;
      continue;
    }

    if (!this->set_temp_address_(*channel.sensor, channel.address)) {
      ESP_LOGW(TAG, "%s address change to 0x%02X failed", channel.source_label.c_str(), channel.address);
      this->set_xshut_(index, false);
      channel.next_recovery_ms = millis() + SENSOR_RECOVERY_BASE_MS;
      continue;
    }

    delay(this->post_address_delay_ms_);

    if (!this->configure_sensor_(channel)) {
      ESP_LOGW(TAG, "%s configuration failed (err %d)", channel.source_label.c_str(), channel.last_error);
      this->set_xshut_(index, false);
      channel.next_recovery_ms = millis() + SENSOR_RECOVERY_BASE_MS;
      continue;
    }

    ESP_LOGI(TAG, "Discovered sensor on %s at 0x%02X", channel.source_label.c_str(), channel.address);
  }

  this->start_all_ranging_();

  this->last_discovery_ms_ = millis();
  if (!this->persisted_state_loaded_) {
    this->load_persisted_state_();
    this->persisted_state_loaded_ = true;
  }
  if (!this->has_restored_calibration_()) {
    this->recalibrate();
  } else {
    this->calibration_active_ = false;
    this->phase_text_ = "Restored calibration, waiting for live sensor readings";
    this->system_status_ = STATUS_BOOTING;
    ESP_LOGI(TAG, "Restored calibration from persisted state; skipping auto recalibration on boot");
  }

  ESP_LOGI(TAG, "Discovery complete: %u sensors active", static_cast<unsigned>(this->get_discovered_sensor_count()));
}

void TofOverdoorCounter::recalibrate() {
  this->calibration_active_ = true;
  this->calibration_started_ms_ = millis();
  this->calibration_clear_since_ms_ = 0;
  this->person_standing_in_door_ = false;
  this->blocked_sensor_text_ = "None";
  this->phase_text_ = "Waiting for clear doorway to calibrate";
  this->clear_event_tracking_();
  this->update_passage_state_(PASSAGE_IDLE);
  this->cooldown_until_ms_ = 0;
  this->boot_clear_since_ms_ = 0;
  this->startup_clear_validated_ = false;

  for (auto &channel : this->channels_) {
    channel.active = false;
    channel.blocked = false;
    channel.valid_measurement = false;
    channel.sample_rejected = false;
    channel.rising_edge = false;
    channel.falling_edge = false;
    channel.active_candidate_since_ms = 0;
    channel.clear_candidate_since_ms = 0;
    channel.active_since_ms = 0;
    channel.last_rising_ms = 0;
    channel.last_falling_ms = 0;
    channel.active_duration_ms = 0;
    channel.calibrated = false;
    channel.baseline = NAN;
    channel.noise = NAN;
    channel.calibration_quality = 0;
    channel.has_sampled_distance = false;
    channel.sampled_distance = 0;
    channel.calibration_sum = 0.0f;
    channel.calibration_sq_sum = 0.0f;
    channel.calibration_min = NAN;
    channel.calibration_max = NAN;
    channel.calibration_samples = 0;
    this->reset_channel_path_tracker_(channel);
    for (auto &zone : channel.zones) {
      zone = ZoneState{};
    }
  }

  ESP_LOGI(TAG, "Calibration requested - waiting for a stable and empty doorway");
}

void TofOverdoorCounter::reset_counts() {
  this->people_inside_ = 0;
  this->confirmed_in_count_ = 0;
  this->confirmed_out_count_ = 0;
  this->last_direction_ = "Reset";
  this->last_reason_ = "Confirmed people counters reset";
  this->state_dirty_ = true;
}

void TofOverdoorCounter::set_people_inside(int value) {
  this->people_inside_ = std::max(0, std::min(value, static_cast<int>(this->max_people_inside_)));
  this->last_reason_ = "People count corrected manually";
  this->state_dirty_ = true;
}

void TofOverdoorCounter::reset_unsure_in() {
  this->unsure_in_count_ = 0;
  this->last_reason_ = "Unsure IN counter reset";
  this->state_dirty_ = true;
}

void TofOverdoorCounter::reset_unsure_out() {
  this->unsure_out_count_ = 0;
  this->last_reason_ = "Unsure OUT counter reset";
  this->state_dirty_ = true;
}

void TofOverdoorCounter::reset_all_counters() {
  this->people_inside_ = 0;
  this->confirmed_in_count_ = 0;
  this->confirmed_out_count_ = 0;
  this->unsure_in_count_ = 0;
  this->unsure_out_count_ = 0;
  this->rejected_count_ = 0;
  this->last_direction_ = "Reset";
  this->last_reason_ = "All counters reset";
  this->state_dirty_ = true;
}

void TofOverdoorCounter::reset_trace_buffer() {
  this->history_head_ = 0;
  this->history_count_ = 0;
  this->event_log_count_ = 0;
  std::fill(std::begin(this->history_), std::end(this->history_), HistorySample{});
  std::fill(std::begin(this->event_log_), std::end(this->event_log_), std::string{});
  this->last_reason_ = "Trace buffer reset";
}

void TofOverdoorCounter::set_invert_direction(bool invert_direction) {
  if (this->invert_direction_ == invert_direction) {
    return;
  }
  this->invert_direction_ = invert_direction;
  this->cooldown_until_ms_ = 0;
  this->person_standing_in_door_ = false;
  this->clear_event_tracking_();
  for (auto &channel : this->channels_) {
    this->reset_channel_path_tracker_(channel);
  }
  this->last_reason_ = invert_direction ? "Direction reversed immediately" : "Direction restored immediately";
  this->state_dirty_ = true;
}

void TofOverdoorCounter::process_calibration_() {
  const uint32_t now = millis();
  uint8_t healthy_sensors = 0;
  uint8_t clear_sensors = 0;

  for (auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    bool both_zones_healthy = true;
    bool both_zones_clear = true;
    for (const auto &zone : channel.zones) {
      const float sample = this->zone_logic_distance_(zone);
      const bool zone_stale = zone.last_good_read_ms == 0 || (now - zone.last_good_read_ms) > STALE_READING_MS;
      both_zones_healthy = both_zones_healthy && zone.has_reading && !zone_stale &&
                           zone.consecutive_invalid < 3 && !std::isnan(sample);
      both_zones_clear = both_zones_clear && !std::isnan(sample) && sample >= this->minimum_clear_distance_mm_;
    }
    if (both_zones_healthy) {
      healthy_sensors++;
      if (both_zones_clear) {
        clear_sensors++;
      }
    }
  }

  if (healthy_sensors < this->min_valid_sensors_) {
    this->calibration_clear_since_ms_ = 0;
    this->phase_text_ = "Waiting for enough healthy sensors to calibrate";
    return;
  }

  if (clear_sensors < this->min_valid_sensors_) {
    this->calibration_clear_since_ms_ = 0;
    this->phase_text_ = "Waiting for clear doorway to calibrate";
    return;
  }

  if (this->calibration_clear_since_ms_ == 0) {
    this->calibration_clear_since_ms_ = now;
  }

  const uint32_t clear_stable_ms = now - this->calibration_clear_since_ms_;
  if (clear_stable_ms < CALIBRATION_CLEAR_SETTLE_MS) {
    this->phase_text_ = "Waiting for doorway to stay clear (" + std::to_string(clear_stable_ms) + "/" +
                        std::to_string(CALIBRATION_CLEAR_SETTLE_MS) + " ms)";
    return;
  }

  for (auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    for (auto &zone : channel.zones) {
      const float sample = this->zone_logic_distance_(zone);
      const bool zone_stale = zone.last_good_read_ms == 0 || (now - zone.last_good_read_ms) > STALE_READING_MS;
      if (!zone.has_reading || zone_stale || zone.consecutive_invalid >= 3 || std::isnan(sample) ||
          sample < this->minimum_clear_distance_mm_ || zone.calibration_samples >= this->calibration_samples_) {
        continue;
      }
      zone.calibration_sum += sample;
      zone.calibration_sq_sum += sample * sample;
      zone.calibration_min = std::isnan(zone.calibration_min) ? sample : std::min(zone.calibration_min, sample);
      zone.calibration_max = std::isnan(zone.calibration_max) ? sample : std::max(zone.calibration_max, sample);
      zone.calibration_samples++;
    }
  }

  std::array<uint16_t, SENSOR_COUNT> sample_counts{};
  size_t sample_count_size = 0;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    sample_counts[sample_count_size++] =
        std::min(channel.zones[ZONE_OUT].calibration_samples, channel.zones[ZONE_IN].calibration_samples);
  }
  std::sort(sample_counts.begin(), sample_counts.begin() + sample_count_size, std::greater<uint16_t>());
  const size_t required_rank = std::min<size_t>(std::max<uint8_t>(1, this->min_valid_sensors_), sample_count_size);
  const uint16_t samples = sample_count_size == 0 ? 0 : sample_counts[required_rank - 1];
  this->phase_text_ =
      "Collecting calibration samples (" + std::to_string(samples) + "/" + std::to_string(this->calibration_samples_) + ")";

  if (samples < this->calibration_samples_) {
    return;
  }

  uint8_t stable_channels = 0;
  std::array<std::array<float, SENSOR_ZONE_COUNT>, SENSOR_COUNT> baselines{};
  std::array<std::array<float, SENSOR_ZONE_COUNT>, SENSOR_COUNT> noises{};
  std::array<std::array<uint8_t, SENSOR_ZONE_COUNT>, SENSOR_COUNT> qualities{};
  for (auto &sensor_values : baselines) sensor_values.fill(NAN);
  for (auto &sensor_values : noises) sensor_values.fill(NAN);
  for (size_t index = 0; index < this->channels_.size(); index++) {
    auto &channel = this->channels_[index];
    if (!channel.initialized) {
      continue;
    }
    bool channel_stable = true;
    for (size_t zone_index = 0; zone_index < SENSOR_ZONE_COUNT; zone_index++) {
      auto &zone = channel.zones[zone_index];
      if (zone.calibration_samples < this->calibration_samples_) {
        channel_stable = false;
        break;
      }
      const float count = static_cast<float>(zone.calibration_samples);
      const float mean = zone.calibration_sum / count;
      const float variance = std::max(0.0f, (zone.calibration_sq_sum / count) - (mean * mean));
      const float stddev = sqrtf(variance);
      const float span = zone.calibration_max - zone.calibration_min;
      if (span > static_cast<float>(this->baseline_tolerance_mm_ * 4U) ||
          stddev > static_cast<float>(this->baseline_tolerance_mm_)) {
        ESP_LOGW(TAG, "Calibration unstable on %s %s: mean=%.1f stddev=%.1f span=%.1f",
                 channel.sensor_label.c_str(), zone_name(zone_index), mean, stddev, span);
        channel_stable = false;
        break;
      }
      baselines[index][zone_index] = mean;
      noises[index][zone_index] = std::max(1.0f, stddev);
      qualities[index][zone_index] = clamp_quality(100.0f - (stddev * 2.5f) - (span * 0.3f));
    }
    if (channel_stable) {
      stable_channels++;
    } else {
      for (auto &zone : channel.zones) {
        zone.calibration_sum = 0.0f;
        zone.calibration_sq_sum = 0.0f;
        zone.calibration_min = NAN;
        zone.calibration_max = NAN;
        zone.calibration_samples = 0;
      }
    }
  }

  if (stable_channels < this->min_valid_sensors_) {
    this->phase_text_ = "Calibration still collecting stable samples";
    return;
  }

  for (size_t index = 0; index < this->channels_.size(); index++) {
    auto &channel = this->channels_[index];
    if (!channel.initialized || std::isnan(baselines[index][ZONE_OUT]) || std::isnan(baselines[index][ZONE_IN])) {
      continue;
    }
    for (size_t zone_index = 0; zone_index < SENSOR_ZONE_COUNT; zone_index++) {
      auto &zone = channel.zones[zone_index];
      zone.baseline = baselines[index][zone_index];
      zone.noise = noises[index][zone_index];
      zone.calibration_quality = qualities[index][zone_index];
      zone.calibrated = true;
    }
    channel.baseline = (baselines[index][ZONE_OUT] + baselines[index][ZONE_IN]) * 0.5f;
    channel.noise = std::max(noises[index][ZONE_OUT], noises[index][ZONE_IN]);
    channel.calibration_quality = std::min(qualities[index][ZONE_OUT], qualities[index][ZONE_IN]);
    channel.calibrated = true;
  }

  this->calibration_active_ = false;
  this->calibration_clear_since_ms_ = 0;
  this->phase_text_ = "Calibration completed";
  this->last_reason_ = "Calibration completed successfully";
  this->state_dirty_ = true;
  this->persist_runtime_state();
  ESP_LOGI(TAG, "Calibration completed successfully");
}

void TofOverdoorCounter::update_sensor_states_() {
  const uint32_t now = millis();

  for (auto &channel : this->channels_) {
    const bool was_active = channel.active;
    const uint32_t previous_active_since = channel.active_since_ms;
    channel.rising_edge = false;
    channel.falling_edge = false;

    if (!channel.initialized || !channel.has_reading || channel.stale || channel.consecutive_invalid >= 3) {
      for (auto &zone : channel.zones) {
        zone.rising_edge = false;
        zone.falling_edge = false;
        zone.active = false;
        zone.blocked = false;
        zone.active_candidate_since_ms = 0;
        zone.clear_candidate_since_ms = 0;
        zone.active_since_ms = 0;
        zone.active_duration_ms = 0;
      }
      channel.active = false;
      channel.blocked = false;
      channel.active_candidate_since_ms = 0;
      channel.clear_candidate_since_ms = 0;
      channel.active_since_ms = 0;
      if (was_active) {
        channel.falling_edge = true;
        channel.last_falling_ms = now;
      }
      continue;
    }

    for (auto &zone : channel.zones) {
      zone.rising_edge = false;
      zone.falling_edge = false;
      const float distance = this->zone_logic_distance_(zone);
      const bool zone_stale = zone.last_good_read_ms == 0 || (now - zone.last_good_read_ms) > STALE_READING_MS;

      if (!zone.has_reading || !zone.calibrated || std::isnan(zone.baseline) || zone_stale ||
          zone.consecutive_invalid >= 3 || std::isnan(distance)) {
        if (zone.active) {
          zone.falling_edge = true;
          zone.last_falling_ms = now;
        }
        zone.active = false;
        zone.blocked = false;
        zone.active_candidate_since_ms = 0;
        zone.clear_candidate_since_ms = 0;
        zone.active_since_ms = 0;
        zone.active_duration_ms = 0;
        continue;
      }

      const float drop = zone.baseline - distance;
      const float trigger_delta = this->adaptive_trigger_delta_(zone);
      const float release_delta = this->adaptive_release_delta_(zone);

      if (!zone.active) {
        if (drop >= trigger_delta) {
          if (zone.active_candidate_since_ms == 0) {
            zone.active_candidate_since_ms = now;
          } else if ((now - zone.active_candidate_since_ms) >= this->debounce_ms_) {
            zone.active = true;
            zone.active_since_ms = zone.active_candidate_since_ms;
            zone.last_rising_ms = zone.active_since_ms;
            zone.active_duration_ms = 0;
            zone.rising_edge = true;
            zone.clear_candidate_since_ms = 0;
          }
        } else {
          zone.active_candidate_since_ms = 0;
        }
      } else {
        if (drop <= release_delta) {
          if (zone.clear_candidate_since_ms == 0) {
            zone.clear_candidate_since_ms = now;
          } else if ((now - zone.clear_candidate_since_ms) >= this->debounce_ms_) {
            zone.active = false;
            zone.blocked = false;
            zone.last_falling_ms = now;
            zone.active_duration_ms = zone.active_since_ms == 0 ? 0 : now - zone.active_since_ms;
            zone.falling_edge = true;
            zone.active_candidate_since_ms = 0;
            zone.clear_candidate_since_ms = 0;
            zone.active_since_ms = 0;
          }
        } else {
          zone.clear_candidate_since_ms = 0;
        }
      }

      if (zone.active && zone.active_since_ms != 0 && (now - zone.active_since_ms) >= this->blocked_timeout_ms_) {
        zone.blocked = true;
      }
      if (zone.active && zone.active_since_ms != 0) {
        zone.active_duration_ms = now - zone.active_since_ms;
      }
    }

    channel.active = false;
    channel.blocked = false;
    channel.active_since_ms = 0;
    channel.active_duration_ms = 0;
    for (const auto &zone : channel.zones) {
      if (!zone.active) {
        continue;
      }
      channel.active = true;
      channel.blocked = channel.blocked || zone.blocked;
      if (channel.active_since_ms == 0 || (zone.active_since_ms != 0 && zone.active_since_ms < channel.active_since_ms)) {
        channel.active_since_ms = zone.active_since_ms;
      }
    }

    if (channel.active && channel.active_since_ms != 0) {
      channel.active_duration_ms = now - channel.active_since_ms;
    }
    if (!was_active && channel.active) {
      channel.rising_edge = true;
      channel.last_rising_ms = channel.active_since_ms != 0 ? channel.active_since_ms : now;
      channel.active_candidate_since_ms = channel.last_rising_ms;
      channel.clear_candidate_since_ms = 0;
    } else if (was_active && !channel.active) {
      channel.falling_edge = true;
      channel.last_falling_ms = now;
      channel.active_duration_ms = previous_active_since == 0 ? 0 : now - previous_active_since;
      channel.active_candidate_since_ms = 0;
      channel.clear_candidate_since_ms = 0;
    }
  }

  if (!this->startup_clear_validated_) {
    if (this->active_sensor_count_() == 0 && this->reporting_sensor_count_() >= this->min_valid_sensors_) {
      if (this->boot_clear_since_ms_ == 0) {
        this->boot_clear_since_ms_ = now;
      } else if ((now - this->boot_clear_since_ms_) >= BOOT_CLEAR_SETTLE_MS) {
        this->startup_clear_validated_ = true;
      }
    } else {
      this->boot_clear_since_ms_ = 0;
    }
  }
}

void TofOverdoorCounter::apply_idle_baseline_tracking_() {
  if (this->event_active_ || this->calibration_active_) {
    return;
  }

  for (auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading || !channel.calibrated || channel.active || channel.blocked) {
      continue;
    }
    for (auto &zone : channel.zones) {
      const float distance = this->zone_logic_distance_(zone);
      if (!zone.calibrated || zone.active || std::isnan(distance) || std::isnan(zone.baseline)) {
        continue;
      }
      const float delta = fabsf(distance - zone.baseline);
      if (delta > static_cast<float>(this->baseline_tolerance_mm_)) {
        continue;
      }
      zone.baseline = (zone.baseline * (1.0f - BASELINE_TRACK_ALPHA)) + (distance * BASELINE_TRACK_ALPHA);
      const float deviation = fabsf(distance - zone.baseline);
      zone.noise = std::isnan(zone.noise) ? deviation
                                          : (zone.noise * (1.0f - NOISE_TRACK_ALPHA)) +
                                                (deviation * NOISE_TRACK_ALPHA);
    }
    channel.baseline = (channel.zones[ZONE_OUT].baseline + channel.zones[ZONE_IN].baseline) * 0.5f;
    channel.noise = std::max(channel.zones[ZONE_OUT].noise, channel.zones[ZONE_IN].noise);
  }
}

void TofOverdoorCounter::update_blocked_state_() {
  this->blocked_sensor_text_ = "None";
  bool any_blocked = false;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized || !channel.blocked) {
      continue;
    }
    if (!any_blocked) {
      this->blocked_sensor_text_ = channel.sensor_label;
    } else {
      this->blocked_sensor_text_ += ", " + channel.sensor_label;
    }
    any_blocked = true;
  }
}

void TofOverdoorCounter::clear_event_tracking_() {
  this->event_active_ = false;
  this->event_started_ms_ = 0;
  this->event_last_activity_ms_ = 0;
  this->standing_clear_since_ms_ = 0;
  this->event_first_group_ = GROUP_NONE;
  this->event_second_group_ = GROUP_NONE;
  this->event_direction_group_ = GROUP_NONE;
  this->event_sensor_mask_ = 0;
  this->event_rising_mask_ = 0;
  this->event_falling_mask_ = 0;
  this->event_peak_active_count_ = 0;
  this->event_peak_group_counts_[0] = 0;
  this->event_peak_group_counts_[1] = 0;
  this->event_group_confirmed_ms_[GROUP_OUT] = 0;
  this->event_group_confirmed_ms_[GROUP_IN] = 0;
  this->event_direction_decided_ms_ = 0;
  this->event_first_edge_ms_ = 0;
  this->event_last_edge_ms_ = 0;
  this->event_edge_count_ = 0;
  this->sensor_vote_count_ = 0;
  this->event_path_size_ = 0;
  this->event_last_state_code_ = GROUP_STATE_NONE;
  std::fill(std::begin(this->event_path_), std::end(this->event_path_), GROUP_STATE_NONE);
  std::fill(std::begin(this->event_edges_), std::end(this->event_edges_), EventEdge{});
  std::fill(std::begin(this->sensor_votes_), std::end(this->sensor_votes_), SensorVote{});
  for (auto &channel : this->channels_) {
    channel.first_trigger_in_event_ms = 0;
    channel.pending_vote = GROUP_NONE;
    channel.pending_vote_ms = 0;
    channel.pending_vote_path.clear();
  }
}

SensorGroup TofOverdoorCounter::determine_first_group_from_current_state_() const {
  uint32_t out_ts = 0;
  uint32_t in_ts = 0;

  for (size_t index = 0; index < this->channels_.size(); index++) {
    const auto &channel = this->channels_[index];
    if (!channel.initialized || !channel.active) {
      continue;
    }
    if (channel.group == GROUP_OUT) {
      if (out_ts == 0 || (channel.active_since_ms != 0 && channel.active_since_ms < out_ts)) {
        out_ts = channel.active_since_ms != 0 ? channel.active_since_ms : millis();
      }
    } else if (channel.group == GROUP_IN) {
      if (in_ts == 0 || (channel.active_since_ms != 0 && channel.active_since_ms < in_ts)) {
        in_ts = channel.active_since_ms != 0 ? channel.active_since_ms : millis();
      }
    }
  }

  if (out_ts != 0 && in_ts == 0) {
    return GROUP_OUT;
  }
  if (in_ts != 0 && out_ts == 0) {
    return GROUP_IN;
  }
  if (out_ts != 0 && in_ts != 0) {
    if (out_ts < in_ts) {
      return GROUP_OUT;
    }
    if (in_ts < out_ts) {
      return GROUP_IN;
    }
  }

  const uint8_t active_out = this->active_sensor_count_for_group_(GROUP_OUT);
  const uint8_t active_in = this->active_sensor_count_for_group_(GROUP_IN);
  if (active_out > active_in) {
    return GROUP_OUT;
  }
  if (active_in > active_out) {
    return GROUP_IN;
  }
  return GROUP_NONE;
}

SensorGroup TofOverdoorCounter::resolve_event_first_group_() const {
  const uint32_t out_ts = this->first_trigger_ts_for_group_(GROUP_OUT);
  const uint32_t in_ts = this->first_trigger_ts_for_group_(GROUP_IN);
  const uint32_t out_confirmed_ts = this->event_group_confirmed_ms_[GROUP_OUT];
  const uint32_t in_confirmed_ts = this->event_group_confirmed_ms_[GROUP_IN];
  const SensorGroup path_first_group = this->first_group_from_path_();

  // If the compressed path starts as OUT-only or IN-only, that is the clearest
  // physical ordering signal. Do not let a later two-sensor confirmation on the
  // other side rewrite the event; that produced misleading logs like
  // "IN side first, path CLEAR->OUT->BOTH->CLEAR".
  if (path_first_group != GROUP_NONE) {
    return path_first_group;
  }

  if (out_ts != 0 && in_ts == 0) {
    return GROUP_OUT;
  }
  if (in_ts != 0 && out_ts == 0) {
    return GROUP_IN;
  }
  if (out_ts != 0 && in_ts != 0) {
    const uint32_t delta = out_ts > in_ts ? (out_ts - in_ts) : (in_ts - out_ts);
    if (delta > this->direction_window_ms_) {
      if (out_ts < in_ts) {
        return GROUP_OUT;
      }
      if (in_ts < out_ts) {
        return GROUP_IN;
      }
    }

    if (out_confirmed_ts != 0 && in_confirmed_ts == 0) {
      return GROUP_OUT;
    }
    if (in_confirmed_ts != 0 && out_confirmed_ts == 0) {
      return GROUP_IN;
    }
    if (out_confirmed_ts != 0 && in_confirmed_ts != 0) {
      if (out_confirmed_ts < in_confirmed_ts) {
        return GROUP_OUT;
      }
      if (in_confirmed_ts < out_confirmed_ts) {
        return GROUP_IN;
      }
    }
  }

  if (this->event_peak_group_counts_[GROUP_OUT] > this->event_peak_group_counts_[GROUP_IN]) {
    return GROUP_OUT;
  }
  if (this->event_peak_group_counts_[GROUP_IN] > this->event_peak_group_counts_[GROUP_OUT]) {
    return GROUP_IN;
  }

  return this->event_first_group_;
}

SensorGroup TofOverdoorCounter::map_physical_group_to_direction_(SensorGroup physical_group) const {
  if (!this->invert_direction_) {
    return physical_group;
  }
  if (physical_group == GROUP_OUT) {
    return GROUP_IN;
  }
  if (physical_group == GROUP_IN) {
    return GROUP_OUT;
  }
  return GROUP_NONE;
}

std::string TofOverdoorCounter::direction_text_for_group_(SensorGroup physical_group, bool unsure) const {
  const auto mapped = this->map_physical_group_to_direction_(physical_group);
  if (mapped == GROUP_IN) {
    return unsure ? "UNSURE_IN" : "IN";
  }
  if (mapped == GROUP_OUT) {
    return unsure ? "UNSURE_OUT" : "OUT";
  }
  return unsure ? "UNSURE" : "UNKNOWN";
}

std::string TofOverdoorCounter::passage_state_text_(PassageState state) const {
  switch (state) {
    case PASSAGE_IDLE:
      return "idle";
    case PASSAGE_POSSIBLE:
      return "possible_passage";
    case PASSAGE_OCCUPIED:
      return "person_in_doorway";
    case PASSAGE_SEQUENCE:
      return "sequence_observed";
    case PASSAGE_DIRECTION_DECIDED:
      return "direction_decided";
    case PASSAGE_COMPLETED:
      return "passage_completed";
    case PASSAGE_CANCELLED:
      return "cancelled";
    case PASSAGE_TIMEOUT:
      return "timeout";
    default:
      return "unknown";
  }
}

void TofOverdoorCounter::reset_channel_path_tracker_(Channel &channel) {
  channel.roode_path[0] = 0;
  channel.roode_path[1] = 0;
  channel.roode_path[2] = 0;
  channel.roode_path[3] = 0;
  channel.roode_path_filling_size = 1;
  // Synchronize to the live state. This is important when a quorum is reached
  // before the person has left the second ROI: resetting to CLEAR here would
  // turn the later falling edge into a second artificial passage.
  channel.roode_previous_status[ZONE_OUT] = channel.zones[ZONE_OUT].active ? ROODE_SOMEONE : ROODE_NOBODY;
  channel.roode_previous_status[ZONE_IN] = channel.zones[ZONE_IN].active ? ROODE_SOMEONE : ROODE_NOBODY;
  channel.roode_event_started_ms = 0;
  channel.roode_last_activity_ms = 0;
  channel.roode_first_zone = GROUP_NONE;
  channel.roode_last_solo_zone = GROUP_NONE;
  channel.roode_seen_out = false;
  channel.roode_seen_in = false;
  channel.roode_seen_both = false;
  channel.roode_vote_latched = false;
  channel.roode_transition_count = 0;
  channel.pending_vote = GROUP_NONE;
  channel.pending_vote_ms = 0;
  channel.pending_vote_path.clear();
  channel.last_path_text = "CLEAR";
}

std::string TofOverdoorCounter::roode_path_text_(const Channel &channel) const {
  std::ostringstream oss;
  oss << "0";
  for (uint8_t index = 1; index < channel.roode_path_filling_size && index < 4; index++) {
    oss << "->" << static_cast<unsigned>(channel.roode_path[index]);
  }
  if (channel.roode_previous_status[ZONE_OUT] == ROODE_NOBODY &&
      channel.roode_previous_status[ZONE_IN] == ROODE_NOBODY &&
      (channel.roode_path_filling_size == 1 || channel.roode_path[channel.roode_path_filling_size - 1] != 0)) {
    oss << "->0";
  }
  return oss.str();
}

void TofOverdoorCounter::update_channel_path_tracker_(Channel &channel, size_t index, uint32_t now) {
  if (!channel.initialized || index >= SENSOR_COUNT) {
    return;
  }

  const bool has_edge = channel.zones[ZONE_OUT].rising_edge || channel.zones[ZONE_OUT].falling_edge ||
                        channel.zones[ZONE_IN].rising_edge || channel.zones[ZONE_IN].falling_edge;
  if (!has_edge) {
    return;
  }

  // Consume both ROI edges atomically. Processing OUT and IN one by one could
  // briefly manufacture a CLEAR state when one ROI fell in the same update in
  // which the other rose, causing fast walkers to be rejected.
  const uint8_t previous_state =
      (channel.roode_previous_status[ZONE_OUT] == ROODE_SOMEONE ? GROUP_STATE_OUT_ONLY : 0) |
      (channel.roode_previous_status[ZONE_IN] == ROODE_SOMEONE ? GROUP_STATE_IN_ONLY : 0);
  const uint8_t current_state = (channel.zones[ZONE_OUT].active ? GROUP_STATE_OUT_ONLY : 0) |
                                (channel.zones[ZONE_IN].active ? GROUP_STATE_IN_ONLY : 0);
  if (current_state == previous_state) {
    return;
  }

  channel.roode_previous_status[ZONE_OUT] = channel.zones[ZONE_OUT].active ? ROODE_SOMEONE : ROODE_NOBODY;
  channel.roode_previous_status[ZONE_IN] = channel.zones[ZONE_IN].active ? ROODE_SOMEONE : ROODE_NOBODY;
  channel.roode_last_activity_ms = now;

  if (current_state != GROUP_STATE_NONE && channel.roode_event_started_ms == 0) {
    uint32_t first_edge_ms = now;
    if (channel.zones[ZONE_OUT].active && channel.zones[ZONE_OUT].last_rising_ms != 0) {
      first_edge_ms = std::min(first_edge_ms, channel.zones[ZONE_OUT].last_rising_ms);
    }
    if (channel.zones[ZONE_IN].active && channel.zones[ZONE_IN].last_rising_ms != 0) {
      first_edge_ms = std::min(first_edge_ms, channel.zones[ZONE_IN].last_rising_ms);
    }
    channel.roode_event_started_ms = first_edge_ms;
    channel.roode_path[0] = GROUP_STATE_NONE;
    channel.roode_path_filling_size = 1;
  }

  if (current_state == GROUP_STATE_OUT_ONLY) {
    channel.roode_seen_out = true;
    channel.roode_last_solo_zone = GROUP_OUT;
    if (channel.roode_first_zone == GROUP_NONE && !channel.roode_seen_both) {
      channel.roode_first_zone = GROUP_OUT;
    }
  } else if (current_state == GROUP_STATE_IN_ONLY) {
    channel.roode_seen_in = true;
    channel.roode_last_solo_zone = GROUP_IN;
    if (channel.roode_first_zone == GROUP_NONE && !channel.roode_seen_both) {
      channel.roode_first_zone = GROUP_IN;
    }
  } else if (current_state == GROUP_STATE_BOTH) {
    channel.roode_seen_out = true;
    channel.roode_seen_in = true;
    channel.roode_seen_both = true;
  }

  channel.roode_transition_count = std::min<uint8_t>(channel.roode_transition_count + 1U, 255);
  if (current_state != GROUP_STATE_NONE) {
    if (channel.roode_path_filling_size < 4) {
      channel.roode_path[channel.roode_path_filling_size++] = current_state;
    } else {
      channel.roode_path[3] = current_state;
    }
  }

  const bool visited_both_sides = channel.roode_seen_out && channel.roode_seen_in;
  const bool ended_opposite = channel.roode_first_zone != GROUP_NONE &&
                              channel.roode_last_solo_zone != GROUP_NONE &&
                              channel.roode_last_solo_zone != channel.roode_first_zone;
  const uint32_t duration = channel.roode_event_started_ms == 0 ? 0 : now - channel.roode_event_started_ms;
  const bool crossing_committed = current_state != GROUP_STATE_NONE && current_state != GROUP_STATE_BOTH &&
                                  visited_both_sides && ended_opposite &&
                                  duration >= this->min_active_duration_ms_;

  // Emit as soon as the person has left the starting ROI and occupies only the
  // opposite ROI. Waiting for the opposite ROI to clear added human dwell time
  // to every result. The latch prevents a second vote until the field is clear.
  if (!channel.roode_vote_latched && crossing_committed) {
    const SensorGroup vote = channel.roode_first_zone;
    const std::string path = this->roode_path_text_(channel);
    const std::string vote_text = std::string(group_name(vote)) + " early path " + path + " duration " +
                                  std::to_string(duration) + "ms transitions " +
                                  std::to_string(channel.roode_transition_count);
    channel.pending_vote = vote;
    channel.pending_vote_ms = now;
    channel.pending_vote_path = path;
    channel.last_vote_text = vote_text;
    channel.last_path_text = path;
    channel.roode_vote_latched = true;
    if (this->debug_logging_) {
      ESP_LOGD(TAG, "%s early per-sensor vote %s (%s)", channel.sensor_label.c_str(), group_name(vote),
               vote_text.c_str());
    }
  }

  if (current_state != GROUP_STATE_NONE) {
    channel.last_path_text = this->roode_path_text_(channel);
    return;
  }

  const std::string completed_path = this->roode_path_text_(channel);
  const bool fallback_vote = !channel.roode_vote_latched && visited_both_sides && ended_opposite &&
                             duration >= this->min_active_duration_ms_;
  const bool turned_back = visited_both_sides && channel.roode_first_zone != GROUP_NONE &&
                           channel.roode_last_solo_zone == channel.roode_first_zone;
  const SensorGroup vote = fallback_vote ? channel.roode_first_zone : GROUP_NONE;
  const std::string vote_text = vote != GROUP_NONE
                                    ? std::string(group_name(vote)) + " clear path " + completed_path + " duration " +
                                          std::to_string(duration) + "ms"
                                    : std::string(turned_back ? "turn-around " : "rejected ") + "path " +
                                          completed_path + " duration " + std::to_string(duration) + "ms";
  if (this->debug_logging_ && vote == GROUP_NONE && !channel.roode_vote_latched) {
    ESP_LOGD(TAG, "%s per-sensor path rejected (%s)", channel.sensor_label.c_str(), vote_text.c_str());
  }

  const bool already_voted = channel.roode_vote_latched;
  this->reset_channel_path_tracker_(channel);
  channel.last_path_text = completed_path;
  if (vote != GROUP_NONE && !already_voted) {
    channel.pending_vote = vote;
    channel.pending_vote_ms = now;
    channel.pending_vote_path = completed_path;
    channel.last_vote_text = vote_text;
  } else if (!already_voted) {
    channel.last_vote_text = vote_text;
  }
}

void TofOverdoorCounter::clear_sensor_vote_window_() {
  this->sensor_vote_count_ = 0;
  std::fill(std::begin(this->sensor_votes_), std::end(this->sensor_votes_), SensorVote{});
}

void TofOverdoorCounter::collect_pending_sensor_votes_(uint32_t now) {
  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    auto &channel = this->channels_[index];
    if (channel.pending_vote == GROUP_NONE) {
      continue;
    }

    bool replaced_existing = false;
    for (uint8_t vote_index = 0; vote_index < this->sensor_vote_count_; vote_index++) {
      if (this->sensor_votes_[vote_index].sensor_index == index) {
        auto &vote = this->sensor_votes_[vote_index];
        vote.timestamp_ms = channel.pending_vote_ms == 0 ? now : channel.pending_vote_ms;
        vote.direction = channel.pending_vote;
        vote.sensor_label = channel.sensor_label;
        vote.path_text = channel.pending_vote_path;
        vote.reason = channel.last_vote_text;
        replaced_existing = true;
        break;
      }
    }

    if (!replaced_existing && this->sensor_vote_count_ < SENSOR_COUNT) {
      auto &vote = this->sensor_votes_[this->sensor_vote_count_++];
      vote.timestamp_ms = channel.pending_vote_ms == 0 ? now : channel.pending_vote_ms;
      vote.sensor_index = static_cast<uint8_t>(index);
      vote.direction = channel.pending_vote;
      vote.sensor_label = channel.sensor_label;
      vote.path_text = channel.pending_vote_path;
      vote.reason = channel.last_vote_text;
    }

    channel.pending_vote = GROUP_NONE;
    channel.pending_vote_ms = 0;
    channel.pending_vote_path.clear();
  }

  if (this->sensor_vote_count_ == 0) {
    return;
  }

  uint8_t write_index = 0;
  for (uint8_t read_index = 0; read_index < this->sensor_vote_count_; read_index++) {
    const auto &vote = this->sensor_votes_[read_index];
    if (vote.timestamp_ms != 0 && (now - vote.timestamp_ms) <= this->detection_timeout_ms_) {
      if (write_index != read_index) {
        this->sensor_votes_[write_index] = vote;
      }
      write_index++;
    }
  }
  for (uint8_t index = write_index; index < SENSOR_COUNT; index++) {
    this->sensor_votes_[index] = SensorVote{};
  }
  this->sensor_vote_count_ = write_index;
}

std::string TofOverdoorCounter::sensor_vote_text_() const {
  if (this->sensor_vote_count_ == 0) {
    return "none";
  }
  std::ostringstream oss;
  for (uint8_t index = 0; index < this->sensor_vote_count_; index++) {
    if (index > 0) {
      oss << " ";
    }
    const auto &vote = this->sensor_votes_[index];
    oss << vote.sensor_label << "=" << group_name(vote.direction) << "(" << vote.path_text << ")";
  }
  return oss.str();
}

void TofOverdoorCounter::update_detection_state_machine_() {
  const uint32_t now = millis();

  if (!this->ready_for_counting_()) {
    this->phase_text_ = "Waiting for stable calibration";
    this->clear_event_tracking_();
    this->person_standing_in_door_ = false;
    this->update_passage_state_(PASSAGE_IDLE);
    return;
  }

  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    this->update_channel_path_tracker_(this->channels_[index], index, now);
  }

  if (this->cooldown_until_ms_ != 0 && static_cast<int32_t>(now - this->cooldown_until_ms_) < 0) {
    this->phase_text_ = "Cooldown after last detection";
    this->clear_sensor_vote_window_();
    for (auto &channel : this->channels_) {
      this->reset_channel_path_tracker_(channel);
    }
    return;
  }
  if (this->cooldown_until_ms_ != 0 && static_cast<int32_t>(now - this->cooldown_until_ms_) >= 0) {
    this->cooldown_until_ms_ = 0;
  }

  const uint8_t active_count = this->active_sensor_count_();
  if (active_count > 0) {
    if (!this->event_active_) {
      this->event_active_ = true;
      this->event_started_ms_ = now;
      this->event_last_activity_ms_ = now;
      this->event_peak_active_count_ = active_count;
      this->update_passage_state_(PASSAGE_POSSIBLE);
    } else {
      this->event_last_activity_ms_ = now;
      this->event_peak_active_count_ = std::max(this->event_peak_active_count_, active_count);
      this->update_passage_state_(active_count >= 2 ? PASSAGE_OCCUPIED : PASSAGE_POSSIBLE);
    }
  }

  this->collect_pending_sensor_votes_(now);

  if (this->sensor_vote_count_ > 0 && !this->event_active_) {
    this->event_active_ = true;
    this->event_started_ms_ = this->sensor_votes_[0].timestamp_ms;
    this->event_last_activity_ms_ = now;
    this->event_peak_active_count_ = std::max<uint8_t>(this->event_peak_active_count_, active_count);
    this->update_passage_state_(PASSAGE_SEQUENCE);
  }

  uint8_t physical_in_votes = 0;
  uint8_t physical_out_votes = 0;
  uint8_t vote_mask = 0;
  for (uint8_t index = 0; index < this->sensor_vote_count_; index++) {
    const auto &vote = this->sensor_votes_[index];
    if (vote.direction == GROUP_IN) {
      physical_in_votes++;
    } else if (vote.direction == GROUP_OUT) {
      physical_out_votes++;
    }
    vote_mask |= (1U << vote.sensor_index);
  }

  const uint8_t healthy = std::max<uint8_t>(2, this->healthy_sensor_count_());
  const uint8_t required = std::min<uint8_t>(std::max<uint8_t>(2, this->min_event_sensors_), healthy);
  const uint8_t logical_in_votes = this->invert_direction_ ? physical_out_votes : physical_in_votes;
  const uint8_t logical_out_votes = this->invert_direction_ ? physical_in_votes : physical_out_votes;

  if (logical_in_votes >= required || logical_out_votes >= required) {
    DetectionOutcome outcome = OUTCOME_NONE;
    std::string reason;

    if (logical_in_votes >= required && logical_out_votes >= required) {
      reason = "Detection cancelled: conflicting per-sensor votes (" + this->sensor_vote_text_() + ")";
    } else if (logical_in_votes >= required) {
      outcome = OUTCOME_IN;
      reason = "Detection approved: " + std::to_string(logical_in_votes) + "/" + std::to_string(healthy) +
               " sensors voted IN (" + this->sensor_vote_text_() + ")";
    } else {
      outcome = OUTCOME_OUT;
      reason = "Detection approved: " + std::to_string(logical_out_votes) + "/" + std::to_string(healthy) +
               " sensors voted OUT (" + this->sensor_vote_text_() + ")";
    }

    const uint8_t agreeing_votes = std::max(logical_in_votes, logical_out_votes);
    float quality_sum = 0.0f;
    uint8_t quality_count = 0;
    for (uint8_t vote_index = 0; vote_index < this->sensor_vote_count_; vote_index++) {
      const auto sensor_index = this->sensor_votes_[vote_index].sensor_index;
      if (sensor_index >= this->channels_.size()) continue;
      const auto &channel = this->channels_[sensor_index];
      quality_sum += std::min(channel.zones[ZONE_OUT].calibration_quality,
                              channel.zones[ZONE_IN].calibration_quality);
      quality_count++;
    }
    const float average_quality = quality_count == 0 ? 0.0f : quality_sum / quality_count;
    uint8_t confidence = outcome == OUTCOME_NONE
                             ? 0
                             : clamp_quality(35.0f + (static_cast<float>(agreeing_votes) * 15.0f) +
                                             (average_quality * 0.20f) -
                                             (static_cast<float>(std::min(logical_in_votes, logical_out_votes)) * 20.0f));

    this->last_decision_latency_ms_ = this->event_started_ms_ == 0 ? 0 : now - this->event_started_ms_;
    reason += "; decision latency " + std::to_string(this->last_decision_latency_ms_) + "ms";
    this->event_sensor_mask_ = vote_mask;
    this->update_passage_state_(outcome == OUTCOME_NONE ? PASSAGE_CANCELLED : PASSAGE_COMPLETED);
    this->register_detection_(outcome, confidence, reason);
    this->cooldown_until_ms_ = now + this->cooldown_ms_;
    this->person_standing_in_door_ = false;
    this->clear_event_tracking_();
    for (auto &channel : this->channels_) {
      this->reset_channel_path_tracker_(channel);
    }
    this->phase_text_ = outcome == OUTCOME_NONE ? "Detection cancelled" : "Detection recorded";
    return;
  }

  const bool vote_window_timed_out =
      this->event_active_ && this->event_started_ms_ != 0 && (now - this->event_started_ms_) >= this->detection_timeout_ms_;
  const bool standing =
      active_count > 0 && this->event_started_ms_ != 0 && (now - this->event_started_ms_) >= this->standing_timeout_ms_;

  if (vote_window_timed_out && active_count == 0) {
    std::string reason = "Detection cancelled: fewer than " + std::to_string(required) +
                         " sensors agreed before timeout (votes " + this->sensor_vote_text_() + ")";
    this->update_passage_state_(PASSAGE_TIMEOUT);
    this->register_detection_(OUTCOME_NONE, 0, reason);
    this->cooldown_until_ms_ = now + this->cooldown_ms_;
    this->person_standing_in_door_ = false;
    this->clear_event_tracking_();
    this->phase_text_ = "Detection cancelled";
    return;
  }

  if (standing) {
    this->person_standing_in_door_ = true;
    this->phase_text_ = "Person standing in doorway";
    this->update_passage_state_(PASSAGE_OCCUPIED);
    return;
  }

  if (active_count == 0 && this->sensor_vote_count_ == 0) {
    this->person_standing_in_door_ = false;
    this->standing_clear_since_ms_ = 0;
    this->event_active_ = false;
    this->event_started_ms_ = 0;
    this->event_last_activity_ms_ = 0;
    this->event_peak_active_count_ = 0;
    this->phase_text_ = "Ready - waiting for fused passage tracks";
    this->update_passage_state_(PASSAGE_IDLE);
    return;
  }

  this->phase_text_ = "Waiting for " + std::to_string(required) + " sensors to agree (" +
                      std::to_string(logical_in_votes) + " IN, " + std::to_string(logical_out_votes) + " OUT)";
  this->update_passage_state_(this->sensor_vote_count_ > 0 ? PASSAGE_SEQUENCE : PASSAGE_POSSIBLE);
}

void TofOverdoorCounter::register_detection_(DetectionOutcome outcome, uint8_t confidence, const std::string &reason) {
  this->last_confidence_ = confidence;
  this->last_reason_ = reason;
  this->last_detection_ms_ = millis();
  this->last_detection_outcome_ = outcome;
  this->last_direction_ = "CANCELLED";

  switch (outcome) {
    case OUTCOME_IN:
      this->confirmed_in_count_++;
      this->people_inside_ = std::min<int>(this->people_inside_ + 1, this->max_people_inside_);
      this->last_direction_ = "IN";
      break;
    case OUTCOME_OUT:
      this->confirmed_out_count_++;
      this->people_inside_ = std::max<int>(this->people_inside_ - 1, 0);
      this->last_direction_ = "OUT";
      break;
    case OUTCOME_UNSURE_IN:
      this->unsure_in_count_++;
      this->last_direction_ = "UNSURE_IN";
      break;
    case OUTCOME_UNSURE_OUT:
      this->unsure_out_count_++;
      this->last_direction_ = "UNSURE_OUT";
      break;
    case OUTCOME_NONE:
    default:
      this->rejected_count_++;
      this->last_direction_ = "CANCELLED";
      break;
  }

  this->log_event_(this->format_uptime_(this->last_detection_ms_) + " - " + this->last_direction_ + " - " + reason +
                   " - confidence " + std::to_string(confidence) + "%");
  this->state_dirty_ = true;
}

void TofOverdoorCounter::finalize_event_(bool timed_out) {
  const uint8_t distinct_triggered = popcount_u8(this->event_sensor_mask_);
  const uint8_t out_triggered = this->triggered_sensor_count_for_group_(GROUP_OUT);
  const uint8_t in_triggered = this->triggered_sensor_count_for_group_(GROUP_IN);
  const bool both_groups_seen = out_triggered > 0 && in_triggered > 0;
  const uint8_t healthy = std::max<uint8_t>(2, this->healthy_sensor_count_());
  const uint8_t required = std::min<uint8_t>(std::max<uint8_t>(2, this->min_event_sensors_), healthy);
  const SensorGroup resolved_first_group = this->resolve_event_first_group_();
  const uint32_t event_duration_ms = this->event_started_ms_ == 0 ? 0 : millis() - this->event_started_ms_;
  const bool long_enough =
      event_duration_ms >= this->min_active_duration_ms_ || this->event_peak_active_count_ >= 2 ||
      (this->event_last_edge_ms_ > this->event_first_edge_ms_ &&
       (this->event_last_edge_ms_ - this->event_first_edge_ms_) >= this->min_active_duration_ms_);

  uint8_t first_state = GROUP_STATE_NONE;
  uint8_t last_state = GROUP_STATE_NONE;
  bool saw_both_state = false;
  bool saw_same_side_after_both = false;
  bool saw_opposite_side_after_both = false;

  for (uint8_t index = 0; index < this->event_path_size_; index++) {
    const uint8_t state = this->event_path_[index];
    if (first_state == GROUP_STATE_NONE) {
      first_state = state;
    }
    last_state = state;
    if (state == GROUP_STATE_BOTH) {
      saw_both_state = true;
    } else if (state == GROUP_STATE_OUT_ONLY) {
      if (saw_both_state && resolved_first_group == GROUP_OUT) {
        saw_same_side_after_both = true;
      }
      if (saw_both_state && resolved_first_group == GROUP_IN) {
        saw_opposite_side_after_both = true;
      }
    } else if (state == GROUP_STATE_IN_ONLY) {
      if (saw_both_state && resolved_first_group == GROUP_IN) {
        saw_same_side_after_both = true;
      }
      if (saw_both_state && resolved_first_group == GROUP_OUT) {
        saw_opposite_side_after_both = true;
      }
    }
  }

  bool path_crossed_doorway = false;
  if (resolved_first_group == GROUP_OUT) {
    path_crossed_doorway = saw_opposite_side_after_both || (first_state == GROUP_STATE_OUT_ONLY && last_state == GROUP_STATE_IN_ONLY);
  } else if (resolved_first_group == GROUP_IN) {
    path_crossed_doorway = saw_opposite_side_after_both || (first_state == GROUP_STATE_IN_ONLY && last_state == GROUP_STATE_OUT_ONLY);
  }

  const bool fast_cross_path = (resolved_first_group == GROUP_OUT && first_state == GROUP_STATE_OUT_ONLY && last_state == GROUP_STATE_IN_ONLY) ||
                               (resolved_first_group == GROUP_IN && first_state == GROUP_STATE_IN_ONLY && last_state == GROUP_STATE_OUT_ONLY);
  const bool clean_one_sided_start =
      (resolved_first_group == GROUP_OUT && first_state == GROUP_STATE_OUT_ONLY) ||
      (resolved_first_group == GROUP_IN && first_state == GROUP_STATE_IN_ONLY);
  const bool overlap_after_clean_start = clean_one_sided_start && saw_both_state;
  const bool direction_shape_is_clear = path_crossed_doorway || fast_cross_path || overlap_after_clean_start;
  const bool ambiguous_both_start = first_state == GROUP_STATE_BOTH;
  const bool ordered_two_group_sequence = resolved_first_group != GROUP_NONE && this->event_second_group_ != GROUP_NONE &&
                                          this->event_second_group_ != resolved_first_group;
  const bool valid_crossing = both_groups_seen && ordered_two_group_sequence && distinct_triggered >= required &&
                              long_enough && direction_shape_is_clear && !ambiguous_both_start;
  const bool backed_out_after_both = saw_same_side_after_both && !saw_opposite_side_after_both;

  DetectionOutcome outcome = OUTCOME_NONE;
  std::string reason = "Detection cancelled";

  if (backed_out_after_both && resolved_first_group != GROUP_NONE) {
    reason = "Detection cancelled: returned to the " + std::string(group_name(resolved_first_group)) +
             " side after entering the doorway (path " + this->event_path_text_() + ")";
  } else if (resolved_first_group != GROUP_NONE && valid_crossing) {
    const SensorGroup direction_group = this->map_physical_group_to_direction_(resolved_first_group);
    outcome = direction_group == GROUP_IN ? OUTCOME_IN : OUTCOME_OUT;
    reason = "Detection approved: " + std::to_string(distinct_triggered) + "/" + std::to_string(healthy) +
             " sensors triggered, " + std::string(group_name(resolved_first_group)) +
             " side first, path " + this->event_path_text_() + ", " +
             this->event_timing_text_(resolved_first_group) + ", edges " + this->event_edge_text_();
  } else if (resolved_first_group != GROUP_NONE && both_groups_seen && distinct_triggered >= 2 && long_enough) {
    const SensorGroup direction_group = this->map_physical_group_to_direction_(resolved_first_group);
    outcome = direction_group == GROUP_IN ? OUTCOME_UNSURE_IN : OUTCOME_UNSURE_OUT;
    reason = "Unsure detection: " + std::to_string(distinct_triggered) + "/" + std::to_string(healthy) +
             " sensors triggered, " + std::string(group_name(resolved_first_group)) +
             " side first, path " + this->event_path_text_() + ", " +
             this->event_timing_text_(resolved_first_group) + ", edges " + this->event_edge_text_();
  } else if (resolved_first_group != GROUP_NONE && !both_groups_seen) {
    reason = "Detection cancelled: only one physical side triggered (sensors " +
             this->sensor_mask_text_(this->event_sensor_mask_) + ", path " + this->event_path_text_() + ")";
  } else if (!long_enough) {
    reason = "Detection cancelled: trigger was shorter than " + std::to_string(this->min_active_duration_ms_) +
             " ms (path " + this->event_path_text_() + ")";
  } else if (timed_out && resolved_first_group != GROUP_NONE) {
    reason = "Timed out before enough sensors agreed (path " + this->event_path_text_() + ")";
  } else if (this->person_standing_in_door_) {
    reason = "Doorway stayed occupied too long (path " + this->event_path_text_() + ")";
  }

  uint8_t confidence = 0;
  if (outcome != OUTCOME_NONE) {
    float raw_confidence = static_cast<float>(distinct_triggered) * 20.0f;
    raw_confidence += both_groups_seen ? 18.0f : 0.0f;
    raw_confidence += saw_both_state ? 14.0f : 0.0f;
    raw_confidence += path_crossed_doorway ? 14.0f : 0.0f;
    raw_confidence += std::min<uint8_t>(20, this->event_peak_active_count_ * 5U);
    if (!path_crossed_doorway && !fast_cross_path) {
      raw_confidence -= 12.0f;
    }
    if (ambiguous_both_start) {
      raw_confidence -= 30.0f;
    }
    if (timed_out) {
      raw_confidence -= 15.0f;
    }
    if (this->person_standing_in_door_) {
      raw_confidence -= 8.0f;
    }
    if (backed_out_after_both) {
      raw_confidence -= 35.0f;
    }
    if (outcome == OUTCOME_UNSURE_IN || outcome == OUTCOME_UNSURE_OUT) {
      raw_confidence -= 22.0f;
    }
    if (healthy < SENSOR_COUNT) {
      raw_confidence -= 10.0f;
    }
    confidence = clamp_quality(raw_confidence);
  }

  this->update_passage_state_(outcome == OUTCOME_NONE ? (timed_out ? PASSAGE_TIMEOUT : PASSAGE_CANCELLED)
                                                       : PASSAGE_COMPLETED);
  this->register_detection_(outcome, confidence, reason);
  this->cooldown_until_ms_ = millis() + this->cooldown_ms_;
  this->person_standing_in_door_ = false;
  this->clear_event_tracking_();
  this->phase_text_ = outcome == OUTCOME_NONE ? "Detection cancelled" : "Detection recorded";
}

void TofOverdoorCounter::record_history_snapshot_(uint32_t now) {
  bool has_edge = false;
  for (const auto &channel : this->channels_) {
    if (channel.rising_edge || channel.falling_edge) {
      has_edge = true;
      break;
    }
  }
  if (!has_edge && this->history_count_ > 0) {
    const auto &previous = this->history_[(this->history_head_ + HISTORY_SIZE - 1) % HISTORY_SIZE];
    if ((now - previous.timestamp_ms) < TRACE_SAMPLE_INTERVAL_MS) {
      return;
    }
  }

  auto &snapshot = this->history_[this->history_head_];
  snapshot = HistorySample{};
  snapshot.timestamp_ms = now;
  snapshot.passage_state = this->passage_state_;

  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    const auto &channel = this->channels_[index];
    snapshot.raw_distance[index] = channel.raw_distance;
    const float logic_distance = this->channel_logic_distance_(channel);
    snapshot.filtered_distance[index] = std::isnan(logic_distance) ? 0 : static_cast<uint16_t>(logic_distance);
    snapshot.range_status[index] = channel.range_status;
    if (channel.initialized && channel.has_reading && !channel.stale && channel.valid_measurement) {
      snapshot.valid_mask |= (1U << index);
    }
    if (channel.initialized && channel.active) {
      snapshot.active_mask |= (1U << index);
    }
    if (channel.rising_edge) {
      snapshot.rising_mask |= (1U << index);
    }
    if (channel.falling_edge) {
      snapshot.falling_mask |= (1U << index);
    }
  }

  this->history_head_ = (this->history_head_ + 1) % HISTORY_SIZE;
  if (this->history_count_ < HISTORY_SIZE) {
    this->history_count_++;
  }
}

void TofOverdoorCounter::record_event_edge_(size_t index, const Channel &channel, bool rising, uint32_t now,
                                            uint8_t active_mask) {
  if (index >= SENSOR_COUNT) {
    return;
  }
  if (this->event_edge_count_ < EVENT_EDGE_SIZE) {
    auto &edge = this->event_edges_[this->event_edge_count_++];
    edge.timestamp_ms = now;
    edge.sensor_index = static_cast<uint8_t>(index);
    edge.group = channel.group;
    edge.rising = rising;
    edge.active_mask = active_mask;
  } else {
    for (size_t i = 1; i < EVENT_EDGE_SIZE; i++) {
      this->event_edges_[i - 1] = this->event_edges_[i];
    }
    auto &edge = this->event_edges_[EVENT_EDGE_SIZE - 1];
    edge.timestamp_ms = now;
    edge.sensor_index = static_cast<uint8_t>(index);
    edge.group = channel.group;
    edge.rising = rising;
    edge.active_mask = active_mask;
  }

  if (this->event_first_edge_ms_ == 0 || now < this->event_first_edge_ms_) {
    this->event_first_edge_ms_ = now;
  }
  this->event_last_edge_ms_ = now;
  this->event_last_activity_ms_ = now;

  if (this->debug_logging_) {
    ESP_LOGD(TAG, "%s edge %s at %u ms group=%s active=%s", channel.sensor_label.c_str(),
             rising ? "rising" : "falling", static_cast<unsigned>(now), group_name(channel.group),
             this->sensor_mask_text_(active_mask).c_str());
  }
}

void TofOverdoorCounter::update_passage_state_(PassageState state) {
  if (this->passage_state_ == state) {
    return;
  }
  if (state != PASSAGE_IDLE && state < this->passage_state_ &&
      this->passage_state_ != PASSAGE_COMPLETED && this->passage_state_ != PASSAGE_CANCELLED &&
      this->passage_state_ != PASSAGE_TIMEOUT) {
    return;
  }
  if (this->debug_logging_) {
    ESP_LOGD(TAG, "Passage state %s -> %s", this->passage_state_text_(this->passage_state_).c_str(),
             this->passage_state_text_(state).c_str());
  }
  this->passage_state_ = state;
}

void TofOverdoorCounter::debug_log_sample_(uint32_t now) {
  if (!this->debug_logging_) {
    return;
  }
  if (this->last_debug_sample_log_ms_ != 0 &&
      (now - this->last_debug_sample_log_ms_) < this->debug_sample_interval_ms_) {
    return;
  }
  this->last_debug_sample_log_ms_ = now;

  std::ostringstream oss;
  oss << "sample t=" << now << " state=" << this->passage_state_text_(this->passage_state_)
      << " active=" << this->sensor_mask_text_(this->active_sensor_count_() == 0 ? 0 : this->history_[(this->history_head_ + HISTORY_SIZE - 1) % HISTORY_SIZE].active_mask);
  for (size_t index = 0; index < this->channels_.size(); index++) {
    const auto &channel = this->channels_[index];
    if (!channel.initialized) {
      continue;
    }
    const float logic_distance = this->channel_logic_distance_(channel);
    oss << " " << channel.sensor_label << "{raw=" << channel.raw_distance << ",f=";
    if (std::isnan(logic_distance)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(logic_distance);
    }
    oss << ",drop=";
    if (std::isnan(channel.baseline) || std::isnan(logic_distance)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(channel.baseline - logic_distance);
    }
    oss << ",active=" << (channel.active ? "1" : "0") << ",status=" << range_status_name(channel.range_status)
        << ",out_zone=";
    const float out_zone = this->zone_logic_distance_(channel.zones[ZONE_OUT]);
    if (std::isnan(out_zone)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(out_zone);
    }
    oss << "/" << (channel.zones[ZONE_OUT].active ? "1" : "0") << ",in_zone=";
    const float in_zone = this->zone_logic_distance_(channel.zones[ZONE_IN]);
    if (std::isnan(in_zone)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(in_zone);
    }
    oss << "/" << (channel.zones[ZONE_IN].active ? "1" : "0") << ",vote=" << channel.last_vote_text << "}";
  }
  ESP_LOGD(TAG, "%s", oss.str().c_str());
}

void TofOverdoorCounter::update_sensor_health_() {
  const uint32_t now = millis();
  for (auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    channel.stale = channel.last_good_read_ms == 0 || (now - channel.last_good_read_ms) > STALE_READING_MS;
  }
}

void TofOverdoorCounter::update_system_status_() {
  if (this->channels_.empty() || this->get_discovered_sensor_count() < 1.0f) {
    this->system_status_ = STATUS_ERROR;
    return;
  }

  if (this->calibration_active_) {
    this->system_status_ = STATUS_CALIBRATING;
    return;
  }

  const uint8_t healthy = this->healthy_sensor_count_();
  const uint8_t reporting = this->reporting_sensor_count_();
  uint8_t calibrated = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.zones[ZONE_OUT].calibrated && channel.zones[ZONE_IN].calibrated) calibrated++;
  }

  if (reporting < this->min_valid_sensors_ || healthy < this->min_valid_sensors_) {
    this->system_status_ = STATUS_ERROR;
    return;
  }

  if (this->person_standing_in_door_ || this->blocked_sensor_text_ != "None") {
    this->system_status_ = STATUS_BLOCKED;
    return;
  }

  if (this->event_active_) {
    this->system_status_ = STATUS_DETECTING;
    return;
  }

  if (healthy < this->get_discovered_sensor_count() || calibrated < this->get_discovered_sensor_count()) {
    this->system_status_ = STATUS_DEGRADED;
    return;
  }

  this->system_status_ = STATUS_READY;
}

bool TofOverdoorCounter::ready_for_counting_() const {
  if (this->calibration_active_) {
    return false;
  }
  uint8_t calibrated = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.zones[ZONE_OUT].calibrated && channel.zones[ZONE_IN].calibrated) {
      calibrated++;
    }
  }
  return calibrated >= this->min_valid_sensors_ && this->reporting_sensor_count_() >= this->min_valid_sensors_ &&
         this->startup_clear_validated_;
}

bool TofOverdoorCounter::has_restored_calibration_() const {
  uint8_t calibrated = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.zones[ZONE_OUT].calibrated && channel.zones[ZONE_IN].calibrated &&
        !std::isnan(channel.zones[ZONE_OUT].baseline) && !std::isnan(channel.zones[ZONE_IN].baseline)) {
      calibrated++;
    }
  }
  return calibrated >= this->min_valid_sensors_;
}

bool TofOverdoorCounter::all_reporting_() const { return this->reporting_sensor_count_() == this->get_discovered_sensor_count(); }

uint8_t TofOverdoorCounter::healthy_sensor_count_() const {
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    if (channel.has_reading && !channel.stale && channel.consecutive_errors < 3 && channel.consecutive_invalid < 3) {
      count++;
    }
  }
  return count;
}

uint8_t TofOverdoorCounter::reporting_sensor_count_() const {
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.has_reading && channel.consecutive_invalid < 3) {
      count++;
    }
  }
  return count;
}

uint8_t TofOverdoorCounter::active_sensor_count_() const {
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.active) {
      count++;
    }
  }
  return count;
}

uint8_t TofOverdoorCounter::active_sensor_count_for_group_(SensorGroup group) const {
  uint8_t count = 0;
  if (group == GROUP_NONE) {
    return count;
  }
  const uint8_t zone_index = group == GROUP_OUT ? ZONE_OUT : ZONE_IN;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.zones[zone_index].active) {
      count++;
    }
  }
  return count;
}

uint8_t TofOverdoorCounter::triggered_sensor_count_for_group_(SensorGroup group) const {
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.group == group && channel.first_trigger_in_event_ms != 0) {
      count++;
    }
  }
  return count;
}

uint32_t TofOverdoorCounter::first_trigger_ts_for_group_(SensorGroup group) const {
  uint32_t earliest = 0;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized || channel.group != group || channel.first_trigger_in_event_ms == 0) {
      continue;
    }
    if (earliest == 0 || channel.first_trigger_in_event_ms < earliest) {
      earliest = channel.first_trigger_in_event_ms;
    }
  }
  return earliest;
}

bool TofOverdoorCounter::group_is_active_(SensorGroup group) const {
  return this->active_sensor_count_for_group_(group) > 0;
}

uint8_t TofOverdoorCounter::current_group_state_code_(uint8_t active_out, uint8_t active_in) const {
  const bool out_active = active_out > 0;
  const bool in_active = active_in > 0;
  if (out_active && in_active) {
    return GROUP_STATE_BOTH;
  }
  if (out_active) {
    return GROUP_STATE_OUT_ONLY;
  }
  if (in_active) {
    return GROUP_STATE_IN_ONLY;
  }
  return GROUP_STATE_NONE;
}

void TofOverdoorCounter::append_event_path_state_(uint8_t state_code, uint32_t now) {
  if (state_code == this->event_last_state_code_) {
    return;
  }

  this->event_last_state_code_ = state_code;
  this->event_last_activity_ms_ = now;

  if (state_code == GROUP_STATE_NONE) {
    return;
  }

  if (this->event_path_size_ < sizeof(this->event_path_)) {
    this->event_path_[this->event_path_size_++] = state_code;
    return;
  }

  for (size_t index = 1; index < sizeof(this->event_path_); index++) {
    this->event_path_[index - 1] = this->event_path_[index];
  }
  this->event_path_[sizeof(this->event_path_) - 1] = state_code;
}

std::string TofOverdoorCounter::event_path_text_() const {
  if (this->event_path_size_ == 0) {
    return "CLEAR";
  }

  std::ostringstream oss;
  oss << "CLEAR";
  for (uint8_t index = 0; index < this->event_path_size_; index++) {
    oss << "->" << group_state_name(this->event_path_[index]);
  }
  oss << "->CLEAR";
  return oss.str();
}

SensorGroup TofOverdoorCounter::first_group_from_path_() const {
  if (this->event_path_size_ == 0) {
    return GROUP_NONE;
  }

  // A BOTH-first path means the two physical sides overlapped before the state
  // machine saw a clean side lead. Later one-sided states are useful for path
  // shape, but they should not be rewritten as "first".
  return group_from_state_code(this->event_path_[0]);
}

float TofOverdoorCounter::group_distance_internal_(SensorGroup group) const {
  float nearest = NAN;
  if (group == GROUP_NONE) {
    return nearest;
  }
  const uint8_t zone_index = group == GROUP_OUT ? ZONE_OUT : ZONE_IN;
  for (const auto &channel : this->channels_) {
    const auto &zone = channel.zones[zone_index];
    const float logic_distance = this->zone_logic_distance_(zone);
    if (!channel.initialized || !zone.has_reading || std::isnan(logic_distance)) {
      continue;
    }
    nearest = std::isnan(nearest) ? logic_distance : std::min(nearest, logic_distance);
  }
  return nearest;
}

float TofOverdoorCounter::group_baseline_internal_(SensorGroup group) const {
  if (group == GROUP_NONE) return NAN;
  const uint8_t zone_index = group == GROUP_OUT ? ZONE_OUT : ZONE_IN;
  float total = 0.0f;
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    const auto &zone = channel.zones[zone_index];
    if (!channel.initialized || !zone.calibrated || std::isnan(zone.baseline)) {
      continue;
    }
    total += zone.baseline;
    count++;
  }
  return count == 0 ? NAN : total / static_cast<float>(count);
}

float TofOverdoorCounter::group_drop_internal_(SensorGroup group) const {
  float drop = NAN;
  if (group == GROUP_NONE) {
    return drop;
  }
  const uint8_t zone_index = group == GROUP_OUT ? ZONE_OUT : ZONE_IN;
  for (const auto &channel : this->channels_) {
    const auto &zone = channel.zones[zone_index];
    const float logic_distance = this->zone_logic_distance_(zone);
    if (!channel.initialized || !zone.calibrated || std::isnan(zone.baseline) || std::isnan(logic_distance)) {
      continue;
    }
    const float sensor_drop = zone.baseline - logic_distance;
    drop = std::isnan(drop) ? sensor_drop : std::max(drop, sensor_drop);
  }
  return drop;
}

SensorGroup TofOverdoorCounter::group_for_index_(size_t index) const {
  if (index >= this->channels_.size()) {
    return GROUP_NONE;
  }
  return this->channels_[index].group;
}

std::string TofOverdoorCounter::system_status_text_(SystemStatus status) const {
  switch (status) {
    case STATUS_CALIBRATING:
      return "Calibrating";
    case STATUS_READY:
      return "Ready";
    case STATUS_DETECTING:
      return "Detecting";
    case STATUS_BLOCKED:
      return "Blocked";
    case STATUS_DEGRADED:
      return "Degraded";
    case STATUS_ERROR:
      return "Error";
    case STATUS_BOOTING:
    default:
      return "Booting";
  }
}

std::string TofOverdoorCounter::health_text_for_(const Channel &channel) const {
  if (!channel.initialized) {
    return "Error";
  }
  if (channel.stale || channel.consecutive_errors >= 3) {
    return "Error";
  }
  if (channel.consecutive_invalid >= 3) {
    return "Warning";
  }
  if (channel.blocked || channel.last_error != 0) {
    return "Warning";
  }
  return "OK";
}

std::string TofOverdoorCounter::status_text_for_(const Channel &channel) const {
  if (!channel.initialized) {
    return "Missing";
  }
  if (channel.consecutive_errors >= 3) {
    return "Read error " + std::to_string(channel.last_error);
  }
  if (channel.consecutive_invalid >= 3) {
    return "Invalid range " + std::string(range_status_name(channel.range_status));
  }
  if (!channel.has_reading) {
    return "Waiting";
  }
  if (this->calibration_active_) {
    return "Calibrating";
  }
  if (channel.blocked) {
    return "Blocked";
  }
  if (channel.active) {
    return "Triggered";
  }
  return "Clear";
}

std::string TofOverdoorCounter::format_uptime_(uint32_t ms) const {
  const uint32_t total_seconds = ms / 1000UL;
  const uint32_t hours = total_seconds / 3600UL;
  const uint32_t minutes = (total_seconds % 3600UL) / 60UL;
  const uint32_t seconds = total_seconds % 60UL;
  char buffer[24];
  snprintf(buffer, sizeof(buffer), "%02u:%02u:%02u", static_cast<unsigned>(hours), static_cast<unsigned>(minutes),
           static_cast<unsigned>(seconds));
  return buffer;
}

std::string TofOverdoorCounter::sensor_mask_text_(uint8_t mask) const {
  if (mask == 0) {
    return "none";
  }
  std::ostringstream oss;
  bool first = true;
  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    if ((mask & (1U << index)) == 0) {
      continue;
    }
    if (!first) {
      oss << ",";
    }
    oss << this->channels_[index].sensor_label;
    first = false;
  }
  return first ? "none" : oss.str();
}

std::string TofOverdoorCounter::event_edge_text_() const {
  if (this->event_edge_count_ == 0) {
    return "none";
  }
  std::ostringstream oss;
  for (uint8_t index = 0; index < this->event_edge_count_; index++) {
    if (index > 0) {
      oss << " ";
    }
    const auto &edge = this->event_edges_[index];
    const char *label = edge.sensor_index < this->channels_.size()
                            ? this->channels_[edge.sensor_index].sensor_label.c_str()
                            : "S?";
    oss << (edge.timestamp_ms - this->event_started_ms_) << "ms:" << label << (edge.rising ? "+" : "-");
  }
  return oss.str();
}

std::string TofOverdoorCounter::event_timing_text_(SensorGroup resolved_first_group) const {
  const uint32_t out_ts = this->first_trigger_ts_for_group_(GROUP_OUT);
  const uint32_t in_ts = this->first_trigger_ts_for_group_(GROUP_IN);
  const uint32_t out_confirmed_ts = this->event_group_confirmed_ms_[GROUP_OUT];
  const uint32_t in_confirmed_ts = this->event_group_confirmed_ms_[GROUP_IN];
  const SensorGroup path_first_group = this->first_group_from_path_();

  auto relative_text = [this](uint32_t timestamp) -> std::string {
    if (timestamp == 0 || this->event_started_ms_ == 0) {
      return "n/a";
    }
    return std::to_string(timestamp - this->event_started_ms_) + "ms";
  };

  std::ostringstream oss;
  oss << "timing first=" << group_debug_name(resolved_first_group)
      << " path_first=" << group_debug_name(path_first_group)
      << " edge_out=" << relative_text(out_ts)
      << " edge_in=" << relative_text(in_ts)
      << " pair_out=" << relative_text(out_confirmed_ts)
      << " pair_in=" << relative_text(in_confirmed_ts);
  return oss.str();
}

void TofOverdoorCounter::log_event_(const std::string &message) {
  this->event_log_[this->event_log_count_ % EVENT_LOG_SIZE] = message;
  this->event_log_count_++;
}

float TofOverdoorCounter::get_discovered_sensor_count() const {
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized) {
      count++;
    }
  }
  return static_cast<float>(count);
}

float TofOverdoorCounter::get_reporting_sensor_count() const { return static_cast<float>(this->reporting_sensor_count_()); }

float TofOverdoorCounter::get_cycle_duration_ms() const { return static_cast<float>(this->cycle_duration_ms_); }

float TofOverdoorCounter::get_last_decision_latency_ms() const {
  return static_cast<float>(this->last_decision_latency_ms_);
}

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
  float nearest = NAN;
  for (const auto &channel : this->channels_) {
    const float logic_distance = this->channel_logic_distance_(channel);
    if (!channel.initialized || !channel.has_reading || std::isnan(logic_distance)) {
      continue;
    }
    nearest = std::isnan(nearest) ? logic_distance : std::min(nearest, logic_distance);
  }
  return nearest;
}

float TofOverdoorCounter::get_average_distance_mm() const {
  float total = 0.0f;
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    const float logic_distance = this->channel_logic_distance_(channel);
    if (!channel.initialized || !channel.has_reading || std::isnan(logic_distance)) {
      continue;
    }
    total += logic_distance;
    count++;
  }
  return count == 0 ? NAN : total / static_cast<float>(count);
}

float TofOverdoorCounter::get_distance_span_mm() const {
  float min_distance = NAN;
  float max_distance = NAN;
  for (const auto &channel : this->channels_) {
    const float logic_distance = this->channel_logic_distance_(channel);
    if (!channel.initialized || !channel.has_reading || std::isnan(logic_distance)) {
      continue;
    }
    min_distance = std::isnan(min_distance) ? logic_distance : std::min(min_distance, logic_distance);
    max_distance = std::isnan(max_distance) ? logic_distance : std::max(max_distance, logic_distance);
  }
  if (std::isnan(min_distance) || std::isnan(max_distance)) {
    return NAN;
  }
  return max_distance - min_distance;
}

float TofOverdoorCounter::get_distance_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized || !this->channels_[index].has_reading) {
    return NAN;
  }
  return this->channel_logic_distance_(this->channels_[index]);
}

float TofOverdoorCounter::get_raw_distance_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized || !this->channels_[index].has_reading) {
    return NAN;
  }
  return static_cast<float>(this->channels_[index].raw_distance);
}

float TofOverdoorCounter::get_filtered_distance_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized || !this->channels_[index].has_reading ||
      std::isnan(this->channels_[index].filtered_distance)) {
    return NAN;
  }
  return this->channels_[index].filtered_distance;
}

float TofOverdoorCounter::get_baseline_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized || std::isnan(this->channels_[index].baseline)) {
    return NAN;
  }
  return this->channels_[index].baseline;
}

float TofOverdoorCounter::get_delta_mm(size_t index) const {
  if (index >= this->channels_.size()) {
    return NAN;
  }
  const auto &channel = this->channels_[index];
  float largest_drop = NAN;
  for (const auto &zone : channel.zones) {
    const float distance = this->zone_logic_distance_(zone);
    if (!channel.initialized || !zone.calibrated || std::isnan(zone.baseline) || std::isnan(distance)) continue;
    const float drop = zone.baseline - distance;
    largest_drop = std::isnan(largest_drop) ? drop : std::max(largest_drop, drop);
  }
  return largest_drop;
}

float TofOverdoorCounter::get_noise_mm(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized || std::isnan(this->channels_[index].noise)) {
    return NAN;
  }
  return this->channels_[index].noise;
}

float TofOverdoorCounter::get_calibration_quality(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized) {
    return NAN;
  }
  return static_cast<float>(this->channels_[index].calibration_quality);
}

float TofOverdoorCounter::get_row_distance_mm(size_t row_index) const {
  return this->group_distance_internal_(row_index == 0 ? GROUP_OUT : GROUP_IN);
}

float TofOverdoorCounter::get_row_baseline_mm(size_t row_index) const {
  return this->group_baseline_internal_(row_index == 0 ? GROUP_OUT : GROUP_IN);
}

float TofOverdoorCounter::get_row_drop_mm(size_t row_index) const {
  return this->group_drop_internal_(row_index == 0 ? GROUP_OUT : GROUP_IN);
}

float TofOverdoorCounter::get_entry_count() const { return static_cast<float>(this->confirmed_in_count_); }

float TofOverdoorCounter::get_exit_count() const { return static_cast<float>(this->confirmed_out_count_); }

float TofOverdoorCounter::get_people_count() const { return static_cast<float>(this->people_inside_); }

float TofOverdoorCounter::get_unsure_in_count() const { return static_cast<float>(this->unsure_in_count_); }

float TofOverdoorCounter::get_unsure_out_count() const { return static_cast<float>(this->unsure_out_count_); }

float TofOverdoorCounter::get_rejected_count() const { return static_cast<float>(this->rejected_count_); }

float TofOverdoorCounter::get_presence_state() const {
  return (this->active_sensor_count_() > 0 || this->event_active_ || this->person_standing_in_door_) ? 1.0f : 0.0f;
}

float TofOverdoorCounter::get_ready_state() const { return this->ready_for_counting_() ? 1.0f : 0.0f; }

float TofOverdoorCounter::get_row_active_state(size_t row_index) const {
  return this->group_is_active_(row_index == 0 ? GROUP_OUT : GROUP_IN) ? 1.0f : 0.0f;
}

float TofOverdoorCounter::get_sensor_active_state(size_t index) const {
  if (index >= this->channels_.size() || !this->channels_[index].initialized) {
    return 0.0f;
  }
  return this->channels_[index].active ? 1.0f : 0.0f;
}

float TofOverdoorCounter::get_person_standing_state() const { return this->person_standing_in_door_ ? 1.0f : 0.0f; }

float TofOverdoorCounter::get_confidence_score() const { return static_cast<float>(this->last_confidence_); }

float TofOverdoorCounter::get_calibration_progress() const {
  if (!this->calibration_active_ || this->channels_.empty()) {
    return 100.0f;
  }
  std::array<uint16_t, SENSOR_COUNT> sample_counts{};
  size_t count_size = 0;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    sample_counts[count_size++] =
        std::min(channel.zones[ZONE_OUT].calibration_samples, channel.zones[ZONE_IN].calibration_samples);
  }
  if (count_size == 0 || this->calibration_samples_ == 0) {
    return 0.0f;
  }
  std::sort(sample_counts.begin(), sample_counts.begin() + count_size, std::greater<uint16_t>());
  const size_t required_rank = std::min<size_t>(std::max<uint8_t>(1, this->min_valid_sensors_), count_size);
  const float supporting_samples = static_cast<float>(sample_counts[required_rank - 1]);
  return clampf((supporting_samples / static_cast<float>(this->calibration_samples_)) * 100.0f, 0.0f, 100.0f);
}

float TofOverdoorCounter::get_max_people_inside_value() const { return static_cast<float>(this->max_people_inside_); }

float TofOverdoorCounter::get_trigger_threshold_value() const { return static_cast<float>(this->trigger_threshold_mm_); }

float TofOverdoorCounter::get_clear_threshold_value() const { return static_cast<float>(this->clear_threshold_mm_); }

float TofOverdoorCounter::get_baseline_tolerance_value() const { return static_cast<float>(this->baseline_tolerance_mm_); }

float TofOverdoorCounter::get_debounce_value() const { return static_cast<float>(this->debounce_ms_); }

float TofOverdoorCounter::get_detection_timeout_value() const { return static_cast<float>(this->detection_timeout_ms_); }

float TofOverdoorCounter::get_cooldown_value() const { return static_cast<float>(this->cooldown_ms_); }

float TofOverdoorCounter::get_min_valid_sensors_value() const { return static_cast<float>(this->min_valid_sensors_); }

float TofOverdoorCounter::get_min_event_sensors_value() const { return static_cast<float>(this->min_event_sensors_); }

float TofOverdoorCounter::get_min_active_duration_value() const {
  return static_cast<float>(this->min_active_duration_ms_);
}

float TofOverdoorCounter::get_direction_window_value() const { return static_cast<float>(this->direction_window_ms_); }

std::string TofOverdoorCounter::get_mode_text() const { return this->mode_ == OperatingMode::COUNT ? "Count" : "Monitor"; }

std::string TofOverdoorCounter::get_group_label(size_t group_index) const {
  return group_index == 0 ? "OUT vote zones (all sensors)" : "IN vote zones (all sensors)";
}

std::string TofOverdoorCounter::get_status_text(size_t index) const {
  if (index >= this->channels_.size()) {
    return "Unused";
  }
  return this->status_text_for_(this->channels_[index]);
}

std::string TofOverdoorCounter::get_sensor_health_text(size_t index) const {
  if (index >= this->channels_.size()) {
    return "Error";
  }
  return this->health_text_for_(this->channels_[index]);
}

std::string TofOverdoorCounter::get_source_label(size_t index) const {
  if (index >= this->channels_.size()) {
    return "Unused";
  }
  return this->channels_[index].source_label;
}

std::string TofOverdoorCounter::get_phase_text() const { return this->phase_text_; }

std::string TofOverdoorCounter::get_system_status_text() const { return this->system_status_text_(this->system_status_); }

std::string TofOverdoorCounter::get_last_direction_text() const { return this->last_direction_; }

std::string TofOverdoorCounter::get_last_detection_timestamp_text() const {
  return this->last_detection_ms_ == 0 ? "Never" : this->format_uptime_(this->last_detection_ms_);
}

std::string TofOverdoorCounter::get_last_reason_text() const { return this->last_reason_; }

std::string TofOverdoorCounter::get_passage_state_text() const { return this->passage_state_text_(this->passage_state_); }

std::string TofOverdoorCounter::get_debug_snapshot_text() const {
  if (this->history_count_ == 0) {
    return "No samples yet";
  }
  const auto &snapshot = this->history_[(this->history_head_ + HISTORY_SIZE - 1) % HISTORY_SIZE];
  std::ostringstream oss;
  oss << "t=" << snapshot.timestamp_ms << " active=" << this->sensor_mask_text_(snapshot.active_mask)
      << " rising=" << this->sensor_mask_text_(snapshot.rising_mask)
      << " falling=" << this->sensor_mask_text_(snapshot.falling_mask) << " state=" << this->get_passage_state_text();
  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    oss << " " << this->channels_[index].sensor_label << "[raw=" << snapshot.raw_distance[index]
        << ",filtered=" << snapshot.filtered_distance[index]
        << ",status=" << range_status_name(snapshot.range_status[index]) << "]";
  }
  return oss.str();
}

std::string TofOverdoorCounter::get_compact_state_text() const {
  std::ostringstream oss;
  oss << "status=" << this->system_status_text_(this->system_status_)
      << "\tphase=" << this->phase_text_
      << "\tstate=" << this->passage_state_text_(this->passage_state_)
      << "\tpresence=" << (this->get_presence_state() > 0.5f ? "1" : "0")
      << "\tlast_direction=" << this->last_direction_
      << "\tin=" << this->confirmed_in_count_
      << "\tout=" << this->confirmed_out_count_
      << "\tunsure_in=" << this->unsure_in_count_
      << "\tunsure_out=" << this->unsure_out_count_
      << "\tconfidence=" << static_cast<unsigned>(this->last_confidence_)
      << "\treason=" << this->last_reason_;

  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    const auto &channel = this->channels_[index];
    const float logic_distance = this->channel_logic_distance_(channel);
    oss << "\t" << channel.sensor_label << "_raw=" << channel.raw_distance
        << "\t" << channel.sensor_label << "_filtered=";
    if (std::isnan(logic_distance)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(logic_distance);
    }
    oss << "\t" << channel.sensor_label << "_drop=";
    if (std::isnan(channel.baseline) || std::isnan(logic_distance)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(channel.baseline - logic_distance);
    }
    oss << "\t" << channel.sensor_label << "_active=" << (channel.active ? "1" : "0")
        << "\t" << channel.sensor_label << "_status=" << range_status_name(channel.range_status);
    const float out_zone = this->zone_logic_distance_(channel.zones[ZONE_OUT]);
    const float in_zone = this->zone_logic_distance_(channel.zones[ZONE_IN]);
    oss << "\t" << channel.sensor_label << "_out_zone=";
    if (std::isnan(out_zone)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(out_zone);
    }
    oss << "\t" << channel.sensor_label << "_out_active=" << (channel.zones[ZONE_OUT].active ? "1" : "0")
        << "\t" << channel.sensor_label << "_in_zone=";
    if (std::isnan(in_zone)) {
      oss << "nan";
    } else {
      oss << static_cast<int>(in_zone);
    }
    oss << "\t" << channel.sensor_label << "_in_active=" << (channel.zones[ZONE_IN].active ? "1" : "0")
        << "\t" << channel.sensor_label << "_vote=" << channel.last_vote_text
        << "\t" << channel.sensor_label << "_path=" << channel.last_path_text;
  }
  return oss.str();
}

std::string TofOverdoorCounter::get_trace_log_text() const {
  std::ostringstream oss;
  oss << "# Roode compact trace\n";
  oss << "# " << this->get_compact_state_text() << "\n";
  oss << "# event_log_begin\n" << this->get_event_log() << "\n# event_log_end\n";
  oss << "t_ms\tstate\tactive\trising\tfalling";
  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    const auto &channel = this->channels_[index];
    oss << "\t" << channel.sensor_label << "_raw"
        << "\t" << channel.sensor_label << "_filtered"
        << "\t" << channel.sensor_label << "_drop"
        << "\t" << channel.sensor_label << "_status";
  }
  oss << "\n";

  const uint32_t count = std::min<uint32_t>(this->history_count_, HISTORY_SIZE);
  for (uint32_t offset = 0; offset < count; offset++) {
    const uint32_t index = (this->history_head_ + HISTORY_SIZE - count + offset) % HISTORY_SIZE;
    const auto &snapshot = this->history_[index];
    oss << snapshot.timestamp_ms << "\t" << this->passage_state_text_(static_cast<PassageState>(snapshot.passage_state))
        << "\t" << this->sensor_mask_text_(snapshot.active_mask)
        << "\t" << this->sensor_mask_text_(snapshot.rising_mask)
        << "\t" << this->sensor_mask_text_(snapshot.falling_mask);

    for (size_t sensor_index = 0; sensor_index < this->channels_.size() && sensor_index < SENSOR_COUNT; sensor_index++) {
      const auto &channel = this->channels_[sensor_index];
      const int filtered = snapshot.filtered_distance[sensor_index] == 0 ? -1 : snapshot.filtered_distance[sensor_index];
      int drop = -9999;
      if (!std::isnan(channel.baseline) && filtered >= 0) {
        drop = static_cast<int>(channel.baseline) - filtered;
      }
      oss << "\t" << snapshot.raw_distance[sensor_index]
          << "\t" << filtered
          << "\t" << drop
          << "\t" << range_status_name(snapshot.range_status[sensor_index]);
    }
    oss << "\n";
  }
  return oss.str();
}

std::string TofOverdoorCounter::get_blocked_sensor_text() const { return this->blocked_sensor_text_; }

std::string TofOverdoorCounter::get_summary() const {
  std::ostringstream oss;
  oss << "Inside " << this->people_inside_ << " | IN " << this->confirmed_in_count_ << " | OUT "
      << this->confirmed_out_count_ << " | Unsure IN " << this->unsure_in_count_ << " | Unsure OUT "
      << this->unsure_out_count_ << " | " << this->system_status_text_(this->system_status_) << " | "
      << this->passage_state_text_(this->passage_state_);
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
    oss << channel.sensor_label << "=independent";
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

std::string TofOverdoorCounter::get_event_log() const {
  if (this->event_log_count_ == 0) {
    return "No events yet";
  }
  std::ostringstream oss;
  const uint32_t count = std::min<uint32_t>(this->event_log_count_, EVENT_LOG_SIZE);
  for (uint32_t i = 0; i < count; i++) {
    if (i > 0) {
      oss << "\n";
    }
    const uint32_t index = (this->event_log_count_ - count + i) % EVENT_LOG_SIZE;
    oss << this->event_log_[index];
  }
  return oss.str();
}

}  // namespace tof_overdoor_counter
}  // namespace esphome
