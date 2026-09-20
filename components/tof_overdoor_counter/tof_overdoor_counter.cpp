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
constexpr uint8_t PERSISTED_STATE_VERSION = 7;
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
// A valid sub-30 mm result is near presence, not a failed sensor. Clamp only
// the detection distance; keep the raw result in the trace.
constexpr uint16_t MIN_LOGIC_DISTANCE_MM = 30;
constexpr uint16_t COVER_DISTANCE_MM = 100;
constexpr uint32_t OCCUPIED_DROPOUT_HOLD_MS = 150;
constexpr uint16_t MAX_VALID_DISTANCE_MM = 4000;
constexpr uint8_t ROODE_ZONE_WIDTH = 8;
constexpr uint8_t ROODE_ZONE_HEIGHT = 16;
constexpr uint8_t ROODE_OUT_ZONE_CENTER = 167;
constexpr uint8_t ROODE_IN_ZONE_CENTER = 231;

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

const char *zone_name(uint8_t zone_index) {
  return zone_index == ZONE_OUT ? "OUT-zone" : "IN-zone";
}

uint8_t roi_center_for_zone(uint8_t zone_index) {
  return zone_index == ZONE_OUT ? ROODE_OUT_ZONE_CENTER : ROODE_IN_ZONE_CENTER;
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
    for (auto &zone : channel.zones) zone.fresh = false;
    if (!channel.initialized) {
      continue;
    }
    if (!this->read_channel_(channel)) {
      // Recovery is deliberately deferred and isolated to this channel. A
      // single marginal sensor must never restart the other three streams.
    }
    App.feed_wdt();
  }

  this->update_sensor_health_();
  this->service_recovery_(millis());
  this->cycle_duration_ms_ = millis() - started;

  if (this->calibration_active_ && this->has_restored_calibration_()) this->calibration_active_ = false;
  if (this->calibration_active_) {
    this->process_calibration_();
    this->update_system_status_();
    this->cycle_duration_ms_ = millis() - started;
    return;
  }

  this->update_sensor_states_();
  this->process_calibration_();  // Calibrate a newly recovered, previously absent channel while idle.
  this->debug_log_sample_(millis());
  this->update_blocked_state_();

  if (this->mode_ == OperatingMode::COUNT) {
    this->update_detection_state_machine_();
  } else {
    this->clear_event_tracking_();
    this->update_passage_state_(PASSAGE_IDLE);
    this->person_standing_in_door_ = false;
    this->phase_text_ = this->ready_for_counting_() ? "Monitoring live distances" : "Waiting for stable readings";
  }

  this->record_history_snapshot_(millis());
  this->apply_idle_baseline_tracking_();
  this->update_system_status_();

  if (this->state_dirty_ && this->auto_save_enabled_) {
    this->persist_runtime_state();
  }
  this->cycle_duration_ms_ = millis() - started;
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
  VL53L1_Error status = VL53L1_ERROR_NONE;
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
  channel.initialized_ms = millis();
  channel.ranging_started = false;
  channel.last_error = 0;
  return true;
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
    channel.consecutive_errors = std::min<unsigned>(255, channel.consecutive_errors + 1U);
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
    channel.consecutive_errors = std::min<unsigned>(255, channel.consecutive_errors + 1U);
    return false;
  }

  status = sensor.ClearInterrupt();
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    channel.consecutive_errors = std::min<unsigned>(255, channel.consecutive_errors + 1U);
    return false;
  }

  const uint32_t now = millis();
  zone.fresh = true;
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
    zone.consecutive_invalid = std::min<unsigned>(255, zone.consecutive_invalid + 1U);
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

  this->update_zone_sampling_(zone, std::max<uint16_t>(MIN_LOGIC_DISTANCE_MM, result.Distance));
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
  if (result.Distance > MAX_VALID_DISTANCE_MM) {
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
  if (sensor.StopRanging() != VL53L1_ERROR_NONE) {
    channel.ranging_started = false;
    channel.consecutive_errors++;
    return false;
  }
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

void TofOverdoorCounter::fail_recovery_() {
  auto &channel = this->channels_[this->recovery_index_];
  this->set_xshut_(this->recovery_index_, false);
  channel.initialized = channel.ranging_started = false;
  channel.recovery_attempts = std::min<unsigned>(12, channel.recovery_attempts + 1U);
  channel.next_recovery_ms = millis() + std::min<uint32_t>(SENSOR_RECOVERY_MAX_MS,
      SENSOR_RECOVERY_BASE_MS << (channel.recovery_attempts > this->init_retries_ ? channel.recovery_attempts - this->init_retries_ : 0));
  this->recovery_stage_ = RecoveryStage::IDLE;
  ESP_LOGW(TAG, "%s recovery failed; retry scheduled", channel.sensor_label.c_str());
}

void TofOverdoorCounter::service_recovery_(uint32_t now) {
  if (this->recovery_stage_ == RecoveryStage::IDLE) {
    for (size_t i = 0; i < this->channels_.size(); ++i) {
      auto &channel = this->channels_[i];
      const bool overdue = now - channel.initialized_ms > 2000U;
      if (channel.initialized && channel.consecutive_errors < ERRORS_BEFORE_POWER_CYCLE &&
          (!overdue || this->channel_healthy_(channel, now, false))) continue;
      if (channel.next_recovery_ms && static_cast<int32_t>(now - channel.next_recovery_ms) < 0) continue;
      this->recovery_index_ = i;
      this->set_xshut_(i, false);
      channel.initialized = channel.ranging_started = false;
      channel.has_reading = channel.active = false;
      channel.stale = true;
      // Keep calibration, but never carry old samples or edges across reset.
      for (auto &zone : channel.zones) {
        const auto saved = this->build_persisted_calibration_(zone);
        zone = ZoneState{};
        this->restore_persisted_calibration_(channel, &zone - channel.zones, saved);
      }
      channel.sensor = std::make_unique<VL53L1X_ULD>();
      this->recovery_deadline_ = now + 15;
      this->recovery_stage_ = RecoveryStage::POWER_OFF;
      break;
    }
    return;
  }
  auto &channel = this->channels_[this->recovery_index_];
  auto &sensor = *channel.sensor;
  switch (this->recovery_stage_) {
    case RecoveryStage::POWER_OFF:
      if (static_cast<int32_t>(now - this->recovery_deadline_) < 0) return;
      this->set_xshut_(this->recovery_index_, true);
      this->recovery_deadline_ = now + this->wake_delay_ms_ + this->timeout_ms_;
      channel.initialized_ms = now + this->wake_delay_ms_;
      this->recovery_stage_ = RecoveryStage::BOOT;
      return;
    case RecoveryStage::BOOT: {
      if (static_cast<int32_t>(now - channel.initialized_ms) < 0) return;
      if (static_cast<int32_t>(now - this->recovery_deadline_) >= 0) {
        this->fail_recovery_();
        // A bus reset invalidates temporal evidence, including healthy channels.
        this->clear_event_tracking_();
        this->startup_clear_validated_ = false;
        this->boot_clear_since_ms_ = 0;
        this->recover_wire_();
        return;
      }
      uint8_t booted = 0;
      if (sensor.GetBootState(&booted) != VL53L1_ERROR_NONE || !(booted & 1)) return;
      if (!this->set_temp_address_(sensor, channel.address)) { this->fail_recovery_(); return; }
      this->recovery_deadline_ = now + this->post_address_delay_ms_;
      this->recovery_stage_ = RecoveryStage::ADDRESS_SETTLE;
      return;
    }
    case RecoveryStage::ADDRESS_SETTLE:
      if (static_cast<int32_t>(now - this->recovery_deadline_) < 0) return;
      this->sensor_init_.reset(now, this->timeout_ms_);
      this->recovery_stage_ = RecoveryStage::INIT_REGISTERS;
      return;
    case RecoveryStage::INIT_REGISTERS: {
      const auto result = this->sensor_init_.step(sensor, now);
      if (result == counting_core::SensorInit::FAILED) { this->fail_recovery_(); return; }
      if (result == counting_core::SensorInit::READY) this->recovery_stage_ = RecoveryStage::CONFIGURE;
      return;
    }
    case RecoveryStage::CONFIGURE:
      if (!this->configure_sensor_(channel) || !this->restart_ranging_(channel)) {
        this->fail_recovery_(); return;
      }
      channel.consecutive_errors = channel.consecutive_invalid = 0;
      channel.recovery_attempts = 0;
      channel.next_recovery_ms = now + 2000U;
      this->recovery_stage_ = RecoveryStage::IDLE;
      ESP_LOGI(TAG, "%s recovered at 0x%02X", channel.sensor_label.c_str(), channel.address);
      return;
    default: return;
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
                (state.version == PERSISTED_STATE_VERSION || state.version == 5 || state.version == 6);

  if (loaded && state.version < PERSISTED_STATE_VERSION) {
    // Older calibration counted scheduler ticks, not independent measurements.
    // Retain installation tuning/counts but obtain new per-ROI statistics.
    for (auto &sensor : state.calibrations) for (auto &zone : sensor) zone.valid = 0;
    state.version = PERSISTED_STATE_VERSION;
    this->state_dirty_ = true;
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
  // Public actions only schedule discovery; no sleeps in an HTTP callback.
  if (this->persisted_state_loaded_) this->persist_runtime_state();
  this->channels_.clear();
  this->channels_.resize(this->xshut_pins_.size());
  this->clear_event_tracking_();
  this->startup_clear_validated_ = false;
  this->boot_clear_since_ms_ = 0;
  this->set_all_xshut_(false);
  this->recovery_stage_ = RecoveryStage::IDLE;
  const uint32_t now = millis();
  for (size_t i = 0; i < this->channels_.size(); ++i) {
    auto &channel = this->channels_[i];
    channel.pin_number = this->xshut_pin_numbers_[i];
    channel.address = this->base_address_ + i;
    channel.group = group_for_pin(channel.pin_number);
    channel.sensor_label = sensor_name_for_pin(channel.pin_number);
    channel.source_label = channel.sensor_label + " / GPIO" + std::to_string(channel.pin_number);
    channel.next_recovery_ms = now + this->wake_delay_ms_;
  }
  this->load_persisted_state_();
  this->persisted_state_loaded_ = true;
  this->last_discovery_ms_ = now;
  this->calibration_active_ = true;
  this->phase_text_ = "Sensor discovery scheduled";
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
  this->person_standing_in_door_ = false;
  this->clear_event_tracking_();
  this->last_reason_ = invert_direction ? "Direction reversed immediately" : "Direction restored immediately";
  this->state_dirty_ = true;
}

void TofOverdoorCounter::process_calibration_() {
  const uint32_t now = millis();
  if (this->event_active_ || this->active_sensor_count_() > 0) return;
  uint8_t clear = 0;
  for (const auto &channel : this->channels_) {
    if (!this->channel_healthy_(channel, now, false)) continue;
    bool empty = true;
    for (const auto &zone : channel.zones) {
      empty = empty && zone.raw_distance >= this->minimum_clear_distance_mm_;
      if (zone.calibrated) empty = empty && zone.baseline - zone.raw_distance < this->adaptive_release_delta_(zone);
    }
    if (empty) ++clear;
  }
  if (clear < this->min_valid_sensors_) {
    this->calibration_clear_since_ms_ = 0;
    for (auto &channel : this->channels_)
      for (auto &zone : channel.zones) if (!zone.calibrated) { zone.calibration = {}; zone.calibration_samples = 0; }
    if (this->calibration_active_) this->phase_text_ = "Waiting for a healthy, empty doorway";
    return;
  }
  if (!this->calibration_clear_since_ms_) this->calibration_clear_since_ms_ = now;
  if (now - this->calibration_clear_since_ms_ < CALIBRATION_CLEAR_SETTLE_MS) return;
  bool changed = false;
  for (auto &channel : this->channels_) {
    if (!this->channel_healthy_(channel, now, false) || channel.calibrated) continue;
    for (auto &zone : channel.zones) {
      if (!zone.fresh) continue;
      if (!zone.valid_measurement || zone.raw_distance < this->minimum_clear_distance_mm_) {
        zone.calibration = {};
        zone.calibration_samples = 0;
        continue;
      }
      if (zone.calibration.count < this->calibration_samples_) zone.calibration.add(zone.raw_distance);
      zone.calibration_samples = zone.calibration.count;
    }
    if (channel.zones[0].calibration.count < this->calibration_samples_ ||
        channel.zones[1].calibration.count < this->calibration_samples_) continue;
    bool stable = true;
    for (const auto &zone : channel.zones)
      stable = stable && zone.calibration.deviation() <= this->baseline_tolerance_mm_ &&
          zone.calibration.high - zone.calibration.low <= this->baseline_tolerance_mm_ * 4.0f;
    if (!stable) {
      for (auto &zone : channel.zones) { zone.calibration = {}; zone.calibration_samples = 0; }
      continue;
    }
    for (auto &zone : channel.zones) {
      zone.baseline = zone.calibration.mean;
      zone.noise = std::max(1.0f, zone.calibration.deviation());
      zone.calibration_quality = clamp_quality(100.0f - zone.noise * 2.5f -
          (zone.calibration.high - zone.calibration.low) * 0.3f);
      zone.calibrated = true;
    }
    channel.calibrated = true;
    channel.baseline = (channel.zones[0].baseline + channel.zones[1].baseline) * 0.5f;
    channel.noise = std::max(channel.zones[0].noise, channel.zones[1].noise);
    channel.calibration_quality = std::min(channel.zones[0].calibration_quality, channel.zones[1].calibration_quality);
    changed = true;
  }
  if (this->calibration_active_ && this->has_restored_calibration_()) {
    this->calibration_active_ = false;
    this->startup_clear_validated_ = false;
    this->boot_clear_since_ms_ = 0;
    this->phase_text_ = "Calibration complete; verifying clear doorway";
    this->last_reason_ = "Calibrated with independent, valid ROI measurements";
  }
  if (changed) {
    this->state_dirty_ = true;
    if (this->auto_save_enabled_) this->persist_runtime_state();
  }
}

void TofOverdoorCounter::update_sensor_states_() {
  const uint32_t now = millis();

  for (auto &channel : this->channels_) {
    const bool was_active = channel.active;
    const uint32_t previous_active_since = channel.active_since_ms;
    channel.rising_edge = false;
    channel.falling_edge = false;

    if (!channel.initialized || !channel.has_reading || channel.stale) {
      for (auto &zone : channel.zones) {
        zone.debounce = {};
        zone.rising_edge = false;
        zone.falling_edge = false;
        zone.active = false;
        zone.blocked = false;
        zone.near_pending = false;
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
      const bool zone_stale = zone.last_good_read_ms == 0 || (now - zone.last_good_read_ms) > this->stale_reading_ms_();

      if (!zone.has_reading || !zone.calibrated || !this->zone_measurement_usable_(zone, now) ||
          std::isnan(zone.baseline) || zone_stale || std::isnan(distance)) {
        zone.debounce = {};
        zone.active = false;
        zone.blocked = false;
        zone.active_since_ms = 0;
        zone.near_pending = false;
        continue;
      }
      // A held sample cannot supply a new edge or confirm an empty doorway.
      if (!zone.valid_measurement) continue;
      if (zone.fresh) {
        const float drop = zone.baseline - distance;
        const bool target = zone.active ? drop > this->adaptive_release_delta_(zone)
                                        : drop >= this->adaptive_trigger_delta_(zone);
        if (zone.debounce.update(target, now, this->debounce_ms_)) {
          zone.active = zone.debounce.active;
          if (zone.active) {
            zone.active_since_ms = zone.debounce.since;
            zone.last_rising_ms = now;
            zone.rising_edge = true;
          } else {
            zone.last_falling_ms = now;
            zone.falling_edge = true;
            zone.active_duration_ms = now - zone.active_since_ms;
            zone.active_since_ms = 0;
            zone.blocked = false;
          }
        }
      }

      if (zone.fresh) {
        if (zone.active && zone.raw_distance <= COVER_DISTANCE_MM) {
          if (!zone.near_pending) { zone.near_pending = true; zone.near_since_ms = now; }
          zone.blocked = now - zone.near_since_ms >= this->blocked_timeout_ms_;
        } else {
          zone.near_pending = false;
          zone.blocked = false;
        }
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
      if (channel.active_since_ms == 0 || (zone.active_since_ms != 0 && zone.active_since_ms < channel.active_since_ms)) {
        channel.active_since_ms = zone.active_since_ms;
      }
    }

    // Persistent cover is excluded from fusion, but remains monitored for release.
    channel.blocked = channel.zones[0].blocked && channel.zones[1].blocked;
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
    if (this->active_sensor_count_() == 0 && this->healthy_sensor_count_() >= std::max(this->min_valid_sensors_, this->min_event_sensors_)) {
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
  if (this->event_active_ || this->calibration_active_ || !this->startup_clear_validated_ || this->active_sensor_count_() > 0) {
    return;
  }

  for (auto &channel : this->channels_) {
    if (!channel.initialized || !channel.has_reading || !channel.calibrated || channel.active || channel.blocked) {
      continue;
    }
    for (auto &zone : channel.zones) {
      const float distance = zone.raw_distance;
      if (!zone.fresh || !zone.valid_measurement || !zone.calibrated || zone.active || std::isnan(distance) || std::isnan(zone.baseline)) {
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
  this->fusion_.reset();
  this->event_active_ = false;
  this->event_started_ms_ = 0;
  this->person_standing_in_door_ = false;
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

void TofOverdoorCounter::update_detection_state_machine_() {
  const uint32_t now = millis();
  std::array<uint8_t, SENSOR_COUNT> states{}, fresh{};
  uint8_t healthy = 0;
  bool unsettled = false;
  for (size_t i = 0; i < this->channels_.size(); ++i) {
    auto &channel = this->channels_[i];
    if (!channel.blocked && this->channel_healthy_(channel, now)) healthy |= 1U << i;
    for (size_t z = 0; z < SENSOR_ZONE_COUNT; ++z) {
      if ((healthy & (1U << i)) && channel.zones[z].debounce.hits) unsettled = true;
      if (channel.zones[z].active) states[i] |= 1U << z;
      if (channel.zones[z].fresh && channel.zones[z].valid_measurement) fresh[i] |= 1U << z;
    }
  }
  this->fusion_.required = this->min_event_sensors_;
  this->fusion_.required_health = this->min_valid_sensors_;
  this->fusion_.clear_ms = std::max(this->direction_window_ms_, this->cooldown_ms_);
  this->fusion_.minimum_ms = this->min_active_duration_ms_;
  this->fusion_.agreement_ms = this->detection_timeout_ms_;
  this->fusion_.invert = this->invert_direction_;
  const auto result = this->fusion_.update(now, this->startup_clear_validated_ ? healthy : 0, states, fresh, unsettled);
  this->event_active_ = this->fusion_.active();
  this->event_started_ms_ = this->fusion_.started();
  this->person_standing_in_door_ = this->event_active_ && now - this->event_started_ms_ >= this->standing_timeout_ms_;
  for (size_t i = 0; i < this->channels_.size(); ++i) {
    const auto &path = this->fusion_.path(i);
    this->channels_[i].last_path_text = "first=" + std::to_string(path.first) + " last=" +
        std::to_string(path.last) + " seen=" + std::to_string(path.seen) + (path.ambiguous ? " ambiguous" : "");
  }
  if (result.decision != counting_core::Decision::NONE) {
    DetectionOutcome outcome = OUTCOME_NONE;
    switch (result.decision) {
      case counting_core::Decision::IN: outcome = OUTCOME_IN; break;
      case counting_core::Decision::OUT: outcome = OUTCOME_OUT; break;
      case counting_core::Decision::UNSURE_IN: outcome = OUTCOME_UNSURE_IN; break;
      case counting_core::Decision::UNSURE_OUT: outcome = OUTCOME_UNSURE_OUT; break;
      default: break;
    }
    this->last_decision_latency_ms_ = result.duration_ms;
    const auto votes = std::max(result.in_votes, result.out_votes);
    const uint8_t confidence = outcome == OUTCOME_NONE ? 0 : std::min<unsigned>(99, votes * 25);
    this->register_detection_(outcome, confidence, std::string(this->ready_for_counting_() ? "Completed clear-doorway episode: " : "Sensor health interrupted episode: ") +
        std::to_string(result.in_votes) + " IN, " + std::to_string(result.out_votes) +
        " OUT; required=" + std::to_string(this->fusion_.required) +
        "; duration=" + std::to_string(result.duration_ms) + "ms; score is heuristic, not accuracy");
    this->update_passage_state_(outcome == OUTCOME_IN || outcome == OUTCOME_OUT ? PASSAGE_COMPLETED : PASSAGE_CANCELLED);
  } else {
    this->update_passage_state_(this->event_active_ ? PASSAGE_OCCUPIED : PASSAGE_IDLE);
  }
  this->phase_text_ = !this->ready_for_counting_() ? "Waiting for healthy sensors and clear doorway" :
      this->person_standing_in_door_ ? "Person standing in doorway" :
      this->event_active_ ? "Tracking passage; waiting for doorway to clear" : "Ready";
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
                   " - evidence score " + std::to_string(confidence) + "/100");
  this->state_dirty_ = true;
}

void TofOverdoorCounter::record_history_snapshot_(uint32_t now) {
  bool fresh = false;
  for (const auto &channel : this->channels_) for (const auto &zone : channel.zones) fresh |= zone.fresh;
  if (!fresh && this->history_count_ &&
      now - this->history_[(this->history_head_ + HISTORY_SIZE - 1) % HISTORY_SIZE].timestamp_ms < TRACE_SAMPLE_INTERVAL_MS) return;
  auto &snapshot = this->history_[this->history_head_];
  snapshot = HistorySample{};
  snapshot.timestamp_ms = now;
  snapshot.passage_state = this->passage_state_;
  snapshot.outcome = this->last_detection_outcome_;
  for (size_t i = 0; i < this->channels_.size(); ++i) {
    const auto &channel = this->channels_[i];
    if (this->channel_healthy_(channel, now)) snapshot.healthy_mask |= 1U << i;
    for (size_t z = 0; z < SENSOR_ZONE_COUNT; ++z) {
      const auto &zone = channel.zones[z];
      const size_t k = i * SENSOR_ZONE_COUNT + z;
      snapshot.sample_ms[k] = zone.last_update_ms;
      snapshot.raw_distance[k] = zone.raw_distance;
      snapshot.filtered_distance[k] = std::isfinite(zone.filtered_distance) ? zone.filtered_distance : 0;
      snapshot.baseline[k] = std::isfinite(zone.baseline) ? zone.baseline : 0;
      snapshot.trigger[k] = this->adaptive_trigger_delta_(zone);
      snapshot.release[k] = this->adaptive_release_delta_(zone);
      snapshot.range_status[k] = zone.range_status;
      if (zone.valid_measurement && now - zone.last_good_read_ms <= this->stale_reading_ms_()) snapshot.valid_mask |= 1U << k;
      if (zone.fresh) snapshot.fresh_mask |= 1U << k;
      if (zone.active) snapshot.active_mask |= 1U << k;
      if (zone.rising_edge) snapshot.rising_mask |= 1U << k;
      if (zone.falling_edge) snapshot.falling_mask |= 1U << k;
    }
  }
  this->history_head_ = (this->history_head_ + 1) % HISTORY_SIZE;
  if (this->history_count_ < HISTORY_SIZE) ++this->history_count_;
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
    channel.stale = false;
    for (const auto &zone : channel.zones) channel.stale = channel.stale || !zone.has_reading || (now - zone.last_good_read_ms) > this->stale_reading_ms_();
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

  if (reporting < this->min_valid_sensors_ || healthy < std::max(this->min_valid_sensors_, this->min_event_sensors_)) {
    this->system_status_ = STATUS_ERROR;
    return;
  }

  if (!this->startup_clear_validated_) { this->system_status_ = STATUS_BOOTING; return; }

  if (this->blocked_sensor_text_ != "None") {
    this->system_status_ = STATUS_BLOCKED;
    return;
  }

  if (this->event_active_) {
    this->system_status_ = STATUS_DETECTING;
    return;
  }

  if (healthy < SENSOR_COUNT || calibrated < SENSOR_COUNT) {
    this->system_status_ = STATUS_DEGRADED;
    return;
  }

  this->system_status_ = STATUS_READY;
}

bool TofOverdoorCounter::zone_measurement_usable_(const ZoneState &zone, uint32_t now) const {
  if (zone.valid_measurement) return true;
  // Bridge only a short optical dropout in an already occupied field. Never
  // bridge missing initialization, a bus/hardware fault, or a clear field.
  const bool optical_dropout = zone.range_status == SigmaFail || zone.range_status == SignalFail ||
      zone.range_status == MinRangeFail || zone.range_status == PhaseOutOfLimit || zone.range_status == WrapTargetFail;
  return zone.active && optical_dropout && zone.last_good_read_ms != 0 &&
      now - zone.last_good_read_ms <= OCCUPIED_DROPOUT_HOLD_MS;
}

bool TofOverdoorCounter::channel_healthy_(const Channel &channel, uint32_t now, bool calibrated) const {
  if (!channel.initialized || !channel.ranging_started || channel.consecutive_errors != 0) return false;
  for (const auto &zone : channel.zones) {
    if (!zone.has_reading || !this->zone_measurement_usable_(zone, now) || now - zone.last_good_read_ms > this->stale_reading_ms_() ||
        (calibrated && (!zone.calibrated || !std::isfinite(zone.baseline)))) return false;
  }
  return true;
}

bool TofOverdoorCounter::ready_for_counting_() const {
  return !this->calibration_active_ && this->startup_clear_validated_ &&
      this->healthy_sensor_count_() >= std::max(this->min_valid_sensors_, this->min_event_sensors_);
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

uint8_t TofOverdoorCounter::healthy_sensor_count_() const {
  uint8_t count = 0;
  for (const auto &channel : this->channels_) if (!channel.blocked && this->channel_healthy_(channel, millis())) ++count;
  return count;
}

uint8_t TofOverdoorCounter::reporting_sensor_count_() const {
  uint8_t count = 0;
  for (const auto &channel : this->channels_) if (this->channel_healthy_(channel, millis(), false)) ++count;
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

bool TofOverdoorCounter::group_is_active_(SensorGroup group) const {
  return this->active_sensor_count_for_group_(group) > 0;
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
  if (!this->channel_healthy_(channel, millis(), false)) return "Error";
  if (!channel.calibrated || channel.blocked) return "Warning";
  return "OK";
}

std::string TofOverdoorCounter::status_text_for_(const Channel &channel) const {
  if (!channel.initialized) return "Missing / recovering";
  if (channel.consecutive_errors) return "Read error " + std::to_string(channel.last_error);
  for (size_t z = 0; z < SENSOR_ZONE_COUNT; ++z) {
    const auto &zone = channel.zones[z];
    if (!zone.has_reading) return std::string(zone_name(z)) + " waiting";
    if (millis() - zone.last_good_read_ms > this->stale_reading_ms_()) return std::string(zone_name(z)) + " stale";
    if (!zone.valid_measurement) return std::string(zone_name(z)) + " invalid range " + range_status_name(zone.range_status);
  }
  if (!channel.calibrated || this->calibration_active_) return "Calibrating";
  if (channel.blocked) return "Blocked";
  if (channel.active) return "Triggered";
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

std::string TofOverdoorCounter::get_trace_log_text(uint32_t after_ms, size_t limit) const {
  limit = std::max<size_t>(1, std::min<size_t>(64, limit));
  std::ostringstream oss;
  oss << "# uptime_ms=" << millis() << "\n";
  oss << "# Roode ROI trace v2; masks use sensor_index*2+zone (OUT=0, IN=1)\n";
  oss << "# debounce_ms=" << this->debounce_ms_ << " clear_ms=" << std::max(this->direction_window_ms_, this->cooldown_ms_)
      << " quorum=" << unsigned(this->min_event_sensors_) << " min_healthy=" << unsigned(this->min_valid_sensors_)
      << " agreement_ms=" << this->detection_timeout_ms_ << " invert=" << this->invert_direction_ << "\n";
  oss << "# event_log_begin\n" << this->get_event_log() << "\n# event_log_end\n";
  oss << "t_ms\tstate\toutcome\thealthy\tvalid\tfresh\tactive\trising\tfalling";
  for (size_t i = 0; i < this->channels_.size(); ++i) for (size_t z = 0; z < SENSOR_ZONE_COUNT; ++z) {
    const std::string label = this->channels_[i].sensor_label + (z == 0 ? "_out" : "_in");
    for (const char *field : {"sample_ms", "raw", "filtered", "baseline", "drop", "trigger", "release", "status"})
      oss << "\t" << label << "_" << field;
  }
  oss << "\n";
  size_t emitted = 0;
  for (size_t offset = 0; offset < this->history_count_ && emitted < limit; ++offset) {
    const auto &v = this->history_[(this->history_head_ + HISTORY_SIZE - this->history_count_ + offset) % HISTORY_SIZE];
    if (after_ms && static_cast<int32_t>(v.timestamp_ms - after_ms) <= 0) continue;
    ++emitted;
    oss << v.timestamp_ms << "\t" << unsigned(v.passage_state) << "\t" << unsigned(v.outcome)
        << "\t" << unsigned(v.healthy_mask) << "\t" << unsigned(v.valid_mask) << "\t" << unsigned(v.fresh_mask)
        << "\t" << unsigned(v.active_mask) << "\t" << unsigned(v.rising_mask) << "\t" << unsigned(v.falling_mask);
    for (size_t k = 0; k < this->channels_.size() * SENSOR_ZONE_COUNT; ++k)
      oss << "\t" << v.sample_ms[k] << "\t" << v.raw_distance[k] << "\t" << v.filtered_distance[k]
          << "\t" << v.baseline[k] << "\t" << (int(v.baseline[k]) - int(v.filtered_distance[k]))
          << "\t" << v.trigger[k] << "\t" << v.release[k] << "\t" << unsigned(v.range_status[k]);
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
