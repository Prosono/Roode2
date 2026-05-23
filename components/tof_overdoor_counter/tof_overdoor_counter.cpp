#include "tof_overdoor_counter.h"

#include <algorithm>
#include <cstdint>
#include <sstream>

namespace esphome {
namespace tof_overdoor_counter {
namespace {

static const char *const TAG = "tof_overdoor_counter";
constexpr uint8_t PERSISTED_STATE_VERSION = 4;
constexpr uint32_t STALE_READING_MS = 450;
constexpr uint32_t COLD_BOOT_SETTLE_MS = 250;
constexpr uint32_t CALIBRATION_CLEAR_SETTLE_MS = 120;
constexpr uint32_t TRACE_SAMPLE_INTERVAL_MS = 25;
constexpr float FILTER_ALPHA = 0.65f;
constexpr float BASELINE_TRACK_ALPHA = 0.015f;
constexpr float NOISE_TRACK_ALPHA = 0.08f;
constexpr uint16_t MIN_VALID_DISTANCE_MM = 30;
constexpr uint16_t MAX_VALID_DISTANCE_MM = 4000;
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
  // The enclosure has two vertical working pairs: U3/U4 on one doorway side and U7/U8 on the other.
  // U5/U6 are the broken middle pair and are intentionally not part of this four-sensor component.
  switch (pin_number) {
    case 16:
    case 17:
      return GROUP_OUT;
    case 23:
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
  ESP_LOGI(TAG, "Cold boot settle for %u ms before sensor bring-up", static_cast<unsigned>(COLD_BOOT_SETTLE_MS));
  delay(COLD_BOOT_SETTLE_MS);
  this->prepare_xshut_pins_();
  if (!this->initialize_wire_()) {
    this->mark_failed();
    return;
  }
  this->init_preferences_();
  this->rediscover();
}

void TofOverdoorCounter::update() {
  if (this->channels_.empty()) {
    this->system_status_ = STATUS_ERROR;
    this->phase_text_ = "No sensors discovered";
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
    this->start_all_ranging_();
  }

  this->update_sensor_health_();

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
  ESP_LOGCONFIG(TAG, "  Clear Threshold: %u mm", this->clear_threshold_mm_);
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
  const EDistanceMode sensor_distance_mode =
      this->distance_mode_ == DISTANCE_MODE_SHORT ? EDistanceMode::Short : EDistanceMode::Long;
  status = sensor.SetDistanceMode(sensor_distance_mode);
  if (status != VL53L1_ERROR_NONE) {
    channel.last_error = status;
    return false;
  }
  const uint16_t min_timing_budget = this->distance_mode_ == DISTANCE_MODE_SHORT ? 20 : 33;
  const uint16_t timing_budget_ms = std::max<uint16_t>(this->timing_budget_ms_, min_timing_budget);
  const uint16_t intermeasurement_ms = std::max<uint16_t>(this->intermeasurement_ms_, timing_budget_ms);

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
    this->restart_ranging_(channel);
    return false;
  }

  if (!ready) {
    return true;
  }

  VL53L1X_Result_t result{};
  status = sensor.GetResult(&result);
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

  channel.raw_distance = result.Distance;
  channel.range_status = result.Status;
  channel.signal_per_spad = result.SigPerSPAD;
  channel.ambient_rate = result.Ambient;
  channel.spad_count = result.NumSPADs;
  channel.last_update_ms = millis();

  if (!this->range_result_is_valid_(result)) {
    channel.valid_measurement = false;
    channel.sample_rejected = true;
    channel.consecutive_invalid++;
    channel.last_error = 0;
    channel.consecutive_errors = 0;
    channel.last_read_duration_ms = millis() - started;
    if (this->debug_logging_) {
      ESP_LOGD(TAG, "%s rejected range sample distance=%u status=%s signal=%u ambient=%u spads=%u",
               channel.sensor_label.c_str(), result.Distance, range_status_name(result.Status), result.SigPerSPAD,
               result.Ambient, result.NumSPADs);
    }
    return true;
  }

  this->update_channel_sampling_(channel, result.Distance);
  const float sampled_distance = channel.has_sampled_distance ? static_cast<float>(channel.sampled_distance)
                                                              : static_cast<float>(result.Distance);
  channel.filtered_distance =
      std::isnan(channel.filtered_distance) ? sampled_distance
                                            : (channel.filtered_distance * (1.0f - FILTER_ALPHA)) +
                                                  (sampled_distance * FILTER_ALPHA);
  channel.median_distance = sampled_distance;
  channel.has_reading = true;
  channel.valid_measurement = true;
  channel.sample_rejected = false;
  channel.last_error = 0;
  channel.consecutive_errors = 0;
  channel.consecutive_invalid = 0;
  channel.last_good_read_ms = channel.last_update_ms;
  channel.last_read_duration_ms = millis() - started;
  return true;
}

bool TofOverdoorCounter::range_result_is_valid_(const VL53L1X_Result_t &result) const {
  if (result.Distance < MIN_VALID_DISTANCE_MM || result.Distance > MAX_VALID_DISTANCE_MM) {
    return false;
  }
  return result.Status == RangeValid || result.Status == RangeValidNoWrapCheck;
}

void TofOverdoorCounter::update_channel_sampling_(Channel &channel, uint16_t distance) {
  channel.samples.insert(channel.samples.begin(), distance);
  if (channel.samples.size() > this->sampling_size_) {
    channel.samples.pop_back();
  }

  if (channel.samples.empty()) {
    channel.has_sampled_distance = false;
    channel.sampled_distance = 0;
    return;
  }

  std::vector<uint16_t> sorted = channel.samples;
  std::sort(sorted.begin(), sorted.end());
  channel.sampled_distance = sorted[sorted.size() / 2];
  channel.has_sampled_distance = true;
}

float TofOverdoorCounter::channel_logic_distance_(const Channel &channel) const {
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

void TofOverdoorCounter::restore_persisted_calibration_(Channel &channel, size_t index,
                                                        const PersistedCalibration &persisted) {
  if (index >= SENSOR_COUNT || persisted.valid == 0) {
    return;
  }
  channel.baseline = static_cast<float>(persisted.baseline_mm);
  channel.noise = static_cast<float>(std::max<uint16_t>(persisted.noise_mm, 1));
  channel.calibration_quality = persisted.quality;
  channel.calibrated = persisted.valid != 0;
}

TofOverdoorCounter::PersistedCalibration TofOverdoorCounter::build_persisted_calibration_(const Channel &channel) const {
  PersistedCalibration calibration{};
  if (!channel.calibrated || std::isnan(channel.baseline)) {
    return calibration;
  }
  calibration.valid = 1;
  calibration.baseline_mm = static_cast<uint16_t>(std::max(0.0f, channel.baseline));
  calibration.noise_mm = static_cast<uint16_t>(std::max(1.0f, std::isnan(channel.noise) ? 1.0f : channel.noise));
  calibration.quality = channel.calibration_quality;
  return calibration;
}

void TofOverdoorCounter::load_persisted_state_() {
  if (!this->persisted_state_ready_) {
    return;
  }

  PersistedState state{};
  bool loaded = this->persisted_state_pref_.load(&state) && state.version == PERSISTED_STATE_VERSION;

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
      state.clear_threshold_mm = legacy.clear_threshold_mm;
      state.baseline_tolerance_mm = legacy.baseline_tolerance_mm;
      state.minimum_clear_distance_mm = legacy.minimum_clear_distance_mm;
      state.debounce_ms = legacy.debounce_ms;
      state.detection_timeout_ms = legacy.detection_timeout_ms;
      state.cooldown_ms = legacy.cooldown_ms;
      state.blocked_timeout_ms = legacy.blocked_timeout_ms;
      state.standing_timeout_ms = legacy.standing_timeout_ms;
      state.calibration_samples = legacy.calibration_samples;
      state.max_people_inside = legacy.max_people_inside;
      state.min_valid_sensors = legacy.min_valid_sensors;
      state.auto_save_enabled = legacy.auto_save_enabled;
      state.invert_direction = legacy.invert_direction;
      for (size_t index = 0; index < SENSOR_COUNT; index++) {
        state.calibrations[index] = legacy.calibrations[index];
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

  this->people_inside_ = std::max<int32_t>(0, state.people_inside);
  this->confirmed_in_count_ = state.confirmed_in;
  this->confirmed_out_count_ = state.confirmed_out;
  this->unsure_in_count_ = state.unsure_in;
  this->unsure_out_count_ = state.unsure_out;
  this->trigger_threshold_mm_ = sanitize_persisted<uint16_t>(state.trigger_threshold_mm, 40, 3000, 320);
  this->clear_threshold_mm_ = sanitize_persisted<uint16_t>(state.clear_threshold_mm, 20, 2500, 180);
  this->baseline_tolerance_mm_ = sanitize_persisted<uint16_t>(state.baseline_tolerance_mm, 10, 500, 80);
  this->minimum_clear_distance_mm_ = sanitize_persisted<uint16_t>(state.minimum_clear_distance_mm, 100, 4000, 600);
  this->debounce_ms_ = sanitize_persisted<uint16_t>(state.debounce_ms, 5, 5000, 45);
  this->detection_timeout_ms_ = sanitize_persisted<uint16_t>(state.detection_timeout_ms, 200, 20000, 1600);
  this->cooldown_ms_ = sanitize_persisted<uint16_t>(state.cooldown_ms, 0, 20000, 500);
  this->blocked_timeout_ms_ = sanitize_persisted<uint16_t>(state.blocked_timeout_ms, 200, 60000, 1800);
  this->standing_timeout_ms_ = sanitize_persisted<uint16_t>(state.standing_timeout_ms, 200, 60000, 2200);
  this->min_event_sensors_ = sanitize_persisted<uint16_t>(state.min_event_sensors, 2, SENSOR_COUNT, 2);
  this->min_active_duration_ms_ = sanitize_persisted<uint16_t>(state.min_active_duration_ms, 0, 1000, 35);
  this->direction_window_ms_ = sanitize_persisted<uint16_t>(state.direction_window_ms, 10, 1000, 90);
  this->calibration_samples_ = sanitize_persisted<uint16_t>(state.calibration_samples, 4, 128, 24);
  this->max_people_inside_ = sanitize_persisted<uint16_t>(state.max_people_inside, 1, 5000, 50);
  this->min_valid_sensors_ = sanitize_persisted<uint8_t>(state.min_valid_sensors, 2, SENSOR_COUNT, 3);
  this->auto_save_enabled_ = state.auto_save_enabled != 0;
  this->invert_direction_ = state.invert_direction != 0;
  this->debug_logging_ = state.debug_logging != 0;

  if (this->clear_threshold_mm_ >= this->trigger_threshold_mm_) {
    this->clear_threshold_mm_ = std::max<uint16_t>(20, this->trigger_threshold_mm_ / 2);
  }
  this->apply_calibration_defaults_();

  for (size_t index = 0; index < this->channels_.size() && index < SENSOR_COUNT; index++) {
    this->restore_persisted_calibration_(this->channels_[index], index, state.calibrations[index]);
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
  this->clear_threshold_mm_ = sanitize_persisted<uint16_t>(this->clear_threshold_mm_, 20, 2500, 180);
  this->baseline_tolerance_mm_ = sanitize_persisted<uint16_t>(this->baseline_tolerance_mm_, 10, 500, 80);
  this->minimum_clear_distance_mm_ = sanitize_persisted<uint16_t>(this->minimum_clear_distance_mm_, 100, 4000, 600);
  this->debounce_ms_ = sanitize_persisted<uint32_t>(this->debounce_ms_, 5, 5000, 45);
  this->detection_timeout_ms_ = sanitize_persisted<uint32_t>(this->detection_timeout_ms_, 200, 20000, 1600);
  this->cooldown_ms_ = sanitize_persisted<uint32_t>(this->cooldown_ms_, 0, 20000, 500);
  this->blocked_timeout_ms_ = sanitize_persisted<uint32_t>(this->blocked_timeout_ms_, 200, 60000, 1800);
  this->standing_timeout_ms_ = sanitize_persisted<uint32_t>(this->standing_timeout_ms_, 200, 60000, 2200);
  this->min_event_sensors_ = sanitize_persisted<uint8_t>(this->min_event_sensors_, 2, SENSOR_COUNT, 2);
  this->min_active_duration_ms_ = sanitize_persisted<uint32_t>(this->min_active_duration_ms_, 0, 1000, 35);
  this->direction_window_ms_ = sanitize_persisted<uint32_t>(this->direction_window_ms_, 10, 1000, 90);
  this->calibration_samples_ = sanitize_persisted<uint16_t>(this->calibration_samples_, 4, 128, 24);
  this->max_people_inside_ = sanitize_persisted<uint16_t>(this->max_people_inside_, 1, 5000, 50);
  this->min_valid_sensors_ = sanitize_persisted<uint8_t>(this->min_valid_sensors_, 2, SENSOR_COUNT, 3);

  const uint16_t min_timing_budget = this->distance_mode_ == DISTANCE_MODE_SHORT ? 20 : 33;
  if (this->timing_budget_ms_ < min_timing_budget) {
    this->timing_budget_ms_ = min_timing_budget;
  }
  if (this->intermeasurement_ms_ < this->timing_budget_ms_) {
    this->intermeasurement_ms_ = this->timing_budget_ms_;
  }
  if (this->clear_threshold_mm_ >= this->trigger_threshold_mm_) {
    this->clear_threshold_mm_ = std::max<uint16_t>(20, this->trigger_threshold_mm_ / 2);
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
    state.calibrations[index] = this->build_persisted_calibration_(this->channels_[index]);
  }

  this->persisted_state_pref_.save(&state);
  if (global_preferences != nullptr) {
    global_preferences->sync();
  }
  this->state_dirty_ = false;
}

void TofOverdoorCounter::rediscover() {
  ESP_LOGI(TAG, "Starting four-sensor discovery");
  this->channels_.clear();
  this->channels_.resize(this->xshut_pins_.size());
  this->clear_event_tracking_();
  this->update_passage_state_(PASSAGE_IDLE);
  this->set_all_xshut_(false);
  delay(this->wake_delay_ms_);

  uint8_t next_address = this->base_address_;
  for (size_t index = 0; index < this->xshut_pins_.size(); index++) {
    auto &channel = this->channels_[index];
    channel.pin_number = this->xshut_pin_numbers_[index];
    channel.address = next_address;
    channel.group = group_for_pin(channel.pin_number);
    channel.sensor_label = sensor_name_for_pin(channel.pin_number);
    channel.source_label = channel.sensor_label + std::string(" / ") + group_name(channel.group) + " / GPIO" +
                           std::to_string(channel.pin_number);

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

  this->start_all_ranging_();

  this->last_discovery_ms_ = millis();
  this->load_persisted_state_();
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
    channel.samples.clear();
    channel.has_sampled_distance = false;
    channel.sampled_distance = 0;
    channel.calibration_sum = 0.0f;
    channel.calibration_sq_sum = 0.0f;
    channel.calibration_min = NAN;
    channel.calibration_max = NAN;
    channel.calibration_samples = 0;
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

void TofOverdoorCounter::process_calibration_() {
  const uint32_t now = millis();
  uint8_t healthy_sensors = 0;
  uint8_t clear_sensors = 0;

  for (auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }

    const float sample = this->channel_logic_distance_(channel);

    if (!channel.has_reading || channel.stale || channel.consecutive_errors >= 3 || std::isnan(sample)) {
      continue;
    }

    healthy_sensors++;
    if (sample >= this->minimum_clear_distance_mm_) {
      clear_sensors++;
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
    const float sample = this->channel_logic_distance_(channel);
    if (!channel.has_reading || channel.stale || channel.consecutive_errors >= 3 ||
        std::isnan(sample) || sample < this->minimum_clear_distance_mm_) {
      continue;
    }
    if (channel.calibration_samples >= this->calibration_samples_) {
      continue;
    }
    channel.calibration_sum += sample;
    channel.calibration_sq_sum += sample * sample;
    channel.calibration_min = std::isnan(channel.calibration_min) ? sample : std::min(channel.calibration_min, sample);
    channel.calibration_max = std::isnan(channel.calibration_max) ? sample : std::max(channel.calibration_max, sample);
    channel.calibration_samples++;
  }

  std::vector<uint16_t> sample_counts;
  sample_counts.reserve(this->channels_.size());
  for (const auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    sample_counts.push_back(channel.calibration_samples);
  }
  std::sort(sample_counts.begin(), sample_counts.end(), std::greater<uint16_t>());
  const size_t required_rank = std::min<size_t>(std::max<uint8_t>(1, this->min_valid_sensors_), sample_counts.size());
  const uint16_t samples = sample_counts.empty() ? 0 : sample_counts[required_rank - 1];
  this->phase_text_ =
      "Collecting calibration samples (" + std::to_string(samples) + "/" + std::to_string(this->calibration_samples_) + ")";

  if (samples < this->calibration_samples_) {
    return;
  }

  uint8_t stable_channels = 0;
  std::vector<float> baselines(this->channels_.size(), NAN);
  std::vector<float> noises(this->channels_.size(), NAN);
  std::vector<uint8_t> qualities(this->channels_.size(), 0);
  for (size_t index = 0; index < this->channels_.size(); index++) {
    auto &channel = this->channels_[index];
    if (!channel.initialized || channel.calibration_samples < this->calibration_samples_) {
      continue;
    }

    const float sample_count = static_cast<float>(channel.calibration_samples);
    const float mean = channel.calibration_sum / sample_count;
    const float variance = std::max(0.0f, (channel.calibration_sq_sum / sample_count) - (mean * mean));
    const float stddev = sqrtf(variance);
    const float span = channel.calibration_max - channel.calibration_min;

    if (span > static_cast<float>(this->baseline_tolerance_mm_ * 10U) ||
        stddev > static_cast<float>(this->baseline_tolerance_mm_) * 3.0f) {
      ESP_LOGW(TAG, "Calibration unstable on %s: mean=%.1f stddev=%.1f span=%.1f", channel.sensor_label.c_str(), mean,
               stddev, span);
      channel.calibration_sum = 0.0f;
      channel.calibration_sq_sum = 0.0f;
      channel.calibration_min = NAN;
      channel.calibration_max = NAN;
      channel.calibration_samples = 0;
      continue;
    }

    baselines[index] = mean;
    noises[index] = std::max(1.0f, stddev);
    qualities[index] = clamp_quality(100.0f - (stddev * 2.5f) - (span * 0.3f));
    stable_channels++;
  }

  if (stable_channels < this->min_valid_sensors_) {
    this->phase_text_ = "Calibration still collecting stable samples";
    return;
  }

  for (size_t index = 0; index < this->channels_.size(); index++) {
    auto &channel = this->channels_[index];
    if (!channel.initialized) {
      continue;
    }
    channel.baseline = baselines[index];
    channel.noise = noises[index];
    channel.calibration_quality = qualities[index];
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
    channel.rising_edge = false;
    channel.falling_edge = false;
    const float logic_distance = this->channel_logic_distance_(channel);

    if (!channel.initialized || !channel.has_reading || !channel.calibrated || std::isnan(logic_distance) ||
        std::isnan(channel.baseline) || channel.stale || channel.consecutive_invalid >= 3) {
      channel.active = false;
      channel.blocked = false;
      channel.active_candidate_since_ms = 0;
      channel.clear_candidate_since_ms = 0;
      channel.active_since_ms = 0;
      continue;
    }

    const float drop = channel.baseline - logic_distance;

    if (!channel.active) {
      if (drop >= static_cast<float>(this->trigger_threshold_mm_)) {
        if (channel.active_candidate_since_ms == 0) {
          channel.active_candidate_since_ms = now;
        } else if ((now - channel.active_candidate_since_ms) >= this->debounce_ms_) {
          channel.active = true;
          channel.active_since_ms = channel.active_candidate_since_ms;
          channel.last_rising_ms = channel.active_since_ms;
          channel.active_duration_ms = 0;
          channel.rising_edge = true;
          channel.clear_candidate_since_ms = 0;
        }
      } else {
        channel.active_candidate_since_ms = 0;
      }
    } else {
      if (drop <= static_cast<float>(this->clear_threshold_mm_)) {
        if (channel.clear_candidate_since_ms == 0) {
          channel.clear_candidate_since_ms = now;
        } else if ((now - channel.clear_candidate_since_ms) >= this->debounce_ms_) {
          channel.active = false;
          channel.blocked = false;
          channel.last_falling_ms = now;
          channel.active_duration_ms = channel.active_since_ms == 0 ? 0 : now - channel.active_since_ms;
          channel.falling_edge = true;
          channel.active_candidate_since_ms = 0;
          channel.clear_candidate_since_ms = 0;
          channel.active_since_ms = 0;
        }
      } else {
        channel.clear_candidate_since_ms = 0;
      }
    }

    if (channel.active && channel.active_since_ms != 0 && (now - channel.active_since_ms) >= this->blocked_timeout_ms_) {
      channel.blocked = true;
    }
    if (channel.active && channel.active_since_ms != 0) {
      channel.active_duration_ms = now - channel.active_since_ms;
    }
  }
}

void TofOverdoorCounter::apply_idle_baseline_tracking_() {
  if (this->event_active_ || this->calibration_active_) {
    return;
  }

  for (auto &channel : this->channels_) {
    const float logic_distance = this->channel_logic_distance_(channel);
    if (!channel.initialized || !channel.has_reading || !channel.calibrated || channel.active || channel.blocked ||
        std::isnan(logic_distance) || std::isnan(channel.baseline)) {
      continue;
    }

    const float delta = fabsf(logic_distance - channel.baseline);
    if (delta > static_cast<float>(this->baseline_tolerance_mm_)) {
      continue;
    }

    channel.baseline = (channel.baseline * (1.0f - BASELINE_TRACK_ALPHA)) + (logic_distance * BASELINE_TRACK_ALPHA);
    const float deviation = fabsf(logic_distance - channel.baseline);
    channel.noise = std::isnan(channel.noise) ? deviation
                                              : (channel.noise * (1.0f - NOISE_TRACK_ALPHA)) +
                                                    (deviation * NOISE_TRACK_ALPHA);
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
  this->event_path_size_ = 0;
  this->event_last_state_code_ = GROUP_STATE_NONE;
  std::fill(std::begin(this->event_path_), std::end(this->event_path_), GROUP_STATE_NONE);
  std::fill(std::begin(this->event_edges_), std::end(this->event_edges_), EventEdge{});
  for (auto &channel : this->channels_) {
    channel.first_trigger_in_event_ms = 0;
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

void TofOverdoorCounter::update_detection_state_machine_() {
  const uint32_t now = millis();

  if (!this->ready_for_counting_()) {
    this->phase_text_ = "Waiting for stable calibration";
    this->clear_event_tracking_();
    this->person_standing_in_door_ = false;
    this->update_passage_state_(PASSAGE_IDLE);
    return;
  }

  if (this->cooldown_until_ms_ != 0 && now < this->cooldown_until_ms_) {
    this->phase_text_ = "Cooldown after last detection";
    return;
  }
  if (this->cooldown_until_ms_ != 0 && now >= this->cooldown_until_ms_) {
    this->cooldown_until_ms_ = 0;
  }

  const uint8_t active_count = this->active_sensor_count_();
  const uint8_t active_out = this->active_sensor_count_for_group_(GROUP_OUT);
  const uint8_t active_in = this->active_sensor_count_for_group_(GROUP_IN);
  const uint8_t current_state_code = this->current_group_state_code_(active_out, active_in);
  uint8_t active_mask = 0;
  for (size_t index = 0; index < this->channels_.size(); index++) {
    if (this->channels_[index].initialized && this->channels_[index].active) {
      active_mask |= (1U << index);
    }
  }

  // A pass is evaluated as a whole event: rising/falling edges, group order, and the compressed
  // group path are kept until the doorway clears, then finalize_event_ decides whether to count.
  if (!this->event_active_) {
    if (current_state_code == GROUP_STATE_NONE) {
      this->person_standing_in_door_ = false;
      this->standing_clear_since_ms_ = 0;
      this->phase_text_ = "Ready";
      this->update_passage_state_(PASSAGE_IDLE);
      return;
    }

    this->event_active_ = true;
    uint32_t first_active_ts = now;
    for (const auto &channel : this->channels_) {
      if (!channel.initialized || !channel.active || channel.active_since_ms == 0) {
        continue;
      }
      first_active_ts = std::min(first_active_ts, channel.active_since_ms);
    }
    this->event_started_ms_ = first_active_ts;
    this->event_last_activity_ms_ = now;
    this->event_first_edge_ms_ = first_active_ts;
    this->event_last_edge_ms_ = now;
    this->event_first_group_ = this->determine_first_group_from_current_state_();
    this->event_second_group_ = GROUP_NONE;
    this->event_direction_group_ = GROUP_NONE;
    this->event_sensor_mask_ = 0;
    this->event_rising_mask_ = 0;
    this->event_falling_mask_ = 0;
    this->event_peak_active_count_ = 0;
    this->event_peak_group_counts_[0] = 0;
    this->event_peak_group_counts_[1] = 0;
    this->person_standing_in_door_ = false;
    this->update_passage_state_(PASSAGE_POSSIBLE);
    this->append_event_path_state_(current_state_code, now);
  } else if (current_state_code != this->event_last_state_code_) {
    this->append_event_path_state_(current_state_code, now);
  }

  for (size_t index = 0; index < this->channels_.size(); index++) {
    auto &channel = this->channels_[index];
    if (!channel.initialized || !channel.active) {
      continue;
    }
    if ((this->event_sensor_mask_ & (1U << index)) == 0) {
      this->event_sensor_mask_ |= (1U << index);
      channel.first_trigger_in_event_ms = channel.active_since_ms != 0 ? channel.active_since_ms : now;
    }
    if (channel.rising_edge) {
      this->event_rising_mask_ |= (1U << index);
      this->record_event_edge_(index, channel, true, channel.last_rising_ms != 0 ? channel.last_rising_ms : now,
                               active_mask);
    }
  }
  for (size_t index = 0; index < this->channels_.size(); index++) {
    const auto &channel = this->channels_[index];
    if (!channel.initialized || !channel.falling_edge) {
      continue;
    }
    this->event_falling_mask_ |= (1U << index);
    this->record_event_edge_(index, channel, false, channel.last_falling_ms != 0 ? channel.last_falling_ms : now,
                             active_mask);
  }

  this->event_peak_active_count_ = std::max(this->event_peak_active_count_, active_count);
  this->event_peak_group_counts_[GROUP_OUT] = std::max(this->event_peak_group_counts_[GROUP_OUT], active_out);
  this->event_peak_group_counts_[GROUP_IN] = std::max(this->event_peak_group_counts_[GROUP_IN], active_in);
  if (active_out >= 2 && this->event_group_confirmed_ms_[GROUP_OUT] == 0) {
    this->event_group_confirmed_ms_[GROUP_OUT] = now;
  }
  if (active_in >= 2 && this->event_group_confirmed_ms_[GROUP_IN] == 0) {
    this->event_group_confirmed_ms_[GROUP_IN] = now;
  }

  if (this->event_first_group_ == GROUP_NONE) {
    this->event_first_group_ = this->determine_first_group_from_current_state_();
  }
  const SensorGroup resolved_first_group = this->resolve_event_first_group_();
  if (resolved_first_group != GROUP_NONE) {
    this->event_first_group_ = resolved_first_group;
  }
  if (this->event_first_group_ != GROUP_NONE) {
    const SensorGroup other_group = this->event_first_group_ == GROUP_OUT ? GROUP_IN : GROUP_OUT;
    if (this->triggered_sensor_count_for_group_(other_group) > 0) {
      this->event_second_group_ = other_group;
    }
  }
  if (active_count >= 2 || current_state_code == GROUP_STATE_BOTH) {
    this->update_passage_state_(PASSAGE_OCCUPIED);
  }
  if (this->event_first_group_ != GROUP_NONE && this->event_second_group_ != GROUP_NONE) {
    this->update_passage_state_(PASSAGE_SEQUENCE);
  }
  if (this->event_first_group_ != GROUP_NONE && this->event_second_group_ != GROUP_NONE &&
      this->event_direction_group_ == GROUP_NONE) {
    this->event_direction_group_ = this->map_physical_group_to_direction_(this->event_first_group_);
    this->event_direction_decided_ms_ = now;
    this->update_passage_state_(PASSAGE_DIRECTION_DECIDED);
  }

  if (current_state_code == GROUP_STATE_NONE) {
    this->finalize_event_(false);
    return;
  }

  const bool stalled = this->event_last_activity_ms_ != 0 && (now - this->event_last_activity_ms_) >= this->detection_timeout_ms_;
  const bool standing = (now - this->event_started_ms_) >= this->standing_timeout_ms_;
  if (stalled || standing) {
    this->person_standing_in_door_ = true;
    this->standing_clear_since_ms_ = 0;
    this->update_passage_state_(stalled ? PASSAGE_TIMEOUT : PASSAGE_OCCUPIED);
  }

  if (this->person_standing_in_door_) {
    this->phase_text_ = "Person standing in doorway";
    return;
  }

  switch (current_state_code) {
    case GROUP_STATE_OUT_ONLY:
      this->phase_text_ = "OUT side active";
      break;
    case GROUP_STATE_IN_ONLY:
      this->phase_text_ = "IN side active";
      break;
    case GROUP_STATE_BOTH:
      this->phase_text_ = "Both sides active";
      break;
    case GROUP_STATE_NONE:
    default:
      this->phase_text_ = "Watching trigger order";
      break;
  }
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
  bool saw_out_only = false;
  bool saw_in_only = false;
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
      saw_out_only = true;
      if (saw_both_state && resolved_first_group == GROUP_OUT) {
        saw_same_side_after_both = true;
      }
      if (saw_both_state && resolved_first_group == GROUP_IN) {
        saw_opposite_side_after_both = true;
      }
    } else if (state == GROUP_STATE_IN_ONLY) {
      saw_in_only = true;
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
  const bool ordered_two_group_sequence = resolved_first_group != GROUP_NONE && this->event_second_group_ != GROUP_NONE &&
                                          this->event_second_group_ != resolved_first_group;
  const bool valid_crossing = both_groups_seen && ordered_two_group_sequence && distinct_triggered >= required &&
                              long_enough && (path_crossed_doorway || saw_both_state || fast_cross_path);
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
             " side first, path " + this->event_path_text_() + ", edges " + this->event_edge_text_();
  } else if (resolved_first_group != GROUP_NONE && both_groups_seen && distinct_triggered >= 2 && long_enough) {
    const SensorGroup direction_group = this->map_physical_group_to_direction_(resolved_first_group);
    outcome = direction_group == GROUP_IN ? OUTCOME_UNSURE_IN : OUTCOME_UNSURE_OUT;
    reason = "Unsure detection: " + std::to_string(distinct_triggered) + "/" + std::to_string(healthy) +
             " sensors triggered, " + std::string(group_name(resolved_first_group)) +
             " side first, path " + this->event_path_text_() + ", edges " + this->event_edge_text_();
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
        << "}";
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
  if (this->channels_.empty()) {
    this->system_status_ = STATUS_ERROR;
    return;
  }

  if (this->calibration_active_) {
    this->system_status_ = STATUS_CALIBRATING;
    return;
  }

  const uint8_t healthy = this->healthy_sensor_count_();
  const uint8_t reporting = this->reporting_sensor_count_();

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

  if (healthy < this->get_discovered_sensor_count()) {
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
    if (channel.initialized && channel.calibrated) {
      calibrated++;
    }
  }
  return calibrated >= this->min_valid_sensors_ && this->reporting_sensor_count_() >= this->min_valid_sensors_;
}

bool TofOverdoorCounter::has_restored_calibration_() const {
  uint8_t calibrated = 0;
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.calibrated && !std::isnan(channel.baseline)) {
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
  for (const auto &channel : this->channels_) {
    if (channel.initialized && channel.group == group && channel.active) {
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

float TofOverdoorCounter::group_distance_internal_(SensorGroup group) const {
  float nearest = NAN;
  for (const auto &channel : this->channels_) {
    const float logic_distance = this->channel_logic_distance_(channel);
    if (!channel.initialized || !channel.has_reading || channel.group != group || std::isnan(logic_distance)) {
      continue;
    }
    nearest = std::isnan(nearest) ? logic_distance : std::min(nearest, logic_distance);
  }
  return nearest;
}

float TofOverdoorCounter::group_baseline_internal_(SensorGroup group) const {
  float total = 0.0f;
  uint8_t count = 0;
  for (const auto &channel : this->channels_) {
    if (!channel.initialized || !channel.calibrated || channel.group != group || std::isnan(channel.baseline)) {
      continue;
    }
    total += channel.baseline;
    count++;
  }
  return count == 0 ? NAN : total / static_cast<float>(count);
}

float TofOverdoorCounter::group_drop_internal_(SensorGroup group) const {
  float drop = NAN;
  for (const auto &channel : this->channels_) {
    const float logic_distance = this->channel_logic_distance_(channel);
    if (!channel.initialized || !channel.calibrated || channel.group != group || std::isnan(channel.baseline) ||
        std::isnan(logic_distance)) {
      continue;
    }
    const float sensor_drop = channel.baseline - logic_distance;
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
  const float logic_distance = this->channel_logic_distance_(channel);
  if (!channel.initialized || std::isnan(channel.baseline) || std::isnan(logic_distance)) {
    return NAN;
  }
  return channel.baseline - logic_distance;
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
  std::vector<uint16_t> sample_counts;
  sample_counts.reserve(this->channels_.size());
  for (const auto &channel : this->channels_) {
    if (!channel.initialized) {
      continue;
    }
    sample_counts.push_back(channel.calibration_samples);
  }
  if (sample_counts.empty() || this->calibration_samples_ == 0) {
    return 0.0f;
  }
  std::sort(sample_counts.begin(), sample_counts.end(), std::greater<uint16_t>());
  const size_t required_rank = std::min<size_t>(std::max<uint8_t>(1, this->min_valid_sensors_), sample_counts.size());
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
  return group_index == 0 ? "OUT group (U3/U4)" : "IN group (U7/U8)";
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
    oss << channel.sensor_label << "=" << group_name(channel.group);
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
