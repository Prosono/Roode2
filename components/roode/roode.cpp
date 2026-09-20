#include "roode.h"

namespace esphome {
namespace roode {
namespace {
constexpr float PEOPLE_COUNTER_MIN = 0.0f;
constexpr float PEOPLE_COUNTER_MAX = 50.0f;
constexpr uint32_t MILLIS_PER_MINUTE = 60000UL;
constexpr uint8_t PERSISTED_TUNING_VERSION = 1;

uint8_t current_threshold_percentage(const Threshold *threshold, bool is_max, uint8_t fallback) {
  auto configured = is_max ? threshold->max_percentage : threshold->min_percentage;
  if (configured.has_value()) {
    return configured.value();
  }

  uint16_t idle = threshold->idle;
  uint16_t value = is_max ? threshold->max : threshold->min;
  if (idle == 0) {
    return fallback;
  }
  return (value * 100U) / idle;
}

uint8_t current_roi_value(uint8_t override_value, uint8_t active_value, uint8_t fallback) {
  if (override_value != 0) {
    return override_value;
  }
  if (active_value != 0) {
    return active_value;
  }
  return fallback;
}

float clamp_people_counter(float value) {
  if (value < PEOPLE_COUNTER_MIN) {
    return PEOPLE_COUNTER_MIN;
  }
  if (value > PEOPLE_COUNTER_MAX) {
    return PEOPLE_COUNTER_MAX;
  }
  return value;
}
}  // namespace

void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  ESP_LOGCONFIG(TAG, "  Auto recalibration: %u min", this->get_auto_recalibration_interval_minutes());
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

void Roode::setup() {
  ESP_LOGI(SETUP, "Booting Roode %s", VERSION);
  if (version_sensor != nullptr) {
    version_sensor->publish_state(VERSION);
  }
  this->publish_presence_state_(false);
  this->publish_masking_state_(false);
  ESP_LOGI(SETUP, "Using sampling with sampling size: %d", samples);

  if (this->distanceSensor->is_failed()) {
    this->mark_failed();
    ESP_LOGE(TAG, "Roode cannot be setup without a valid VL53L1X sensor");
    return;
  }

  this->init_tuning_preferences_();
  this->load_persisted_tuning_();
  this->reset_runtime_detection_state_();

  if (this->status_sensor != nullptr) {
    this->status_sensor->publish_state(VL53L1_ERROR_NONE);
  }

  calibrate_zones();
}

void Roode::update() {
  if (distance_entry != nullptr) {
    distance_entry->publish_state(entry->getDistance());
  }
  if (distance_exit != nullptr) {
    distance_exit->publish_state(exit->getDistance());
  }
}

void Roode::loop() {
  if (!this->calibrated_) {
    if (!this->distanceSensor->is_failed() && millis() - this->calibration_retry_ms_ >= 10000U) this->calibrate_zones();
    return;
  }
  this->sensor_status = this->current_zone->readDistance(distanceSensor);
  if (this->sensor_status == VL53L1_ERROR_NONE) {
    update_masking_state_();
    path_tracking(this->current_zone);
  } else {
    this->fusion_.reset();
    this->publish_presence_state_(false);
  }
  handle_sensor_status();
  handle_auto_recalibration_();
  this->current_zone = this->current_zone == this->entry ? this->exit : this->entry;
}

bool Roode::handle_sensor_status() {
  bool check_status = false;
  if (last_sensor_status != sensor_status && sensor_status == VL53L1_ERROR_NONE) {
    if (status_sensor != nullptr) {
      status_sensor->publish_state(sensor_status);
    }
    check_status = true;
  }
  if (sensor_status < 28 && sensor_status != VL53L1_ERROR_NONE) {
    ESP_LOGE(TAG, "Ranging failed with an error. status: %d", sensor_status);
    if (status_sensor != nullptr) status_sensor->publish_state(sensor_status);
    check_status = false;
  }

  last_sensor_status = sensor_status;
  sensor_status = VL53L1_ERROR_NONE;
  return check_status;
}

void Roode::path_tracking(Zone *zone) {
  std::array<uint8_t, 4> states{}, fresh{};
  const bool entry_active = entry->hasDistance() && entry->getMinDistance() < entry->threshold->max && entry->getMinDistance() > entry->threshold->min;
  const bool exit_active = exit->hasDistance() && exit->getMinDistance() < exit->threshold->max && exit->getMinDistance() > exit->threshold->min;
  states[0] = (entry_active ? 1 : 0) | (exit_active ? 2 : 0);
  fresh[0] = zone == entry ? 1 : 2;
  this->publish_presence_state_(states[0] != 0);
  this->fusion_.required = 1;
  this->fusion_.clear_ms = 90;
  this->fusion_.minimum_ms = 25;
  this->fusion_.invert = this->invert_direction_;
  const auto result = this->fusion_.update(millis(), entry->hasDistance() && exit->hasDistance() ? 1 : 0, states, fresh);
  if (result.decision == counting_core::Decision::IN || result.decision == counting_core::Decision::OUT) {
    const bool entering = result.decision == counting_core::Decision::IN;
    this->updateCounter(entering ? 1 : -1);
    this->last_direction_ = entering ? "Entry" : "Exit";
    if (entry_exit_event_sensor != nullptr) entry_exit_event_sensor->publish_state(this->last_direction_);
  }
}

uint16_t Roode::get_auto_recalibration_interval_minutes() const {
  return this->auto_recalibration_interval_ms_ == 0 ? 0 : this->auto_recalibration_interval_ms_ / MILLIS_PER_MINUTE;
}

uint32_t Roode::tuning_preference_key_() const {
  uint32_t address = this->distanceSensor != nullptr ? this->distanceSensor->get_address() : 0x29;
  return 0x524F4F44UL ^ address;
}

void Roode::init_tuning_preferences_() {
  if (this->tuning_pref_ready_ || global_preferences == nullptr) {
    return;
  }
  this->tuning_pref_ = global_preferences->make_preference<PersistedTuningState>(this->tuning_preference_key_(), true);
  this->tuning_pref_ready_ = true;
}

void Roode::load_persisted_tuning_() {
  if (!this->tuning_pref_ready_) {
    return;
  }

  PersistedTuningState state{};
  if (!this->tuning_pref_.load(&state) || state.version != PERSISTED_TUNING_VERSION) {
    return;
  }

  ESP_LOGI(TAG,
           "Loaded saved tuning auto=%u sampling=%u saved_invert=%s min=%u max=%u roi=%ux%u",
           static_cast<unsigned int>(state.auto_recalibration_minutes), static_cast<unsigned int>(state.sampling),
           state.invert_direction ? "true" : "false", static_cast<unsigned int>(state.min_threshold_percentage),
           static_cast<unsigned int>(state.max_threshold_percentage), static_cast<unsigned int>(state.roi_width),
           static_cast<unsigned int>(state.roi_height));

  this->set_auto_recalibration_interval_minutes(state.auto_recalibration_minutes);
  this->set_sampling_size(state.sampling);
  this->set_min_threshold_percentage(state.min_threshold_percentage);
  this->set_max_threshold_percentage(state.max_threshold_percentage);
  this->set_roi_width(state.roi_width);
  this->set_roi_height(state.roi_height);
}

void Roode::persist_runtime_settings() {
  if (!this->tuning_pref_ready_) {
    return;
  }

  PersistedTuningState state{};
  state.version = PERSISTED_TUNING_VERSION;
  state.auto_recalibration_minutes = this->get_auto_recalibration_interval_minutes();
  state.sampling = this->get_sampling_size();
  state.invert_direction = this->get_invert_direction();
  state.min_threshold_percentage = this->get_min_threshold_percentage();
  state.max_threshold_percentage = this->get_max_threshold_percentage();
  state.roi_width = this->get_roi_width();
  state.roi_height = this->get_roi_height();

  ESP_LOGI(TAG,
           "Saving tuning auto=%u sampling=%u invert=%s min=%u max=%u roi=%ux%u",
           static_cast<unsigned int>(state.auto_recalibration_minutes), static_cast<unsigned int>(state.sampling),
           state.invert_direction ? "true" : "false", static_cast<unsigned int>(state.min_threshold_percentage),
           static_cast<unsigned int>(state.max_threshold_percentage), static_cast<unsigned int>(state.roi_width),
           static_cast<unsigned int>(state.roi_height));

  this->tuning_pref_.save(&state);
  if (global_preferences != nullptr) {
    global_preferences->sync();
  }
}

uint32_t Roode::get_minutes_since_last_recalibration() const {
  if (this->last_recalibration_ms_ == 0) {
    return 0;
  }
  return (millis() - this->last_recalibration_ms_) / MILLIS_PER_MINUTE;
}

uint32_t Roode::get_minutes_until_next_recalibration() const {
  if (this->auto_recalibration_interval_ms_ == 0 || this->last_recalibration_ms_ == 0) {
    return 0;
  }

  uint32_t elapsed = millis() - this->last_recalibration_ms_;
  if (elapsed >= this->auto_recalibration_interval_ms_) {
    return 0;
  }

  return (this->auto_recalibration_interval_ms_ - elapsed) / MILLIS_PER_MINUTE;
}

bool Roode::is_ready_for_recalibration() const {
  if (this->distanceSensor == nullptr || this->distanceSensor->is_failed()) {
    return false;
  }
  if (this->masking_detected_) {
    return false;
  }
  return this->left_previous_status_ == NOBODY && this->right_previous_status_ == NOBODY;
}

int Roode::get_sensor_status_code() const {
  if (this->distanceSensor != nullptr && this->distanceSensor->is_failed() && this->last_sensor_status == VL53L1_ERROR_NONE) {
    return -1;
  }
  return this->last_sensor_status;
}

uint16_t Roode::get_entry_distance() const {
  return this->entry->hasDistance() ? this->entry->getDistance() : 0;
}

uint16_t Roode::get_exit_distance() const {
  return this->exit->hasDistance() ? this->exit->getDistance() : 0;
}

float Roode::get_people_counter_value() const {
  if (this->people_counter == nullptr || isnan(this->people_counter->state)) {
    return 0.0f;
  }
  return this->people_counter->state;
}

uint8_t Roode::get_min_threshold_percentage() const {
  return current_threshold_percentage(this->entry->threshold, false, 0);
}

void Roode::set_min_threshold_percentage(uint8_t percentage) {
  this->entry->threshold->set_min_percentage(percentage);
  this->exit->threshold->set_min_percentage(percentage);
}

uint8_t Roode::get_max_threshold_percentage() const {
  return current_threshold_percentage(this->entry->threshold, true, 85);
}

void Roode::set_max_threshold_percentage(uint8_t percentage) {
  this->entry->threshold->set_max_percentage(percentage);
  this->exit->threshold->set_max_percentage(percentage);
}

uint8_t Roode::get_roi_width() const {
  return current_roi_value(this->entry->roi_override->width, this->entry->roi->width, 6);
}

void Roode::set_roi_width(uint8_t width) {
  this->entry->roi_override->width = width;
  this->exit->roi_override->width = width;
}

uint8_t Roode::get_roi_height() const {
  return current_roi_value(this->entry->roi_override->height, this->entry->roi->height, 16);
}

void Roode::set_roi_height(uint8_t height) {
  this->entry->roi_override->height = height;
  this->exit->roi_override->height = height;
}

void Roode::updateCounter(int delta) {
  if (this->people_counter == nullptr) {
    return;
  }
  float current = isnan(this->people_counter->state) ? 0.0f : this->people_counter->state;
  float next = clamp_people_counter(current + (float) delta);
  ESP_LOGI(TAG, "Updating people count: %d", (int) next);
  auto call = this->people_counter->make_call();
  call.set_value(next);
  call.perform();
}

void Roode::set_people_counter_value(float value) {
  if (this->people_counter == nullptr) {
    return;
  }
  auto call = this->people_counter->make_call();
  call.set_value(clamp_people_counter(value));
  call.perform();
}

void Roode::publish_presence_state_(bool state) {
  if (this->presence_detected_ == state) {
    return;
  }
  this->presence_detected_ = state;
  if (this->presence_sensor != nullptr) {
    this->presence_sensor->publish_state(state);
  }
}

void Roode::publish_masking_state_(bool state) {
  if (this->masking_sensor == nullptr) {
    this->masking_detected_ = state;
    return;
  }
  if (this->masking_detected_ == state) {
    return;
  }
  this->masking_detected_ = state;
  this->masking_sensor->publish_state(state);
}

void Roode::update_masking_state_() {
  if (this->masking_sensor == nullptr) {
    return;
  }

  if (this->distanceSensor->is_failed()) {
    this->masking_candidate_since_ms_ = 0;
    this->publish_masking_state_(false);
    return;
  }

  uint16_t nearest_distance = UINT16_MAX;
  if (this->entry->hasDistance()) {
    nearest_distance = this->entry->getMinDistance();
  }
  if (this->exit->hasDistance() && this->exit->getMinDistance() < nearest_distance) {
    nearest_distance = this->exit->getMinDistance();
  }

  if (nearest_distance == UINT16_MAX || nearest_distance > this->masking_distance_threshold_mm_) {
    this->masking_candidate_since_ms_ = 0;
    this->publish_masking_state_(false);
    return;
  }

  if (this->masking_candidate_since_ms_ == 0) {
    this->masking_candidate_since_ms_ = millis();
  }

  if ((millis() - this->masking_candidate_since_ms_) >= this->masking_hold_time_ms_) {
    this->publish_masking_state_(true);
  }
}

void Roode::recalibration() {
  if (this->distanceSensor->is_failed()) {
    ESP_LOGE(TAG, "Cannot recalibrate while VL53L1X sensor is failed");
    return;
  }
  this->reset_runtime_detection_state_();
  calibrate_zones();
}

void Roode::reset_runtime_state() {
  this->reset_runtime_detection_state_();
}

void Roode::reset_runtime_detection_state_() {
  this->fusion_.reset();
  this->current_zone = this->entry;
  this->last_sensor_status = VL53L1_ERROR_NONE;
  this->sensor_status = VL53L1_ERROR_NONE;
  this->last_direction_ = "Waiting";
  this->masking_candidate_since_ms_ = 0;
  this->publish_presence_state_(false);
  this->publish_masking_state_(false);
  this->path_track_[0] = 0;
  this->path_track_[1] = 0;
  this->path_track_[2] = 0;
  this->path_track_[3] = 0;
  this->path_track_filling_size_ = 1;
  this->left_previous_status_ = NOBODY;
  this->right_previous_status_ = NOBODY;
}

void Roode::handle_auto_recalibration_() {
  if (this->auto_recalibration_interval_ms_ == 0 || this->distanceSensor == nullptr ||
      this->distanceSensor->is_failed()) {
    return;
  }

  if (this->last_recalibration_ms_ == 0) {
    this->last_recalibration_ms_ = millis();
    return;
  }

  if ((millis() - this->last_recalibration_ms_) < this->auto_recalibration_interval_ms_) {
    return;
  }

  if (!this->is_ready_for_recalibration()) {
    return;
  }

  ESP_LOGI(TAG, "Running automatic recalibration");
  this->recalibration();
}

const RangingMode *Roode::determine_raning_mode(uint16_t average_entry_zone_distance,
                                                uint16_t average_exit_zone_distance) {
  uint16_t min = average_entry_zone_distance < average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  uint16_t max = average_entry_zone_distance > average_exit_zone_distance ? average_entry_zone_distance
                                                                          : average_exit_zone_distance;
  if (min <= short_distance_threshold) {
    return Ranging::Short;
  }
  if (max > short_distance_threshold && min <= medium_distance_threshold) {
    return Ranging::Medium;
  }
  if (max > medium_distance_threshold && min <= medium_long_distance_threshold) {
    return Ranging::Long;
  }
  if (max > medium_long_distance_threshold && min <= long_distance_threshold) {
    return Ranging::Longer;
  }
  return Ranging::Longest;
}

void Roode::calibrate_zones() {
  this->calibrated_ = false;
  this->calibration_retry_ms_ = millis();
  this->fusion_.reset();
  if (this->distanceSensor->is_failed()) {
    ESP_LOGE(TAG, "Skipping calibration because VL53L1X sensor is failed");
    return;
  }

  ESP_LOGI(SETUP, "Calibrating sensor zones");

  entry->reset_roi(orientation_ == Parallel ? 167 : 195);
  exit->reset_roi(orientation_ == Parallel ? 231 : 60);

  if (!calibrateDistance()) return;

  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  if (!entry->calibrateThreshold(distanceSensor, number_attempts)) return;
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  if (!exit->calibrateThreshold(distanceSensor, number_attempts)) return;

  publish_sensor_configuration(entry, exit, true);
  App.feed_wdt();
  publish_sensor_configuration(entry, exit, false);
  this->last_recalibration_ms_ = millis();
  this->calibrated_ = true;
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
}

bool Roode::calibrateDistance() {
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  if (!entry->calibrateThreshold(distanceSensor, number_attempts) ||
      !exit->calibrateThreshold(distanceSensor, number_attempts)) return false;

  if (distanceSensor->get_ranging_mode_override().has_value()) {
    return true;
  }
  auto *mode = determine_raning_mode(entry->threshold->idle, exit->threshold->idle);
  if (mode != initial) {
    distanceSensor->set_ranging_mode(mode);
  }
  return true;
}

void Roode::publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax) {
  if (isMax) {
    if (max_threshold_entry_sensor != nullptr) {
      max_threshold_entry_sensor->publish_state(entry->threshold->max);
    }

    if (max_threshold_exit_sensor != nullptr) {
      max_threshold_exit_sensor->publish_state(exit->threshold->max);
    }
  } else {
    if (min_threshold_entry_sensor != nullptr) {
      min_threshold_entry_sensor->publish_state(entry->threshold->min);
    }
    if (min_threshold_exit_sensor != nullptr) {
      min_threshold_exit_sensor->publish_state(exit->threshold->min);
    }
  }

  if (entry_roi_height_sensor != nullptr) {
    entry_roi_height_sensor->publish_state(entry->roi->height);
  }
  if (entry_roi_width_sensor != nullptr) {
    entry_roi_width_sensor->publish_state(entry->roi->width);
  }

  if (exit_roi_height_sensor != nullptr) {
    exit_roi_height_sensor->publish_state(exit->roi->height);
  }
  if (exit_roi_width_sensor != nullptr) {
    exit_roi_width_sensor->publish_state(exit->roi->width);
  }
}
}  // namespace roode
}  // namespace esphome
