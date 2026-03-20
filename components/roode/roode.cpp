#include "roode.h"

namespace esphome {
namespace roode {
namespace {
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
}  // namespace

void Roode::dump_config() {
  ESP_LOGCONFIG(TAG, "Roode:");
  ESP_LOGCONFIG(TAG, "  Sample size: %d", samples);
  LOG_UPDATE_INTERVAL(this);
  entry->dump_config();
  exit->dump_config();
}

void Roode::setup() {
  ESP_LOGI(SETUP, "Booting Roode %s", VERSION);
  if (version_sensor != nullptr) {
    version_sensor->publish_state(VERSION);
  }
  ESP_LOGI(SETUP, "Using sampling with sampling size: %d", samples);

  if (this->distanceSensor->is_failed()) {
    this->mark_failed();
    ESP_LOGE(TAG, "Roode cannot be setup without a valid VL53L1X sensor");
    return;
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
  // unsigned long start = micros();
  this->current_zone->readDistance(distanceSensor);
  // uint16_t samplingDistance = sampling(this->current_zone);
  path_tracking(this->current_zone);
  handle_sensor_status();
  this->current_zone = this->current_zone == this->entry ? this->exit : this->entry;
  // ESP_LOGI("Experimental", "Entry zone: %d, exit zone: %d",
  // entry->getDistance(Roode::distanceSensor, Roode::sensor_status),
  // exit->getDistance(Roode::distanceSensor, Roode::sensor_status)); unsigned
  // long end = micros(); unsigned long delta = end - start; ESP_LOGI("Roode
  // loop", "loop took %lu microseconds", delta);
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
    status_sensor->publish_state(sensor_status);
    check_status = false;
  }

  last_sensor_status = sensor_status;
  sensor_status = VL53L1_ERROR_NONE;
  return check_status;
}

void Roode::path_tracking(Zone *zone) {
  int CurrentZoneStatus = NOBODY;
  int AllZonesCurrentStatus = 0;
  int AnEventHasOccured = 0;

  // PathTrack algorithm
  if (zone->getMinDistance() < zone->threshold->max && zone->getMinDistance() > zone->threshold->min) {
    // Someone is in the sensing area
    CurrentZoneStatus = SOMEONE;
    if (presence_sensor != nullptr) {
      presence_sensor->publish_state(true);
    }
  }

  // left zone
  if (zone == (this->invert_direction_ ? this->exit : this->entry)) {
    if (CurrentZoneStatus != this->left_previous_status_) {
      // event in left zone has occured
      AnEventHasOccured = 1;

      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 1;
      }
      // need to check right zone as well ...
      if (this->right_previous_status_ == SOMEONE) {
        // event in right zone has occured
        AllZonesCurrentStatus += 2;
      }
      // remember for next time
      this->left_previous_status_ = CurrentZoneStatus;
    }
  }
  // right zone
  else {
    if (CurrentZoneStatus != this->right_previous_status_) {
      // event in right zone has occured
      AnEventHasOccured = 1;
      if (CurrentZoneStatus == SOMEONE) {
        AllZonesCurrentStatus += 2;
      }
      // need to check left zone as well ...
      if (this->left_previous_status_ == SOMEONE) {
        // event in left zone has occured
        AllZonesCurrentStatus += 1;
      }
      // remember for next time
      this->right_previous_status_ = CurrentZoneStatus;
    }
  }

  // if an event has occured
  if (AnEventHasOccured) {
    ESP_LOGD(TAG, "Event has occured, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
    if (this->path_track_filling_size_ < 4) {
      this->path_track_filling_size_++;
    }

    // if nobody anywhere lets check if an exit or entry has happened
    if ((this->left_previous_status_ == NOBODY) && (this->right_previous_status_ == NOBODY)) {
      ESP_LOGD(TAG, "Nobody anywhere, AllZonesCurrentStatus: %d", AllZonesCurrentStatus);
      // check exit or entry only if PathTrackFillingSize is 4 (for example 0 1
      // 3 2) and last event is 0 (nobobdy anywhere)
      if (this->path_track_filling_size_ == 4) {
        // check exit or entry. no need to check PathTrack[0] == 0 , it is
        // always the case

        if ((this->path_track_[1] == 1) && (this->path_track_[2] == 3) && (this->path_track_[3] == 2)) {
          // This an exit
          ESP_LOGI("Roode pathTracking", "Exit detected.");

          this->updateCounter(-1);
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Exit");
          }
        } else if ((this->path_track_[1] == 2) && (this->path_track_[2] == 3) && (this->path_track_[3] == 1)) {
          // This an entry
          ESP_LOGI("Roode pathTracking", "Entry detected.");
          this->updateCounter(1);
          if (entry_exit_event_sensor != nullptr) {
            entry_exit_event_sensor->publish_state("Entry");
          }
        }
      }

      this->path_track_filling_size_ = 1;
    } else {
      // update PathTrack
      // example of PathTrack update
      // 0
      // 0 1
      // 0 1 3
      // 0 1 3 1
      // 0 1 3 3
      // 0 1 3 2 ==> if next is 0 : check if exit
      this->path_track_[this->path_track_filling_size_ - 1] = AllZonesCurrentStatus;
    }
  }
  if (presence_sensor != nullptr) {
    if (CurrentZoneStatus == NOBODY && this->left_previous_status_ == NOBODY && this->right_previous_status_ == NOBODY) {
      // nobody is in the sensing area
      presence_sensor->publish_state(false);
    }
  }
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
  auto next = this->people_counter->state + (float) delta;
  ESP_LOGI(TAG, "Updating people count: %d", (int) next);
  auto call = this->people_counter->make_call();
  call.set_value(next);
  call.perform();
}
void Roode::recalibration() {
  if (this->distanceSensor->is_failed()) {
    ESP_LOGE(TAG, "Cannot recalibrate while VL53L1X sensor is failed");
    return;
  }
  calibrate_zones();
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
  if (this->distanceSensor->is_failed()) {
    ESP_LOGE(TAG, "Skipping calibration because VL53L1X sensor is failed");
    return;
  }

  ESP_LOGI(SETUP, "Calibrating sensor zones");

  entry->reset_roi(orientation_ == Parallel ? 167 : 195);
  exit->reset_roi(orientation_ == Parallel ? 231 : 60);

  calibrateDistance();

  entry->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  entry->calibrateThreshold(distanceSensor, number_attempts);
  exit->roi_calibration(entry->threshold->idle, exit->threshold->idle, orientation_);
  exit->calibrateThreshold(distanceSensor, number_attempts);

  publish_sensor_configuration(entry, exit, true);
  App.feed_wdt();
  publish_sensor_configuration(entry, exit, false);
  ESP_LOGI(SETUP, "Finished calibrating sensor zones");
}

void Roode::calibrateDistance() {
  auto *const initial = distanceSensor->get_ranging_mode_override().value_or(Ranging::Longest);
  distanceSensor->set_ranging_mode(initial);

  entry->calibrateThreshold(distanceSensor, number_attempts);
  exit->calibrateThreshold(distanceSensor, number_attempts);

  if (distanceSensor->get_ranging_mode_override().has_value()) {
    return;
  }
  auto *mode = determine_raning_mode(entry->threshold->idle, exit->threshold->idle);
  if (mode != initial) {
    distanceSensor->set_ranging_mode(mode);
  }
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
