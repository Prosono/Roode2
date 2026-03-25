#pragma once
#include <math.h>
#include <string>
#include <stdint.h>

#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "../vl53l1x/vl53l1x.h"
#include "orientation.h"
#include "zone.h"

using namespace esphome::vl53l1x;
using TofSensor = esphome::vl53l1x::VL53L1X;

namespace esphome {
namespace roode {
#define NOBODY 0
#define SOMEONE 1
#define VERSION "1.5.2"
static const char *const TAG = "Roode";
static const char *const SETUP = "Setup";
static const char *const CALIBRATION = "Sensor Calibration";

/*
Use the VL53L1X_SetTimingBudget function to set the TB in milliseconds. The TB
values available are [15, 20, 33, 50, 100, 200, 500]. This function must be
called after VL53L1X_SetDistanceMode. Note: 15 ms only works with Short distance
mode. 100 ms is the default value. The TB can be adjusted to improve the
standard deviation (SD) of the measurement. Increasing the TB, decreases the SD
but increases the power consumption.
*/

static int delay_between_measurements = 0;
static int time_budget_in_ms = 0;

/*
Parameters which define the time between two different measurements in various
modes (https://www.st.com/resource/en/datasheet/vl53l1x.pdf) The timing budget
and inter-measurement period should not be called when the sensor is ranging.
The user has to stop the ranging, change these parameters, and restart ranging
The minimum inter-measurement period must be longer than the timing budget + 4
ms.
// Lowest possible is 15ms with the ULD API
(https://www.st.com/resource/en/user_manual/um2510-a-guide-to-using-the-vl53l1x-ultra-lite-driver-stmicroelectronics.pdf)
Valid values: [15,20,33,50,100,200,500]
*/
static int time_budget_in_ms_short = 15;  // max range: 1.3m
static int time_budget_in_ms_medium = 33;
static int time_budget_in_ms_medium_long = 50;
static int time_budget_in_ms_long = 100;
static int time_budget_in_ms_max = 200;  // max range: 4m

class Roode : public PollingComponent {
 public:
  void setup() override;
  void update() override;
  void loop() override;
  void dump_config() override;
  /** Roode uses data from sensors */
  float get_setup_priority() const override { return setup_priority::PROCESSOR; };

  TofSensor *get_tof_sensor() { return this->distanceSensor; }
  void set_tof_sensor(TofSensor *sensor) { this->distanceSensor = sensor; }
  void set_invert_direction(bool dir) { invert_direction_ = dir; }
  bool get_invert_direction() const { return invert_direction_; }
  void set_orientation(Orientation val) { orientation_ = val; }
  uint8_t get_sampling_size() const { return samples; }
  void set_sampling_size(uint8_t size) {
    samples = size;
    entry->set_max_samples(size);
    exit->set_max_samples(size);
  }
  uint16_t get_auto_recalibration_interval_minutes() const;
  void set_auto_recalibration_interval_ms(uint32_t duration_ms) { auto_recalibration_interval_ms_ = duration_ms; }
  void set_auto_recalibration_interval_minutes(uint16_t minutes) {
    auto_recalibration_interval_ms_ = minutes * 60000UL;
  }
  uint32_t get_minutes_since_last_recalibration() const;
  uint32_t get_minutes_until_next_recalibration() const;
  bool is_ready_for_recalibration() const;
  bool is_presence_detected() const { return presence_detected_; }
  bool is_masking_detected() const { return masking_detected_; }
  int get_sensor_status_code() const;
  uint16_t get_entry_distance() const;
  uint16_t get_exit_distance() const;
  uint16_t get_entry_max_threshold() const { return this->entry->threshold->max; }
  uint16_t get_exit_max_threshold() const { return this->exit->threshold->max; }
  uint16_t get_entry_min_threshold() const { return this->entry->threshold->min; }
  uint16_t get_exit_min_threshold() const { return this->exit->threshold->min; }
  float get_people_counter_value() const;
  void set_people_counter_value(float value);
  void reset_people_counter() { this->set_people_counter_value(0.0f); }
  const std::string &get_last_direction() const { return last_direction_; }
  uint8_t get_min_threshold_percentage() const;
  void set_min_threshold_percentage(uint8_t percentage);
  uint8_t get_max_threshold_percentage() const;
  void set_max_threshold_percentage(uint8_t percentage);
  uint8_t get_roi_width() const;
  void set_roi_width(uint8_t width);
  uint8_t get_roi_height() const;
  void set_roi_height(uint8_t height);
  void set_masking_distance_threshold(uint16_t distance_mm) { masking_distance_threshold_mm_ = distance_mm; }
  void set_masking_hold_time_ms(uint32_t duration_ms) { masking_hold_time_ms_ = duration_ms; }
  void set_distance_entry(sensor::Sensor *distance_entry_) { distance_entry = distance_entry_; }
  void set_distance_exit(sensor::Sensor *distance_exit_) { distance_exit = distance_exit_; }
  void set_people_counter(number::Number *counter) { this->people_counter = counter; }
  void set_max_threshold_entry_sensor(sensor::Sensor *max_threshold_entry_sensor_) {
    max_threshold_entry_sensor = max_threshold_entry_sensor_;
  }
  void set_max_threshold_exit_sensor(sensor::Sensor *max_threshold_exit_sensor_) {
    max_threshold_exit_sensor = max_threshold_exit_sensor_;
  }
  void set_min_threshold_entry_sensor(sensor::Sensor *min_threshold_entry_sensor_) {
    min_threshold_entry_sensor = min_threshold_entry_sensor_;
  }
  void set_min_threshold_exit_sensor(sensor::Sensor *min_threshold_exit_sensor_) {
    min_threshold_exit_sensor = min_threshold_exit_sensor_;
  }
  void set_entry_roi_height_sensor(sensor::Sensor *roi_height_sensor_) { entry_roi_height_sensor = roi_height_sensor_; }
  void set_entry_roi_width_sensor(sensor::Sensor *roi_width_sensor_) { entry_roi_width_sensor = roi_width_sensor_; }
  void set_exit_roi_height_sensor(sensor::Sensor *roi_height_sensor_) { exit_roi_height_sensor = roi_height_sensor_; }
  void set_exit_roi_width_sensor(sensor::Sensor *roi_width_sensor_) { exit_roi_width_sensor = roi_width_sensor_; }
  void set_sensor_status_sensor(sensor::Sensor *status_sensor_) { status_sensor = status_sensor_; }
  void set_presence_sensor_binary_sensor(binary_sensor::BinarySensor *presence_sensor_) {
    presence_sensor = presence_sensor_;
  }
  void set_masking_detected_binary_sensor(binary_sensor::BinarySensor *masking_sensor_) {
    masking_sensor = masking_sensor_;
  }
  void set_version_text_sensor(text_sensor::TextSensor *version_sensor_) { version_sensor = version_sensor_; }
  void set_entry_exit_event_text_sensor(text_sensor::TextSensor *entry_exit_event_sensor_) {
    entry_exit_event_sensor = entry_exit_event_sensor_;
  }
  void recalibration();
  Zone *entry = new Zone(0);
  Zone *exit = new Zone(1);

 protected:
  TofSensor *distanceSensor;
  Zone *current_zone = entry;
  sensor::Sensor *distance_entry;
  sensor::Sensor *distance_exit;
  number::Number *people_counter;
  sensor::Sensor *max_threshold_entry_sensor;
  sensor::Sensor *max_threshold_exit_sensor;
  sensor::Sensor *min_threshold_entry_sensor;
  sensor::Sensor *min_threshold_exit_sensor;
  sensor::Sensor *exit_roi_height_sensor;
  sensor::Sensor *exit_roi_width_sensor;
  sensor::Sensor *entry_roi_height_sensor;
  sensor::Sensor *entry_roi_width_sensor;
  sensor::Sensor *status_sensor;
  binary_sensor::BinarySensor *presence_sensor;
  binary_sensor::BinarySensor *masking_sensor{nullptr};
  text_sensor::TextSensor *version_sensor;
  text_sensor::TextSensor *entry_exit_event_sensor;

  VL53L1_Error last_sensor_status = VL53L1_ERROR_NONE;
  VL53L1_Error sensor_status = VL53L1_ERROR_NONE;
  void path_tracking(Zone *zone);
  bool handle_sensor_status();
  void calibrateDistance();
  void calibrate_zones();
  const RangingMode *determine_raning_mode(uint16_t average_entry_zone_distance, uint16_t average_exit_zone_distance);
  void publish_sensor_configuration(Zone *entry, Zone *exit, bool isMax);
  void updateCounter(int delta);
  void update_masking_state_();
  void publish_masking_state_(bool state);
  void publish_presence_state_(bool state);
  void handle_auto_recalibration_();
  Orientation orientation_{Parallel};
  uint8_t samples{2};
  bool invert_direction_{false};
  uint32_t auto_recalibration_interval_ms_{0};
  uint32_t last_recalibration_ms_{0};
  bool presence_detected_{false};
  uint16_t masking_distance_threshold_mm_{150};
  uint32_t masking_hold_time_ms_{2000};
  uint32_t masking_candidate_since_ms_{0};
  bool masking_detected_{false};
  std::string last_direction_{"Waiting"};
  int number_attempts = 20;  // TO DO: make this configurable
  int short_distance_threshold = 1300;
  int medium_distance_threshold = 2000;
  int medium_long_distance_threshold = 2700;
  int long_distance_threshold = 3400;
  int path_track_[4] = {0, 0, 0, 0};
  int path_track_filling_size_{1};
  int left_previous_status_{NOBODY};
  int right_previous_status_{NOBODY};
};

}  // namespace roode
}  // namespace esphome
