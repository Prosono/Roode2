#pragma once
#include <math.h>
#include "../counting_core/counting_core.h"

#include "esphome/core/application.h"
#include "esphome/core/log.h"
#include "esphome/core/optional.h"
#include "../vl53l1x/vl53l1x.h"
#include "orientation.h"

using TofSensor = esphome::vl53l1x::VL53L1X;
using esphome::vl53l1x::ROI;

static const char *const TAG = "Zone";
static const char *const CALIBRATION = "Zone calibration";
namespace esphome {
namespace roode {
struct Threshold {
  /** Automatically determined idling distance (average of several measurements) */
  uint16_t idle{0};
  uint16_t min{0};
  optional<uint8_t> min_percentage{};
  uint16_t max{0};
  optional<uint8_t> max_percentage{};
  void set_min(uint16_t min) { this->min = min; }
  void set_min_percentage(uint8_t min) { this->min_percentage = min; }
  void set_max(uint16_t max) { this->max = max; }
  void set_max_percentage(uint8_t max) { this->max_percentage = max; }
};

class Zone {
 public:
  explicit Zone(uint8_t id) : id{id} {};
  void dump_config() const;
  VL53L1_Error readDistance(TofSensor *distanceSensor);
  void reset_roi(uint8_t default_center);
  bool calibrateThreshold(TofSensor *distanceSensor, int number_attempts);
  void roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold, Orientation orientation);
  const uint8_t id;
  uint16_t getDistance() const;
  uint16_t getMinDistance() const;
  bool hasDistance() const;
  ROI *roi = new ROI();
  ROI *roi_override = new ROI();
  Threshold *threshold = new Threshold();
  void set_max_samples(uint8_t max) { max_samples = std::max<uint8_t>(1, max); samples.clear(); };

 protected:
  VL53L1_Error last_sensor_status = VL53L1_ERROR_NONE;
  VL53L1_Error sensor_status = VL53L1_ERROR_NONE;
  uint16_t last_distance{0};
  uint16_t min_distance{0};
  std::vector<uint16_t> samples;
  uint8_t max_samples{2};
};
}  // namespace roode
}  // namespace esphome
