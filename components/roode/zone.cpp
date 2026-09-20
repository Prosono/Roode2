#include "zone.h"

namespace esphome {
namespace roode {

void Zone::dump_config() const {
  ESP_LOGCONFIG(TAG, "   %s", id == 0U ? "Entry" : "Exit");
  ESP_LOGCONFIG(TAG, "     ROI: { width: %d, height: %d, center: %d }", roi->width, roi->height, roi->center);
  ESP_LOGCONFIG(TAG, "     Threshold: { min: %dmm (%d%%), max: %dmm (%d%%), idle: %dmm }", threshold->min,
                threshold->min_percentage.value_or(threshold->idle ? (threshold->min * 100) / threshold->idle : 0), threshold->max,
                threshold->max_percentage.value_or(threshold->idle ? (threshold->max * 100) / threshold->idle : 0), threshold->idle);
}

VL53L1_Error Zone::readDistance(TofSensor *distanceSensor) {
  last_sensor_status = sensor_status;

  auto result = distanceSensor->read_distance(roi, sensor_status);
  if (!result.has_value() || sensor_status != VL53L1_ERROR_NONE || result.value() == 0) {
    samples.clear();
    if (sensor_status == VL53L1_ERROR_NONE) sensor_status = VL53L1_ERROR_TIME_OUT;
    return sensor_status;
  }

  last_distance = result.value();
  samples.insert(samples.begin(), result.value());
  if (samples.size() > max_samples) {
    samples.pop_back();
  };
  auto sorted = samples;
  std::sort(sorted.begin(), sorted.end());
  min_distance = sorted[sorted.size() / 2];

  return sensor_status;
}

/**
 * This sets the ROI for the zone to the given overrides or the standard default.
 * This is needed to do initial calibration of thresholds & ROI.
 */
void Zone::reset_roi(uint8_t default_center) {
  roi->width = roi_override->width ?: 6;
  roi->height = roi_override->height ?: 16;
  roi->center = roi_override->center ?: default_center;
  ESP_LOGD(TAG, "%s ROI reset: { width: %d, height: %d, center: %d }", id == 0U ? "Entry" : "Exit", roi->width,
           roi->height, roi->center);
}

bool Zone::calibrateThreshold(TofSensor *distanceSensor, int number_attempts) {
  if (number_attempts < 2) return false;
  counting_core::Statistics stats;
  samples.clear();
  for (int i = 0; i < number_attempts; ++i) {
    if (this->readDistance(distanceSensor) == VL53L1_ERROR_NONE && this->getDistance() > 0)
      stats.add(this->getDistance());
    App.feed_wdt();
  }
  samples.clear();
  if (stats.count < static_cast<unsigned>((number_attempts * 4 + 4) / 5) ||
      stats.mean <= 0 || stats.deviation() > stats.mean * 0.10f) {
    ESP_LOGW(CALIBRATION, "Invalid or unstable calibration in zone %u", id);
    return false;
  }
  const auto idle = static_cast<uint16_t>(std::max(1.0, stats.mean - stats.deviation()));
  const uint16_t minimum = threshold->min_percentage.has_value() ? idle * *threshold->min_percentage / 100 : threshold->min;
  const uint16_t maximum = threshold->max_percentage.has_value() ? idle * *threshold->max_percentage / 100 : threshold->max;
  if (minimum >= maximum) return false;
  threshold->idle = idle;
  threshold->min = minimum;
  threshold->max = maximum;
  ESP_LOGI(CALIBRATION, "Zone %u calibrated from %u valid samples: idle=%u min=%u max=%u", id, stats.count, idle, minimum, maximum);
  return true;
}

void Zone::roi_calibration(uint16_t entry_threshold, uint16_t exit_threshold, Orientation orientation) {
  // the value of the average distance is used for computing the optimal size of the ROI and consequently also the
  // center of the two zones
  int function_of_the_distance = 16 * (1 - (0.15 * 2) / (0.34 * (std::max<uint16_t>(1, std::min(entry_threshold, exit_threshold)) / 1000.0f)));
  int ROI_size = min(8, max(4, function_of_the_distance));
  this->roi->width = this->roi_override->width ?: ROI_size;
  this->roi->height = this->roi_override->height ?: ROI_size * 2;
  if (this->roi_override->center) {
    this->roi->center = this->roi_override->center;
  } else {
    // now we set the position of the center of the two zones
    if (orientation == Parallel) {
      switch (this->roi->width) {
        case 4:
          this->roi->center = this->id == 0U ? 150 : 247;
          break;
        case 5:
        case 6:
          this->roi->center = this->id == 0U ? 159 : 239;
          break;
        case 7:
        case 8:
          this->roi->center = this->id == 0U ? 167 : 231;
          break;
      }
    } else {
      switch (this->roi->width) {
        case 4:
          this->roi->center = this->id == 0U ? 193 : 58;
          break;
        case 5:
        case 6:
          this->roi->center = this->id == 0U ? 194 : 59;
          break;
        case 7:
        case 8:
          this->roi->center = this->id == 0U ? 195 : 60;
          break;
      }
    }
  }
  ESP_LOGI(CALIBRATION, "Calibrated ROI for zone. zoneId: %d, width: %d, height: %d, center: %d", id, roi->width,
           roi->height, roi->center);
}

uint16_t Zone::getDistance() const { return this->last_distance; }
uint16_t Zone::getMinDistance() const { return this->min_distance; }
bool Zone::hasDistance() const { return !this->samples.empty(); }
}  // namespace roode
}  // namespace esphome
