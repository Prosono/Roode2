#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <Wire.h>
#include "VL53L1X_ULD.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/core/log.h"
#include "esphome/core/preferences.h"

namespace esphome {
namespace tof_overdoor_counter {

enum OperatingMode : uint8_t {
  MONITOR = 0,
  COUNT = 1,
};

enum SensorDistanceMode : uint8_t {
  DISTANCE_MODE_SHORT = 0,
  DISTANCE_MODE_LONG = 1,
};

enum SensorGroup : uint8_t {
  GROUP_OUT = 0,
  GROUP_IN = 1,
  GROUP_NONE = 2,
};

enum SystemStatus : uint8_t {
  STATUS_BOOTING = 0,
  STATUS_CALIBRATING = 1,
  STATUS_READY = 2,
  STATUS_DETECTING = 3,
  STATUS_BLOCKED = 4,
  STATUS_DEGRADED = 5,
  STATUS_ERROR = 6,
};

enum DetectionOutcome : uint8_t {
  OUTCOME_NONE = 0,
  OUTCOME_IN = 1,
  OUTCOME_OUT = 2,
  OUTCOME_UNSURE_IN = 3,
  OUTCOME_UNSURE_OUT = 4,
};

class TofOverdoorCounter : public PollingComponent {
 public:
  static constexpr size_t SENSOR_COUNT = 4;
  static constexpr size_t EVENT_LOG_SIZE = 12;

  struct PersistedCalibration {
    uint8_t valid{0};
    uint16_t baseline_mm{0};
    uint16_t noise_mm{0};
    uint8_t quality{0};
  };

  struct PersistedState {
    uint8_t version{3};
    int32_t people_inside{0};
    uint32_t confirmed_in{0};
    uint32_t confirmed_out{0};
    uint32_t unsure_in{0};
    uint32_t unsure_out{0};
    uint16_t trigger_threshold_mm{320};
    uint16_t clear_threshold_mm{180};
    uint16_t baseline_tolerance_mm{80};
    uint16_t minimum_clear_distance_mm{600};
    uint16_t debounce_ms{45};
    uint16_t detection_timeout_ms{1600};
    uint16_t cooldown_ms{500};
    uint16_t blocked_timeout_ms{1800};
    uint16_t standing_timeout_ms{2200};
    uint16_t calibration_samples{24};
    uint16_t max_people_inside{50};
    uint8_t min_valid_sensors{3};
    uint8_t auto_save_enabled{1};
    uint8_t invert_direction{0};
    PersistedCalibration calibrations[SENSOR_COUNT];
  };

  void setup() override;
  void update() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_sda_pin(uint8_t pin) { this->sda_pin_ = pin; }
  void set_scl_pin(uint8_t pin) { this->scl_pin_ = pin; }
  void set_i2c_frequency(uint32_t frequency) { this->i2c_frequency_ = frequency; }
  void set_timeout_ms(uint32_t timeout_ms) { this->timeout_ms_ = timeout_ms; }
  void set_base_address(uint8_t address) { this->base_address_ = address; }
  void set_wake_delay_ms(uint32_t wake_delay_ms) { this->wake_delay_ms_ = wake_delay_ms; }
  void set_post_address_delay_ms(uint32_t post_address_delay_ms) {
    this->post_address_delay_ms_ = post_address_delay_ms;
  }
  void set_distance_mode(SensorDistanceMode distance_mode) { this->distance_mode_ = distance_mode; }
  void set_timing_budget_ms(uint16_t timing_budget_ms) { this->timing_budget_ms_ = timing_budget_ms; }
  void set_intermeasurement_ms(uint16_t intermeasurement_ms) { this->intermeasurement_ms_ = intermeasurement_ms; }
  void set_init_retries(uint8_t init_retries) { this->init_retries_ = init_retries; }
  void set_sampling_size(uint8_t sampling_size) { this->sampling_size_ = sampling_size; }
  void set_trigger_delta_mm(uint16_t trigger_delta_mm) { this->trigger_threshold_mm_ = trigger_delta_mm; }
  void set_release_delta_mm(uint16_t release_delta_mm) { this->clear_threshold_mm_ = release_delta_mm; }
  void set_sequence_timeout_ms(uint32_t sequence_timeout_ms) { this->detection_timeout_ms_ = sequence_timeout_ms; }
  void set_cooldown_ms(uint32_t cooldown_ms) { this->cooldown_ms_ = cooldown_ms; }
  void set_baseline_tolerance_mm(uint16_t baseline_tolerance_mm) {
    this->baseline_tolerance_mm_ = baseline_tolerance_mm;
  }
  void set_debounce_ms(uint32_t debounce_ms) { this->debounce_ms_ = debounce_ms; }
  void set_blocked_timeout_ms(uint32_t blocked_timeout_ms) { this->blocked_timeout_ms_ = blocked_timeout_ms; }
  void set_standing_timeout_ms(uint32_t standing_timeout_ms) { this->standing_timeout_ms_ = standing_timeout_ms; }
  void set_minimum_clear_distance_mm(uint16_t minimum_clear_distance_mm) {
    this->minimum_clear_distance_mm_ = minimum_clear_distance_mm;
  }
  void set_calibration_samples(uint16_t calibration_samples) { this->calibration_samples_ = calibration_samples; }
  void set_min_valid_sensors(uint8_t min_valid_sensors) { this->min_valid_sensors_ = min_valid_sensors; }
  void set_max_people_inside(uint16_t max_people_inside) { this->max_people_inside_ = max_people_inside; }
  void set_auto_save_enabled(bool auto_save_enabled) { this->auto_save_enabled_ = auto_save_enabled; }
  void set_invert_direction(bool invert_direction) { this->invert_direction_ = invert_direction; }
  void set_mode(OperatingMode mode) { this->mode_ = mode; }
  void add_xshut_pin(GPIOPin *pin, uint8_t number) {
    this->xshut_pins_.push_back(pin);
    this->xshut_pin_numbers_.push_back(number);
  }

  void rediscover();
  void recalibrate();
  void reset_counts();
  void reset_unsure_in();
  void reset_unsure_out();
  void reset_all_counters();
  void persist_runtime_state();

  float get_discovered_sensor_count() const;
  float get_reporting_sensor_count() const;
  float get_cycle_duration_ms() const;
  float get_update_skew_ms() const;
  float get_nearest_distance_mm() const;
  float get_average_distance_mm() const;
  float get_distance_span_mm() const;
  float get_distance_mm(size_t index) const;
  float get_raw_distance_mm(size_t index) const;
  float get_filtered_distance_mm(size_t index) const;
  float get_baseline_mm(size_t index) const;
  float get_delta_mm(size_t index) const;
  float get_noise_mm(size_t index) const;
  float get_calibration_quality(size_t index) const;
  float get_row_distance_mm(size_t row_index) const;
  float get_row_baseline_mm(size_t row_index) const;
  float get_row_drop_mm(size_t row_index) const;
  float get_entry_count() const;
  float get_exit_count() const;
  float get_people_count() const;
  float get_unsure_in_count() const;
  float get_unsure_out_count() const;
  float get_presence_state() const;
  float get_ready_state() const;
  float get_row_active_state(size_t row_index) const;
  float get_sensor_active_state(size_t index) const;
  float get_person_standing_state() const;
  float get_confidence_score() const;
  float get_calibration_progress() const;
  float get_max_people_inside_value() const;
  float get_trigger_threshold_value() const;
  float get_clear_threshold_value() const;
  float get_baseline_tolerance_value() const;
  float get_debounce_value() const;
  float get_detection_timeout_value() const;
  float get_cooldown_value() const;
  float get_min_valid_sensors_value() const;
  bool get_invert_direction() const { return this->invert_direction_; }
  bool get_auto_save_enabled() const { return this->auto_save_enabled_; }
  bool is_monitor_mode() const { return this->mode_ == OperatingMode::MONITOR; }
  bool is_count_mode() const { return this->mode_ == OperatingMode::COUNT; }
  std::string get_mode_text() const;
  std::string get_group_label(size_t group_index) const;
  std::string get_status_text(size_t index) const;
  std::string get_sensor_health_text(size_t index) const;
  std::string get_source_label(size_t index) const;
  std::string get_phase_text() const;
  std::string get_system_status_text() const;
  std::string get_last_direction_text() const;
  std::string get_last_detection_timestamp_text() const;
  std::string get_last_reason_text() const;
  std::string get_blocked_sensor_text() const;
  std::string get_summary() const;
  std::string get_discovery_map() const;
  std::string get_event_log() const;

 protected:
  struct Channel {
    std::unique_ptr<VL53L1X_ULD> sensor;
    uint8_t pin_number{0};
    uint8_t address{0};
    SensorGroup group{GROUP_NONE};
    bool initialized{false};
    bool ranging_started{false};
    bool has_reading{false};
    bool calibrated{false};
    bool active{false};
    bool blocked{false};
    bool stale{true};
    uint16_t raw_distance{0};
    uint16_t sampled_distance{0};
    bool has_sampled_distance{false};
    float filtered_distance{NAN};
    float baseline{NAN};
    float noise{NAN};
    uint8_t calibration_quality{0};
    int last_error{0};
    uint8_t consecutive_errors{0};
    uint32_t last_update_ms{0};
    uint32_t last_good_read_ms{0};
    uint32_t last_read_duration_ms{0};
    uint32_t active_candidate_since_ms{0};
    uint32_t clear_candidate_since_ms{0};
    uint32_t active_since_ms{0};
    uint32_t first_trigger_in_event_ms{0};
    float calibration_sum{0.0f};
    float calibration_sq_sum{0.0f};
    float calibration_min{NAN};
    float calibration_max{NAN};
    uint16_t calibration_samples{0};
    std::vector<uint16_t> samples;
    std::string source_label;
    std::string sensor_label;
  };

  std::vector<GPIOPin *> xshut_pins_;
  std::vector<uint8_t> xshut_pin_numbers_;
  std::vector<Channel> channels_;
  uint8_t sda_pin_{21};
  uint8_t scl_pin_{22};
  uint32_t i2c_frequency_{100000};
  uint32_t timeout_ms_{1500};
  uint8_t base_address_{0x30};
  uint32_t wake_delay_ms_{60};
  uint32_t post_address_delay_ms_{80};
  SensorDistanceMode distance_mode_{DISTANCE_MODE_LONG};
  uint16_t timing_budget_ms_{33};
  uint16_t intermeasurement_ms_{33};
  uint8_t init_retries_{3};
  uint8_t sampling_size_{2};
  uint16_t trigger_threshold_mm_{320};
  uint16_t clear_threshold_mm_{180};
  uint16_t baseline_tolerance_mm_{80};
  uint16_t minimum_clear_distance_mm_{600};
  uint32_t debounce_ms_{45};
  uint32_t detection_timeout_ms_{1600};
  uint32_t cooldown_ms_{500};
  uint32_t blocked_timeout_ms_{1800};
  uint32_t standing_timeout_ms_{2200};
  uint16_t calibration_samples_{24};
  uint16_t max_people_inside_{50};
  uint8_t min_valid_sensors_{3};
  bool auto_save_enabled_{true};
  bool invert_direction_{false};
  OperatingMode mode_{OperatingMode::COUNT};
  bool wire_initialized_{false};
  bool calibration_active_{true};
  bool person_standing_in_door_{false};
  bool event_active_{false};
  bool state_dirty_{false};
  SensorGroup event_first_group_{GROUP_NONE};
  SensorGroup event_second_group_{GROUP_NONE};
  uint8_t event_sensor_mask_{0};
  uint8_t event_peak_active_count_{0};
  uint8_t event_peak_group_counts_[2] = {0, 0};
  uint32_t calibration_started_ms_{0};
  uint32_t calibration_clear_since_ms_{0};
  uint32_t event_started_ms_{0};
  uint32_t event_last_activity_ms_{0};
  uint32_t cooldown_until_ms_{0};
  uint32_t cycle_duration_ms_{0};
  uint32_t last_discovery_ms_{0};
  uint32_t last_detection_ms_{0};
  uint32_t event_log_count_{0};
  int people_inside_{0};
  uint32_t confirmed_in_count_{0};
  uint32_t confirmed_out_count_{0};
  uint32_t unsure_in_count_{0};
  uint32_t unsure_out_count_{0};
  uint8_t last_confidence_{0};
  SystemStatus system_status_{STATUS_BOOTING};
  DetectionOutcome last_detection_outcome_{OUTCOME_NONE};
  std::string phase_text_{"Booting"};
  std::string last_direction_{"Waiting"};
  std::string last_reason_{"Booting"};
  std::string blocked_sensor_text_{"None"};
  std::string event_log_[EVENT_LOG_SIZE];
  ESPPreferenceObject persisted_state_pref_;
  bool persisted_state_ready_{false};

  bool initialize_wire_();
  bool recover_wire_();
  void prepare_xshut_pins_();
  void set_all_xshut_(bool state);
  void set_xshut_(size_t index, bool state);
  bool probe_address_(uint8_t address);
  bool wait_for_boot_(VL53L1X_ULD &sensor);
  bool set_temp_address_(VL53L1X_ULD &sensor, uint8_t address);
  bool configure_sensor_(Channel &channel);
  bool start_all_ranging_();
  bool read_channel_(Channel &channel);
  bool restart_ranging_(Channel &channel);
  void update_channel_sampling_(Channel &channel, uint16_t distance);
  float channel_logic_distance_(const Channel &channel) const;
  void init_preferences_();
  void load_persisted_state_();
  uint32_t preference_key_() const;
  void apply_calibration_defaults_();
  void process_calibration_();
  void update_sensor_states_();
  void update_sensor_health_();
  void update_system_status_();
  void update_detection_state_machine_();
  void finalize_event_(bool timed_out);
  void apply_idle_baseline_tracking_();
  void clear_event_tracking_();
  void update_blocked_state_();
  bool ready_for_counting_() const;
  bool all_reporting_() const;
  uint8_t healthy_sensor_count_() const;
  uint8_t reporting_sensor_count_() const;
  uint8_t active_sensor_count_() const;
  uint8_t active_sensor_count_for_group_(SensorGroup group) const;
  uint8_t triggered_sensor_count_for_group_(SensorGroup group) const;
  bool group_is_active_(SensorGroup group) const;
  float group_distance_internal_(SensorGroup group) const;
  float group_baseline_internal_(SensorGroup group) const;
  float group_drop_internal_(SensorGroup group) const;
  SensorGroup group_for_index_(size_t index) const;
  SensorGroup determine_first_group_from_current_state_() const;
  SensorGroup map_physical_group_to_direction_(SensorGroup physical_group) const;
  std::string direction_text_for_group_(SensorGroup physical_group, bool unsure) const;
  std::string system_status_text_(SystemStatus status) const;
  std::string status_text_for_(const Channel &channel) const;
  std::string health_text_for_(const Channel &channel) const;
  std::string format_uptime_(uint32_t ms) const;
  void log_event_(const std::string &message);
  void register_detection_(DetectionOutcome outcome, uint8_t confidence, const std::string &reason);
  void restore_persisted_calibration_(Channel &channel, size_t index, const PersistedCalibration &persisted);
  PersistedCalibration build_persisted_calibration_(const Channel &channel) const;
};

}  // namespace tof_overdoor_counter
}  // namespace esphome
