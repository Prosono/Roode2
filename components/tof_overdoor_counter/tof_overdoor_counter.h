#pragma once

#include <algorithm>
#include <array>
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

enum SensorZone : uint8_t {
  ZONE_OUT = 0,
  ZONE_IN = 1,
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

enum PassageState : uint8_t {
  PASSAGE_IDLE = 0,
  PASSAGE_POSSIBLE = 1,
  PASSAGE_OCCUPIED = 2,
  PASSAGE_SEQUENCE = 3,
  PASSAGE_DIRECTION_DECIDED = 4,
  PASSAGE_COMPLETED = 5,
  PASSAGE_CANCELLED = 6,
  PASSAGE_TIMEOUT = 7,
};

class TofOverdoorCounter : public PollingComponent {
 public:
  static constexpr size_t SENSOR_COUNT = 4;
  static constexpr size_t SENSOR_ZONE_COUNT = 2;
  static constexpr size_t EVENT_LOG_SIZE = 12;
  // 6.4 seconds at the 25 ms trace cadence is enough to cover the longest
  // configured event while keeping static RAM and on-demand trace strings bounded.
  static constexpr size_t HISTORY_SIZE = 256;
  static constexpr size_t EVENT_EDGE_SIZE = 20;

  struct PersistedCalibration {
    uint8_t valid{0};
    uint16_t baseline_mm{0};
    uint16_t noise_mm{0};
    uint8_t quality{0};
  };

  struct PersistedZoneCalibration {
    uint8_t valid{0};
    uint16_t baseline_mm{0};
    uint16_t noise_mm{0};
    uint8_t quality{0};
  };

  struct PersistedState {
    uint8_t version{6};
    int32_t people_inside{0};
    uint32_t confirmed_in{0};
    uint32_t confirmed_out{0};
    uint32_t unsure_in{0};
    uint32_t unsure_out{0};
    uint32_t rejected{0};
    uint16_t trigger_threshold_mm{320};
    uint16_t clear_threshold_mm{160};
    uint16_t baseline_tolerance_mm{80};
    uint16_t minimum_clear_distance_mm{600};
    uint16_t debounce_ms{25};
    uint16_t detection_timeout_ms{1600};
    uint16_t cooldown_ms{80};
    uint16_t blocked_timeout_ms{1800};
    uint16_t standing_timeout_ms{2200};
    uint16_t min_event_sensors{3};
    uint16_t min_active_duration_ms{25};
    uint16_t direction_window_ms{90};
    uint16_t calibration_samples{24};
    uint16_t max_people_inside{50};
    uint8_t min_valid_sensors{3};
    uint8_t auto_save_enabled{1};
    uint8_t invert_direction{0};
    uint8_t debug_logging{0};
    PersistedZoneCalibration calibrations[SENSOR_COUNT][SENSOR_ZONE_COUNT];
  };

  struct PersistedStateV4 {
    uint8_t version{4};
    int32_t people_inside{0};
    uint32_t confirmed_in{0};
    uint32_t confirmed_out{0};
    uint32_t unsure_in{0};
    uint32_t unsure_out{0};
    uint16_t trigger_threshold_mm{320};
    uint16_t clear_threshold_mm{500};
    uint16_t baseline_tolerance_mm{80};
    uint16_t minimum_clear_distance_mm{600};
    uint16_t debounce_ms{45};
    uint16_t detection_timeout_ms{1600};
    uint16_t cooldown_ms{500};
    uint16_t blocked_timeout_ms{1800};
    uint16_t standing_timeout_ms{2200};
    uint16_t min_event_sensors{2};
    uint16_t min_active_duration_ms{35};
    uint16_t direction_window_ms{90};
    uint16_t calibration_samples{24};
    uint16_t max_people_inside{50};
    uint8_t min_valid_sensors{3};
    uint8_t auto_save_enabled{1};
    uint8_t invert_direction{0};
    uint8_t debug_logging{0};
    PersistedCalibration calibrations[SENSOR_COUNT];
  };

  struct PersistedStateV3 {
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
  // XSHUT must be asserted before the four sensors can collide at 0x29. The
  // potentially slow discovery itself is deferred until the main loop.
  float get_setup_priority() const override { return setup_priority::BUS; }

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
  void set_trigger_delta_mm(uint16_t trigger_delta_mm) {
    this->trigger_threshold_mm_ = trigger_delta_mm;
    if (this->clear_threshold_mm_ >= this->trigger_threshold_mm_) {
      this->clear_threshold_mm_ = static_cast<uint16_t>(this->trigger_threshold_mm_ / 2U);
    }
  }
  void set_release_delta_mm(uint16_t release_delta_mm) {
    this->clear_threshold_mm_ =
        release_delta_mm >= this->trigger_threshold_mm_ ? static_cast<uint16_t>(this->trigger_threshold_mm_ / 2U)
                                                        : release_delta_mm;
  }
  void set_sequence_timeout_ms(uint32_t sequence_timeout_ms) { this->detection_timeout_ms_ = sequence_timeout_ms; }
  void set_cooldown_ms(uint32_t cooldown_ms) { this->cooldown_ms_ = cooldown_ms; }
  void set_baseline_tolerance_mm(uint16_t baseline_tolerance_mm) {
    this->baseline_tolerance_mm_ = baseline_tolerance_mm;
  }
  void set_debounce_ms(uint32_t debounce_ms) { this->debounce_ms_ = debounce_ms; }
  void set_blocked_timeout_ms(uint32_t blocked_timeout_ms) { this->blocked_timeout_ms_ = blocked_timeout_ms; }
  void set_standing_timeout_ms(uint32_t standing_timeout_ms) { this->standing_timeout_ms_ = standing_timeout_ms; }
  void set_min_event_sensors(uint8_t min_event_sensors) { this->min_event_sensors_ = min_event_sensors; }
  void set_min_active_duration_ms(uint32_t min_active_duration_ms) {
    this->min_active_duration_ms_ = min_active_duration_ms;
  }
  void set_direction_window_ms(uint32_t direction_window_ms) { this->direction_window_ms_ = direction_window_ms; }
  void set_minimum_clear_distance_mm(uint16_t minimum_clear_distance_mm) {
    this->minimum_clear_distance_mm_ = minimum_clear_distance_mm;
  }
  void set_calibration_samples(uint16_t calibration_samples) { this->calibration_samples_ = calibration_samples; }
  void set_min_valid_sensors(uint8_t min_valid_sensors) { this->min_valid_sensors_ = min_valid_sensors; }
  void set_max_people_inside(uint16_t max_people_inside) {
    this->max_people_inside_ = max_people_inside;
    this->people_inside_ = std::min<int>(this->people_inside_, this->max_people_inside_);
  }
  void set_auto_save_enabled(bool auto_save_enabled) { this->auto_save_enabled_ = auto_save_enabled; }
  void set_invert_direction(bool invert_direction);
  void set_debug_logging(bool debug_logging) { this->debug_logging_ = debug_logging; }
  void set_debug_sample_interval_ms(uint32_t debug_sample_interval_ms) {
    this->debug_sample_interval_ms_ = debug_sample_interval_ms;
  }
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
  void reset_trace_buffer();
  void persist_runtime_state();
  void set_people_inside(int value);

  float get_discovered_sensor_count() const;
  float get_reporting_sensor_count() const;
  float get_cycle_duration_ms() const;
  float get_last_decision_latency_ms() const;
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
  float get_rejected_count() const;
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
  float get_min_event_sensors_value() const;
  float get_min_active_duration_value() const;
  float get_direction_window_value() const;
  bool get_invert_direction() const { return this->invert_direction_; }
  bool get_auto_save_enabled() const { return this->auto_save_enabled_; }
  bool get_debug_logging() const { return this->debug_logging_; }
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
  std::string get_passage_state_text() const;
  std::string get_debug_snapshot_text() const;
  std::string get_compact_state_text() const;
  std::string get_trace_log_text() const;
  std::string get_blocked_sensor_text() const;
  std::string get_summary() const;
  std::string get_discovery_map() const;
  std::string get_event_log() const;

 protected:
  struct ZoneState {
    bool has_reading{false};
    bool valid_measurement{false};
    bool sample_rejected{false};
    bool active{false};
    bool rising_edge{false};
    bool falling_edge{false};
    bool blocked{false};
    uint16_t raw_distance{0};
    uint16_t sampled_distance{0};
    bool has_sampled_distance{false};
    float filtered_distance{NAN};
    uint8_t range_status{255};
    uint32_t last_update_ms{0};
    uint32_t last_good_read_ms{0};
    uint32_t active_candidate_since_ms{0};
    uint32_t clear_candidate_since_ms{0};
    uint32_t active_since_ms{0};
    uint32_t last_rising_ms{0};
    uint32_t last_falling_ms{0};
    uint32_t active_duration_ms{0};
    uint8_t consecutive_invalid{0};
    float baseline{NAN};
    float noise{NAN};
    uint8_t calibration_quality{0};
    bool calibrated{false};
    float calibration_sum{0.0f};
    float calibration_sq_sum{0.0f};
    float calibration_min{NAN};
    float calibration_max{NAN};
    uint16_t calibration_samples{0};
    std::array<uint16_t, 8> samples{};
    uint8_t sample_head{0};
    uint8_t sample_count{0};
  };

  struct SensorVote {
    uint32_t timestamp_ms{0};
    uint8_t sensor_index{0};
    SensorGroup direction{GROUP_NONE};
    std::string sensor_label;
    std::string path_text;
    std::string reason;
  };

  struct Channel {
    std::unique_ptr<VL53L1X_ULD> sensor;
    uint8_t pin_number{0};
    uint8_t address{0};
    SensorGroup group{GROUP_NONE};
    ZoneState zones[SENSOR_ZONE_COUNT];
    uint8_t current_zone{ZONE_OUT};
    bool initialized{false};
    bool ranging_started{false};
    bool has_reading{false};
    bool calibrated{false};
    bool active{false};
    bool blocked{false};
    bool stale{true};
    bool valid_measurement{false};
    bool sample_rejected{false};
    bool rising_edge{false};
    bool falling_edge{false};
    uint16_t raw_distance{0};
    uint16_t sampled_distance{0};
    bool has_sampled_distance{false};
    float median_distance{NAN};
    float filtered_distance{NAN};
    float baseline{NAN};
    float noise{NAN};
    uint8_t range_status{255};
    uint16_t signal_per_spad{0};
    uint16_t ambient_rate{0};
    uint16_t spad_count{0};
    uint8_t calibration_quality{0};
    int last_error{0};
    uint8_t consecutive_errors{0};
    uint8_t consecutive_invalid{0};
    uint32_t last_update_ms{0};
    uint32_t last_good_read_ms{0};
    uint32_t last_read_duration_ms{0};
    uint32_t next_recovery_ms{0};
    uint8_t recovery_attempts{0};
    uint32_t active_candidate_since_ms{0};
    uint32_t clear_candidate_since_ms{0};
    uint32_t active_since_ms{0};
    uint32_t last_rising_ms{0};
    uint32_t last_falling_ms{0};
    uint32_t active_duration_ms{0};
    uint32_t first_trigger_in_event_ms{0};
    uint8_t roode_path[4] = {0, 0, 0, 0};
    uint8_t roode_path_filling_size{1};
    uint8_t roode_previous_status[SENSOR_ZONE_COUNT] = {0, 0};
    uint32_t roode_event_started_ms{0};
    uint32_t roode_last_activity_ms{0};
    SensorGroup roode_first_zone{GROUP_NONE};
    SensorGroup roode_last_solo_zone{GROUP_NONE};
    bool roode_seen_out{false};
    bool roode_seen_in{false};
    bool roode_seen_both{false};
    bool roode_vote_latched{false};
    uint8_t roode_transition_count{0};
    SensorGroup pending_vote{GROUP_NONE};
    uint32_t pending_vote_ms{0};
    std::string pending_vote_path;
    std::string last_vote_text{"none"};
    std::string last_path_text{"CLEAR"};
    float calibration_sum{0.0f};
    float calibration_sq_sum{0.0f};
    float calibration_min{NAN};
    float calibration_max{NAN};
    uint16_t calibration_samples{0};
    std::string source_label;
    std::string sensor_label;
  };

  struct HistorySample {
    uint32_t timestamp_ms{0};
    uint16_t raw_distance[SENSOR_COUNT] = {0, 0, 0, 0};
    uint16_t filtered_distance[SENSOR_COUNT] = {0, 0, 0, 0};
    uint8_t range_status[SENSOR_COUNT] = {255, 255, 255, 255};
    uint8_t valid_mask{0};
    uint8_t active_mask{0};
    uint8_t rising_mask{0};
    uint8_t falling_mask{0};
    uint8_t passage_state{PASSAGE_IDLE};
  };

  struct EventEdge {
    uint32_t timestamp_ms{0};
    uint8_t sensor_index{0};
    SensorGroup group{GROUP_NONE};
    bool rising{false};
    uint8_t active_mask{0};
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
  uint16_t intermeasurement_ms_{37};
  uint8_t init_retries_{3};
  uint8_t sampling_size_{2};
  uint16_t trigger_threshold_mm_{320};
  uint16_t clear_threshold_mm_{160};
  uint16_t baseline_tolerance_mm_{80};
  uint16_t minimum_clear_distance_mm_{600};
  uint32_t debounce_ms_{25};
  uint32_t detection_timeout_ms_{1600};
  uint32_t cooldown_ms_{80};
  uint32_t blocked_timeout_ms_{1800};
  uint32_t standing_timeout_ms_{2200};
  uint8_t min_event_sensors_{3};
  uint32_t min_active_duration_ms_{25};
  uint32_t direction_window_ms_{90};
  uint16_t calibration_samples_{24};
  uint16_t max_people_inside_{50};
  uint8_t min_valid_sensors_{3};
  bool auto_save_enabled_{true};
  bool invert_direction_{false};
  bool debug_logging_{false};
  uint32_t debug_sample_interval_ms_{250};
  OperatingMode mode_{OperatingMode::COUNT};
  bool wire_initialized_{false};
  bool calibration_active_{true};
  bool person_standing_in_door_{false};
  bool event_active_{false};
  bool state_dirty_{false};
  bool persisted_state_loaded_{false};
  bool startup_clear_validated_{false};
  SensorGroup event_first_group_{GROUP_NONE};
  SensorGroup event_second_group_{GROUP_NONE};
  SensorGroup event_direction_group_{GROUP_NONE};
  uint8_t event_sensor_mask_{0};
  uint8_t event_rising_mask_{0};
  uint8_t event_falling_mask_{0};
  uint8_t event_peak_active_count_{0};
  uint8_t event_peak_group_counts_[2] = {0, 0};
  uint32_t event_group_confirmed_ms_[2] = {0, 0};
  uint32_t calibration_started_ms_{0};
  uint32_t calibration_clear_since_ms_{0};
  uint32_t standing_clear_since_ms_{0};
  uint32_t event_started_ms_{0};
  uint32_t event_last_activity_ms_{0};
  uint32_t event_direction_decided_ms_{0};
  uint32_t event_first_edge_ms_{0};
  uint32_t event_last_edge_ms_{0};
  uint32_t cooldown_until_ms_{0};
  uint8_t event_path_[8] = {0};
  uint8_t event_path_size_{0};
  uint8_t event_last_state_code_{0};
  uint32_t cycle_duration_ms_{0};
  uint32_t last_decision_latency_ms_{0};
  uint32_t last_discovery_ms_{0};
  uint32_t last_detection_ms_{0};
  uint32_t event_log_count_{0};
  uint16_t history_head_{0};
  uint16_t history_count_{0};
  uint8_t event_edge_count_{0};
  uint8_t sensor_vote_count_{0};
  uint32_t last_debug_sample_log_ms_{0};
  uint32_t next_rediscovery_ms_{0};
  uint32_t boot_clear_since_ms_{0};
  int people_inside_{0};
  uint32_t confirmed_in_count_{0};
  uint32_t confirmed_out_count_{0};
  uint32_t unsure_in_count_{0};
  uint32_t unsure_out_count_{0};
  uint32_t rejected_count_{0};
  uint8_t last_confidence_{0};
  SystemStatus system_status_{STATUS_BOOTING};
  DetectionOutcome last_detection_outcome_{OUTCOME_NONE};
  PassageState passage_state_{PASSAGE_IDLE};
  std::string phase_text_{"Booting"};
  std::string last_direction_{"Waiting"};
  std::string last_reason_{"Booting"};
  std::string blocked_sensor_text_{"None"};
  std::string event_log_[EVENT_LOG_SIZE];
  HistorySample history_[HISTORY_SIZE];
  EventEdge event_edges_[EVENT_EDGE_SIZE];
  SensorVote sensor_votes_[SENSOR_COUNT];
  ESPPreferenceObject persisted_state_pref_;
  bool persisted_state_ready_{false};

  bool initialize_wire_();
  bool recover_wire_();
  bool clear_i2c_bus_();
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
  bool recover_channel_(size_t index, const char *reason);
  void service_recovery_(uint32_t now);
  bool set_channel_roi_(Channel &channel, uint8_t zone_index);
  bool switch_channel_zone_(Channel &channel);
  bool range_result_is_valid_(const VL53L1X_Result_t &result) const;
  void update_zone_sampling_(ZoneState &zone, uint16_t distance);
  void refresh_channel_aggregate_from_zones_(Channel &channel);
  float channel_logic_distance_(const Channel &channel) const;
  float zone_logic_distance_(const ZoneState &zone) const;
  float adaptive_trigger_delta_(const ZoneState &zone) const;
  float adaptive_release_delta_(const ZoneState &zone) const;
  void init_preferences_();
  void load_persisted_state_();
  uint32_t preference_key_() const;
  void apply_calibration_defaults_();
  void process_calibration_();
  void update_sensor_states_();
  void update_sensor_health_();
  void update_system_status_();
  void update_detection_state_machine_();
  void update_channel_path_tracker_(Channel &channel, size_t index, uint32_t now);
  void reset_channel_path_tracker_(Channel &channel);
  void collect_pending_sensor_votes_(uint32_t now);
  void clear_sensor_vote_window_();
  std::string sensor_vote_text_() const;
  std::string roode_path_text_(const Channel &channel) const;
  void finalize_event_(bool timed_out);
  void record_history_snapshot_(uint32_t now);
  void record_event_edge_(size_t index, const Channel &channel, bool rising, uint32_t now, uint8_t active_mask);
  void update_passage_state_(PassageState state);
  void debug_log_sample_(uint32_t now);
  void apply_idle_baseline_tracking_();
  void clear_event_tracking_();
  void update_blocked_state_();
  bool ready_for_counting_() const;
  bool has_restored_calibration_() const;
  bool all_reporting_() const;
  uint8_t healthy_sensor_count_() const;
  uint8_t reporting_sensor_count_() const;
  uint8_t active_sensor_count_() const;
  uint8_t active_sensor_count_for_group_(SensorGroup group) const;
  uint8_t triggered_sensor_count_for_group_(SensorGroup group) const;
  uint32_t first_trigger_ts_for_group_(SensorGroup group) const;
  bool group_is_active_(SensorGroup group) const;
  uint8_t current_group_state_code_(uint8_t active_out, uint8_t active_in) const;
  void append_event_path_state_(uint8_t state_code, uint32_t now);
  std::string event_path_text_() const;
  SensorGroup first_group_from_path_() const;
  float group_distance_internal_(SensorGroup group) const;
  float group_baseline_internal_(SensorGroup group) const;
  float group_drop_internal_(SensorGroup group) const;
  SensorGroup group_for_index_(size_t index) const;
  SensorGroup determine_first_group_from_current_state_() const;
  SensorGroup resolve_event_first_group_() const;
  SensorGroup map_physical_group_to_direction_(SensorGroup physical_group) const;
  std::string direction_text_for_group_(SensorGroup physical_group, bool unsure) const;
  std::string passage_state_text_(PassageState state) const;
  std::string system_status_text_(SystemStatus status) const;
  std::string status_text_for_(const Channel &channel) const;
  std::string health_text_for_(const Channel &channel) const;
  std::string format_uptime_(uint32_t ms) const;
  std::string sensor_mask_text_(uint8_t mask) const;
  std::string event_edge_text_() const;
  std::string event_timing_text_(SensorGroup resolved_first_group) const;
  void log_event_(const std::string &message);
  void register_detection_(DetectionOutcome outcome, uint8_t confidence, const std::string &reason);
  void restore_persisted_calibration_(Channel &channel, size_t zone_index,
                                      const PersistedZoneCalibration &persisted);
  PersistedZoneCalibration build_persisted_calibration_(const ZoneState &zone) const;
};

}  // namespace tof_overdoor_counter
}  // namespace esphome
