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

namespace esphome {
namespace tof_overdoor_counter {

class TofOverdoorCounter : public PollingComponent {
 public:
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
  void set_init_retries(uint8_t init_retries) { this->init_retries_ = init_retries; }
  void set_trigger_delta_mm(uint16_t trigger_delta_mm) { this->trigger_delta_mm_ = trigger_delta_mm; }
  void set_release_delta_mm(uint16_t release_delta_mm) { this->release_delta_mm_ = release_delta_mm; }
  void set_sequence_timeout_ms(uint32_t sequence_timeout_ms) { this->sequence_timeout_ms_ = sequence_timeout_ms; }
  void set_cooldown_ms(uint32_t cooldown_ms) { this->cooldown_ms_ = cooldown_ms; }
  void set_invert_direction(bool invert_direction) { this->invert_direction_ = invert_direction; }
  void add_xshut_pin(GPIOPin *pin, uint8_t number) {
    this->xshut_pins_.push_back(pin);
    this->xshut_pin_numbers_.push_back(number);
  }

  void rediscover();
  void recalibrate();
  void reset_counts();

  float get_discovered_sensor_count() const;
  float get_cycle_duration_ms() const;
  float get_update_skew_ms() const;
  float get_distance_mm(size_t index) const;
  float get_baseline_mm(size_t index) const;
  float get_row_distance_mm(size_t row_index) const;
  float get_row_baseline_mm(size_t row_index) const;
  float get_row_drop_mm(size_t row_index) const;
  float get_entry_count() const;
  float get_exit_count() const;
  float get_people_count() const;
  float get_presence_state() const;
  float get_ready_state() const;
  float get_row_active_state(size_t row_index) const;
  std::string get_status_text(size_t index) const;
  std::string get_source_label(size_t index) const;
  std::string get_phase_text() const;
  std::string get_last_direction_text() const;
  std::string get_summary() const;
  std::string get_discovery_map() const;

 protected:
  struct Channel {
    std::unique_ptr<VL53L1X_ULD> sensor;
    uint8_t pin_number{0};
    uint8_t address{0};
    bool initialized{false};
    bool has_reading{false};
    bool occupied{false};
    uint16_t last_distance{0};
    int last_error{0};
    uint32_t last_update_ms{0};
    uint32_t last_read_duration_ms{0};
    float baseline{NAN};
    uint16_t baseline_samples{0};
    std::string source_label;
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
  uint8_t init_retries_{3};
  uint16_t trigger_delta_mm_{350};
  uint16_t release_delta_mm_{220};
  uint32_t sequence_timeout_ms_{2000};
  uint32_t cooldown_ms_{600};
  bool invert_direction_{false};
  bool wire_initialized_{false};
  bool waiting_for_clear_{false};
  uint8_t start_row_{0};
  uint8_t last_single_row_{0};
  bool saw_both_{false};
  uint32_t sequence_started_ms_{0};
  uint32_t cooldown_until_ms_{0};
  uint32_t cycle_duration_ms_{0};
  uint32_t last_discovery_ms_{0};
  uint32_t entry_count_{0};
  uint32_t exit_count_{0};
  int people_count_{0};
  std::string last_direction_{"Waiting"};
  std::string phase_text_{"Booting"};

  bool initialize_wire_();
  bool recover_wire_();
  void prepare_xshut_pins_();
  void set_all_xshut_(bool state);
  void set_xshut_(size_t index, bool state);
  bool probe_address_(uint8_t address);
  bool wait_for_boot_(VL53L1X_ULD &sensor);
  bool set_temp_address_(VL53L1X_ULD &sensor, uint8_t address);
  bool configure_sensor_(Channel &channel);
  bool read_channel_(Channel &channel);
  void update_baselines_();
  void update_occupancy_states_();
  void update_state_machine_();
  void finalize_sequence_();
  void reset_sequence_();
  bool all_calibrated_() const;
  bool row_is_active_(uint8_t row_index) const;
  float row_distance_internal_(uint8_t row_index) const;
  float row_baseline_internal_(uint8_t row_index) const;
  float row_drop_internal_(uint8_t row_index) const;
  std::string status_text_for_(const Channel &channel) const;
};

}  // namespace tof_overdoor_counter
}  // namespace esphome
