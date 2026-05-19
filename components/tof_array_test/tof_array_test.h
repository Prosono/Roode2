#pragma once

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
namespace tof_array_test {

struct TestROI {
  uint8_t width;
  uint8_t height;
  uint8_t center;
};

class TofArrayTest : public PollingComponent {
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
  void set_post_address_delay_ms(uint32_t post_address_delay_ms) { this->post_address_delay_ms_ = post_address_delay_ms; }
  void set_init_retries(uint8_t init_retries) { this->init_retries_ = init_retries; }
  void set_roi(uint8_t width, uint8_t height, uint8_t center);
  void add_xshut_pin(GPIOPin *pin, uint8_t number) {
    this->xshut_pins_.push_back(pin);
    this->xshut_pin_numbers_.push_back(number);
  }

  void rediscover();

  float get_discovered_sensor_count() const;
  float get_cycle_duration_ms() const;
  float get_update_skew_ms() const;
  float get_distance_span_mm() const;
  float get_distance_mm(size_t index) const;
  float get_age_ms(size_t index) const;
  float get_read_duration_ms(size_t index) const;
  float get_status_code(size_t index) const;
  float get_address_decimal(size_t index) const;
  std::string get_status_text(size_t index) const;
  std::string get_address_hex(size_t index) const;
  std::string get_source_label(size_t index) const;
  std::string get_summary() const;
  std::string get_discovery_map() const;
  std::string get_candidate_trace() const;

 protected:
  struct Channel {
    std::unique_ptr<VL53L1X_ULD> sensor;
    int xshut_index{-1};
    uint8_t address{0};
    bool initialized{false};
    bool has_reading{false};
    uint16_t last_distance{0};
    int last_error{0};
    uint32_t last_update_ms{0};
    uint32_t last_read_duration_ms{0};
    std::string source_label;
  };

  std::vector<GPIOPin *> xshut_pins_;
  std::vector<uint8_t> xshut_pin_numbers_;
  std::vector<Channel> channels_;
  TestROI roi_{16, 16, 199};
  uint8_t sda_pin_{21};
  uint8_t scl_pin_{22};
  uint32_t i2c_frequency_{400000};
  uint32_t timeout_ms_{1500};
  uint8_t base_address_{0x30};
  uint32_t wake_delay_ms_{20};
  uint32_t post_address_delay_ms_{30};
  uint8_t init_retries_{2};
  uint32_t cycle_duration_ms_{0};
  uint32_t last_discovery_ms_{0};
  bool wire_initialized_{false};
  std::vector<std::string> candidate_trace_;

  bool initialize_wire_();
  bool recover_wire_();
  void prepare_xshut_pins_();
  void set_all_xshut_(bool state);
  void set_xshut_(size_t index, bool state);
  bool probe_address_(uint8_t address);
  bool probe_default_sensor_();
  bool wait_for_boot_(VL53L1X_ULD &sensor);
  bool set_temp_address_(VL53L1X_ULD &sensor, uint8_t address);
  bool configure_sensor_(Channel &channel);
  bool read_channel_(Channel &channel);
  std::string status_text_for_(const Channel &channel) const;
};

}  // namespace tof_array_test
}  // namespace esphome
