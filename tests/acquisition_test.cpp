#include "components/tof_overdoor_counter/tof_overdoor_counter.h"
#include <cassert>
#include <iostream>
#include <limits>

uint32_t test_now = 10000;
using namespace esphome;

class AcquisitionHarness : public tof_overdoor_counter::TofOverdoorCounter {
 public:
  AcquisitionHarness() {
    mock_failed_operation.clear(); mock_result = {}; mock_ready = true; mock_operations.clear();
    mock_delayed_operation.clear(); mock_delay_ms = 0;
    mock_clock_pll = 1000; mock_guard_ticks = 0; mock_corrupt_guard_readback = false;
    channels_.resize(4);
    for (size_t i = 0; i < channels_.size(); ++i) {
      xshut_pins_.push_back(&pins_[i]);
      auto &c = channels_[i];
      c.sensor = std::make_unique<VL53L1X_ULD>();
      c.initialized = c.ranging_started = true;
      c.initialized_ms = test_now - 5000;
      c.last_result_ms = c.ranging_started_ms = test_now;
      c.address = 0x30 + i;
      for (auto &z : c.zones) {
        z.has_reading = z.valid_measurement = z.calibrated = true;
        z.last_update_ms = z.last_good_read_ms = test_now;
        z.baseline = z.filtered_distance = 700; z.noise = 1;
      }
    }
  }

  void optical_failures_do_not_reboot() {
    auto &c = channels_[0];
    // Several seconds of optically invalid results still represent a live
    // device. They must stay invalid for counting, without resetting XSHUT.
    for (unsigned i = 0; i < 50; ++i) {
      test_now += 70;
      for (size_t j = 1; j < channels_.size(); ++j) channels_[j].last_result_ms = test_now;
      mock_result.Status = i % 2 ? SignalFail : SigmaFail;
      assert(read_channel_(c));
      service_recovery_(test_now);
      assert(c.initialized && c.ranging_started);
      assert(recovery_stage_ == RecoveryStage::IDLE);
      assert(c.last_result_ms == test_now);
      assert(!channel_healthy_(c, test_now));
    }
    mock_result.Status = RangeValid;
    test_now += 70; assert(read_channel_(c));
    test_now += 70; assert(read_channel_(c));
    assert(channel_healthy_(c, test_now));
  }

  void stalled_and_failed_devices_recover() {
    auto &c = channels_[0];
    c.last_result_ms = test_now - 2001;
    service_recovery_(test_now);
    assert(!c.initialized && recovery_stage_ == RecoveryStage::POWER_OFF);
  }

  void communication_errors_recover() {
    auto &c = channels_[0];
    mock_failed_operation = "result";
    for (unsigned i = 0; i < 3; ++i) assert(!read_channel_(c));
    assert(c.consecutive_errors == 3);
    service_recovery_(test_now);
    assert(!c.initialized);
  }

  void hardware_faults_recover() {
    auto &c = channels_[0]; mock_result.Status = HardwareFail;
    for (unsigned i = 0; i < 3; ++i) { test_now += 70; assert(read_channel_(c)); }
    service_recovery_(test_now);
    assert(!c.initialized);
  }

  void missing_sensor_is_isolated() {
    recovery_index_ = 3; recovery_stage_ = RecoveryStage::BOOT;
    recovery_deadline_ = test_now; channels_[3].initialized_ms = test_now - 100;
    startup_clear_validated_ = true; event_active_ = true;
    service_recovery_(test_now);
    assert(!channels_[3].initialized);
    assert(startup_clear_validated_ && event_active_);
    assert(channels_[0].initialized && channels_[1].initialized && channels_[2].initialized);
  }

  void guarded_roi_and_overrun() {
    auto &c = channels_[0];
    assert(configure_sensor_(c));
    assert(guarded_intermeasurement_ms_() == 1000);
    assert(mock_guard_ticks == 1075000);  // More than 16 bits; readback must retain all bits.
    assert(restart_ranging_(c));
    assert(!c.ranging_started && c.settling_after_stop);
    test_now += timing_budget_ms_ + 4;
    mock_operations.clear();
    assert(restart_ranging_(c));
    assert((mock_operations == std::vector<std::string>{"clear", "roi", "center", "start"}));
    assert(c.current_zone == tof_overdoor_counter::ZONE_OUT);
    // Normal ROI alternation does not sleep an extra timing budget.
    test_now += timing_budget_ms_ + 4;
    const auto before = test_now; mock_operations.clear();
    assert(read_channel_(c));
    assert(test_now == before);
    assert(c.current_zone == tof_overdoor_counter::ZONE_IN);
    assert((mock_operations == std::vector<std::string>{"ready", "result", "clear", "stop", "roi", "center", "start"}));
    // A late result might belong to an autonomous range; never use it as a
    // fresh IN sample. Stop, yield, drain the interrupt, then restart IN.
    test_now += 960; mock_operations.clear();
    assert(read_channel_(c));
    assert((mock_operations == std::vector<std::string>{"stop"}));
    assert(c.settling_after_stop && !c.ranging_started);
    assert(c.zones[1].last_update_ms != test_now);
    mock_operations.clear(); test_now += timing_budget_ms_ + 3;
    assert(read_channel_(c)); assert(mock_operations.empty());
    ++test_now; mock_ready = false;
    assert(read_channel_(c));
    assert(c.ranging_started && !c.settling_after_stop);
    assert((mock_operations == std::vector<std::string>{"clear", "roi", "center", "start", "ready"}));
    // Deadlines remain valid across the 32-bit uptime rollover.
    test_now = std::numeric_limits<uint32_t>::max() - 10;
    c.ranging_started = false; assert(restart_ranging_(c));
    test_now += timing_budget_ms_ + 4; assert(restart_ranging_(c));
    assert(c.ranging_started && !c.settling_after_stop);
  }

  void failed_switch_never_claims_ranging() {
    auto &c = channels_[0];
    mock_failed_operation = "stop";
    assert(!switch_channel_zone_(c));
    assert(!c.ranging_started && c.current_zone == tof_overdoor_counter::ZONE_OUT);
    assert(c.last_error == VL53L1_ERROR_CONTROL_INTERFACE && c.consecutive_errors == 1);
    assert(!restart_ranging_(c)); assert(c.consecutive_errors == 2);
    assert(!restart_ranging_(c)); service_recovery_(test_now); assert(!c.initialized);
  }

  void slow_transaction_cannot_cross_roi_guard() {
    auto &c = channels_[0];
    const auto old_sample_ms = c.zones[0].last_update_ms;
    test_now += 900;
    mock_delayed_operation = "result"; mock_delay_ms = 60;
    assert(read_channel_(c));
    assert(c.current_zone == tof_overdoor_counter::ZONE_OUT);
    assert(c.zones[0].last_update_ms == old_sample_ms);
    assert(c.last_result_ms == test_now);
    assert(!c.ranging_started && c.settling_after_stop);
    assert((mock_operations == std::vector<std::string>{"ready", "result", "clear", "stop"}));
  }

  void long_budget_and_guard_programming() {
    auto &c = channels_[0];
    timing_budget_ms_ = 500; intermeasurement_ms_ = 504;
    assert(configure_sensor_(c));
    assert(guarded_intermeasurement_ms_() == 1100);
    assert(mock_guard_ticks == 1182500);
    assert(restart_ranging_(c));
    test_now += 504; assert(restart_ranging_(c));
    const auto sample_before = c.zones[0].last_update_ms;
    test_now += 504; assert(read_channel_(c));
    assert(c.zones[0].last_update_ms > sample_before);
    assert(c.current_zone == tof_overdoor_counter::ZONE_IN && c.ranging_started);
    // Maximum supported interval and clock register value must fit in the
    // actual 32-bit sensor register without the driver's getter truncation.
    intermeasurement_ms_ = 5000; mock_clock_pll = 1023;
    assert(configure_sensor_(c));
    assert(guarded_intermeasurement_ms_() == 5000 && mock_guard_ticks == 5498625);
    // Fail each bus operation independently; a bad guard must never be
    // accepted merely because an earlier clock read succeeded.
    for (const auto &failure : {"clock_read", "guard_write", "guard_read"}) {
      mock_failed_operation = failure;
      assert(!configure_sensor_(c));
      assert(c.last_error == VL53L1_ERROR_CONTROL_INTERFACE);
    }
    mock_failed_operation.clear(); mock_corrupt_guard_readback = true;
    assert(!configure_sensor_(c));
    mock_corrupt_guard_readback = false; mock_clock_pll = 0;
    assert(!configure_sensor_(c));
  }

 private:
  std::array<GPIOPin, 4> pins_;
};

int main() {
  AcquisitionHarness a; a.optical_failures_do_not_reboot();
  AcquisitionHarness b; b.stalled_and_failed_devices_recover();
  AcquisitionHarness c; c.communication_errors_recover();
  AcquisitionHarness d; d.hardware_faults_recover();
  AcquisitionHarness e; e.missing_sensor_is_isolated();
  AcquisitionHarness f; f.guarded_roi_and_overrun();
  AcquisitionHarness g; g.failed_switch_never_claims_ranging();
  AcquisitionHarness h; h.slow_transaction_cannot_cross_roi_guard();
  AcquisitionHarness i; i.long_budget_and_guard_programming();
  std::cout << "Acquisition: optical failures, transport recovery, isolated boot failures, ROI guard, drain, errors and rollover passed\n";
}
