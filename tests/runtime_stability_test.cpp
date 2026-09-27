#include "components/tof_overdoor_counter/tof_overdoor_counter.h"

#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <iostream>
#include <new>

// Track C++ heap ownership as well as allocation churn. Measuring only net
// bytes misses short-lived diagnostic strings that fragment an embedded heap.
namespace {
struct alignas(std::max_align_t) AllocationHeader { size_t size; };
size_t live_bytes = 0;
size_t tracked_allocations = 0;
bool track_allocations = false;
}

void *operator new(size_t size) {
  auto *header = static_cast<AllocationHeader *>(std::malloc(sizeof(AllocationHeader) + size));
  if (header == nullptr) throw std::bad_alloc();
  header->size = size;
  live_bytes += size;
  if (track_allocations) ++tracked_allocations;
  return header + 1;
}
void operator delete(void *ptr) noexcept {
  if (ptr == nullptr) return;
  auto *header = static_cast<AllocationHeader *>(ptr) - 1;
  live_bytes -= header->size;
  std::free(header);
}
void *operator new[](size_t size) { return ::operator new(size); }
void operator delete[](void *ptr) noexcept { ::operator delete(ptr); }
void operator delete(void *ptr, size_t) noexcept { ::operator delete(ptr); }
void operator delete[](void *ptr, size_t) noexcept { ::operator delete(ptr); }

uint32_t test_now = 10000;
using namespace esphome;

class RuntimeHarness : public tof_overdoor_counter::TofOverdoorCounter {
 public:
  RuntimeHarness() {
    channels_.resize(SENSOR_COUNT);
    for (size_t i = 0; i < channels_.size(); ++i) {
      auto &c = channels_[i];
      c.sensor = std::make_unique<VL53L1X_ULD>();
      c.sensor_label = "S" + std::to_string(i);
      c.initialized = c.ranging_started = c.has_reading = c.calibrated = true;
      c.stale = false; c.last_result_ms = c.ranging_started_ms = test_now;
      for (auto &z : c.zones) {
        z.has_reading = z.valid_measurement = z.calibrated = true;
        z.last_good_read_ms = z.last_update_ms = z.sample_started_ms = test_now;
        z.raw_distance = 700; z.baseline = z.filtered_distance = 700; z.noise = 1;
      }
    }
    wire_initialized_ = startup_clear_validated_ = cold_boot_reset_evaluated_ = true;
    calibration_active_ = false;
    sampling_size_ = 1; min_event_sensors_ = 2; min_valid_sensors_ = 3;
    trigger_threshold_mm_ = 200; clear_threshold_mm_ = 80;
    mock_result.Status = RangeValid; mock_result.Distance = 700;
  }

  void run_eight_hours_idle() {
    // Warm the history ring, diagnostic formatting, and the initial phase.
    step(0, 1000);
    (void)get_compact_state_text(); (void)get_trace_log_text();
    const auto before = live_bytes;
    tracked_allocations = 0; track_allocations = true;
    constexpr unsigned iterations = 8U * 60U * 60U * 1000U / 5U;
    for (unsigned i = 0; i < iterations; ++i) {
      test_now += 5;
      update();
    }
    track_allocations = false;
    assert(tracked_allocations == 0);
    assert(live_bytes == before);
    assert(history_count_ == HISTORY_SIZE);
    assert(confirmed_in_count_ == 0 && confirmed_out_count_ == 0 && rejected_count_ == 0);
    assert(ready_for_counting_());
    assert(get_compact_state_text().find("S0_path=first=0 last=0 seen=0") != std::string::npos);

    // The counter must still accept actual direction sequences after the soak.
    step(1); step(3); step(2); step(0);
    step(2); step(3); step(1); step(0);
    assert(confirmed_out_count_ == 1 && confirmed_in_count_ == 1);
    // Repeated diagnostics are temporary allocations, not retained history.
    const auto before_diagnostics = live_bytes;
    for (unsigned i = 0; i < 100; ++i) {
      (void)get_compact_state_text(); (void)get_trace_log_text();
    }
    assert(live_bytes == before_diagnostics);
  }

  void ambiguous_path_has_no_hotloop_strings() {
    // An ambiguous held episode creates >SSO-sized path text on both host
    // libc++ and ESP32 libstdc++; this catches the original per-loop churn.
    step(0);
    for (auto &c : channels_) for (auto &z : c.zones) z.active = true;
    for (unsigned i = 0; i < 1000; ++i) held_frame();
    assert(fusion_.path(0).ambiguous);
    assert(get_compact_state_text().find("S0_path=first=0 last=0 seen=3 ambiguous") != std::string::npos);
    const auto before = live_bytes;
    tracked_allocations = 0; track_allocations = true;
    for (unsigned i = 0; i < 100000; ++i) held_frame();
    track_allocations = false;
    assert(tracked_allocations == 0 && live_bytes == before);
  }

 private:
  void step(uint8_t state, unsigned iterations = 100) {
    for (unsigned i = 0; i < iterations; ++i) {
      test_now += 5;
      mock_result.Distance = (state & (1U << channels_[0].current_zone)) ? 200 : 700;
      update();
    }
  }
  void held_frame() {
    test_now += 5;
    for (auto &c : channels_) for (auto &z : c.zones) z.last_good_read_ms = test_now;
    update_detection_state_machine_();
  }
};

int main() {
  RuntimeHarness idle; idle.run_eight_hours_idle();
  RuntimeHarness ambiguous; ambiguous.ambiguous_path_has_no_hotloop_strings();
  std::cout << "Runtime: eight simulated hours at 5 ms, zero steady allocations, bounded diagnostics, and subsequent passages passed\n";
}
