#include "components/tof_overdoor_counter/tof_overdoor_counter.h"
#include "esphome/components/wifi/wifi_component.h"
#include <cassert>
#include <iostream>

uint32_t test_now = 100;
using namespace esphome;

class Harness : public tof_overdoor_counter::TofOverdoorCounter {
 public:
  Harness() {
    persisted_state_ready_ = true;
    persisted_state_pref_.key = 123;
    people_inside_ = 7;
    confirmed_in_count_ = 20;
    confirmed_out_count_ = 13;
    auto_save_enabled_ = false;
  }
  bool check(uint32_t now) { test_now = now; return check_wifi_recovery_(now); }
  void verify_saved() {
    PersistedState state{};
    assert(persisted_state_pref_.load(&state));
    assert(state.people_inside == 7 && state.confirmed_in == 20 && state.confirmed_out == 13);
  }
};

int main() {
  Harness absent;
  assert(!absent.check(100));
  assert(!absent.check(600000));
  assert(App.safe_reboots == 0);

  wifi::WiFiComponent network;
  wifi::global_wifi_component = &network;
  Harness recovery;
  assert(!recovery.check(100));
  assert(!recovery.check(300099));
  assert(recovery.check(300100));
  assert(App.safe_reboots == 1);
  recovery.verify_saved();
  assert(!recovery.check(600100));
  assert(App.safe_reboots == 1);

  Harness canceled;
  assert(!canceled.check(700000));
  network.connected = true;
  assert(!canceled.check(1000000));
  assert(!canceled.check(5000000));
  assert(App.safe_reboots == 1);
  network.connected = false;
  assert(!canceled.check(5000001));
  network.disabled = true;
  assert(!canceled.check(5300001));
  assert(App.safe_reboots == 1);
  network.disabled = false;

  Harness opt_out;
  opt_out.set_wifi_recovery_timeout_ms(0);
  assert(!opt_out.check(1));
  assert(!opt_out.check(3600000));
  assert(App.safe_reboots == 1);
  std::cout << "Wi-Fi recovery: no-Wi-Fi/disabled/HA-only outage ignored; STA timeout saves counts and reboots once\n";
}
