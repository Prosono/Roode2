#include "components/tof_overdoor_counter/tof_overdoor_counter.h"
#include <cassert>
#include <iostream>
uint32_t test_now=100;
using namespace esphome;
class MigrationHarness : public tof_overdoor_counter::TofOverdoorCounter {
 public:
  void run() {
    Preferences preferences;
    global_preferences=&preferences;
    channels_.resize(4);
    persisted_state_pref_=preferences.make_preference<PersistedState>(preference_key_(),true);
    persisted_state_ready_=true;
    PersistedState old;
    old.version=7;old.people_inside=6;old.confirmed_in=31;old.confirmed_out=25;
    old.unsure_in=4;old.unsure_out=3;old.trigger_threshold_mm=200;old.clear_threshold_mm=80;
    old.min_event_sensors=2;old.invert_direction=1;
    for(auto &sensor:old.calibrations)for(auto &zone:sensor)zone={1,850,10,80};
    persisted_state_pref_.save(&old);
    load_persisted_state_();
    assert(people_inside_==6 && confirmed_in_count_==31 && confirmed_out_count_==25);
    assert(unsure_in_count_==4 && unsure_out_count_==3 && invert_direction_);
    assert(trigger_threshold_mm_==200 && clear_threshold_mm_==80 && min_event_sensors_==2);
    for(const auto &channel:channels_)for(const auto &zone:channel.zones)assert(!zone.calibrated);
    // A fresh calibration saved by the new firmware survives another boot.
    for(auto &channel:channels_)for(auto &zone:channel.zones) {
      zone.calibrated=true;zone.baseline=860;zone.noise=9;zone.calibration_quality=88;
    }
    persist_runtime_state();
    PersistedState saved;assert(persisted_state_pref_.load(&saved) && saved.version==8);
    load_persisted_state_();
    assert(people_inside_==6 && confirmed_in_count_==31 && confirmed_out_count_==25);
    for(const auto &channel:channels_)for(const auto &zone:channel.zones)
      assert(zone.calibrated && zone.baseline==860 && zone.noise==9 && zone.calibration_quality==88);
    global_preferences=nullptr;mock_preferences.clear();
  }
};
int main(){MigrationHarness h;h.run();std::cout<<"Calibration migration preserves counts/tuning and refreshes legacy ROI baselines\n";}
