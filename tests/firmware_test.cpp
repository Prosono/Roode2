#include "components/tof_overdoor_counter/tof_overdoor_counter.h"
#include <cassert>
#include <iostream>
uint32_t test_now=100;
using namespace esphome;
class Harness : public tof_overdoor_counter::TofOverdoorCounter {
 public:
  Harness() {
    channels_.resize(4);
    for(auto &c:channels_) {
      c.initialized=c.ranging_started=c.has_reading=true;c.stale=false;
      c.sensor=std::make_unique<VL53L1X_ULD>();
      for(auto &z:c.zones){z.has_reading=z.valid_measurement=true;z.raw_distance=2000;z.filtered_distance=2000;z.last_good_read_ms=test_now;}
    }
    calibration_samples_=24;
  }
  void sample(bool fresh=true) {
    test_now+=5;
    for(auto &c:channels_)for(auto &z:c.zones){z.fresh=fresh;if(fresh)z.last_good_read_ms=test_now;}
  }
  void calibrated(){calibration_active_=false;startup_clear_validated_=true;for(auto &c:channels_){c.calibrated=true;for(auto &z:c.zones){z.calibrated=true;z.baseline=2000;z.noise=1;}}}
  void tests() {
    // 100 scheduler calls without a new sensor result contribute no samples.
    for(int i=0;i<100;++i){sample(false);process_calibration_();}
    assert(channels_[0].zones[0].calibration.count==0);
    for(int i=0;i<60 && calibration_active_;++i){sample();process_calibration_();}
    assert(!calibration_active_);
    assert(channels_[0].zones[0].calibration.count==24);
    calibrated();
    auto &z=channels_[0].zones[0];z.filtered_distance=1000;
    sample();update_sensor_states_();assert(!z.active);
    for(int i=0;i<20;++i){sample(false);update_sensor_states_();}assert(!z.active);
    sample();update_sensor_states_();assert(z.active);
    // A missing/invalid ROI invalidates the entire directional sensor.
    channels_[3].zones[1].valid_measurement=false;
    assert(healthy_sensor_count_()==3);
    channels_[2].zones[1].last_good_read_ms=test_now-1000;
    assert(healthy_sensor_count_()==2 && !ready_for_counting_());
    // Trace stores both fields and the baseline AT sample time.
    z.baseline=2100;record_history_snapshot_(test_now);z.baseline=2500;
    assert(history_[0].baseline[0]==2100 && history_[0].filtered_distance[0]==1000);
    assert(get_trace_log_text().find("_out_baseline")!=std::string::npos);
    // Recovery yields between power, boot, register and measurement stages.
    std::array<GPIOPin,4> pins;
    for(auto &pin:pins)xshut_pins_.push_back(&pin);
    channels_[3].initialized=false;channels_[3].next_recovery_ms=0;
    // Keep other channels healthy while the fourth is initialized.
    for(int i=0;i<400 && !channels_[3].initialized;++i) {
      sample();
      for(int j=0;j<3;++j)for(auto &zone:channels_[j].zones)zone.valid_measurement=true;
      const auto before=test_now;service_recovery_(test_now);
      assert(test_now-before<=2); // No wake/post-address sleeps in a loop turn.
    }
    assert(channels_[3].initialized);
    // The actual measurement -> debounce -> fusion -> counter integration.
    calibrated();fusion_.reset();for(auto &c:channels_)for(auto &zone:c.zones){zone.debounce={};zone.active=false;zone.noise=1;zone.baseline=2000;zone.valid_measurement=true;}
    auto frame=[&](uint8_t state) {
      for(int n=0;n<4;++n) {
        test_now+=75;
        for(auto &c:channels_)for(int k=0;k<2;++k){auto &zone=c.zones[k];zone.fresh=true;zone.last_good_read_ms=test_now;zone.filtered_distance=(state&(1<<k))?1000:2000;}
        update_sensor_states_();update_detection_state_machine_();
      }
    };
    frame(0);frame(1);frame(3);frame(2);assert(confirmed_out_count_==0);frame(0);assert(confirmed_out_count_==1);
    frame(1);frame(3);frame(2);frame(3);frame(1);frame(0);assert(confirmed_out_count_==1);
    // SensorInit yields, times out, and propagates errors instead of spinning.
    counting_core::SensorInit init;VL53L1X_ULD sensor;
    init.reset(0,1500);mock_ready=false;
    assert(init.step(sensor,1500)==counting_core::SensorInit::FAILED);
    init.reset(0,1500);mock_error=1;assert(init.step(sensor,1)==counting_core::SensorInit::FAILED);
    mock_error=0;mock_ready=true;init.reset(0,1500);
    auto state=counting_core::SensorInit::WAITING;
    for(int i=0;i<120 && state==counting_core::SensorInit::WAITING;++i)state=init.step(sensor,i*5);
    assert(state==counting_core::SensorInit::READY);
  }
};
int main(){Harness h;h.tests();std::cout<<"Firmware integration: fresh calibration, debounce, ROI health, trace and bounded initialization passed\n";}
