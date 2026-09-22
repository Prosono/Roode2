#include "components/tof_overdoor_counter/tof_overdoor_counter.h"
#include <cassert>
#include <iostream>
#include <fstream>
#include <sstream>
uint32_t test_now=100;
using namespace esphome;
class Harness : public tof_overdoor_counter::TofOverdoorCounter {
 public:
  Harness() {
    channels_.resize(4);
    unsigned index=0;
    for(auto &c:channels_) {
      c.sensor_label="S"+std::to_string(index++);
      c.initialized=c.ranging_started=c.has_reading=true;c.stale=false;
      c.last_result_ms=test_now;
      c.sensor=std::make_unique<VL53L1X_ULD>();
      for(auto &z:c.zones){z.has_reading=z.valid_measurement=true;z.raw_distance=2000;z.filtered_distance=2000;z.last_good_read_ms=test_now;}
    }
    calibration_samples_=24;
  }
  void sample(bool fresh=true) {
    test_now+=5;
    for(auto &c:channels_) {
      if(fresh)c.last_result_ms=test_now;
      for(auto &z:c.zones){z.fresh=fresh;if(fresh)z.last_good_read_ms=z.last_update_ms=z.sample_started_ms=test_now;}
    }
  }
  void calibrated(){calibration_active_=false;startup_clear_validated_=true;for(auto &c:channels_){c.calibrated=true;for(auto &z:c.zones){z.calibrated=true;z.baseline=2000;z.noise=1;}}}
  void replay_near_passages(unsigned votes) {
    min_event_sensors_=votes; min_valid_sensors_=3;
    calibrated(); sampling_size_=1; set_trigger_delta_mm(200); set_release_delta_mm(100);
    debounce_ms_=25; direction_window_ms_=90; cooldown_ms_=80; detection_timeout_ms_=3000;
    std::ifstream file("tests/fixtures/near_passages.txt"); assert(file.good());
    std::string line; bool first=true;
    while(std::getline(file,line)) {
      if(line.empty() || line[0]=='#')continue;
      std::istringstream row(line);unsigned now,fresh;row>>now>>fresh;test_now=now;
      for(int i=0;i<4;++i)for(int k=0;k<2;++k) {
        unsigned raw,status,baseline;row>>raw>>status>>baseline;assert(row);
        auto &c=channels_[i];auto &z=c.zones[k];z.fresh=false;
        if(first) {z.baseline=baseline;z.filtered_distance=baseline;z.raw_distance=baseline;z.last_good_read_ms=now;}
        if(status==255) {z.valid_measurement=false;z.range_status=255;continue;}
        if(!(fresh&(1U<<(i*2+k))))continue;
        c.current_zone=k;mock_result.Distance=raw;mock_result.Status=status;
        assert(read_channel_(c));
      }
      first=false;update_sensor_health_();update_sensor_states_();update_blocked_state_();
      update_detection_state_machine_();
    }
    std::cout<<"Recorded close passes: IN="<<confirmed_in_count_<<" OUT="<<confirmed_out_count_<<" rejected="<<rejected_count_<<" unsure="<<unsure_in_count_+unsure_out_count_<<"\n";
    if(votes==2) assert(confirmed_in_count_==1 && confirmed_out_count_==1 && rejected_count_==0 && unsure_in_count_+unsure_out_count_==0);
    else assert(confirmed_in_count_==0 && confirmed_out_count_==0 && unsure_in_count_==1 && unsure_out_count_==1 && rejected_count_==0);
    mock_result={};
  }
  void coverage_and_dropout_tests() {
    calibrated(); sampling_size_=1;debounce_ms_=25;direction_window_ms_=90;cooldown_ms_=80;
    auto frame=[&](std::array<uint8_t,4> states, unsigned repeats=6, uint16_t occupied=0) {
      for(unsigned n=0;n<repeats;++n) {
        test_now+=75;
        for(int i=0;i<4;++i)for(int k=0;k<2;++k) {
          auto &c=channels_[i];c.current_zone=k;
          mock_result.Status=RangeValid;mock_result.Distance=(states[i]&(1<<k))?occupied:2000;
          assert(read_channel_(c));
        }
        update_sensor_health_();service_recovery_(test_now);
        for(const auto &c:channels_)assert(c.initialized); // valid near readings never power-cycle
        update_sensor_states_();update_blocked_state_();update_detection_state_machine_();
      }
    };
    // An ordinary zero-distance pass retains direction without a cover alarm.
    frame({});frame({1,1,1,1});frame({3,3,3,3});frame({2,2,2,2});frame({});
    assert(confirmed_out_count_==1 && get_blocked_sensor_text()=="None");
    // Cover one sensor: alarm after a sustained obstruction, and the other
    // three continue counting while it remains covered.
    frame({0,0,0,3},30);
    assert(channels_[3].blocked && get_blocked_sensor_text()=="S3" && healthy_sensor_count_()==3 && ready_for_counting_());
    assert(confirmed_out_count_==1 && confirmed_in_count_==0);
    frame({1,1,1,3});frame({3,3,3,3});frame({2,2,2,3});frame({0,0,0,3});
    assert(confirmed_out_count_==2 && channels_[3].blocked);
    frame({2,2,2,3});frame({3,3,3,3});frame({1,1,1,3});frame({0,0,0,3});
    assert(confirmed_in_count_==1);
    frame({});assert(!channels_[3].blocked && confirmed_out_count_==2 && confirmed_in_count_==1);
    // Standing farther away is presence, not a covered-sensor alarm.
    frame({3,3,3,3},35,500);
    update_system_status_();assert(get_blocked_sensor_text()=="None" && get_system_status_text()!="Blocked");frame({});
    // Covering every sensor cannot invent a direction or a count.
    frame({3,3,3,3},35);assert(!ready_for_counting_());frame({});
    assert(confirmed_out_count_==2 && confirmed_in_count_==1);
    // Hold a short optical dropout only in an already occupied ROI.
    frame({1,1,1,1}); auto &c=channels_[0];auto &z=c.zones[0];
    test_now+=50;c.current_zone=0;mock_result.Status=SignalFail;mock_result.Distance=0;
    assert(read_channel_(c));update_sensor_states_();assert(z.active && channel_healthy_(c,test_now));
    test_now+=101;update_sensor_states_();assert(!z.active && !channel_healthy_(c,test_now));
    mock_result.Status=HardwareFail;z.active=true;z.last_good_read_ms=test_now;z.range_status=HardwareFail;
    assert(!zone_measurement_usable_(z,test_now));
    mock_result={};
  }
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
        for(auto &c:channels_)for(int k=0;k<2;++k){auto &zone=c.zones[k];zone.fresh=true;zone.last_good_read_ms=zone.last_update_ms=zone.sample_started_ms=test_now;zone.filtered_distance=(state&(1<<k))?1000:2000;}
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
int main(){Harness replay;replay.replay_near_passages(2);Harness strict;strict.replay_near_passages(3);Harness coverage;coverage.coverage_and_dropout_tests();Harness h;h.tests();std::cout<<"Firmware integration: fresh calibration, debounce, ROI health, trace and bounded initialization passed\n";}
