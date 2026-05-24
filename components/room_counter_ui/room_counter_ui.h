#pragma once

#include <string>

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/switch/switch.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/component.h"

namespace esphome {
namespace room_counter_ui {

class RoomCounterUi : public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::WIFI; }

  void set_title(const std::string &title) { title_ = title; }
  void set_label(const std::string &label) { label_ = label; }
  void set_people_sensor(sensor::Sensor *sensor) { people_sensor_ = sensor; }
  void set_confirmed_in_sensor(sensor::Sensor *sensor) { confirmed_in_sensor_ = sensor; }
  void set_confirmed_out_sensor(sensor::Sensor *sensor) { confirmed_out_sensor_ = sensor; }
  void set_aggregate_invert_switch(switch_::Switch *sw) { aggregate_invert_switch_ = sw; }
  void set_rejected_events_sensor(sensor::Sensor *sensor) { rejected_events_sensor_ = sensor; }
  void set_last_event(text_sensor::TextSensor *sensor) { last_event_ = sensor; }
  void set_last_reason(text_sensor::TextSensor *sensor) { last_reason_ = sensor; }
  void set_vote_window(text_sensor::TextSensor *sensor) { vote_window_ = sensor; }

 protected:
  class Handler;

  std::string title_;
  std::string label_{"People in room"};
  sensor::Sensor *people_sensor_{nullptr};
  sensor::Sensor *confirmed_in_sensor_{nullptr};
  sensor::Sensor *confirmed_out_sensor_{nullptr};
  switch_::Switch *aggregate_invert_switch_{nullptr};
  sensor::Sensor *rejected_events_sensor_{nullptr};
  text_sensor::TextSensor *last_event_{nullptr};
  text_sensor::TextSensor *last_reason_{nullptr};
  text_sensor::TextSensor *vote_window_{nullptr};
  Handler *handler_{nullptr};
};

}  // namespace room_counter_ui
}  // namespace esphome
