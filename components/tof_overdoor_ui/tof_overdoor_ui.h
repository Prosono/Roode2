#pragma once

#include <string>

#include "esphome/components/tof_overdoor_counter/tof_overdoor_counter.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/component.h"

namespace esphome {
namespace tof_overdoor_ui {

class TofOverdoorUi : public Component {
 public:
  void setup() override;
  float get_setup_priority() const override { return setup_priority::WIFI; }

  void set_title(const std::string &title) { title_ = title; }
  void set_label(const std::string &label) { label_ = label; }
  void set_counter(tof_overdoor_counter::TofOverdoorCounter *counter) { counter_ = counter; }

 protected:
  class Handler;

  std::string title_;
  std::string label_;
  tof_overdoor_counter::TofOverdoorCounter *counter_{nullptr};
  Handler *handler_{nullptr};
};

}  // namespace tof_overdoor_ui
}  // namespace esphome
