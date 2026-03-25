#pragma once

#include <string>
#include <vector>

#include "esphome/components/roode/roode.h"
#include "esphome/components/web_server_base/web_server_base.h"
#include "esphome/core/application.h"
#include "esphome/core/component.h"

namespace esphome {
namespace roode_ui {

class RoodeUi : public Component {
 public:
  struct NodeBinding {
    roode::Roode *node;
    std::string label;
  };

  void setup() override;
  float get_setup_priority() const override { return setup_priority::WIFI; }

  void set_title(const std::string &title) { title_ = title; }
  void add_node(roode::Roode *node, const std::string &label) { nodes_.push_back({node, label}); }

 protected:
  class Handler;

  std::string title_;
  std::vector<NodeBinding> nodes_;
  Handler *handler_{nullptr};
};

}  // namespace roode_ui
}  // namespace esphome
