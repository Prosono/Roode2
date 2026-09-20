#pragma once
namespace esphome {
namespace setup_priority {constexpr float BUS=1000;}
class PollingComponent {public: virtual ~PollingComponent()=default; virtual void setup(){} virtual void update(){} virtual void dump_config(){} virtual float get_setup_priority() const{return 0;}};
}
