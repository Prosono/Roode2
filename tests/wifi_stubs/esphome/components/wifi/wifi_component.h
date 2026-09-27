#pragma once
namespace esphome {
namespace wifi {
struct WiFiComponent {
  bool connected{false}, disabled{false};
  bool is_connected() const { return connected; }
  bool is_disabled() const { return disabled; }
};
inline WiFiComponent *global_wifi_component = nullptr;
}
}
