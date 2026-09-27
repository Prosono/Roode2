#pragma once
#include "Arduino.h"
namespace esphome {
struct AppMock {
  unsigned safe_reboots{0};
  void feed_wdt() {}
  void safe_reboot() { ++safe_reboots; }
};
inline AppMock App;
}
