#pragma once
#include <cstdint>

namespace esphome {
namespace counting_core {

// Recover only after an uninterrupted network outage. The caller supplies STA
// connectivity, not Home Assistant/API availability. No work, allocation or
// retry loop occurs while waiting; one recovery is emitted per outage.
class DisconnectionGuard {
 public:
  bool update(uint32_t now, bool connected, uint32_t timeout_ms) {
    if (connected || timeout_ms == 0) {
      this->waiting_ = false;
      this->triggered_ = false;
      return false;
    }
    if (!this->waiting_) {
      this->waiting_ = true;
      this->disconnected_since_ = now;
      return false;
    }
    if (!this->triggered_ && static_cast<uint32_t>(now - this->disconnected_since_) >= timeout_ms) {
      this->triggered_ = true;
      return true;
    }
    return false;
  }

 private:
  uint32_t disconnected_since_{0};
  bool waiting_{false};
  bool triggered_{false};
};

}  // namespace counting_core
}  // namespace esphome
