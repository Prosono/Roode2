#include "components/counting_core/runtime_guard.h"
#include <cassert>
#include <cstdint>
#include <iostream>

using esphome::counting_core::DisconnectionGuard;

int main() {
  constexpr uint32_t timeout = 300000;
  DisconnectionGuard boot;
  // Starting while disconnected allows a complete boot/connect grace period.
  assert(!boot.update(1234, false, timeout));
  assert(!boot.update(1234 + timeout - 1, false, timeout));
  assert(boot.update(1234 + timeout, false, timeout));
  assert(!boot.update(1234 + timeout + 1, false, timeout));
  assert(!boot.update(1234 + timeout * 2, false, timeout));

  DisconnectionGuard recovered;
  assert(!recovered.update(100, false, timeout));
  // A real reconnection cancels the entire pending outage, including at expiry.
  assert(!recovered.update(100 + timeout, true, timeout));
  assert(!recovered.update(200 + timeout, false, timeout));
  assert(!recovered.update(200 + timeout * 2 - 1, false, timeout));
  assert(recovered.update(200 + timeout * 2, false, timeout));
  assert(!recovered.update(201 + timeout * 2, true, timeout));
  assert(!recovered.update(202 + timeout * 2, false, timeout));
  assert(recovered.update(202 + timeout * 3, false, timeout));

  DisconnectionGuard disabled;
  assert(!disabled.update(10, false, timeout));
  assert(!disabled.update(20, false, 0));
  assert(!disabled.update(20 + timeout * 4, false, 0));
  // Re-enabling recovery starts a fresh grace period.
  assert(!disabled.update(20 + timeout * 4, false, timeout));
  assert(disabled.update(20 + timeout * 5, false, timeout));

  DisconnectionGuard wrap;
  const uint32_t start = UINT32_MAX - timeout / 2;
  assert(!wrap.update(start, false, timeout));
  assert(!wrap.update(start + timeout - 1, false, timeout));
  assert(wrap.update(start + timeout, false, timeout));
  assert(!wrap.update(start + timeout + 1, false, timeout));

  DisconnectionGuard connected;
  // Home Assistant could be offline throughout: only STA loss is an input.
  for (uint32_t time = 0; time < 24 * 60 * 60000U; time += 10000)
    assert(!connected.update(time, true, timeout));
  std::cout << "Runtime guard: boot grace, reconnect cancellation, disable, one-shot and rollover passed\n";
}
