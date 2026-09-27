# Availability update — 27 September 2026

Both counters reportedly become unavailable after a few hours, including their
web pages. TCP probes to HTTP and the native API timed out on both `.100` and
`.101`, while Home Assistant remained reachable. No device crash log was
available, so the exact failure on the installed devices is not established.

## Changes

- The custom HTTP handlers previously read and changed counter strings, history,
  settings and I2C state from ESPHome's HTTP task while the main loop could change
  them. All counter access now uses a bounded main-loop job. HTTP request pointers
  remain on the HTTP task. Pending jobs can time out without accumulating in the
  scheduler; a running action may complete even if the HTTP caller times out.
- Full UI state uses measured, owned JSON serialization with a 24 KiB response
  limit. Large event logs could exceed ESPHome 2026.8.1's 5119-byte helper limit.
  This fixes a truncation risk; it is not evidence of heap corruption or proof
  that the installed devices encountered this limit.
- The browser polls at most once per 500 ms after the preceding response, pauses
  in hidden tabs, serializes actions with polling, aborts requests after five
  seconds and backs off after failures.
- Path diagnostics are formatted only on request, removing four temporary
  diagnostic strings from every 5 ms acquisition cycle. This reduces allocation
  churn; no retained leak was established in the counter's steady loop.
- A station-only Wi-Fi guard saves counts and safely restarts after five minutes
  continuously disconnected. Home Assistant availability is not an input.
  `wifi_recovery_timeout: 0s` disables it. The guard does not recover every failure
  where the Wi-Fi stack still reports a connected station. It can interrupt an
  extended fallback-AP configuration session when the station cannot connect.
- The four-sensor YAML profiles disable internal web event telemetry and web log
  streaming, and use normal Wi-Fi scanning. HA entity names, static IP addresses
  and the ten exposed entities per device are retained.
- `/tof-overdoor-ui/state` adds `firmware_revision`, `uptime_ms`, `free_heap`,
  `min_free_heap` and `largest_free_block` for subsequent diagnosis.

## Verification and hardware test

Native tests cover an accelerated eight-hour, 5 ms counter loop with mocked
sensors, subsequent passages, bounded history, diagnostic allocations, Wi-Fi
timeout/reconnect/rollover, persistence before reboot, cross-thread job ownership,
complete JSON above 5 KB, response size bounds and browser request scheduling.
These tests do not emulate the ESP32 Wi-Fi driver, radio environment or heap.

Validation completed for this change:

- `tests/run.sh`: passed, with AddressSanitizer/UndefinedBehaviorSanitizer for
  native C++ tests and the real ArduinoJson headers for the JSON regression.
- `tests/config_test.py`: all 14 ESPHome schema cases passed on 2026.8.1.
- Both named profiles compiled successfully for ESP32 with ESPHome 2026.8.1,
  local component sources and dummy credentials. Each image uses 67.7% of the
  application flash partition; static DRAM usage is 54.0%, which is not a
  measurement of runtime free heap.
- The optional ZIP's checksums and included component sources were checked.

Use ESPHome 2026.8.1 to build the named profiles with your existing secrets:

- `dorteller-oppe.yaml`: Dørteller Oppe, `10.0.0.100`.
- `dorteller-nede.yaml`: Dørteller Nede, `10.0.0.101`.

The Git profiles refresh `main` at every build. The optional source ZIP bundles
the same component files locally and contains no credentials. Validation builds
use dummy credentials and must not be installed on the real devices.

After updating, verify both counters become ready and count IN/OUT. Observe them
for at least 8–12 hours, first with their browser pages closed and then with a
page open. Check HA availability and uptime; a repeating uptime reset means
restarts, not uninterrupted stability. If a device drops again, record whether
HTTP is reachable and obtain serial logs, which remain available even when
network logs cannot be reached. The update is a test candidate until physical
long-duration operation has been verified.
