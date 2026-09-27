#pragma once

#include <ArduinoJson.h>
#include "main_thread_bridge.h"

namespace esphome {
namespace tof_overdoor_ui {

inline UiResponse serialize_state_json(const JsonDocument &document) {
  // This dashboard exceeds ESPHome's small-entity JSON serializer limit once
  // its bounded event log fills. Measure once and serialize into owned storage
  // instead of truncating at that helper's 5120-byte limit.
  constexpr size_t MAX_STATE_BYTES = 24U * 1024U;
  const size_t size = measureJson(document);
  if (document.overflowed() || size > MAX_STATE_BYTES)
    return {500, "application/json", "{\"ok\":false,\"message\":\"State snapshot exceeded memory limit\"}"};
  UiResponse response;
  response.body.reserve(size);
  serializeJson(document, response.body);
  return response;
}

}  // namespace tof_overdoor_ui
}  // namespace esphome
