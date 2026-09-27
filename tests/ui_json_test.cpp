#include "components/tof_overdoor_ui/json_response.h"
#include <cassert>
#include <iostream>
using namespace esphome::tof_overdoor_ui;
int main() {
  JsonDocument document;
  document["event_log"] = std::string(6000, 'e');
  document["last"] = "must survive serialization";
  const auto response = serialize_state_json(document);
  assert(response.code == 200 && response.body.size() > 5120);
  JsonDocument parsed;
  assert(deserializeJson(parsed, response.body) == DeserializationError::Ok);
  assert(parsed["event_log"].as<std::string>().size() == 6000);
  assert(parsed["last"].as<std::string>() == "must survive serialization");
  // Escaping can grow a short input beyond the bound; measure serialized bytes.
  document["event_log"] = std::string(13000, '\n');
  const auto rejected = serialize_state_json(document);
  assert(rejected.code == 500 && rejected.body.size() < 200);
  assert(deserializeJson(parsed, rejected.body) == DeserializationError::Ok);
  std::cout << "UI JSON: >5120-byte complete state and bounded escaped output passed\n";
}
