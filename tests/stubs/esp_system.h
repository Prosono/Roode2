#pragma once
using esp_reset_reason_t=int;
constexpr int ESP_RST_POWERON=1, ESP_RST_BROWNOUT=2;
inline int esp_reset_reason(){return 0;}
