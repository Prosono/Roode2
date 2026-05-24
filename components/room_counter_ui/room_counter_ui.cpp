#include "room_counter_ui.h"

#ifdef USE_WEBSERVER

#include <cmath>
#include <string>

#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace room_counter_ui {

static const char *const TAG = "room_counter_ui";

namespace {

const char ROOM_COUNTER_UI_HTML[] = R"html(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Room Counter</title>
  <style>
    :root {
      color-scheme: light dark;
      --bg: #f5f7fb;
      --panel: #ffffff;
      --ink: #172033;
      --muted: #5c6678;
      --line: #cbd3df;
      --accent: #164c70;
      --ok: #15845a;
      --bad: #b44a4a;
    }
    @media (prefers-color-scheme: dark) {
      :root {
        --bg: #141923;
        --panel: #1c2430;
        --ink: #f3f6fb;
        --muted: #a8b2c2;
        --line: #344154;
        --accent: #8ac7f4;
      }
    }
    * { box-sizing: border-box; }
    body {
      margin: 0;
      min-height: 100vh;
      display: grid;
      place-items: center;
      padding: 20px;
      font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
      color: var(--ink);
      background: var(--bg);
    }
    main {
      width: min(720px, 100%);
      border: 1px solid var(--line);
      border-radius: 18px;
      background: var(--panel);
      padding: clamp(22px, 5vw, 44px);
      box-shadow: 0 24px 70px rgba(20, 31, 48, 0.12);
    }
    .eyebrow {
      margin: 0 0 10px;
      color: var(--muted);
      font-size: 13px;
      font-weight: 700;
      letter-spacing: 0.14em;
      text-transform: uppercase;
    }
    h1 {
      margin: 0;
      font-size: clamp(30px, 6vw, 52px);
      line-height: 1;
      font-weight: 750;
      letter-spacing: 0;
    }
    .count {
      margin: 26px 0 18px;
      font-size: clamp(112px, 30vw, 250px);
      line-height: 0.9;
      font-weight: 850;
      letter-spacing: 0;
      color: var(--accent);
      font-variant-numeric: tabular-nums;
    }
    .label {
      margin: 0;
      color: var(--muted);
      font-size: clamp(20px, 4vw, 30px);
      font-weight: 700;
    }
    .grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 12px;
      margin-top: 28px;
    }
    .stat {
      min-height: 82px;
      border: 1px solid var(--line);
      border-radius: 12px;
      padding: 14px;
      background: color-mix(in srgb, var(--panel), var(--bg) 35%);
    }
    .stat span {
      display: block;
      margin-bottom: 8px;
      color: var(--muted);
      font-size: 12px;
      font-weight: 700;
      letter-spacing: 0.1em;
      text-transform: uppercase;
    }
    .stat strong {
      font-size: 26px;
      font-variant-numeric: tabular-nums;
    }
    .reason {
      margin-top: 18px;
      border-top: 1px solid var(--line);
      padding-top: 18px;
      color: var(--muted);
      line-height: 1.45;
      overflow-wrap: anywhere;
    }
    .status {
      display: inline-flex;
      align-items: center;
      gap: 8px;
      margin-top: 18px;
      color: var(--muted);
      font-size: 14px;
    }
    .actions {
      display: flex;
      flex-wrap: wrap;
      gap: 10px;
      align-items: center;
      margin-top: 18px;
    }
    button {
      appearance: none;
      border: 1px solid var(--line);
      border-radius: 999px;
      background: var(--accent);
      color: var(--panel);
      padding: 12px 16px;
      font: inherit;
      font-weight: 800;
      cursor: pointer;
    }
    button:disabled {
      cursor: not-allowed;
      opacity: 0.55;
    }
    .direction {
      color: var(--muted);
      font-weight: 700;
    }
    .dot {
      width: 10px;
      height: 10px;
      border-radius: 50%;
      background: var(--bad);
    }
    .dot.ok { background: var(--ok); }
    @media (max-width: 560px) {
      main { border-radius: 14px; }
      .grid { grid-template-columns: 1fr; }
      .stat { min-height: auto; }
    }
  </style>
</head>
<body>
  <main>
    <p class="eyebrow" id="title">Room Counter</p>
    <h1 id="heading">People in room</h1>
    <div class="count" id="count">0</div>
    <p class="label" id="event">Waiting</p>
    <section class="grid" aria-label="Counter details">
      <div class="stat"><span>IN</span><strong id="confirmed-in">0</strong></div>
      <div class="stat"><span>OUT</span><strong id="confirmed-out">0</strong></div>
      <div class="stat"><span>Rejected</span><strong id="rejected">0</strong></div>
    </section>
    <div class="reason" id="reason">Loading state...</div>
    <div class="actions">
      <button type="button" id="invert-button">Reverse direction</button>
      <span class="direction" id="direction">Direction: normal</span>
    </div>
    <div class="status"><span class="dot" id="dot"></span><span id="status">Connecting</span></div>
  </main>
  <script>
    const ids = {
      title: document.getElementById('title'),
      heading: document.getElementById('heading'),
      count: document.getElementById('count'),
      event: document.getElementById('event'),
      confirmedIn: document.getElementById('confirmed-in'),
      confirmedOut: document.getElementById('confirmed-out'),
      rejected: document.getElementById('rejected'),
      reason: document.getElementById('reason'),
      invertButton: document.getElementById('invert-button'),
      direction: document.getElementById('direction'),
      dot: document.getElementById('dot'),
      status: document.getElementById('status'),
    };
    let aggregateInvert = false;

    async function refresh() {
      try {
        const response = await fetch('/room-counter-ui/state', { cache: 'no-store' });
        if (!response.ok) throw new Error('state request failed');
        const state = await response.json();
        ids.title.textContent = state.title || 'Room Counter';
        ids.heading.textContent = state.label || 'People in room';
        ids.count.textContent = state.people_inside ?? 0;
        ids.event.textContent = state.last_event || 'Waiting';
        ids.confirmedIn.textContent = state.confirmed_in ?? 0;
        ids.confirmedOut.textContent = state.confirmed_out ?? 0;
        ids.rejected.textContent = state.rejected_events ?? 0;
        ids.reason.textContent = state.last_reason || state.vote_window || 'No event yet';
        aggregateInvert = !!state.aggregate_invert_direction;
        ids.direction.textContent = aggregateInvert ? 'Direction: reversed' : 'Direction: normal';
        ids.invertButton.disabled = !state.aggregate_invert_available;
        ids.invertButton.textContent = aggregateInvert ? 'Use normal direction' : 'Reverse direction';
        ids.dot.classList.add('ok');
        ids.status.textContent = 'Live';
      } catch (err) {
        ids.dot.classList.remove('ok');
        ids.status.textContent = 'Offline';
      }
    }

    async function toggleDirection() {
      ids.invertButton.disabled = true;
      try {
        await fetch('/room-counter-ui/action', {
          method: 'POST',
          headers: { 'Content-Type': 'application/x-www-form-urlencoded' },
          body: 'action=toggle_aggregate_invert'
        });
      } finally {
        await refresh();
      }
    }

    ids.invertButton.addEventListener('click', toggleDirection);
    refresh();
    setInterval(refresh, 700);
  </script>
</body>
</html>
)html";

int sensor_value(sensor::Sensor *sensor) {
  if (sensor == nullptr || std::isnan(sensor->state)) {
    return 0;
  }
  return static_cast<int>(std::round(sensor->state));
}

std::string text_value(text_sensor::TextSensor *sensor, const char *fallback) {
  if (sensor == nullptr || sensor->state.empty()) {
    return fallback;
  }
  return sensor->state;
}

}  // namespace

class RoomCounterUi::Handler : public AsyncWebHandler {
 public:
  explicit Handler(RoomCounterUi *parent) : parent_(parent) {}

  bool canHandle(AsyncWebServerRequest *request) const override {
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buf);
    auto method = request->method();
    if (method == HTTP_GET && (url == "/counter" || url == "/room-counter-ui/state")) {
      return true;
    }
    if (method == HTTP_POST && url == "/room-counter-ui/action") {
      return true;
    }
    return false;
  }

  void handleRequest(AsyncWebServerRequest *request) override {
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buf);
    if (request->method() == HTTP_GET && url == "/counter") {
      this->handle_index_(request);
      return;
    }
    if (request->method() == HTTP_GET && url == "/room-counter-ui/state") {
      this->handle_state_(request);
      return;
    }
    if (request->method() == HTTP_POST && url == "/room-counter-ui/action") {
      this->handle_action_(request);
      return;
    }
    request->send(404, "text/plain", "Not found");
  }

 protected:
  void handle_index_(AsyncWebServerRequest *request) {
    auto *response =
        request->beginResponse(200, "text/html; charset=utf-8",
                               reinterpret_cast<const uint8_t *>(ROOM_COUNTER_UI_HTML),
                               sizeof(ROOM_COUNTER_UI_HTML) - 1);
    request->send(response);
  }

  void handle_state_(AsyncWebServerRequest *request) {
    auto json = json::build_json([this](JsonObject root) {
      root["title"] = this->parent_->title_.empty() ? App.get_friendly_name() : this->parent_->title_;
      root["label"] = this->parent_->label_;
      root["people_inside"] = sensor_value(this->parent_->people_sensor_);
      root["confirmed_in"] = sensor_value(this->parent_->confirmed_in_sensor_);
      root["confirmed_out"] = sensor_value(this->parent_->confirmed_out_sensor_);
      root["aggregate_invert_available"] = this->parent_->aggregate_invert_switch_ != nullptr;
      root["aggregate_invert_direction"] = this->parent_->aggregate_invert_switch_ != nullptr
                                                ? this->parent_->aggregate_invert_switch_->state
                                                : false;
      root["rejected_events"] = sensor_value(this->parent_->rejected_events_sensor_);
      root["last_event"] = text_value(this->parent_->last_event_, "Waiting");
      root["last_reason"] = text_value(this->parent_->last_reason_, "No event yet");
      root["vote_window"] = text_value(this->parent_->vote_window_, "");
    });
    request->send(200, "application/json", json.c_str());
  }

  void handle_action_(AsyncWebServerRequest *request) {
    const auto action = request->arg("action");
    if (action != "toggle_aggregate_invert") {
      request->send(400, "application/json", "{\"ok\":false,\"message\":\"Unsupported action\"}");
      return;
    }
    if (this->parent_->aggregate_invert_switch_ == nullptr) {
      request->send(400, "application/json", "{\"ok\":false,\"message\":\"No direction switch configured\"}");
      return;
    }

    const bool next_state = !this->parent_->aggregate_invert_switch_->state;
    this->parent_->aggregate_invert_switch_->control(next_state);
    request->send(200, "application/json", next_state ? "{\"ok\":true,\"direction\":\"reversed\"}"
                                                       : "{\"ok\":true,\"direction\":\"normal\"}");
  }

  RoomCounterUi *parent_;
};

void RoomCounterUi::setup() {
  if (this->people_sensor_ == nullptr) {
    this->mark_failed();
    ESP_LOGE(TAG, "RoomCounterUi requires people_sensor");
    return;
  }

  if (web_server_base::global_web_server_base == nullptr) {
    this->mark_failed();
    ESP_LOGE(TAG, "RoomCounterUi requires web_server");
    return;
  }

  if (this->handler_ == nullptr) {
    this->handler_ = new Handler(this);
  }

  web_server_base::global_web_server_base->add_handler(this->handler_);
  ESP_LOGI(TAG, "Room counter UI registered on /counter");
}

}  // namespace room_counter_ui
}  // namespace esphome

#endif
