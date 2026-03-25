#include "roode_ui.h"

#ifdef USE_WEBSERVER

#include <cctype>
#include <cstdlib>
#include <string>

#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace roode_ui {

static const char *const TAG = "roode_ui";

namespace {

const char ROODE_UI_HTML[] = R"html(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Roode Control</title>
  <style>
    :root {
      color-scheme: light dark;
      --font-display: "Iowan Old Style", "Palatino Linotype", "URW Palladio L", serif;
      --font-body: "Avenir Next", "Segoe UI", "Helvetica Neue", sans-serif;
      --bg: oklch(96% 0.01 95);
      --bg-soft: oklch(98% 0.005 95);
      --panel: rgba(255, 255, 255, 0.84);
      --panel-strong: rgba(255, 255, 255, 0.96);
      --line: oklch(83% 0.02 250);
      --ink: oklch(28% 0.02 255);
      --ink-soft: oklch(48% 0.02 255);
      --accent: oklch(43% 0.08 242);
      --accent-soft: oklch(88% 0.03 240);
      --success: oklch(56% 0.1 150);
      --warn: oklch(70% 0.12 85);
      --danger: oklch(58% 0.13 28);
      --shadow: 0 20px 60px rgba(30, 36, 50, 0.08);
      --radius-lg: 28px;
      --radius-md: 18px;
      --radius-sm: 12px;
    }

    [data-theme="dark"] {
      --bg: oklch(20% 0.01 255);
      --bg-soft: oklch(24% 0.014 255);
      --panel: rgba(21, 27, 37, 0.88);
      --panel-strong: rgba(26, 33, 44, 0.96);
      --line: oklch(36% 0.015 255);
      --ink: oklch(94% 0.01 95);
      --ink-soft: oklch(74% 0.02 255);
      --accent: oklch(72% 0.08 240);
      --accent-soft: rgba(76, 103, 170, 0.18);
      --shadow: 0 26px 70px rgba(0, 0, 0, 0.28);
    }

    * { box-sizing: border-box; }
    body {
      margin: 0;
      font-family: var(--font-body);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, color-mix(in oklch, var(--accent) 10%, transparent), transparent 32%),
        linear-gradient(180deg, var(--bg-soft), var(--bg));
      min-height: 100vh;
    }

    .shell {
      width: min(1180px, calc(100vw - 2rem));
      margin: 0 auto;
      padding: 1rem 0 2rem;
    }

    .masthead {
      display: grid;
      grid-template-columns: 1fr auto;
      gap: 1rem;
      align-items: start;
      margin-bottom: 1rem;
    }

    .hero {
      padding: 1.4rem 1.5rem;
      border: 1px solid var(--line);
      border-radius: var(--radius-lg);
      background: var(--panel);
      box-shadow: var(--shadow);
      backdrop-filter: blur(14px);
    }

    .eyebrow {
      margin: 0 0 .35rem;
      letter-spacing: .12em;
      text-transform: uppercase;
      font-size: .72rem;
      color: var(--ink-soft);
    }

    h1, h2, h3 {
      font-family: var(--font-display);
      letter-spacing: -0.03em;
      margin: 0;
      font-weight: 600;
    }

    h1 { font-size: clamp(2.3rem, 4vw, 4rem); line-height: .94; }
    h2 { font-size: clamp(1.4rem, 2.2vw, 1.85rem); }
    h3 { font-size: 1rem; }

    .hero p {
      margin: .55rem 0 0;
      max-width: 56ch;
      color: var(--ink-soft);
      line-height: 1.5;
    }

    .top-tools {
      display: grid;
      gap: .9rem;
      min-width: min(320px, 100%);
    }

    .panel {
      border: 1px solid var(--line);
      border-radius: var(--radius-md);
      background: var(--panel-strong);
      box-shadow: var(--shadow);
      backdrop-filter: blur(10px);
    }

    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: .8rem;
      padding: 1rem;
    }

    .status-chip {
      padding: .9rem 1rem;
      border-radius: var(--radius-sm);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
    }

    .status-chip span {
      display: block;
      font-size: .74rem;
      text-transform: uppercase;
      letter-spacing: .08em;
      color: var(--ink-soft);
      margin-bottom: .35rem;
    }

    .status-chip strong {
      font-size: 1rem;
      font-weight: 600;
    }

    .theme-toggle {
      appearance: none;
      border: 1px solid var(--line);
      background: var(--panel-strong);
      color: var(--ink);
      border-radius: 999px;
      padding: .85rem 1.15rem;
      font: inherit;
      cursor: pointer;
      box-shadow: var(--shadow);
    }

    .connection {
      display: inline-flex;
      align-items: center;
      gap: .5rem;
      padding: .65rem .8rem;
      border-radius: 999px;
      background: var(--accent-soft);
      color: var(--accent);
      font-weight: 600;
      font-size: .95rem;
    }

    .connection::before {
      content: "";
      width: .6rem;
      height: .6rem;
      border-radius: 999px;
      background: currentColor;
      box-shadow: 0 0 0 .25rem color-mix(in oklch, currentColor 18%, transparent);
    }

    .node-list {
      display: grid;
      gap: 1rem;
    }

    .node-card {
      padding: 1.25rem;
    }

    .node-head {
      display: flex;
      justify-content: space-between;
      gap: 1rem;
      align-items: start;
      margin-bottom: 1rem;
    }

    .state-pill {
      padding: .55rem .85rem;
      border-radius: 999px;
      font-size: .84rem;
      font-weight: 700;
      letter-spacing: .03em;
      white-space: nowrap;
    }

    .state-ready { background: color-mix(in oklch, var(--success) 16%, transparent); color: var(--success); }
    .state-warn { background: color-mix(in oklch, var(--warn) 20%, transparent); color: color-mix(in oklch, var(--warn) 75%, black); }
    .state-error { background: color-mix(in oklch, var(--danger) 16%, transparent); color: var(--danger); }

    .metrics {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: .9rem;
      margin-bottom: .95rem;
    }

    .metric {
      padding: 1rem;
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 88%, var(--bg-soft));
    }

    .metric header {
      display: flex;
      justify-content: space-between;
      gap: .5rem;
      align-items: baseline;
      margin-bottom: .7rem;
      color: var(--ink-soft);
      font-size: .82rem;
      text-transform: uppercase;
      letter-spacing: .08em;
    }

    .metric strong {
      display: block;
      font-size: clamp(1.85rem, 4vw, 2.8rem);
      line-height: .95;
      margin-bottom: .7rem;
    }

    .track {
      width: 100%;
      height: .7rem;
      background: color-mix(in oklch, var(--line) 74%, transparent);
      border-radius: 999px;
      overflow: hidden;
    }

    .track > span {
      display: block;
      height: 100%;
      width: 0;
      border-radius: inherit;
      background: linear-gradient(90deg, color-mix(in oklch, var(--accent) 70%, white), var(--accent));
      transition: width .26s ease;
    }

    .badges {
      display: flex;
      flex-wrap: wrap;
      gap: .55rem;
      margin-bottom: 1rem;
    }

    .badge {
      padding: .45rem .7rem;
      border-radius: 999px;
      font-size: .78rem;
      font-weight: 700;
      letter-spacing: .03em;
      background: var(--bg-soft);
      border: 1px solid var(--line);
      color: var(--ink-soft);
    }

    .badge.live { color: var(--accent); border-color: color-mix(in oklch, var(--accent) 30%, var(--line)); }
    .badge.warn { color: color-mix(in oklch, var(--warn) 75%, black); border-color: color-mix(in oklch, var(--warn) 30%, var(--line)); }

    .control-cluster {
      display: flex;
      flex-wrap: wrap;
      gap: .75rem;
      margin-bottom: 1rem;
    }

    button, .button-link {
      appearance: none;
      border: 0;
      border-radius: 999px;
      padding: .88rem 1.15rem;
      font: inherit;
      font-weight: 700;
      cursor: pointer;
      transition: transform .18s ease, opacity .18s ease, background .18s ease;
      text-decoration: none;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: .4rem;
    }

    button:hover, .button-link:hover { transform: translateY(-1px); }
    button:disabled { opacity: .55; cursor: wait; transform: none; }

    .primary {
      color: white;
      background: linear-gradient(135deg, color-mix(in oklch, var(--accent) 72%, black), var(--accent));
    }

    .secondary {
      color: var(--ink);
      background: color-mix(in oklch, var(--panel-strong) 80%, var(--bg-soft));
      border: 1px solid var(--line);
    }

    .counter-box, details {
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 84%, var(--bg-soft));
    }

    .counter-box {
      padding: 1rem;
      display: grid;
      grid-template-columns: 1fr auto auto;
      gap: .7rem;
      align-items: end;
      margin-bottom: .95rem;
    }

    details { overflow: hidden; }
    details summary {
      list-style: none;
      cursor: pointer;
      padding: 1rem 1rem .95rem;
      display: flex;
      justify-content: space-between;
      align-items: center;
      font-weight: 700;
    }

    details summary::-webkit-details-marker { display: none; }

    .tuning-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: .8rem;
      padding: 0 1rem 1rem;
    }

    label {
      display: grid;
      gap: .45rem;
      color: var(--ink-soft);
      font-size: .86rem;
      font-weight: 600;
    }

    input[type="number"], select {
      width: 100%;
      border-radius: .95rem;
      border: 1px solid var(--line);
      padding: .8rem .9rem;
      font: inherit;
      color: var(--ink);
      background: var(--panel-strong);
    }

    .node-footer {
      display: flex;
      justify-content: space-between;
      gap: 1rem;
      align-items: center;
      padding-top: .85rem;
      color: var(--ink-soft);
      font-size: .85rem;
    }

    .toast {
      position: fixed;
      right: 1rem;
      bottom: 1rem;
      padding: .95rem 1.1rem;
      border-radius: 1rem;
      background: var(--panel-strong);
      border: 1px solid var(--line);
      color: var(--ink);
      box-shadow: var(--shadow);
      opacity: 0;
      transform: translateY(8px);
      pointer-events: none;
      transition: opacity .2s ease, transform .2s ease;
    }

    .toast.show {
      opacity: 1;
      transform: translateY(0);
    }

    .empty {
      padding: 2rem;
      text-align: center;
      color: var(--ink-soft);
    }

    @media (max-width: 960px) {
      .masthead { grid-template-columns: 1fr; }
      .metrics, .tuning-grid, .status-grid, .counter-box {
        grid-template-columns: 1fr;
      }
      .node-head, .node-footer {
        flex-direction: column;
        align-items: start;
      }
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="masthead">
      <section class="hero">
        <p class="eyebrow">Roode service console</p>
        <h1 id="page-title">Loading device</h1>
        <p id="page-subtitle">Preparing calibration controls, live distances, and sensor tuning.</p>
      </section>
      <div class="top-tools">
        <button class="theme-toggle" type="button" id="theme-toggle">Switch theme</button>
        <section class="panel status-grid">
          <div class="status-chip">
            <span>Connection</span>
            <strong id="connection-copy">Connecting</strong>
          </div>
          <div class="status-chip">
            <span>Last sync</span>
            <strong id="last-sync">Waiting</strong>
          </div>
        </section>
      </div>
    </div>
    <div class="connection" id="connection-pill">Connecting to device</div>
    <main class="node-list" id="node-list">
      <section class="panel empty">Loading sensor workspace…</section>
    </main>
  </div>
  <div class="toast" id="toast"></div>
  <script>
    const stateUrl = '/roode-ui/state';
    const actionUrl = '/roode-ui/action';
    const nodeList = document.getElementById('node-list');
    const pageTitle = document.getElementById('page-title');
    const pageSubtitle = document.getElementById('page-subtitle');
    const connectionPill = document.getElementById('connection-pill');
    const connectionCopy = document.getElementById('connection-copy');
    const lastSync = document.getElementById('last-sync');
    const toast = document.getElementById('toast');
    const themeToggle = document.getElementById('theme-toggle');

    let busy = false;

    const rememberedTheme = localStorage.getItem('roode-theme');
    if (rememberedTheme) {
      document.documentElement.dataset.theme = rememberedTheme;
    }

    themeToggle.addEventListener('click', () => {
      const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
      document.documentElement.dataset.theme = next;
      localStorage.setItem('roode-theme', next);
    });

    function showToast(message, isError = false) {
      toast.textContent = message;
      toast.style.color = isError ? 'var(--danger)' : 'var(--ink)';
      toast.classList.add('show');
      clearTimeout(showToast._timer);
      showToast._timer = setTimeout(() => toast.classList.remove('show'), 2400);
    }

    function escapeHtml(value) {
      return String(value ?? '')
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;');
    }

    function metricWidth(distance) {
      const clamped = Math.max(0, Math.min(4000, Number(distance) || 0));
      return `${(clamped / 4000) * 100}%`;
    }

    function stateClass(statusText) {
      if (/error/i.test(statusText)) return 'state-error';
      if (/blocked|occupied|wait/i.test(statusText)) return 'state-warn';
      return 'state-ready';
    }

    function renderNodes(payload) {
      pageTitle.textContent = payload.title || 'Roode Control';
      pageSubtitle.textContent = payload.subtitle || 'Tune the people counter directly on the device.';

      if (!payload.nodes || !payload.nodes.length) {
        nodeList.innerHTML = '<section class="panel empty">No sensors are exposed to the UI yet.</section>';
        return;
      }

      nodeList.innerHTML = payload.nodes.map((node) => `
        <section class="panel node-card" data-node="${node.index}">
          <div class="node-head">
            <div>
              <p class="eyebrow">Sensor workspace</p>
              <h2>${escapeHtml(node.label)}</h2>
            </div>
            <div class="state-pill ${stateClass(node.status_text)}">${escapeHtml(node.status_text)}</div>
          </div>

          <div class="metrics">
            <article class="metric">
              <header><span>Entry zone</span><span>Threshold ${node.entry_max_threshold} mm</span></header>
              <strong>${node.entry_distance} mm</strong>
              <div class="track"><span style="width:${metricWidth(node.entry_distance)}"></span></div>
            </article>
            <article class="metric">
              <header><span>Exit zone</span><span>Threshold ${node.exit_max_threshold} mm</span></header>
              <strong>${node.exit_distance} mm</strong>
              <div class="track"><span style="width:${metricWidth(node.exit_distance)}"></span></div>
            </article>
          </div>

          <div class="badges">
            <span class="badge ${node.presence ? 'warn' : 'live'}">Presence ${node.presence ? 'detected' : 'clear'}</span>
            <span class="badge ${node.masking ? 'warn' : 'live'}">Masking ${node.masking ? 'yes' : 'no'}</span>
            <span class="badge live">Direction ${escapeHtml(node.last_direction)}</span>
            <span class="badge">Invert ${node.invert_direction ? 'on' : 'off'}</span>
          </div>

          <div class="control-cluster">
            <button class="primary" data-command="recalibrate" data-node="${node.index}" ${node.can_recalibrate ? '' : 'disabled'}>
              Recalibrate now
            </button>
            <button class="secondary" data-command="reset_counter" data-node="${node.index}">Reset count</button>
          </div>

          <div class="counter-box">
            <label>
              People count
              <input type="number" min="0" max="50" step="1" value="${node.people_counter}" data-field="people_counter" data-node="${node.index}">
            </label>
            <div class="status-chip">
              <span>Last direction</span>
              <strong>${escapeHtml(node.last_direction)}</strong>
            </div>
            <button class="secondary" data-command="set_counter" data-node="${node.index}">Save count</button>
          </div>

          <details>
            <summary>
              <span>Calibration and tuning</span>
              <span>${node.auto_recalibration_minutes > 0 ? `Auto every ${node.auto_recalibration_minutes} min` : 'Manual only'}</span>
            </summary>
            <div class="tuning-grid">
              <label>
                Auto recalibration (min)
                <input type="number" min="0" max="720" step="5" value="${node.auto_recalibration_minutes}" data-field="auto_recalibration" data-node="${node.index}">
              </label>
              <label>
                Sampling
                <input type="number" min="1" max="8" step="1" value="${node.sampling}" data-field="sampling" data-node="${node.index}">
              </label>
              <label>
                Invert direction
                <select data-field="invert_direction" data-node="${node.index}">
                  <option value="false" ${node.invert_direction ? '' : 'selected'}>Normal</option>
                  <option value="true" ${node.invert_direction ? 'selected' : ''}>Reversed</option>
                </select>
              </label>
              <label>
                Min threshold (%)
                <input type="number" min="0" max="40" step="1" value="${node.min_threshold_percent}" data-field="min_threshold" data-node="${node.index}">
              </label>
              <label>
                Max threshold (%)
                <input type="number" min="40" max="95" step="1" value="${node.max_threshold_percent}" data-field="max_threshold" data-node="${node.index}">
              </label>
              <label>
                ROI width
                <input type="number" min="4" max="16" step="1" value="${node.roi_width}" data-field="roi_width" data-node="${node.index}">
              </label>
              <label>
                ROI height
                <input type="number" min="4" max="16" step="1" value="${node.roi_height}" data-field="roi_height" data-node="${node.index}">
              </label>
              <label>
                Minutes since calibration
                <input type="number" value="${node.minutes_since_calibration}" disabled>
              </label>
              <label>
                Minutes until auto recalibration
                <input type="number" value="${node.minutes_until_auto_recalibration}" disabled>
              </label>
            </div>
            <div class="control-cluster" style="padding:0 1rem 1rem;">
              <button class="primary" data-command="apply_tuning" data-node="${node.index}">Apply tuning</button>
            </div>
          </details>

          <div class="node-footer">
            <span>${escapeHtml(node.calibration_hint)}</span>
            <span>Status code ${node.sensor_status}</span>
          </div>
        </section>
      `).join('');
    }

    async function fetchState() {
      const response = await fetch(stateUrl, { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return response.json();
    }

    async function postAction(form) {
      const response = await fetch(actionUrl, {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded;charset=UTF-8' },
        body: new URLSearchParams(form),
      });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return response.json();
    }

    async function refresh() {
      try {
        const payload = await fetchState();
        renderNodes(payload);
        connectionPill.textContent = payload.connection_text || 'Connected to device';
        connectionCopy.textContent = 'Online';
        lastSync.textContent = new Date().toLocaleTimeString();
      } catch (error) {
        connectionPill.textContent = 'Connection lost - retrying';
        connectionCopy.textContent = 'Offline';
        showToast('Could not refresh device state', true);
      }
    }

    document.addEventListener('click', async (event) => {
      const button = event.target.closest('button[data-command]');
      if (!button || busy) return;
      const nodeIndex = button.dataset.node;
      const command = button.dataset.command;
      const container = button.closest('[data-node]');
      const fields = container ? container.querySelectorAll('[data-field]') : [];

      const payload = { node: nodeIndex, action: command };
      if (command === 'set_counter') {
        payload.value = container.querySelector('[data-field="people_counter"]').value;
      }
      if (command === 'apply_tuning') {
        fields.forEach((field) => { payload[field.dataset.field] = field.value; });
      }

      busy = true;
      button.disabled = true;
      try {
        const result = await postAction(payload);
        showToast(result.message || 'Updated');
        await refresh();
      } catch (error) {
        showToast('Unable to apply change', true);
      } finally {
        busy = false;
        button.disabled = false;
      }
    });

    refresh();
    setInterval(refresh, 1500);
  </script>
</body>
</html>
)html";

bool parse_bool_arg(const std::string &value) {
  if (value.empty()) {
    return false;
  }
  char first = static_cast<char>(std::tolower(value[0]));
  return first == '1' || first == 't' || first == 'y' || first == 'o';
}

uint32_t parse_u32_arg(const std::string &value, uint32_t fallback) {
  if (value.empty()) {
    return fallback;
  }
  char *end = nullptr;
  unsigned long parsed = std::strtoul(value.c_str(), &end, 10);
  if (end == value.c_str()) {
    return fallback;
  }
  return static_cast<uint32_t>(parsed);
}

int status_code_for(const roode::Roode *node) { return node->get_sensor_status_code(); }

std::string status_text_for(const roode::Roode *node) {
  int status = status_code_for(node);
  if (status > 0) {
    return "Sensor error";
  }
  if (node->is_masking_detected()) {
    return "Blocked for calibration";
  }
  if (node->is_presence_detected()) {
    return "Doorway occupied";
  }
  if (node->is_ready_for_recalibration()) {
    return "Ready to calibrate";
  }
  return "Monitoring";
}

std::string calibration_hint_for(const roode::Roode *node) {
  if (status_code_for(node) > 0) {
    return "Fix sensor communication before tuning.";
  }
  if (node->is_masking_detected()) {
    return "Move away from the sensor and leave the field clear.";
  }
  if (node->is_presence_detected()) {
    return "Wait until the doorway is empty before recalibrating.";
  }
  return "Leave the doorway empty, review the live distances, then recalibrate.";
}

}  // namespace

class RoodeUi::Handler : public AsyncWebHandler {
 public:
  explicit Handler(RoodeUi *parent) : parent_(parent) {}

  bool canHandle(AsyncWebServerRequest *request) const override {
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buf);
    auto method = request->method();
    if (method == HTTP_GET && (url == "/" || url == "/roode-ui/state")) {
      return true;
    }
    if (method == HTTP_POST && url == "/roode-ui/action") {
      return true;
    }
    return false;
  }

  void handleRequest(AsyncWebServerRequest *request) override {
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buf);

    if (request->method() == HTTP_GET && url == "/") {
      this->handle_index_(request);
      return;
    }
    if (request->method() == HTTP_GET && url == "/roode-ui/state") {
      this->handle_state_(request);
      return;
    }
    if (request->method() == HTTP_POST && url == "/roode-ui/action") {
      this->handle_action_(request);
      return;
    }
    request->send(404, "text/plain", "Not found");
  }

 protected:
  void handle_index_(AsyncWebServerRequest *request) {
    auto *response =
        request->beginResponse(200, "text/html; charset=utf-8",
                               reinterpret_cast<const uint8_t *>(ROODE_UI_HTML), sizeof(ROODE_UI_HTML) - 1);
    request->send(response);
  }

  void handle_state_(AsyncWebServerRequest *request) {
    auto json = json::build_json([this](JsonObject root) {
      root["title"] = this->parent_->title_.empty() ? App.get_friendly_name() : this->parent_->title_;
      root["subtitle"] = "Professional calibration, live distances, and field tuning directly on the device.";
      root["connection_text"] = "Connected to device";

      auto nodes = root["nodes"].to<JsonArray>();
      for (size_t index = 0; index < this->parent_->nodes_.size(); index++) {
        auto &binding = this->parent_->nodes_[index];
        auto node = nodes.add<JsonObject>();
        node["index"] = static_cast<int>(index);
        node["label"] = binding.label;
        node["status_text"] = status_text_for(binding.node);
        node["sensor_status"] = status_code_for(binding.node);
        node["entry_distance"] = binding.node->get_entry_distance();
        node["exit_distance"] = binding.node->get_exit_distance();
        node["entry_max_threshold"] = binding.node->get_entry_max_threshold();
        node["exit_max_threshold"] = binding.node->get_exit_max_threshold();
        node["presence"] = binding.node->is_presence_detected();
        node["masking"] = binding.node->is_masking_detected();
        node["can_recalibrate"] = binding.node->is_ready_for_recalibration();
        node["people_counter"] = static_cast<int>(binding.node->get_people_counter_value());
        node["sampling"] = binding.node->get_sampling_size();
        node["min_threshold_percent"] = binding.node->get_min_threshold_percentage();
        node["max_threshold_percent"] = binding.node->get_max_threshold_percentage();
        node["roi_width"] = binding.node->get_roi_width();
        node["roi_height"] = binding.node->get_roi_height();
        node["invert_direction"] = binding.node->get_invert_direction();
        node["auto_recalibration_minutes"] = binding.node->get_auto_recalibration_interval_minutes();
        node["minutes_since_calibration"] = binding.node->get_minutes_since_last_recalibration();
        node["minutes_until_auto_recalibration"] = binding.node->get_auto_recalibration_interval_minutes() == 0
                                                       ? 0
                                                       : binding.node->get_minutes_until_next_recalibration();
        node["last_direction"] = binding.node->get_last_direction();
        node["calibration_hint"] = calibration_hint_for(binding.node);
      }
    });
    request->send(200, "application/json", json.c_str());
  }

  void handle_action_(AsyncWebServerRequest *request) {
    size_t index = parse_u32_arg(request->arg("node"), 0);
    if (index >= this->parent_->nodes_.size()) {
      request->send(404, "application/json", "{\"ok\":false,\"message\":\"Unknown sensor\"}");
      return;
    }

    auto *node = this->parent_->nodes_[index].node;
    auto action = request->arg("action");
    std::string message = "Updated";

    if (action == "recalibrate") {
      node->recalibration();
      message = "Recalibration started";
    } else if (action == "reset_counter") {
      node->reset_people_counter();
      message = "Counter reset";
    } else if (action == "set_counter") {
      node->set_people_counter_value(parse_u32_arg(request->arg("value"), 0));
      message = "Counter saved";
    } else if (action == "apply_tuning") {
      node->set_auto_recalibration_interval_minutes(
          static_cast<uint16_t>(parse_u32_arg(request->arg("auto_recalibration"),
                                              node->get_auto_recalibration_interval_minutes())));
      node->set_sampling_size(
          static_cast<uint8_t>(parse_u32_arg(request->arg("sampling"), node->get_sampling_size())));
      node->set_invert_direction(parse_bool_arg(request->arg("invert_direction")));
      node->set_min_threshold_percentage(
          static_cast<uint8_t>(parse_u32_arg(request->arg("min_threshold"), node->get_min_threshold_percentage())));
      node->set_max_threshold_percentage(
          static_cast<uint8_t>(parse_u32_arg(request->arg("max_threshold"), node->get_max_threshold_percentage())));
      node->set_roi_width(static_cast<uint8_t>(parse_u32_arg(request->arg("roi_width"), node->get_roi_width())));
      node->set_roi_height(static_cast<uint8_t>(parse_u32_arg(request->arg("roi_height"), node->get_roi_height())));
      node->recalibration();
      message = "Tuning applied and recalibration started";
    } else {
      request->send(400, "application/json", "{\"ok\":false,\"message\":\"Unsupported action\"}");
      return;
    }

    auto response = json::build_json([&message](JsonObject root) {
      root["ok"] = true;
      root["message"] = message;
    });
    request->send(200, "application/json", response.c_str());
  }

  RoodeUi *parent_;
};

void RoodeUi::setup() {
  if (this->nodes_.empty()) {
    this->mark_failed();
    ESP_LOGE(TAG, "Roode UI needs at least one Roode node");
    return;
  }

  if (web_server_base::global_web_server_base == nullptr) {
    this->mark_failed();
    ESP_LOGE(TAG, "Roode UI requires web_server");
    return;
  }

  if (this->handler_ == nullptr) {
    this->handler_ = new Handler(this);
  }

  web_server_base::global_web_server_base->add_handler(this->handler_);
  ESP_LOGI(TAG, "Custom Roode UI registered on /");
}

}  // namespace roode_ui
}  // namespace esphome

#endif
