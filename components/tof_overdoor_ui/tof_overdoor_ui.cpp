#include "tof_overdoor_ui.h"

#ifdef USE_WEBSERVER

#include <cmath>
#include <cstdlib>
#include <string>

#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tof_overdoor_ui {

static const char *const TAG = "tof_overdoor_ui";

namespace {

const char *const SENSOR_CARD_LABELS[] = {"U3", "U4", "U7", "U8"};
const char *const SENSOR_GROUP_LABELS[] = {"Independent vote", "Independent vote", "Independent vote", "Independent vote"};

int parse_int_arg(AsyncWebServerRequest *request, const char *name, int fallback) {
  if (!request->hasArg(name)) {
    return fallback;
  }
  auto raw = request->arg(name);
  if (raw.empty()) {
    return fallback;
  }
  char *end = nullptr;
  const long parsed = strtol(raw.c_str(), &end, 10);
  return end != raw.c_str() ? static_cast<int>(parsed) : fallback;
}

bool parse_bool_arg(AsyncWebServerRequest *request, const char *name, bool fallback) {
  if (!request->hasArg(name)) {
    return fallback;
  }
  const auto raw = request->arg(name);
  return raw == "1" || raw == "true" || raw == "on" || raw == "yes";
}

const char OVERDOOR_UI_HTML[] = R"html(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Roode Overdoor Counter</title>
  <style>
    :root {
      color-scheme: light dark;
      --font-display: "Iowan Old Style", "Palatino Linotype", "URW Palladio L", serif;
      --font-body: "Avenir Next", "Segoe UI", "Helvetica Neue", sans-serif;
      --bg: oklch(96.4% 0.008 95);
      --bg-soft: oklch(98.4% 0.004 95);
      --panel: rgba(255, 255, 255, 0.9);
      --panel-strong: rgba(255, 255, 255, 0.97);
      --line: oklch(82% 0.014 245);
      --line-strong: oklch(70% 0.03 245);
      --ink: oklch(27% 0.02 255);
      --ink-soft: oklch(47% 0.02 255);
      --accent: oklch(43% 0.08 243);
      --accent-soft: oklch(92% 0.022 242);
      --success: oklch(58% 0.1 154);
      --success-soft: color-mix(in oklch, var(--success) 16%, white);
      --warn: oklch(71% 0.12 85);
      --warn-soft: color-mix(in oklch, var(--warn) 18%, white);
      --danger: oklch(58% 0.13 28);
      --danger-soft: color-mix(in oklch, var(--danger) 16%, white);
      --shadow: 0 28px 70px rgba(24, 32, 45, 0.08);
      --radius-xl: 30px;
      --radius-lg: 24px;
      --radius-md: 16px;
      --radius-sm: 12px;
    }

    [data-theme="dark"] {
      --bg: oklch(20% 0.01 255);
      --bg-soft: oklch(23.5% 0.012 255);
      --panel: rgba(20, 27, 38, 0.9);
      --panel-strong: rgba(25, 33, 45, 0.97);
      --line: oklch(35% 0.012 255);
      --line-strong: oklch(52% 0.025 242);
      --ink: oklch(95% 0.008 95);
      --ink-soft: oklch(74% 0.018 255);
      --accent: oklch(73% 0.07 240);
      --accent-soft: rgba(72, 103, 170, 0.16);
      --success-soft: color-mix(in oklch, var(--success) 22%, transparent);
      --warn-soft: color-mix(in oklch, var(--warn) 22%, transparent);
      --danger-soft: color-mix(in oklch, var(--danger) 18%, transparent);
      --shadow: 0 26px 80px rgba(0, 0, 0, 0.3);
    }

    * { box-sizing: border-box; }

    body {
      margin: 0;
      min-height: 100vh;
      font-family: var(--font-body);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, color-mix(in oklch, var(--accent) 10%, transparent), transparent 32%),
        linear-gradient(180deg, var(--bg-soft), var(--bg));
    }

    h1, h2, h3 {
      margin: 0;
      font-family: var(--font-display);
      letter-spacing: -0.03em;
      font-weight: 600;
    }

    h1 {
      font-size: clamp(2.4rem, 4vw, 4rem);
      line-height: 0.96;
    }

    h2 { font-size: clamp(1.45rem, 2vw, 1.95rem); }
    h3 { font-size: 1.05rem; }
    p { margin: 0; }

    .shell {
      width: min(1380px, calc(100vw - 2rem));
      margin: 0 auto;
      padding: 1.15rem 0 2.2rem;
    }

    .panel {
      border: 1px solid var(--line);
      border-radius: var(--radius-lg);
      background: var(--panel-strong);
      box-shadow: var(--shadow);
      backdrop-filter: blur(10px);
    }

    .eyebrow {
      margin-bottom: 0.4rem;
      letter-spacing: 0.12em;
      text-transform: uppercase;
      font-size: 0.73rem;
      color: var(--ink-soft);
    }

    .masthead {
      display: grid;
      grid-template-columns: minmax(0, 1.25fr) minmax(300px, 0.75fr);
      gap: 1rem;
      margin-bottom: 1rem;
    }

    .hero {
      padding: 1.3rem 1.45rem 1.45rem;
    }

    .hero-copy {
      margin-top: 0.72rem;
      color: var(--ink-soft);
      line-height: 1.58;
      font-size: 1.02rem;
      max-width: 64ch;
    }

    .hero-grid {
      margin-top: 1rem;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 0.8rem;
    }

    .hero-tip {
      padding: 0.95rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 78%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .hero-tip strong {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      width: 1.55rem;
      height: 1.55rem;
      border-radius: 999px;
      background: var(--accent-soft);
      color: var(--accent);
      font-size: 0.82rem;
      margin-bottom: 0.45rem;
    }

    .hero-tip p {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.92rem;
    }

    .aside {
      display: grid;
      gap: 0.9rem;
    }

    .theme-toggle {
      appearance: none;
      border: 1px solid var(--line);
      border-radius: 999px;
      background: var(--panel-strong);
      color: var(--ink);
      box-shadow: var(--shadow);
      padding: 0.88rem 1.1rem;
      font: inherit;
      font-weight: 700;
      cursor: pointer;
    }

    .aside-card {
      padding: 1rem;
      display: grid;
      gap: 0.85rem;
    }

    .status-strip {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.8rem;
    }

    .status-chip {
      padding: 0.92rem 0.98rem;
      border-radius: var(--radius-md);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .status-chip span {
      display: block;
      margin-bottom: 0.32rem;
      font-size: 0.74rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
    }

    .status-chip strong {
      display: block;
      font-size: 1rem;
      font-weight: 800;
    }

    .aside-note {
      padding: 0.95rem 1rem;
      border-radius: var(--radius-md);
      background: var(--accent-soft);
      color: var(--ink);
    }

    .aside-note strong {
      display: block;
      margin-bottom: 0.25rem;
      font-size: 0.95rem;
    }

    .aside-note p {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.93rem;
    }

    .connection {
      display: inline-flex;
      align-items: center;
      gap: 0.55rem;
      padding: 0.72rem 0.95rem;
      border-radius: 999px;
      background: var(--accent-soft);
      color: var(--accent);
      font-weight: 700;
      margin-bottom: 1rem;
    }

    .connection::before {
      content: "";
      width: 0.65rem;
      height: 0.65rem;
      border-radius: 999px;
      background: currentColor;
      box-shadow: 0 0 0 0.26rem color-mix(in oklch, currentColor 18%, transparent);
    }

    .section { padding: 1.2rem; }
    .stack { display: grid; gap: 1rem; }

    .main-grid {
      display: grid;
      grid-template-columns: minmax(0, 1.25fr) minmax(330px, 0.75fr);
      gap: 1rem;
    }

    .metrics-grid {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 0.8rem;
      margin-top: 1rem;
    }

    .metric {
      padding: 0.95rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 78%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .metric span {
      display: block;
      margin-bottom: 0.3rem;
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
    }

    .metric strong {
      display: block;
      font-size: clamp(1.35rem, 2vw, 2rem);
      line-height: 1;
      margin-bottom: 0.32rem;
    }

    .metric p {
      color: var(--ink-soft);
      line-height: 1.4;
      font-size: 0.86rem;
    }

    .banner {
      margin-top: 1rem;
      display: grid;
      gap: 0.4rem;
      padding: 1rem 1.05rem;
      border-radius: var(--radius-md);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      background: color-mix(in oklch, var(--panel-strong) 80%, var(--bg-soft));
    }

    .pill {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      width: fit-content;
      padding: 0.48rem 0.82rem;
      border-radius: 999px;
      font-size: 0.8rem;
      font-weight: 800;
      letter-spacing: 0.03em;
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      background: var(--bg-soft);
      color: var(--ink-soft);
    }

    .pill.ready { background: var(--success-soft); color: var(--success); }
    .pill.warn { background: var(--warn-soft); color: color-mix(in oklch, var(--warn) 74%, black); }
    .pill.error { background: var(--danger-soft); color: var(--danger); }
    .pill.accent { background: var(--accent-soft); color: var(--accent); }

    .banner p {
      color: var(--ink-soft);
      line-height: 1.5;
    }

    .actions {
      display: flex;
      flex-wrap: wrap;
      gap: 0.75rem;
      margin-top: 1rem;
    }

    button {
      appearance: none;
      border: 0;
      border-radius: 999px;
      padding: 0.9rem 1.1rem;
      font: inherit;
      font-weight: 800;
      cursor: pointer;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 0.45rem;
      transition: transform 0.18s ease, opacity 0.18s ease;
    }

    button:hover { transform: translateY(-1px); }
    button:disabled { opacity: 0.55; cursor: wait; transform: none; }

    .primary { color: white; background: linear-gradient(135deg, color-mix(in oklch, var(--accent) 72%, black), var(--accent)); }
    .secondary {
      color: var(--ink);
      background: color-mix(in oklch, var(--panel-strong) 78%, var(--bg-soft));
      border: 1px solid var(--line);
    }
    .danger {
      color: var(--danger);
      background: var(--danger-soft);
      border: 1px solid color-mix(in oklch, var(--danger) 24%, var(--line));
    }

    .door-grid,
    .sensor-grid,
    .diag-grid {
      display: grid;
      gap: 0.8rem;
    }

    .door-grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
      margin-top: 1rem;
    }

    .group-card,
    .sensor-card,
    .diag-card,
    .settings-card,
    .chart-card {
      padding: 1rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 82%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .group-card.active {
      border-color: color-mix(in oklch, var(--accent) 28%, var(--line));
      background: color-mix(in oklch, var(--accent) 8%, var(--bg-soft));
    }

    .group-head,
    .sensor-head,
    .diag-head {
      display: flex;
      justify-content: space-between;
      gap: 0.8rem;
      align-items: start;
      margin-bottom: 0.7rem;
    }

    .group-head p,
    .sensor-copy,
    .diag-card p,
    .settings-copy {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.92rem;
    }

    .mini-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 0.7rem;
    }

    .mini {
      padding: 0.82rem 0.88rem;
      border-radius: var(--radius-sm);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
    }

    .mini span {
      display: block;
      margin-bottom: 0.26rem;
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
    }

    .mini strong {
      display: block;
      font-size: 1.08rem;
      margin-bottom: 0.2rem;
    }

    .mini p {
      color: var(--ink-soft);
      line-height: 1.35;
      font-size: 0.84rem;
    }

    .bar {
      width: 100%;
      height: 0.68rem;
      border-radius: 999px;
      overflow: hidden;
      background: color-mix(in oklch, var(--line) 75%, transparent);
      margin: 0.65rem 0 0.55rem;
    }

    .bar > span {
      display: block;
      height: 100%;
      width: 0;
      border-radius: inherit;
      background: linear-gradient(90deg, color-mix(in oklch, var(--accent) 62%, white), var(--accent));
      transition: width 0.22s ease;
    }

    .sensor-grid {
      grid-template-columns: repeat(2, minmax(0, 1fr));
      margin-top: 1rem;
    }

    .sensor-card.active {
      border-color: color-mix(in oklch, var(--accent) 28%, var(--line));
      background: color-mix(in oklch, var(--accent) 7%, var(--bg-soft));
    }

    .sensor-meta {
      display: grid;
      gap: 0.12rem;
    }

    .sensor-meta span {
      color: var(--ink-soft);
      font-size: 0.83rem;
    }

    .sensor-distance {
      display: flex;
      justify-content: space-between;
      gap: 0.8rem;
      align-items: baseline;
    }

    .sensor-distance strong {
      font-size: clamp(1.5rem, 2vw, 2rem);
      line-height: 1;
    }

    .sensor-distance span {
      color: var(--ink-soft);
      font-size: 0.9rem;
    }

    .sensor-detail-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 0.55rem;
    }

    .sensor-detail {
      padding: 0.72rem 0.78rem;
      border-radius: var(--radius-sm);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
    }

    .sensor-detail span {
      display: block;
      margin-bottom: 0.22rem;
      font-size: 0.69rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
    }

    .sensor-detail strong { font-size: 0.98rem; }

    .chart-card {
      margin-top: 1rem;
    }

    .chart-shell {
      margin-top: 0.85rem;
      border-radius: var(--radius-md);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      background: var(--bg-soft);
      padding: 0.9rem;
    }

    .chart-stage {
      width: 100%;
      aspect-ratio: 2.3 / 1;
    }

    .chart-legend {
      margin-top: 0.8rem;
      display: flex;
      flex-wrap: wrap;
      gap: 0.55rem 0.8rem;
    }

    .legend-item {
      display: inline-flex;
      align-items: center;
      gap: 0.45rem;
      color: var(--ink-soft);
      font-size: 0.86rem;
    }

    .legend-swatch {
      width: 0.9rem;
      height: 0.26rem;
      border-radius: 999px;
    }

    .diag-grid { margin-top: 1rem; }

    .diag-card pre {
      margin: 0;
      white-space: pre-wrap;
      word-break: break-word;
      font: inherit;
      color: var(--ink-soft);
      line-height: 1.5;
    }

    .settings-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.7rem;
      margin-top: 0.8rem;
    }

    label.field {
      display: grid;
      gap: 0.28rem;
      font-size: 0.86rem;
      color: var(--ink-soft);
    }

    label.field span {
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }

    input[type="number"] {
      width: 100%;
      padding: 0.82rem 0.88rem;
      border-radius: var(--radius-sm);
      border: 1px solid var(--line);
      background: var(--bg-soft);
      color: var(--ink);
      font: inherit;
      font-weight: 700;
    }

    .toggle-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.7rem;
      margin-top: 0.7rem;
    }

    .toggle {
      display: flex;
      align-items: center;
      justify-content: space-between;
      gap: 0.8rem;
      padding: 0.82rem 0.88rem;
      border-radius: var(--radius-sm);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
      color: var(--ink);
      font-size: 0.9rem;
    }

    .toggle input { inline-size: 1.1rem; block-size: 1.1rem; }

    .settings-actions {
      display: flex;
      flex-wrap: wrap;
      gap: 0.7rem;
      margin-top: 0.85rem;
    }

    .toast {
      position: fixed;
      right: 1rem;
      bottom: 1rem;
      padding: 0.95rem 1.1rem;
      border-radius: 1rem;
      background: var(--panel-strong);
      border: 1px solid var(--line);
      color: var(--ink);
      box-shadow: var(--shadow);
      opacity: 0;
      transform: translateY(8px);
      pointer-events: none;
      transition: opacity 0.2s ease, transform 0.2s ease;
    }

    .toast.show { opacity: 1; transform: translateY(0); }

    @media (max-width: 1180px) {
      .masthead,
      .main-grid,
      .metrics-grid,
      .hero-grid {
        grid-template-columns: 1fr;
      }
    }

    @media (max-width: 920px) {
      .door-grid,
      .sensor-grid,
      .settings-grid,
      .toggle-grid,
      .mini-grid,
      .sensor-detail-grid,
      .status-strip {
        grid-template-columns: 1fr;
      }

      .group-head,
      .sensor-head,
      .diag-head {
        flex-direction: column;
        align-items: start;
      }
    }
  </style>
</head>
<body>
  <div class="shell">
    <div class="masthead">
      <section class="panel hero">
        <p class="eyebrow">Roode service console</p>
        <h1 id="page-title">Loading counter</h1>
        <p class="hero-copy" id="page-subtitle">Preparing live sensor data, detection state, and calibration information.</p>
        <div class="hero-grid">
          <article class="hero-tip">
            <strong>1</strong>
            <p>Calibrate only when the doorway is empty and stable for a few seconds.</p>
          </article>
          <article class="hero-tip">
            <strong>2</strong>
            <p>Each sensor now runs its own two-zone Roode path and votes IN or OUT.</p>
          </article>
          <article class="hero-tip">
            <strong>3</strong>
            <p>A clean count needs at least two sensors to vote the same direction.</p>
          </article>
        </div>
      </section>

      <aside class="aside">
        <button class="theme-toggle" type="button" id="theme-toggle">Switch to dark mode</button>
        <section class="panel aside-card">
          <div class="status-strip">
            <div class="status-chip">
              <span>Connection</span>
              <strong id="connection-copy">Connecting</strong>
            </div>
            <div class="status-chip">
              <span>Last sync</span>
              <strong id="last-sync">Waiting</strong>
            </div>
          </div>
          <div class="aside-note">
            <strong>What to watch first</strong>
            <p>Start with the event reason, then the vote zones and per-sensor cards if one sensor looks suspicious or noisy.</p>
          </div>
        </section>
      </aside>
    </div>

    <div class="connection" id="connection-pill">Connecting to device</div>

    <section class="panel section">
      <p class="eyebrow">System overview</p>
      <h2 id="counter-label">Front door counter</h2>
      <div class="metrics-grid">
        <article class="metric">
          <span>Sensor ready</span>
          <strong id="metric-ready">-</strong>
          <p id="metric-ready-copy">Waiting for state.</p>
        </article>
        <article class="metric">
          <span>System status</span>
          <strong id="metric-system-status">-</strong>
          <p id="metric-system-copy">Current state machine state.</p>
        </article>
        <article class="metric">
          <span>People inside</span>
          <strong id="metric-people">0</strong>
          <p>Net confirmed count stored on the ESP.</p>
        </article>
        <article class="metric">
          <span>Last direction</span>
          <strong id="metric-direction">Waiting</strong>
          <p id="metric-direction-copy">No confirmed pass yet.</p>
        </article>
        <article class="metric">
          <span>Confirmed IN</span>
          <strong id="metric-entry">0</strong>
          <p>Number of confirmed entries.</p>
        </article>
        <article class="metric">
          <span>Confirmed OUT</span>
          <strong id="metric-exit">0</strong>
          <p>Number of confirmed exits.</p>
        </article>
        <article class="metric">
          <span>Unsure IN / OUT</span>
          <strong id="metric-unsure">0 / 0</strong>
          <p>Events that did not have enough agreement to count.</p>
        </article>
        <article class="metric">
          <span>Last confidence</span>
          <strong id="metric-confidence">0%</strong>
          <p>Internal confidence for the most recent recorded event.</p>
        </article>
      </div>

      <div class="banner">
        <div class="pill accent" id="phase-pill">Monitoring</div>
        <h3 id="phase-title">Waiting for live data</h3>
        <p id="phase-copy">The counter will describe what it is doing right now, and why it decided to count, defer, or reject a pass.</p>
      </div>

      <div class="actions">
        <button class="primary" data-command="recalibrate" data-confirm="Start a new calibration now? Make sure the doorway is empty first.">Calibrate sensor</button>
        <button class="secondary" data-command="rediscover" data-confirm="Rediscover sensors now? This briefly interrupts live counting.">Rediscover sensors</button>
        <button class="secondary" data-command="reset_counts" data-confirm="Reset People Inside, confirmed IN, and confirmed OUT?">Reset count</button>
        <button class="secondary" data-command="reset_unsure_in" data-confirm="Reset Unsure Detection IN?">Reset unsure IN</button>
        <button class="secondary" data-command="reset_unsure_out" data-confirm="Reset Unsure Detection OUT?">Reset unsure OUT</button>
        <button class="danger" data-command="reset_all_counters" data-confirm="Reset all counters, including unsure events?">Reset all counters</button>
        <button class="danger" data-command="restart" data-confirm="Restart the ESP now?">Restart ESP</button>
      </div>
    </section>

    <div class="main-grid">
      <div class="stack">
        <section class="panel section">
          <div class="diag-head">
            <div>
              <p class="eyebrow">Live doorway view</p>
              <h2>OUT and IN vote zones</h2>
            </div>
            <div class="pill" id="standing-pill">Doorway clear</div>
          </div>
          <p class="settings-copy" id="doorway-copy">Every physical sensor alternates between an OUT ROI and an IN ROI. Two matching sensor votes are required before the counter increments.</p>
          <div class="door-grid" id="group-grid"></div>
        </section>

        <section class="panel section chart-card">
          <div class="diag-head">
            <div>
              <p class="eyebrow">Live detection graph</p>
              <h2>Distance history</h2>
            </div>
            <div class="pill" id="graph-scale-pill">Live</div>
          </div>
          <p class="settings-copy">This graph makes it easier to see who moved first, whether one sensor missed the pass, and whether the empty-doorway baseline is drifting.</p>
          <div class="chart-shell">
            <svg class="chart-stage" id="history-chart" viewBox="0 0 1000 420" preserveAspectRatio="none"></svg>
            <div class="chart-legend" id="chart-legend"></div>
          </div>
        </section>

        <section class="panel section">
          <p class="eyebrow">Individual sensors</p>
          <h2>Live sensor health</h2>
          <div class="sensor-grid" id="sensor-grid"></div>
        </section>
      </div>

      <div class="stack">
        <section class="panel section">
          <p class="eyebrow">Calibration</p>
          <h2>Calibration health</h2>
          <div class="diag-grid">
            <article class="diag-card">
              <strong>Progress</strong>
              <p id="calibration-progress-copy">Waiting for state.</p>
            </article>
            <article class="diag-card">
              <strong>Blocked sensor</strong>
              <p id="blocked-copy">None</p>
            </article>
            <article class="diag-card">
              <strong>Last detection time</strong>
              <p id="last-detection-copy">Never</p>
            </article>
            <article class="diag-card">
              <strong>Last decision</strong>
              <p id="last-reason-copy">No decision logged yet.</p>
            </article>
          </div>
        </section>

        <section class="panel section settings-card">
          <p class="eyebrow">Settings</p>
          <h2>Detection tuning</h2>
          <p class="settings-copy">These values are stored on the ESP. Start with small changes, then test a few clean passes before changing more than one setting at a time.</p>
          <form id="settings-form">
            <div class="settings-grid">
              <label class="field">
                <span>Trigger threshold (mm)</span>
                <input type="number" name="trigger_threshold" id="setting-trigger" min="80" max="1200" step="10" value="320">
              </label>
              <label class="field">
                <span>Clear threshold (mm)</span>
                <input type="number" name="clear_threshold" id="setting-clear" min="40" max="900" step="10" value="180">
              </label>
              <label class="field">
                <span>Baseline tolerance (mm)</span>
                <input type="number" name="baseline_tolerance" id="setting-baseline" min="20" max="300" step="5" value="80">
              </label>
              <label class="field">
                <span>Debounce (ms)</span>
                <input type="number" name="debounce_ms" id="setting-debounce" min="5" max="300" step="5" value="45">
              </label>
              <label class="field">
                <span>Detection timeout (ms)</span>
                <input type="number" name="detection_timeout_ms" id="setting-timeout" min="300" max="4000" step="50" value="1600">
              </label>
              <label class="field">
                <span>Cooldown (ms)</span>
                <input type="number" name="cooldown_ms" id="setting-cooldown" min="0" max="3000" step="50" value="500">
              </label>
              <label class="field">
                <span>Min valid sensors</span>
                <input type="number" name="min_valid_sensors" id="setting-min-valid" min="2" max="4" step="1" value="3">
              </label>
              <label class="field">
                <span>Max people inside</span>
                <input type="number" name="max_people_inside" id="setting-max-people" min="1" max="500" step="1" value="50">
              </label>
            </div>
            <div class="toggle-grid">
              <label class="toggle">
                <span>Invert direction</span>
                <input type="checkbox" name="invert_direction" id="setting-invert">
              </label>
              <label class="toggle">
                <span>Auto save enabled</span>
                <input type="checkbox" name="auto_save_enabled" id="setting-autosave">
              </label>
            </div>
            <div class="settings-actions">
              <button class="primary" type="submit">Save settings</button>
            </div>
          </form>
        </section>

        <section class="panel section">
          <p class="eyebrow">Debug</p>
          <h2>Reasoning and event log</h2>
          <div class="diag-grid">
            <article class="diag-card">
              <strong>Discovery map</strong>
              <p id="discovery-map-copy">Waiting for state.</p>
            </article>
            <article class="diag-card">
              <strong>Sync timing</strong>
              <p id="timing-copy">Waiting for state.</p>
            </article>
            <article class="diag-card">
              <strong>Summary</strong>
              <p id="summary-copy">Waiting for state.</p>
            </article>
            <article class="diag-card">
              <strong>Recent events</strong>
              <pre id="event-log-copy">No events yet.</pre>
            </article>
          </div>
        </section>
      </div>
    </div>
  </div>

  <div class="toast" id="toast"></div>

  <script>
    const stateUrl = '/tof-overdoor-ui/state';
    const actionUrl = '/tof-overdoor-ui/action';
    const pageTitle = document.getElementById('page-title');
    const pageSubtitle = document.getElementById('page-subtitle');
    const counterLabel = document.getElementById('counter-label');
    const connectionPill = document.getElementById('connection-pill');
    const connectionCopy = document.getElementById('connection-copy');
    const lastSync = document.getElementById('last-sync');
    const themeToggle = document.getElementById('theme-toggle');
    const toast = document.getElementById('toast');
    const groupGrid = document.getElementById('group-grid');
    const sensorGrid = document.getElementById('sensor-grid');
    const historyChart = document.getElementById('history-chart');
    const chartLegend = document.getElementById('chart-legend');
    const settingsForm = document.getElementById('settings-form');

    const chartColors = ['#2b5876', '#357266', '#a86d32', '#8a3d4b'];
    const chartHistory = [[], [], [], []];
    const historyLimit = 90;
    let refreshTimer = null;
    let busy = false;
    let lastOnline = true;
    let settingsDirty = false;

    const rememberedTheme = localStorage.getItem('roode-overdoor-theme');
    if (rememberedTheme) {
      document.documentElement.dataset.theme = rememberedTheme;
    }

    function updateThemeLabel() {
      const dark = document.documentElement.dataset.theme === 'dark';
      themeToggle.textContent = dark ? 'Switch to light mode' : 'Switch to dark mode';
    }

    updateThemeLabel();

    themeToggle.addEventListener('click', () => {
      const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
      document.documentElement.dataset.theme = next;
      localStorage.setItem('roode-overdoor-theme', next);
      updateThemeLabel();
    });

    function showToast(message, isError = false) {
      toast.textContent = message;
      toast.style.color = isError ? 'var(--danger)' : 'var(--ink)';
      toast.classList.add('show');
      clearTimeout(showToast._timer);
      showToast._timer = setTimeout(() => toast.classList.remove('show'), 2600);
    }

    function escapeHtml(value) {
      return String(value ?? '')
        .replaceAll('&', '&amp;')
        .replaceAll('<', '&lt;')
        .replaceAll('>', '&gt;')
        .replaceAll('"', '&quot;');
    }

    function formatMm(value) {
      const num = Number(value);
      if (!Number.isFinite(num)) return 'N/A';
      return `${Math.round(num)} mm`;
    }

    function formatInt(value) {
      const num = Number(value);
      if (!Number.isFinite(num)) return '0';
      return `${Math.round(num)}`;
    }

    function formatPercent(value) {
      const num = Number(value);
      if (!Number.isFinite(num)) return '0%';
      return `${Math.round(num)}%`;
    }

    function chipClass(status) {
      const text = String(status || '').toLowerCase();
      if (text.includes('error') || text.includes('missing')) return 'pill error';
      if (text.includes('blocked') || text.includes('warning') || text.includes('degraded') || text.includes('unsure')) return 'pill warn';
      if (text.includes('detect') || text.includes('trigger')) return 'pill accent';
      return 'pill ready';
    }

    function sensorCopy(sensor) {
      const health = String(sensor.health || '');
      if (sensor.status === 'Blocked') {
        return 'This sensor has stayed triggered longer than the blocked timeout and needs a clear doorway before the next clean pass.';
      }
      if (sensor.status && String(sensor.status).startsWith('Read error')) {
        return 'This sensor is reporting read errors. Watch whether the raw and filtered values recover or stay unstable.';
      }
      if (sensor.active) {
        return `This sensor currently contributes to an active ${sensor.group}.`;
      }
      if (health === 'Warning') {
        return 'This sensor is online, but it has recently looked noisy or borderline compared with its baseline.';
      }
      if (health === 'Error') {
        return 'This sensor is missing, stale, or has too many consecutive read errors right now.';
      }
      return 'This sensor currently looks clear and stable.';
    }

    function directionCopy(direction) {
      const text = String(direction || '').toUpperCase();
      if (text === 'IN') return 'A clean pass was recorded as entering the room.';
      if (text === 'OUT') return 'A clean pass was recorded as leaving the room.';
      if (text === 'UNSURE_IN') return 'The device saw an IN-like sequence, but not enough sensors agreed to count it.';
      if (text === 'UNSURE_OUT') return 'The device saw an OUT-like sequence, but not enough sensors agreed to count it.';
      if (text === 'RESET') return 'Counters were reset manually.';
      if (text === 'CANCELLED') return 'The last movement pattern was intentionally rejected as not trustworthy enough.';
      return 'No recorded event yet.';
    }

    function phaseCopy(state) {
      const phase = String(state.phase || '').toLowerCase();
      if (state.system_status === 'Calibrating') {
        return 'Calibration is running now. Keep the doorway empty and stable until the progress reaches 100%.';
      }
      if (state.system_status === 'Blocked') {
        return 'At least one sensor has stayed active too long, or someone is standing in the doorway. The counter waits for the opening to clear before it trusts a new event.';
      }
      if (phase.includes('waiting for 2 sensors')) {
        return 'One or more sensors has produced a direction vote. The counter is waiting for a second matching vote before it counts.';
      }
      if (phase.includes('per-sensor')) {
        return 'Each physical sensor is watching its own OUT and IN zones and will vote only after a complete Roode path.';
      }
      if (phase.includes('cooldown')) {
        return 'A short cooldown is active to keep one pass from being counted twice.';
      }
      return String(state.last_reason || 'The device is ready for a clean pass.');
    }

    function groupDescription(group, active) {
      if (active) {
        return `${group.label} currently sees something closer than the trigger threshold.`;
      }
      return `${group.label} looks clear right now.`;
    }

    function updateMetric(id, value) {
      const node = document.getElementById(id);
      if (node) node.textContent = value;
    }

    function setCheckbox(id, value) {
      const node = document.getElementById(id);
      if (node) node.checked = !!value;
    }

    function setNumber(id, value) {
      const node = document.getElementById(id);
      if (node && Number.isFinite(Number(value))) {
        node.value = Math.round(Number(value));
      }
    }

    function shouldSyncSettingsFromState(force = false) {
      if (force) return true;
      if (settingsDirty) return false;
      return !settingsForm.contains(document.activeElement);
    }

    function updateHistory(state) {
      (state.sensors || []).forEach((sensor, index) => {
        if (!chartHistory[index]) return;
        const value = Number(sensor.distance);
        if (Number.isFinite(value)) {
          chartHistory[index].push(value);
          if (chartHistory[index].length > historyLimit) {
            chartHistory[index].shift();
          }
        }
      });
    }

    function renderChart(state) {
      updateHistory(state);
      const series = chartHistory.filter((points) => points.length > 1);
      if (!series.length) {
        historyChart.innerHTML = '';
        chartLegend.innerHTML = '';
        return;
      }

      let min = Infinity;
      let max = -Infinity;
      chartHistory.forEach((points) => {
        points.forEach((value) => {
          min = Math.min(min, value);
          max = Math.max(max, value);
        });
      });
      if (!Number.isFinite(min) || !Number.isFinite(max)) return;
      if (max - min < 120) {
        max += 60;
        min -= 60;
      }

      const width = 1000;
      const height = 420;
      const left = 36;
      const right = 26;
      const top = 22;
      const bottom = 36;
      const chartWidth = width - left - right;
      const chartHeight = height - top - bottom;

      const gridLines = [0, 0.25, 0.5, 0.75, 1].map((ratio) => {
        const y = top + chartHeight * ratio;
        const value = Math.round(max - ((max - min) * ratio));
        return `
          <line x1="${left}" y1="${y}" x2="${width - right}" y2="${y}" stroke="rgba(120,132,148,0.18)" stroke-width="1"/>
          <text x="0" y="${y + 4}" fill="currentColor" opacity="0.55" font-size="16">${value} mm</text>
        `;
      }).join('');

      const lines = chartHistory.map((points, index) => {
        if (points.length < 2) return '';
        const step = chartWidth / Math.max(1, historyLimit - 1);
        const pointsAttr = points.map((value, pointIndex) => {
          const x = left + step * pointIndex;
          const y = top + chartHeight * (1 - ((value - min) / (max - min)));
          return `${x},${y}`;
        }).join(' ');
        return `<polyline fill="none" stroke="${chartColors[index]}" stroke-width="3" stroke-linecap="round" stroke-linejoin="round" points="${pointsAttr}"/>`;
      }).join('');

      historyChart.innerHTML = `
        <rect x="${left}" y="${top}" width="${chartWidth}" height="${chartHeight}" rx="18" fill="none" stroke="rgba(120,132,148,0.22)" stroke-width="1.5"/>
        ${gridLines}
        ${lines}
      `;

      chartLegend.innerHTML = (state.sensors || []).map((sensor, index) => `
        <span class="legend-item">
          <span class="legend-swatch" style="background:${chartColors[index]};"></span>
          <span>${escapeHtml(sensor.label)} · ${escapeHtml(sensor.group)}</span>
        </span>
      `).join('');

      updateMetric('graph-scale-pill', `${Math.round(min)}-${Math.round(max)} mm`);
    }

    function renderGroups(state) {
      groupGrid.innerHTML = (state.groups || []).map((group) => `
        <article class="group-card ${group.active ? 'active' : ''}">
          <div class="group-head">
            <div>
              <h3>${escapeHtml(group.label)}</h3>
              <p>${escapeHtml(group.description)}</p>
            </div>
            <div class="${chipClass(group.active ? 'detecting' : group.health)}">${group.active ? 'Triggered' : escapeHtml(group.health || 'Ready')}</div>
          </div>
          <div class="mini-grid">
            <div class="mini">
              <span>Nearest distance</span>
              <strong>${formatMm(group.distance)}</strong>
              <p>Closest live reading within this group.</p>
            </div>
            <div class="mini">
              <span>Baseline</span>
              <strong>${formatMm(group.baseline)}</strong>
              <p>Average empty-doorway reference for this group.</p>
            </div>
            <div class="mini">
              <span>Drop from baseline</span>
              <strong>${formatMm(group.drop)}</strong>
              <p>${escapeHtml(groupDescription(group, group.active))}</p>
            </div>
          </div>
        </article>
      `).join('');
    }

    function renderSensors(state) {
      sensorGrid.innerHTML = (state.sensors || []).map((sensor) => `
        <article class="sensor-card ${sensor.active ? 'active' : ''}">
          <div class="sensor-head">
            <div class="sensor-meta">
              <strong>${escapeHtml(sensor.label)}</strong>
              <span>${escapeHtml(sensor.group)} · ${escapeHtml(sensor.source)}</span>
            </div>
            <div class="${chipClass(sensor.status)}">${escapeHtml(sensor.status)}</div>
          </div>
          <div class="sensor-distance">
            <strong>${formatMm(sensor.distance)}</strong>
            <span>Baseline ${formatMm(sensor.baseline)}</span>
          </div>
          <div class="bar"><span style="width:${Math.max(0, Math.min(100, Number(sensor.distance || 0) / 25))}%"></span></div>
          <div class="sensor-detail-grid">
            <div class="sensor-detail">
              <span>Drop</span>
              <strong>${formatMm(sensor.drop)}</strong>
            </div>
            <div class="sensor-detail">
              <span>Noise</span>
              <strong>${formatMm(sensor.noise)}</strong>
            </div>
            <div class="sensor-detail">
              <span>Cal quality</span>
              <strong>${formatPercent(sensor.quality)}</strong>
            </div>
          </div>
          <p class="sensor-copy">${escapeHtml(sensorCopy(sensor))}</p>
        </article>
      `).join('');
    }

    function renderState(state, options = {}) {
      const forceSettingsSync = !!options.forceSettingsSync;
      pageTitle.textContent = state.title || 'Roode Overdoor Counter';
      pageSubtitle.textContent = state.subtitle || 'Local ESP-based counting with four ToF sensors.';
      counterLabel.textContent = state.label || 'Front door counter';

      updateMetric('metric-ready', state.ready ? 'Yes' : 'No');
      updateMetric('metric-ready-copy', state.ready
        ? 'Baselines are valid and enough healthy sensors are reporting.'
        : 'Still calibrating, rediscovering, or waiting for enough healthy sensors.');
      updateMetric('metric-system-status', state.system_status || 'Booting');
      updateMetric('metric-system-copy', state.person_standing
        ? 'Someone appears to be standing in the doorway.'
        : 'Live state for the local counter state machine.');
      updateMetric('metric-people', formatInt(state.people_count));
      updateMetric('metric-direction', state.last_direction || 'Waiting');
      updateMetric('metric-direction-copy', directionCopy(state.last_direction));
      updateMetric('metric-entry', formatInt(state.entry_count));
      updateMetric('metric-exit', formatInt(state.exit_count));
      updateMetric('metric-unsure', `${formatInt(state.unsure_in_count)} / ${formatInt(state.unsure_out_count)}`);
      updateMetric('metric-confidence', formatPercent(state.last_confidence));

      const phasePill = document.getElementById('phase-pill');
      phasePill.className = chipClass(state.system_status);
      phasePill.textContent = state.phase || state.system_status || 'Monitoring';
      updateMetric('phase-title', state.system_status === 'Ready'
        ? 'Counter is ready for clean passes'
        : (state.system_status || 'Monitoring'));
      updateMetric('phase-copy', phaseCopy(state));

      const standingPill = document.getElementById('standing-pill');
      standingPill.className = chipClass(state.person_standing ? 'blocked' : 'ready');
      standingPill.textContent = state.person_standing ? 'Person standing in door' : 'Doorway clear';

      updateMetric('calibration-progress-copy', `${formatPercent(state.calibration_progress)} complete. Last detection at ${state.last_detection_time || 'Never'}.`);
      updateMetric('blocked-copy', state.blocked_sensor || 'None');
      updateMetric('last-detection-copy', state.last_detection_time || 'Never');
      updateMetric('last-reason-copy', state.last_reason || 'No decision logged yet.');
      updateMetric('discovery-map-copy', state.discovery_map || 'No discovery data yet.');
      updateMetric('timing-copy', `${formatInt(state.reporting_sensors)} of ${formatInt(state.discovered_sensors)} sensors reporting, cycle ${formatInt(state.cycle_duration_ms)} ms, skew ${formatInt(state.update_skew_ms)} ms.`);
      updateMetric('summary-copy', state.summary || 'No summary yet.');
      document.getElementById('event-log-copy').textContent = state.event_log || 'No events yet.';

      if (shouldSyncSettingsFromState(forceSettingsSync)) {
        setNumber('setting-trigger', state.trigger_threshold);
        setNumber('setting-clear', state.clear_threshold);
        setNumber('setting-baseline', state.baseline_tolerance);
        setNumber('setting-debounce', state.debounce_ms);
        setNumber('setting-timeout', state.detection_timeout_ms);
        setNumber('setting-cooldown', state.cooldown_ms);
        setNumber('setting-min-valid', state.min_valid_sensors);
        setNumber('setting-max-people', state.max_people_inside);
        setCheckbox('setting-invert', state.invert_direction);
        setCheckbox('setting-autosave', state.auto_save_enabled);
      }

      renderGroups(state);
      renderSensors(state);
      renderChart(state);
    }

    async function fetchState() {
      const response = await fetch(stateUrl, { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return response.json();
    }

    async function postParams(params) {
      const response = await fetch(`${actionUrl}?${params.toString()}`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/x-www-form-urlencoded;charset=UTF-8' },
        body: params,
      });
      const payload = await response.json().catch(() => ({}));
      if (!response.ok) {
        throw new Error(payload.message || `HTTP ${response.status}`);
      }
      return payload;
    }

    async function refresh(force = false) {
      try {
        const payload = await fetchState();
        connectionPill.textContent = payload.connection_text || 'Connected to device';
        connectionCopy.textContent = 'Online';
        lastSync.textContent = new Date().toLocaleTimeString();
        renderState(payload, { forceSettingsSync: force && !settingsDirty });
        lastOnline = true;
      } catch (error) {
        connectionPill.textContent = 'Connection lost. Retrying.';
        connectionCopy.textContent = 'Offline';
        if (lastOnline || force) {
          showToast('Could not refresh device state', true);
        }
        lastOnline = false;
      } finally {
        scheduleRefresh();
      }
    }

    function refreshDelay() {
      if (document.hidden) return 1800;
      if (busy) return 600;
      return 120;
    }

    function scheduleRefresh() {
      clearTimeout(refreshTimer);
      refreshTimer = setTimeout(() => refresh(), refreshDelay());
    }

    document.addEventListener('click', async (event) => {
      const button = event.target.closest('button[data-command]');
      if (!button || busy) return;
      const confirmText = button.dataset.confirm;
      if (confirmText && !window.confirm(confirmText)) return;
      busy = true;
      button.disabled = true;
      try {
        const params = new URLSearchParams({ action: button.dataset.command });
        const result = await postParams(params);
        showToast(result.message || 'Updated');
        await refresh(true);
      } catch (error) {
        showToast(error.message || 'Unable to apply change', true);
      } finally {
        busy = false;
        button.disabled = false;
      }
    });

    Array.from(settingsForm.querySelectorAll('input')).forEach((input) => {
      const markDirty = () => {
        settingsDirty = true;
      };
      input.addEventListener('input', markDirty);
      input.addEventListener('change', markDirty);
    });

    settingsForm.addEventListener('submit', async (event) => {
      event.preventDefault();
      if (busy) return;
      busy = true;
      try {
        const data = new FormData(settingsForm);
        const params = new URLSearchParams();
        params.set('action', 'apply_settings');
        params.set('trigger_threshold', data.get('trigger_threshold'));
        params.set('clear_threshold', data.get('clear_threshold'));
        params.set('baseline_tolerance', data.get('baseline_tolerance'));
        params.set('debounce_ms', data.get('debounce_ms'));
        params.set('detection_timeout_ms', data.get('detection_timeout_ms'));
        params.set('cooldown_ms', data.get('cooldown_ms'));
        params.set('min_valid_sensors', data.get('min_valid_sensors'));
        params.set('max_people_inside', data.get('max_people_inside'));
        params.set('invert_direction', document.getElementById('setting-invert').checked ? '1' : '0');
        params.set('auto_save_enabled', document.getElementById('setting-autosave').checked ? '1' : '0');
        const result = await postParams(params);
        settingsDirty = false;
        showToast(result.message || 'Settings saved');
        await refresh(true);
      } catch (error) {
        showToast(error.message || 'Unable to save settings', true);
      } finally {
        busy = false;
      }
    });

    document.addEventListener('visibilitychange', () => {
      if (!document.hidden) refresh(true);
    });

    refresh(true);
  </script>
</body>
</html>
)html";

std::string default_subtitle(const tof_overdoor_counter::TofOverdoorCounter *counter) {
  return counter->is_monitor_mode()
             ? "Live monitor for four working ToF sensors, focused on baselines, agreement, and placement stability."
             : "Local four-sensor people counter where each ToF sensor votes independently, and two matching votes confirm IN or OUT.";
}

const char *sensor_group_label_for_index(size_t index) {
  return index < 4 ? SENSOR_GROUP_LABELS[index] : "Unknown";
}

}  // namespace

class TofOverdoorUi::Handler : public AsyncWebHandler {
 public:
  explicit Handler(TofOverdoorUi *parent) : parent_(parent) {}

  bool canHandle(AsyncWebServerRequest *request) const override {
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buf);
    auto method = request->method();
    if (method == HTTP_GET && (url == "/" || url == "/tof-overdoor-ui/state" || url == "/tof-overdoor-ui/compact" ||
                               url == "/tof-overdoor-ui/trace")) {
      return true;
    }
    if (method == HTTP_POST && url == "/tof-overdoor-ui/action") {
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
    if (request->method() == HTTP_GET && url == "/tof-overdoor-ui/state") {
      this->handle_state_(request);
      return;
    }
    if (request->method() == HTTP_GET && url == "/tof-overdoor-ui/compact") {
      this->handle_compact_(request);
      return;
    }
    if (request->method() == HTTP_GET && url == "/tof-overdoor-ui/trace") {
      this->handle_trace_(request);
      return;
    }
    if (request->method() == HTTP_POST && url == "/tof-overdoor-ui/action") {
      this->handle_action_(request);
      return;
    }
    request->send(404, "text/plain", "Not found");
  }

 protected:
  void handle_index_(AsyncWebServerRequest *request) {
    auto *response =
        request->beginResponse(200, "text/html; charset=utf-8",
                               reinterpret_cast<const uint8_t *>(OVERDOOR_UI_HTML), sizeof(OVERDOOR_UI_HTML) - 1);
    request->send(response);
  }

  void handle_state_(AsyncWebServerRequest *request) {
    auto *counter = this->parent_->counter_;
    auto json = json::build_json([this, counter](JsonObject root) {
      root["title"] = this->parent_->title_.empty() ? App.get_friendly_name() : this->parent_->title_;
      root["subtitle"] = default_subtitle(counter);
      root["label"] = this->parent_->label_.empty() ? "Front Door Counter" : this->parent_->label_;
      root["connection_text"] = "Connected to device";
      root["mode"] = counter->get_mode_text();
      root["ready"] = counter->get_ready_state() > 0.5f;
      root["presence"] = counter->get_presence_state() > 0.5f;
      root["person_standing"] = counter->get_person_standing_state() > 0.5f;
      root["system_status"] = counter->get_system_status_text();
      root["phase"] = counter->get_phase_text();
      root["people_count"] = static_cast<int>(counter->get_people_count());
      root["entry_count"] = static_cast<int>(counter->get_entry_count());
      root["exit_count"] = static_cast<int>(counter->get_exit_count());
      root["unsure_in_count"] = static_cast<int>(counter->get_unsure_in_count());
      root["unsure_out_count"] = static_cast<int>(counter->get_unsure_out_count());
      root["last_direction"] = counter->get_last_direction_text();
      root["last_detection_time"] = counter->get_last_detection_timestamp_text();
      root["last_reason"] = counter->get_last_reason_text();
      root["passage_state"] = counter->get_passage_state_text();
      root["debug_snapshot"] = counter->get_debug_snapshot_text();
      root["blocked_sensor"] = counter->get_blocked_sensor_text();
      root["summary"] = counter->get_summary();
      root["discovery_map"] = counter->get_discovery_map();
      root["event_log"] = counter->get_event_log();
      root["discovered_sensors"] = static_cast<int>(counter->get_discovered_sensor_count());
      root["reporting_sensors"] = static_cast<int>(counter->get_reporting_sensor_count());
      root["cycle_duration_ms"] = counter->get_cycle_duration_ms();
      root["update_skew_ms"] = counter->get_update_skew_ms();
      root["nearest_distance"] = counter->get_nearest_distance_mm();
      root["average_distance"] = counter->get_average_distance_mm();
      root["distance_span"] = counter->get_distance_span_mm();
      root["last_confidence"] = counter->get_confidence_score();
      root["calibration_progress"] = counter->get_calibration_progress();
      root["trigger_threshold"] = counter->get_trigger_threshold_value();
      root["clear_threshold"] = counter->get_clear_threshold_value();
      root["baseline_tolerance"] = counter->get_baseline_tolerance_value();
      root["debounce_ms"] = counter->get_debounce_value();
      root["detection_timeout_ms"] = counter->get_detection_timeout_value();
      root["cooldown_ms"] = counter->get_cooldown_value();
      root["min_valid_sensors"] = counter->get_min_valid_sensors_value();
      root["max_people_inside"] = counter->get_max_people_inside_value();
      root["invert_direction"] = counter->get_invert_direction();
      root["auto_save_enabled"] = counter->get_auto_save_enabled();

      auto groups = root["groups"].to<JsonArray>();
      for (size_t group_index = 0; group_index < 2; group_index++) {
        auto group = groups.add<JsonObject>();
        group["label"] = counter->get_group_label(group_index);
        group["description"] = group_index == 0 ? "Nearest OUT ROI across all four physical sensors." :
                                                    "Nearest IN ROI across all four physical sensors.";
        group["active"] = counter->get_row_active_state(group_index) > 0.5f;
        group["distance"] = counter->get_row_distance_mm(group_index);
        group["baseline"] = counter->get_row_baseline_mm(group_index);
        group["drop"] = counter->get_row_drop_mm(group_index);
        group["health"] = counter->get_row_active_state(group_index) > 0.5f ? "Triggered" : "Ready";
      }

      auto sensors = root["sensors"].to<JsonArray>();
      for (size_t index = 0; index < 4; index++) {
        auto sensor = sensors.add<JsonObject>();
        sensor["label"] = SENSOR_CARD_LABELS[index];
        sensor["group"] = sensor_group_label_for_index(index);
        sensor["source"] = counter->get_source_label(index);
        sensor["status"] = counter->get_status_text(index);
        sensor["health"] = counter->get_sensor_health_text(index);
        sensor["distance"] = counter->get_distance_mm(index);
        sensor["raw"] = counter->get_raw_distance_mm(index);
        sensor["filtered"] = counter->get_filtered_distance_mm(index);
        sensor["baseline"] = counter->get_baseline_mm(index);
        sensor["noise"] = counter->get_noise_mm(index);
        sensor["quality"] = counter->get_calibration_quality(index);
        sensor["drop"] = counter->get_delta_mm(index);
        sensor["active"] = counter->get_sensor_active_state(index) > 0.5f;
      }
    });
    request->send(200, "application/json", json.c_str());
  }

  void handle_compact_(AsyncWebServerRequest *request) {
    const std::string body = this->parent_->counter_->get_compact_state_text();
    request->send(200, "text/plain; charset=utf-8", body.c_str());
  }

  void handle_trace_(AsyncWebServerRequest *request) {
    const std::string body = this->parent_->counter_->get_trace_log_text();
    request->send(200, "text/plain; charset=utf-8", body.c_str());
  }

  void handle_action_(AsyncWebServerRequest *request) {
    auto *counter = this->parent_->counter_;
    const auto action = request->arg("action");
    std::string message = "Updated";

    if (action == "recalibrate") {
      counter->recalibrate();
      message = "Calibration started. Keep the doorway empty.";
    } else if (action == "rediscover") {
      counter->rediscover();
      message = "Sensor rediscovery started.";
    } else if (action == "reset_counts") {
      counter->reset_counts();
      counter->persist_runtime_state();
      message = "Confirmed counters reset.";
    } else if (action == "reset_unsure_in") {
      counter->reset_unsure_in();
      counter->persist_runtime_state();
      message = "Unsure IN counter reset.";
    } else if (action == "reset_unsure_out") {
      counter->reset_unsure_out();
      counter->persist_runtime_state();
      message = "Unsure OUT counter reset.";
    } else if (action == "reset_all_counters") {
      counter->reset_all_counters();
      counter->persist_runtime_state();
      message = "All counters reset.";
    } else if (action == "reset_trace") {
      counter->reset_trace_buffer();
      message = "Trace buffer reset.";
    } else if (action == "restart") {
      message = "Restarting ESP.";
      auto response = json::build_json([&message](JsonObject root) {
        root["ok"] = true;
        root["message"] = message;
      });
      request->send(200, "application/json", response.c_str());
      App.safe_reboot();
      return;
    } else if (action == "apply_settings") {
      counter->set_trigger_delta_mm(static_cast<uint16_t>(
          std::max(80, std::min(1200, parse_int_arg(request, "trigger_threshold", static_cast<int>(counter->get_trigger_threshold_value()))))));
      counter->set_release_delta_mm(static_cast<uint16_t>(
          std::max(40, std::min(900, parse_int_arg(request, "clear_threshold", static_cast<int>(counter->get_clear_threshold_value()))))));
      counter->set_baseline_tolerance_mm(static_cast<uint16_t>(
          std::max(20, std::min(300, parse_int_arg(request, "baseline_tolerance", static_cast<int>(counter->get_baseline_tolerance_value()))))));
      counter->set_debounce_ms(static_cast<uint32_t>(
          std::max(5, std::min(300, parse_int_arg(request, "debounce_ms", static_cast<int>(counter->get_debounce_value()))))));
      counter->set_sequence_timeout_ms(static_cast<uint32_t>(std::max(
          300, std::min(4000, parse_int_arg(request, "detection_timeout_ms", static_cast<int>(counter->get_detection_timeout_value()))))));
      counter->set_cooldown_ms(static_cast<uint32_t>(
          std::max(0, std::min(3000, parse_int_arg(request, "cooldown_ms", static_cast<int>(counter->get_cooldown_value()))))));
      counter->set_min_valid_sensors(static_cast<uint8_t>(
          std::max(2, std::min(4, parse_int_arg(request, "min_valid_sensors", static_cast<int>(counter->get_min_valid_sensors_value()))))));
      counter->set_max_people_inside(static_cast<uint16_t>(
          std::max(1, std::min(500, parse_int_arg(request, "max_people_inside", static_cast<int>(counter->get_max_people_inside_value()))))));
      counter->set_invert_direction(parse_bool_arg(request, "invert_direction", counter->get_invert_direction()));
      counter->set_auto_save_enabled(parse_bool_arg(request, "auto_save_enabled", counter->get_auto_save_enabled()));
      counter->persist_runtime_state();
      message = "Detection settings saved to the ESP.";
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

  TofOverdoorUi *parent_;
};

void TofOverdoorUi::setup() {
  if (this->counter_ == nullptr) {
    this->mark_failed();
    ESP_LOGE(TAG, "TofOverdoorUi requires a counter");
    return;
  }

  if (web_server_base::global_web_server_base == nullptr) {
    this->mark_failed();
    ESP_LOGE(TAG, "TofOverdoorUi requires web_server");
    return;
  }

  if (this->handler_ == nullptr) {
    this->handler_ = new Handler(this);
  }

  web_server_base::global_web_server_base->add_handler(this->handler_);
  ESP_LOGI(TAG, "Custom overdoor UI registered on /");
}

}  // namespace tof_overdoor_ui
}  // namespace esphome

#endif
