#include "tof_overdoor_ui.h"

#ifdef USE_WEBSERVER

#include <cmath>
#include <string>

#include "esphome/components/json/json_util.h"
#include "esphome/core/application.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tof_overdoor_ui {

static const char *const TAG = "tof_overdoor_ui";

namespace {

const char *const SENSOR_CARD_LABELS[] = {"U3", "U4", "U7", "U8"};

const char OVERDOOR_UI_HTML[] = R"html(
<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Roode Overdoor</title>
  <style>
    :root {
      color-scheme: light dark;
      --font-display: "Iowan Old Style", "Palatino Linotype", "URW Palladio L", serif;
      --font-body: "Avenir Next", "Segoe UI", "Helvetica Neue", sans-serif;
      --bg: oklch(96.5% 0.008 95);
      --bg-soft: oklch(98.5% 0.004 95);
      --panel: rgba(255, 255, 255, 0.9);
      --panel-strong: rgba(255, 255, 255, 0.97);
      --line: oklch(82% 0.014 245);
      --line-strong: oklch(70% 0.032 245);
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

    * {
      box-sizing: border-box;
    }

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
      font-family: var(--font-display);
      letter-spacing: -0.03em;
      margin: 0;
      font-weight: 600;
    }

    h1 {
      font-size: clamp(2.6rem, 4vw, 4.2rem);
      line-height: 0.95;
    }

    h2 {
      font-size: clamp(1.55rem, 2vw, 2rem);
    }

    h3 {
      font-size: 1.1rem;
    }

    p {
      margin: 0;
    }

    .shell {
      width: min(1320px, calc(100vw - 2rem));
      margin: 0 auto;
      padding: 1.1rem 0 2rem;
    }

    .panel {
      border: 1px solid var(--line);
      border-radius: var(--radius-lg);
      background: var(--panel-strong);
      box-shadow: var(--shadow);
      backdrop-filter: blur(10px);
    }

    .eyebrow {
      margin-bottom: 0.42rem;
      letter-spacing: 0.12em;
      text-transform: uppercase;
      font-size: 0.74rem;
      color: var(--ink-soft);
    }

    .masthead {
      display: grid;
      grid-template-columns: minmax(0, 1.3fr) minmax(300px, 0.7fr);
      gap: 1rem;
      margin-bottom: 1rem;
    }

    .hero {
      padding: 1.35rem 1.45rem 1.45rem;
    }

    .hero-copy {
      margin-top: 0.7rem;
      color: var(--ink-soft);
      line-height: 1.58;
      font-size: 1.03rem;
      max-width: 60ch;
    }

    .hero-steps {
      margin-top: 1.15rem;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 0.85rem;
    }

    .step {
      padding: 1rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 74%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 86%, transparent);
    }

    .step strong {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      width: 1.6rem;
      height: 1.6rem;
      border-radius: 999px;
      background: var(--accent-soft);
      color: var(--accent);
      font-size: 0.84rem;
      margin-bottom: 0.5rem;
    }

    .step p {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.93rem;
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
      padding: 0.88rem 1.15rem;
      font: inherit;
      font-weight: 700;
      cursor: pointer;
    }

    .aside-card {
      padding: 1rem;
      display: grid;
      gap: 0.85rem;
    }

    .top-status {
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
    }

    .aside-note strong {
      display: block;
      margin-bottom: 0.25rem;
      font-size: 0.96rem;
    }

    .aside-note p {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.94rem;
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

    .layout {
      display: grid;
      grid-template-columns: minmax(0, 1.15fr) minmax(320px, 0.85fr);
      gap: 1rem;
    }

    .stack {
      display: grid;
      gap: 1rem;
    }

    .section {
      padding: 1.2rem;
    }

    .summary-grid {
      display: grid;
      grid-template-columns: repeat(5, minmax(0, 1fr));
      gap: 0.85rem;
      margin-top: 1rem;
    }

    .summary-card {
      padding: 1rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 78%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .summary-card span {
      display: block;
      margin-bottom: 0.28rem;
      font-size: 0.74rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
    }

    .summary-card strong {
      display: block;
      font-size: clamp(1.4rem, 2.6vw, 2.2rem);
      line-height: 1;
      margin-bottom: 0.28rem;
    }

    .summary-card p {
      color: var(--ink-soft);
      line-height: 1.4;
      font-size: 0.88rem;
    }

    .state-banner {
      margin-top: 1rem;
      padding: 1rem 1.05rem;
      border-radius: var(--radius-md);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      background: color-mix(in oklch, var(--panel-strong) 78%, var(--bg-soft));
      display: grid;
      gap: 0.35rem;
    }

    .state-pill {
      display: inline-flex;
      align-items: center;
      justify-content: center;
      width: fit-content;
      padding: 0.5rem 0.8rem;
      border-radius: 999px;
      font-size: 0.82rem;
      font-weight: 800;
      letter-spacing: 0.03em;
    }

    .state-ready {
      background: var(--success-soft);
      color: var(--success);
    }

    .state-warn {
      background: var(--warn-soft);
      color: color-mix(in oklch, var(--warn) 76%, black);
    }

    .state-error {
      background: var(--danger-soft);
      color: var(--danger);
    }

    .state-banner p {
      color: var(--ink-soft);
      line-height: 1.5;
    }

    .doorway {
      display: grid;
      gap: 1rem;
    }

    .door-visual {
      position: relative;
      padding: 1.2rem;
      border-radius: var(--radius-lg);
      background: linear-gradient(180deg, color-mix(in oklch, var(--panel-strong) 75%, var(--bg-soft)), var(--panel-strong));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      overflow: hidden;
    }

    .door-visual::before {
      content: "";
      position: absolute;
      inset: 1rem;
      border-radius: 1.2rem;
      border: 1px dashed color-mix(in oklch, var(--line-strong) 45%, transparent);
      pointer-events: none;
    }

    .door-title {
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 1rem;
      margin-bottom: 0.9rem;
    }

    .door-title p {
      color: var(--ink-soft);
      line-height: 1.45;
    }

    .door-grid {
      display: grid;
      gap: 0.95rem;
    }

    .row-card {
      display: grid;
      gap: 0.8rem;
      padding: 1rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 86%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .row-head {
      display: flex;
      justify-content: space-between;
      gap: 1rem;
      align-items: start;
    }

    .row-head p {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.92rem;
    }

    .row-badge {
      display: inline-flex;
      align-items: center;
      gap: 0.45rem;
      padding: 0.45rem 0.75rem;
      border-radius: 999px;
      font-size: 0.8rem;
      font-weight: 800;
      letter-spacing: 0.03em;
      background: var(--bg-soft);
      color: var(--ink-soft);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      white-space: nowrap;
    }

    .row-badge.active {
      background: var(--accent-soft);
      color: var(--accent);
      border-color: color-mix(in oklch, var(--accent) 26%, var(--line));
    }

    .row-metrics {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 0.75rem;
    }

    .mini-metric {
      padding: 0.82rem 0.9rem;
      border-radius: var(--radius-sm);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
    }

    .mini-metric span {
      display: block;
      margin-bottom: 0.28rem;
      font-size: 0.72rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
    }

    .mini-metric strong {
      display: block;
      font-size: 1.1rem;
      margin-bottom: 0.2rem;
    }

    .mini-metric p {
      color: var(--ink-soft);
      line-height: 1.35;
      font-size: 0.84rem;
    }

    .sensor-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.75rem;
    }

    .sensor-card {
      padding: 0.95rem;
      border-radius: var(--radius-md);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      display: grid;
      gap: 0.55rem;
    }

    .sensor-card.active {
      border-color: color-mix(in oklch, var(--accent) 28%, var(--line));
      background: color-mix(in oklch, var(--accent) 7%, var(--bg-soft));
    }

    .sensor-top {
      display: flex;
      justify-content: space-between;
      gap: 0.8rem;
      align-items: start;
    }

    .sensor-title {
      display: grid;
      gap: 0.18rem;
    }

    .sensor-title strong {
      font-size: 1rem;
    }

    .sensor-title span {
      color: var(--ink-soft);
      font-size: 0.84rem;
    }

    .sensor-state {
      padding: 0.32rem 0.58rem;
      border-radius: 999px;
      font-size: 0.76rem;
      font-weight: 800;
      white-space: nowrap;
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

    .bar {
      width: 100%;
      height: 0.68rem;
      border-radius: 999px;
      overflow: hidden;
      background: color-mix(in oklch, var(--line) 75%, transparent);
    }

    .bar > span {
      display: block;
      height: 100%;
      width: 0;
      border-radius: inherit;
      background: linear-gradient(90deg, color-mix(in oklch, var(--accent) 62%, white), var(--accent));
      transition: width 0.22s ease;
    }

    .sensor-copy {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.9rem;
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
      padding: 0.9rem 1.15rem;
      font: inherit;
      font-weight: 800;
      cursor: pointer;
      transition: transform 0.18s ease, opacity 0.18s ease;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 0.45rem;
    }

    button:hover {
      transform: translateY(-1px);
    }

    button:disabled {
      opacity: 0.55;
      cursor: wait;
      transform: none;
    }

    .primary {
      color: white;
      background: linear-gradient(135deg, color-mix(in oklch, var(--accent) 72%, black), var(--accent));
    }

    .secondary {
      color: var(--ink);
      background: color-mix(in oklch, var(--panel-strong) 78%, var(--bg-soft));
      border: 1px solid var(--line);
    }

    .debug-grid {
      display: grid;
      gap: 0.85rem;
    }

    .debug-card {
      padding: 1rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 82%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      display: grid;
      gap: 0.45rem;
    }

    .debug-card strong {
      font-size: 0.96rem;
    }

    .debug-card p {
      color: var(--ink-soft);
      line-height: 1.45;
      word-break: break-word;
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

    .toast.show {
      opacity: 1;
      transform: translateY(0);
    }

    @media (max-width: 1160px) {
      .layout,
      .masthead,
      .hero-steps,
      .summary-grid {
        grid-template-columns: 1fr;
      }
    }

    @media (max-width: 900px) {
      .sensor-grid,
      .row-metrics,
      .top-status {
        grid-template-columns: 1fr;
      }

      .row-head,
      .sensor-top,
      .door-title {
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
        <h1 id="page-title">Loading device</h1>
        <p class="hero-copy" id="page-subtitle">Preparing the live doorway view, sensor health, and counting status.</p>

        <div class="hero-steps">
          <article class="step">
            <strong>1</strong>
            <p>Leave the doorway empty and wait for the counter to report that it is ready.</p>
          </article>
          <article class="step">
            <strong>2</strong>
            <p>Press recalibrate if the board was just moved or mounted in a new position.</p>
          </article>
          <article class="step">
            <strong>3</strong>
            <p>Walk through the doorway in one clear direction and confirm that the phase and count make sense.</p>
          </article>
        </div>
      </section>

      <aside class="aside">
        <button class="theme-toggle" type="button" id="theme-toggle">Switch to dark mode</button>
        <section class="panel aside-card">
          <div class="top-status">
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
            <strong>What this page helps you see</strong>
            <p>Which row saw movement first, whether all four sensors agree, and whether the device counted an entry, an exit, or cancelled the sequence.</p>
          </div>
        </section>
      </aside>
    </div>

    <div class="connection" id="connection-pill">Connecting to device</div>

    <div class="layout">
      <div class="stack">
        <section class="panel section">
          <p class="eyebrow">Counting summary</p>
          <h2 id="counter-label">Doorway Counter</h2>

          <div class="summary-grid">
            <article class="summary-card">
              <span>Ready state</span>
              <strong id="ready-value">-</strong>
              <p id="ready-copy">Waiting for device state.</p>
            </article>
            <article class="summary-card">
              <span>People count</span>
              <strong id="people-count">0</strong>
              <p>Net number of people currently believed to be inside.</p>
            </article>
            <article class="summary-card">
              <span>Entries</span>
              <strong id="entry-count">0</strong>
              <p>Total confirmed entry events since the last reset.</p>
            </article>
            <article class="summary-card">
              <span>Exits</span>
              <strong id="exit-count">0</strong>
              <p>Total confirmed exit events since the last reset.</p>
            </article>
            <article class="summary-card">
              <span>Last direction</span>
              <strong id="last-direction">Waiting</strong>
              <p id="direction-copy">No event confirmed yet.</p>
            </article>
          </div>

          <div class="state-banner">
            <div class="state-pill state-ready" id="phase-pill">Monitoring</div>
            <h3 id="phase-title">Waiting for first live state</h3>
            <p id="phase-copy">The page will explain what the counter is doing right now and what to test next.</p>
          </div>

          <div class="actions">
            <button class="primary" data-command="recalibrate">Recalibrate floor reference</button>
            <button class="secondary" data-command="rediscover">Rediscover sensors</button>
            <button class="secondary" data-command="reset_counts">Reset counts</button>
          </div>
        </section>

        <section class="panel section doorway">
          <div class="door-title">
            <div>
              <p class="eyebrow">Live doorway view</p>
              <h2>What each row sees</h2>
            </div>
            <p id="row-copy">Lower distance means an object is closer to the board. A row becomes active when one of its two sensors drops far enough below its normal baseline.</p>
          </div>

          <div class="door-visual">
            <div class="door-grid">
              <article class="row-card">
                <div class="row-head">
                  <div>
                    <h3>Row A</h3>
                    <p id="row-a-copy">Top pair of sensors watching the first half of the doorway.</p>
                  </div>
                  <div class="row-badge" id="row-a-badge">Clear</div>
                </div>
                <div class="row-metrics">
                  <div class="mini-metric">
                    <span>Nearest distance</span>
                    <strong id="row-a-distance">-</strong>
                    <p>Closest reading from either sensor in Row A.</p>
                  </div>
                  <div class="mini-metric">
                    <span>Baseline</span>
                    <strong id="row-a-baseline">-</strong>
                    <p>Normal empty-doorway reference for Row A.</p>
                  </div>
                  <div class="mini-metric">
                    <span>Drop from baseline</span>
                    <strong id="row-a-drop">-</strong>
                    <p>How much closer something is than the usual baseline.</p>
                  </div>
                </div>
              </article>

              <article class="row-card">
                <div class="row-head">
                  <div>
                    <h3>Row B</h3>
                    <p id="row-b-copy">Bottom pair of sensors watching the second half of the doorway.</p>
                  </div>
                  <div class="row-badge" id="row-b-badge">Clear</div>
                </div>
                <div class="row-metrics">
                  <div class="mini-metric">
                    <span>Nearest distance</span>
                    <strong id="row-b-distance">-</strong>
                    <p>Closest reading from either sensor in Row B.</p>
                  </div>
                  <div class="mini-metric">
                    <span>Baseline</span>
                    <strong id="row-b-baseline">-</strong>
                    <p>Normal empty-doorway reference for Row B.</p>
                  </div>
                  <div class="mini-metric">
                    <span>Drop from baseline</span>
                    <strong id="row-b-drop">-</strong>
                    <p>How much closer something is than the usual baseline.</p>
                  </div>
                </div>
              </article>
            </div>
          </div>
        </section>
      </div>

      <div class="stack">
        <section class="panel section">
          <p class="eyebrow">Sensor cards</p>
          <h2>Individual sensor health</h2>
          <div class="sensor-grid" id="sensor-grid"></div>
        </section>

        <section class="panel section">
          <p class="eyebrow">Diagnostics</p>
          <h2>Debug clues</h2>
          <div class="debug-grid">
            <article class="debug-card">
              <strong>Discovery map</strong>
              <p id="discovery-map">Waiting for device state.</p>
            </article>
            <article class="debug-card">
              <strong>Sync timing</strong>
              <p id="timing-copy">Waiting for device state.</p>
            </article>
            <article class="debug-card">
              <strong>How to test direction</strong>
              <p id="test-copy">Walk one clear path through the doorway so one row becomes active before the other. Covering all four sensors at once only proves detection, not direction.</p>
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
    const sensorGrid = document.getElementById('sensor-grid');
    const pageTitle = document.getElementById('page-title');
    const pageSubtitle = document.getElementById('page-subtitle');
    const counterLabel = document.getElementById('counter-label');
    const connectionPill = document.getElementById('connection-pill');
    const connectionCopy = document.getElementById('connection-copy');
    const lastSync = document.getElementById('last-sync');
    const toast = document.getElementById('toast');
    const themeToggle = document.getElementById('theme-toggle');

    let refreshTimer = null;
    let busy = false;
    let lastOnline = true;

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
      showToast._timer = setTimeout(() => toast.classList.remove('show'), 2400);
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

    function barWidth(distance) {
      const num = Number(distance);
      if (!Number.isFinite(num)) return '0%';
      const clamped = Math.max(0, Math.min(2500, num));
      return `${(clamped / 2500) * 100}%`;
    }

    function phaseStyle(phase) {
      const text = String(phase || '').toLowerCase();
      if (text.includes('ready') || text.includes('monitoring')) return 'state-ready';
      if (text.includes('timeout') || text.includes('cancel') || text.includes('clear')) return 'state-warn';
      return 'state-ready';
    }

    function phaseDescription(state) {
      const phase = String(state.phase || '').toLowerCase();
      if (!state.ready) {
        return 'The board is still learning empty-doorway baselines. Leave the doorway clear for a moment so the reference values settle.';
      }
      if (phase.includes('row a leading')) {
        return 'Row A changed first. If Row B follows next and the doorway clears, the event should become a count.';
      }
      if (phase.includes('row b leading')) {
        return 'Row B changed first. If Row A follows next and the doorway clears, the event should become a count in the opposite direction.';
      }
      if (phase.includes('both rows active')) {
        return 'Both rows currently see an object. This is expected mid-pass, but it is not enough on its own to decide direction.';
      }
      if (phase.includes('settling')) {
        return 'A recent event finished and the counter is pausing briefly before accepting the next one.';
      }
      if (phase.includes('cancel')) {
        return 'The last movement pattern did not look like a clean pass through the doorway, so it was ignored on purpose.';
      }
      if (phase.includes('clear')) {
        return 'The counter is waiting for the doorway to become empty again before treating a new sequence as valid.';
      }
      return 'The counter is ready. A clean pass should activate one row first, then the other, and then clear again.';
    }

    function readyCopy(state) {
      if (!state.ready) return 'Learning empty-doorway baseline';
      return state.presence ? 'Ready, but something is still in view' : 'Ready for real pass testing';
    }

    function directionCopy(direction) {
      if (direction === 'Entry') return 'The most recent clean pass was counted as an entry.';
      if (direction === 'Exit') return 'The most recent clean pass was counted as an exit.';
      if (direction === 'Cancelled') return 'The last motion pattern was intentionally ignored because it did not look like a valid pass.';
      if (direction === 'Reset') return 'Counts were reset by a user action.';
      return 'No clean pass has been confirmed yet.';
    }

    function rowStatusCopy(active, label) {
      return active
        ? `${label} currently sees something closer than its normal doorway baseline.`
        : `${label} looks clear right now.`;
    }

    function sensorStatusClass(status) {
      const text = String(status || '').toLowerCase();
      if (text.includes('error') || text.includes('missing')) return 'state-error';
      if (text.includes('occupied')) return 'state-warn';
      return 'state-ready';
    }

    function renderState(state) {
      pageTitle.textContent = state.title || 'Roode Overdoor';
      pageSubtitle.textContent = state.subtitle || 'Visual verification for the four working doorway sensors.';
      counterLabel.textContent = state.label || 'Doorway Counter';

      document.getElementById('ready-value').textContent = state.ready ? 'Ready' : 'Calibrating';
      document.getElementById('ready-copy').textContent = readyCopy(state);
      document.getElementById('people-count').textContent = String(state.people_count ?? 0);
      document.getElementById('entry-count').textContent = String(state.entry_count ?? 0);
      document.getElementById('exit-count').textContent = String(state.exit_count ?? 0);
      document.getElementById('last-direction').textContent = state.last_direction || 'Waiting';
      document.getElementById('direction-copy').textContent = directionCopy(state.last_direction);

      const phasePill = document.getElementById('phase-pill');
      phasePill.className = `state-pill ${phaseStyle(state.phase)}`;
      phasePill.textContent = state.phase || 'Monitoring';
      document.getElementById('phase-title').textContent = state.summary || 'Monitoring doorway';
      document.getElementById('phase-copy').textContent = phaseDescription(state);

      const rowABadge = document.getElementById('row-a-badge');
      rowABadge.className = `row-badge ${state.row_a_active ? 'active' : ''}`;
      rowABadge.textContent = state.row_a_active ? 'Active now' : 'Clear';
      document.getElementById('row-a-distance').textContent = formatMm(state.row_a_distance);
      document.getElementById('row-a-baseline').textContent = formatMm(state.row_a_baseline);
      document.getElementById('row-a-drop').textContent = formatMm(state.row_a_drop);
      document.getElementById('row-a-copy').textContent = rowStatusCopy(state.row_a_active, 'Row A');

      const rowBBadge = document.getElementById('row-b-badge');
      rowBBadge.className = `row-badge ${state.row_b_active ? 'active' : ''}`;
      rowBBadge.textContent = state.row_b_active ? 'Active now' : 'Clear';
      document.getElementById('row-b-distance').textContent = formatMm(state.row_b_distance);
      document.getElementById('row-b-baseline').textContent = formatMm(state.row_b_baseline);
      document.getElementById('row-b-drop').textContent = formatMm(state.row_b_drop);
      document.getElementById('row-b-copy').textContent = rowStatusCopy(state.row_b_active, 'Row B');

      document.getElementById('discovery-map').textContent = state.discovery_map || 'No discovery data';
      document.getElementById('timing-copy').textContent =
        `${state.discovered_sensors} sensors active, cycle ${Math.round(Number(state.cycle_duration_ms || 0))} ms, skew ${Math.round(Number(state.update_skew_ms || 0))} ms.`;

      sensorGrid.innerHTML = (state.sensors || []).map((sensor) => `
        <article class="sensor-card ${sensor.active ? 'active' : ''}">
          <div class="sensor-top">
            <div class="sensor-title">
              <strong>${escapeHtml(sensor.label)}</strong>
              <span>${escapeHtml(sensor.source)}</span>
            </div>
            <div class="sensor-state ${sensorStatusClass(sensor.status)}">${escapeHtml(sensor.status)}</div>
          </div>
          <div class="sensor-distance">
            <strong>${formatMm(sensor.distance)}</strong>
            <span>Baseline ${formatMm(sensor.baseline)}</span>
          </div>
          <div class="bar"><span style="width:${barWidth(sensor.distance)}"></span></div>
          <p class="sensor-copy">Drop from baseline: ${formatMm(sensor.drop)}. ${sensor.active ? 'This sensor currently contributes to an active row.' : 'This sensor currently sees a clear doorway.'}</p>
        </article>
      `).join('');
    }

    async function fetchState() {
      const response = await fetch(stateUrl, { cache: 'no-store' });
      if (!response.ok) throw new Error(`HTTP ${response.status}`);
      return response.json();
    }

    async function postAction(action) {
      const params = new URLSearchParams({ action });
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
        renderState(payload);
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
      if (document.hidden) return 8000;
      if (busy) return 4000;
      return 1200;
    }

    function scheduleRefresh() {
      clearTimeout(refreshTimer);
      refreshTimer = setTimeout(() => refresh(), refreshDelay());
    }

    document.addEventListener('click', async (event) => {
      const button = event.target.closest('button[data-command]');
      if (!button || busy) return;
      busy = true;
      button.disabled = true;
      try {
        const result = await postAction(button.dataset.command);
        showToast(result.message || 'Updated');
        await refresh(true);
      } catch (error) {
        showToast(error.message || 'Unable to apply change', true);
      } finally {
        busy = false;
        button.disabled = false;
      }
    });

    document.addEventListener('visibilitychange', () => {
      if (!document.hidden) {
        refresh(true);
      }
    });

    refresh(true);
  </script>
</body>
</html>
)html";

std::string readiness_copy(const tof_overdoor_counter::TofOverdoorCounter *counter) {
  if (counter->get_ready_state() < 0.5f) {
    return "Calibrating";
  }
  if (counter->get_presence_state() > 0.5f) {
    return "Ready, but occupied";
  }
  return "Ready";
}

}  // namespace

class TofOverdoorUi::Handler : public AsyncWebHandler {
 public:
  explicit Handler(TofOverdoorUi *parent) : parent_(parent) {}

  bool canHandle(AsyncWebServerRequest *request) const override {
    char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
    auto url = request->url_to(url_buf);
    auto method = request->method();
    if (method == HTTP_GET && (url == "/" || url == "/tof-overdoor-ui/state")) {
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
      root["subtitle"] = "A clearer live view of the four working doorway sensors, the two logical rows, and the counting state machine.";
      root["label"] = this->parent_->label_.empty() ? "Doorway Counter" : this->parent_->label_;
      root["connection_text"] = "Connected to device";
      root["ready"] = counter->get_ready_state() > 0.5f;
      root["presence"] = counter->get_presence_state() > 0.5f;
      root["people_count"] = static_cast<int>(counter->get_people_count());
      root["entry_count"] = static_cast<int>(counter->get_entry_count());
      root["exit_count"] = static_cast<int>(counter->get_exit_count());
      root["phase"] = counter->get_phase_text();
      root["last_direction"] = counter->get_last_direction_text();
      root["summary"] = counter->get_summary();
      root["discovery_map"] = counter->get_discovery_map();
      root["discovered_sensors"] = static_cast<int>(counter->get_discovered_sensor_count());
      root["cycle_duration_ms"] = counter->get_cycle_duration_ms();
      root["update_skew_ms"] = counter->get_update_skew_ms();
      root["row_a_active"] = counter->get_row_active_state(0) > 0.5f;
      root["row_b_active"] = counter->get_row_active_state(1) > 0.5f;
      root["row_a_distance"] = counter->get_row_distance_mm(0);
      root["row_b_distance"] = counter->get_row_distance_mm(1);
      root["row_a_baseline"] = counter->get_row_baseline_mm(0);
      root["row_b_baseline"] = counter->get_row_baseline_mm(1);
      root["row_a_drop"] = counter->get_row_drop_mm(0);
      root["row_b_drop"] = counter->get_row_drop_mm(1);
      root["invert_direction"] = counter->get_invert_direction();
      root["ready_copy"] = readiness_copy(counter);

      auto sensors = root["sensors"].to<JsonArray>();
      for (size_t index = 0; index < 4; index++) {
        auto sensor = sensors.add<JsonObject>();
        sensor["label"] = SENSOR_CARD_LABELS[index];
        sensor["source"] = counter->get_source_label(index);
        sensor["status"] = counter->get_status_text(index);
        sensor["distance"] = counter->get_distance_mm(index);
        sensor["baseline"] = counter->get_baseline_mm(index);
        const auto baseline = counter->get_baseline_mm(index);
        const auto distance = counter->get_distance_mm(index);
        sensor["drop"] = std::isnan(baseline) || std::isnan(distance) ? NAN : baseline - distance;
        sensor["active"] = counter->get_row_active_state(index < 2 ? 0 : 1) > 0.5f &&
                           counter->get_status_text(index) == "Occupied";
      }
    });
    request->send(200, "application/json", json.c_str());
  }

  void handle_action_(AsyncWebServerRequest *request) {
    auto *counter = this->parent_->counter_;
    auto action = request->arg("action");
    std::string message = "Updated";

    if (action == "recalibrate") {
      counter->recalibrate();
      message = "Floor reference recalibration started";
    } else if (action == "rediscover") {
      counter->rediscover();
      message = "Sensor rediscovery started";
    } else if (action == "reset_counts") {
      counter->reset_counts();
      message = "Counts reset";
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
