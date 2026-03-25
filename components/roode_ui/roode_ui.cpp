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
  <title>Roode Service</title>
  <style>
    :root {
      color-scheme: light dark;
      --font-display: "Iowan Old Style", "Palatino Linotype", "URW Palladio L", serif;
      --font-body: "Avenir Next", "Segoe UI", "Helvetica Neue", sans-serif;
      --bg: oklch(96.5% 0.01 92);
      --bg-soft: oklch(98.4% 0.004 92);
      --panel: rgba(255, 255, 255, 0.88);
      --panel-strong: rgba(255, 255, 255, 0.96);
      --line: oklch(82% 0.015 246);
      --line-strong: oklch(72% 0.03 244);
      --ink: oklch(27% 0.018 255);
      --ink-soft: oklch(47% 0.02 255);
      --accent: oklch(42% 0.075 244);
      --accent-soft: oklch(91% 0.02 240);
      --success: oklch(57% 0.09 154);
      --success-soft: color-mix(in oklch, var(--success) 16%, white);
      --warn: oklch(70% 0.12 85);
      --warn-soft: color-mix(in oklch, var(--warn) 16%, white);
      --danger: oklch(58% 0.13 28);
      --danger-soft: color-mix(in oklch, var(--danger) 14%, white);
      --shadow: 0 24px 70px rgba(23, 31, 45, 0.08);
      --radius-xl: 30px;
      --radius-lg: 22px;
      --radius-md: 16px;
      --radius-sm: 12px;
    }

    [data-theme="dark"] {
      --bg: oklch(20% 0.01 255);
      --bg-soft: oklch(23.5% 0.012 255);
      --panel: rgba(21, 27, 37, 0.88);
      --panel-strong: rgba(26, 33, 44, 0.96);
      --line: oklch(35% 0.012 255);
      --line-strong: oklch(52% 0.025 242);
      --ink: oklch(95% 0.008 95);
      --ink-soft: oklch(74% 0.018 255);
      --accent: oklch(73% 0.07 240);
      --accent-soft: rgba(72, 103, 170, 0.16);
      --success-soft: color-mix(in oklch, var(--success) 22%, transparent);
      --warn-soft: color-mix(in oklch, var(--warn) 22%, transparent);
      --danger-soft: color-mix(in oklch, var(--danger) 18%, transparent);
      --shadow: 0 24px 70px rgba(0, 0, 0, 0.28);
    }

    * {
      box-sizing: border-box;
    }

    body {
      margin: 0;
      font-family: var(--font-body);
      color: var(--ink);
      background:
        radial-gradient(circle at top left, color-mix(in oklch, var(--accent) 10%, transparent), transparent 34%),
        linear-gradient(180deg, var(--bg-soft), var(--bg));
      min-height: 100vh;
    }

    h1,
    h2,
    h3 {
      font-family: var(--font-display);
      letter-spacing: -0.03em;
      margin: 0;
      font-weight: 600;
    }

    h1 {
      font-size: clamp(2.5rem, 4vw, 4.1rem);
      line-height: 0.96;
    }

    h2 {
      font-size: clamp(1.6rem, 2.3vw, 2rem);
    }

    h3 {
      font-size: 1.18rem;
    }

    p {
      margin: 0;
    }

    .shell {
      width: min(1240px, calc(100vw - 2rem));
      margin: 0 auto;
      padding: 1.2rem 0 2.1rem;
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
      grid-template-columns: minmax(0, 1.25fr) minmax(320px, 0.75fr);
      gap: 1rem;
      align-items: start;
      margin-bottom: 1rem;
    }

    .hero {
      padding: 1.4rem 1.5rem 1.5rem;
    }

    .hero-copy {
      margin-top: 0.7rem;
      max-width: 58ch;
      color: var(--ink-soft);
      line-height: 1.55;
      font-size: 1.03rem;
    }

    .guide-grid {
      margin-top: 1.25rem;
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.9rem;
    }

    .guide-card {
      padding: 1rem 1rem 1.05rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 72%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 86%, transparent);
    }

    .guide-card h3 {
      margin-bottom: 0.45rem;
    }

    .guide-card p,
    .guide-card li {
      color: var(--ink-soft);
      line-height: 1.5;
    }

    .guide-card ol {
      margin: 0.4rem 0 0;
      padding-left: 1.1rem;
      display: grid;
      gap: 0.45rem;
    }

    .guide-card strong {
      color: var(--ink);
    }

    .top-tools {
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

    .top-summary {
      padding: 1rem;
      display: grid;
      gap: 0.85rem;
    }

    .status-grid {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.8rem;
    }

    .status-chip {
      padding: 0.95rem 1rem;
      border-radius: var(--radius-md);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 82%, transparent);
    }

    .status-chip span {
      display: block;
      font-size: 0.74rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
      margin-bottom: 0.35rem;
    }

    .status-chip strong {
      font-size: 1rem;
      font-weight: 700;
    }

    .summary-note {
      padding: 0.95rem 1rem;
      border-radius: var(--radius-md);
      background: var(--accent-soft);
      color: var(--ink);
    }

    .summary-note strong {
      display: block;
      margin-bottom: 0.25rem;
      font-size: 0.96rem;
    }

    .summary-note p {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.95rem;
    }

    .connection {
      display: inline-flex;
      align-items: center;
      gap: 0.52rem;
      padding: 0.7rem 0.9rem;
      border-radius: 999px;
      background: var(--accent-soft);
      color: var(--accent);
      font-weight: 700;
      font-size: 0.96rem;
      margin-bottom: 1rem;
    }

    .connection::before {
      content: "";
      width: 0.62rem;
      height: 0.62rem;
      border-radius: 999px;
      background: currentColor;
      box-shadow: 0 0 0 0.25rem color-mix(in oklch, currentColor 18%, transparent);
    }

    .workspace-note {
      margin-bottom: 1rem;
      padding: 1rem 1.1rem;
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 0.85rem;
    }

    .workspace-note article {
      padding: 0.95rem 1rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 72%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 86%, transparent);
    }

    .workspace-note strong {
      display: block;
      margin-bottom: 0.28rem;
      font-size: 0.95rem;
    }

    .workspace-note p {
      color: var(--ink-soft);
      font-size: 0.93rem;
      line-height: 1.45;
    }

    .node-list {
      display: grid;
      gap: 1rem;
    }

    .node-card {
      padding: 1.2rem;
    }

    .node-head {
      display: flex;
      justify-content: space-between;
      gap: 1rem;
      align-items: start;
      margin-bottom: 1rem;
    }

    .node-head .lede {
      margin-top: 0.45rem;
      color: var(--ink-soft);
      font-size: 1.02rem;
      line-height: 1.45;
      max-width: 60ch;
    }

    .state-box {
      display: grid;
      gap: 0.5rem;
      justify-items: end;
      text-align: right;
    }

    .state-pill {
      padding: 0.55rem 0.9rem;
      border-radius: 999px;
      font-size: 0.84rem;
      font-weight: 800;
      letter-spacing: 0.03em;
      white-space: nowrap;
    }

    .state-ready {
      background: var(--success-soft);
      color: var(--success);
    }

    .state-warn {
      background: var(--warn-soft);
      color: color-mix(in oklch, var(--warn) 75%, black);
    }

    .state-error {
      background: var(--danger-soft);
      color: var(--danger);
    }

    .state-caption {
      max-width: 28ch;
      color: var(--ink-soft);
      font-size: 0.92rem;
      line-height: 1.4;
    }

    .info-grid {
      display: grid;
      grid-template-columns: 1.1fr 1fr 1fr;
      gap: 0.85rem;
      margin-bottom: 1rem;
    }

    .info-card {
      padding: 1rem;
      border-radius: var(--radius-md);
      background: color-mix(in oklch, var(--panel-strong) 82%, var(--bg-soft));
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .info-card strong {
      display: block;
      margin-bottom: 0.35rem;
      font-size: 0.96rem;
    }

    .info-card p {
      color: var(--ink-soft);
      line-height: 1.48;
      font-size: 0.95rem;
    }

    .info-card.primary-note {
      border-color: color-mix(in oklch, var(--accent) 26%, var(--line));
      background: color-mix(in oklch, var(--accent) 6%, var(--panel-strong));
    }

    .metrics {
      display: grid;
      grid-template-columns: repeat(2, minmax(0, 1fr));
      gap: 0.9rem;
      margin-bottom: 0.75rem;
    }

    .metric {
      padding: 1rem;
      border-radius: var(--radius-md);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      background: color-mix(in oklch, var(--panel-strong) 82%, var(--bg-soft));
    }

    .metric header {
      display: flex;
      justify-content: space-between;
      gap: 0.6rem;
      align-items: baseline;
      margin-bottom: 0.72rem;
      color: var(--ink-soft);
      font-size: 0.82rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
    }

    .metric strong {
      display: block;
      font-size: clamp(1.95rem, 4vw, 2.9rem);
      line-height: 0.95;
      margin-bottom: 0.62rem;
    }

    .metric p {
      color: var(--ink-soft);
      font-size: 0.93rem;
      line-height: 1.45;
      margin-bottom: 0.7rem;
    }

    .track {
      width: 100%;
      height: 0.72rem;
      background: color-mix(in oklch, var(--line) 72%, transparent);
      border-radius: 999px;
      overflow: hidden;
    }

    .track > span {
      display: block;
      height: 100%;
      width: 0;
      border-radius: inherit;
      background: linear-gradient(90deg, color-mix(in oklch, var(--accent) 68%, white), var(--accent));
      transition: width 0.26s ease;
    }

    .metric-summary {
      margin-bottom: 1rem;
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.95rem;
    }

    .status-board {
      display: grid;
      grid-template-columns: repeat(4, minmax(0, 1fr));
      gap: 0.8rem;
      margin-bottom: 1rem;
    }

    .status-item {
      padding: 0.95rem 1rem;
      border-radius: var(--radius-md);
      background: var(--bg-soft);
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
    }

    .status-item span {
      display: block;
      margin-bottom: 0.3rem;
      font-size: 0.74rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
    }

    .status-item strong {
      display: block;
      margin-bottom: 0.28rem;
      font-size: 1rem;
    }

    .status-item p {
      color: var(--ink-soft);
      font-size: 0.9rem;
      line-height: 1.4;
    }

    .button-row {
      display: flex;
      flex-wrap: wrap;
      gap: 0.75rem;
      margin-bottom: 1rem;
    }

    button {
      appearance: none;
      border: 0;
      border-radius: 999px;
      padding: 0.88rem 1.15rem;
      font: inherit;
      font-weight: 800;
      cursor: pointer;
      transition: transform 0.18s ease, opacity 0.18s ease, background 0.18s ease;
      display: inline-flex;
      align-items: center;
      justify-content: center;
      gap: 0.4rem;
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
      background: color-mix(in oklch, var(--panel-strong) 76%, var(--bg-soft));
      border: 1px solid var(--line);
    }

    .counter-box,
    details {
      border: 1px solid color-mix(in oklch, var(--line) 84%, transparent);
      border-radius: var(--radius-lg);
      background: color-mix(in oklch, var(--panel-strong) 84%, var(--bg-soft));
    }

    .counter-box {
      padding: 1rem;
      margin-bottom: 1rem;
      display: grid;
      gap: 0.85rem;
    }

    .counter-head {
      display: flex;
      justify-content: space-between;
      gap: 1rem;
      align-items: start;
    }

    .counter-head p {
      color: var(--ink-soft);
      line-height: 1.45;
      font-size: 0.94rem;
      max-width: 54ch;
      margin-top: 0.25rem;
    }

    .counter-grid {
      display: grid;
      grid-template-columns: minmax(0, 1fr) auto auto;
      gap: 0.7rem;
      align-items: end;
    }

    label {
      display: grid;
      gap: 0.45rem;
      color: var(--ink);
      font-size: 0.92rem;
      font-weight: 700;
    }

    .field-copy {
      color: var(--ink-soft);
      font-size: 0.86rem;
      font-weight: 500;
      line-height: 1.35;
    }

    input[type="number"],
    select {
      width: 100%;
      border-radius: 0.95rem;
      border: 1px solid var(--line);
      padding: 0.8rem 0.9rem;
      font: inherit;
      color: var(--ink);
      background: var(--panel-strong);
    }

    .mini-card {
      min-width: 190px;
      padding: 0.9rem 1rem;
      border-radius: var(--radius-md);
      background: var(--panel-strong);
      border: 1px solid var(--line);
    }

    .mini-card span {
      display: block;
      font-size: 0.74rem;
      text-transform: uppercase;
      letter-spacing: 0.08em;
      color: var(--ink-soft);
      margin-bottom: 0.28rem;
    }

    .mini-card strong {
      font-size: 1rem;
      font-weight: 800;
    }

    details {
      overflow: hidden;
    }

    details summary {
      list-style: none;
      cursor: pointer;
      padding: 1rem 1rem 0.95rem;
      display: flex;
      justify-content: space-between;
      align-items: center;
      gap: 1rem;
      font-weight: 800;
    }

    details summary::-webkit-details-marker {
      display: none;
    }

    .advanced-copy {
      padding: 0 1rem 0.85rem;
      color: var(--ink-soft);
      line-height: 1.48;
      max-width: 70ch;
    }

    .tuning-grid {
      display: grid;
      grid-template-columns: repeat(3, minmax(0, 1fr));
      gap: 0.8rem;
      padding: 0 1rem 1rem;
    }

    .apply-row {
      padding: 0 1rem 1rem;
    }

    .node-footer {
      display: flex;
      justify-content: space-between;
      gap: 1rem;
      align-items: center;
      padding-top: 0.9rem;
      color: var(--ink-soft);
      font-size: 0.88rem;
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

    .empty {
      padding: 2rem;
      text-align: center;
      color: var(--ink-soft);
    }

    @media (max-width: 1080px) {
      .masthead,
      .guide-grid,
      .info-grid,
      .workspace-note {
        grid-template-columns: 1fr;
      }
    }

    @media (max-width: 920px) {
      .status-grid,
      .metrics,
      .status-board,
      .tuning-grid,
      .counter-grid {
        grid-template-columns: 1fr;
      }

      .node-head,
      .counter-head,
      .node-footer {
        flex-direction: column;
        align-items: start;
      }

      .state-box {
        justify-items: start;
        text-align: left;
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
        <p class="hero-copy" id="page-subtitle">Preparing calibration controls, live distances, and guided tuning.</p>

        <div class="guide-grid">
          <article class="guide-card">
            <h3>What this page is for</h3>
            <p>Use this page to confirm the doorway is clear, calibrate the sensor after installation, and check that direction and people count behave as expected.</p>
          </article>
          <article class="guide-card">
            <h3>Quick calibration guide</h3>
            <ol>
              <li><strong>Make the doorway empty.</strong> Nobody should stand in front of the sensor.</li>
              <li><strong>Wait for a ready state.</strong> The page will tell you when recalibration is safe.</li>
              <li><strong>Press Recalibrate sensor.</strong> Then walk through the doorway once to test.</li>
            </ol>
          </article>
        </div>
      </section>

      <aside class="top-tools">
        <button class="theme-toggle" type="button" id="theme-toggle">Switch to dark mode</button>
        <section class="panel top-summary">
          <div class="status-grid">
            <div class="status-chip">
              <span>Connection</span>
              <strong id="connection-copy">Connecting</strong>
            </div>
            <div class="status-chip">
              <span>Last sync</span>
              <strong id="last-sync">Waiting</strong>
            </div>
          </div>
          <div class="summary-note">
            <strong>Simple mode first</strong>
            <p>Most users only need the live status, Recalibrate sensor, and people count. Advanced tuning is kept lower on the page for installers.</p>
          </div>
        </section>
      </aside>
    </div>

    <div class="connection" id="connection-pill">Connecting to device</div>

    <section class="panel workspace-note">
      <article>
        <strong>How direction is detected</strong>
        <p>The sensor watches two measurement zones. The system compares which zone changes first to decide whether someone entered or exited.</p>
      </article>
      <article>
        <strong>How to read distance</strong>
        <p>Lower distance means something is closer to the sensor. A zone becomes active when the live distance drops below its threshold.</p>
      </article>
      <article>
        <strong>When to avoid calibration</strong>
        <p>Do not recalibrate while someone is in the doorway or while the sensor face is covered, dusty, or blocked by a hand or object.</p>
      </article>
    </section>

    <main class="node-list" id="node-list">
      <section class="panel empty">Loading sensor workspace...</section>
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

    function updateThemeLabel() {
      const dark = document.documentElement.dataset.theme === 'dark';
      themeToggle.textContent = dark ? 'Switch to light mode' : 'Switch to dark mode';
    }

    updateThemeLabel();

    themeToggle.addEventListener('click', () => {
      const next = document.documentElement.dataset.theme === 'dark' ? 'light' : 'dark';
      document.documentElement.dataset.theme = next;
      localStorage.setItem('roode-theme', next);
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

    function metricWidth(distance) {
      const clamped = Math.max(0, Math.min(4000, Number(distance) || 0));
      return `${(clamped / 4000) * 100}%`;
    }

    function stateClass(statusText) {
      if (/error/i.test(statusText)) return 'state-error';
      if (/blocked|occupied|wait/i.test(statusText)) return 'state-warn';
      return 'state-ready';
    }

    function readinessFor(node) {
      if (Number(node.sensor_status) > 0) {
        return {
          title: 'Sensor needs attention',
          detail: 'The sensor has reported an error. Check power, wiring, and the sensor face before changing any settings.',
          caption: 'Fix the sensor issue before recalibrating or tuning.',
        };
      }
      if (node.masking) {
        return {
          title: 'Sensor face looks blocked',
          detail: 'Something is very close to the sensor. Move hands, tape, packaging, or other objects away from the lens area.',
          caption: 'Clear the sensor face, then wait for a ready state.',
        };
      }
      if (node.presence) {
        return {
          title: 'Someone is in the doorway',
          detail: 'The sensor currently sees movement or a person in front of it. Wait until the doorway is empty before recalibrating.',
          caption: 'Recalibrate only when the space is clear and stable.',
        };
      }
      return {
        title: 'Ready for calibration',
        detail: 'The doorway is empty and the sensor is stable. This is the right time to recalibrate or confirm the live readings.',
        caption: 'You can safely recalibrate now.',
      };
    }

    function zoneMessage(distance, threshold) {
      if (Number(distance) <= Number(threshold)) {
        return 'This zone is currently inside the active detection range.';
      }
      return 'This zone is currently outside the active detection range.';
    }

    function presenceSummary(node) {
      return node.presence
        ? 'A person or movement is currently detected in front of the sensor.'
        : 'The monitored area is clear right now.';
    }

    function maskingSummary(node) {
      return node.masking
        ? 'The sensor face may be covered or too closely obstructed.'
        : 'No nearby masking is detected.';
    }

    function directionSummary(node) {
      if (node.last_direction === 'Entry') {
        return 'The most recent pass was counted as someone entering.';
      }
      if (node.last_direction === 'Exit') {
        return 'The most recent pass was counted as someone leaving.';
      }
      return 'No confirmed pass has been recorded since the last reset or reboot.';
    }

    function invertSummary(node) {
      return node.invert_direction
        ? 'Direction logic is currently reversed for this installation.'
        : 'Direction logic is using the normal sensor order.';
    }

    function renderNodes(payload) {
      pageTitle.textContent = payload.title || 'Roode Service';
      pageSubtitle.textContent = payload.subtitle || 'Check sensor health, recalibrate safely, and adjust field settings directly on the device.';

      if (!payload.nodes || !payload.nodes.length) {
        nodeList.innerHTML = '<section class="panel empty">No sensors are exposed to the UI yet.</section>';
        return;
      }

      nodeList.innerHTML = payload.nodes.map((node) => {
        const readiness = readinessFor(node);
        return `
          <section class="panel node-card" data-node="${node.index}">
            <div class="node-head">
              <div>
                <p class="eyebrow">Sensor workspace</p>
                <h2>${escapeHtml(node.label)}</h2>
                <p class="lede">${escapeHtml(readiness.detail)}</p>
              </div>
              <div class="state-box">
                <div class="state-pill ${stateClass(node.status_text)}">${escapeHtml(readiness.title)}</div>
                <p class="state-caption">${escapeHtml(readiness.caption)}</p>
              </div>
            </div>

            <div class="info-grid">
              <article class="info-card primary-note">
                <strong>What to do now</strong>
                <p>${escapeHtml(readiness.detail)}</p>
              </article>
              <article class="info-card">
                <strong>How the two zones work</strong>
                <p>Zone 0 and Zone 1 are the two watch areas used to detect direction. The system compares which zone changes first.</p>
              </article>
              <article class="info-card">
                <strong>Before changing settings</strong>
                <p>Use advanced tuning only if the count is unstable after a clean recalibration. For normal use, leave the installer settings alone.</p>
              </article>
            </div>

            <div class="metrics">
              <article class="metric">
                <header><span>Entry zone</span><span>Threshold ${node.entry_max_threshold} mm</span></header>
                <strong>${node.entry_distance} mm</strong>
                <p>${zoneMessage(node.entry_distance, node.entry_max_threshold)}</p>
                <div class="track"><span style="width:${metricWidth(node.entry_distance)}"></span></div>
              </article>
              <article class="metric">
                <header><span>Exit zone</span><span>Threshold ${node.exit_max_threshold} mm</span></header>
                <strong>${node.exit_distance} mm</strong>
                <p>${zoneMessage(node.exit_distance, node.exit_max_threshold)}</p>
                <div class="track"><span style="width:${metricWidth(node.exit_distance)}"></span></div>
              </article>
            </div>

            <p class="metric-summary">Live distance helps you judge whether the sensor has a clear view. Lower values mean something is closer to the sensor.</p>

            <div class="status-board">
              <article class="status-item">
                <span>Doorway</span>
                <strong>${node.presence ? 'Occupied' : 'Clear'}</strong>
                <p>${presenceSummary(node)}</p>
              </article>
              <article class="status-item">
                <span>Sensor face</span>
                <strong>${node.masking ? 'Possibly blocked' : 'Clear'}</strong>
                <p>${maskingSummary(node)}</p>
              </article>
              <article class="status-item">
                <span>Last direction</span>
                <strong>${escapeHtml(node.last_direction)}</strong>
                <p>${directionSummary(node)}</p>
              </article>
              <article class="status-item">
                <span>Direction mode</span>
                <strong>${node.invert_direction ? 'Reversed' : 'Normal'}</strong>
                <p>${invertSummary(node)}</p>
              </article>
            </div>

            <div class="button-row">
              <button class="primary" data-command="recalibrate" data-node="${node.index}" ${node.can_recalibrate ? '' : 'disabled'}>
                Recalibrate sensor
              </button>
              <button class="secondary" data-command="reset_counter" data-node="${node.index}">
                Reset count to 0
              </button>
            </div>

            <section class="counter-box">
              <div class="counter-head">
                <div>
                  <h3>Current people count</h3>
                  <p>Only change this manually if the displayed count is wrong and you want to correct it on the device.</p>
                </div>
              </div>

              <div class="counter-grid">
                <label>
                  People currently counted
                  <input type="number" min="0" max="50" step="1" value="${node.people_counter}" data-field="people_counter" data-node="${node.index}">
                </label>
                <div class="mini-card">
                  <span>Last direction</span>
                  <strong>${escapeHtml(node.last_direction)}</strong>
                </div>
                <button class="secondary" data-command="set_counter" data-node="${node.index}">
                  Save manual correction
                </button>
              </div>
            </section>

            <details>
              <summary>
                <span>Advanced tuning for installers</span>
                <span>${node.auto_recalibration_minutes > 0 ? `Auto every ${node.auto_recalibration_minutes} min` : 'Manual recalibration only'}</span>
              </summary>
              <p class="advanced-copy">These settings are mainly for installation and troubleshooting. After changing them, use Apply tuning to save the values and run a new recalibration.</p>

              <div class="tuning-grid">
                <label>
                  Auto recalibration (minutes)
                  <span class="field-copy">Set to 0 to recalibrate only when a person presses the button.</span>
                  <input type="number" min="0" max="720" step="5" value="${node.auto_recalibration_minutes}" data-field="auto_recalibration" data-node="${node.index}">
                </label>
                <label>
                  Sampling
                  <span class="field-copy">Higher sampling smooths noise but can make the sensor react a little slower.</span>
                  <input type="number" min="1" max="8" step="1" value="${node.sampling}" data-field="sampling" data-node="${node.index}">
                </label>
                <label>
                  Direction mode
                  <span class="field-copy">Use Reversed only if entry and exit are being counted the wrong way around.</span>
                  <select data-field="invert_direction" data-node="${node.index}">
                    <option value="false" ${node.invert_direction ? '' : 'selected'}>Normal</option>
                    <option value="true" ${node.invert_direction ? 'selected' : ''}>Reversed</option>
                  </select>
                </label>
                <label>
                  Minimum threshold (%)
                  <span class="field-copy">Lower values keep close objects from being ignored.</span>
                  <input type="number" min="0" max="40" step="1" value="${node.min_threshold_percent}" data-field="min_threshold" data-node="${node.index}">
                </label>
                <label>
                  Maximum threshold (%)
                  <span class="field-copy">Higher values make the zone react further away from the sensor.</span>
                  <input type="number" min="40" max="95" step="1" value="${node.max_threshold_percent}" data-field="max_threshold" data-node="${node.index}">
                </label>
                <label>
                  ROI width
                  <span class="field-copy">Adjusts how wide each watch area is across the doorway.</span>
                  <input type="number" min="4" max="16" step="1" value="${node.roi_width}" data-field="roi_width" data-node="${node.index}">
                </label>
                <label>
                  ROI height
                  <span class="field-copy">Adjusts how tall each watch area is inside the sensor image.</span>
                  <input type="number" min="4" max="16" step="1" value="${node.roi_height}" data-field="roi_height" data-node="${node.index}">
                </label>
                <label>
                  Minutes since calibration
                  <span class="field-copy">Useful for checking whether the latest recalibration already happened.</span>
                  <input type="number" value="${node.minutes_since_calibration}" disabled>
                </label>
                <label>
                  Minutes until automatic recalibration
                  <span class="field-copy">Shows when the next automatic recalibration is due.</span>
                  <input type="number" value="${node.minutes_until_auto_recalibration}" disabled>
                </label>
              </div>

              <div class="apply-row">
                <button class="primary" data-command="apply_tuning" data-node="${node.index}">
                  Apply tuning and recalibrate
                </button>
              </div>
            </details>

            <div class="node-footer">
              <span>${escapeHtml(node.calibration_hint)}</span>
              <span>Status code ${node.sensor_status}</span>
            </div>
          </section>
        `;
      }).join('');
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
        connectionPill.textContent = 'Connection lost. Retrying.';
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
