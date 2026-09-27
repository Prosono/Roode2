'use strict';
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../components/tof_overdoor_ui/tof_overdoor_ui.cpp'), 'utf8');
const declarations = source.slice(source.indexOf('    let refreshTimer = null;'), source.indexOf('    const rememberedTheme'));
const functions = source.slice(source.indexOf('    async function fetchJson('), source.indexOf("    document.addEventListener('click'"));

function rig() {
  const timers = new Map();
  const requests = [];
  const rendered = [];
  let timerId = 0;
  const context = {
    AbortController, Date, Math,
    stateUrl: '/tof-overdoor-ui/state', actionUrl: '/tof-overdoor-ui/action',
    connectionPill: {}, connectionCopy: {}, lastSync: {}, document: {hidden: false},
    renderState: (value, options) => rendered.push({value, options}), showToast() {},
    setTimeout(fn, delay) { timers.set(++timerId, {fn, delay}); return timerId; },
    clearTimeout(id) { timers.delete(id); },
    fetch(url, options) {
      return new Promise((resolve, reject) => {
        const request = {url, options, resolve, reject};
        options.signal.addEventListener('abort', () => reject(new Error('aborted')), {once: true});
        requests.push(request);
      });
    },
  };
  vm.createContext(context);
  vm.runInContext(declarations + functions + `\nthis.api = {
    refresh, scheduleRefresh, refreshDelay, pausePollingForAction, postParams,
    setBusy(value) { busy = value; }, failures() { return failedRefreshes; }
  };`, context);
  return {context, timers, requests, rendered, api: context.api};
}
const success = request => request.resolve({ok: true, json: async () => ({ready: true})});

(async () => {
  {
    const r = rig();
    const first = r.api.refresh();
    const joined = r.api.refresh(true);
    assert.equal(first, joined);
    assert.equal(r.requests.length, 1);
    success(r.requests[0]); await first;
    assert.equal(r.rendered.length, 1);
    assert.equal(r.rendered[0].options.forceSettingsSync, true);
    assert.equal(r.timers.size, 1);
    assert.equal([...r.timers.values()][0].delay, 500);
    // An action waits for any existing GET and suppresses scheduled polls.
    const second = r.api.refresh();
    r.api.setBusy(true);
    let actionReady = false;
    const paused = r.api.pausePollingForAction().then(() => { actionReady = true; });
    await Promise.resolve(); assert.equal(actionReady, false);
    success(r.requests[1]); await second; await paused;
    assert.equal(actionReady, true); assert.equal(r.timers.size, 0);
  }
  {
    const r = rig();
    const pending = r.api.refresh();
    const timeout = [...r.timers.entries()].find(([, timer]) => timer.delay === 5000);
    assert.ok(timeout);
    r.timers.delete(timeout[0]); timeout[1].fn(); await pending;
    assert.equal(r.requests[0].options.signal.aborted, true);
    assert.equal(r.api.failures(), 1);
    assert.equal(r.api.refreshDelay(), 1500);
    // A hung request must release the in-flight slot and allow recovery.
    const recovery = r.api.refresh(); assert.equal(r.requests.length, 2);
    success(r.requests[1]); await recovery;
    assert.equal(r.api.failures(), 0);
  }
  {
    const r = rig();
    for (let i = 0; i < 12; ++i) {
      const pending = r.api.refresh();
      r.requests[i].reject(new Error('offline')); await pending;
    }
    assert.equal(r.api.failures(), 5);
    assert.equal(r.api.refreshDelay(), 24000);
    assert.equal(r.timers.size, 1);
    r.context.document.hidden = true;
    r.api.scheduleRefresh(); assert.equal(r.timers.size, 0);
    r.context.document.hidden = false;
    r.api.scheduleRefresh(); assert.equal(r.timers.size, 1);
  }
  {
    const r = rig();
    const pending = r.api.postParams('action=reset_counts');
    assert.equal(r.requests[0].url, '/tof-overdoor-ui/action');
    assert.equal(r.requests[0].options.method, 'POST');
    assert.equal(r.requests[0].options.body, 'action=reset_counts');
    success(r.requests[0]); await pending;
    assert.equal(r.timers.size, 0);
  }
  console.log('UI polling: single flight, action serialization, timeout recovery, hidden pause and backoff passed');
})().catch(error => { console.error(error); process.exitCode = 1; });
