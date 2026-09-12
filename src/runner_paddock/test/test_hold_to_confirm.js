"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const { HoldToConfirm } = require("../runner_paddock/static/hold_to_confirm.js");

function harness() {
  let now = 0;
  let timer = null;
  let completed = 0;
  const progress = [];
  const hold = new HoldToConfirm({
    durationMs: 2000,
    now: () => now,
    setTimer: (callback) => { timer = callback; return 1; },
    clearTimer: () => { timer = null; },
    requestFrame: () => 2,
    cancelFrame: () => {},
    onProgress: (value, active) => progress.push([value, active]),
    onComplete: () => { completed += 1; },
  });
  return {
    hold,
    progress,
    completed: () => completed,
    advance: (milliseconds) => { now += milliseconds; },
    fireTimer: () => { if (timer) timer(); },
  };
}

test("releasing before two seconds cancels without completing", () => {
  const state = harness();
  state.hold.start();
  state.advance(1999);
  state.hold.tick();
  state.hold.cancel();
  state.fireTimer();

  assert.equal(state.completed(), 0);
  assert.deepEqual(state.progress.at(-1), [0, false]);
});

test("a continuous two-second hold completes exactly once", () => {
  const state = harness();
  state.hold.start();
  state.advance(2000);
  state.hold.tick();
  state.fireTimer();
  state.fireTimer();

  assert.equal(state.completed(), 1);
  assert.deepEqual(state.progress.at(-1), [1, false]);
});
