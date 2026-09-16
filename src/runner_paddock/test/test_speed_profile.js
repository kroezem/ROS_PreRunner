const assert = require("node:assert/strict");
const test = require("node:test");
const profile = require("../runner_paddock/static/speed_profile.js");

const settings = {
  desired_linear_vel: 1, minimum_traversal_speed: 0.25,
  tight_clearance: 0.05, open_clearance: 0.7,
  clearance_curve_family: 2, clearance_curve_shape: 1,
};
const closeTo = (actual, expected, tolerance = 1e-9) => assert.ok(
  Math.abs(actual - expected) < tolerance,
  `expected ${actual} to be within ${tolerance} of ${expected}`,
);

test("curve endpoints match tight and open policy", () => {
  assert.equal(profile.geometrySpeed(settings.open_clearance, settings), settings.desired_linear_vel);
  assert.equal(profile.geometrySpeed(settings.tight_clearance, settings), settings.minimum_traversal_speed);
});

test("minimum traversal speed is the bottom end of the curve directly, with no preset-relative scaling", () => {
  const custom = { ...settings, minimum_traversal_speed: 0.6, desired_linear_vel: 1.5 };
  assert.equal(profile.geometrySpeed(custom.tight_clearance, custom), 0.6);
});

test("only one field, no PRESETS array or preset matching, drives the curve", () => {
  assert.equal(profile.PRESETS, undefined);
});

test("power and smoothstep shapes are selectable", () => {
  const clearance = (settings.tight_clearance + settings.open_clearance) / 2;
  closeTo(profile.curveProgress(clearance, { ...settings, clearance_curve_family: 0 }), 0.5);
  closeTo(profile.curveProgress(clearance, { ...settings, clearance_curve_family: 1, clearance_curve_shape: 2 }), 0.25);
  closeTo(profile.curveProgress(clearance, { ...settings, clearance_curve_family: 2 }), 0.5);
});

// Regression coverage for: drag a slider, release it, click elsewhere with
// no Apply/Save — the edit must survive blur and every subsequent
// paddock-tuning broadcast until Apply resolves or fails it.
test("draft store: an edited field survives repeated confirmed broadcasts (blur is not discard)", () => {
  const store = profile.createDraftStore({ cost_penalty: 2.0, minimum_traversal_speed: 0.25 });
  store.edit("cost_penalty", 50);
  assert.ok(store.isDirty("cost_penalty"));

  // Simulate several ROS tuning refreshes (e.g. >5s of 10 Hz broadcasts,
  // or TUNING_REFRESH_SEC ticks) still reporting the old live value.
  for (let i = 0; i < 5; i += 1) {
    store.receiveConfirmed({ cost_penalty: 2.0, minimum_traversal_speed: 0.25 });
  }
  assert.equal(store.values().cost_penalty, 50, "dirty edit must not snap back on blur/broadcast");
  assert.ok(store.isDirty("cost_penalty"));
});

test("draft store: unedited fields keep following confirmed ROS updates", () => {
  const store = profile.createDraftStore({ cost_penalty: 2.0, minimum_traversal_speed: 0.25 });
  store.edit("cost_penalty", 50);
  store.receiveConfirmed({ cost_penalty: 2.0, minimum_traversal_speed: 0.40 });
  assert.equal(store.values().cost_penalty, 50, "dirty field stays on the draft");
  assert.equal(store.values().minimum_traversal_speed, 0.40, "clean field follows confirmed state");
});

test("draft store: a verified-applied read-back clears dirty and adopts confirmed values", () => {
  const store = profile.createDraftStore({ cost_penalty: 2.0 });
  store.edit("cost_penalty", 50);
  store.resolveApplied({ cost_penalty: 50 });
  assert.equal(store.values().cost_penalty, 50);
  assert.ok(!store.isDirty("cost_penalty"));
});

test("draft store: a failed apply keeps the draft dirty instead of reverting", () => {
  const store = profile.createDraftStore({ cost_penalty: 2.0 });
  store.edit("cost_penalty", 50);
  // A failed apply's read-back reports the unchanged live value; it must
  // never be treated as a resolution.
  store.receiveConfirmed({ cost_penalty: 2.0 });
  assert.equal(store.values().cost_penalty, 50, "failed apply must not discard the draft");
  assert.ok(store.isDirty("cost_penalty"));
});

test("draft store: loading a saved profile marks every loaded field dirty without an apply cycle", () => {
  const store = profile.createDraftStore({ desired_linear_vel: 1, minimum_traversal_speed: 0.25, cost_penalty: 2.0 });
  store.load({ desired_linear_vel: 1.5, minimum_traversal_speed: 0.4 });
  assert.equal(store.values().desired_linear_vel, 1.5);
  assert.equal(store.values().minimum_traversal_speed, 0.4);
  assert.ok(store.isDirty("desired_linear_vel"));
  assert.ok(store.isDirty("minimum_traversal_speed"));
  // cost_penalty (Planner Settings) is untouched by a Speed Profile load.
  assert.equal(store.values().cost_penalty, 2.0);
  assert.ok(!store.isDirty("cost_penalty"));
});

test("draft store: a loaded field survives confirmed broadcasts just like a manual edit, until Apply", () => {
  const store = profile.createDraftStore({ desired_linear_vel: 1 });
  store.load({ desired_linear_vel: 1.5 });
  store.receiveConfirmed({ desired_linear_vel: 1 });
  assert.equal(store.values().desired_linear_vel, 1.5, "load must not silently apply, but must survive broadcasts until Apply");
});

test("SPEED_PROFILE_FIELDS excludes cost_penalty (Planner Settings is never part of a named profile)", () => {
  assert.ok(!profile.SPEED_PROFILE_FIELDS.includes("cost_penalty"));
  assert.ok(profile.SPEED_PROFILE_FIELDS.includes("desired_linear_vel"));
  assert.ok(profile.SPEED_PROFILE_FIELDS.includes("minimum_traversal_speed"));
});

test("shouldResolveApplied only fires on the applying->applied rising edge", () => {
  assert.equal(profile.shouldResolveApplied("applied", "applying"), true);
  assert.equal(profile.shouldResolveApplied("applied", "applied"), false, "steady-state applied must not re-resolve a later edit");
  assert.equal(profile.shouldResolveApplied("applied", null), true, "first observed applied (e.g. page load) resolves once");
  assert.equal(profile.shouldResolveApplied("failed", "applying"), false);
  assert.equal(profile.shouldResolveApplied("applying", "applied"), false);
});

test("draft store: a whole apply/resolve cycle matches the expected UI model", () => {
  const store = profile.createDraftStore({ cost_penalty: 2.0 });
  let lastStatus = null;

  // User edits, then broadcasts keep arriving before they click Apply.
  store.edit("cost_penalty", 50);
  for (const status of ["current", "current", "current"]) {
    if (profile.shouldResolveApplied(status, lastStatus)) store.resolveApplied({ cost_penalty: 2.0 });
    else store.receiveConfirmed({ cost_penalty: 2.0 });
    lastStatus = status;
  }
  assert.equal(store.values().cost_penalty, 50);

  // Apply clicked: applying -> applied, with the confirmed read-back now
  // matching what was requested.
  for (const status of ["applying", "applied"]) {
    if (profile.shouldResolveApplied(status, lastStatus)) store.resolveApplied({ cost_penalty: 50 });
    else store.receiveConfirmed({ cost_penalty: 50 });
    lastStatus = status;
  }
  assert.equal(store.values().cost_penalty, 50);
  assert.ok(!store.isDirty("cost_penalty"));

  // A later, unrelated broadcast still reporting "applied" must not touch
  // a brand-new edit.
  store.edit("cost_penalty", 75);
  if (profile.shouldResolveApplied("applied", lastStatus)) store.resolveApplied({ cost_penalty: 50 });
  else store.receiveConfirmed({ cost_penalty: 50 });
  assert.equal(store.values().cost_penalty, 75, "new edit must survive a stale steady-state applied broadcast");
});
