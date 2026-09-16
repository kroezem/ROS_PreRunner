const assert = require("node:assert/strict");
const test = require("node:test");
const profile = require("../runner_paddock/static/speed_profile.js");

const settings = {
  minimum_traversal_speed: 0.25, constrained_speed_scaling: 0.2,
  scaling_reference_speed: 2, tight_clearance: 0.05, open_clearance: 0.7,
  clearance_curve_family: 2, clearance_curve_shape: 1,
};
const closeTo = (actual, expected, tolerance = 1e-9) => assert.ok(
  Math.abs(actual - expected) < tolerance,
  `expected ${actual} to be within ${tolerance} of ${expected}`,
);

test("curve endpoints match tight and open policy", () => {
  const preset = profile.PRESETS[1];
  assert.equal(profile.geometrySpeed(settings.open_clearance, preset, settings), 1);
  assert.equal(profile.geometrySpeed(settings.tight_clearance, preset, settings), profile.constrainedSpeed(preset, settings));
});

test("fully scaled bottom remains below each preset maximum", () => {
  const scaled = { ...settings, constrained_speed_scaling: 1 };
  for (const preset of profile.PRESETS) {
    closeTo(
      profile.constrainedSpeed(preset, scaled),
      scaled.minimum_traversal_speed * preset.maxSpeed / scaled.scaling_reference_speed,
    );
    assert.ok(profile.constrainedSpeed(preset, scaled) < preset.maxSpeed);
  }
});

test("fully scaled bottom follows an explicit scaling reference, not a hardcoded one", () => {
  const scaled = { ...settings, constrained_speed_scaling: 1, scaling_reference_speed: 4 };
  const preset = profile.PRESETS[1];
  closeTo(
    profile.constrainedSpeed(preset, scaled),
    scaled.minimum_traversal_speed * preset.maxSpeed / 4,
  );
});

test("power and smoothstep shapes are selectable", () => {
  const clearance = (settings.tight_clearance + settings.open_clearance) / 2;
  closeTo(profile.curveProgress(clearance, { ...settings, clearance_curve_family: 0 }), 0.5);
  closeTo(profile.curveProgress(clearance, { ...settings, clearance_curve_family: 1, clearance_curve_shape: 2 }), 0.25);
  closeTo(profile.curveProgress(clearance, { ...settings, clearance_curve_family: 2 }), 0.5);
});
