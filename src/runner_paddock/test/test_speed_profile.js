const assert = require("node:assert/strict");
const test = require("node:test");
const profile = require("../runner_paddock/static/speed_profile.js");

test("default geometry spans constrained to preset speed", () => {
  const settings = profile.sanitize(profile.DEFAULTS);
  assert.equal(settings.minSpeed, 0.50);
  assert.equal(profile.geometrySpeed(settings.tightThreshold, profile.PRESETS[1], settings), 0.50);
  assert.equal(profile.geometrySpeed(settings.openThreshold, profile.PRESETS[1], settings), 1.00);
  const linear = { ...settings, curveFamily: "linear" };
  assert.ok(Math.abs(profile.curveProgress(0.4, linear) - 0.5) < 1e-12);
});

test("scaling ranges from shared to preset-dependent constrained speed", () => {
  const settings = profile.sanitize(profile.DEFAULTS);
  const shared = { ...settings, presetScaling: 0 };
  assert.equal(profile.constrainedSpeed(profile.PRESETS[1], shared), 0.50);
  assert.equal(profile.constrainedSpeed(profile.PRESETS[3], shared), 0.50);
  const scaled = { ...settings, presetScaling: 100 };
  assert.equal(profile.constrainedSpeed(profile.PRESETS[1], scaled), 1.00);
  assert.equal(profile.constrainedSpeed(profile.PRESETS[3], scaled), 2.00);
});

test("synthetic path uses settled distance and never exceeds raw ceiling", () => {
  const settings = profile.sanitize(profile.DEFAULTS);
  for (const preset of profile.PRESETS) {
    const result = profile.pathProfile(preset, settings);
    assert.equal(result.points.length, 301);
    assert.ok(Math.abs(result.settledDistance - result.target * settings.approachTime) < 1e-12);
    assert.ok(result.points.every((point) => point.final <= point.raw + 1e-9));
    assert.ok(result.points.every((point) => point.final <= preset.maxSpeed + 1e-9));
  }
});
