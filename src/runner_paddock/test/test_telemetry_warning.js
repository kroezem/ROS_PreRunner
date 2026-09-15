// Copyright 2026 matti
// SPDX-License-Identifier: Apache-2.0

"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const warning = require("../runner_paddock/static/telemetry_warning.js");

const availableFresh = { available: true, fresh: true };

test("CPU warns only above 90", () => {
  const cpuWarning = (value) => value > 90;
  assert.equal(warning.isActive(availableFresh, true, 90, cpuWarning), false);
  assert.equal(warning.isActive(availableFresh, true, 90.1, cpuWarning), true);
});

test("battery warns only below 3.20", () => {
  const batteryWarning = (value) => value < 3.20;
  assert.equal(warning.isActive(availableFresh, true, 3.20, batteryWarning), false);
  assert.equal(warning.isActive(availableFresh, true, 3.19, batteryWarning), true);
});

test("missing, invalid, stale, and non-finite telemetry never warns", () => {
  const alwaysViolated = () => true;
  assert.equal(warning.isActive(undefined, true, 100, alwaysViolated), false);
  assert.equal(warning.isActive({ available: false, fresh: true }, true, 100, alwaysViolated), false);
  assert.equal(warning.isActive({ available: true, fresh: false }, true, 100, alwaysViolated), false);
  assert.equal(warning.isActive(availableFresh, false, 100, alwaysViolated), false);
  assert.equal(warning.isActive(availableFresh, true, NaN, alwaysViolated), false);
});
