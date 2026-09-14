"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const route = require("../runner_paddock/static/route_visualization.js");

test("rainbow scale is normalized by the authoritative profile ceiling", () => {
  assert.equal(route.rainbowColor(0, 1.5), "hsl(240 100% 50%)");
  assert.equal(route.rainbowColor(0.75, 1.5), "hsl(120 100% 50%)");
  assert.equal(route.rainbowColor(1.5, 1.5), "hsl(0 100% 50%)");
  assert.equal(route.rainbowColor(2.0, 1.5), "hsl(0 100% 50%)");
});

test("route color ignores direction and interpolates adjacent points", () => {
  assert.equal(route.rainbowColor(-0.5, 1.0), route.rainbowColor(0.5, 1.0));
  assert.equal(route.segmentSpeed([
    { speed_ceiling_mps: -0.2 }, { speed_ceiling_mps: 0.6 },
  ], 0), 0.4);
});
