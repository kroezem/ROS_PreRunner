"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const geometry = require("../runner_paddock/static/joystick_geometry.js");

const cases = [
  ["full forward", 0, -1, 0.40, 0],
  ["full right", 1, 0, 0, -1],
  ["full forward-right", 1, -1, 0.40, -1],
  ["full forward-left", -1, -1, 0.40, 1],
  ["full reverse-right", 1, 1, -0.40, -1],
  ["full reverse-left", -1, 1, -0.40, 1],
];

for (const [name, x, y, speed, steering] of cases) {
  test(`${name} uses independent square axes`, () => {
    const demand = geometry.demandFromAxes(x, y, 0.40);
    assert.equal(demand.speed_mps, speed);
    assert.equal(demand.steering, steering);
  });
}

test("axes clamp independently outside the square", () => {
  assert.deepEqual(geometry.demandFromAxes(2, -3, 0.40), {
    x: 1, y: -1, speed_mps: 0.40, steering: -1,
  });
});
