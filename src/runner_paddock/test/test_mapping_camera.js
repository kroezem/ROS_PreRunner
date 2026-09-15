"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const geometry = require("../runner_paddock/static/map_geometry.js");
const mappingCamera = require("../runner_paddock/static/mapping_camera.js");

function pose(x, y, yaw) {
  return {
    position: { x, y },
    orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
  };
}

test("mapping camera centers Runner with its forward heading screen-up", () => {
  const runner = pose(4.2, -1.3, -0.7);
  const view = mappingCamera.viewForPose(runner, 80, geometry.yawOf);
  const center = geometry.worldToScreen(view, 800, 600, 4.2, -1.3);
  const ahead = geometry.worldToScreen(
    view, 800, 600,
    4.2 + Math.cos(-0.7), -1.3 + Math.sin(-0.7),
  );

  assert.deepEqual(center, { x: 400, y: 300 });
  assert.ok(Math.abs(ahead.x - center.x) < 1e-10);
  assert.ok(ahead.y < center.y);
});

test("mapping zoom is independent and remains unchanged across poses", () => {
  const autonomyView = { x: 8, y: 9, scale: 125, rotation: 0.4, fitted: true };
  const selectedScale = mappingCamera.zoomScale(50, 1.25);
  const first = mappingCamera.viewForPose(pose(1, 2, 0), selectedScale, geometry.yawOf);
  const moved = mappingCamera.viewForPose(pose(6, -3, 1.1), selectedScale, geometry.yawOf);

  assert.equal(first.scale, 62.5);
  assert.equal(moved.scale, 62.5);
  assert.deepEqual(autonomyView, { x: 8, y: 9, scale: 125, rotation: 0.4, fitted: true });
});
