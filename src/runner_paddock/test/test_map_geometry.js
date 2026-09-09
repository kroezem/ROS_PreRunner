"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const geometry = require("../runner_paddock/static/map_geometry.js");

test("known rotated grid cell round-trips through map coordinates", () => {
  const yaw = Math.PI / 6;
  const grid = {
    width: 40,
    height: 30,
    resolution: 0.05,
    origin: {
      position: { x: -1.25, y: 2.5 },
      orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
    },
  };
  // Cell (7, 11) center, not its lower-left edge.
  const gridPoint = { x: 7.5, y: 11.5 };
  const expected = {
    x: -1.25 + Math.cos(yaw) * 0.375 - Math.sin(yaw) * 0.575,
    y: 2.5 + Math.sin(yaw) * 0.375 + Math.cos(yaw) * 0.575,
  };
  const world = geometry.gridToWorld(grid, gridPoint.x, gridPoint.y);
  assert.ok(Math.abs(world.x - expected.x) < 1e-12);
  assert.ok(Math.abs(world.y - expected.y) < 1e-12);
  const inverse = geometry.worldToGrid(grid, world.x, world.y);
  assert.ok(Math.abs(inverse.x - gridPoint.x) < 1e-12);
  assert.ok(Math.abs(inverse.y - gridPoint.y) < 1e-12);
});

test("goal click pixel converts to map coordinates and back", () => {
  const view = { x: 1, y: -2, scale: 100 };

  const world = geometry.screenToWorld(view, 800, 600, 500, 100);

  assert.deepEqual(world, { x: 2, y: 0 });
  assert.deepEqual(
    geometry.worldToScreen(view, 800, 600, world.x, world.y),
    { x: 500, y: 100 },
  );
});
