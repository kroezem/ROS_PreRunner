"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const semanticPaint = require("../runner_paddock/static/semantic_paint.js");

test("paintDisk paints a disk of the requested radius and records prior values", () => {
  const width = 7;
  const height = 7;
  const data = new Uint8Array(width * height);
  const edits = new Map();

  semanticPaint.paintDisk(data, width, height, 3, 3, 2, 1, edits);

  // Center and both axis-aligned points at radius 2 are painted...
  assert.equal(data[3 * width + 3], 1);
  assert.equal(data[3 * width + 5], 1);
  assert.equal(data[1 * width + 3], 1);
  // ...but a corner strictly outside the disk (distance sqrt(8) > 2) is not.
  assert.equal(data[1 * width + 1], 0);
  // Every painted cell's original value (0) was recorded for undo.
  for (const [, previousValue] of edits) assert.equal(previousValue, 0);
  assert.ok(edits.size > 0);
});

test("paintDisk does not record an edit for a cell already at the target value", () => {
  const width = 3;
  const height = 3;
  const data = new Uint8Array(width * height).fill(2);
  const edits = new Map();

  semanticPaint.paintDisk(data, width, height, 1, 1, 5, 2, edits);

  assert.equal(edits.size, 0);
  assert.ok(data.every((value) => value === 2));
});

test("paintDisk clips at grid boundaries without touching out-of-range cells", () => {
  const width = 4;
  const height = 4;
  const data = new Uint8Array(width * height);
  const edits = new Map();

  assert.doesNotThrow(() => {
    semanticPaint.paintDisk(data, width, height, 0, 0, 3, 1, edits);
  });
  for (const index of edits.keys()) {
    assert.ok(index >= 0 && index < width * height);
  }
});

test("applyUndo restores exactly the recorded cells and nothing else", () => {
  const width = 4;
  const height = 4;
  const data = new Uint8Array(width * height).fill(0);
  const edits = new Map();
  semanticPaint.paintDisk(data, width, height, 1, 1, 1, 2, edits);
  const paintedSnapshot = data.slice();
  assert.ok(paintedSnapshot.some((value) => value === 2));

  semanticPaint.applyUndo(data, edits);

  assert.ok(data.every((value) => value === 0));
});

test("segmentSteps covers a zero-length drag with a single point", () => {
  const point = { x: 1.5, y: -2.25 };
  const steps = semanticPaint.segmentSteps(point, point, 0.05);

  assert.ok(steps.length >= 1);
  for (const step of steps) {
    assert.equal(step.x, point.x);
    assert.equal(step.y, point.y);
  }
});

test("segmentSteps never leaves gaps larger than the requested step distance", () => {
  const from = { x: 0, y: 0 };
  const to = { x: 1, y: 0.4 };
  const stepDistance = 0.05;
  const steps = semanticPaint.segmentSteps(from, to, stepDistance);

  assert.deepEqual(steps[0], from);
  assert.deepEqual(steps[steps.length - 1], to);
  for (let i = 1; i < steps.length; i += 1) {
    const gap = Math.hypot(steps[i].x - steps[i - 1].x, steps[i].y - steps[i - 1].y);
    assert.ok(gap <= stepDistance + 1e-9);
  }
});

test("flipRows moves grid row 0 to the last image row, matching the occupancy convention", () => {
  const width = 3;
  const height = 2;
  // Grid row-major (row 0 = origin): row 0 is [1,2,3], row 1 is [4,5,6].
  const gridOrder = new Uint8Array([1, 2, 3, 4, 5, 6]);

  const imageOrder = semanticPaint.flipRows(gridOrder, width, height);

  // Image order (row 0 = top) must have grid row 1 first, grid row 0 last --
  // the same flip OccupancyRaster.to_pgm_bytes applies on the Python side.
  assert.deepEqual(Array.from(imageOrder), [4, 5, 6, 1, 2, 3]);
});

test("flipRows is its own inverse", () => {
  const width = 5;
  const height = 4;
  const original = new Uint8Array(width * height).map((_, i) => i % 3);

  const roundTripped = semanticPaint.flipRows(
    semanticPaint.flipRows(original, width, height), width, height,
  );

  assert.deepEqual(Array.from(roundTripped), Array.from(original));
});
