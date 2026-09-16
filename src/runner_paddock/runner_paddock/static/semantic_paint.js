"use strict";

// Pure semantic-brush geometry and stroke bookkeeping: disk cell selection,
// drag-segment interpolation, undo application, and the grid<->image row
// flip shared with the backend's occupancy/semantics convention (row 0 of
// the grid is the origin/bottom row; row 0 of the on-disk image is the top
// row). No DOM, no fetch, no rendering -- safe to unit test directly.
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.PaddockSemanticPaint = api;
}(typeof globalThis !== "undefined" ? globalThis : this, function () {
  function paintCell(data, width, height, gx, gy, value, edits) {
    if (gx < 0 || gy < 0 || gx >= width || gy >= height) return;
    const index = gy * width + gx;
    if (data[index] === value) return;
    if (!edits.has(index)) edits.set(index, data[index]);
    data[index] = value;
  }

  function paintDisk(data, width, height, centerX, centerY, radiusCells, value, edits) {
    const r = Math.round(radiusCells);
    const rSquared = r * r;
    const cx = Math.round(centerX);
    const cy = Math.round(centerY);
    for (let dy = -r; dy <= r; dy += 1) {
      for (let dx = -r; dx <= r; dx += 1) {
        if (dx * dx + dy * dy > rSquared) continue;
        paintCell(data, width, height, cx + dx, cy + dy, value, edits);
      }
    }
  }

  // Points along one drag segment, spaced no farther apart than
  // stepDistance, so continuous dragging never leaves gaps in the stroke.
  function segmentSteps(from, to, stepDistance) {
    const distance = Math.hypot(to.x - from.x, to.y - from.y);
    const steps = Math.max(1, Math.ceil(distance / stepDistance));
    const points = [];
    for (let i = 0; i <= steps; i += 1) {
      const t = i / steps;
      points.push({ x: from.x + (to.x - from.x) * t, y: from.y + (to.y - from.y) * t });
    }
    return points;
  }

  function applyUndo(data, edits) {
    for (const [index, previousValue] of edits) {
      data[index] = previousValue;
    }
  }

  // Its own inverse: converts grid row-major (row 0 = origin) to on-disk
  // image order (row 0 = top), and back, identically.
  function flipRows(pixels, width, height) {
    const out = new Uint8Array(pixels.length);
    for (let row = 0; row < height; row += 1) {
      const source = row * width;
      const destination = (height - 1 - row) * width;
      out.set(pixels.subarray(source, source + width), destination);
    }
    return out;
  }

  return { paintCell, paintDisk, segmentSteps, applyUndo, flipRows };
}));
