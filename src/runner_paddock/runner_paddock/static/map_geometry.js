"use strict";

// ROS OccupancyGrid geometry. Grid row zero is the origin-side (bottom) row;
// canvas row zero is the top row, so raster construction flips only the row.
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.PaddockMapGeometry = api;
}(typeof globalThis !== "undefined" ? globalThis : this, function () {
  function yawOf(orientation) {
    const q = orientation || {};
    const x = Number(q.x) || 0;
    const y = Number(q.y) || 0;
    const z = Number(q.z) || 0;
    const w = Number.isFinite(Number(q.w)) ? Number(q.w) : 1;
    return Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
  }

  function gridToWorld(grid, gx, gy) {
    const yaw = yawOf(grid.origin.orientation);
    const localX = gx * grid.resolution;
    const localY = gy * grid.resolution;
    const cosine = Math.cos(yaw);
    const sine = Math.sin(yaw);
    return {
      x: grid.origin.position.x + cosine * localX - sine * localY,
      y: grid.origin.position.y + sine * localX + cosine * localY,
    };
  }

  function worldToGrid(grid, x, y) {
    const yaw = yawOf(grid.origin.orientation);
    const dx = x - grid.origin.position.x;
    const dy = y - grid.origin.position.y;
    const cosine = Math.cos(yaw);
    const sine = Math.sin(yaw);
    return {
      x: (cosine * dx + sine * dy) / grid.resolution,
      y: (-sine * dx + cosine * dy) / grid.resolution,
    };
  }

  function gridBounds(grid) {
    const corners = [
      gridToWorld(grid, 0, 0),
      gridToWorld(grid, grid.width, 0),
      gridToWorld(grid, 0, grid.height),
      gridToWorld(grid, grid.width, grid.height),
    ];
    return {
      minX: Math.min(...corners.map((p) => p.x)),
      maxX: Math.max(...corners.map((p) => p.x)),
      minY: Math.min(...corners.map((p) => p.y)),
      maxY: Math.max(...corners.map((p) => p.y)),
    };
  }

  function screenToWorld(view, width, height, x, y) {
    const rotation = Number(view.rotation) || 0;
    const cosine = Math.cos(rotation);
    const sine = Math.sin(rotation);
    const rotatedX = (x - width / 2) / view.scale;
    const rotatedY = -(y - height / 2) / view.scale;
    return {
      x: view.x + cosine * rotatedX + sine * rotatedY,
      y: view.y - sine * rotatedX + cosine * rotatedY,
    };
  }

  function worldToScreen(view, width, height, x, y) {
    const rotation = Number(view.rotation) || 0;
    const cosine = Math.cos(rotation);
    const sine = Math.sin(rotation);
    const dx = x - view.x;
    const dy = y - view.y;
    return {
      x: width / 2 + (cosine * dx - sine * dy) * view.scale,
      y: height / 2 - (sine * dx + cosine * dy) * view.scale,
    };
  }

  function rotatedGridBounds(grid, rotation) {
    const cosine = Math.cos(rotation || 0);
    const sine = Math.sin(rotation || 0);
    const corners = [
      gridToWorld(grid, 0, 0),
      gridToWorld(grid, grid.width, 0),
      gridToWorld(grid, 0, grid.height),
      gridToWorld(grid, grid.width, grid.height),
    ].map((point) => ({
      x: cosine * point.x - sine * point.y,
      y: sine * point.x + cosine * point.y,
    }));
    return {
      minX: Math.min(...corners.map((p) => p.x)),
      maxX: Math.max(...corners.map((p) => p.x)),
      minY: Math.min(...corners.map((p) => p.y)),
      maxY: Math.max(...corners.map((p) => p.y)),
    };
  }

  return {
    yawOf, gridToWorld, worldToGrid, gridBounds, rotatedGridBounds,
    screenToWorld, worldToScreen,
  };
}));
