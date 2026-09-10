"use strict";

// Convert screen-space joystick axes into browser-manual semantic demand.
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.PaddockJoystickGeometry = api;
}(typeof globalThis !== "undefined" ? globalThis : this, function () {
  function clampAxis(value) {
    return Math.max(-1, Math.min(1, Number(value) || 0));
  }

  function demandFromAxes(rawX, rawY, maxSpeedMps) {
    const x = clampAxis(rawX);
    const y = clampAxis(rawY);
    const ceiling = Math.max(0, Number(maxSpeedMps) || 0);
    return {
      x,
      y,
      speed_mps: y === 0 ? 0 : -y * ceiling,
      // Screen x grows rightward. The established normalized vehicle command
      // convention requires a negative demand for physical right steering.
      steering: x === 0 ? 0 : -x,
    };
  }

  return { demandFromAxes };
}));
