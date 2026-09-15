"use strict";

// Presentation-only mapping camera. Pose owns position and heading; the
// operator owns only zoom while the mapping runtime is active.
(function (root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  root.PaddockMappingCamera = api;
}(typeof globalThis !== "undefined" ? globalThis : this, function () {
  const MIN_SCALE = 2;
  const MAX_SCALE = 5000;

  function viewForPose(pose, scale, yawOf) {
    if (!pose || !pose.position || !pose.orientation ||
        !Number.isFinite(pose.position.x) || !Number.isFinite(pose.position.y)) return null;
    const yaw = yawOf(pose.orientation);
    if (!Number.isFinite(yaw)) return null;
    return {
      x: pose.position.x,
      y: pose.position.y,
      scale: Math.min(MAX_SCALE, Math.max(MIN_SCALE, scale)),
      rotation: Math.PI / 2 - yaw,
    };
  }

  function zoomScale(scale, factor) {
    return Math.min(MAX_SCALE, Math.max(MIN_SCALE, scale * factor));
  }

  return { viewForPose, zoomScale };
}));
