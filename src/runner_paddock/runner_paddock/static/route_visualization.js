"use strict";

(function exportRouteVisualization(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.PaddockRouteVisualization = api;
}(typeof globalThis === "object" ? globalThis : this, () => {
  function clamp(value, low, high) {
    return Math.max(low, Math.min(high, value));
  }

  function rainbowColor(speedMps, ceilingMps) {
    const ceiling = Number(ceilingMps);
    const magnitude = Math.abs(Number(speedMps));
    const ratio = Number.isFinite(ceiling) && ceiling > 0 && Number.isFinite(magnitude)
      ? clamp(magnitude / ceiling, 0, 1) : 0;
    return `hsl(${240 * (1 - ratio)} 100% 50%)`;
  }

  function segmentSpeed(points, index) {
    if (!Array.isArray(points) || !points[index]) return 0;
    const first = Math.abs(Number(points[index].speed_ceiling_mps));
    const next = points[index + 1];
    if (!Number.isFinite(first)) return 0;
    if (!next) return first;
    const second = Math.abs(Number(next.speed_ceiling_mps));
    return Number.isFinite(second) ? (first + second) / 2 : first;
  }

  return { rainbowColor, segmentSpeed };
}));
