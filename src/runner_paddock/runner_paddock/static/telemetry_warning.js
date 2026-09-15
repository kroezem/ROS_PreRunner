"use strict";

(function expose(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.PaddockTelemetryWarning = api;
})(typeof globalThis !== "undefined" ? globalThis : this, () => {
  function isActive(source, valid, value, violatesThreshold) {
    return source?.available === true && source.fresh === true && valid === true &&
      Number.isFinite(value) && violatesThreshold(value);
  }

  return { isActive };
});
