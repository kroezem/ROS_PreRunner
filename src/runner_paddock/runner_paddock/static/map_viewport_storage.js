"use strict";

(function expose(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.PaddockMapViewportStorage = api;
})(typeof globalThis !== "undefined" ? globalThis : this, () => {
  const KEY_PREFIX = "runner-paddock-map-viewport-v1:";

  function storageKey(mapIdentity) {
    return `${KEY_PREFIX}${encodeURIComponent(mapIdentity)}`;
  }

  function validViewport(value) {
    return value && typeof value === "object" &&
      Number.isFinite(value.x) && Number.isFinite(value.y) &&
      Number.isFinite(value.scale) && value.scale >= 2 && value.scale <= 5000 &&
      Number.isFinite(value.rotation);
  }

  function load(storage, mapIdentity) {
    if (!storage || typeof mapIdentity !== "string" || !mapIdentity) return null;
    try {
      const value = JSON.parse(storage.getItem(storageKey(mapIdentity)) || "null");
      if (!validViewport(value)) return null;
      return {
        x: value.x,
        y: value.y,
        scale: value.scale,
        rotation: value.rotation,
      };
    } catch (error) {
      return null;
    }
  }

  function save(storage, mapIdentity, viewport) {
    if (!storage || typeof mapIdentity !== "string" || !mapIdentity ||
        !validViewport(viewport)) return false;
    try {
      storage.setItem(storageKey(mapIdentity), JSON.stringify({
        x: viewport.x,
        y: viewport.y,
        scale: viewport.scale,
        rotation: viewport.rotation,
      }));
      return true;
    } catch (error) {
      return false;
    }
  }

  return { load, save };
});
