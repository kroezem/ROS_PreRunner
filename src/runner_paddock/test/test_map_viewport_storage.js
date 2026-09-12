"use strict";

const test = require("node:test");
const assert = require("node:assert/strict");
const viewportStorage = require("../runner_paddock/static/map_viewport_storage.js");

function memoryStorage() {
  const values = new Map();
  return {
    getItem: (key) => values.get(key) ?? null,
    setItem: (key, value) => values.set(key, value),
  };
}

test("viewport state is independent for each stable map identity", () => {
  const storage = memoryStorage();
  const mapA = { x: 1, y: 2, scale: 80, rotation: 0.3 };
  const mapB = { x: -4, y: 5, scale: 125, rotation: -0.7 };

  assert.equal(viewportStorage.save(storage, "map A", mapA), true);
  assert.equal(viewportStorage.save(storage, "map/B", mapB), true);
  assert.deepEqual(viewportStorage.load(storage, "map A"), mapA);
  assert.deepEqual(viewportStorage.load(storage, "map/B"), mapB);
});

test("missing, corrupt, and invalid viewport state falls back cleanly", () => {
  const storage = memoryStorage();
  assert.equal(viewportStorage.load(storage, "missing"), null);

  storage.setItem("runner-paddock-map-viewport-v1:corrupt", "not json");
  assert.equal(viewportStorage.load(storage, "corrupt"), null);

  storage.setItem("runner-paddock-map-viewport-v1:invalid", JSON.stringify({
    x: 1, y: 2, scale: -10, rotation: 0,
  }));
  assert.equal(viewportStorage.load(storage, "invalid"), null);
});

test("unavailable browser storage cannot break viewport rendering", () => {
  const blocked = {
    getItem() { throw new Error("blocked"); },
    setItem() { throw new Error("blocked"); },
  };
  const viewport = { x: 1, y: 2, scale: 50, rotation: 0 };

  assert.equal(viewportStorage.load(blocked, "map"), null);
  assert.equal(viewportStorage.save(blocked, "map", viewport), false);
});
