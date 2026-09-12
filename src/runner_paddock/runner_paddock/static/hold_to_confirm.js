"use strict";

(function expose(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.PaddockHoldToConfirm = api;
})(typeof globalThis !== "undefined" ? globalThis : this, () => {
  class HoldToConfirm {
    constructor(options) {
      this.durationMs = options.durationMs;
      this.onProgress = options.onProgress;
      this.onComplete = options.onComplete;
      this.now = options.now || (() => performance.now());
      this.setTimer = options.setTimer || setTimeout;
      this.clearTimer = options.clearTimer || clearTimeout;
      this.requestFrame = options.requestFrame || requestAnimationFrame;
      this.cancelFrame = options.cancelFrame || cancelAnimationFrame;
      this.active = false;
      this.timer = null;
      this.frame = null;
      this.startedAt = 0;
    }

    start() {
      if (this.active) return;
      this.active = true;
      this.startedAt = this.now();
      this.onProgress(0, true);
      this.timer = this.setTimer(() => this.complete(), this.durationMs);
      this.frame = this.requestFrame(() => this.tick());
    }

    tick() {
      if (!this.active) return;
      const progress = Math.min(1, (this.now() - this.startedAt) / this.durationMs);
      this.onProgress(progress, true);
      if (progress < 1) this.frame = this.requestFrame(() => this.tick());
    }

    cancel() {
      if (!this.active) return;
      this.active = false;
      this.clearTimer(this.timer);
      this.cancelFrame(this.frame);
      this.timer = null;
      this.frame = null;
      this.onProgress(0, false);
    }

    complete() {
      if (!this.active) return;
      this.active = false;
      this.cancelFrame(this.frame);
      this.timer = null;
      this.frame = null;
      this.onProgress(1, false);
      this.onComplete();
    }
  }

  return { HoldToConfirm };
});
