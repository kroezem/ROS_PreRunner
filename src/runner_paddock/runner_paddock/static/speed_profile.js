"use strict";
(function expose(root, factory) { const api = factory(); if (typeof module === "object" && module.exports) module.exports = api; if (root) root.PaddockSpeedProfile = api; }(typeof window !== "undefined" ? window : globalThis, () => {
  const CURVE_COLOR = "#19c4df";
  const DEFAULTS = {
    desired_linear_vel: 1.00, maximum_commanded_speed: 1.00,
    regulated_linear_scaling_min_speed: .40, regulated_linear_scaling_min_radius: .75,
    min_lookahead_dist: .30, max_lookahead_dist: .80, lookahead_time: 1.0,
    max_allowed_time_to_collision_up_to_carrot: .60,
    minimum_traversal_speed: .25, tight_clearance: .05, open_clearance: .7,
    clearance_curve_family: 2, clearance_curve_shape: 1,
    approach_time_s: 0, curvature_window: .40, max_lateral_acceleration: .35,
    footprint_front: .230, footprint_rear: .060, footprint_half_width: .0825,
    braking_linear: 1.6, braking_constant: .27, reaction_time_s: .40,
    recovery_acceleration_gain: 1.6, recovery_acceleration_floor: .60,
    cost_penalty: 2.0,
  };
  // Every field a named Speed Profile owns -- everything above except
  // GridBased.cost_penalty (Planner Settings, never part of a saved profile).
  const SPEED_PROFILE_FIELDS = Object.keys(DEFAULTS).filter(field => field !== "cost_penalty");
  const clamp = (v, low, high) => Math.max(low, Math.min(high, v));
  function curveProgress(clearance, s) { const x = clamp((clearance - s.tight_clearance) / (s.open_clearance - s.tight_clearance), 0, 1); if (+s.clearance_curve_family === 0) return x; const shaped = Math.pow(x, s.clearance_curve_shape); return +s.clearance_curve_family === 2 ? shaped * shaped * (3 - 2 * shaped) : shaped; }
  // The bottom of the curve is now minimum_traversal_speed directly -- no
  // preset-relative scaling. The top is the draft's own maximum speed.
  function geometrySpeed(clearance, s) { const top = Math.max(s.minimum_traversal_speed, s.desired_linear_vel); const bottom = clamp(s.minimum_traversal_speed, 0, top); return bottom + (top - bottom) * curveProgress(clearance, s); }
  function draw(canvas, points) { const width = Math.max(320, Math.round(canvas.getBoundingClientRect().width)); const height = 260; const ratio = window.devicePixelRatio || 1; canvas.width = width * ratio; canvas.height = height * ratio; const ctx = canvas.getContext("2d"); ctx.setTransform(ratio, 0, 0, ratio, 0, 0); const p = { left: 44, right: 14, top: 18, bottom: 34 }; const x = v => p.left + v / 2 * (width - p.left - p.right); const y = v => height - p.bottom - v / 2.1 * (height - p.top - p.bottom); ctx.clearRect(0, 0, width, height); ctx.font = "10px system-ui"; for (let i = 0; i <= 4; i += 1) { const v = i * .5; ctx.strokeStyle = "#293639"; ctx.beginPath(); ctx.moveTo(p.left, y(v)); ctx.lineTo(width - p.right, y(v)); ctx.stroke(); ctx.fillStyle = "#819095"; ctx.fillText(v.toFixed(1), 10, y(v) + 3); ctx.fillText(v.toFixed(1), x(v) - 8, height - 12); } ctx.fillText("clearance (m)", width - 82, height - 2); ctx.strokeStyle = CURVE_COLOR; ctx.lineWidth = 2.2; ctx.beginPath(); points.forEach((point, i) => i ? ctx.lineTo(x(point.x), y(point.y)) : ctx.moveTo(x(point.x), y(point.y))); ctx.stroke(); }
  const CONTROLS = [["desired_linear_vel", "Maximum speed", .1, 2, .01], ["minimum_traversal_speed", "Minimum traversal speed", 0, 2, .01], ["tight_clearance", "Tight clearance", 0, 1, .01], ["open_clearance", "Open clearance", .01, 2, .01], ["clearance_curve_shape", "Curve shape", .25, 4, .05], ["approach_time_s", "Approach time", 0, 3, .05], ["curvature_window", "Curvature window", .05, 2, .01], ["max_lateral_acceleration", "Max lateral acceleration", .05, 3, .01], ["footprint_front", "Footprint front", .01, 1, .005], ["footprint_rear", "Footprint rear", 0, 1, .005], ["footprint_half_width", "Footprint half width", .01, 1, .005], ["braking_linear", "Braking linear", .01, 5, .01], ["braking_constant", "Braking constant", .01, 2, .01], ["reaction_time_s", "Reaction time", 0, 3, .05], ["recovery_acceleration_gain", "Recovery gain", .01, 5, .01], ["recovery_acceleration_floor", "Recovery floor", .01, 3, .01], ["maximum_commanded_speed", "Absolute ceiling (adapter)", .1, 2, .01], ["regulated_linear_scaling_min_speed", "Regulated minimum (curvature floor)", .1, 2, .01], ["regulated_linear_scaling_min_radius", "Curvature radius", .01, 5, .01], ["min_lookahead_dist", "Minimum lookahead", .01, 5, .01], ["max_lookahead_dist", "Maximum lookahead", .01, 5, .01], ["lookahead_time", "Lookahead time", .01, 5, .01], ["max_allowed_time_to_collision_up_to_carrot", "Collision horizon (safety backstop)", .01, 5, .01]];
  const markup = ([field, label, min, max, step]) => `<label>${label}<span><input data-profile-range="${field}" type="range" min="${min}" max="${max}" step="${step}"><input data-tuning-field="${field}" type="number" min="${min}" max="${max}" step="${step}"></span></label>`;

  // Draft/dirty state model, kept free of the DOM so it can be unit tested.
  //
  // `values` starts at the confirmed ROS read-back and only ever changes for
  // a field either by a local edit (which also marks it dirty) or by a
  // verified-applied read-back for that exact field (which clears dirty).
  // Confirmed broadcasts that arrive while a field is dirty are dropped for
  // that field only — that is the whole fix: no timer, no blur, no focus
  // check decides whether an edit survives.
  function createDraftStore(defaults) {
    let values = { ...defaults };
    const dirty = new Set();
    return {
      values() { return { ...values }; },
      isDirty(field) { return dirty.has(field); },
      dirtyFields() { return new Set(dirty); },
      edit(field, value) {
        values = { ...values, [field]: value };
        dirty.add(field);
      },
      // Merge a confirmed ROS snapshot in, skipping any field currently
      // being edited so an in-flight or unrelated broadcast can never
      // clobber an unsaved edit.
      receiveConfirmed(confirmed) {
        const next = { ...values };
        Object.entries(confirmed).forEach(([field, value]) => {
          if (!dirty.has(field)) next[field] = value;
        });
        values = next;
      },
      // A verified-applied read-back covers every field in the atomic
      // snapshot that was sent, so it resolves every dirty field at once.
      resolveApplied(confirmed) {
        values = { ...values, ...confirmed };
        dirty.clear();
      },
      // Loading a saved profile copies every field of the snapshot into the
      // draft and marks each one dirty (it now differs from live ROS state
      // and must survive broadcasts) -- it never applies anything live.
      load(snapshot) {
        values = { ...values, ...snapshot };
        Object.keys(snapshot).forEach(field => dirty.add(field));
      },
    };
  }

  // A dirty field only resolves on the rising edge into "applied" status;
  // a steady-state "applied" on a later, unrelated broadcast must not
  // re-fire, or an edit made after a prior successful apply would be wiped
  // out by the very next broadcast.
  function shouldResolveApplied(status, lastStatus) {
    return status === "applied" && lastStatus !== "applied";
  }

  function initialize() {
    const canvas = document.getElementById("profile-clearance-chart");
    if (!canvas) return;
    document.getElementById("profile-policy-controls").innerHTML = CONTROLS.map(markup).join("");
    document.getElementById("profile-planner-controls").innerHTML = markup(["cost_penalty", "GridBased.cost_penalty", .01, 100, .01]);

    const store = createDraftStore(DEFAULTS);
    let lastStatus = null;

    const render = () => {
      const current = store.values();
      draw(canvas, Array.from({ length: 101 }, (_, i) => ({ x: i / 50, y: geometrySpeed(i / 50, current) })));
      const top = Math.max(current.minimum_traversal_speed, current.desired_linear_vel);
      document.getElementById("profile-readouts").innerHTML = `<div style="--profile-color:${CURVE_COLOR}"><strong>Current draft</strong><span>${geometrySpeed(current.tight_clearance, current).toFixed(2)} m/s tight</span><span>${top.toFixed(2)} m/s open</span></div>`;
    };

    // Push the draft into every control, including the dirty ones (their
    // draft value is already what should be shown). Skip the element the
    // operator is actively typing/dragging in purely to avoid disturbing
    // that gesture — this is a UX nicety, not the ownership mechanism.
    const syncControls = () => {
      const current = store.values();
      document.querySelectorAll("#panel-speed-profile [data-tuning-field]").forEach(input => {
        const field = input.dataset.tuningField;
        const value = current[field];
        if (!Number.isFinite(value)) return;
        if (document.activeElement !== input) input.value = String(value);
        const range = document.querySelector(`#panel-speed-profile [data-profile-range="${field}"]`);
        if (range && document.activeElement !== range) range.value = String(value);
      });
    };

    const editField = (field, rawValue) => {
      const value = Number(rawValue);
      if (!Number.isFinite(value)) return;
      store.edit(field, value);
      render();
    };

    document.querySelectorAll("#panel-speed-profile [data-profile-range]").forEach(range => range.addEventListener("input", () => {
      const field = range.dataset.profileRange;
      const number = document.querySelector(`#panel-speed-profile [data-tuning-field="${field}"]`);
      if (number) number.value = range.value;
      editField(field, range.value);
    }));
    document.querySelectorAll("#panel-speed-profile [data-tuning-field]").forEach(input => input.addEventListener("input", () => {
      const field = input.dataset.tuningField;
      const range = document.querySelector(`#panel-speed-profile [data-profile-range="${field}"]`);
      if (range) range.value = input.value;
      editField(field, input.value);
    }));

    window.addEventListener("paddock-tuning", event => {
      const detail = event.detail || {};
      const confirmed = detail.values || {};
      const status = detail.status;
      // Only a fresh applying->applied transition resolves dirty fields;
      // a steady-state "applied" on later broadcasts must not re-trigger,
      // or an edit made after a successful apply would be wiped out by
      // the very next unrelated broadcast.
      const justApplied = shouldResolveApplied(status, lastStatus);
      lastStatus = status;
      if (justApplied) store.resolveApplied(confirmed);
      else store.receiveConfirmed(confirmed);
      syncControls();
      render();
    });

    window.addEventListener("paddock-load-speed-profile", event => {
      const snapshot = (event.detail || {}).values || {};
      store.load(snapshot);
      syncControls();
      render();
    });

    window.PaddockSpeedProfileDraft = {
      values: () => store.values(),
      speedProfileValues: () => {
        const current = store.values();
        const snapshot = {};
        SPEED_PROFILE_FIELDS.forEach(field => { snapshot[field] = current[field]; });
        return snapshot;
      },
    };

    new ResizeObserver(render).observe(canvas.parentElement);
    render();
  }

  return {
    DEFAULTS, SPEED_PROFILE_FIELDS, curveProgress, geometrySpeed,
    createDraftStore, shouldResolveApplied, initialize,
  };
}));
