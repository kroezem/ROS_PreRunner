"use strict";

(function exposeSpeedProfile(root, factory) {
  const api = factory();
  if (typeof module === "object" && module.exports) module.exports = api;
  if (root) root.PaddockSpeedProfile = api;
}(typeof window !== "undefined" ? window : globalThis, () => {
  const PRESETS = [
    { id: "timid", label: "TIMID", maxSpeed: 0.45, color: "#f6c85f" },
    { id: "confident", label: "CONFIDENT", maxSpeed: 1.00, color: "#19c4df" },
    { id: "insane", label: "INSANE", maxSpeed: 1.50, color: "#b279ff" },
    { id: "absurd", label: "ABSURD", maxSpeed: 2.00, color: "#ff6b7a" },
  ];
  const DEFAULTS = {
    minSpeed: 0.50,
    presetScaling: 0,
    tightThreshold: 0.10,
    openThreshold: 0.70,
    curveFamily: "smoothstep",
    curveShape: 1.00,
    approachTime: 0.50,
    selectedClearance: 0.30,
  };
  // These are the current D2 longitudinal coefficients. This module has no
  // runtime authority and deliberately does not write them anywhere.
  const D2 = {
    brakingLinear: 1.6,
    brakingConstant: 0.27,
    reactionTime: 0.40,
    recoveryGain: 1.6,
    recoveryFloor: 0.60,
  };
  const STORAGE_KEY = "runner-paddock-speed-profile-design-v1";

  const clamp = (value, low, high) => Math.max(low, Math.min(high, value));

  function sanitize(candidate = {}) {
    const finite = (name, low, high) => {
      const value = Number(candidate[name]);
      return Number.isFinite(value) ? clamp(value, low, high) : DEFAULTS[name];
    };
    const tightThreshold = finite("tightThreshold", 0, 1);
    return {
      minSpeed: finite("minSpeed", 0.1, 1),
      presetScaling: finite("presetScaling", 0, 100),
      tightThreshold,
      openThreshold: Math.max(tightThreshold + 0.01, finite("openThreshold", 0.05, 2)),
      curveFamily: ["linear", "power", "smoothstep"].includes(candidate.curveFamily)
        ? candidate.curveFamily : DEFAULTS.curveFamily,
      curveShape: finite("curveShape", 0.25, 4),
      approachTime: finite("approachTime", 0, 3),
      selectedClearance: finite("selectedClearance", 0, 2),
    };
  }

  function constrainedSpeed(preset, settings) {
    const shared = Math.min(settings.minSpeed, preset.maxSpeed);
    return shared + (preset.maxSpeed - shared) * settings.presetScaling / 100;
  }

  function curveProgress(clearance, settings) {
    const span = settings.openThreshold - settings.tightThreshold;
    const t = clamp((clearance - settings.tightThreshold) / span, 0, 1);
    if (settings.curveFamily === "power") return Math.pow(t, settings.curveShape);
    if (settings.curveFamily === "smoothstep") {
      const shaped = Math.pow(t, settings.curveShape);
      return shaped * shaped * (3 - 2 * shaped);
    }
    return t;
  }

  function geometrySpeed(clearance, preset, settings) {
    const low = constrainedSpeed(preset, settings);
    return low + (preset.maxSpeed - low) * curveProgress(clearance, settings);
  }

  function stoppingPotential(speed) {
    const k = D2.brakingLinear;
    const c = D2.brakingConstant;
    if (speed <= 0) return 0;
    return speed / k - c / (k * k) * Math.log((k * speed + c) / c);
  }

  function brakingPotential(speed) {
    return stoppingPotential(speed) + D2.reactionTime * speed;
  }

  function reachableByBraking(fromSpeed, distance, ceiling) {
    const target = brakingPotential(fromSpeed) + Math.max(0, distance);
    if (brakingPotential(ceiling) <= target) return ceiling;
    let low = 0;
    let high = ceiling;
    for (let iteration = 0; iteration < 48; iteration += 1) {
      const middle = 0.5 * (low + high);
      if (brakingPotential(middle) <= target) low = middle;
      else high = middle;
    }
    return low;
  }

  function recoveryDistance(fromSpeed, speed) {
    if (speed <= fromSpeed) return 0;
    const gain = D2.recoveryGain;
    const floor = D2.recoveryFloor;
    return (speed - fromSpeed) / gain - floor / (gain * gain) *
      Math.log((gain * speed + floor) / (gain * fromSpeed + floor));
  }

  function reachableByRecovery(fromSpeed, distance, ceiling) {
    if (fromSpeed >= ceiling || recoveryDistance(fromSpeed, ceiling) <= distance) return ceiling;
    let low = fromSpeed;
    let high = ceiling;
    for (let iteration = 0; iteration < 48; iteration += 1) {
      const middle = 0.5 * (low + high);
      if (recoveryDistance(fromSpeed, middle) <= distance) low = middle;
      else high = middle;
    }
    return low;
  }

  function syntheticClearance(distance) {
    if (distance < 3) return 1.4;
    if (distance < 5) {
      const t = (distance - 3) / 2;
      return 1.4 + (0.04 - 1.4) * (t * t * (3 - 2 * t));
    }
    if (distance <= 7) return 0.04;
    if (distance < 9) {
      const t = (distance - 7) / 2;
      return 0.04 + (1.4 - 0.04) * (t * t * (3 - 2 * t));
    }
    return 1.4;
  }

  function pathProfile(preset, settings, step = 0.04) {
    const points = [];
    for (let distance = 0; distance <= 12 + step / 2; distance += step) {
      const clearance = syntheticClearance(distance);
      const raw = geometrySpeed(clearance, preset, settings);
      points.push({ distance, clearance, raw, ceiling: raw });
    }
    const target = geometrySpeed(0.04, preset, settings);
    const settledDistance = target * settings.approachTime;
    const pinchEntry = 5;
    const settledAt = Math.max(0, pinchEntry - settledDistance);
    for (const point of points) {
      if (point.distance >= settledAt && point.distance <= pinchEntry) {
        point.ceiling = Math.min(point.ceiling, target);
      }
      point.final = point.ceiling;
    }
    for (let index = 1; index < points.length; index += 1) {
      points[index].final = Math.min(
        points[index].final,
        reachableByRecovery(points[index - 1].final, step, preset.maxSpeed),
      );
    }
    for (let index = points.length - 1; index > 0; index -= 1) {
      points[index - 1].final = Math.min(
        points[index - 1].final,
        reachableByBraking(points[index].final, step, preset.maxSpeed),
      );
    }
    const slowdownPoint = points.find((point) => point.final < preset.maxSpeed - 0.005);
    return {
      points,
      target,
      settledDistance,
      settledAt,
      slowdownDistance: slowdownPoint ? pinchEntry - slowdownPoint.distance : 0,
    };
  }

  function drawChart(canvas, series, options) {
    const rect = canvas.getBoundingClientRect();
    const width = Math.max(320, Math.round(rect.width));
    const height = Math.max(220, Math.round(rect.height));
    const ratio = window.devicePixelRatio || 1;
    canvas.width = Math.round(width * ratio);
    canvas.height = Math.round(height * ratio);
    const context = canvas.getContext("2d");
    context.setTransform(ratio, 0, 0, ratio, 0, 0);
    const pad = { left: 44, right: 14, top: 18, bottom: 34 };
    const x = (value) => pad.left + value / options.xMax * (width - pad.left - pad.right);
    const y = (value) => height - pad.bottom - value / options.yMax * (height - pad.top - pad.bottom);
    context.clearRect(0, 0, width, height);
    context.font = "10px system-ui";
    context.fillStyle = "#819095";
    context.strokeStyle = "#293639";
    context.lineWidth = 1;
    for (let tick = 0; tick <= 4; tick += 1) {
      const value = options.yMax * tick / 4;
      context.beginPath(); context.moveTo(pad.left, y(value)); context.lineTo(width - pad.right, y(value)); context.stroke();
      context.fillText(value.toFixed(1), 10, y(value) + 3);
    }
    for (let tick = 0; tick <= 4; tick += 1) {
      const value = options.xMax * tick / 4;
      context.fillText(value.toFixed(1), x(value) - 8, height - 12);
    }
    context.fillText(options.xLabel, width - pad.right - 50, height - 2);
    for (const line of series) {
      context.beginPath();
      context.setLineDash(line.dashed ? [5, 4] : []);
      context.globalAlpha = line.dashed ? 0.42 : 1;
      context.strokeStyle = line.color;
      context.lineWidth = line.dashed ? 1.4 : 2.2;
      line.points.forEach((point, index) => {
        const px = x(point.x); const py = y(point.y);
        if (index === 0) context.moveTo(px, py); else context.lineTo(px, py);
      });
      context.stroke();
    }
    context.setLineDash([]);
    context.globalAlpha = 1;
    if (Number.isFinite(options.marker)) {
      context.strokeStyle = "#e8f0f1";
      context.lineWidth = 1;
      context.beginPath(); context.moveTo(x(options.marker), pad.top); context.lineTo(x(options.marker), height - pad.bottom); context.stroke();
    }
  }

  function initialize() {
    const clearanceCanvas = document.getElementById("profile-clearance-chart");
    const pathCanvas = document.getElementById("profile-path-chart");
    if (!clearanceCanvas || !pathCanvas) return;
    let stored = {};
    try { stored = JSON.parse(window.localStorage.getItem(STORAGE_KEY) || "{}"); } catch (error) { stored = {}; }
    let settings = sanitize({ ...DEFAULTS, ...stored });
    const inputs = {
      minSpeed: document.getElementById("profile-min-speed"),
      presetScaling: document.getElementById("profile-preset-scaling"),
      tightThreshold: document.getElementById("profile-tight-threshold"),
      openThreshold: document.getElementById("profile-open-threshold"),
      curveFamily: document.getElementById("profile-curve-family"),
      curveShape: document.getElementById("profile-curve-shape"),
      approachTime: document.getElementById("profile-approach-time"),
      selectedClearance: document.getElementById("profile-selected-clearance"),
    };

    function render() {
      Object.entries(inputs).forEach(([name, input]) => { input.value = String(settings[name]); });
      const units = { minSpeed: " m/s", presetScaling: "%", tightThreshold: " m", openThreshold: " m", curveShape: "", approachTime: " s", selectedClearance: " m" };
      Object.entries(units).forEach(([name, unit]) => {
        document.querySelector(`[data-profile-output="${name}"]`).textContent = `${Number(settings[name]).toFixed(name === "presetScaling" ? 0 : 2)}${unit}`;
      });
      document.getElementById("profile-shape-label").hidden = settings.curveFamily === "linear";
      document.getElementById("profile-clearance-readout").textContent = `${settings.selectedClearance.toFixed(2)} m clearance`;
      const profiles = PRESETS.map((preset) => ({ preset, profile: pathProfile(preset, settings) }));
      document.getElementById("profile-readouts").innerHTML = profiles.map(({ preset, profile }) =>
        `<div style="--profile-color:${preset.color}"><strong>${preset.label}</strong><span>${geometrySpeed(settings.selectedClearance, preset, settings).toFixed(2)} m/s selected</span><span>${profile.slowdownDistance.toFixed(2)} m slowdown</span><span>${profile.settledDistance.toFixed(2)} m settled entry</span></div>`).join("");
      const clearanceSeries = PRESETS.map((preset) => ({
        color: preset.color,
        points: Array.from({ length: 101 }, (_, index) => {
          const clearance = 2 * index / 100;
          return { x: clearance, y: geometrySpeed(clearance, preset, settings) };
        }),
      }));
      drawChart(clearanceCanvas, clearanceSeries, { xMax: 2, yMax: 2.1, xLabel: "clearance (m)", marker: settings.selectedClearance });
      const pathSeries = profiles.flatMap(({ preset, profile }) => [
        { color: preset.color, dashed: true, points: profile.points.map((point) => ({ x: point.distance, y: point.raw })) },
        { color: preset.color, points: profile.points.map((point) => ({ x: point.distance, y: point.final })) },
      ]);
      drawChart(pathCanvas, pathSeries, { xMax: 12, yMax: 2.1, xLabel: "distance (m)" });
    }

    Object.entries(inputs).forEach(([name, input]) => {
      input.addEventListener(name === "curveFamily" ? "change" : "input", () => {
        settings = sanitize({ ...settings, [name]: name === "curveFamily" ? input.value : Number(input.value) });
        try { window.localStorage.setItem(STORAGE_KEY, JSON.stringify(settings)); } catch (error) { /* Optional preference storage. */ }
        render();
      });
    });
    const observer = new ResizeObserver(() => {
      if (!clearanceCanvas.closest("[hidden]")) render();
    });
    observer.observe(clearanceCanvas.parentElement);
    render();
  }

  return {
    DEFAULTS, D2, PRESETS, constrainedSpeed, curveProgress, geometrySpeed,
    pathProfile, reachableByBraking, reachableByRecovery, sanitize, initialize,
  };
}));
