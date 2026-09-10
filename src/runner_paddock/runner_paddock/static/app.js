"use strict";

// Compact Paddock operator console. Backend state remains authoritative; the
// browser keeps no optimistic RUN, goal, STOP-clear, map, or mission state.

const $ = (id) => document.getElementById(id);
const banner = $("banner");
const acklog = $("acklog");

let socket = null;
let socketReady = false;
let role = "observer";
const latest = {};
let heartbeatTimer = null;
let runHeld = false;
let keyboardRun = false;
let runTimer = null;
let manualTimer = null;
let manualEngaged = false;
let manualPointerId = null;
let manualDemand = { speed_mps: 0, steering: 0 };
let catalogRenderKey = null;
let pendingDeleteName = "";
let pendingDeleteRecordingName = "";
let recordingCatalogKey = null;
let retainedPlan = null;

const mapCanvas = $("map-canvas");
const mapContext = mapCanvas.getContext("2d");
const mapGeometry = window.PaddockMapGeometry;
const joystickGeometry = window.PaddockJoystickGeometry;
const mapView = { x: 0, y: 0, scale: 50, rotation: 0, fitted: false };
const mapLayers = {
  map: { grid: null, raster: null },
  global_costmap: { grid: null, raster: null },
  local_costmap: { grid: null, raster: null },
};
const LAYER_STORAGE_KEY = "runner-paddock-map-layers-v1";
const LAYER_DEFAULTS = {
  map: { visible: true, color: "#171717", opacity: 1 },
  global_costmap: { visible: true, color: "#ff3b30", opacity: 0.58 },
  local_costmap: { visible: true, color: "#b34cff", opacity: 0.62 },
  plan: { visible: true, color: "#ffc247" },
  robot: { visible: true, color: "#00b4d8" },
  goal: { visible: true, color: "#90e0ef" },
};
const layerSettings = loadLayerSettings();
let mapMode = "view";
let mapDrag = null;
const goalInteraction = {
  dragging: false,
  pointerId: null,
  preview: null,
  awaiting: false,
};

function loadLayerSettings() {
  let stored = {};
  try {
    stored = JSON.parse(window.localStorage.getItem(LAYER_STORAGE_KEY) || "{}");
  } catch (error) {
    stored = {};
  }
  return Object.fromEntries(Object.entries(LAYER_DEFAULTS).map(([kind, defaults]) => {
    const candidate = stored && stored[kind] && typeof stored[kind] === "object"
      ? stored[kind] : {};
    const color = /^#[0-9a-f]{6}$/i.test(candidate.color) ? candidate.color : defaults.color;
    const opacity = Number.isFinite(candidate.opacity)
      ? Math.max(0, Math.min(1, candidate.opacity)) : defaults.opacity;
    return [kind, { ...defaults, ...candidate, color, opacity, visible: candidate.visible !== false }];
  }));
}

function saveLayerSettings() {
  try {
    window.localStorage.setItem(LAYER_STORAGE_KEY, JSON.stringify(layerSettings));
  } catch (error) {
    // Presentation preferences remain optional when browser storage is blocked.
  }
}

function initializeLayerControls() {
  Object.keys(LAYER_DEFAULTS).forEach((kind) => {
    const visible = $(`layer-visible-${kind}`);
    const color = $(`layer-color-${kind}`);
    const opacity = $(`layer-opacity-${kind}`);
    visible.checked = layerSettings[kind].visible;
    color.value = layerSettings[kind].color;
    if (opacity) opacity.value = String(layerSettings[kind].opacity);
    visible.addEventListener("change", () => {
      layerSettings[kind].visible = visible.checked;
      saveLayerSettings();
      renderMap();
    });
    color.addEventListener("input", () => {
      layerSettings[kind].color = color.value;
      if (mapLayers[kind] && mapLayers[kind].grid) {
        mapLayers[kind].raster = makeGridRaster(mapLayers[kind].grid, kind);
      }
      saveLayerSettings();
      renderMap();
    });
    if (opacity) opacity.addEventListener("input", () => {
      layerSettings[kind].opacity = Number(opacity.value);
      saveLayerSettings();
      renderMap();
    });
  });
}

function setBanner(value, kind) {
  banner.textContent = value;
  banner.className = `banner ${kind}`;
}

function send(action) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    ack(`not connected: ${action.action}`);
    return;
  }
  socket.send(JSON.stringify(action));
}

function ack(value) {
  acklog.textContent = `${new Date().toLocaleTimeString()}  ${value}`;
}

// --- connection ---------------------------------------------------------

function connect() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${window.location.host}/ws`);

  socket.addEventListener("open", () => {
    socketReady = true;
    setBanner("Connected — acquiring control lease…", "connected");
    send({ action: "acquire" });
    heartbeatTimer = window.setInterval(() => {
      if (role === "controller" && !runHeld) send({ action: "heartbeat" });
    }, 400);
  });

  socket.addEventListener("message", (event) => {
    let frame;
    try {
      frame = JSON.parse(event.data);
    } catch (error) {
      return;
    }
    if (frame.type === "state") {
      Object.assign(latest, frame);
      render();
    } else if (["map", "global_costmap", "local_costmap", "plan"].includes(frame.type)) {
      latest[frame.type] = frame;
      if (["map", "global_costmap", "local_costmap"].includes(frame.type)) {
        updateMapLayer(frame.type, frame);
      } else {
        rememberPlan(frame);
        renderMap();
      }
    } else if (frame.type === "ack") {
      if (typeof frame.role === "string") role = frame.role;
      if (frame.name === "select_goal" && !frame.accepted) {
        goalInteraction.awaiting = false;
      }
      ack(`${frame.name || "action"}: ${frame.accepted ? "ok" : "REJECTED"} — ${frame.reason}`);
      render();
    }
  });

  socket.addEventListener("close", () => {
    socketReady = false;
    role = "observer";
    runHeld = false;
    keyboardRun = false;
    retainedPlan = null;
    window.clearInterval(heartbeatTimer);
    window.clearInterval(runTimer);
    resetManualLocal();
    $("btn-run").classList.remove("armed");
    setBanner("Disconnected — retrying (RUN is revoked)", "waiting");
    window.setTimeout(connect, 1000);
  });
}

// --- CONTROL / CONFIGURE navigation ------------------------------------

function selectView(name) {
  if (name !== "control") stopRun();
  document.body.dataset.view = name;
  document.querySelectorAll(".primary-tab").forEach((tab) => {
    const selected = tab.dataset.view === name;
    tab.classList.toggle("active", selected);
    tab.setAttribute("aria-pressed", String(selected));
  });
  document.querySelectorAll(".primary-view").forEach((view) => {
    const selected = view.id === `view-${name}`;
    view.hidden = !selected;
    view.classList.toggle("active", selected);
  });
  if (name === "control") window.requestAnimationFrame(resizeMapCanvas);
}

function selectConfigTab(name) {
  document.querySelectorAll(".config-tab").forEach((tab) => {
    const selected = tab.dataset.tab === name;
    tab.classList.toggle("active", selected);
    tab.setAttribute("aria-selected", String(selected));
    const panel = $(`panel-${tab.dataset.tab}`);
    panel.hidden = !selected;
    panel.classList.toggle("active", selected);
  });
}

document.querySelectorAll(".primary-tab").forEach((tab) => {
  tab.addEventListener("click", () => selectView(tab.dataset.view));
});
document.querySelectorAll(".config-tab").forEach((tab) => {
  tab.addEventListener("click", () => selectConfigTab(tab.dataset.tab));
});
document.querySelectorAll("[data-open-config]").forEach((button) => {
  button.addEventListener("click", () => {
    selectConfigTab(button.dataset.openConfig);
    selectView("configure");
  });
});

// --- render -------------------------------------------------------------

const RUNTIME = ["IDLE", "MAPPING", "AUTONOMY"];
const RUNTIME_STATUS = ["STABLE", "TRANSITIONING", "FAULT"];
const AUTHORITY = ["NONE", "DUALSENSE", "PADDOCK_MANUAL", "PADDOCK_AUTONOMY"];
const MISSION = ["IDLE", "DISPATCHING", "ACTIVE", "CANCELING", "SUCCEEDED", "FAILED", "CANCELED"];
const RECORDING_STATE = ["IDLE", "STARTING", "RECORDING", "STOPPING", "FAILED"];

function text(id, value) { $(id).textContent = value; }

function runtimeDetail(mode) {
  const runtime = RUNTIME[mode.mode] ?? "UNKNOWN";
  if (mode.status === 1) return humanDetail(mode.detail || `Starting ${runtime}`);
  if (mode.status === 2) return `Runtime fault — ${humanDetail(mode.detail || mode.readiness_reason)}`;
  if (mode.mode !== 0 && !mode.ready) {
    return `${runtime} not ready — ${humanDetail(mode.readiness_reason || mode.detail)}`;
  }
  return mode.ready ? `${runtime} ready` : `${runtime} stable`;
}

function render() {
  const mode = latest.mode || {};
  const auth = latest.command_authority || {};
  const lease = latest.control_lease || {};
  const stop = latest.stop_state || {};
  const local = latest.local_control || {};
  const adapter = latest.adapter_state || {};
  const gw = latest.gateway || {};
  const mapState = latest.map_state || {};
  const nav = latest.navigation_state || {};
  const recording = latest.recording_state || {};
  const config = latest.config || {};
  const health = (latest.health || {}).status || "?";
  const pose = latest.pose;

  setBannerFromState(mode, auth, stop);
  text("g-backend", `${socketReady ? "connected" : "disconnected"} · ${health}`);
  text("system-health", health);
  text("g-role", role + (gw.lease_held ? "" : " (lease free)"));
  text("g-lease", lease.active
    ? `held · gen ${lease.generation} · ${auth.lease_fresh ? "fresh" : "STALE"} (${fmt(auth.lease_age_sec)}s)`
    : "not held");
  text("g-runtime", `${RUNTIME[mode.mode] ?? "?"} / ${RUNTIME_STATUS[mode.status] ?? "?"}`);
  text("g-ready", mode.ready ? "ready" : `not ready — ${mode.readiness_reason || mode.detail || "?"}`);
  text("g-authority", `${AUTHORITY[auth.authority] ?? "?"} · ${auth.reason || ""}`);
  text("g-reason", auth.brake_intent ? (auth.reason || "brake") : "— (autonomy path open)");
  text("g-stop", stopSummary(stop, auth));
  text("g-dualsense", local.active ? `ACTIVE (${local.mode || ""})` :
    (local.connected ? "connected, idle" : "disconnected"));
  renderControlState(mode, auth, stop, local, adapter, nav);
  renderRecording(recording);
  const appliedManualMax = Number.isFinite(config.applied_value)
    ? config.applied_value : 0.40;
  text("manual-max-speed", appliedManualMax.toFixed(2));
  text("manual-max-requested", `${fmt(config.requested_value ?? 0.40)} m/s`);
  text("manual-max-applied", `${appliedManualMax.toFixed(2)} m/s`);
  text("manual-max-result", config.reason || "Waiting for configuration state…");
  if (document.activeElement !== $("manual-max-speed-input")) {
    $("manual-max-speed-input").value = appliedManualMax.toFixed(2);
  }

  const detail = runtimeDetail(mode);
  text("a-runtime-detail", detail);
  text("m-runtime-detail", detail);
  text("a-runtime-chip", `${RUNTIME[mode.mode] ?? "?"} · ${mode.ready ? "READY" : RUNTIME_STATUS[mode.status] ?? "?"}`);
  text("m-runtime-chip", `${RUNTIME[mode.mode] ?? "?"} · ${mode.ready ? "READY" : RUNTIME_STATUS[mode.status] ?? "?"}`);
  text("m-session", mode.mapping_session_id || "—");
  text("m-phase", ["NONE", "STARTING", "READY", "SAVING", "SAVED", "FAILED"][mapState.session_phase] ?? "—");
  text("m-unsaved", mapState.unsaved ? "unsaved content" : "—");
  text("m-save", `${["idle", "running", "succeeded", "failed"][mapState.save_state] ?? "—"} · ${mapState.save_detail || ""}`);
  text("m-selected", mapState.selected_map_applied || "(none)");
  text("m-selection-result", mapState.selected_map_reason
    ? `Selection rejected: ${mapState.selected_map_requested || "(unnamed)"} — ${mapState.selected_map_reason}`
    : (mapState.selected_map_applied ? `Selected: ${mapState.selected_map_applied}` : "No map selected"));
  text("m-delete-result", mapState.delete_state
    ? `${mapState.delete_state === 1 ? "Deleted" : "Delete rejected"}: ${mapState.delete_name || "(unnamed)"} — ${mapState.delete_detail || ""}`
    : "No delete operation this boot");
  text("map-count", String((mapState.catalog || []).length));
  renderCatalog(mapState.catalog || [], mapState.selected_map_applied);

  text("a-map", mode.active_autonomy_map || "(none)");
  text("a-mission", `${MISSION[nav.state] ?? "—"}${nav.mission_valid ? " · valid" : ""} · rev ${nav.mission_revision ?? 0}`);
  text("a-goal", auth.autonomy_goal_selected
    ? `selected · x ${fmt(auth.goal_x)} · y ${fmt(auth.goal_y)} · final yaw ${fmt(auth.goal_yaw)} rad`
    : "none selected");
  text("a-detail", nav.detail || nav.error_meaning || "—");
  text("a-active", auth.autonomy_action_active ? "Nav2 executing" : "not executing");
  const heading = pose ? mapGeometry.yawOf(pose.orientation) : null;
  $("pose-hint").textContent = pose
    ? `Robot: x ${fmt(pose.position.x)} · y ${fmt(pose.position.y)} · heading ${fmt(heading)} rad`
    : "Robot pose: — (no map→base_link TF)";
  renderMetric("speed", adapter.commanded_speed, adapter.measured_speed, 1.0);
  renderMetric("yaw", adapter.commanded_yaw_rate, adapter.measured_yaw_rate, 2.0);

  const controller = role === "controller";
  document.querySelectorAll(
    "button.mode, #btn-clear-stop, #btn-clear-obstacles, #btn-new-map, #btn-new-map-from-maps, " +
    "#btn-save-map, #btn-select-goal, #btn-run, #btn-cancel, " +
    "#btn-goal-mode, #btn-confirm-delete, #btn-confirm-delete-recording, " +
    "#btn-apply-manual-speed",
  ).forEach((button) => {
    button.disabled = !controller;
  });
  $("btn-stop").disabled = !socketReady || !controller;
  $("btn-run").classList.toggle("armed", runHeld);
  const hasAutonomyMap = Boolean(mapState.selected_map_applied);
  document.querySelectorAll('button[data-mode="autonomy"]').forEach((button) => {
    button.disabled = !controller || !hasAutonomyMap;
  });
  $("a-map-hint").textContent = hasAutonomyMap ? "" : "Select a completed map in MAPS before AUTONOMY.";
  const autonomyControl = mode.mode === 2 && mode.status === 0;
  const stopReady = auth.stop_state_fresh && auth.stop_healthy && auth.stop_clear &&
    !auth.stop_applied && !stop.stopped;
  const runAvailable = autonomyControl && mode.ready && stopReady &&
    !auth.dualsense_active && Boolean(auth.autonomy_goal_selected) && controller;
  $("run-controls").hidden = !autonomyControl;
  $("run-hint").hidden = !autonomyControl;
  $("btn-run").disabled = !runAvailable;
  $("btn-cancel").disabled = !controller || !autonomyControl;
  $("btn-goal-mode").hidden = !autonomyControl;
  $("btn-goal-mode").disabled = !controller || !autonomyControl;
  $("btn-clear-stop").hidden = !(stop.stopped || stop.clear_pending || auth.stop_applied);
  if (!runAvailable) stopRun();

  const preview = goalInteraction.preview;
  if (preview && goalInteraction.awaiting && auth.autonomy_goal_selected &&
      Math.abs(auth.goal_x - preview.x) < 1e-6 &&
      Math.abs(auth.goal_y - preview.y) < 1e-6 &&
      Math.abs(auth.goal_yaw - preview.yaw) < 1e-6) {
    goalInteraction.awaiting = false;
    goalInteraction.preview = null;
    setMapMode("view");
  }
  $("health-debug").textContent = JSON.stringify(latest.health || {}, null, 2);
  renderGoalControls();
  renderMap();
}

function formatDuration(seconds) {
  const total = Math.max(0, Math.floor(Number(seconds) || 0));
  const hours = Math.floor(total / 3600);
  const minutes = Math.floor((total % 3600) / 60);
  const remainder = total % 60;
  return hours > 0
    ? `${hours}:${String(minutes).padStart(2, "0")}:${String(remainder).padStart(2, "0")}`
    : `${String(minutes).padStart(2, "0")}:${String(remainder).padStart(2, "0")}`;
}

function formatBytes(value) {
  const bytes = Number(value);
  if (!Number.isFinite(bytes)) return "—";
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 ** 2) return `${(bytes / 1024).toFixed(1)} KiB`;
  if (bytes < 1024 ** 3) return `${(bytes / 1024 ** 2).toFixed(1)} MiB`;
  return `${(bytes / 1024 ** 3).toFixed(2)} GiB`;
}

function renderRecording(recording) {
  const state = Number.isInteger(recording.state) ? recording.state : null;
  const active = [1, 2, 3].includes(state);
  const stopping = state === 3;
  const stateName = state === null ? "UNAVAILABLE" : RECORDING_STATE[state] || "UNKNOWN";
  text("record-action", active ? "STOP REC" : "REC");
  text("record-elapsed", formatDuration(recording.elapsed_sec));
  text("recording-chip", stateName);
  text("recording-detail", recording.detail || "Waiting for recording executor…");
  text("recording-active-name", recording.name || "—");
  text("recording-active-size", active
    ? `${formatDuration(recording.elapsed_sec)} · ${formatBytes(recording.size_bytes)}` : "—");
  text("recording-path", recording.output_path || "—");
  text("recording-pid", recording.recorder_pid
    ? `${recording.recorder_pid} · ${recording.process_healthy ? "healthy" : "NOT HEALTHY"}` : "—");
  $("record-action").parentElement.classList.toggle("active", active);
  $("btn-record").disabled = !socketReady || role !== "controller" || stopping || state === null;
  $("btn-recording-form").textContent = active ? "STOP RECORDING" : "START RECORDING";
  $("btn-recording-form").disabled = role !== "controller" || !socketReady || stopping || state === null;
  $("recording-name").disabled = active;
  $("recording-profile").disabled = active;
  renderRecordingCatalog(recording.recordings || [], active ? recording.name : "");
}

function renderRecordingCatalog(recordings, activeName) {
  const key = JSON.stringify([recordings, activeName, role]);
  if (key === recordingCatalogKey) return;
  recordingCatalogKey = key;
  const list = $("recording-catalog");
  list.replaceChildren();
  if (!recordings.length) {
    const empty = document.createElement("li");
    empty.textContent = "No finalized MCAP recordings.";
    list.append(empty);
    return;
  }
  recordings.forEach((entry) => {
    const item = document.createElement("li");
    const info = document.createElement("div");
    info.className = "catalog-info";
    const title = document.createElement("strong");
    title.textContent = entry.name;
    const detail = document.createElement("span");
    const started = entry.start_time && Number.isFinite(entry.start_time.sec)
      ? new Date(entry.start_time.sec * 1000).toLocaleString() : "unknown time";
    detail.textContent = `${started} · ${formatDuration(entry.duration_sec)} · ${formatBytes(entry.size_bytes)} · ${(entry.profile || "unknown").toUpperCase()}`;
    info.append(title, detail);
    const button = document.createElement("button");
    button.className = "danger";
    button.textContent = "Delete";
    button.disabled = role !== "controller" || entry.name === activeName;
    button.addEventListener("click", () => confirmRecordingDelete(entry.name));
    item.append(info, button);
    list.append(item);
  });
}

function renderControlState(mode, auth, stop, local, adapter, nav) {
  const runtime = RUNTIME[mode.mode] ?? "UNKNOWN";
  const status = RUNTIME_STATUS[mode.status] ?? "WAITING";
  const stable = mode.status === 0;
  const autonomy = stable && mode.mode === 2;
  const mapping = stable && mode.mode === 1;
  const passive = !autonomy && !mapping;

  text("control-kicker", status === "STABLE" ? "Runtime" : status);
  text("control-title", status === "FAULT" ? `${runtime} fault` : runtime);
  text("control-chip", mode.ready ? "READY" : status);
  text("control-detail", runtimeDetail(mode));
  $("control-autonomy").hidden = !autonomy;
  $("control-mapping").hidden = !mapping;
  $("control-passive").hidden = !passive;

  let inhibit = "Ready for operator input.";
  if (!socketReady) inhibit = "Backend disconnected — RUN is revoked.";
  else if (role !== "controller") inhibit = "Observer only — another session holds control.";
  else if (stop.stopped) inhibit = `STOP asserted — ${stop.reason || "clear STOP when safe"}.`;
  else if (!auth.stop_state_fresh) inhibit = "STOP status unavailable — motion remains inhibited.";
  else if (!auth.stop_healthy) inhibit = `STOP enforcer unavailable — ${auth.stop_reason || "motion remains inhibited"}.`;
  else if (!stable) inhibit = status === "FAULT"
    ? humanDetail(mode.detail || mode.readiness_reason || "Open CONFIGURE to inspect the fault")
    : humanDetail(mode.detail || "Runtime transition in progress");
  else if (!mode.ready && mode.mode !== 0) inhibit = humanDetail(mode.readiness_reason || mode.detail);
  else if (auth.dualsense_active) inhibit = "DualSense takeover active — Paddock motion is inhibited.";
  else if (autonomy && !auth.autonomy_goal_selected) inhibit = "Set a goal on the map before holding RUN.";
  else if (autonomy && auth.brake_intent && auth.reason) inhibit = humanDetail(auth.reason);
  else if (mapping) inhibit = "Touch and hold the joystick to drive; release brakes.";
  else if (mode.mode === 0) inhibit = "Select MAPPING or AUTONOMY in CONFIGURE.";
  text("control-inhibit", inhibit);
  $("control-inhibit").classList.toggle("ready", inhibit === "Ready for operator input.");

  text("control-nav", MISSION[nav.state] ?? "—");
  text("control-goal", auth.autonomy_goal_selected ? "selected" : "not selected");
  text("control-speed", fmt(adapter.measured_speed));
  text("control-yaw", fmt(adapter.measured_yaw_rate));
  text("control-local", local.active ? `ACTIVE · ${local.mode || "local"}` :
    (local.connected ? "Connected · controls released" : "DualSense disconnected"));
  text("manual-speed", fmt(auth.manual_applied_speed_mps));
  text("manual-steering", fmt(auth.manual_applied_steering));
  const manualAvailable = mapping && mode.ready && socketReady &&
    role === "controller" && auth.stop_state_fresh && auth.stop_healthy &&
    auth.stop_clear && !stop.stopped && !auth.dualsense_active;
  $("manual-joystick").classList.toggle("disabled", !manualAvailable);
  $("manual-joystick").setAttribute("aria-disabled", String(!manualAvailable));
  if (!manualAvailable) releaseManual();

  let next = "Select a runtime in CONFIGURE.";
  if (!socketReady) next = "Waiting for backend connection.";
  else if (mode.status === 1) next = "Wait for the runtime transition to complete.";
  else if (mode.status === 2) next = "Open CONFIGURE → SYSTEM for backend truth.";
  text("control-next", next);
}

function renderMetric(name, target, current, limit) {
  const source = ((latest.health || {}).sources || {}).adapter_state;
  const available = (!source || source.fresh) && Number.isFinite(target) && Number.isFinite(current);
  text(`a-${name}-target`, available ? target.toFixed(2) : "—");
  text(`a-${name}-current`, available ? current.toFixed(2) : "—");
  setMetricMark(`a-${name}-target-mark`, target, limit, available);
  setMetricMark(`a-${name}-current-mark`, current, limit, available);
}

function setMetricMark(id, value, limit, available) {
  const mark = $(id);
  mark.classList.toggle("unavailable", !available);
  if (available) {
    const normalized = Math.max(-1, Math.min(1, value / limit));
    mark.style.left = `${50 + normalized * 50}%`;
  }
}

// --- map ----------------------------------------------------------------

function updateMapLayer(kind, grid) {
  mapLayers[kind] = { grid, raster: makeGridRaster(grid, kind) };
  if (kind === "map" && !mapView.fitted) fitMap();
  renderMap();
}

function hexChannels(color) {
  const match = /^#([0-9a-f]{6})$/i.exec(color);
  if (!match) return [255, 255, 255];
  const value = Number.parseInt(match[1], 16);
  return [(value >> 16) & 255, (value >> 8) & 255, value & 255];
}

function makeGridRaster(grid, kind) {
  if (!grid || grid.width <= 0 || grid.height <= 0 || grid.data.length !== grid.width * grid.height) return null;
  const raster = document.createElement("canvas");
  raster.width = grid.width;
  raster.height = grid.height;
  const context = raster.getContext("2d");
  const image = context.createImageData(grid.width, grid.height);
  const color = hexChannels(layerSettings[kind].color);
  for (let gy = 0; gy < grid.height; gy += 1) {
    const canvasY = grid.height - 1 - gy;
    for (let gx = 0; gx < grid.width; gx += 1) {
      const value = grid.data[gy * grid.width + gx];
      const offset = (canvasY * grid.width + gx) * 4;
      if (kind !== "map") {
        if (value <= 0) image.data.set([0, 0, 0, 0], offset);
        else image.data.set([...color, Math.round(45 + 210 * value / 100)], offset);
      } else if (value < 0) {
        image.data.set([112, 119, 114, 255], offset);
      } else {
        const occupied = value / 100;
        image.data.set([
          Math.round(235 * (1 - occupied) + color[0] * occupied),
          Math.round(238 * (1 - occupied) + color[1] * occupied),
          Math.round(235 * (1 - occupied) + color[2] * occupied),
          255,
        ], offset);
      }
    }
  }
  context.putImageData(image, 0, 0);
  return raster;
}

function screenFromWorld(x, y) {
  return mapGeometry.worldToScreen(mapView, mapCanvas.width, mapCanvas.height, x, y);
}

function worldFromScreen(x, y) {
  return mapGeometry.screenToWorld(mapView, mapCanvas.width, mapCanvas.height, x, y);
}

function drawGridLayer(kind) {
  const layer = mapLayers[kind];
  const { grid, raster } = layer;
  if (!grid || !raster || !layerSettings[kind].visible) return;
  const angle = mapGeometry.yawOf(grid.origin.orientation) + mapView.rotation;
  const topLeft = mapGeometry.gridToWorld(grid, 0, grid.height);
  const screen = screenFromWorld(topLeft.x, topLeft.y);
  const cellPixels = mapView.scale * grid.resolution;
  mapContext.save();
  mapContext.globalAlpha = layerSettings[kind].opacity;
  mapContext.imageSmoothingEnabled = false;
  mapContext.setTransform(
    cellPixels * Math.cos(angle), -cellPixels * Math.sin(angle),
    cellPixels * Math.sin(angle), cellPixels * Math.cos(angle),
    screen.x, screen.y,
  );
  mapContext.drawImage(raster, 0, 0);
  mapContext.restore();
}

function drawDirectionalPose(pose, color, radiusPixels) {
  if (!pose || !pose.position || !pose.orientation) return;
  const center = screenFromWorld(pose.position.x, pose.position.y);
  const yaw = mapGeometry.yawOf(pose.orientation);
  const lengthPixels = Math.max(radiusPixels * 2.8, 28 * devicePixelRatio);
  const lengthWorld = lengthPixels / mapView.scale;
  const tip = screenFromWorld(
    pose.position.x + Math.cos(yaw) * lengthWorld,
    pose.position.y + Math.sin(yaw) * lengthWorld,
  );
  const screenYaw = Math.atan2(-(tip.y - center.y), tip.x - center.x);
  mapContext.save();
  mapContext.strokeStyle = "#07100a";
  mapContext.fillStyle = color;
  mapContext.lineWidth = 5 * devicePixelRatio;
  mapContext.beginPath();
  mapContext.arc(center.x, center.y, radiusPixels, 0, Math.PI * 2);
  mapContext.fill();
  mapContext.stroke();
  mapContext.strokeStyle = color;
  mapContext.lineWidth = 4 * devicePixelRatio;
  mapContext.beginPath();
  mapContext.moveTo(center.x, center.y);
  mapContext.lineTo(tip.x, tip.y);
  mapContext.stroke();
  mapContext.fillStyle = color;
  mapContext.beginPath();
  mapContext.moveTo(tip.x, tip.y);
  mapContext.lineTo(tip.x - Math.cos(screenYaw - 0.55) * 10 * devicePixelRatio,
    tip.y + Math.sin(screenYaw - 0.55) * 10 * devicePixelRatio);
  mapContext.lineTo(tip.x - Math.cos(screenYaw + 0.55) * 10 * devicePixelRatio,
    tip.y + Math.sin(screenYaw + 0.55) * 10 * devicePixelRatio);
  mapContext.closePath();
  mapContext.fill();
  mapContext.restore();
}

function poseFromXYYaw(x, y, yaw) {
  return {
    position: { x, y, z: 0 },
    orientation: { x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2) },
  };
}

function planIsInMapFrame(plan) {
  return plan && plan.frame_id === "map" && Array.isArray(plan.poses) &&
    plan.poses.every((item) => !item.frame_id || item.frame_id === "map");
}

function rememberPlan(plan) {
  if (!planIsInMapFrame(plan) || plan.poses.length < 2) return;
  const nav = latest.navigation_state || {};
  const mode = latest.mode || {};
  retainedPlan = {
    plan,
    bootId: nav.boot_id || "",
    actionGeneration: nav.action_generation,
    missionRevision: nav.mission_revision,
    runtimeEpoch: mode.runtime_epoch,
  };
}

function planDisplayState() {
  const nav = latest.navigation_state || {};
  const mode = latest.mode || {};
  const source = ((latest.health || {}).sources || {}).plan;
  const inFlight = [1, 2, 3].includes(nav.state);
  const applicable = mode.mode === 2 && mode.status === 0 && nav.mission_valid && inFlight;
  if (!applicable) return { plan: null, kind: "none", label: "plan unavailable" };

  const contextMatches = retainedPlan &&
    retainedPlan.bootId === (nav.boot_id || "") &&
    retainedPlan.actionGeneration === nav.action_generation &&
    retainedPlan.missionRevision === nav.mission_revision &&
    retainedPlan.runtimeEpoch === mode.runtime_epoch;
  if (!contextMatches) return { plan: null, kind: "none", label: "awaiting plan" };

  const livePlan = latest.plan;
  const liveAvailable = planIsInMapFrame(livePlan) && livePlan.poses.length >= 2 &&
    (!source || source.fresh) && livePlan.revision === retainedPlan.plan.revision;
  if (liveAvailable) {
    return { plan: retainedPlan.plan, kind: "current", label: `plan ${retainedPlan.plan.poses.length} points` };
  }
  return {
    plan: retainedPlan.plan,
    kind: "last-known",
    label: `last known plan ${retainedPlan.plan.poses.length} points · live plan unavailable`,
  };
}

function drawPlan() {
  const display = planDisplayState();
  const plan = display.plan;
  if (!layerSettings.plan.visible || !plan) return;
  mapContext.save();
  mapContext.strokeStyle = layerSettings.plan.color;
  mapContext.lineWidth = (display.kind === "current" ? 3 : 2.5) * devicePixelRatio;
  mapContext.globalAlpha = display.kind === "current" ? 1 : 0.48;
  if (display.kind !== "current") {
    mapContext.setLineDash([8 * devicePixelRatio, 7 * devicePixelRatio]);
  }
  mapContext.lineJoin = "round";
  mapContext.beginPath();
  plan.poses.forEach((item, index) => {
    const point = screenFromWorld(item.pose.position.x, item.pose.position.y);
    if (index === 0) mapContext.moveTo(point.x, point.y);
    else mapContext.lineTo(point.x, point.y);
  });
  mapContext.stroke();
  mapContext.restore();
}

function renderMap() {
  mapCanvas.dataset.view = JSON.stringify({
    x: mapView.x, y: mapView.y, scale: mapView.scale,
    rotation: mapView.rotation, mode: mapMode,
  });
  mapContext.setTransform(1, 0, 0, 1, 0, 0);
  mapContext.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  if (layerSettings.map.visible) drawGridLayer("map");
  const sources = ((latest.health || {}).sources || {});
  const globalFresh = !sources.global_costmap || sources.global_costmap.fresh;
  const localFresh = !sources.local_costmap || sources.local_costmap.fresh;
  const poseFresh = !sources.pose || sources.pose.fresh;
  if (globalFresh) drawGridLayer("global_costmap");
  if (localFresh) drawGridLayer("local_costmap");
  drawPlan();
  const auth = latest.command_authority || {};
  if (layerSettings.goal.visible && auth.autonomy_goal_selected) {
    drawDirectionalPose(poseFromXYYaw(auth.goal_x, auth.goal_y, auth.goal_yaw), layerSettings.goal.color, 6 * devicePixelRatio);
  }
  if (layerSettings.robot.visible && poseFresh) {
    drawDirectionalPose(latest.pose, layerSettings.robot.color, 7 * devicePixelRatio);
  }
  if (layerSettings.goal.visible && goalInteraction.preview) {
    drawDirectionalPose(poseFromXYYaw(
      goalInteraction.preview.x, goalInteraction.preview.y, goalInteraction.preview.yaw,
    ), layerSettings.goal.color, 6 * devicePixelRatio);
  }
  const global = mapLayers.map.grid;
  const globalCostmap = mapLayers.global_costmap.grid;
  const local = mapLayers.local_costmap.grid;
  const planDisplay = planDisplayState();
  const rawPlan = latest.plan;
  let planStatus = planDisplay.label;
  if (rawPlan && !planIsInMapFrame(rawPlan)) {
    planStatus = `plan frame rejected (${rawPlan.frame_id || "empty"})`;
  }
  $("plan-badge").hidden = planDisplay.kind !== "last-known";
  $("map-status").textContent = global
    ? `${global.width}×${global.height} · ${global.resolution.toFixed(3)} m/cell · ${global.frame_id || "?"} · ` +
      `global costmap ${globalCostmap && globalFresh ? "available" : globalCostmap ? "stale" : "unavailable"} · ` +
      (local && localFresh ? `local costmap from ${local.source_frame_id || local.frame_id}` : `local costmap ${local ? "stale" : "unavailable"}`) +
      ` · ${planStatus}`
    : "Waiting for /map…";
  updateLayerStatuses(sources, auth);
}

function layerSourceStatus(source, available) {
  if (!available || !source || !source.available) return "unavailable";
  return source.fresh ? "available" : "stale";
}

function updateLayerStatuses(sources, auth) {
  const planDisplay = planDisplayState();
  const statuses = {
    map: layerSourceStatus(sources.map, Boolean(mapLayers.map.grid)),
    global_costmap: layerSourceStatus(
      sources.global_costmap, Boolean(mapLayers.global_costmap.grid),
    ),
    local_costmap: layerSourceStatus(
      sources.local_costmap, Boolean(mapLayers.local_costmap.grid),
    ),
    plan: planDisplay.kind === "current" ? "available" :
      (planDisplay.kind === "last-known" ? "last known" : "unavailable"),
    robot: layerSourceStatus(sources.pose, Boolean(latest.pose)),
    goal: layerSourceStatus(
      sources.command_authority, Boolean(auth.autonomy_goal_selected),
    ),
  };
  Object.entries(statuses).forEach(([kind, status]) => {
    const element = $(`layer-status-${kind}`);
    element.textContent = status;
    element.classList.toggle("bad", status !== "available" && status !== "last known");
  });
}

function renderGoalControls() {
  const preview = goalInteraction.preview;
  $("goal-preview-controls").hidden = mapMode !== "goal";
  $("btn-confirm-goal").disabled = role !== "controller" || !preview || goalInteraction.awaiting;
  $("goal-preview-text").textContent = !preview
    ? "Press and drag position → heading"
    : goalInteraction.awaiting
      ? "Waiting for authority confirmation…"
      : `x ${preview.x.toFixed(2)} · y ${preview.y.toFixed(2)} · yaw ${preview.yaw.toFixed(2)}`;
}

function resizeMapCanvas() {
  const bounds = mapCanvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  const width = Math.max(1, Math.round(bounds.width * ratio));
  const height = Math.max(1, Math.round(bounds.height * ratio));
  if (mapCanvas.width !== width || mapCanvas.height !== height) {
    mapCanvas.width = width;
    mapCanvas.height = height;
    if (mapLayers.map.grid && !mapView.fitted) fitMap();
    renderMap();
  }
}

function fitMap() {
  const grid = mapLayers.map.grid;
  if (!grid || !mapCanvas.width || !mapCanvas.height) return;
  mapView.rotation = 0;
  const bounds = mapGeometry.rotatedGridBounds(grid, mapView.rotation);
  const center = mapGeometry.gridToWorld(grid, grid.width / 2, grid.height / 2);
  mapView.x = center.x;
  mapView.y = center.y;
  mapView.scale = Math.max(2, 0.9 * Math.min(
    mapCanvas.width / Math.max(bounds.maxX - bounds.minX, grid.resolution),
    mapCanvas.height / Math.max(bounds.maxY - bounds.minY, grid.resolution),
  ));
  mapView.fitted = true;
  renderMap();
}

function zoomMap(factor, screenX = mapCanvas.width / 2, screenY = mapCanvas.height / 2) {
  const before = worldFromScreen(screenX, screenY);
  mapView.scale = Math.min(5000, Math.max(2, mapView.scale * factor));
  const after = worldFromScreen(screenX, screenY);
  mapView.x += before.x - after.x;
  mapView.y += before.y - after.y;
  mapView.fitted = true;
  renderMap();
}

function setMapMode(mode) {
  mapMode = mode;
  mapDrag = null;
  goalInteraction.dragging = false;
  goalInteraction.pointerId = null;
  if (mode !== "goal") {
    goalInteraction.preview = null;
    goalInteraction.awaiting = false;
  }
  mapCanvas.className = `mode-${mode}`;
  document.querySelectorAll("[data-map-mode]").forEach((button) => {
    button.classList.toggle("active", button.dataset.mapMode === mode);
  });
  text("map-mode-label", mode.toUpperCase());
  renderGoalControls();
  renderMap();
}

function pointerCanvas(event) {
  const rect = mapCanvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  return { x: (event.clientX - rect.left) * ratio, y: (event.clientY - rect.top) * ratio };
}

function pointerWorld(event) {
  const point = pointerCanvas(event);
  return worldFromScreen(point.x, point.y);
}

function pointIsInsideGlobalMap(point) {
  const grid = mapLayers.map.grid;
  if (!grid) return false;
  const cell = mapGeometry.worldToGrid(grid, point.x, point.y);
  return cell.x >= 0 && cell.y >= 0 && cell.x < grid.width && cell.y < grid.height;
}

mapCanvas.addEventListener("pointerdown", (event) => {
  if (mapMode === "view") return;
  if (mapMode === "goal") {
    const point = pointerWorld(event);
    if (!pointIsInsideGlobalMap(point)) {
      ack("goal must be inside the current global map");
      return;
    }
    mapCanvas.setPointerCapture(event.pointerId);
    goalInteraction.dragging = true;
    goalInteraction.pointerId = event.pointerId;
    goalInteraction.preview = { x: point.x, y: point.y, yaw: 0 };
    goalInteraction.awaiting = false;
    renderGoalControls();
    renderMap();
    return;
  }
  mapCanvas.setPointerCapture(event.pointerId);
  const point = pointerCanvas(event);
  mapDrag = { pointerId: event.pointerId, x: point.x, y: point.y };
  mapCanvas.classList.add("dragging");
});

mapCanvas.addEventListener("pointermove", (event) => {
  if (goalInteraction.dragging && goalInteraction.pointerId === event.pointerId) {
    const point = pointerWorld(event);
    const dx = point.x - goalInteraction.preview.x;
    const dy = point.y - goalInteraction.preview.y;
    if (Math.hypot(dx, dy) > 0.01) goalInteraction.preview.yaw = Math.atan2(dy, dx);
    renderGoalControls();
    renderMap();
    return;
  }
  if (!mapDrag || mapDrag.pointerId !== event.pointerId) return;
  const point = pointerCanvas(event);
  if (mapMode === "pan") {
    const before = worldFromScreen(mapDrag.x, mapDrag.y);
    const after = worldFromScreen(point.x, point.y);
    mapView.x += before.x - after.x;
    mapView.y += before.y - after.y;
    mapView.fitted = true;
  } else if (mapMode === "rotate") {
    mapView.rotation += (point.x - mapDrag.x) * 0.008 /
      (window.devicePixelRatio || 1);
    mapView.fitted = true;
  }
  mapDrag.x = point.x;
  mapDrag.y = point.y;
  renderMap();
});

function endMapDrag(event) {
  if (goalInteraction.dragging && goalInteraction.pointerId === event.pointerId) {
    goalInteraction.dragging = false;
    goalInteraction.pointerId = null;
    renderGoalControls();
    return;
  }
  if (!mapDrag || mapDrag.pointerId !== event.pointerId) return;
  mapDrag = null;
  mapCanvas.classList.remove("dragging");
}

mapCanvas.addEventListener("pointerup", endMapDrag);
mapCanvas.addEventListener("pointercancel", endMapDrag);
$("btn-fit-map").addEventListener("click", fitMap);
$("btn-zoom-in").addEventListener("click", () => zoomMap(1.25));
$("btn-zoom-out").addEventListener("click", () => zoomMap(0.8));
document.querySelectorAll("[data-map-mode]").forEach((button) => {
  button.addEventListener("click", () => setMapMode(button.dataset.mapMode));
});
new ResizeObserver(resizeMapCanvas).observe($("map-viewport"));

$("btn-cancel-goal-preview").addEventListener("click", () => setMapMode("view"));
$("btn-confirm-goal").addEventListener("click", () => {
  const preview = goalInteraction.preview;
  if (!preview || goalInteraction.awaiting) return;
  goalInteraction.awaiting = true;
  send({ action: "select_goal", frame: "map", x: preview.x, y: preview.y, yaw: preview.yaw });
  renderGoalControls();
});

// --- maps ---------------------------------------------------------------

function renderCatalog(catalog, selected) {
  const renderKey = JSON.stringify([role, selected, catalog]);
  if (renderKey === catalogRenderKey) return;
  catalogRenderKey = renderKey;
  const list = $("map-catalog");
  list.innerHTML = "";
  if (!catalog.length) {
    const empty = document.createElement("li");
    empty.className = "hint";
    empty.textContent = "no saved maps";
    list.appendChild(empty);
    return;
  }
  for (const entry of catalog) {
    const item = document.createElement("li");
    const info = document.createElement("div");
    info.className = "catalog-info";
    const name = document.createElement("strong");
    name.textContent = `${entry.name}${entry.name === selected ? " · SELECTED" : ""}`;
    const meta = document.createElement("span");
    meta.textContent = entry.complete
      ? `${entry.width}×${entry.height} · ${entry.resolution.toFixed(3)} m/cell${entry.revision ? ` · rev ${entry.revision}` : ""}`
      : `INCOMPLETE · ${entry.reason || "invalid bundle"}`;
    info.append(name, meta);
    const actions = document.createElement("div");
    actions.className = "catalog-actions";
    const select = document.createElement("button");
    select.textContent = entry.name === selected ? "Selected" : "Select";
    select.disabled = role !== "controller" || !entry.complete || entry.name === selected;
    select.addEventListener("click", () => send({ action: "select_map", name: entry.name }));
    const remove = document.createElement("button");
    remove.textContent = "Delete";
    remove.className = "danger";
    remove.disabled = role !== "controller" || !entry.complete || entry.name === selected;
    if (entry.name === selected) remove.title = "Select another map before deleting this one";
    remove.addEventListener("click", () => openDeleteDialog(entry.name));
    actions.append(select, remove);
    item.append(info, actions);
    list.appendChild(item);
  }
}

function openDeleteDialog(name) {
  pendingDeleteName = name;
  text("delete-map-name", name);
  $("delete-dialog").showModal();
}

$("delete-dialog").addEventListener("close", () => {
  if ($("delete-dialog").returnValue === "delete" && pendingDeleteName) {
    send({ action: "delete_map", name: pendingDeleteName });
  }
  pendingDeleteName = "";
});

function confirmRecordingDelete(name) {
  pendingDeleteRecordingName = name;
  text("delete-recording-name", name);
  $("delete-recording-dialog").showModal();
}

$("delete-recording-dialog").addEventListener("close", () => {
  if (
    $("delete-recording-dialog").returnValue === "delete" &&
    pendingDeleteRecordingName
  ) {
    send({ action: "delete_recording", name: pendingDeleteRecordingName });
  }
  pendingDeleteRecordingName = "";
});

// --- global state helpers ----------------------------------------------

function setBannerFromState(mode, auth, stop) {
  if (stop.stopped) setBanner("STOP ASSERTED — global inhibit latched", "stop");
  else if (mode.status === 2) setBanner(`Runtime fault — ${humanDetail(mode.detail || mode.readiness_reason)}`, "stop");
  else if (mode.status === 1) setBanner(humanDetail(mode.detail || "Runtime transition in progress"), "waiting");
  else if (auth.dualsense_active) setBanner("DUALSENSE TAKEOVER — local manual control", "takeover");
  else if (auth.authority === 3 && auth.autonomy_action_active) setBanner("AUTONOMOUS — Nav2 executing under RUN", "run");
  else if (!mode.ready && mode.mode !== 0) setBanner(`Not ready — ${humanDetail(mode.readiness_reason)}`, "waiting");
  else setBanner(`Connected · ${role}`, "connected");
}

function humanDetail(value) {
  return String(value || "unknown reason").replaceAll("->", "→");
}

function stopSummary(stop, auth) {
  if (!stop.stamp && !auth.stop_state_fresh) return "unknown (fail closed)";
  const bits = [];
  if (stop.stopped) bits.push("ASSERTED");
  if (stop.locked) bits.push("locked");
  if (stop.clear_pending) bits.push("clear-pending");
  if (!stop.healthy) bits.push("ENFORCER UNHEALTHY");
  if (!stop.stopped && stop.healthy) bits.push("clear");
  return `${bits.join(", ")} · ${stop.reason || ""}`;
}

function fmt(value) {
  return typeof value === "number" && Number.isFinite(value) ? value.toFixed(2) : "—";
}

// --- controls and deadman ----------------------------------------------

$("btn-stop").addEventListener("click", () => {
  stopRun();
  releaseManual();
  send({ action: "stop" });
});
$("btn-clear-stop").addEventListener("click", () => send({ action: "clear_stop" }));
$("btn-clear-obstacles").addEventListener("click", () => send({ action: "clear_obstacles" }));
function toggleRecording() {
  const state = (latest.recording_state || {}).state;
  if ([1, 2].includes(state)) {
    send({ action: "stop_recording" });
  } else if (state !== 3) {
    send({
      action: "start_recording",
      name: $("recording-name").value.trim(),
      profile: $("recording-profile").value,
    });
  }
}
$("btn-record").addEventListener("click", toggleRecording);
$("btn-recording-form").addEventListener("click", toggleRecording);
$("btn-new-map").addEventListener("click", () => send({ action: "new_map" }));
$("btn-new-map-from-maps").addEventListener("click", () => send({ action: "new_map" }));
$("btn-save-map").addEventListener("click", () => send({ action: "save_map", name: $("save-name").value.trim() }));
$("btn-select-goal").addEventListener("click", () => send({
  action: "select_goal",
  x: Number($("goal-x").value),
  y: Number($("goal-y").value),
  yaw: Number($("goal-yaw").value),
}));
document.querySelectorAll("button.mode").forEach((button) => {
  button.addEventListener("click", () => send({ action: "select_mode", mode: button.dataset.mode }));
});

const runButton = $("btn-run");

function runIsAvailable() {
  return role === "controller" && socketReady && !runButton.disabled && !$("run-controls").hidden;
}

function startRun(event, source = "pointer") {
  if (event) event.preventDefault();
  if (!runIsAvailable() || runHeld) return;
  runHeld = true;
  keyboardRun = source === "keyboard";
  send({ action: "run", held: true });
  runTimer = window.setInterval(() => send({ action: "run", held: true }), 100);
  runButton.classList.add("armed");
}

function stopRun() {
  if (!runHeld) return;
  runHeld = false;
  keyboardRun = false;
  window.clearInterval(runTimer);
  send({ action: "run", held: false });
  runButton.classList.remove("armed");
}

function cancelRun() {
  if (runHeld) stopRun();
  else send({ action: "run", held: false });
}

const joystick = $("manual-joystick");
const joystickKnob = $("joystick-knob");

function manualIsAvailable() {
  return socketReady && role === "controller" &&
    joystick.getAttribute("aria-disabled") !== "true";
}

function updateManualFromPointer(event) {
  const bounds = joystick.getBoundingClientRect();
  const halfWidth = bounds.width / 2;
  const halfHeight = bounds.height / 2;
  const rawX = (event.clientX - (bounds.left + halfWidth)) / halfWidth;
  const rawY = (event.clientY - (bounds.top + halfHeight)) / halfHeight;
  const config = latest.config || {};
  const maxSpeed = Number.isFinite(config.applied_value)
    ? config.applied_value : 0.40;
  const demand = joystickGeometry.demandFromAxes(rawX, rawY, maxSpeed);
  manualDemand = {
    speed_mps: demand.speed_mps,
    steering: demand.steering,
  };
  joystickKnob.style.left = `${50 + demand.x * 36}%`;
  joystickKnob.style.top = `${50 + demand.y * 36}%`;
}

function sendManualSample() {
  if (!manualEngaged || !manualIsAvailable()) return;
  send({ action: "manual", active: true, ...manualDemand });
}

function engageManual(event) {
  if (!manualIsAvailable() || manualEngaged) return;
  event.preventDefault();
  stopRun();
  manualEngaged = true;
  manualPointerId = event.pointerId;
  joystick.setPointerCapture(event.pointerId);
  joystick.classList.add("engaged");
  updateManualFromPointer(event);
  sendManualSample();
  manualTimer = window.setInterval(sendManualSample, 100);
}

function resetManualLocal() {
  manualEngaged = false;
  manualPointerId = null;
  manualDemand = { speed_mps: 0, steering: 0 };
  window.clearInterval(manualTimer);
  manualTimer = null;
  if (joystick) {
    joystick.classList.remove("engaged");
    joystickKnob.style.left = "50%";
    joystickKnob.style.top = "50%";
  }
}

function releaseManual() {
  if (!manualEngaged) return;
  const pointerId = manualPointerId;
  resetManualLocal();
  send({ action: "manual", active: false, speed_mps: 0, steering: 0 });
  if (pointerId !== null && joystick.hasPointerCapture(pointerId)) {
    joystick.releasePointerCapture(pointerId);
  }
}

joystick.addEventListener("pointerdown", engageManual);
joystick.addEventListener("pointermove", (event) => {
  if (!manualEngaged || event.pointerId !== manualPointerId) return;
  event.preventDefault();
  updateManualFromPointer(event);
  sendManualSample();
});
joystick.addEventListener("pointerup", releaseManual);
joystick.addEventListener("pointercancel", releaseManual);
joystick.addEventListener("lostpointercapture", releaseManual);

function isEditableOrInteractive(target) {
  return target instanceof Element && Boolean(target.closest(
    "input, textarea, select, button, [contenteditable='true'], [role='textbox']",
  ));
}

runButton.addEventListener("pointerdown", (event) => startRun(event, "pointer"));
runButton.addEventListener("pointerup", stopRun);
runButton.addEventListener("pointerleave", stopRun);
runButton.addEventListener("pointercancel", stopRun);
$("btn-cancel").addEventListener("click", cancelRun);
$("btn-apply-manual-speed").addEventListener("click", () => {
  const config = latest.config || {};
  send({
    action: "set_config",
    field: "manual_max_speed_mps",
    value: Number($("manual-max-speed-input").value),
    expected_revision: Number(config.revision || 0),
  });
});
window.addEventListener("keydown", (event) => {
  if (event.code !== "Space" || event.repeat || isEditableOrInteractive(event.target)) return;
  if (runIsAvailable()) startRun(event, "keyboard");
});
window.addEventListener("keyup", (event) => {
  if (event.code !== "Space" || !keyboardRun) return;
  if (!isEditableOrInteractive(event.target)) event.preventDefault();
  stopRun();
});
window.addEventListener("blur", () => { stopRun(); releaseManual(); });
document.addEventListener("visibilitychange", () => {
  if (document.hidden) { stopRun(); releaseManual(); }
});

function debugRender() {
  $("debug").textContent = JSON.stringify(latest, (key, value) => key === "data" ? "[grid]" : value, 2);
}

window.setInterval(debugRender, 500);
initializeLayerControls();
setMapMode("view");
connect();
if ("serviceWorker" in navigator) {
  navigator.serviceWorker.register("/static/service-worker.js").catch(() => {});
}
