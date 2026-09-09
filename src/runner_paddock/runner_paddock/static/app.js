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
let catalogRenderKey = null;
let pendingDeleteName = "";

const mapCanvas = $("map-canvas");
const mapContext = mapCanvas.getContext("2d");
const mapGeometry = window.PaddockMapGeometry;
const mapView = { x: 0, y: 0, scale: 50, rotation: 0, fitted: false };
const mapLayers = {
  map: { grid: null, raster: null },
  local_costmap: { grid: null, raster: null },
};
let mapMode = "view";
let mapDrag = null;
const goalInteraction = {
  dragging: false,
  pointerId: null,
  preview: null,
  awaiting: false,
};

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
    } else if (frame.type === "map" || frame.type === "local_costmap" || frame.type === "plan") {
      latest[frame.type] = frame;
      if (frame.type === "map" || frame.type === "local_costmap") {
        updateMapLayer(frame.type, frame);
      } else {
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
    window.clearInterval(heartbeatTimer);
    window.clearInterval(runTimer);
    $("btn-run").classList.remove("armed");
    setBanner("Disconnected — retrying (RUN is revoked)", "waiting");
    window.setTimeout(connect, 1000);
  });
}

// --- tabs ---------------------------------------------------------------

function selectTab(name) {
  stopRun();
  document.querySelectorAll(".tab").forEach((tab) => {
    const selected = tab.dataset.tab === name;
    tab.classList.toggle("active", selected);
    tab.setAttribute("aria-selected", String(selected));
    const panel = $(`panel-${tab.dataset.tab}`);
    panel.hidden = !selected;
    panel.classList.toggle("active", selected);
  });
  window.requestAnimationFrame(resizeMapCanvas);
}

document.querySelectorAll(".tab").forEach((tab) => {
  tab.addEventListener("click", () => selectTab(tab.dataset.tab));
});

// --- render -------------------------------------------------------------

const RUNTIME = ["IDLE", "MAPPING", "AUTONOMY"];
const RUNTIME_STATUS = ["STABLE", "TRANSITIONING", "FAULT"];
const AUTHORITY = ["NONE", "DUALSENSE", "PADDOCK_MANUAL", "PADDOCK_AUTONOMY"];
const MISSION = ["IDLE", "DISPATCHING", "ACTIVE", "CANCELING", "SUCCEEDED", "FAILED", "CANCELED"];

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
  const health = (latest.health || {}).status || "?";
  const pose = latest.pose;

  setBannerFromState(mode, auth, stop);
  text("g-backend", health);
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
    "button.mode, #btn-clear-stop, #btn-new-map, #btn-new-map-from-maps, " +
    "#btn-save-map, #btn-select-goal, #btn-run, #btn-cancel, " +
    "#btn-goal-mode, #btn-confirm-delete",
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
  const runRelevant = mode.mode === 2 && mode.status === 0 && mode.ready &&
    Boolean(auth.autonomy_goal_selected);
  $("run-controls").hidden = !runRelevant;
  $("run-hint").hidden = !runRelevant;
  if (!runRelevant) stopRun();

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
  mapLayers[kind] = { grid, raster: makeGridRaster(grid, kind === "local_costmap") };
  if (kind === "map" && !mapView.fitted) fitMap();
  renderMap();
}

function makeGridRaster(grid, local) {
  if (!grid || grid.width <= 0 || grid.height <= 0 || grid.data.length !== grid.width * grid.height) return null;
  const raster = document.createElement("canvas");
  raster.width = grid.width;
  raster.height = grid.height;
  const context = raster.getContext("2d");
  const image = context.createImageData(grid.width, grid.height);
  for (let gy = 0; gy < grid.height; gy += 1) {
    const canvasY = grid.height - 1 - gy;
    for (let gx = 0; gx < grid.width; gx += 1) {
      const value = grid.data[gy * grid.width + gx];
      const offset = (canvasY * grid.width + gx) * 4;
      if (local) {
        if (value < 0) image.data.set([130, 92, 180, 65], offset);
        else if (value === 0) image.data.set([0, 0, 0, 0], offset);
        else image.data.set([235, 64, 38, Math.round(55 + 180 * value / 100)], offset);
      } else if (value < 0) {
        image.data.set([112, 119, 114, 255], offset);
      } else {
        const shade = Math.round(235 - 215 * value / 100);
        image.data.set([shade, shade + 3, shade, 255], offset);
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

function drawGridLayer(layer) {
  const { grid, raster } = layer;
  if (!grid || !raster) return;
  const angle = mapGeometry.yawOf(grid.origin.orientation) + mapView.rotation;
  const topLeft = mapGeometry.gridToWorld(grid, 0, grid.height);
  const screen = screenFromWorld(topLeft.x, topLeft.y);
  const cellPixels = mapView.scale * grid.resolution;
  mapContext.save();
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

function drawPlan() {
  const plan = latest.plan;
  const source = ((latest.health || {}).sources || {}).plan;
  if (!planIsInMapFrame(plan) || (source && !source.fresh) || plan.poses.length < 2) return;
  mapContext.save();
  mapContext.strokeStyle = "#ffc247";
  mapContext.lineWidth = 3 * devicePixelRatio;
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
  drawGridLayer(mapLayers.map);
  const sources = ((latest.health || {}).sources || {});
  const localFresh = !sources.local_costmap || sources.local_costmap.fresh;
  const poseFresh = !sources.pose || sources.pose.fresh;
  if (localFresh) drawGridLayer(mapLayers.local_costmap);
  drawPlan();
  const auth = latest.command_authority || {};
  if (auth.autonomy_goal_selected) {
    drawDirectionalPose(poseFromXYYaw(auth.goal_x, auth.goal_y, auth.goal_yaw), "#90e0ef", 6 * devicePixelRatio);
  }
  if (poseFresh) drawDirectionalPose(latest.pose, "#00b4d8", 7 * devicePixelRatio);
  if (goalInteraction.preview) {
    drawDirectionalPose(poseFromXYYaw(
      goalInteraction.preview.x, goalInteraction.preview.y, goalInteraction.preview.yaw,
    ), "#ffc247", 6 * devicePixelRatio);
  }
  const global = mapLayers.map.grid;
  const local = mapLayers.local_costmap.grid;
  const plan = latest.plan;
  let planStatus = "plan unavailable";
  if (plan) {
    if (!planIsInMapFrame(plan)) planStatus = `plan frame rejected (${plan.frame_id || "empty"})`;
    else if (sources.plan && !sources.plan.fresh) planStatus = "plan stale";
    else planStatus = `plan ${plan.poses.length} points`;
  }
  $("map-status").textContent = global
    ? `${global.width}×${global.height} · ${global.resolution.toFixed(3)} m/cell · ${global.frame_id || "?"} · ` +
      (local && localFresh ? `local costmap from ${local.source_frame_id || local.frame_id}` : `local costmap ${local ? "stale" : "unavailable"}`) +
      ` · ${planStatus}`
    : "Waiting for /map…";
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
  send({ action: "stop" });
});
$("btn-clear-stop").addEventListener("click", () => send({ action: "clear_stop" }));
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
window.addEventListener("keydown", (event) => {
  if (event.code !== "Space" || event.repeat || isEditableOrInteractive(event.target)) return;
  if (runIsAvailable()) startRun(event, "keyboard");
});
window.addEventListener("keyup", (event) => {
  if (event.code !== "Space" || !keyboardRun) return;
  if (!isEditableOrInteractive(event.target)) event.preventDefault();
  stopRun();
});
window.addEventListener("blur", stopRun);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopRun();
});

function debugRender() {
  $("debug").textContent = JSON.stringify(latest, (key, value) => key === "data" ? "[grid]" : value, 2);
}

window.setInterval(debugRender, 500);
setMapMode("view");
connect();
if ("serviceWorker" in navigator) {
  navigator.serviceWorker.register("/static/service-worker.js").catch(() => {});
}
