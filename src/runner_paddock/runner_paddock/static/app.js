"use strict";

// Minimal Paddock operator console. Renders backend truth only; the browser
// keeps no optimistic RUN / goal / STOP-clear state across a reconnect.

const $ = (id) => document.getElementById(id);
const banner = $("banner");
const acklog = $("acklog");

let socket = null;
let socketReady = false;
let role = "observer";
const latest = {};          // last state snapshot fields
let heartbeatTimer = null;
let runHeld = false;
let runTimer = null;
let catalogRenderKey = null;

const mapCanvas = $("map-canvas");
const mapContext = mapCanvas.getContext("2d");
const mapGeometry = window.PaddockMapGeometry;
const mapView = { x: 0, y: 0, scale: 50, fitted: false };
const mapLayers = {
  map: { grid: null, raster: null },
  local_costmap: { grid: null, raster: null },
};
let mapDrag = null;
const goalInteraction = {
  active: false,
  dragging: false,
  pointerId: null,
  preview: null,
  awaiting: false,
};

function setBanner(text, kind) {
  banner.textContent = text;
  banner.className = `banner ${kind}`;
}

function send(action) {
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    ack(`not connected: ${action.action}`);
    return;
  }
  socket.send(JSON.stringify(action));
}

function ack(text) {
  const stamp = new Date().toLocaleTimeString();
  acklog.textContent = `${stamp}  ${text}`;
}

// --- connection -----------------------------------------------------------

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
      if (frame.type === "map" || frame.type === "local_costmap") updateMapLayer(frame.type, frame);
    } else if (frame.type === "ack") {
      if (typeof frame.role === "string") role = frame.role;
      if (frame.name === "select_goal" && !frame.accepted) goalInteraction.awaiting = false;
      ack(`${frame.name || "action"}: ${frame.accepted ? "ok" : "REJECTED"} — ${frame.reason}`);
      render();
    }
  });

  socket.addEventListener("close", () => {
    socketReady = false;
    role = "observer";
    runHeld = false;
    window.clearInterval(heartbeatTimer);
    window.clearInterval(runTimer);
    setBanner("Disconnected — retrying (RUN / goal / STOP-clear are NOT retained)", "waiting");
    window.setTimeout(connect, 1000);
  });
}

// --- render --------------------------------------------------------------

const RUNTIME = ["IDLE", "MAPPING", "AUTONOMY"];
const RUNTIME_STATUS = ["STABLE", "TRANSITIONING", "FAULT"];
const AUTHORITY = ["NONE", "DUALSENSE", "PADDOCK_MANUAL", "PADDOCK_AUTONOMY"];
const MISSION = ["IDLE", "DISPATCHING", "ACTIVE", "CANCELING", "SUCCEEDED", "FAILED", "CANCELED"];

function text(id, value) { $(id).textContent = value; }
function flag(id, value, goodWhenTrue = true) {
  const el = $(id);
  el.textContent = value ? "yes" : "no";
  el.className = value === goodWhenTrue ? "ok" : "bad";
}

function render() {
  const mode = latest.mode || {};
  const auth = latest.command_authority || {};
  const lease = latest.control_lease || {};
  const stop = latest.stop_state || {};
  const local = latest.local_control || {};
  const gw = latest.gateway || {};
  const mapState = latest.map_state || {};
  const nav = latest.navigation_state || {};
  const health = (latest.health || {}).status || "?";
  const pose = latest.pose;

  setBannerFromState(mode, auth, stop);

  text("g-backend", `${health}`);
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

  text("m-session", mode.mapping_session_id || "—");
  text("m-phase", ["NONE", "STARTING", "READY", "SAVING", "SAVED", "FAILED"][mapState.session_phase] ?? "—");
  text("m-unsaved", mapState.unsaved ? "unsaved content" : "—");
  text("m-save", `${["idle", "running", "succeeded", "failed"][mapState.save_state] ?? "—"} · ${mapState.save_detail || ""}`);
  text("m-selected", mapState.selected_map_applied || "(none)");
  text("m-selection-result", mapState.selected_map_reason
    ? `REJECTED ${mapState.selected_map_requested || "(unnamed)"} — ${mapState.selected_map_reason}`
    : (mapState.selected_map_applied
      ? `applied — ${mapState.selected_map_applied}`
      : "—"));
  renderCatalog(mapState.catalog || [], mapState.selected_map_applied);
  renderMap();

  text("a-map", mode.active_autonomy_map || "(none)");
  text("a-mission", `${MISSION[nav.state] ?? "—"}${nav.mission_valid ? " · mission valid" : ""} · rev ${nav.mission_revision ?? 0}`);
  text("a-goal", auth.autonomy_goal_selected
    ? `map · x ${fmt(auth.goal_x)} · y ${fmt(auth.goal_y)} · yaw ${fmt(auth.goal_yaw)} rad`
    : "none");
  text("a-detail", nav.detail || nav.error_meaning || "—");
  text("a-active", auth.autonomy_action_active ? "yes (Nav2 executing)" : "no");
  $("pose-hint").textContent = pose
    ? `Robot pose: x=${fmt(pose.position.x)} y=${fmt(pose.position.y)}`
    : "Robot pose: — (no map->base_link TF)";

  const controller = role === "controller";
  document.querySelectorAll("button").forEach((b) => {
    if (b.classList.contains("view-control")) return;
    if (b.id === "btn-stop") return;               // STOP stays enabled when possible
    b.disabled = !controller;
  });
  $("btn-stop").disabled = !controller;
  $("btn-run").classList.toggle("armed", runHeld);

  // AUTONOMY needs a completed map selected first; keep the control out of
  // reach until then and say why, rather than let the runtime fault.
  const hasAutonomyMap = Boolean(mapState.selected_map_applied);
  const autoBtn = document.querySelector('button[data-mode="autonomy"]');
  if (autoBtn) autoBtn.disabled = !controller || !hasAutonomyMap;
  $("a-map-hint").textContent = hasAutonomyMap
    ? ""
    : "Select a completed map (Mapping ▸ Saved maps) before AUTONOMY.";
  const runRelevant = mode.mode === 2 && mode.status === 0 &&
    Boolean(auth.autonomy_goal_selected);
  $("run-controls").hidden = !runRelevant;
  $("run-hint").hidden = !runRelevant;

  const preview = goalInteraction.preview;
  if (preview && goalInteraction.awaiting && auth.autonomy_goal_selected &&
      Math.abs(auth.goal_x - preview.x) < 1e-6 &&
      Math.abs(auth.goal_y - preview.y) < 1e-6 &&
      Math.abs(auth.goal_yaw - preview.yaw) < 1e-6) {
    goalInteraction.active = false;
    goalInteraction.awaiting = false;
    goalInteraction.preview = null;
  }
  renderGoalControls();
}

// --- map -----------------------------------------------------------------

function updateMapLayer(kind, grid) {
  mapLayers[kind] = { grid, raster: makeGridRaster(grid, kind === "local_costmap") };
  if (kind === "map" && !mapView.fitted) fitMap();
  renderMap();
}

function makeGridRaster(grid, local) {
  if (!grid || grid.width <= 0 || grid.height <= 0 ||
      grid.data.length !== grid.width * grid.height) return null;
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
        if (value < 0) {
          image.data.set([130, 92, 180, 65], offset);
        } else if (value === 0) {
          image.data.set([0, 0, 0, 0], offset);
        } else {
          const alpha = Math.round(55 + 180 * value / 100);
          image.data.set([235, 64, 38, alpha], offset);
        }
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
  return {
    x: mapCanvas.width / 2 + (x - mapView.x) * mapView.scale,
    y: mapCanvas.height / 2 - (y - mapView.y) * mapView.scale,
  };
}

function worldFromScreen(x, y) {
  return {
    x: mapView.x + (x - mapCanvas.width / 2) / mapView.scale,
    y: mapView.y - (y - mapCanvas.height / 2) / mapView.scale,
  };
}

function drawGridLayer(layer) {
  const { grid, raster } = layer;
  if (!grid || !raster) return;
  const yaw = mapGeometry.yawOf(grid.origin.orientation);
  const topLeft = mapGeometry.gridToWorld(grid, 0, grid.height);
  const screen = screenFromWorld(topLeft.x, topLeft.y);
  const cellPixels = mapView.scale * grid.resolution;
  const cosine = Math.cos(yaw);
  const sine = Math.sin(yaw);
  mapContext.save();
  mapContext.imageSmoothingEnabled = false;
  mapContext.setTransform(
    cellPixels * cosine,
    -cellPixels * sine,
    cellPixels * sine,
    cellPixels * cosine,
    screen.x,
    screen.y,
  );
  mapContext.drawImage(raster, 0, 0);
  mapContext.restore();
}

function drawDirectionalPose(pose, color, radiusPixels) {
  if (!pose || !pose.position || !pose.orientation) return;
  const center = screenFromWorld(pose.position.x, pose.position.y);
  const yaw = mapGeometry.yawOf(pose.orientation);
  const length = Math.max(radiusPixels * 2.8, 28 * devicePixelRatio);
  const tip = {
    x: center.x + Math.cos(yaw) * length,
    y: center.y - Math.sin(yaw) * length,
  };
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
  mapContext.lineTo(tip.x - Math.cos(yaw - 0.55) * 10 * devicePixelRatio,
                    tip.y + Math.sin(yaw - 0.55) * 10 * devicePixelRatio);
  mapContext.lineTo(tip.x - Math.cos(yaw + 0.55) * 10 * devicePixelRatio,
                    tip.y + Math.sin(yaw + 0.55) * 10 * devicePixelRatio);
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

function drawGoalPreview(preview) {
  if (!preview) return;
  const pose = poseFromXYYaw(preview.x, preview.y, preview.yaw);
  drawDirectionalPose(pose, "#ffc247", 6 * devicePixelRatio);
}

function renderMap() {
  mapContext.setTransform(1, 0, 0, 1, 0, 0);
  mapContext.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  drawGridLayer(mapLayers.map);
  drawGridLayer(mapLayers.local_costmap);
  const auth = latest.command_authority || {};
  if (auth.autonomy_goal_selected) {
    drawDirectionalPose(
      poseFromXYYaw(auth.goal_x, auth.goal_y, auth.goal_yaw),
      "#90e0ef",
      6 * devicePixelRatio,
    );
  }
  drawDirectionalPose(latest.pose, "#00b4d8", 7 * devicePixelRatio);
  drawGoalPreview(goalInteraction.preview);
  const global = mapLayers.map.grid;
  const local = mapLayers.local_costmap.grid;
  $("map-status").textContent = global
    ? `${global.width}×${global.height} · ${global.resolution.toFixed(3)} m/cell · ${global.frame_id || "?"}` +
      (local ? ` · local costmap aligned from ${local.source_frame_id || local.frame_id}` : " · local costmap unavailable")
    : "Waiting for /map…";
}

function renderGoalControls() {
  const controls = $("goal-preview-controls");
  const preview = goalInteraction.preview;
  controls.hidden = !goalInteraction.active;
  $("btn-goal-mode").classList.toggle("armed", goalInteraction.active);
  mapCanvas.classList.toggle("selecting", goalInteraction.active);
  $("btn-confirm-goal").disabled = role !== "controller" || !preview || goalInteraction.awaiting;
  $("goal-preview-text").textContent = !preview
    ? "Press on the map and drag toward the desired heading"
    : goalInteraction.awaiting
      ? "Sending goal — waiting for command-authority confirmation…"
      : `Preview: x ${preview.x.toFixed(2)} · y ${preview.y.toFixed(2)} · yaw ${preview.yaw.toFixed(2)} rad`;
  renderMap();
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
  const bounds = mapGeometry.gridBounds(grid);
  const width = Math.max(bounds.maxX - bounds.minX, grid.resolution);
  const height = Math.max(bounds.maxY - bounds.minY, grid.resolution);
  mapView.x = (bounds.minX + bounds.maxX) / 2;
  mapView.y = (bounds.minY + bounds.maxY) / 2;
  mapView.scale = Math.max(2, 0.9 * Math.min(mapCanvas.width / width, mapCanvas.height / height));
  mapView.fitted = true;
  renderMap();
}

function zoomMap(factor, screenX = mapCanvas.width / 2, screenY = mapCanvas.height / 2) {
  const anchor = worldFromScreen(screenX, screenY);
  mapView.scale = Math.min(5000, Math.max(2, mapView.scale * factor));
  mapView.x = anchor.x - (screenX - mapCanvas.width / 2) / mapView.scale;
  mapView.y = anchor.y + (screenY - mapCanvas.height / 2) / mapView.scale;
  renderMap();
}

mapCanvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const rect = mapCanvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  zoomMap(event.deltaY < 0 ? 1.15 : 1 / 1.15,
          (event.clientX - rect.left) * ratio, (event.clientY - rect.top) * ratio);
}, { passive: false });
mapCanvas.addEventListener("pointerdown", (event) => {
  mapCanvas.setPointerCapture(event.pointerId);
  if (goalInteraction.active) {
    const point = pointerWorld(event);
    if (!pointIsInsideGlobalMap(point)) {
      ack("goal must be inside the current global map");
      return;
    }
    goalInteraction.dragging = true;
    goalInteraction.pointerId = event.pointerId;
    goalInteraction.preview = { x: point.x, y: point.y, yaw: 0 };
    goalInteraction.awaiting = false;
    renderGoalControls();
    return;
  }
  mapDrag = { pointerId: event.pointerId, x: event.clientX, y: event.clientY };
  mapCanvas.classList.add("dragging");
});
mapCanvas.addEventListener("pointermove", (event) => {
  if (goalInteraction.dragging && goalInteraction.pointerId === event.pointerId) {
    const point = pointerWorld(event);
    const preview = goalInteraction.preview;
    const dx = point.x - preview.x;
    const dy = point.y - preview.y;
    if (Math.hypot(dx, dy) > 0.01) preview.yaw = Math.atan2(dy, dx);
    renderGoalControls();
    return;
  }
  if (!mapDrag || mapDrag.pointerId !== event.pointerId) return;
  const ratio = window.devicePixelRatio || 1;
  mapView.x -= (event.clientX - mapDrag.x) * ratio / mapView.scale;
  mapView.y += (event.clientY - mapDrag.y) * ratio / mapView.scale;
  mapDrag.x = event.clientX;
  mapDrag.y = event.clientY;
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
new ResizeObserver(resizeMapCanvas).observe($("map-viewport"));

function pointerWorld(event) {
  const rect = mapCanvas.getBoundingClientRect();
  const ratio = window.devicePixelRatio || 1;
  return worldFromScreen(
    (event.clientX - rect.left) * ratio,
    (event.clientY - rect.top) * ratio,
  );
}

function pointIsInsideGlobalMap(point) {
  const grid = mapLayers.map.grid;
  if (!grid) return false;
  const cell = mapGeometry.worldToGrid(grid, point.x, point.y);
  return cell.x >= 0 && cell.y >= 0 && cell.x < grid.width && cell.y < grid.height;
}

$("btn-goal-mode").addEventListener("click", () => {
  goalInteraction.active = !goalInteraction.active;
  goalInteraction.dragging = false;
  goalInteraction.preview = null;
  goalInteraction.awaiting = false;
  renderGoalControls();
});
$("btn-cancel-goal-preview").addEventListener("click", () => {
  goalInteraction.active = false;
  goalInteraction.dragging = false;
  goalInteraction.preview = null;
  goalInteraction.awaiting = false;
  renderGoalControls();
});
$("btn-confirm-goal").addEventListener("click", () => {
  const preview = goalInteraction.preview;
  if (!preview || goalInteraction.awaiting) return;
  goalInteraction.awaiting = true;
  send({ action: "select_goal", frame: "map", x: preview.x, y: preview.y, yaw: preview.yaw });
  renderGoalControls();
});

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

function renderCatalog(catalog, selected) {
  // State arrives at 10 Hz. Keep the actual buttons mounted while their
  // backend-derived content is unchanged so a pointer/touch gesture cannot
  // lose its click target between press and release.
  const renderKey = JSON.stringify([role, selected, catalog]);
  if (renderKey === catalogRenderKey) return;
  catalogRenderKey = renderKey;

  const list = $("map-catalog");
  list.innerHTML = "";
  if (!catalog.length) {
    list.innerHTML = "<li class='hint'>no saved maps</li>";
    return;
  }
  for (const entry of catalog) {
    const li = document.createElement("li");
    const label = `${entry.name}${entry.complete ? "" : " (incomplete)"}` +
      (entry.name === selected ? "  ◀ selected" : "");
    li.textContent = label;
    if (entry.complete) {
      const btn = document.createElement("button");
      btn.textContent = "select";
      btn.disabled = role !== "controller";
      btn.addEventListener("click", () => send({ action: "select_map", name: entry.name }));
      li.appendChild(btn);
    }
    list.appendChild(li);
  }
}

function fmt(value) {
  return typeof value === "number" && isFinite(value) ? value.toFixed(2) : "—";
}

// --- controls -----------------------------------------------------------

$("btn-stop").addEventListener("click", () => send({ action: "stop" }));
$("btn-clear-stop").addEventListener("click", () => send({ action: "clear_stop" }));
$("btn-new-map").addEventListener("click", () => send({ action: "new_map" }));
$("btn-save-map").addEventListener("click", () =>
  send({ action: "save_map", name: $("save-name").value.trim() }));
$("btn-cancel").addEventListener("click", () => send({ action: "run", held: false }));
$("btn-select-goal").addEventListener("click", () => send({
  action: "select_goal",
  x: Number($("goal-x").value),
  y: Number($("goal-y").value),
  yaw: Number($("goal-yaw").value),
}));

document.querySelectorAll("button.mode").forEach((btn) => {
  btn.addEventListener("click", () => send({ action: "select_mode", mode: btn.dataset.mode }));
});

// Hold-to-run: press starts a fast repeat that keeps the lease fresh; any
// release path (pointer up/leave, blur, tab hide) sends RUN released.
const runButton = $("btn-run");
function startRun(event) {
  if (event) event.preventDefault();
  if (role !== "controller" || runHeld) return;
  runHeld = true;
  send({ action: "run", held: true });
  runTimer = window.setInterval(() => send({ action: "run", held: true }), 100);
  runButton.classList.add("armed");
}
function stopRun() {
  if (!runHeld) return;
  runHeld = false;
  window.clearInterval(runTimer);
  send({ action: "run", held: false });
  runButton.classList.remove("armed");
}
runButton.addEventListener("pointerdown", startRun);
runButton.addEventListener("pointerup", stopRun);
runButton.addEventListener("pointerleave", stopRun);
runButton.addEventListener("pointercancel", stopRun);
window.addEventListener("blur", stopRun);
document.addEventListener("visibilitychange", () => {
  if (document.hidden) stopRun();
});

function debugRender() {
  $("debug").textContent = JSON.stringify(latest, (k, v) => (k === "data" ? "[grid]" : v), 2);
}
window.setInterval(debugRender, 500);

connect();
if ("serviceWorker" in navigator) {
  navigator.serviceWorker.register("/static/service-worker.js").catch(() => {});
}
