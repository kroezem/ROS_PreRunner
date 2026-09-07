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
    } else if (frame.type === "map" || frame.type === "plan") {
      latest[frame.type] = frame;
    } else if (frame.type === "ack") {
      if (typeof frame.role === "string") role = frame.role;
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
  renderCatalog(mapState.catalog || [], mapState.selected_map_applied);

  text("a-map", mode.active_autonomy_map || "(none)");
  text("a-mission", `${MISSION[nav.state] ?? "—"}${nav.mission_valid ? " · mission valid" : ""} · rev ${nav.mission_revision ?? 0}`);
  text("a-detail", nav.detail || nav.error_meaning || "—");
  text("a-active", auth.autonomy_action_active ? "yes (Nav2 executing)" : "no");
  $("pose-hint").textContent = pose
    ? `Robot pose: x=${fmt(pose.position.x)} y=${fmt(pose.position.y)}`
    : "Robot pose: — (no map->base_link TF)";

  const controller = role === "controller";
  document.querySelectorAll("button").forEach((b) => {
    if (b.id === "btn-stop") return;               // STOP stays enabled when possible
    b.disabled = !controller;
  });
  $("btn-stop").disabled = !controller;
  $("btn-run").classList.toggle("armed", runHeld);
}

function setBannerFromState(mode, auth, stop) {
  if (stop.stopped) setBanner("STOP ASSERTED — global inhibit latched", "stop");
  else if (auth.dualsense_active) setBanner("DUALSENSE TAKEOVER — local manual control", "takeover");
  else if (auth.authority === 3 && auth.autonomy_action_active) setBanner("AUTONOMOUS — Nav2 executing under RUN", "run");
  else setBanner(`Connected · ${role}`, "connected");
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
