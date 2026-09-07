// main.js -- page shell: mode tab switching, settings bar, the single
// shared /api/status poller (used by all 3 modes), and the confirm modal
// that mode1's per-point pause renders through.

const App = (() => {
  let activeMode = "mode1";
  let pollTimer = null;

  function switchMode(mode) {
    activeMode = mode;
    document.querySelectorAll(".tab-btn").forEach((b) => b.classList.toggle("active", b.dataset.mode === mode));
    document.querySelectorAll(".mode-panel").forEach((p) => p.classList.toggle("active", p.id === mode + "-panel"));
  }

  async function loadSettings() {
    const res = await fetch("/api/settings");
    const s = await res.json();
    const box = document.getElementById("settings-summary");
    const robotPill = `<span class="pill ${s.simulate_robot ? "sim" : "real"}">Robot: ${s.simulate_robot ? "SIMULATED" : "REAL (" + s.robot_ip + ")"}</span>`;
    const sensorPill = `<span class="pill ${s.simulate_sensor ? "sim" : "real"}">Sensor: ${s.simulate_sensor ? "SIMULATED" : "REAL (" + s.sensor_port + ")"}</span>`;
    const calibPill = `<span class="pill ${s.table_calib_loaded ? "ok" : "bad"}">Table calib: ${s.table_calib_loaded ? "loaded" : "NOT FOUND"}</span>`;
    box.innerHTML = robotPill + sensorPill + calibPill;
    return s;
  }

  async function loadTableGrid() {
    const res = await fetch("/api/table-grid");
    return await res.json();
  }

  function renderStatusBar(snap) {
    document.getElementById("status-text").textContent = snap.status_text || "idle";
    const dot = document.getElementById("status-running-dot");
    dot.className = "dot " + (snap.error ? "error" : snap.running ? "running" : "idle");
    document.getElementById("status-error").textContent = snap.error ? "error: " + snap.error : "";
  }

  function renderConfirmModal(snap) {
    const modal = document.getElementById("confirm-modal");
    if (!snap.pending_confirm) {
      modal.classList.add("hidden");
      return;
    }
    modal.classList.remove("hidden");
    const c = snap.pending_confirm;
    document.getElementById("confirm-details").textContent =
      `target ${c.target_index} (radius ${c.radius != null ? c.radius.toFixed(3) : "?"} mm) -- at standoff.`;
  }

  async function respondConfirm(response) {
    await fetch("/api/confirm", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ response }),
    });
  }

  async function abort() {
    await fetch("/api/abort", { method: "POST" });
  }

  async function simulateCollision() {
    const res = await fetch("/api/simulate-collision", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify({ force_n: 15.0 }),
    });
    const data = await res.json();
    if (!res.ok) alert(data.error || "simulate-collision failed");
  }

  async function poll() {
    try {
      const res = await fetch("/api/status");
      const snap = await res.json();
      renderStatusBar(snap);
      renderConfirmModal(snap);
      Mode1.renderStatus(snap);
      Mode2.renderStatus(snap);
      Mode3.renderStatus(snap);
      pollTimer = setTimeout(poll, snap.running ? 200 : 1000);
    } catch (e) {
      pollTimer = setTimeout(poll, 2000);
    }
  }

  async function init() {
    document.querySelectorAll(".tab-btn").forEach((b) => {
      b.addEventListener("click", () => switchMode(b.dataset.mode));
    });
    document.getElementById("confirm-continue").addEventListener("click", () => respondConfirm("continue"));
    document.getElementById("confirm-skip").addEventListener("click", () => respondConfirm("skip"));
    document.getElementById("confirm-abort").addEventListener("click", () => respondConfirm("abort"));

    await loadSettings();
    const tableGrid = await loadTableGrid();

    Mode1.init(tableGrid);
    Mode2.init();
    Mode3.init(tableGrid);

    poll();
  }

  return { init, abort, simulateCollision };
})();

window.addEventListener("DOMContentLoaded", App.init);
