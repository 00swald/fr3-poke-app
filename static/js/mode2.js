// mode2.js -- drag-teach aim + gamepad release + force-limited poke.
// Live force-vs-time graph consumes the same /api/status poller's
// force_history field that mode1's approaches also feed, via Plotly
// (vendored locally, not the CDN, so it works with no internet on the Pi).

const Mode2 = (() => {
  let graphInitialized = false;

  function init() {
    document.getElementById("m2-start-btn").addEventListener("click", start);
    document.getElementById("m2-stop-btn").addEventListener("click", stop);
    document.getElementById("m2-simulate-collision-btn").addEventListener("click", () => App.simulateCollision());
  }

  async function start() {
    const payload = {
      force_threshold_n: parseFloat(document.getElementById("m2-force-threshold").value) || 5.0,
      search_distance_mm: parseFloat(document.getElementById("m2-search-distance").value) || 200,
      approach_vel: parseFloat(document.getElementById("m2-approach-vel").value) || 20,
      sensor_stale_ms: parseFloat(document.getElementById("m2-sensor-stale").value) || 200,
    };
    const res = await fetch("/api/mode2/start", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (!res.ok) alert(data.error || "start failed");
  }

  async function stop() {
    await fetch("/api/mode2/stop", { method: "POST" });
  }

  function renderStatus(snap) {
    if (snap.mode !== "mode2") return;
    const t = snap.force_history.t;
    const f = snap.force_history.f;
    const trace = { x: t, y: f, mode: "lines", line: { color: "#2563eb", width: 2 } };
    const layout = {
      title: "Live Force Curve (Fz)",
      xaxis: { title: "Time (s)" },
      yaxis: { title: "Force Fz (N)" },
      margin: { t: 40 },
    };
    if (!graphInitialized) {
      Plotly.newPlot("m2-graph", [trace], layout);
      graphInitialized = true;
    } else {
      Plotly.react("m2-graph", [trace], layout);
    }
  }

  return { init, renderStatus };
})();
