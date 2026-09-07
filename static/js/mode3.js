// mode3.js -- height-map probing: center-hole selection on the shared
// HoleGrid widget, a client-side preview overlay of the 11x11 probe
// points (server recomputes authoritatively at run time), and a simple
// colored <table> heatmap of the results.

const Mode3 = (() => {
  let grid = null;
  const GRID_HALF_EXTENT = 5; // matches hardware.py's GRID_HALF_EXTENT -> 11x11

  function init(tableGrid) {
    grid = new HoleGrid(document.getElementById("m3-grid"), {
      nCols: tableGrid.n_cols,
      nRows: tableGrid.n_rows,
      onSelect: (col, row) => {
        document.getElementById("m3-col").value = col;
        document.getElementById("m3-row").value = row;
        updateOverlay(tableGrid.pitch_mm);
      },
    });
    grid.select(Math.floor(tableGrid.n_cols / 2), Math.floor(tableGrid.n_rows / 2), true);
    document.getElementById("m3-col").value = grid.selected.col;
    document.getElementById("m3-row").value = grid.selected.row;
    updateOverlay(tableGrid.pitch_mm);

    document.getElementById("m3-col").addEventListener("change", () => {
      const col = parseInt(document.getElementById("m3-col").value, 10) || 0;
      const row = parseInt(document.getElementById("m3-row").value, 10) || 0;
      grid.select(col, row, true);
      updateOverlay(tableGrid.pitch_mm);
    });
    document.getElementById("m3-row").addEventListener("change", () => {
      document.getElementById("m3-col").dispatchEvent(new Event("change"));
    });
    document.getElementById("m3-spacing").addEventListener("change", () => updateOverlay(tableGrid.pitch_mm));

    document.getElementById("m3-start-btn").addEventListener("click", start);
    document.getElementById("m3-abort-btn").addEventListener("click", () => App.abort());
    document.getElementById("m3-simulate-collision-btn").addEventListener("click", () => App.simulateCollision());
  }

  function updateOverlay(pitchMm) {
    const spacingMm = parseFloat(document.getElementById("m3-spacing").value) || 5;
    const spacingHoles = spacingMm / pitchMm; // fractional hole-grid units, for display only
    const pts = [];
    for (let i = -GRID_HALF_EXTENT; i <= GRID_HALF_EXTENT; i++) {
      for (let j = -GRID_HALF_EXTENT; j <= GRID_HALF_EXTENT; j++) {
        pts.push({ col: grid.selected.col + i * spacingHoles, row: grid.selected.row + j * spacingHoles });
      }
    }
    grid.setOverlayPoints(pts);
  }

  async function start() {
    const payload = {
      center_hole: [
        parseFloat(document.getElementById("m3-col").value),
        parseFloat(document.getElementById("m3-row").value),
      ],
      spacing_mm: parseFloat(document.getElementById("m3-spacing").value) || 5,
      force_threshold_n: parseFloat(document.getElementById("m3-force-threshold").value) || 5.0,
      sensor_stale_ms: parseFloat(document.getElementById("m3-sensor-stale").value) || 200,
      probe_start_z_offset: parseFloat(document.getElementById("m3-probe-start-z").value) || 50,
      probe_min_z_offset: parseFloat(document.getElementById("m3-probe-min-z").value) || -20,
      probe_vel: parseFloat(document.getElementById("m3-probe-vel").value) || 10,
    };
    const res = await fetch("/api/mode3/start", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (!res.ok) alert(data.error || "start failed");
  }

  function renderStatus(snap) {
    const box = document.getElementById("m3-heatmap");
    if (snap.mode !== "mode3" || !snap.heightmap) {
      return;
    }
    const hm = snap.heightmap;
    const n = hm.n;
    const half = (n - 1) / 2;
    const byIJ = {};
    let minZ = Infinity, maxZ = -Infinity;
    for (const p of hm.points) {
      byIJ[p.i + "," + p.j] = p;
      if (p.z_contact != null) {
        minZ = Math.min(minZ, p.z_contact);
        maxZ = Math.max(maxZ, p.z_contact);
      }
    }
    const range = maxZ > minZ ? maxZ - minZ : 1;

    let html = "<table>";
    for (let j = -half; j <= half; j++) {
      html += "<tr>";
      for (let i = -half; i <= half; i++) {
        const p = byIJ[i + "," + j];
        if (!p || p.z_contact == null) {
          html += `<td style="background:#e5e7eb">&mdash;</td>`;
        } else {
          const frac = (p.z_contact - minZ) / range;
          const color = heatColor(frac);
          html += `<td style="background:${color}" title="i=${i} j=${j}">${p.z_contact.toFixed(1)}</td>`;
        }
      }
      html += "</tr>";
    }
    html += "</table>";
    box.innerHTML = html;
  }

  function heatColor(frac) {
    // simple blue (low) -> red (high) linear scale
    const r = Math.round(255 * frac);
    const b = Math.round(255 * (1 - frac));
    return `rgb(${r},80,${b})`;
  }

  return { init, renderStatus };
})();
