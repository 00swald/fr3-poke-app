// mode1.js -- STL upload, mount-hole selection, preview, execute, and
// rendering of the live results / registration summary for Mode 1.

const Mode1 = (() => {
  let grid = null;
  let spheresData = null;
  let runId = null;

  function init(tableGrid) {
    grid = new HoleGrid(document.getElementById("m1-grid"), {
      nCols: tableGrid.n_cols,
      nRows: tableGrid.n_rows,
      onSelect: (col, row) => {
        document.getElementById("m1-col").value = col;
        document.getElementById("m1-row").value = row;
      },
    });
    grid.select(Math.floor(tableGrid.n_cols / 2), Math.floor(tableGrid.n_rows / 2), true);
    document.getElementById("m1-col").value = grid.selected.col;
    document.getElementById("m1-row").value = grid.selected.row;

    document.getElementById("m1-col").addEventListener("change", syncFromInputs);
    document.getElementById("m1-row").addEventListener("change", syncFromInputs);

    document.getElementById("m1-upload-btn").addEventListener("click", uploadStl);
    document.getElementById("m1-preview-btn").addEventListener("click", preview);
    document.getElementById("m1-execute-btn").addEventListener("click", execute);
    document.getElementById("m1-abort-btn").addEventListener("click", () => App.abort());
    document.getElementById("m1-simulate-collision-btn").addEventListener("click", () => App.simulateCollision());
  }

  function syncFromInputs() {
    const col = parseInt(document.getElementById("m1-col").value, 10) || 0;
    const row = parseInt(document.getElementById("m1-row").value, 10) || 0;
    grid.select(col, row, true);
  }

  function mountHole() {
    return [
      parseFloat(document.getElementById("m1-col").value),
      parseFloat(document.getElementById("m1-row").value),
    ];
  }

  async function uploadStl() {
    const fileInput = document.getElementById("m1-stl-file");
    const box = document.getElementById("m1-upload-result");
    if (!fileInput.files.length) {
      box.textContent = "choose an STL file first";
      return;
    }
    const fd = new FormData();
    fd.append("stl", fileInput.files[0]);
    box.textContent = "processing STL...";
    try {
      const res = await fetch("/api/mode1/upload-stl", { method: "POST", body: fd });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || "upload failed");
      spheresData = data.spheres_data;
      runId = data.run_id;
      renderUploadResult(spheresData);
    } catch (e) {
      box.textContent = "error: " + e.message;
    }
  }

  function renderUploadResult(sd) {
    const box = document.getElementById("m1-upload-result");
    let html = `<p>volume=${sd.volume.toFixed(2)} com=[${sd.com.map((v) => v.toFixed(2)).join(", ")}]
      weld_quality=${sd.weld_quality.toFixed(3)} n_patches=${sd.n_patches}</p>`;
    if (sd.warnings && sd.warnings.length) {
      html += `<p style="color:#d97706">${sd.warnings.join("<br>")}</p>`;
    }
    if (!sd.spheres.length) {
      html += `<p>no spheres detected. rejections: ${JSON.stringify(sd.rejections)}</p>`;
    } else {
      html += "<table><tr><th>index</th><th>radius</th><th>n_faces</th><th>rms_frac</th></tr>";
      for (const s of sd.spheres) {
        html += `<tr><td>${s.index}</td><td>${s.radius.toFixed(3)}</td><td>${s.n_faces}</td><td>${s.rms_residual_frac.toFixed(4)}</td></tr>`;
      }
      html += "</table>";
    }
    box.innerHTML = html;
  }

  async function preview() {
    const box = document.getElementById("m1-preview-result");
    if (!spheresData) {
      box.textContent = "upload an STL first";
      return;
    }
    box.textContent = "computing preview...";
    const payload = {
      run_id: runId,
      spheres_data: spheresData,
      mount_hole: mountHole(),
      mount_yaw_deg: parseFloat(document.getElementById("m1-yaw").value) || 0,
      reach_to: document.getElementById("m1-reach-to").value,
      standoff_mm: parseFloat(document.getElementById("m1-standoff-mm").value) || 30,
    };
    try {
      const res = await fetch("/api/mode1/preview", {
        method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload),
      });
      const data = await res.json();
      if (!res.ok) throw new Error(data.error || "preview failed");
      renderPreview(data);
    } catch (e) {
      box.textContent = "error: " + e.message;
    }
  }

  function fmtPose(p) {
    return "[" + p.map((v) => v.toFixed(2)).join(", ") + "]";
  }

  function renderPreview(data) {
    const box = document.getElementById("m1-preview-result");
    if (data.warning) {
      box.textContent = data.warning;
      return;
    }
    let html = `<p>pre-probe XY: ${fmtPose(data.initial_probe_xy)}</p>`;
    html += "<table><tr><th>index</th><th>radius</th><th>standoff pose</th><th>contact pose</th></tr>";
    for (const t of data.targets) {
      html += `<tr><td>${t.index}</td><td>${t.radius.toFixed(3)}</td><td>${fmtPose(t.standoff_pose)}</td><td>${fmtPose(t.contact_pose)}</td></tr>`;
    }
    html += "</table>";
    box.innerHTML = html;
  }

  async function execute() {
    if (!spheresData) {
      alert("upload an STL first");
      return;
    }
    const payload = {
      run_id: runId,
      spheres_data: spheresData,
      mount_hole: mountHole(),
      mount_yaw_deg: parseFloat(document.getElementById("m1-yaw").value) || 0,
      reach_to: document.getElementById("m1-reach-to").value,
      standoff_mm: parseFloat(document.getElementById("m1-standoff-mm").value) || 30,
      standoff_vel: parseFloat(document.getElementById("m1-standoff-vel").value) || 20,
      approach_vel: parseFloat(document.getElementById("m1-approach-vel").value) || 10,
      force_threshold_n: parseFloat(document.getElementById("m1-force-threshold").value) || 5.0,
      sensor_stale_ms: parseFloat(document.getElementById("m1-sensor-stale").value) || 200,
      probe_start_z_offset: parseFloat(document.getElementById("m1-probe-start-z").value) || 50,
      probe_min_z_offset: parseFloat(document.getElementById("m1-probe-min-z").value) || -20,
      probe_vel: parseFloat(document.getElementById("m1-probe-vel").value) || 10,
      trusted_run: document.getElementById("m1-trusted-run").checked,
    };
    const res = await fetch("/api/mode1/execute", {
      method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (!res.ok) alert(data.error || "execute failed");
  }

  function renderStatus(snap) {
    const resultsBox = document.getElementById("m1-results");
    if (snap.mode !== "mode1" || !snap.records.length) {
      if (snap.mode !== "mode1") resultsBox.innerHTML = "";
      return;
    }
    let html = "<table><tr><th>index</th><th>outcome</th><th>reached</th><th>actual contact</th></tr>";
    for (const r of snap.records) {
      const reachedClass = r.reached ? "reached-true" : "reached-false";
      const actual = r.actual_contact_pose ? fmtPose(r.actual_contact_pose) : (r.z_contact != null ? "z=" + r.z_contact.toFixed(2) : "-");
      html += `<tr><td>${r.index}</td><td>${r.outcome || ""}</td><td class="${reachedClass}">${r.reached}</td><td>${actual}</td></tr>`;
    }
    html += "</table>";
    resultsBox.innerHTML = html;

    const regBox = document.getElementById("m1-registration");
    if (snap.registration_result) {
      const reg = snap.registration_result;
      let rhtml = `<h3>Registration (actual part position vs. nominal STL placement)</h3>`;
      rhtml += `<p>n_points=${reg.n_points} translation=${fmtPose(reg.translation)} mm
        rotation_angle=${reg.rotation_angle_deg != null ? reg.rotation_angle_deg.toFixed(3) : "-"} deg
        residual_rms=${reg.residual_rms_mm != null ? reg.residual_rms_mm.toFixed(3) : "-"} mm
        degenerate=${reg.degenerate}</p>`;
      if (reg.warnings && reg.warnings.length) {
        rhtml += `<p style="color:#d97706">${reg.warnings.join("<br>")}</p>`;
      }
      regBox.innerHTML = rhtml;
    } else {
      regBox.innerHTML = "";
    }
  }

  return { init, renderStatus };
})();
