(() => {
  const $ = (id) => document.getElementById(id);
  let socket;
  let jogHeartbeat;
  let activeJog = false;
  let currentStatus;
  const operationId = () => (crypto.randomUUID ? crypto.randomUUID() : `${Date.now()}-${Math.random()}`);
  const showMessage = (text, error = false) => {
    const node = $("message");
    node.textContent = text || "";
    node.classList.toggle("error", error);
  };
  const request = async (path, payload = {}, method = "POST") => {
    const body = { ...payload };
    if (method !== "GET" && path !== "/api/v1/settings") body.operationId ||= operationId();
    const response = await fetch(path, {
      method,
      headers: method === "GET" ? {} : { "Content-Type": "application/json" },
      body: method === "GET" ? undefined : JSON.stringify(body),
      cache: "no-store",
    });
    const data = await response.json();
    if (!response.ok || data.ok === false) throw new Error(data.error?.message || `Request failed (${response.status})`);
    return data;
  };
  const run = async (path, payload) => {
    try {
      const data = await request(path, payload);
      showMessage(data.message || "Command accepted.");
      return data;
    } catch (error) {
      showMessage(error.message, true);
      return null;
    }
  };
  const render = (data) => {
    currentStatus = data;
    if (data.position) {
      $("angle").textContent = Number(data.position.estimatedAngle).toFixed(2);
      $("confidence").textContent = data.position.verified
        ? "Software position estimate · not encoder feedback"
        : "Position unverified · set software zero before movement";
    }
    $("motor").textContent = `Motor ${data.motor?.enabled ? "enabled" : "disabled"}`;
    $("motion").textContent = (data.motion || "idle").replaceAll("_", " ");
    const scan = data.scan || {};
    $("scan-state").textContent = scan.state || "idle";
    $("scan-view").textContent = scan.active ? `${scan.viewIndex} / ${scan.viewCount}` : "—";
    $("scan-target").textContent = scan.active ? `${Number(scan.targetAngle).toFixed(2)}°` : "—";
    if (scan.message) $("scan-message").textContent = scan.message;
    $("scan-ack").disabled = !scan.active || !scan.ready;
    $("scan-pause").disabled = !scan.active || scan.state === "paused";
    $("scan-resume").disabled = !scan.active || scan.state !== "paused";
    $("scan-cancel").disabled = !scan.active;
    $("scan-start").disabled = scan.active || data.motion !== "idle";
  };
  const refresh = async () => {
    try { render(await request("/api/v1/status", {}, "GET")); }
    catch (error) { showMessage(error.message, true); }
  };
  const connect = () => {
    const status = $("connection");
    const scheme = location.protocol === "https:" ? "wss" : "ws";
    socket = new WebSocket(`${scheme}://${location.host}/api/v1/events`);
    socket.onopen = () => {
      status.classList.add("online");
      status.lastElementChild.textContent = "Connected";
      refresh();
    };
    socket.onmessage = (event) => {
      try { render(JSON.parse(event.data)); } catch (_) { /* Ignore malformed snapshots. */ }
    };
    socket.onclose = () => {
      status.classList.remove("online");
      status.lastElementChild.textContent = "Reconnecting…";
      setTimeout(connect, 1500);
    };
    socket.onerror = () => socket.close();
  };

  $("speed").addEventListener("input", () => $("speed-label").textContent = `${$("speed").value}°/s`);
  $("scan-mode").addEventListener("change", () => {
    const byViews = $("scan-mode").value === "views";
    $("views-field").classList.toggle("hidden", !byViews);
    $("increment-field").classList.toggle("hidden", byViews);
  });
  document.querySelectorAll("[data-step]").forEach((button) =>
    button.addEventListener("click", () => run("/api/v1/move", {
      mode: "relative", degrees: Number(button.dataset.step), speedDegreesPerSecond: Number($("speed").value),
    })));
  $("relative-go").addEventListener("click", () => run("/api/v1/move", {
    mode: "relative", degrees: Number($("relative").value), speedDegreesPerSecond: Number($("speed").value),
  }));
  $("absolute-go").addEventListener("click", () => run("/api/v1/move", {
    mode: "absolute", degrees: Number($("absolute").value),
    directionPolicy: $("direction-policy").value, speedDegreesPerSecond: Number($("speed").value),
  }));
  const stopJog = () => {
    if (!activeJog) return;
    activeJog = false;
    clearInterval(jogHeartbeat);
    run("/api/v1/jog/stop");
  };
  [["jog-cw", "clockwise"], ["jog-ccw", "counterclockwise"]].forEach(([id, direction]) => {
    const button = $(id);
    button.addEventListener("pointerdown", (event) => {
      event.preventDefault();
      if (activeJog) return;
      activeJog = true;
      button.setPointerCapture(event.pointerId);
      const heartbeat = () => run("/api/v1/jog/start", {
        direction, speedDegreesPerSecond: Number($("speed").value),
      });
      heartbeat();
      jogHeartbeat = setInterval(heartbeat, 500);
    });
    ["pointerup", "pointercancel", "lostpointercapture"].forEach((name) => button.addEventListener(name, stopJog));
  });
  $("continuous-start").addEventListener("click", () => run("/api/v1/continuous/start", {
    direction: $("continuous-direction").value, speedDegreesPerSecond: Number($("speed").value),
  }));
  $("stop").addEventListener("click", () => run("/api/v1/stop"));
  $("zero").addEventListener("click", () => run("/api/v1/position/zero"));
  $("go-zero").addEventListener("click", () => run("/api/v1/position/go-zero"));
  $("motor-enable").addEventListener("click", () => run("/api/v1/motor/enable"));
  $("motor-disable").addEventListener("click", async () => {
    if (confirm("Disable holding torque? The table may move under external force.")) await run("/api/v1/motor/disable");
  });
  $("scan-start").addEventListener("click", () => {
    const payload = {
      mode: $("scan-mode").value, totalDegrees: Number($("scan-angle").value),
      speedDegreesPerSecond: Number($("scan-speed").value),
      autoAdvance: $("auto-advance").checked, returnToStart: $("return-start").checked,
    };
    if (payload.mode === "views") payload.views = Number($("scan-views").value);
    else payload.incrementDegrees = Number($("scan-increment").value);
    run("/api/v1/scan/start", payload);
  });
  $("scan-ack").addEventListener("click", () => run("/api/v1/scan/acknowledge-view"));
  $("scan-pause").addEventListener("click", () => run("/api/v1/scan/pause"));
  $("scan-resume").addEventListener("click", () => run("/api/v1/scan/resume"));
  $("scan-cancel").addEventListener("click", () => {
    if (confirm("Cancel and clear the active scan?")) run("/api/v1/scan/cancel");
  });
  $("settings-save").addEventListener("click", async () => {
    const payload = {
      maxSpeedDegreesPerSecond: Number($("setting-speed").value),
      accelerationDegreesPerSecondSquared: Number($("setting-accel").value),
      settleMs: Number($("setting-settle").value),
      jogLeaseMs: Number($("setting-lease").value),
      defaultScanDegrees: Number($("setting-scan-angle").value),
      defaultScanViews: Number($("setting-scan-views").value),
      autoAdvanceDefault: $("setting-auto-advance").checked,
      returnToStartDefault: $("setting-return-start").checked,
      ssid: $("setting-ssid").value,
    };
    if ($("setting-password").value) payload.password = $("setting-password").value;
    try {
      const result = await request("/api/v1/settings", payload, "PUT");
      showMessage(result.message + (result.restartRequired ? " Reboot required to apply AP credentials." : ""));
      $("setting-password").value = "";
      await loadSettings();
    } catch (error) { showMessage(error.message, true); }
  });
  async function loadSettings() {
    try {
      const s = await request("/api/v1/settings", {}, "GET");
      $("setting-speed").value = s.maxSpeedDegreesPerSecond;
      $("setting-accel").value = s.accelerationDegreesPerSecondSquared;
      $("setting-settle").value = s.settleMs;
      $("setting-lease").value = s.jogLeaseMs;
      $("setting-scan-angle").value = s.defaultScanDegrees;
      $("setting-scan-views").value = s.defaultScanViews;
      $("setting-auto-advance").checked = s.autoAdvanceDefault;
      $("setting-return-start").checked = s.returnToStartDefault;
      $("setting-ssid").value = s.ssid;
      $("scan-angle").value = s.defaultScanDegrees;
      $("scan-views").value = s.defaultScanViews;
      $("auto-advance").checked = s.autoAdvanceDefault;
      $("return-start").checked = s.returnToStartDefault;
    } catch (_) { /* Status reconnect will display connectivity and retry is explicit. */ }
  }
  window.addEventListener("beforeunload", stopJog);
  connect();
  loadSettings();
})();
