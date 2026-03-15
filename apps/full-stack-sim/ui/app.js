const state = {
  schema: null,
  snapshot: null,
  eventSource: null,
  history: [],
  runTimer: null,
  commandInFlight: false,
  map: {
    plane: "xy",
    zoom: 1.0,
    panX: 0,
    panY: 0,
    dragging: false,
    moved: false,
    lastX: 0,
    lastY: 0,
  },
};

const MAX_HISTORY = 120;
const EARTH_RADIUS_M = 6_371_000.0;
const ORIGIN_LAT_DEG = 34.7420;
const ORIGIN_LON_DEG = -120.5724;
const CARD_LAYOUT_KEY = "astraforge-sim-card-layout-v1";
const CARD_VISIBILITY_KEY = "astraforge-sim-card-visibility-v1";
const MIN_CARD_WIDTH = 220;
const MIN_CARD_HEIGHT = 120;

const elements = {
  grid: document.querySelector(".grid"),
  connectionStatus: document.getElementById("connection-status"),
  apiVersion: document.getElementById("api-version"),
  schemaVersion: document.getElementById("schema-version"),
  missionTime: document.getElementById("mission-time"),
  phaseValue: document.getElementById("phase-value"),
  phaseControlValue: document.getElementById("phase-control-value"),
  vehicleModeValue: document.getElementById("vehicle-mode-value"),
  fcModeValue: document.getElementById("fc-mode-value"),
  phaseStrip: document.getElementById("phase-strip"),
  commandPanels: document.getElementById("command-panels"),
  systemsGrid: document.getElementById("systems-grid"),
  missionMap: document.getElementById("mission-map"),
  vehicleGeodetic: document.getElementById("vehicle-geodetic"),
  targetGeodetic: document.getElementById("target-geodetic"),
  mapView: document.getElementById("map-view"),
  truthPosition: document.getElementById("truth-position"),
  truthVelocity: document.getElementById("truth-velocity"),
  gncAttitude: document.getElementById("gnc-attitude"),
  downrangeValue: document.getElementById("downrange-value"),
  trendPositionValue: document.getElementById("trend-position-value"),
  trendVelocityValue: document.getElementById("trend-velocity-value"),
  trendAttitudeValue: document.getElementById("trend-attitude-value"),
  trendPosition: document.getElementById("trend-position"),
  trendVelocity: document.getElementById("trend-velocity"),
  trendAttitude: document.getElementById("trend-attitude"),
  radarAltitude: document.getElementById("radar-altitude"),
  verticalSpeed: document.getElementById("vertical-speed"),
  airspeed: document.getElementById("airspeed"),
  signalStrength: document.getElementById("signal-strength"),
  batterySoc: document.getElementById("battery-soc"),
  batterySocBar: document.getElementById("battery-soc-bar"),
  cpuLoad: document.getElementById("cpu-load"),
  cpuLoadBar: document.getElementById("cpu-load-bar"),
  avionicsTemp: document.getElementById("avionics-temp"),
  avionicsTempBar: document.getElementById("avionics-temp-bar"),
  anomaliesList: document.getElementById("anomalies-list"),
  faultsList: document.getElementById("faults-list"),
  rawTelemetry: document.getElementById("raw-telemetry"),
  focusVehicleButton: document.getElementById("focus-vehicle-button"),
  focusTargetButton: document.getElementById("focus-target-button"),
  resetViewButton: document.getElementById("reset-view-button"),
  propThrottleValue: document.getElementById("prop-throttle-value"),
  propThrustValue: document.getElementById("prop-thrust-value"),
  propFlowValue: document.getElementById("prop-flow-value"),
  propFuelValue: document.getElementById("prop-fuel-value"),
  propThrottleInput: document.getElementById("prop-throttle-input"),
  propThrustInput: document.getElementById("prop-thrust-input"),
  propIspInput: document.getElementById("prop-isp-input"),
  propDryMassInput: document.getElementById("prop-dry-mass-input"),
  propPropellantInput: document.getElementById("prop-propellant-input"),
  propConsumptionInput: document.getElementById("prop-consumption-input"),
  propulsionForm: document.getElementById("propulsion-form"),
  windowsButton: document.getElementById("windows-button"),
  windowsPopup: document.getElementById("windows-popup"),
  windowsCloseButton: document.getElementById("windows-close-button"),
  windowsList: document.getElementById("windows-list"),
  refreshButton: document.getElementById("refresh-button"),
  launchButton: document.getElementById("launch-button"),
  runToggle: document.getElementById("run-toggle"),
  step1: document.getElementById("step-1"),
  step10: document.getElementById("step-10"),
  resetButton: document.getElementById("reset-button"),
  commandTemplate: document.getElementById("select-command-template"),
};

state.layout = {
  initialized: false,
  enabled: false,
  active: null,
  zCounter: 2,
  visibility: {},
};

async function boot() {
  bindTopLevelControls();
  try {
    state.schema = await fetchJson("/schema");
    renderSchema(state.schema);
    state.snapshot = await fetchJson("/snapshot");
    renderSnapshot(state.snapshot);
    window.requestAnimationFrame(() => {
      initCardLayout();
      syncLayoutMode();
    });
    connectEvents();
  } catch (error) {
    setConnection("offline", `Load failed: ${error.message}`);
  }
}

function bindTopLevelControls() {
  window.addEventListener("resize", syncLayoutMode);
  document.addEventListener("click", handleDocumentClick);
  elements.refreshButton.addEventListener("click", async () => {
    state.snapshot = await fetchJson("/snapshot");
    renderSnapshot(state.snapshot);
  });
  elements.launchButton.addEventListener("click", handleLaunch);
  elements.runToggle.addEventListener("click", toggleRunLoop);
  elements.focusVehicleButton.addEventListener("click", () => {
    if (state.snapshot) {
      focusMapOn(state.snapshot.truth.position_m);
      renderMap(state.snapshot);
    }
  });
  elements.focusTargetButton.addEventListener("click", () => {
    if (state.snapshot) {
      focusMapOn(localFromGeodetic(state.snapshot.target));
      renderMap(state.snapshot);
    }
  });
  elements.resetViewButton.addEventListener("click", () => {
    state.map.plane = "xy";
    state.map.zoom = 1.0;
    state.map.panX = 0;
    state.map.panY = 0;
    if (state.snapshot) {
      renderMap(state.snapshot);
    }
  });
  bindMapControls();
  elements.propThrottleInput.addEventListener("input", () => {
    elements.propThrottleValue.textContent = `${elements.propThrottleInput.value}%`;
  });
  elements.propulsionForm.addEventListener("submit", handlePropulsionSubmit);
  elements.step1.addEventListener("click", () => sendCommand({ command: "step", count: 1 }));
  elements.step10.addEventListener("click", () => sendCommand({ command: "step", count: 10 }));
  elements.resetButton.addEventListener("click", async () => {
    stopRunLoop();
    await sendCommand({ command: "reset" });
  });
  elements.windowsButton.addEventListener("click", () => {
    const hidden = elements.windowsPopup.hasAttribute("hidden");
    elements.windowsPopup.toggleAttribute("hidden", !hidden);
  });
  elements.windowsCloseButton.addEventListener("click", () => {
    elements.windowsPopup.setAttribute("hidden", "");
  });
}

function renderSchema(schema) {
  elements.apiVersion.textContent = schema.api_version;
  elements.schemaVersion.textContent = schema.schema_version;
  renderPhaseStrip(schema.phase_values);
  renderCommandPanels(schema.command_catalog);
  if (state.layout.initialized) {
    window.requestAnimationFrame(() => updateGridHeight());
  }
}

function renderPhaseStrip(phases) {
  elements.phaseStrip.innerHTML = "";
  phases.forEach((phase) => {
    const node = document.createElement("div");
    node.className = "phase-node";
    node.dataset.phase = phase;
    node.textContent = phase;
    elements.phaseStrip.appendChild(node);
  });
}

function renderCommandPanels(commands) {
  const filtered = commands.filter(
    (command) =>
      ![
        "step",
        "reset",
        "set_target",
        "configure_propulsion",
      ].includes(command.name)
  );
  elements.commandPanels.innerHTML = "";
  filtered.forEach((command) => {
    const fragment = elements.commandTemplate.content.cloneNode(true);
    const panel = fragment.querySelector(".command-panel");
    panel.querySelector(".command-label").textContent = command.label;
    panel.querySelector(".command-description").textContent = command.description;

    const form = panel.querySelector(".command-form");
    const fieldGrid = panel.querySelector(".field-grid");

    if (command.fields.length === 0) {
      fieldGrid.remove();
    } else {
      command.fields.forEach((field) => {
        fieldGrid.appendChild(buildField(command, field));
      });
    }

    form.addEventListener("submit", async (event) => {
      event.preventDefault();
      const payload = { command: command.name };
      command.fields.forEach((field) => {
        const input = form.elements.namedItem(field.name);
        if (!input) {
          return;
        }
        if (field.type === "u32") {
          payload[field.name] = Number.parseInt(input.value, 10);
        } else {
          payload[field.name] = input.value;
        }
      });
      await sendCommand(payload);
    });

    elements.commandPanels.appendChild(fragment);
  });
}

function buildField(command, field) {
  const label = document.createElement("label");
  label.textContent = field.name.replaceAll("_", " ");

  let input;
  if (field.type === "enum") {
    input = document.createElement("select");
    const values = state.schema[field.enum_ref] || [];
    values.forEach((value) => {
      const option = document.createElement("option");
      option.value = value;
      option.textContent = value;
      input.appendChild(option);
    });
  } else {
    input = document.createElement("input");
    input.type = "number";
    if (field.min !== undefined) {
      input.min = String(field.min);
    }
    input.value = String(field.default ?? 1);
  }

  input.name = field.name;
  input.id = `${command.name}-${field.name}`;
  label.htmlFor = input.id;
  label.appendChild(input);
  return label;
}

function initCardLayout() {
  if (state.layout.initialized) {
    return;
  }
  const savedLayout = loadCardLayout();
  state.layout.visibility = loadCardVisibility();
  const cards = Array.from(elements.grid.querySelectorAll(":scope > .card"));
  const gridRect = elements.grid.getBoundingClientRect();
  cards.forEach((card) => {
    const id = card.dataset.cardId;
    const rect = card.getBoundingClientRect();
    const layout = savedLayout[id] || {
      left: rect.left - gridRect.left,
      top: rect.top - gridRect.top,
      width: rect.width,
      height: rect.height,
    };
    card.dataset.defaultLeft = String(layout.left);
    card.dataset.defaultTop = String(layout.top);
    card.dataset.defaultWidth = String(layout.width);
    card.dataset.defaultHeight = String(layout.height);
    applyCardLayout(card, layout);
    ensureCardControls(card);
    setCardVisibility(card, state.layout.visibility[id] !== false);
  });
  renderWindowsList();
  state.layout.initialized = true;
}

function ensureCardControls(card) {
  if (!card.querySelector(".card-resize-handle")) {
    const handle = document.createElement("div");
    handle.className = "card-resize-handle";
    card.appendChild(handle);
  }
  if (!card.querySelector(".card-head-actions")) {
    const actions = document.createElement("div");
    actions.className = "card-head-actions";
    const closeButton = document.createElement("button");
    closeButton.type = "button";
    closeButton.className = "secondary-button card-close-button";
    closeButton.textContent = "x";
    closeButton.addEventListener("click", () => {
      setCardVisibility(card, false);
      saveCardVisibility();
      renderWindowsList();
      updateGridHeight();
    });
    actions.appendChild(closeButton);
    card.querySelector(".card-head").appendChild(actions);
  }
  const head = card.querySelector(".card-head");
  if (!head.dataset.layoutBound) {
    head.dataset.layoutBound = "true";
    head.addEventListener("pointerdown", (event) => {
      if (!state.layout.enabled || event.button !== 0) {
        return;
      }
      if (event.target.closest("button, input, select, label")) {
        return;
      }
      event.preventDefault();
      beginCardInteraction(event, card, "drag");
    });
  }
  const handle = card.querySelector(".card-resize-handle");
  if (!handle.dataset.layoutBound) {
    handle.dataset.layoutBound = "true";
    handle.addEventListener("pointerdown", (event) => {
      if (!state.layout.enabled || event.button !== 0) {
        return;
      }
      event.preventDefault();
      beginCardInteraction(event, card, "resize");
    });
  }
}

function beginCardInteraction(event, card, mode) {
  const cardRect = card.getBoundingClientRect();
  const gridRect = elements.grid.getBoundingClientRect();
  state.layout.active = {
    card,
    mode,
    startX: event.clientX,
    startY: event.clientY,
    startLeft: cardRect.left - gridRect.left,
    startTop: cardRect.top - gridRect.top,
    startWidth: cardRect.width,
    startHeight: cardRect.height,
  };
  card.classList.add("dragging");
  card.style.zIndex = String(++state.layout.zCounter);
  window.addEventListener("pointermove", onCardInteractionMove);
  window.addEventListener("pointerup", endCardInteraction);
}

function onCardInteractionMove(event) {
  const active = state.layout.active;
  if (!active) {
    return;
  }
  const gridWidth = elements.grid.clientWidth;
  const dx = event.clientX - active.startX;
  const dy = event.clientY - active.startY;
  const layout =
    active.mode === "drag"
      ? {
          left: clamp(active.startLeft + dx, 0, Math.max(0, gridWidth - active.startWidth)),
          top: Math.max(0, active.startTop + dy),
          width: active.startWidth,
          height: active.startHeight,
        }
      : {
          left: active.startLeft,
          top: active.startTop,
          width: clamp(active.startWidth + dx, MIN_CARD_WIDTH, Math.max(MIN_CARD_WIDTH, gridWidth - active.startLeft)),
          height: Math.max(MIN_CARD_HEIGHT, active.startHeight + dy),
        };
  applyCardLayout(active.card, layout);
  updateGridHeight();
}

function endCardInteraction() {
  if (!state.layout.active) {
    return;
  }
  state.layout.active.card.classList.remove("dragging");
  saveCardLayout();
  state.layout.active = null;
  window.removeEventListener("pointermove", onCardInteractionMove);
  window.removeEventListener("pointerup", endCardInteraction);
}

function applyCardLayout(card, layout) {
  card.style.left = `${layout.left}px`;
  card.style.top = `${layout.top}px`;
  card.style.width = `${layout.width}px`;
  card.style.height = `${layout.height}px`;
}

function updateGridHeight() {
  if (!state.layout.enabled) {
    elements.grid.style.height = "";
    return;
  }
  const cards = Array.from(elements.grid.querySelectorAll(":scope > .card"));
  const maxBottom = cards.reduce((bottom, card) => {
    if (card.classList.contains("hidden-card")) {
      return bottom;
    }
    const top = Number.parseFloat(card.style.top || "0");
    const height = Number.parseFloat(card.style.height || String(card.offsetHeight));
    return Math.max(bottom, top + height);
  }, 0);
  elements.grid.style.height = `${Math.ceil(maxBottom + 12)}px`;
}

function syncLayoutMode() {
  if (!state.layout.initialized) {
    return;
  }
  const enable = window.innerWidth > 960;
  state.layout.enabled = enable;
  elements.grid.classList.toggle("free-layout", enable);
  const cards = Array.from(elements.grid.querySelectorAll(":scope > .card"));
  cards.forEach((card) => {
    if (enable) {
      if (!card.style.left && !card.classList.contains("hidden-card")) {
        applyCardLayout(card, defaultCardLayout(card));
      }
    } else {
      card.style.left = "";
      card.style.top = "";
      card.style.width = "";
      card.style.height = "";
      card.style.zIndex = "";
    }
  });
  renderWindowsList();
  updateGridHeight();
}

function defaultCardLayout(card) {
  return {
    left: Number.parseFloat(card.dataset.defaultLeft || "0"),
    top: Number.parseFloat(card.dataset.defaultTop || "0"),
    width: Number.parseFloat(card.dataset.defaultWidth || String(card.offsetWidth)),
    height: Number.parseFloat(card.dataset.defaultHeight || String(card.offsetHeight)),
  };
}

function saveCardLayout() {
  if (!state.layout.enabled) {
    return;
  }
  const layout = {};
  elements.grid.querySelectorAll(":scope > .card").forEach((card) => {
    if (card.classList.contains("hidden-card")) {
      return;
    }
    layout[card.dataset.cardId] = {
      left: Number.parseFloat(card.style.left || "0"),
      top: Number.parseFloat(card.style.top || "0"),
      width: Number.parseFloat(card.style.width || String(card.offsetWidth)),
      height: Number.parseFloat(card.style.height || String(card.offsetHeight)),
    };
  });
  window.localStorage.setItem(CARD_LAYOUT_KEY, JSON.stringify(layout));
}

function loadCardLayout() {
  try {
    const raw = window.localStorage.getItem(CARD_LAYOUT_KEY);
    return raw ? JSON.parse(raw) : {};
  } catch (_) {
    return {};
  }
}

function renderWindowsList() {
  const cards = Array.from(elements.grid.querySelectorAll(":scope > .card"));
  elements.windowsList.innerHTML = "";
  cards.forEach((card) => {
    const row = document.createElement("label");
    row.className = "window-toggle";
    const name = document.createElement("span");
    name.textContent = card.querySelector(".card-head h2").textContent;
    const toggle = document.createElement("input");
    toggle.type = "checkbox";
    toggle.checked = !card.classList.contains("hidden-card");
    toggle.addEventListener("change", () => {
      setCardVisibility(card, toggle.checked);
      saveCardVisibility();
      updateGridHeight();
    });
    row.appendChild(name);
    row.appendChild(toggle);
    elements.windowsList.appendChild(row);
  });
}

function setCardVisibility(card, visible) {
  card.classList.toggle("hidden-card", !visible);
  state.layout.visibility[card.dataset.cardId] = visible;
}

function saveCardVisibility() {
  window.localStorage.setItem(CARD_VISIBILITY_KEY, JSON.stringify(state.layout.visibility));
}

function loadCardVisibility() {
  try {
    const raw = window.localStorage.getItem(CARD_VISIBILITY_KEY);
    return raw ? JSON.parse(raw) : {};
  } catch (_) {
    return {};
  }
}

function handleDocumentClick(event) {
  if (elements.windowsPopup.hasAttribute("hidden")) {
    return;
  }
  if (
    event.target.closest("#windows-popup") ||
    event.target.closest("#windows-button")
  ) {
    return;
  }
  elements.windowsPopup.setAttribute("hidden", "");
}

async function sendCommand(payload) {
  if (state.commandInFlight) {
    return state.snapshot;
  }
  try {
    state.commandInFlight = true;
    setConnection("busy", "Sending command");
    state.snapshot = await fetchJson("/command", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    renderSnapshot(state.snapshot);
    setConnection("online", "Live");
  } catch (error) {
    setConnection("offline", `Command failed: ${error.message}`);
    throw error;
  } finally {
    state.commandInFlight = false;
  }
}

async function handleLaunch() {
  stopRunLoop();
  await sendCommand({
    command: "set_phase_control",
    phase_control: "manual",
  });
  await sendCommand({
    command: "set_phase",
    phase: "launch",
  });
}

async function handleMapClick(event) {
  if (!state.snapshot) {
    return;
  }
  if (!event.shiftKey) {
    cycleMapPlane();
    renderMap(state.snapshot);
    return;
  }
  const geodetic = mapClickToTarget(event);
  if (!geodetic) {
    return;
  }
  const confirmed = window.confirm(
    `Reassign target to ${geodetic.latitude_deg.toFixed(4)}, ${geodetic.longitude_deg.toFixed(4)}, altitude ${formatNumber(
      geodetic.altitude_m ?? 0
    )} m?`
  );
  if (!confirmed) {
    return;
  }
  await sendCommand({
    command: "set_target",
    latitude_deg: Number(geodetic.latitude_deg.toFixed(5)),
    longitude_deg: Number(geodetic.longitude_deg.toFixed(5)),
    altitude_m: Number((geodetic.altitude_m ?? 0).toFixed(1)),
  });
}

async function handlePropulsionSubmit(event) {
  event.preventDefault();
  const throttlePercent = Number.parseInt(elements.propThrottleInput.value, 10);
  const maxThrustKn = Number.parseFloat(elements.propThrustInput.value);
  const ispS = Number.parseFloat(elements.propIspInput.value);
  const dryMassKg = Number.parseFloat(elements.propDryMassInput.value);
  const propellantMassKg = Number.parseFloat(elements.propPropellantInput.value);
  const consumptionScale = Number.parseFloat(elements.propConsumptionInput.value);

  await sendCommand({
    command: "configure_propulsion",
    throttle_percent: throttlePercent,
    max_thrust_kn: maxThrustKn,
    isp_s: ispS,
    dry_mass_kg: dryMassKg,
    propellant_mass_kg: propellantMassKg,
    consumption_scale: consumptionScale,
  });
}

function toggleRunLoop() {
  if (state.runTimer) {
    stopRunLoop();
    return;
  }
  state.runTimer = window.setInterval(() => {
    void sendCommand({ command: "step", count: 5 });
  }, 350);
  elements.runToggle.textContent = "Pause";
  elements.runToggle.classList.add("running");
}

function stopRunLoop() {
  if (state.runTimer) {
    window.clearInterval(state.runTimer);
    state.runTimer = null;
  }
  elements.runToggle.textContent = "Run";
  elements.runToggle.classList.remove("running");
}

function connectEvents() {
  const eventSource = new EventSource("/events");
  state.eventSource = eventSource;
  setConnection("online", "Live");

  eventSource.addEventListener("snapshot", (event) => {
    try {
      const snapshot = JSON.parse(event.data);
      state.snapshot = snapshot;
      renderSnapshot(snapshot);
      setConnection("online", "Streaming");
    } catch (error) {
      setConnection("offline", `Event parse failed: ${error.message}`);
    }
  });

  eventSource.onerror = () => {
    setConnection("offline", "Reconnect pending");
  };
}

function renderSnapshot(snapshot) {
  pushHistory(snapshot);
  elements.apiVersion.textContent = snapshot.api_version;
  elements.schemaVersion.textContent = snapshot.schema_version;
  elements.missionTime.textContent = `${snapshot.mission_time_ms} ms`;
  elements.phaseValue.textContent = snapshot.phase;
  elements.phaseControlValue.textContent = snapshot.phase_control;
  elements.vehicleModeValue.textContent = snapshot.telemetry.vehicle.mode;
  elements.fcModeValue.textContent = snapshot.flight_computer.mode;
  elements.vehicleGeodetic.textContent = formatGeodetic(snapshot.geodetic);
  elements.targetGeodetic.textContent = formatGeodetic(snapshot.target);

  document.querySelectorAll(".phase-node").forEach((node) => {
    node.classList.toggle("active", node.dataset.phase === snapshot.phase);
  });

  elements.truthPosition.textContent = formatVec3(snapshot.truth.position_m, "m");
  elements.truthVelocity.textContent = formatVec3(snapshot.truth.velocity_mps, "m/s");
  elements.gncAttitude.textContent = formatVec3(
    snapshot.systems.gnc_solution.attitude_rad,
    "rad"
  );
  elements.downrangeValue.textContent = `${formatNumber(snapshot.sensors.downrange_m)} m`;

  elements.radarAltitude.textContent = `${formatNumber(snapshot.sensors.radar_altitude_m)} m`;
  elements.verticalSpeed.textContent = `${formatNumber(snapshot.sensors.vertical_speed_mps)} m/s`;
  elements.airspeed.textContent = `${formatNumber(snapshot.sensors.airspeed_mps)} m/s`;
  elements.signalStrength.textContent = `${formatNumber(snapshot.sensors.signal_strength_dbm)} dBm`;

  const batteryPercent = snapshot.telemetry.power.battery_soc * 100;
  elements.batterySoc.textContent = `${formatNumber(batteryPercent)}%`;
  elements.batterySocBar.style.width = `${clamp(batteryPercent, 0, 100)}%`;

  const cpuPercent = snapshot.flight_computer.cpu_load * 100;
  elements.cpuLoad.textContent = `${formatNumber(cpuPercent)}%`;
  elements.cpuLoadBar.style.width = `${clamp(cpuPercent, 0, 100)}%`;

  const tempPercent = clamp((snapshot.flight_computer.avionics_temp_c / 100) * 100, 0, 100);
  elements.avionicsTemp.textContent = `${formatNumber(snapshot.flight_computer.avionics_temp_c)} C`;
  elements.avionicsTempBar.style.width = `${tempPercent}%`;
  elements.propThrottleValue.textContent = `${snapshot.propulsion.throttle_percent}%`;
  elements.propThrustValue.textContent = `${formatNumber(snapshot.propulsion.current_thrust_kn)} kN`;
  elements.propFlowValue.textContent = `${formatNumber(snapshot.propulsion.mass_flow_kgps)} kg/s`;
  elements.propFuelValue.textContent = `${formatNumber(snapshot.propulsion.propellant_mass_kg)} kg`;
  syncPropulsionInputs(snapshot.propulsion);
  if (
    state.runTimer &&
    (snapshot.phase === "impact" ||
      (snapshot.phase !== "pad" && snapshot.sensors.radar_altitude_m <= 0))
  ) {
    stopRunLoop();
  }

  renderSystems(snapshot);
  renderMap(snapshot);
  renderTrends(snapshot);
  renderAnomalies(snapshot.anomalies);
  renderFaults(snapshot.last_fault_responses);
  renderRawTelemetry(snapshot.telemetry.raw);
}

function pushHistory(snapshot) {
  const latest = state.history[state.history.length - 1];
  if (latest && latest.missionTimeMs === snapshot.mission_time_ms) {
    state.history[state.history.length - 1] = historySampleFromSnapshot(snapshot);
    return;
  }
  state.history.push(historySampleFromSnapshot(snapshot));
  if (state.history.length > MAX_HISTORY) {
    state.history.shift();
  }
}

function historySampleFromSnapshot(snapshot) {
  return {
    missionTimeMs: snapshot.mission_time_ms,
    latitude: snapshot.geodetic.latitude_deg,
    longitude: snapshot.geodetic.longitude_deg,
    truthPosition: [...snapshot.truth.position_m],
    positionError: vectorDifference(snapshot.truth.position_m, snapshot.systems.gnc_solution.position_m),
    velocityError: vectorDifference(snapshot.truth.velocity_mps, snapshot.systems.gnc_solution.velocity_mps),
    attitudeError: vectorDifference(snapshot.truth.attitude_rad, snapshot.systems.gnc_solution.attitude_rad),
  };
}

function renderSystems(snapshot) {
  const systems = [
    {
      name: "Vehicle",
      mode: snapshot.telemetry.vehicle.mode,
      health: "nominal",
      detail: `phase ${snapshot.phase}`,
    },
    {
      name: "Power",
      mode: snapshot.telemetry.power.mode,
      health: snapshot.telemetry.power.health,
      detail: `${formatNumber(snapshot.telemetry.power.bus_voltage_v)} V`,
    },
    {
      name: "Thermal",
      mode: snapshot.telemetry.thermal.mode,
      health: snapshot.telemetry.thermal.health,
      detail: `${formatNumber(snapshot.flight_computer.avionics_temp_c)} C avionics`,
    },
    {
      name: "Payload",
      mode: snapshot.telemetry.payload.mode,
      health: snapshot.telemetry.payload.health,
      detail: `${formatNumber(snapshot.telemetry.payload.current_a)} A`,
    },
    {
      name: "Comms",
      mode: snapshot.telemetry.communications.link_status,
      health: snapshot.telemetry.communications.health,
      detail: `${formatNumber(snapshot.telemetry.communications.signal_strength_dbm)} dBm`,
    },
    {
      name: "GNC",
      mode: "tracking",
      health: "nominal",
      detail: `${formatNumber(snapshot.systems.gnc_solution.position_m[2])} m alt`,
    },
  ];

  elements.systemsGrid.innerHTML = "";
  systems.forEach((system) => {
    const tile = document.createElement("div");
    tile.className = "system-tile";
    tile.innerHTML = `
      <span class="label">${system.name}</span>
      <strong>${system.mode}</strong>
      <span class="status-chip ${system.health}">${system.health}</span>
      <div class="muted" style="margin-top:10px">${system.detail}</div>
    `;
    elements.systemsGrid.appendChild(tile);
  });
}

function renderMap(snapshot) {
  drawProjectedMap(snapshot);
  elements.mapView.textContent = `${state.map.plane} zoom ${state.map.zoom.toFixed(2)}`;
}

function renderTrends(snapshot) {
  elements.trendPositionValue.textContent = formatAxisVector(
    vectorDifference(snapshot.truth.position_m, snapshot.systems.gnc_solution.position_m),
    "d",
    "m"
  );
  elements.trendVelocityValue.textContent = formatAxisVector(
    vectorDifference(snapshot.truth.velocity_mps, snapshot.systems.gnc_solution.velocity_mps),
    "d",
    "m/s"
  );
  elements.trendAttitudeValue.textContent = formatAxisVector(
    vectorDifference(snapshot.truth.attitude_rad, snapshot.systems.gnc_solution.attitude_rad),
    ["dr", "dp", "dy"],
    "rad"
  );

  drawVectorTrend(elements.trendPosition, state.history.map((point) => point.positionError));
  drawVectorTrend(elements.trendVelocity, state.history.map((point) => point.velocityError));
  drawVectorTrend(elements.trendAttitude, state.history.map((point) => point.attitudeError));
}

function drawVectorTrend(svg, vectors) {
  const width = 320;
  const height = 120;
  const padX = 10;
  const padY = 12;
  const seriesVectors = vectors.length ? vectors : [[0, 0, 0]];
  const pointCount = Math.max(seriesVectors.length, 1);
  const axes = [0, 1, 2];
  const series = axes.flatMap((axis) => seriesVectors.map((values) => values[axis]));
  const min = Math.min(...series);
  const max = Math.max(...series);
  const span = max - min || 1;
  const baselineValue = min <= 0 && max >= 0 ? 0 : min;
  const baselineY = height - padY - ((baselineValue - min) / span) * (height - padY * 2);
  const axisNames = ["axis-x", "axis-y", "axis-z"];
  const paths = axes.map((axis) => {
    const path = pointsToPath(seriesToPoints(seriesVectors, axis, width, height, padX, padY, min, span, pointCount));
    return `<path class="trend-line ${axisNames[axis]}" d="${path}"></path>`;
  });

  svg.innerHTML = `
    <line class="trend-baseline" x1="${padX}" y1="${baselineY}" x2="${width - padX}" y2="${baselineY}"></line>
    ${paths.join("")}
  `;
}

function seriesToPoints(vectors, axis, width, height, padX, padY, min, span, pointCount) {
  return vectors.map((values, index) => {
    const x = padX + (index / Math.max(pointCount - 1, 1)) * (width - padX * 2);
    const y = height - padY - ((values[axis] - min) / span) * (height - padY * 2);
    return [x, y];
  });
}

function pointsToPath(points) {
  if (!points.length) {
    return "M 0 0";
  }
  return points
    .map((point, index) => {
      const x = Array.isArray(point) ? point[0] : point.x;
      const y = Array.isArray(point) ? point[1] : point.y;
      return `${index === 0 ? "M" : "L"} ${x} ${y}`;
    })
    .join(" ");
}

function bindMapControls() {
  const canvas = elements.missionMap;
  canvas.addEventListener("mousedown", (event) => {
    state.map.dragging = true;
    state.map.moved = false;
    state.map.lastX = event.clientX;
    state.map.lastY = event.clientY;
    canvas.classList.add("dragging");
  });
  window.addEventListener("mousemove", (event) => {
    if (!state.map.dragging) {
      return;
    }
    const dx = event.clientX - state.map.lastX;
    const dy = event.clientY - state.map.lastY;
    state.map.lastX = event.clientX;
    state.map.lastY = event.clientY;
    if (Math.abs(dx) > 1 || Math.abs(dy) > 1) {
      state.map.moved = true;
    }
    state.map.panX += dx;
    state.map.panY += dy;
    if (state.snapshot) {
      renderMap(state.snapshot);
    }
  });
  window.addEventListener("mouseup", async (event) => {
    const wasDragging = state.map.dragging;
    const moved = state.map.moved;
    state.map.dragging = false;
    canvas.classList.remove("dragging");
    if (wasDragging && !moved) {
      await handleMapClick(event);
    }
  });
  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    state.map.zoom = clamp(state.map.zoom - event.deltaY * 0.0025, 0.35, 10.0);
    if (state.snapshot) {
      renderMap(state.snapshot);
    }
  });
}

function drawProjectedMap(snapshot) {
  const canvas = elements.missionMap;
  const ctx = canvas.getContext("2d");
  const dpr = window.devicePixelRatio || 1;
  const width = canvas.clientWidth || 720;
  const height = canvas.clientHeight || 250;
  if (canvas.width !== Math.round(width * dpr) || canvas.height !== Math.round(height * dpr)) {
    canvas.width = Math.round(width * dpr);
    canvas.height = Math.round(height * dpr);
  }
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  ctx.clearRect(0, 0, width, height);
  const centerX = width * 0.5 + state.map.panX;
  const centerY = height * 0.5 + state.map.panY;
  const targetLocal = localFromGeodetic(snapshot.target);
  const trailPoints = state.history.map((point) => point.truthPosition);
  const extents = buildMapExtents(trailPoints, snapshot.truth.position_m, targetLocal);
  const scale = computeMapScale(extents, width, height);

  drawMapGrid(ctx, width, height, centerX, centerY, scale);
  drawMapAxes(ctx, width, height, centerX, centerY);
  drawMapTrail(ctx, trailPoints, centerX, centerY, scale);
  drawMapMarker(ctx, snapshot.truth.position_m, centerX, centerY, scale, "#5dc2ff", "Vehicle");
  drawMapMarker(ctx, targetLocal, centerX, centerY, scale, "#ffb84d", "Target");
}

function drawMapGrid(ctx, width, height, centerX, centerY, scale) {
  ctx.strokeStyle = "rgba(145, 173, 182, 0.12)";
  ctx.lineWidth = 1;
  const spacing = 50;
  for (let x = centerX % spacing; x <= width; x += spacing) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = centerY % spacing; y <= height; y += spacing) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  ctx.fillStyle = "rgba(145, 173, 182, 0.72)";
  ctx.font = "12px Segoe UI";
  const [horizontalLabel, verticalLabel] = planeLabels(state.map.plane);
  ctx.fillText(`${horizontalLabel} / ${verticalLabel}`, 14, 18);
  ctx.fillText(`${formatNumber(50 / scale)} m per grid`, 14, 34);
}

function drawMapAxes(ctx, width, height, centerX, centerY) {
  ctx.strokeStyle = "rgba(255, 255, 255, 0.18)";
  ctx.lineWidth = 1.2;
  ctx.beginPath();
  ctx.moveTo(0, centerY);
  ctx.lineTo(width, centerY);
  ctx.moveTo(centerX, 0);
  ctx.lineTo(centerX, height);
  ctx.stroke();
}

function drawMapTrail(ctx, positions, centerX, centerY, scale) {
  const projected = positions.map((position) => projectLocalToCanvas(position, centerX, centerY, scale));
  if (projected.length < 2) {
    return;
  }
  ctx.strokeStyle = "#5dc2ff";
  ctx.lineWidth = 2.2;
  ctx.beginPath();
  projected.forEach((point, index) => {
    if (index === 0) {
      ctx.moveTo(point.x, point.y);
    } else {
      ctx.lineTo(point.x, point.y);
    }
  });
  ctx.stroke();
}

function drawMapMarker(ctx, position, centerX, centerY, scale, color, label) {
  const point = projectLocalToCanvas(position, centerX, centerY, scale);
  ctx.strokeStyle = "rgba(255,255,255,0.85)";
  ctx.fillStyle = color;
  ctx.lineWidth = 2;
  ctx.beginPath();
  ctx.arc(point.x, point.y, 5.5, 0, Math.PI * 2);
  ctx.fill();
  ctx.stroke();
  ctx.fillStyle = "#e6f1f3";
  ctx.font = "12px Segoe UI";
  ctx.fillText(label, point.x + 9, point.y - 8);
}

function mapClickToTarget(event) {
  const canvas = elements.missionMap;
  const rect = canvas.getBoundingClientRect();
  const width = rect.width;
  const height = rect.height;
  const centerX = width * 0.5 + state.map.panX;
  const centerY = height * 0.5 + state.map.panY;
  const targetLocal = localFromGeodetic(state.snapshot.target);
  const extents = buildMapExtents(
    state.history.map((point) => point.truthPosition),
    state.snapshot.truth.position_m,
    targetLocal
  );
  const scale = computeMapScale(extents, width, height);
  const mapX = (event.clientX - rect.left - centerX) / scale;
  const mapY = -(event.clientY - rect.top - centerY) / scale;
  const currentTarget = targetLocal;
  let local;
  switch (state.map.plane) {
    case "xz":
      local = [mapX, currentTarget[1], mapY];
      break;
    case "yz":
      local = [currentTarget[0], mapX, mapY];
      break;
    case "xy":
    default:
      local = [mapX, mapY, currentTarget[2]];
      break;
  }
  return geodeticFromLocal(local);
}

function focusMapOn(position) {
  const projected = projectPlane(position);
  state.map.panX = -projected[0] * state.map.zoom;
  state.map.panY = projected[1] * state.map.zoom;
  state.map.zoom = Math.max(state.map.zoom, 1.6);
}

function cycleMapPlane() {
  const next = { xy: "xz", xz: "yz", yz: "xy" };
  state.map.plane = next[state.map.plane] || "xy";
}

function planeLabels(plane) {
  switch (plane) {
    case "xz":
      return ["X", "Z"];
    case "yz":
      return ["Y", "Z"];
    case "xy":
    default:
      return ["X", "Y"];
  }
}

function projectPlane(position) {
  switch (state.map.plane) {
    case "xz":
      return [position[0], position[2]];
    case "yz":
      return [position[1], position[2]];
    case "xy":
    default:
      return [position[0], position[1]];
  }
}

function projectLocalToCanvas(position, centerX, centerY, scale) {
  const [horizontal, vertical] = projectPlane(position);
  return {
    x: centerX + horizontal * scale,
    y: centerY - vertical * scale,
  };
}

function buildMapExtents(positions, vehiclePosition, targetPosition) {
  const projected = positions
    .concat([vehiclePosition, targetPosition])
    .map((position) => projectPlane(position));
  const xs = projected.map((point) => point[0]);
  const ys = projected.map((point) => point[1]);
  const minX = Math.min(...xs, 0);
  const maxX = Math.max(...xs, 0);
  const minY = Math.min(...ys, 0);
  const maxY = Math.max(...ys, 0);
  return {
    spanX: Math.max(maxX - minX, 400),
    spanY: Math.max(maxY - minY, 400),
  };
}

function computeMapScale(extents, width, height) {
  const availableWidth = Math.max(width - 60, 120);
  const availableHeight = Math.max(height - 60, 120);
  const scaleX = availableWidth / extents.spanX;
  const scaleY = availableHeight / extents.spanY;
  return Math.min(scaleX, scaleY) * state.map.zoom;
}

function localFromGeodetic(position) {
  const northM = radians(position.latitude_deg - ORIGIN_LAT_DEG) * EARTH_RADIUS_M;
  const eastM =
    radians(position.longitude_deg - ORIGIN_LON_DEG) *
    EARTH_RADIUS_M *
    Math.cos(radians(ORIGIN_LAT_DEG));
  return [eastM, northM, position.altitude_m ?? 0];
}

function geodeticFromLocal(position) {
  const latitudeDeg = ORIGIN_LAT_DEG + degrees(position[1] / EARTH_RADIUS_M);
  const longitudeDeg =
    ORIGIN_LON_DEG +
    degrees(position[0] / (EARTH_RADIUS_M * Math.cos(radians(ORIGIN_LAT_DEG))));
  return {
    latitude_deg: latitudeDeg,
    longitude_deg: longitudeDeg,
    altitude_m: Math.max(position[2], 0),
  };
}

function renderAnomalies(anomalies) {
  elements.anomaliesList.innerHTML = "";
  if (!anomalies.length) {
    elements.anomaliesList.innerHTML = '<span class="tag">nominal</span>';
    return;
  }
  anomalies.forEach((anomaly) => {
    const tag = document.createElement("span");
    tag.className = "tag";
    tag.textContent = anomaly;
    elements.anomaliesList.appendChild(tag);
  });
}

function renderFaults(faults) {
  elements.faultsList.innerHTML = "";
  if (!faults.length) {
    elements.faultsList.innerHTML = '<span class="tag">no active fault responses</span>';
    return;
  }
  faults.forEach((fault) => {
    const pill = document.createElement("span");
    pill.className = "fault-pill";
    pill.textContent = `${fault.component}: ${fault.response}`;
    elements.faultsList.appendChild(pill);
  });
}

function renderRawTelemetry(raw) {
  elements.rawTelemetry.textContent = [
    raw.vehicle_mode_line,
    raw.power_status_line,
    raw.thermal_status_line,
    raw.payload_status_line,
    raw.comm_status_line,
    raw.gnc_solution_line,
    raw.latest_downlink_frame,
  ].join("\n");
}

function setConnection(kind, text) {
  elements.connectionStatus.textContent = text;
  elements.connectionStatus.style.color =
    kind === "online" ? "var(--accent)" :
    kind === "busy" ? "var(--warn)" :
    "var(--danger)";
}

function syncPropulsionInputs(propulsion) {
  if (document.activeElement !== elements.propThrottleInput) {
    elements.propThrottleInput.value = propulsion.throttle_percent;
  }
  if (document.activeElement !== elements.propThrustInput) {
    elements.propThrustInput.value = formatFixed(propulsion.max_thrust_kn);
  }
  if (document.activeElement !== elements.propIspInput) {
    elements.propIspInput.value = formatFixed(propulsion.isp_s);
  }
  if (document.activeElement !== elements.propDryMassInput) {
    elements.propDryMassInput.value = formatFixed(propulsion.dry_mass_kg);
  }
  if (document.activeElement !== elements.propPropellantInput) {
    elements.propPropellantInput.value = formatFixed(propulsion.propellant_mass_kg);
  }
  if (document.activeElement !== elements.propConsumptionInput) {
    elements.propConsumptionInput.value = formatFixed(propulsion.consumption_scale, 2);
  }
}

async function fetchJson(url, options = {}) {
  const response = await fetch(url, options);
  if (!response.ok) {
    throw new Error(`${response.status} ${response.statusText}`);
  }
  return response.json();
}

function formatVec3(values, unit) {
  return `${formatNumber(values[0])}, ${formatNumber(values[1])}, ${formatNumber(values[2])} ${unit}`;
}

function formatNumber(value) {
  return Number(value).toFixed(1).replace(/\.0$/, "");
}

function formatFixed(value, digits = 1) {
  return Number(value).toFixed(digits);
}

function formatGeodetic(position) {
  return `${position.latitude_deg.toFixed(4)}, ${position.longitude_deg.toFixed(4)}, ${formatNumber(position.altitude_m)} m`;
}

function formatAxisVector(values, prefix, unit) {
  const labels = Array.isArray(prefix) ? prefix : [`${prefix}x`, `${prefix}y`, `${prefix}z`];
  return `${labels[0]} ${formatNumber(values[0])} ${labels[1]} ${formatNumber(values[1])} ${labels[2]} ${formatNumber(values[2])} ${unit}`;
}

function vectorDifference(lhs, rhs) {
  return [lhs[0] - rhs[0], lhs[1] - rhs[1], lhs[2] - rhs[2]];
}

function radians(value) {
  return (value * Math.PI) / 180;
}

function degrees(value) {
  return (value * 180) / Math.PI;
}

function clamp(value, min, max) {
  return Math.min(Math.max(value, min), max);
}

boot();
