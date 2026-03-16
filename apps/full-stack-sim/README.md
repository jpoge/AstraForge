# Full-Stack Sim

`full-stack-sim` is a standalone host simulation for AstraForge. It exercises the existing single-string flight software stack across mission phases and keeps the control surface library-first so the same engine can back a future web application.

The sim composes:

- `simple-gnc` for navigation propagation from IMU, GPS, and magnetometer data
- `communications-template` for uplink/downlink and link-health behavior
- `vehicle-management-template` for mission-mode/state-machine logic
- `power-management-template` for EPS-style health and load-shed behavior
- `thermal-management-template` for thermal zone evaluation
- `payload-interface-template` for payload command/fault handling

The simulated mission phases are:

- `PadInitialization`
- `Launch`
- `Flight`
- `Landing`
- `Impact`

The synthetic sensor and instrument set includes:

- IMU
- GPS
- magnetometer
- barometric altitude
- radar altitude
- airspeed and vertical speed
- downrange
- battery voltage, current, and state of charge
- signal strength
- avionics and payload temperatures
- payload current
- impact indication

The simulation model now also includes:

- geodetic vehicle position derived from the local flight state
- a selectable geodetic target that can be updated from the web map
- a configurable propulsion model with throttle, thrust, dry mass, propellant mass, ISP, and fuel-consumption scaling
- a configurable 6-DoF vehicle model with aero reference geometry, force/moment coefficients, control-surface effectiveness, thrust-axis factors, and inertia scaling

The sim now uses split configuration:

- scenario/runtime settings in [`config/scenarios/default.toml`](C:\Users\jimpo\Desktop\FSW SDK\apps\full-stack-sim\config\scenarios\default.toml)
- base airframe settings in [`config/vehicles/single-string-default.toml`](C:\Users\jimpo\Desktop\FSW SDK\apps\full-stack-sim\config\vehicles\single-string-default.toml)

The scenario file points at a vehicle file with `vehicle_config = ...`, so you can swap airframes or scenario timing independently. The Rust `SimConfig` in `src/lib.rs` still carries a `vehicle` field of type `VehicleModelConfig`, but the normal way to tune the simulated airframe is to edit these config files instead of recompiling.

## Running

Start the interactive simulator:

```powershell
cargo run -p full-stack-sim
```

That automatically loads the default scenario:

```text
apps/full-stack-sim/config/scenarios/default.toml
```

Start the HTTP control server for a browser or other client:

```powershell
cargo run -p full-stack-sim -- serve 127.0.0.1:8080
```

Then open:

```text
http://127.0.0.1:8080/
```

You can also use:

```powershell
cargo run -p full-stack-sim -- --http 127.0.0.1:8080
```

To run with a different scenario file:

```powershell
cargo run -p full-stack-sim -- --config apps/full-stack-sim/config/scenarios/default.toml
```

or:

```powershell
cargo run -p full-stack-sim -- --config path\\to\\my-scenario.toml serve 127.0.0.1:8080
```

The default launch vector points east with zero elevation. Elevation and azimuth only define the direction; the altitude stays at whatever you last entered (default 0) unless you override it explicitly. Override the az/el/alt values when you need a different launch direction or climb height:

```powershell
cargo run -p full-stack-sim -- --launch-az 135 --launch-el 12 --launch-alt 500
```

`--launch-az` is the heading measured clockwise from true north, `--launch-el` is the elevation above the horizon, and `--launch-range` sets the synthetic target distance used to initialize the GNC stack.

To define a new airframe, create another vehicle file under `config/vehicles/` and point a scenario at it:

```text
vehicle_config = ../vehicles/my-airframe.toml
```

Useful commands:

```text
status
step [n]
reset
control <auto|manual>
phase <pad|launch|flight|landing|impact>
inject <gps-dropout|imu-bias|battery-sag|thermal-runaway|comm-loss|payload-fault|engine-failure|hard-landing>
clear <anomaly|all>
event <boot-complete|enter-operational|fault|power-emergency|thermal-emergency|recover-safe>
quit
```

## Webapp Direction

The simulation engine lives in `src/lib.rs` and exposes command/state types such as `SimulatorCommand`, `SimulationSnapshot`, and `FullStackSim`. That is intentional: a future browser UI can call the same APIs through a server layer or WASM boundary without having to replicate mission logic from the CLI.

The current CLI in `src/main.rs` is only a thin shell around that engine.

There is also a lightweight HTTP transport in `src/transport.rs` with permissive CORS enabled. It exposes:

- `GET /` to load the graphical mission console
- `GET /snapshot` to fetch the current sim state as JSON
- `POST /command` to apply a command and receive the updated snapshot
- `GET /schema` to enumerate supported commands and enum values
- `GET /events` to receive pushed snapshot updates as server-sent events

Typical browser flow:

1. Load `GET /` in the browser.
2. The console calls `GET /schema` at startup.
3. The console fetches `GET /snapshot` for the initial state.
4. The console opens `GET /events` with `EventSource` for live updates.
5. User actions go to `POST /command`.

The transport JSON is normalized for frontend use:

- enum values are lower-case strings
- top-level map and propulsion state is exposed through `geodetic`, `target`, and `propulsion`
- subsystem telemetry is structured under `telemetry.vehicle`, `telemetry.power`, `telemetry.thermal`, `telemetry.payload`, `telemetry.communications`, and `telemetry.guidance_navigation_control`
- targeting and propulsion views are also available under `telemetry.targeting` and `telemetry.propulsion`
  `telemetry.targeting.target_tracking` includes truth range, estimated range, cross-track error, vertical error, and closing speed for target convergence monitoring
- the older human-readable status lines are still available under `telemetry.raw` for debugging
- every snapshot and event includes `api_version` and `schema_version`
- `GET /schema` exposes a `command_catalog` block with labels, descriptions, paths, methods, and field metadata for UI generation

The graphical console served at `GET /` includes:

- mission summary and current phase strip
- dedicated `Launch` and `Run/Pause` controls for leaving pad and continuously advancing the sim
- a clickable mission map that shows vehicle latitude, longitude, altitude, flight trail, and target marker
  target changes from the map require confirmation before they are applied
- a propulsion panel for throttle, thrust, ISP, dry mass, propellant mass, and consumption scaling
- generated command/control panels
- subsystem health cards
- navigation and instrument readouts
- target-convergence readouts for truth/estimated range, cross-track error, vertical error, and closing speed
- a Controlled Flight toggle card that enables/disables GNC control surfaces
- live anomaly and fault-response views
- rolling trend plots for altitude, vertical speed, battery state of charge, and avionics temperature
  the UI automatically pauses the run loop when the vehicle reaches the ground and enters impact

Snapshot top-level fields:

- `api_version`
- `schema_version`
- `mission_time_ms`
- `phase`
- `phase_control`
- `flight_computer`
- `truth`
- `geodetic`
- `target`
- `propulsion`
- `sensors`
- `systems`
- `anomalies`
- `telemetry`
- `last_fault_responses`

Schema fields intended for frontend generation:

- `routes`
- `snapshot_stream_event`
- `snapshot_fields`
- `commands`
- `command_catalog`
- `phase_values`
- `phase_control_values`
- `anomaly_values`
- `vehicle_event_values`

Example command request:

```json
{
  "command": "inject",
  "anomaly": "comm-loss"
}
```

Example step request:

```json
{
  "command": "step",
  "count": 10
}
```

Example browser-side event subscription:

```javascript
const events = new EventSource("http://127.0.0.1:8080/events");
events.addEventListener("snapshot", (event) => {
  const snapshot = JSON.parse(event.data);
  console.log(snapshot.phase, snapshot.mission_time_ms);
});
```

Example frontend schema usage:

```javascript
const schema = await fetch("http://127.0.0.1:8080/schema").then((r) => r.json());
console.log(schema.api_version, schema.schema_version);
for (const command of schema.command_catalog) {
  console.log(command.label, command.fields);
}
```

Example command submission:

```javascript
await fetch("http://127.0.0.1:8080/command", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    command: "set_phase_control",
    phase_control: "manual",
  }),
});
```

Example target update from a web client:

```javascript
await fetch("http://127.0.0.1:8080/command", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    command: "set_target",
    latitude_deg: 34.91,
    longitude_deg: -120.22,
    altitude_m: 0,
  }),
});
```

Example propulsion update from a web client:

```javascript
await fetch("http://127.0.0.1:8080/command", {
  method: "POST",
  headers: { "Content-Type": "application/json" },
  body: JSON.stringify({
    command: "configure_propulsion",
    max_thrust_kn: 210,
    isp_s: 275,
    dry_mass_kg: 1450,
    propellant_mass_kg: 540,
    consumption_scale: 1.1,
  }),
});
```
