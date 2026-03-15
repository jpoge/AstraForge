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

## Running

Start the interactive simulator:

```powershell
cargo run -p full-stack-sim
```

Start the HTTP control server for a browser or other client:

```powershell
cargo run -p full-stack-sim -- serve 127.0.0.1:8080
```

You can also use:

```powershell
cargo run -p full-stack-sim -- --http 127.0.0.1:8080
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

- `GET /snapshot` to fetch the current sim state as JSON
- `POST /command` to apply a command and receive the updated snapshot
- `GET /schema` to enumerate supported commands and enum values
- `GET /events` to receive pushed snapshot updates as server-sent events

Typical browser flow:

1. Call `GET /schema` once at startup.
2. Build command controls from `command_catalog` and enum lists.
3. Fetch `GET /snapshot` for the initial state.
4. Open `GET /events` with `EventSource` for live updates.
5. Send user actions to `POST /command`.

The transport JSON is normalized for frontend use:

- enum values are lower-case strings
- subsystem telemetry is structured under `telemetry.vehicle`, `telemetry.power`, `telemetry.thermal`, `telemetry.payload`, `telemetry.communications`, and `telemetry.guidance_navigation_control`
- the older human-readable status lines are still available under `telemetry.raw` for debugging
- every snapshot and event includes `api_version` and `schema_version`
- `GET /schema` exposes a `command_catalog` block with labels, descriptions, paths, methods, and field metadata for UI generation

Snapshot top-level fields:

- `api_version`
- `schema_version`
- `mission_time_ms`
- `phase`
- `phase_control`
- `flight_computer`
- `truth`
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
