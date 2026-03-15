# AstraForge (Rust)

Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC

AstraForge is a small Rust workspace for building flight-software-style applications with explicit contracts for component lifecycle, time, message transport, OS abstraction, and fault detection, isolation, and recovery (FDIR).

It is structured as a foundation rather than a finished avionics stack. The crates in this repository define the core interfaces and a host-side runtime you can use to assemble mission components, validate dependencies, simulate supervision behavior, and evolve toward a target-specific deployment.

## What The SDK Provides

- Strongly typed core contracts for component IDs, lifecycle, faults, time, and messaging.
- A runtime that can register components, validate dependency graphs, start and stop them, and apply FDIR responses.
- Host-side time and OSAL implementations for simulation and development.
- A HAL-style platform boundary for swapping host and target backends.
- A narrow mission-app API for application code that should not depend directly on runtime internals.
- A minimal example showing how a component is registered and supervised.

## Workspace Layout

- `crates/fsw-sdk-core`: shared contracts and data types
- `crates/fsw-sdk-time`: `StaticClock` and `SystemClock`
- `crates/fsw-sdk-osal`: OS abstraction traits and host queue/task backend
- `crates/fsw-sdk-hal`: platform boundary for host-sim and future target backends
- `crates/fsw-sdk-fdir`: policy traits and basic class-to-response mapping
- `crates/fsw-sdk-runtime`: component registry, watchdog/supervisor, manifest parsing, in-memory bus
- `crates/fsw-sdk-app`: mission-facing app trait and context
- `apps/communications-template`: generic communications skeleton
- `apps/vehicle-management-template`: generic vehicle state-machine skeleton
- `apps/power-management-template`: generic power-management skeleton
- `apps/thermal-management-template`: generic thermal-management skeleton
- `apps/payload-interface-template`: generic payload integration skeleton
- `apps/simple-gnc`: config-driven 6-DoF GNC app assembled on top of AstraForge
- `apps/full-stack-sim`: standalone full-stack mission simulation with phase control and anomaly injection
- `examples/minimal-sim`: host-side composition example

## How To Think About The Crates

- Put mission-independent contracts in `fsw-sdk-core`.
- Put target or environment abstractions in `fsw-sdk-time` and `fsw-sdk-osal`.
- Put platform/backend selection and target adaptation in `fsw-sdk-hal`.
- Put system-level policy decisions in `fsw-sdk-fdir`.
- Put orchestration, registration, and supervision in `fsw-sdk-runtime`.
- Put mission logic on top of these crates instead of mixing it into the runtime.

## Current Capabilities

1. Manifest-driven registration with constrained YAML parsing via `register_with_manifest`.
2. Dependency graph validation across component-declared and manifest-declared required edges.
3. Supervisor/watchdog behavior through `SchedulerConfig`, `mark_heartbeat`, `supervisor_step`, and `run_supervisor_cycles`.
4. Fault response handling with distinct `Retry`, `Restart`, `Isolate`, and `EnterSafeMode` behavior.
5. Config-driven application launch path in `apps/simple-gnc` using the `host-sim` HAL backend.
6. A standalone full-stack simulator that exercises GNC, communications, vehicle management, power, thermal, and payload apps across pad, launch, flight, landing, and impact phases.
7. A browser-friendly transport for the full-stack simulator with normalized JSON snapshots, server-sent event push updates, and a versioned command/schema document for UI generation.

## Building Applications On Top Of The SDK

The simplest way to build on this SDK is to create a new crate in the workspace or a separate Cargo project that depends on the SDK crates by path or git reference.

### 1. Add Dependencies

For a workspace-local application crate, add dependencies like:

```toml
[dependencies]
fsw-sdk-core = { path = "../../crates/fsw-sdk-core" }
fsw-sdk-fdir = { path = "../../crates/fsw-sdk-fdir" }
fsw-sdk-runtime = { path = "../../crates/fsw-sdk-runtime" }
fsw-sdk-time = { path = "../../crates/fsw-sdk-time" }
```

If your app uses the mission app interface, also add:

```toml
fsw-sdk-app = { path = "../../crates/fsw-sdk-app" }
```

### 2. Implement A Component

Runtime-managed modules implement `FswComponent` from `fsw-sdk-core`. At minimum, define:

- `id()`
- `dependencies()`
- lifecycle hooks: `init`, `start`, `stop`, `reset`, `deinit`
- `health()`

The example in [`examples/minimal-sim/src/main.rs`](C:\Users\jimpo\Desktop\FSW SDK\examples\minimal-sim\src\main.rs) shows the intended shape.

### 3. Register The Component

Create a `Runtime`, choose an FDIR policy, and register components:

```rust
let policy = ClassMapPolicy::conservative();
let mut runtime = Runtime::new(policy);

runtime.register(Box::new(MyComponent::new()))?;
runtime.start_all()?;
```

If you want manifest validation, use `register_with_manifest` with a module manifest:

```yaml
id: my.component
requires:
- dependency.component
optional:
- telemetry.component
```

### 4. Drive Time And Supervision

For deterministic simulation, use `StaticClock`. For host wall-clock behavior, use `SystemClock`.

Typical host-side simulation flow:

```rust
let clock = StaticClock::new();
runtime.set_scheduler_config(SchedulerConfig {
    period_ms: 100,
    heartbeat_timeout_ms: 250,
});

runtime.start_all()?;
runtime.mark_heartbeat(&ComponentId("my.component".to_string()), MissionTime(0))?;
runtime.run_supervisor_cycles(&clock, 10)?;
```

### 5. Keep Mission Logic Above The Runtime

The runtime should own registration and supervision. Mission code should focus on:

- application state
- telemetry/command handling
- dependency declarations
- health reporting
- target-specific driver integration through future HAL/BSP layers

## Building The Workspace

From the repository root:

```powershell
cargo build
```

Run the test suite:

```powershell
cargo test
```

Run the example:

```powershell
cargo run -p minimal-sim
```

Run the standalone full-stack simulator:

```powershell
cargo run -p full-stack-sim
```

Run the standalone full-stack simulator with its HTTP transport:

```powershell
cargo run -p full-stack-sim -- serve 127.0.0.1:8080
```

The simulator HTTP surface is designed for frontend integration:

- `GET /snapshot`: fetch current normalized sim state
- `POST /command`: apply a simulator command and get the updated state
- `GET /events`: subscribe to pushed snapshot updates over server-sent events
- `GET /schema`: fetch the versioned frontend contract, enum values, and UI-oriented command catalog

The snapshot and schema payloads both include:

- `api_version`
- `schema_version`

## Compiling For A Specific Target

Rust uses target triples such as:

- `x86_64-pc-windows-msvc`
- `x86_64-unknown-linux-gnu`
- `aarch64-unknown-linux-gnu`
- `thumbv7em-none-eabihf`

### 1. Add The Rust Target

Install the target with `rustup`:

```powershell
rustup target add <target-triple>
```

Example:

```powershell
rustup target add aarch64-unknown-linux-gnu
```

### 2. Build For The Target

Build the whole workspace:

```powershell
cargo build --target <target-triple>
```

Or build a specific package:

```powershell
cargo build -p minimal-sim --target <target-triple>
```

Release build example:

```powershell
cargo build -p minimal-sim --release --target aarch64-unknown-linux-gnu
```

### 3. Run For A Host-Compatible Target

If the target matches a runnable environment on your machine, run with:

```powershell
cargo run -p minimal-sim --target <target-triple>
```

For example, on Windows:

```powershell
cargo run -p minimal-sim --target x86_64-pc-windows-msvc
```

### 4. Run For A Non-Host Or Embedded Target

For cross-compiled or embedded targets, `cargo run` is usually not enough. The normal flow is:

1. Compile with `cargo build --target <target-triple>`.
2. Locate the artifact under `target/<target-triple>/<profile>/`.
3. Load or deploy it with the target-specific runner, simulator, debugger, or flashing tool.

Examples:

- Linux SBC target: copy the binary to the board and execute it there.
- RTOS or bare-metal target: flash with probe/debug tooling.
- Simulator target: invoke the simulator with the built artifact.

## Build Profile Guidance

For constrained flight-computer deployments:

- Prefer `release` builds.
- Consider `lto = true`, `codegen-units = 1`, `panic = "abort"`, and `opt-level = "z"` where appropriate.
- Keep dependencies minimal and auditable.
- Avoid unnecessary allocation in hot or safety-critical paths.
- Plan for `std` vs `no_std` separation when introducing real flight-target support.

## Example Development Flow

1. Create a new crate for your mission app or component assembly.
2. Add SDK crate dependencies.
3. Implement one or more `FswComponent` types.
4. Register them in a runtime with a chosen FDIR policy.
5. Test on the host using `StaticClock` or `SystemClock`.
6. Build for the desired target triple.
7. Run locally, deploy remotely, or flash to hardware depending on the target.

## Next Milestones

1. Introduce a HAL crate with a C ABI shim for legacy drivers.
2. Add a fault-injection harness with scripted transient and persistent failures.
3. Split host simulation and flight-target support more explicitly across `std` and `no_std`.
4. Add static scheduling tables and WCET budget enforcement.

## License

Licensed under the Apache License, Version 2.0. See [`LICENSE`](C:\Users\jimpo\Desktop\FSW SDK\LICENSE).
