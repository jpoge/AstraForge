# Simple GNC App

This crate is a standalone mission-app example built on top of AstraForge. It lives under `apps/` so it stays separate from the reusable SDK crates under `crates/`.

## What It Does

- Implements a simple 6-DoF EKF with state:
  - position `(x, y, z)`
  - velocity `(vx, vy, vz)`
  - attitude `(roll, pitch, yaw)`
  - gyro bias `(bx, by, bz)`
  - accelerometer bias `(ax, ay, az)`
- Accepts:
  - IMU readings for propagation
  - GPS position and velocity updates
  - magnetometer vector updates
- Publishes the latest navigation solution on the `gnc.solution` topic.
- Exposes the GNC logic as an SDK-integrated component and mission app.

## Mission Config

The app loads its EKF and simulation settings from [`config/mission.toml`](C:\Users\jimpo\Desktop\FSW SDK\apps\simple-gnc\config\mission.toml).

The file controls:

- platform/backend selection
- EKF propagation rate
- process and measurement noise settings
- simulation duration
- IMU, GPS, and magnetometer update periods
- watchdog heartbeat timeout

## Run It

```powershell
cargo run -p simple-gnc -- --config apps/simple-gnc/config/mission.toml
```

Optional override:

```powershell
cargo run -p simple-gnc -- --config apps/simple-gnc/config/mission.toml --rate-hz 50
```

`--rate-hz` overrides `ekf_rate_hz` from the mission config file.

## Layout

- `config/mission.toml`: mission and EKF configuration
- `src/ekf.rs`: EKF state, propagation, and measurement updates
- `src/component.rs`: SDK integration through `MissionApp` and `FswComponent`
- `src/launcher.rs`: config-driven mission assembly and platform launch path
- `src/main.rs`: CLI entry point that loads mission config and invokes the launcher
- `module.yaml`: manifest used for runtime registration
