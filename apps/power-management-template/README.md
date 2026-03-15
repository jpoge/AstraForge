# Power Management Template

Generic power-management template for AstraForge missions.

## Purpose

- ingest EPS and battery telemetry
- determine mission power mode and load-shedding policy
- publish status and trigger power-related safing actions

## Hook Points

- `update_snapshot()`: feed in real EPS/BMS telemetry
- `evaluate_power_mode()`: replace placeholder thresholds with mission policy
- `MissionApp::step()`: connect rail switching, inhibit outputs, and safing notifications

## Run The Example Launcher

```powershell
cargo run -p power-management-template
```

The launcher injects one example power snapshot and prints the resulting published power status.
