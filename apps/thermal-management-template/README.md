# Thermal Management Template

Generic thermal-management template for AstraForge missions.

## Purpose

- collect thermal telemetry for mission-defined zones
- evaluate thermal mode and alerting policy
- host the control logic for heaters, radiators, or other thermal actuators

## Hook Points

- `update_zone_temperature()`: feed in thermal sensor telemetry
- `evaluate_mode()`: replace placeholder limits with mission policy
- `MissionApp::step()`: connect actuator commands and thermal safing notifications

## Run The Example Launcher

```powershell
cargo run -p thermal-management-template
```

The launcher injects a couple of zone temperatures and prints the resulting thermal status.
