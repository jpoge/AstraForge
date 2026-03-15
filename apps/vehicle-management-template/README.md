# Vehicle Management Template

Generic vehicle-management template with a mission-mode state machine.

## Purpose

- own the top-level vehicle mode/state machine
- collect subsystem status and autonomy events
- trigger mode-entry and mode-exit actions

## Hook Points

- `collect_mode_inputs()`: read inputs from power, thermal, communications, payload, and autonomy logic
- `apply_event()`: extend the state machine with mission-specific modes and transitions
- `MissionApp::step()`: add mode-entry actions, safing actions, and command broadcasts

## Run The Example Launcher

```powershell
cargo run -p vehicle-management-template
```

The launcher queues a few example events and prints the resulting published mode.
