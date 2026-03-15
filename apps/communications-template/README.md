# Communications Template

Generic communications app template for AstraForge missions.

## Purpose

- provide a place to connect a radio, modem, socket, or transport driver
- route uplink frames into mission parsing logic
- manage downlink buffering and status publication

## Hook Points

- `poll_external_link()`: connect your real link driver and validation path
- `flush_downlink()`: replace the bus-only placeholder with your transmit path
- `MissionApp::step()`: connect frame parsing, routing, and command dispatch

## Run The Example Launcher

```powershell
cargo run -p communications-template
```

The launcher creates a host-sim runtime, queues one example downlink frame, runs one step, and prints the resulting status and downlink payloads.
