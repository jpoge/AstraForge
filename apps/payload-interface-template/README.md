# Payload Interface Template

Generic payload-interface template for AstraForge missions.

## Purpose

- isolate payload transport and packet handling from the rest of the mission software
- translate mission commands into payload-specific transactions
- report payload mode and health back to the vehicle

## Hook Points

- `poll_payload_link()`: connect the real payload transport
- `service_commands()`: translate payload commands and handle responses
- `MissionApp::step()`: add mission-specific interlocks with power, thermal, and vehicle mode

## Run The Example Launcher

```powershell
cargo run -p payload-interface-template
```

The launcher pushes one example command over the internal bus and prints the resulting payload status.
