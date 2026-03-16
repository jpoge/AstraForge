// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use std::io::{self, Write};

use full_stack_sim::{
    config::load_sim_config as load_sim_config_file, transport::HttpSimServer, FullStackSim,
    MissionPhase, PhaseControl, SimAnomaly, SimConfig, SimulatorCommand,
};
use vehicle_management_template::VehicleEvent;

const DEFAULT_SIM_CONFIG_PATH: &str = "apps/full-stack-sim/config/scenarios/default.toml";

fn main() -> Result<(), fsw_sdk_core::SdkError> {
    let args: Vec<String> = std::env::args().collect();
    let mut sim_config = load_sim_config(&args)?;
    apply_launch_overrides(&mut sim_config, &args);
    if let Some(addr) = parse_http_addr(&args) {
        let server = HttpSimServer::new(FullStackSim::new(sim_config)?);
        println!("AstraForge full-stack sim HTTP server listening on http://{addr}");
        return server.run_blocking(&addr);
    }

    let mut sim = FullStackSim::new(sim_config)?;

    println!("AstraForge full-stack sim");
    println!("type `help` for commands");
    print_snapshot(&sim);

    let stdin = io::stdin();
    loop {
        print!("sim> ");
        io::stdout()
            .flush()
            .map_err(|_| fsw_sdk_core::SdkError::BackendFailure)?;

        let mut line = String::new();
        let bytes = stdin
            .read_line(&mut line)
            .map_err(|_| fsw_sdk_core::SdkError::BackendFailure)?;
        if bytes == 0 {
            break;
        }

        let command = line.trim();
        if command.is_empty() {
            continue;
        }
        if matches!(command, "quit" | "exit") {
            break;
        }
        if command == "help" {
            print_help();
            continue;
        }

        match handle_command(&mut sim, command) {
            Ok(()) => print_snapshot(&sim),
            Err(error) => eprintln!("error: {error}"),
        }
    }

    Ok(())
}

fn load_sim_config(args: &[String]) -> Result<SimConfig, fsw_sdk_core::SdkError> {
    match parse_config_path(args) {
        Some(path) => load_sim_config_file(path),
        None => match std::path::Path::new(DEFAULT_SIM_CONFIG_PATH).exists() {
            true => load_sim_config_file(DEFAULT_SIM_CONFIG_PATH),
            false => Ok(SimConfig::default()),
        },
    }
}

fn parse_http_addr(args: &[String]) -> Option<String> {
    for (index, arg) in args.iter().enumerate().skip(1) {
        match arg.as_str() {
            "serve" | "--http" => {
                return Some(
                    args.get(index + 1)
                        .filter(|value| !value.starts_with("--"))
                        .cloned()
                        .unwrap_or_else(|| "127.0.0.1:8080".to_string()),
                );
            }
            _ => {}
        }
    }
    None
}

fn parse_config_path(args: &[String]) -> Option<&str> {
    args.windows(2)
        .find(|window| window[0] == "--config")
        .map(|window| window[1].as_str())
}

fn apply_launch_overrides(config: &mut SimConfig, args: &[String]) {
    if let Some(az) = parse_launch_arg(args, "--launch-az") {
        config.launch_vector.azimuth_deg = az;
    }
    if let Some(el) = parse_launch_arg(args, "--launch-el") {
        config.launch_vector.elevation_deg = el;
    }
    if let Some(alt) = parse_launch_arg(args, "--launch-alt") {
        config.launch_vector.altitude_m = alt;
    }
}

fn parse_launch_arg(args: &[String], flag: &str) -> Option<f64> {
    args.windows(2)
        .find(|window| window[0] == flag)
        .and_then(|window| window[1].parse::<f64>().ok())
}

fn handle_command(sim: &mut FullStackSim, line: &str) -> Result<(), fsw_sdk_core::SdkError> {
    let mut parts = line.split_whitespace();
    let Some(command) = parts.next() else {
        return Ok(());
    };

    match command {
        "status" => Ok(()),
        "step" => {
            let count = parts
                .next()
                .and_then(|value| value.parse::<u32>().ok())
                .unwrap_or(1);
            sim.apply_command(SimulatorCommand::Step(count))
        }
        "reset" => sim.apply_command(SimulatorCommand::Reset),
        "phase" => {
            let phase = parse_phase(parts.next())?;
            sim.apply_command(SimulatorCommand::SetPhaseControl(PhaseControl::Manual))?;
            sim.apply_command(SimulatorCommand::SetPhase(phase))
        }
        "control" => {
            let control = match parts.next() {
                Some("auto") => PhaseControl::Auto,
                Some("manual") => PhaseControl::Manual,
                _ => return Err(fsw_sdk_core::SdkError::InvalidConfig),
            };
            sim.apply_command(SimulatorCommand::SetPhaseControl(control))
        }
        "inject" => {
            let anomaly = parse_anomaly(parts.next())?;
            sim.apply_command(SimulatorCommand::Inject(anomaly))
        }
        "set_controlled_flight" => {
            let enabled = parse_bool_flag(parts.next())?;
            sim.apply_command(SimulatorCommand::SetControlledFlight(enabled))
        }
        "clear" => match parts.next() {
            Some("all") => sim.apply_command(SimulatorCommand::ClearAllAnomalies),
            other => {
                let anomaly = parse_anomaly(other)?;
                sim.apply_command(SimulatorCommand::Clear(anomaly))
            }
        },
        "event" => {
            let event = parse_event(parts.next())?;
            sim.apply_command(SimulatorCommand::QueueVehicleEvent(event))
        }
        _ => Err(fsw_sdk_core::SdkError::InvalidConfig),
    }
}

fn parse_phase(value: Option<&str>) -> Result<MissionPhase, fsw_sdk_core::SdkError> {
    match value {
        Some("pad") => Ok(MissionPhase::PadInitialization),
        Some("launch") => Ok(MissionPhase::Launch),
        Some("flight") => Ok(MissionPhase::Flight),
        Some("landing") => Ok(MissionPhase::Landing),
        Some("impact") => Ok(MissionPhase::Impact),
        _ => Err(fsw_sdk_core::SdkError::InvalidConfig),
    }
}

fn parse_anomaly(value: Option<&str>) -> Result<SimAnomaly, fsw_sdk_core::SdkError> {
    match value {
        Some("gps-dropout") => Ok(SimAnomaly::GpsDropout),
        Some("imu-bias") => Ok(SimAnomaly::ImuBiasSpike),
        Some("battery-sag") => Ok(SimAnomaly::BatterySag),
        Some("thermal-runaway") => Ok(SimAnomaly::ThermalRunaway),
        Some("comm-loss") => Ok(SimAnomaly::CommLoss),
        Some("payload-fault") => Ok(SimAnomaly::PayloadFault),
        Some("engine-failure") => Ok(SimAnomaly::EngineFailure),
        Some("hard-landing") => Ok(SimAnomaly::HardLanding),
        _ => Err(fsw_sdk_core::SdkError::InvalidConfig),
    }
}

fn parse_event(value: Option<&str>) -> Result<VehicleEvent, fsw_sdk_core::SdkError> {
    match value {
        Some("boot-complete") => Ok(VehicleEvent::BootComplete),
        Some("enter-operational") => Ok(VehicleEvent::EnterOperational),
        Some("fault") => Ok(VehicleEvent::FaultDetected),
        Some("power-emergency") => Ok(VehicleEvent::PowerEmergency),
        Some("thermal-emergency") => Ok(VehicleEvent::ThermalEmergency),
        Some("recover-safe") => Ok(VehicleEvent::RecoverToSafe),
        _ => Err(fsw_sdk_core::SdkError::InvalidConfig),
    }
}

fn parse_bool_flag(value: Option<&str>) -> Result<bool, fsw_sdk_core::SdkError> {
    match value.map(|value| value.to_ascii_lowercase()) {
        Some(ref flag) if flag == "true" || flag == "on" => Ok(true),
        Some(ref flag) if flag == "false" || flag == "off" => Ok(false),
        _ => Err(fsw_sdk_core::SdkError::InvalidConfig),
    }
}

fn print_help() {
    println!("--config <path>    load sim/vehicle config from file");
    println!("--launch-az <deg>  override launch azimuth");
    println!("--launch-el <deg>  override launch elevation");
    println!("--launch-alt <m>   override launch altitude");
    println!("status");
    println!("step [n]");
    println!("reset");
    println!("control <auto|manual>");
    println!("set_controlled_flight <on|off>");
    println!("phase <pad|launch|flight|landing|impact>");
    println!("inject <gps-dropout|imu-bias|battery-sag|thermal-runaway|comm-loss|payload-fault|engine-failure|hard-landing>");
    println!("clear <anomaly|all>");
    println!("event <boot-complete|enter-operational|fault|power-emergency|thermal-emergency|recover-safe>");
    println!("quit");
}

fn print_snapshot(sim: &FullStackSim) {
    let snapshot = sim.snapshot();
    println!(
        "t={}ms phase={:?} control={:?} anomalies={:?}",
        snapshot.mission_time.0, snapshot.phase, snapshot.phase_control, snapshot.anomalies
    );
    println!(
        "truth pos=({:.1},{:.1},{:.1})m vel=({:.1},{:.1},{:.1})mps",
        snapshot.truth.position_m[0],
        snapshot.truth.position_m[1],
        snapshot.truth.position_m[2],
        snapshot.truth.velocity_mps[0],
        snapshot.truth.velocity_mps[1],
        snapshot.truth.velocity_mps[2]
    );
    println!(
        "sensors alt={:.1}m vs={:.1}mps air={:.1}mps batt={:.2}V/{:.0}% signal={:.1}dBm",
        snapshot.sensors.radar_altitude_m,
        snapshot.sensors.vertical_speed_mps,
        snapshot.sensors.airspeed_mps,
        snapshot.sensors.battery_voltage_v,
        snapshot.sensors.battery_soc * 100.0,
        snapshot.sensors.signal_strength_dbm
    );
    println!(
        "systems vehicle={:?} power={:?} thermal={:?} payload={:?} comm={:?}",
        snapshot.systems.vehicle_mode,
        snapshot.systems.power_mode,
        snapshot.systems.thermal_mode,
        snapshot.systems.payload_mode,
        snapshot.systems.comm_link_status
    );
    println!(
        "fc mode={:?} single_string={} cpu_load={:.0}% avionics_temp={:.1}C resets={}",
        snapshot.flight_computer.mode,
        snapshot.flight_computer.single_string,
        snapshot.flight_computer.cpu_load * 100.0,
        snapshot.flight_computer.avionics_temp_c,
        snapshot.flight_computer.resets
    );
    if !snapshot.last_fault_responses.is_empty() {
        println!("faults={:?}", snapshot.last_fault_responses);
    }
}
