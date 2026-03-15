// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Minimal HTTP control surface for the full-stack simulator.

use std::io::{Read, Write};
use std::net::{TcpListener, TcpStream};
use std::sync::mpsc::{self, Receiver, Sender};
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::Duration;

use crate::{
    FlightComputerMode, FlightComputerStatus, FullStackSim, MissionPhase, PhaseControl,
    SensorSnapshot, SimAnomaly, SimulationSnapshot, SimulatorCommand, SystemSnapshot, TruthState,
};
use fsw_sdk_core::SdkError;
use payload_interface_template::PayloadMode;
use power_management_template::PowerMode;
use simple_gnc::{GpsReading, ImuReading, MagnetometerReading, NavigationSolution};
use thermal_management_template::ThermalMode;
use vehicle_management_template::{VehicleEvent, VehicleMode};

const API_VERSION: &str = "2026-03-15";
const SCHEMA_VERSION: &str = "1.0.0";

pub type SharedSim = Arc<Mutex<FullStackSim>>;

pub struct HttpSimServer {
    sim: SharedSim,
    events: SharedEventHub,
}

impl HttpSimServer {
    #[must_use]
    pub fn new(sim: FullStackSim) -> Self {
        Self {
            sim: Arc::new(Mutex::new(sim)),
            events: Arc::new(Mutex::new(EventHub::default())),
        }
    }

    #[must_use]
    pub fn from_shared(sim: SharedSim) -> Self {
        Self {
            sim,
            events: Arc::new(Mutex::new(EventHub::default())),
        }
    }

    #[must_use]
    pub fn shared_sim(&self) -> SharedSim {
        Arc::clone(&self.sim)
    }

    pub fn run_blocking(&self, addr: &str) -> Result<(), SdkError> {
        let listener = TcpListener::bind(addr).map_err(|_| SdkError::BackendFailure)?;
        for stream in listener.incoming() {
            let mut stream = stream.map_err(|_| SdkError::BackendFailure)?;
            let sim = Arc::clone(&self.sim);
            let events = Arc::clone(&self.events);
            thread::spawn(move || {
                let _ = handle_connection(&sim, &events, &mut stream);
            });
        }
        Ok(())
    }
}

type SharedEventHub = Arc<Mutex<EventHub>>;

#[derive(Default)]
struct EventHub {
    subscribers: Vec<Sender<String>>,
}

impl EventHub {
    fn subscribe(&mut self) -> Receiver<String> {
        let (tx, rx) = mpsc::channel();
        self.subscribers.push(tx);
        rx
    }

    fn publish(&mut self, payload: &str) {
        self.subscribers
            .retain(|subscriber| subscriber.send(payload.to_string()).is_ok());
    }
}

pub fn snapshot_json(snapshot: &SimulationSnapshot) -> String {
    let anomalies = join_json_array(
        snapshot
            .anomalies
            .iter()
            .map(|anomaly| format!("\"{}\"", anomaly_name(*anomaly))),
    );
    let fault_responses = join_json_array(snapshot.last_fault_responses.iter().map(|(component, response)| {
        format!(
            "{{\"component\":\"{}\",\"response\":\"{}\"}}",
            escape_json(component),
            escape_json(response)
        )
    }));

    format!(
        concat!(
            "{{",
            "\"api_version\":\"{}\",",
            "\"schema_version\":\"{}\",",
            "\"mission_time_ms\":{},",
            "\"phase\":\"{}\",",
            "\"phase_control\":\"{}\",",
            "\"flight_computer\":{},",
            "\"truth\":{},",
            "\"sensors\":{},",
            "\"systems\":{},",
            "\"anomalies\":[{}],",
            "\"telemetry\":{},",
            "\"last_fault_responses\":[{}]",
            "}}"
        ),
        API_VERSION,
        SCHEMA_VERSION,
        snapshot.mission_time.0,
        phase_name(snapshot.phase),
        phase_control_name(snapshot.phase_control),
        flight_computer_json(snapshot.flight_computer),
        truth_state_json(snapshot.truth),
        sensor_snapshot_json(snapshot.sensors),
        system_snapshot_json(&snapshot.systems),
        anomalies,
        telemetry_snapshot_json(snapshot),
        fault_responses
    )
}

pub fn schema_json() -> String {
    format!(
        concat!(
            "{{",
            "\"api_version\":\"{}\",",
            "\"schema_version\":\"{}\",",
            "\"routes\":[",
            "\"GET /snapshot\",",
            "\"POST /command\",",
            "\"GET /schema\",",
            "\"GET /events\"",
            "],",
            "\"snapshot_stream_event\":\"snapshot\",",
            "\"snapshot_fields\":[",
            "\"api_version\",",
            "\"schema_version\",",
            "\"mission_time_ms\",",
            "\"phase\",",
            "\"phase_control\",",
            "\"flight_computer\",",
            "\"truth\",",
            "\"sensors\",",
            "\"systems\",",
            "\"anomalies\",",
            "\"telemetry\",",
            "\"last_fault_responses\"",
            "],",
            "\"commands\":{{",
            "\"step\":{{\"fields\":[\"count\"]}},",
            "\"reset\":{{\"fields\":[]}},",
            "\"set_phase\":{{\"fields\":[\"phase\"]}},",
            "\"set_phase_control\":{{\"fields\":[\"phase_control\"]}},",
            "\"inject\":{{\"fields\":[\"anomaly\"]}},",
            "\"clear\":{{\"fields\":[\"anomaly\"]}},",
            "\"clear_all_anomalies\":{{\"fields\":[]}},",
            "\"queue_vehicle_event\":{{\"fields\":[\"event\"]}}",
            "}},",
            "\"command_catalog\":[{}],",
            "\"phase_values\":[{}],",
            "\"phase_control_values\":[{}],",
            "\"anomaly_values\":[{}],",
            "\"vehicle_event_values\":[{}]",
            "}}"
        ),
        API_VERSION,
        SCHEMA_VERSION,
        join_json_array(command_catalog().into_iter()),
        join_json_array(
            [
                "pad",
                "launch",
                "flight",
                "landing",
                "impact",
            ]
            .into_iter()
            .map(|value| format!("\"{value}\""))
        ),
        join_json_array(["auto", "manual"].into_iter().map(|value| format!("\"{value}\""))),
        join_json_array(
            [
                "gps-dropout",
                "imu-bias",
                "battery-sag",
                "thermal-runaway",
                "comm-loss",
                "payload-fault",
                "engine-failure",
                "hard-landing",
            ]
            .into_iter()
            .map(|value| format!("\"{value}\""))
        ),
        join_json_array(
            [
                "boot-complete",
                "enter-operational",
                "fault",
                "power-emergency",
                "thermal-emergency",
                "recover-safe",
            ]
            .into_iter()
            .map(|value| format!("\"{value}\""))
        )
    )
}

pub fn parse_command_json(body: &str) -> Result<SimulatorCommand, SdkError> {
    let command = extract_json_string(body, "command").ok_or(SdkError::InvalidConfig)?;
    match command.as_str() {
        "step" => Ok(SimulatorCommand::Step(
            extract_json_u32(body, "count").unwrap_or(1),
        )),
        "reset" => Ok(SimulatorCommand::Reset),
        "set_phase" => Ok(SimulatorCommand::SetPhase(parse_phase(
            extract_json_string(body, "phase").as_deref(),
        )?)),
        "set_phase_control" => Ok(SimulatorCommand::SetPhaseControl(parse_phase_control(
            extract_json_string(body, "phase_control").as_deref(),
        )?)),
        "inject" => Ok(SimulatorCommand::Inject(parse_anomaly(
            extract_json_string(body, "anomaly").as_deref(),
        )?)),
        "clear" => Ok(SimulatorCommand::Clear(parse_anomaly(
            extract_json_string(body, "anomaly").as_deref(),
        )?)),
        "clear_all_anomalies" => Ok(SimulatorCommand::ClearAllAnomalies),
        "queue_vehicle_event" => Ok(SimulatorCommand::QueueVehicleEvent(parse_vehicle_event(
            extract_json_string(body, "event").as_deref(),
        )?)),
        _ => Err(SdkError::InvalidConfig),
    }
}

fn handle_connection(
    sim: &SharedSim,
    events: &SharedEventHub,
    stream: &mut TcpStream,
) -> Result<(), SdkError> {
    let request = read_http_request(stream)?;
    let method = request.method.as_str();
    let path = request.path.as_str();

    if method == "GET" && path == "/events" {
        return stream_events(sim, events, stream);
    }

    let response = match (method, path) {
        ("GET", "/snapshot") => {
            let sim = sim.lock().map_err(|_| SdkError::BackendFailure)?;
            json_response(200, &snapshot_json(&sim.snapshot()))
        }
        ("GET", "/schema") => json_response(200, &schema_json()),
        ("POST", "/command") => {
            let command = parse_command_json(&request.body)?;
            let mut sim = sim.lock().map_err(|_| SdkError::BackendFailure)?;
            sim.apply_command(command)?;
            let snapshot = snapshot_json(&sim.snapshot());
            events
                .lock()
                .map_err(|_| SdkError::BackendFailure)?
                .publish(&snapshot);
            json_response(200, &snapshot)
        }
        ("OPTIONS", "/snapshot")
        | ("OPTIONS", "/command")
        | ("OPTIONS", "/schema")
        | ("OPTIONS", "/events") => {
            empty_response(204)
        }
        _ => json_response(404, "{\"error\":\"not_found\"}"),
    };

    stream
        .write_all(response.as_bytes())
        .map_err(|_| SdkError::BackendFailure)
}

fn stream_events(
    sim: &SharedSim,
    events: &SharedEventHub,
    stream: &mut TcpStream,
) -> Result<(), SdkError> {
    let rx = events
        .lock()
        .map_err(|_| SdkError::BackendFailure)?
        .subscribe();
    let initial_snapshot = {
        let sim = sim.lock().map_err(|_| SdkError::BackendFailure)?;
        snapshot_json(&sim.snapshot())
    };

    let header = concat!(
        "HTTP/1.1 200 OK\r\n",
        "Content-Type: text/event-stream\r\n",
        "Cache-Control: no-cache\r\n",
        "Connection: keep-alive\r\n",
        "Access-Control-Allow-Origin: *\r\n",
        "Access-Control-Allow-Methods: GET, POST, OPTIONS\r\n",
        "Access-Control-Allow-Headers: Content-Type\r\n\r\n"
    );
    stream
        .write_all(header.as_bytes())
        .map_err(|_| SdkError::BackendFailure)?;
    write_sse_event(stream, "snapshot", &initial_snapshot)?;

    loop {
        match rx.recv_timeout(Duration::from_secs(2)) {
            Ok(snapshot) => write_sse_event(stream, "snapshot", &snapshot)?,
            Err(mpsc::RecvTimeoutError::Timeout) => {
                stream
                    .write_all(b": keep-alive\n\n")
                    .map_err(|_| SdkError::BackendFailure)?;
            }
            Err(mpsc::RecvTimeoutError::Disconnected) => return Ok(()),
        }
    }
}

fn write_sse_event(stream: &mut TcpStream, event: &str, payload: &str) -> Result<(), SdkError> {
    let frame = format!("event: {event}\ndata: {payload}\n\n");
    stream
        .write_all(frame.as_bytes())
        .map_err(|_| SdkError::BackendFailure)
}

struct HttpRequest {
    method: String,
    path: String,
    body: String,
}

fn read_http_request(stream: &mut TcpStream) -> Result<HttpRequest, SdkError> {
    let mut buffer = Vec::with_capacity(16 * 1024);
    let mut chunk = [0_u8; 2048];
    let mut header_end = None;
    let mut content_length = 0_usize;

    loop {
        let bytes = stream
            .read(&mut chunk)
            .map_err(|_| SdkError::BackendFailure)?;
        if bytes == 0 {
            break;
        }
        buffer.extend_from_slice(&chunk[..bytes]);
        if header_end.is_none() {
            header_end = find_header_end(&buffer);
            if let Some(end) = header_end {
                let header_text = String::from_utf8_lossy(&buffer[..end]).into_owned();
                content_length = parse_content_length(&header_text);
                if buffer.len() >= end + 4 + content_length {
                    break;
                }
            }
        } else if let Some(end) = header_end {
            if buffer.len() >= end + 4 + content_length {
                break;
            }
        }
    }

    let header_end = header_end.ok_or(SdkError::InvalidConfig)?;
    let header_text = String::from_utf8_lossy(&buffer[..header_end]).into_owned();
    let mut lines = header_text.split("\r\n");
    let request_line = lines.next().ok_or(SdkError::InvalidConfig)?;
    let mut request_parts = request_line.split_whitespace();
    let method = request_parts
        .next()
        .ok_or(SdkError::InvalidConfig)?
        .to_string();
    let path = request_parts
        .next()
        .ok_or(SdkError::InvalidConfig)?
        .to_string();

    let body_start = header_end + 4;
    let body_end = body_start.saturating_add(content_length).min(buffer.len());
    let body = String::from_utf8_lossy(&buffer[body_start..body_end]).into_owned();

    Ok(HttpRequest { method, path, body })
}

fn find_header_end(buffer: &[u8]) -> Option<usize> {
    buffer.windows(4).position(|window| window == b"\r\n\r\n")
}

fn parse_content_length(headers: &str) -> usize {
    headers
        .lines()
        .find_map(|line| {
            let (name, value) = line.split_once(':')?;
            if name.eq_ignore_ascii_case("Content-Length") {
                value.trim().parse::<usize>().ok()
            } else {
                None
            }
        })
        .unwrap_or(0)
}

fn json_response(status: u16, body: &str) -> String {
    let reason = match status {
        200 => "OK",
        204 => "No Content",
        404 => "Not Found",
        _ => "OK",
    };
    format!(
        "HTTP/1.1 {status} {reason}\r\nContent-Type: application/json\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type\r\nContent-Length: {}\r\nConnection: close\r\n\r\n{body}",
        body.len()
    )
}

fn empty_response(status: u16) -> String {
    let reason = if status == 204 { "No Content" } else { "OK" };
    format!(
        "HTTP/1.1 {status} {reason}\r\nAccess-Control-Allow-Origin: *\r\nAccess-Control-Allow-Methods: GET, POST, OPTIONS\r\nAccess-Control-Allow-Headers: Content-Type\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"
    )
}

fn flight_computer_json(status: FlightComputerStatus) -> String {
    format!(
        concat!(
            "{{",
            "\"powered\":{},",
            "\"single_string\":{},",
            "\"resets\":{},",
            "\"mode\":\"{}\",",
            "\"cpu_load\":{},",
            "\"avionics_temp_c\":{}",
            "}}"
        ),
        status.powered,
        status.single_string,
        status.resets,
        flight_computer_mode_name(status.mode),
        status.cpu_load,
        status.avionics_temp_c
    )
}

fn truth_state_json(truth: TruthState) -> String {
    format!(
        "{{\"position_m\":{},\"velocity_mps\":{},\"attitude_rad\":{},\"body_accel_mps2\":{},\"body_rates_rps\":{}}}",
        vec3_json(truth.position_m),
        vec3_json(truth.velocity_mps),
        vec3_json(truth.attitude_rad),
        vec3_json(truth.body_accel_mps2),
        vec3_json(truth.body_rates_rps)
    )
}

fn sensor_snapshot_json(sensor: SensorSnapshot) -> String {
    format!(
        concat!(
            "{{",
            "\"imu\":{},",
            "\"gps\":{},",
            "\"magnetometer\":{},",
            "\"barometric_altitude_m\":{},",
            "\"radar_altitude_m\":{},",
            "\"airspeed_mps\":{},",
            "\"vertical_speed_mps\":{},",
            "\"downrange_m\":{},",
            "\"battery_voltage_v\":{},",
            "\"battery_current_a\":{},",
            "\"battery_soc\":{},",
            "\"signal_strength_dbm\":{},",
            "\"avionics_temp_c\":{},",
            "\"payload_temp_c\":{},",
            "\"payload_current_a\":{},",
            "\"impact_detected\":{}",
            "}}"
        ),
        imu_json(sensor.imu),
        option_gps_json(sensor.gps),
        magnetometer_json(sensor.magnetometer),
        sensor.barometric_altitude_m,
        sensor.radar_altitude_m,
        sensor.airspeed_mps,
        sensor.vertical_speed_mps,
        sensor.downrange_m,
        sensor.battery_voltage_v,
        sensor.battery_current_a,
        sensor.battery_soc,
        sensor.signal_strength_dbm,
        sensor.avionics_temp_c,
        sensor.payload_temp_c,
        sensor.payload_current_a,
        sensor.impact_detected
    )
}

fn system_snapshot_json(system: &SystemSnapshot) -> String {
    format!(
        "{{\"vehicle_mode\":\"{}\",\"power_mode\":\"{}\",\"thermal_mode\":\"{}\",\"payload_mode\":\"{}\",\"comm_link_status\":\"{}\",\"gnc_solution\":{}}}",
        vehicle_mode_name(system.vehicle_mode),
        power_mode_name(system.power_mode),
        thermal_mode_name(system.thermal_mode),
        payload_mode_name(system.payload_mode),
        link_status_name(system.comm_link_status),
        navigation_solution_json(system.gnc_solution)
    )
}

fn telemetry_snapshot_json(snapshot: &SimulationSnapshot) -> String {
    let power_health = status_health_name(&snapshot.telemetry.power_status_line);
    let thermal_health = status_health_name(&snapshot.telemetry.thermal_status_line);
    let payload_health = status_health_name(&snapshot.telemetry.payload_status_line);
    let comm_health = status_health_name(&snapshot.telemetry.comm_status_line);
    format!(
        concat!(
            "{{",
            "\"vehicle\":{{\"mode\":\"{}\"}},",
            "\"power\":{{\"mode\":\"{}\",\"health\":\"{}\",\"bus_voltage_v\":{},\"bus_current_a\":{},\"battery_soc\":{}}},",
            "\"thermal\":{{\"mode\":\"{}\",\"health\":\"{}\",\"zones\":[{}, {}, {}]}},",
            "\"payload\":{{\"mode\":\"{}\",\"health\":\"{}\",\"current_a\":{}}},",
            "\"communications\":{{\"link_status\":\"{}\",\"health\":\"{}\",\"signal_strength_dbm\":{},\"latest_downlink_frame\":\"{}\"}},",
            "\"guidance_navigation_control\":{{\"solution\":{}}},",
            "\"raw\":{{",
            "\"vehicle_mode_line\":\"{}\",",
            "\"power_status_line\":\"{}\",",
            "\"thermal_status_line\":\"{}\",",
            "\"payload_status_line\":\"{}\",",
            "\"comm_status_line\":\"{}\",",
            "\"gnc_solution_line\":\"{}\",",
            "\"latest_downlink_frame\":\"{}\"",
            "}}",
            "}}"
        ),
        vehicle_mode_name(snapshot.systems.vehicle_mode),
        power_mode_name(snapshot.systems.power_mode),
        power_health,
        snapshot.sensors.battery_voltage_v,
        snapshot.sensors.battery_current_a,
        snapshot.sensors.battery_soc,
        thermal_mode_name(snapshot.systems.thermal_mode),
        thermal_health,
        thermal_zone_json("avionics", snapshot.sensors.avionics_temp_c),
        thermal_zone_json("payload", snapshot.sensors.payload_temp_c),
        thermal_zone_json("battery", snapshot.sensors.avionics_temp_c - 4.0),
        payload_mode_name(snapshot.systems.payload_mode),
        payload_health,
        snapshot.sensors.payload_current_a,
        link_status_name(snapshot.systems.comm_link_status),
        comm_health,
        snapshot.sensors.signal_strength_dbm,
        escape_json(&snapshot.telemetry.latest_downlink_frame),
        navigation_solution_json(snapshot.systems.gnc_solution),
        escape_json(&snapshot.telemetry.vehicle_mode_line),
        escape_json(&snapshot.telemetry.power_status_line),
        escape_json(&snapshot.telemetry.thermal_status_line),
        escape_json(&snapshot.telemetry.payload_status_line),
        escape_json(&snapshot.telemetry.comm_status_line),
        escape_json(&snapshot.telemetry.gnc_solution_line),
        escape_json(&snapshot.telemetry.latest_downlink_frame)
    )
}

fn imu_json(imu: ImuReading) -> String {
    format!(
        "{{\"timestamp_ms\":{},\"accel_body_mps2\":{},\"gyro_body_rps\":{}}}",
        imu.timestamp.0,
        vec3_json(imu.accel_body_mps2),
        vec3_json(imu.gyro_body_rps)
    )
}

fn option_gps_json(gps: Option<GpsReading>) -> String {
    gps.map_or_else(
        || "null".to_string(),
        |gps| {
            format!(
                "{{\"timestamp_ms\":{},\"position_m\":{},\"velocity_mps\":{}}}",
                gps.timestamp.0,
                vec3_json(gps.position_m),
                vec3_json(gps.velocity_mps)
            )
        },
    )
}

fn magnetometer_json(mag: MagnetometerReading) -> String {
    format!(
        "{{\"timestamp_ms\":{},\"magnetic_field_body\":{}}}",
        mag.timestamp.0,
        vec3_json(mag.magnetic_field_body)
    )
}

fn navigation_solution_json(solution: NavigationSolution) -> String {
    format!(
        concat!(
            "{{",
            "\"timestamp_ms\":{},",
            "\"position_m\":{},",
            "\"velocity_mps\":{},",
            "\"attitude_rad\":{},",
            "\"gyro_bias_rps\":{},",
            "\"accel_bias_mps2\":{}",
            "}}"
        ),
        solution.timestamp.0,
        vec3_json(solution.position_m),
        vec3_json(solution.velocity_mps),
        vec3_json(solution.attitude_rad),
        vec3_json(solution.gyro_bias_rps),
        vec3_json(solution.accel_bias_mps2)
    )
}

fn command_catalog() -> Vec<String> {
    vec![
        command_descriptor_json(
            "step",
            "Step Simulation",
            "Advance the simulator by one or more scheduler ticks.",
            "[{\"name\":\"count\",\"type\":\"u32\",\"required\":false,\"default\":1,\"min\":1}]",
        ),
        command_descriptor_json(
            "reset",
            "Reset Flight",
            "Reset the simulator back to pad initialization.",
            "[]",
        ),
        command_descriptor_json(
            "set_phase",
            "Set Phase",
            "Switch the mission phase directly. Use with manual phase control.",
            "[{\"name\":\"phase\",\"type\":\"enum\",\"required\":true,\"enum_ref\":\"phase_values\"}]",
        ),
        command_descriptor_json(
            "set_phase_control",
            "Set Phase Control",
            "Choose whether the sim advances phases automatically or manually.",
            "[{\"name\":\"phase_control\",\"type\":\"enum\",\"required\":true,\"enum_ref\":\"phase_control_values\"}]",
        ),
        command_descriptor_json(
            "inject",
            "Inject Anomaly",
            "Inject an anomaly or failure into the flight stack.",
            "[{\"name\":\"anomaly\",\"type\":\"enum\",\"required\":true,\"enum_ref\":\"anomaly_values\"}]",
        ),
        command_descriptor_json(
            "clear",
            "Clear Anomaly",
            "Clear a previously injected anomaly.",
            "[{\"name\":\"anomaly\",\"type\":\"enum\",\"required\":true,\"enum_ref\":\"anomaly_values\"}]",
        ),
        command_descriptor_json(
            "clear_all_anomalies",
            "Clear All Anomalies",
            "Return the sim to a nominal anomaly-free state.",
            "[]",
        ),
        command_descriptor_json(
            "queue_vehicle_event",
            "Queue Vehicle Event",
            "Inject a vehicle-management state-machine event.",
            "[{\"name\":\"event\",\"type\":\"enum\",\"required\":true,\"enum_ref\":\"vehicle_event_values\"}]",
        ),
    ]
}

fn command_descriptor_json(
    name: &str,
    label: &str,
    description: &str,
    fields_json: &str,
) -> String {
    format!(
        concat!(
            "{{",
            "\"name\":\"{}\",",
            "\"label\":\"{}\",",
            "\"description\":\"{}\",",
            "\"method\":\"POST\",",
            "\"path\":\"/command\",",
            "\"body_format\":\"json\",",
            "\"fields\":{}",
            "}}"
        ),
        escape_json(name),
        escape_json(label),
        escape_json(description),
        fields_json
    )
}

fn thermal_zone_json(name: &str, temperature_c: f64) -> String {
    format!(
        "{{\"name\":\"{}\",\"temperature_c\":{}}}",
        escape_json(name),
        temperature_c
    )
}

fn vec3_json(values: [f64; 3]) -> String {
    format!("[{},{},{}]", values[0], values[1], values[2])
}

fn join_json_array<I>(items: I) -> String
where
    I: Iterator<Item = String>,
{
    items.collect::<Vec<_>>().join(",")
}

fn escape_json(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for ch in value.chars() {
        match ch {
            '\\' => escaped.push_str("\\\\"),
            '"' => escaped.push_str("\\\""),
            '\n' => escaped.push_str("\\n"),
            '\r' => escaped.push_str("\\r"),
            '\t' => escaped.push_str("\\t"),
            _ => escaped.push(ch),
        }
    }
    escaped
}

fn status_health_name(line: &str) -> &'static str {
    if line.contains("health=Failed") {
        "failed"
    } else if line.contains("health=Degraded") {
        "degraded"
    } else {
        "nominal"
    }
}

fn extract_json_string(input: &str, key: &str) -> Option<String> {
    let key_index = input.find(&format!("\"{key}\""))?;
    let value_start = input[key_index..].find(':')? + key_index + 1;
    let remainder = input[value_start..].trim_start();
    let quoted = remainder.strip_prefix('"')?;
    let end = quoted.find('"')?;
    Some(quoted[..end].to_string())
}

fn extract_json_u32(input: &str, key: &str) -> Option<u32> {
    let key_index = input.find(&format!("\"{key}\""))?;
    let value_start = input[key_index..].find(':')? + key_index + 1;
    let digits: String = input[value_start..]
        .trim_start()
        .chars()
        .take_while(|ch| ch.is_ascii_digit())
        .collect();
    digits.parse().ok()
}

fn parse_phase(value: Option<&str>) -> Result<MissionPhase, SdkError> {
    match value {
        Some("pad") => Ok(MissionPhase::PadInitialization),
        Some("launch") => Ok(MissionPhase::Launch),
        Some("flight") => Ok(MissionPhase::Flight),
        Some("landing") => Ok(MissionPhase::Landing),
        Some("impact") => Ok(MissionPhase::Impact),
        _ => Err(SdkError::InvalidConfig),
    }
}

fn parse_phase_control(value: Option<&str>) -> Result<PhaseControl, SdkError> {
    match value {
        Some("auto") => Ok(PhaseControl::Auto),
        Some("manual") => Ok(PhaseControl::Manual),
        _ => Err(SdkError::InvalidConfig),
    }
}

fn parse_anomaly(value: Option<&str>) -> Result<SimAnomaly, SdkError> {
    match value {
        Some("gps-dropout") => Ok(SimAnomaly::GpsDropout),
        Some("imu-bias") => Ok(SimAnomaly::ImuBiasSpike),
        Some("battery-sag") => Ok(SimAnomaly::BatterySag),
        Some("thermal-runaway") => Ok(SimAnomaly::ThermalRunaway),
        Some("comm-loss") => Ok(SimAnomaly::CommLoss),
        Some("payload-fault") => Ok(SimAnomaly::PayloadFault),
        Some("engine-failure") => Ok(SimAnomaly::EngineFailure),
        Some("hard-landing") => Ok(SimAnomaly::HardLanding),
        _ => Err(SdkError::InvalidConfig),
    }
}

fn parse_vehicle_event(value: Option<&str>) -> Result<VehicleEvent, SdkError> {
    match value {
        Some("boot-complete") => Ok(VehicleEvent::BootComplete),
        Some("enter-operational") => Ok(VehicleEvent::EnterOperational),
        Some("fault") => Ok(VehicleEvent::FaultDetected),
        Some("power-emergency") => Ok(VehicleEvent::PowerEmergency),
        Some("thermal-emergency") => Ok(VehicleEvent::ThermalEmergency),
        Some("recover-safe") => Ok(VehicleEvent::RecoverToSafe),
        _ => Err(SdkError::InvalidConfig),
    }
}

fn phase_name(value: MissionPhase) -> &'static str {
    match value {
        MissionPhase::PadInitialization => "pad",
        MissionPhase::Launch => "launch",
        MissionPhase::Flight => "flight",
        MissionPhase::Landing => "landing",
        MissionPhase::Impact => "impact",
    }
}

fn phase_control_name(value: PhaseControl) -> &'static str {
    match value {
        PhaseControl::Auto => "auto",
        PhaseControl::Manual => "manual",
    }
}

fn anomaly_name(value: SimAnomaly) -> &'static str {
    match value {
        SimAnomaly::GpsDropout => "gps-dropout",
        SimAnomaly::ImuBiasSpike => "imu-bias",
        SimAnomaly::BatterySag => "battery-sag",
        SimAnomaly::ThermalRunaway => "thermal-runaway",
        SimAnomaly::CommLoss => "comm-loss",
        SimAnomaly::PayloadFault => "payload-fault",
        SimAnomaly::EngineFailure => "engine-failure",
        SimAnomaly::HardLanding => "hard-landing",
    }
}

fn vehicle_mode_name(value: VehicleMode) -> &'static str {
    match value {
        VehicleMode::Boot => "boot",
        VehicleMode::Safe => "safe",
        VehicleMode::Standby => "standby",
        VehicleMode::Operational => "operational",
        VehicleMode::Survival => "survival",
        VehicleMode::Fault => "fault",
    }
}

fn power_mode_name(value: PowerMode) -> &'static str {
    match value {
        PowerMode::Nominal => "nominal",
        PowerMode::LoadShedding => "load_shedding",
        PowerMode::Survival => "survival",
    }
}

fn thermal_mode_name(value: ThermalMode) -> &'static str {
    match value {
        ThermalMode::Nominal => "nominal",
        ThermalMode::Heating => "heating",
        ThermalMode::Cooling => "cooling",
        ThermalMode::Survival => "survival",
    }
}

fn payload_mode_name(value: PayloadMode) -> &'static str {
    match value {
        PayloadMode::Idle => "idle",
        PayloadMode::Configuring => "configuring",
        PayloadMode::Active => "active",
        PayloadMode::Fault => "fault",
    }
}

fn link_status_name(value: communications_template::LinkStatus) -> &'static str {
    match value {
        communications_template::LinkStatus::Offline => "offline",
        communications_template::LinkStatus::Standby => "standby",
        communications_template::LinkStatus::Online => "online",
        communications_template::LinkStatus::Degraded => "degraded",
    }
}

fn flight_computer_mode_name(value: FlightComputerMode) -> &'static str {
    match value {
        FlightComputerMode::Standby => "standby",
        FlightComputerMode::Active => "active",
        FlightComputerMode::Safe => "safe",
        FlightComputerMode::Failed => "failed",
    }
}

#[cfg(test)]
mod tests {
    use super::{parse_command_json, schema_json, snapshot_json, HttpSimServer};
    use crate::{FullStackSim, SimConfig, SimulatorCommand};

    #[test]
    fn parses_step_command_json() {
        let command = parse_command_json(r#"{"command":"step","count":4}"#).expect("parse");
        assert_eq!(command, SimulatorCommand::Step(4));
    }

    #[test]
    fn snapshot_json_contains_phase() {
        let sim = FullStackSim::new(SimConfig::default()).expect("build sim");
        let snapshot = snapshot_json(&sim.snapshot());
        assert!(snapshot.contains("\"api_version\":\"2026-03-15\""));
        assert!(snapshot.contains("\"schema_version\":\"1.0.0\""));
        assert!(snapshot.contains("\"phase\":\"pad\""));
        assert!(snapshot.contains("\"telemetry\":{\"vehicle\":"));
        assert!(snapshot.contains("\"raw\":{\"vehicle_mode_line\""));
    }

    #[test]
    fn schema_exposes_command_route() {
        let schema = schema_json();
        assert!(schema.contains("\"command_catalog\""));
        assert!(schema.contains("\"snapshot_stream_event\":\"snapshot\""));
        assert!(schema.contains("\"POST /command\""));
        assert!(schema.contains("\"GET /events\""));
    }

    #[test]
    fn server_shares_sim_handle() {
        let server = HttpSimServer::new(FullStackSim::new(SimConfig::default()).expect("build sim"));
        let shared = server.shared_sim();
        assert!(shared.lock().is_ok());
    }
}
