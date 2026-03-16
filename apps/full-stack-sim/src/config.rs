// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use std::fs;
use std::path::{Path, PathBuf};

use fsw_sdk_core::SdkError;
use fsw_sdk_runtime::SchedulerConfig;
use simple_gnc::GncConfig;

use crate::{LaunchVector, SimConfig, TargetOffset, VehicleModelConfig};

#[derive(Debug, Clone, PartialEq)]
pub struct ScenarioConfig {
    pub vehicle_config: String,
    pub step_ms: u64,
    pub ekf_rate_hz: u32,
    pub heartbeat_timeout_ms: u64,
    pub launch_azimuth_deg: f64,
    pub launch_elevation_deg: f64,
    pub launch_altitude_m: f64,
    pub target_offset_east_m: f64,
    pub target_offset_north_m: f64,
    pub target_offset_altitude_m: f64,
    pub controlled_flight_enabled: bool,
}

#[derive(Debug, Clone, PartialEq)]
pub struct VehicleConfig {
    pub reference_area_m2: f64,
    pub reference_span_m: f64,
    pub reference_chord_m: f64,
    pub inertia_roll_scale: f64,
    pub inertia_pitch_scale: f64,
    pub inertia_yaw_scale: f64,
    pub drag_base: f64,
    pub drag_alpha2: f64,
    pub drag_beta2: f64,
    pub drag_mach_gain: f64,
    pub side_force_beta: f64,
    pub side_force_yaw_surface: f64,
    pub lift_alpha: f64,
    pub lift_pitch_surface: f64,
    pub roll_moment_beta: f64,
    pub roll_damping_p: f64,
    pub roll_surface_gain: f64,
    pub pitch_moment_alpha: f64,
    pub pitch_damping_q: f64,
    pub pitch_surface_gain: f64,
    pub yaw_moment_beta: f64,
    pub yaw_damping_r: f64,
    pub yaw_surface_gain: f64,
    pub launch_thrust_axis_factor: f64,
    pub flight_thrust_axis_factor: f64,
    pub landing_thrust_axis_factor: f64,
    pub axial_damping_gain: f64,
    pub lateral_damping_gain: f64,
    pub vertical_damping_gain: f64,
    pub max_body_rate_rps: f64,
    pub max_body_accel_mps2: f64,
    pub max_angular_accel_rps2: f64,
}

impl ScenarioConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, SdkError> {
        let contents = fs::read_to_string(path).map_err(|_| SdkError::InvalidConfig)?;
        Self::parse(&contents)
    }

    pub fn parse(contents: &str) -> Result<Self, SdkError> {
        let mut config = Self::default();

        for raw_line in contents.lines() {
            let line = raw_line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }

            let (key, value) = line.split_once('=').ok_or(SdkError::InvalidConfig)?;
            let key = key.trim();
            let value = value.trim().trim_matches('"');

            match key {
                "vehicle_config" => config.vehicle_config = value.to_string(),
                "step_ms" => config.step_ms = parse_u64(value)?,
                "ekf_rate_hz" => config.ekf_rate_hz = parse_u32(value)?,
                "heartbeat_timeout_ms" => config.heartbeat_timeout_ms = parse_u64(value)?,
                "launch_azimuth_deg" => config.launch_azimuth_deg = parse_f64(value)?,
                "launch_elevation_deg" => config.launch_elevation_deg = parse_f64(value)?,
                "launch_altitude_m" => config.launch_altitude_m = parse_f64(value)?,
                "target_offset_east_m" => config.target_offset_east_m = parse_f64(value)?,
                "target_offset_north_m" => config.target_offset_north_m = parse_f64(value)?,
                "target_offset_altitude_m" => config.target_offset_altitude_m = parse_f64(value)?,
                "controlled_flight_enabled" => {
                    config.controlled_flight_enabled = parse_bool(value)?
                }
                _ => return Err(SdkError::InvalidConfig),
            }
        }

        if config.vehicle_config.is_empty()
            || config.step_ms == 0
            || config.ekf_rate_hz == 0
            || config.heartbeat_timeout_ms == 0
        {
            return Err(SdkError::InvalidConfig);
        }

        Ok(config)
    }
}

impl Default for ScenarioConfig {
    fn default() -> Self {
        let sim = SimConfig::default();
        Self {
            vehicle_config: "../vehicles/single-string-default.toml".to_string(),
            step_ms: sim.step_ms,
            ekf_rate_hz: sim.gnc.ekf_rate_hz,
            heartbeat_timeout_ms: sim.scheduler.heartbeat_timeout_ms,
            launch_azimuth_deg: sim.launch_vector.azimuth_deg,
            launch_elevation_deg: sim.launch_vector.elevation_deg,
            launch_altitude_m: sim.launch_vector.altitude_m,
            target_offset_east_m: sim.target_offset.east_m,
            target_offset_north_m: sim.target_offset.north_m,
            target_offset_altitude_m: sim.target_offset.altitude_m,
            controlled_flight_enabled: sim.controlled_flight_enabled,
        }
    }
}

impl VehicleConfig {
    pub fn load(path: impl AsRef<Path>) -> Result<Self, SdkError> {
        let contents = fs::read_to_string(path).map_err(|_| SdkError::InvalidConfig)?;
        Self::parse(&contents)
    }

    pub fn parse(contents: &str) -> Result<Self, SdkError> {
        let mut config = Self::default();

        for raw_line in contents.lines() {
            let line = raw_line.split('#').next().unwrap_or("").trim();
            if line.is_empty() {
                continue;
            }

            let (key, value) = line.split_once('=').ok_or(SdkError::InvalidConfig)?;
            let key = key.trim();
            let value = value.trim().trim_matches('"');

            match key {
                "reference_area_m2" => config.reference_area_m2 = parse_f64(value)?,
                "reference_span_m" => config.reference_span_m = parse_f64(value)?,
                "reference_chord_m" => config.reference_chord_m = parse_f64(value)?,
                "inertia_roll_scale" => config.inertia_roll_scale = parse_f64(value)?,
                "inertia_pitch_scale" => config.inertia_pitch_scale = parse_f64(value)?,
                "inertia_yaw_scale" => config.inertia_yaw_scale = parse_f64(value)?,
                "drag_base" => config.drag_base = parse_f64(value)?,
                "drag_alpha2" => config.drag_alpha2 = parse_f64(value)?,
                "drag_beta2" => config.drag_beta2 = parse_f64(value)?,
                "drag_mach_gain" => config.drag_mach_gain = parse_f64(value)?,
                "side_force_beta" => config.side_force_beta = parse_f64(value)?,
                "side_force_yaw_surface" => config.side_force_yaw_surface = parse_f64(value)?,
                "lift_alpha" => config.lift_alpha = parse_f64(value)?,
                "lift_pitch_surface" => config.lift_pitch_surface = parse_f64(value)?,
                "roll_moment_beta" => config.roll_moment_beta = parse_f64(value)?,
                "roll_damping_p" => config.roll_damping_p = parse_f64(value)?,
                "roll_surface_gain" => config.roll_surface_gain = parse_f64(value)?,
                "pitch_moment_alpha" => config.pitch_moment_alpha = parse_f64(value)?,
                "pitch_damping_q" => config.pitch_damping_q = parse_f64(value)?,
                "pitch_surface_gain" => config.pitch_surface_gain = parse_f64(value)?,
                "yaw_moment_beta" => config.yaw_moment_beta = parse_f64(value)?,
                "yaw_damping_r" => config.yaw_damping_r = parse_f64(value)?,
                "yaw_surface_gain" => config.yaw_surface_gain = parse_f64(value)?,
                "launch_thrust_axis_factor" => config.launch_thrust_axis_factor = parse_f64(value)?,
                "flight_thrust_axis_factor" => config.flight_thrust_axis_factor = parse_f64(value)?,
                "landing_thrust_axis_factor" => {
                    config.landing_thrust_axis_factor = parse_f64(value)?
                }
                "axial_damping_gain" => config.axial_damping_gain = parse_f64(value)?,
                "lateral_damping_gain" => config.lateral_damping_gain = parse_f64(value)?,
                "vertical_damping_gain" => config.vertical_damping_gain = parse_f64(value)?,
                "max_body_rate_rps" => config.max_body_rate_rps = parse_f64(value)?,
                "max_body_accel_mps2" => config.max_body_accel_mps2 = parse_f64(value)?,
                "max_angular_accel_rps2" => config.max_angular_accel_rps2 = parse_f64(value)?,
                _ => return Err(SdkError::InvalidConfig),
            }
        }

        if config.reference_area_m2 <= 0.0
            || config.reference_span_m <= 0.0
            || config.reference_chord_m <= 0.0
            || config.max_body_rate_rps <= 0.0
            || config.max_body_accel_mps2 <= 0.0
            || config.max_angular_accel_rps2 <= 0.0
        {
            return Err(SdkError::InvalidConfig);
        }

        Ok(config)
    }

    #[must_use]
    pub fn vehicle_model(&self) -> VehicleModelConfig {
        VehicleModelConfig {
            reference_area_m2: self.reference_area_m2,
            reference_span_m: self.reference_span_m,
            reference_chord_m: self.reference_chord_m,
            inertia_roll_scale: self.inertia_roll_scale,
            inertia_pitch_scale: self.inertia_pitch_scale,
            inertia_yaw_scale: self.inertia_yaw_scale,
            drag_base: self.drag_base,
            drag_alpha2: self.drag_alpha2,
            drag_beta2: self.drag_beta2,
            drag_mach_gain: self.drag_mach_gain,
            side_force_beta: self.side_force_beta,
            side_force_yaw_surface: self.side_force_yaw_surface,
            lift_alpha: self.lift_alpha,
            lift_pitch_surface: self.lift_pitch_surface,
            roll_moment_beta: self.roll_moment_beta,
            roll_damping_p: self.roll_damping_p,
            roll_surface_gain: self.roll_surface_gain,
            pitch_moment_alpha: self.pitch_moment_alpha,
            pitch_damping_q: self.pitch_damping_q,
            pitch_surface_gain: self.pitch_surface_gain,
            yaw_moment_beta: self.yaw_moment_beta,
            yaw_damping_r: self.yaw_damping_r,
            yaw_surface_gain: self.yaw_surface_gain,
            launch_thrust_axis_factor: self.launch_thrust_axis_factor,
            flight_thrust_axis_factor: self.flight_thrust_axis_factor,
            landing_thrust_axis_factor: self.landing_thrust_axis_factor,
            axial_damping_gain: self.axial_damping_gain,
            lateral_damping_gain: self.lateral_damping_gain,
            vertical_damping_gain: self.vertical_damping_gain,
            max_body_rate_rps: self.max_body_rate_rps,
            max_body_accel_mps2: self.max_body_accel_mps2,
            max_angular_accel_rps2: self.max_angular_accel_rps2,
        }
    }
}

impl Default for VehicleConfig {
    fn default() -> Self {
        let vehicle = SimConfig::default().vehicle;
        Self {
            reference_area_m2: vehicle.reference_area_m2,
            reference_span_m: vehicle.reference_span_m,
            reference_chord_m: vehicle.reference_chord_m,
            inertia_roll_scale: vehicle.inertia_roll_scale,
            inertia_pitch_scale: vehicle.inertia_pitch_scale,
            inertia_yaw_scale: vehicle.inertia_yaw_scale,
            drag_base: vehicle.drag_base,
            drag_alpha2: vehicle.drag_alpha2,
            drag_beta2: vehicle.drag_beta2,
            drag_mach_gain: vehicle.drag_mach_gain,
            side_force_beta: vehicle.side_force_beta,
            side_force_yaw_surface: vehicle.side_force_yaw_surface,
            lift_alpha: vehicle.lift_alpha,
            lift_pitch_surface: vehicle.lift_pitch_surface,
            roll_moment_beta: vehicle.roll_moment_beta,
            roll_damping_p: vehicle.roll_damping_p,
            roll_surface_gain: vehicle.roll_surface_gain,
            pitch_moment_alpha: vehicle.pitch_moment_alpha,
            pitch_damping_q: vehicle.pitch_damping_q,
            pitch_surface_gain: vehicle.pitch_surface_gain,
            yaw_moment_beta: vehicle.yaw_moment_beta,
            yaw_damping_r: vehicle.yaw_damping_r,
            yaw_surface_gain: vehicle.yaw_surface_gain,
            launch_thrust_axis_factor: vehicle.launch_thrust_axis_factor,
            flight_thrust_axis_factor: vehicle.flight_thrust_axis_factor,
            landing_thrust_axis_factor: vehicle.landing_thrust_axis_factor,
            axial_damping_gain: vehicle.axial_damping_gain,
            lateral_damping_gain: vehicle.lateral_damping_gain,
            vertical_damping_gain: vehicle.vertical_damping_gain,
            max_body_rate_rps: vehicle.max_body_rate_rps,
            max_body_accel_mps2: vehicle.max_body_accel_mps2,
            max_angular_accel_rps2: vehicle.max_angular_accel_rps2,
        }
    }
}

pub fn load_sim_config(path: impl AsRef<Path>) -> Result<SimConfig, SdkError> {
    let scenario_path = path.as_ref();
    let scenario = ScenarioConfig::load(scenario_path)?;
    let vehicle_path = resolve_relative_path(scenario_path, &scenario.vehicle_config);
    let vehicle = VehicleConfig::load(vehicle_path)?;

    Ok(SimConfig {
        step_ms: scenario.step_ms,
        scheduler: SchedulerConfig {
            heartbeat_timeout_ms: scenario.heartbeat_timeout_ms,
            ..SchedulerConfig::default()
        },
        gnc: GncConfig {
            ekf_rate_hz: scenario.ekf_rate_hz,
            ..GncConfig::default()
        },
        vehicle: vehicle.vehicle_model(),
        launch_vector: LaunchVector {
            azimuth_deg: scenario.launch_azimuth_deg,
            elevation_deg: scenario.launch_elevation_deg,
            altitude_m: scenario.launch_altitude_m,
        },
        target_offset: TargetOffset {
            east_m: scenario.target_offset_east_m,
            north_m: scenario.target_offset_north_m,
            altitude_m: scenario.target_offset_altitude_m,
        },
        controlled_flight_enabled: scenario.controlled_flight_enabled,
    })
}

fn resolve_relative_path(base_path: &Path, target: &str) -> PathBuf {
    let target_path = Path::new(target);
    if target_path.is_absolute() {
        target_path.to_path_buf()
    } else {
        base_path.parent().map_or_else(
            || target_path.to_path_buf(),
            |parent| parent.join(target_path),
        )
    }
}

fn parse_u32(value: &str) -> Result<u32, SdkError> {
    value.parse::<u32>().map_err(|_| SdkError::InvalidConfig)
}

fn parse_u64(value: &str) -> Result<u64, SdkError> {
    value.parse::<u64>().map_err(|_| SdkError::InvalidConfig)
}

fn parse_f64(value: &str) -> Result<f64, SdkError> {
    value.parse::<f64>().map_err(|_| SdkError::InvalidConfig)
}

fn parse_bool(value: &str) -> Result<bool, SdkError> {
    match value.trim().to_ascii_lowercase().as_str() {
        "true" | "1" => Ok(true),
        "false" | "0" => Ok(false),
        _ => Err(SdkError::InvalidConfig),
    }
}

#[cfg(test)]
mod tests {
    use std::fs;
    use std::time::{SystemTime, UNIX_EPOCH};

    use super::{load_sim_config, ScenarioConfig, VehicleConfig};

    #[test]
    fn parses_scenario_file() {
        let config = ScenarioConfig::parse(
            "vehicle_config = ../vehicles/test.toml
step_ms = 40
ekf_rate_hz = 50
heartbeat_timeout_ms = 350
launch_altitude_m = 1200.0
target_offset_east_m = 1025.0
target_offset_north_m = 150.0
target_offset_altitude_m = 80.0
",
        )
        .expect("config");

        assert_eq!(config.vehicle_config, "../vehicles/test.toml");
        assert_eq!(config.step_ms, 40);
        assert_eq!(config.ekf_rate_hz, 50);
        assert_eq!(config.heartbeat_timeout_ms, 350);
        assert_eq!(config.launch_altitude_m, 1200.0);
        assert_eq!(config.target_offset_east_m, 1025.0);
        assert_eq!(config.target_offset_north_m, 150.0);
        assert_eq!(config.target_offset_altitude_m, 80.0);
        assert!(config.controlled_flight_enabled);
    }

    #[test]
    fn parses_controlled_flight_flag() {
        let config = ScenarioConfig::parse(
            "vehicle_config = ../vehicles/custom.toml
controlled_flight_enabled = false
",
        )
        .expect("config");

        assert!(!config.controlled_flight_enabled);
    }

    #[test]
    fn parses_vehicle_file() {
        let config = VehicleConfig::parse(
            "reference_area_m2 = 2.4
drag_base = 0.31
pitch_surface_gain = 1.45
max_body_rate_rps = 3.2
max_body_accel_mps2 = 72.0
max_angular_accel_rps2 = 9.5
",
        )
        .expect("config");

        assert_eq!(config.reference_area_m2, 2.4);
        assert_eq!(config.drag_base, 0.31);
        assert_eq!(config.pitch_surface_gain, 1.45);
    }

    #[test]
    fn scenario_merges_relative_vehicle_file() {
        let unique = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .expect("time")
            .as_nanos();
        let root = std::env::temp_dir().join(format!("astraforge-full-stack-sim-{unique}"));
        let scenarios_dir = root.join("scenarios");
        let vehicles_dir = root.join("vehicles");
        fs::create_dir_all(&scenarios_dir).expect("scenarios dir");
        fs::create_dir_all(&vehicles_dir).expect("vehicles dir");

        let scenario_path = scenarios_dir.join("default.toml");
        let vehicle_path = vehicles_dir.join("custom.toml");

        fs::write(
            &scenario_path,
            "vehicle_config = ../vehicles/custom.toml
step_ms = 80
ekf_rate_hz = 30
heartbeat_timeout_ms = 500
",
        )
        .expect("write scenario");
        fs::write(
            &vehicle_path,
            "reference_area_m2 = 2.75
reference_span_m = 3.0
reference_chord_m = 0.95
pitch_surface_gain = 1.6
max_body_rate_rps = 3.0
max_body_accel_mps2 = 68.0
max_angular_accel_rps2 = 9.0
",
        )
        .expect("write vehicle");

        let sim = load_sim_config(&scenario_path).expect("load config");
        assert_eq!(sim.step_ms, 80);
        assert_eq!(sim.gnc.ekf_rate_hz, 30);
        assert_eq!(sim.scheduler.heartbeat_timeout_ms, 500);
        assert_eq!(sim.vehicle.reference_area_m2, 2.75);
        assert_eq!(sim.vehicle.pitch_surface_gain, 1.6);

        let _ = fs::remove_dir_all(root);
    }

    #[test]
    fn parses_launch_direction() {
        let config = ScenarioConfig::parse(
            "vehicle_config = ../vehicles/custom.toml
step_ms = 50
ekf_rate_hz = 40
heartbeat_timeout_ms = 300
launch_azimuth_deg = 135.0
launch_elevation_deg = 12.5
launch_altitude_m = 420.0
",
        )
        .expect("config");

        assert_eq!(config.launch_azimuth_deg, 135.0);
        assert_eq!(config.launch_elevation_deg, 12.5);
        assert_eq!(config.launch_altitude_m, 420.0);
    }
}
