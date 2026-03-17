// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use std::fs;
use std::path::Path;

use fsw_sdk_core::SdkError;

use crate::component::{ControlGains, GncConfig};
use crate::ekf::EkfTuning;

#[derive(Debug, Clone, PartialEq)]
pub struct MissionConfig {
    pub platform: String,
    pub ekf_rate_hz: u32,
    pub process_accel_noise: f64,
    pub process_gyro_noise: f64,
    pub process_gyro_bias_noise: f64,
    pub process_accel_bias_noise: f64,
    pub gps_position_noise: f64,
    pub gps_velocity_noise: f64,
    pub magnetometer_heading_noise: f64,
    pub sim_duration_s: f64,
    pub imu_period_ms: u64,
    pub gps_period_ms: u64,
    pub magnetometer_period_ms: u64,
    pub heartbeat_timeout_ms: u64,
    pub guidance_position_gain: f64,
    pub guidance_velocity_gain: f64,
    pub guidance_yaw_gain: f64,
    pub control_roll_gain: f64,
    pub control_pitch_gain: f64,
    pub control_rate_damping: f64,
}

impl MissionConfig {
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
                "platform" => config.platform = value.to_string(),
                "ekf_rate_hz" => config.ekf_rate_hz = parse_u32(value)?,
                "process_accel_noise" => config.process_accel_noise = parse_f64(value)?,
                "process_gyro_noise" => config.process_gyro_noise = parse_f64(value)?,
                "process_gyro_bias_noise" => config.process_gyro_bias_noise = parse_f64(value)?,
                "process_accel_bias_noise" => config.process_accel_bias_noise = parse_f64(value)?,
                "gps_position_noise" => config.gps_position_noise = parse_f64(value)?,
                "gps_velocity_noise" => config.gps_velocity_noise = parse_f64(value)?,
                "magnetometer_heading_noise" => {
                    config.magnetometer_heading_noise = parse_f64(value)?
                }
                "sim_duration_s" => config.sim_duration_s = parse_f64(value)?,
                "imu_period_ms" => config.imu_period_ms = parse_u64(value)?,
                "gps_period_ms" => config.gps_period_ms = parse_u64(value)?,
                "magnetometer_period_ms" => config.magnetometer_period_ms = parse_u64(value)?,
                "heartbeat_timeout_ms" => config.heartbeat_timeout_ms = parse_u64(value)?,
                "guidance_position_gain" => config.guidance_position_gain = parse_f64(value)?,
                "guidance_velocity_gain" => config.guidance_velocity_gain = parse_f64(value)?,
                "guidance_yaw_gain" => config.guidance_yaw_gain = parse_f64(value)?,
                "control_roll_gain" => config.control_roll_gain = parse_f64(value)?,
                "control_pitch_gain" => config.control_pitch_gain = parse_f64(value)?,
                "control_rate_damping" => config.control_rate_damping = parse_f64(value)?,
                _ => return Err(SdkError::InvalidConfig),
            }
        }

        if config.ekf_rate_hz == 0
            || config.ekf_rate_hz > 1000
            || config.imu_period_ms == 0
            || config.gps_period_ms == 0
            || config.magnetometer_period_ms == 0
            || config.heartbeat_timeout_ms == 0
        {
            return Err(SdkError::InvalidConfig);
        }

        Ok(config)
    }

    #[must_use]
    pub fn gnc_config(&self) -> GncConfig {
        GncConfig {
            ekf_rate_hz: self.ekf_rate_hz,
            tuning: self.ekf_tuning(),
            control: ControlGains {
                position_gain: self.guidance_position_gain,
                velocity_gain: self.guidance_velocity_gain,
                yaw_gain: self.guidance_yaw_gain,
                roll_gain: self.control_roll_gain,
                pitch_gain: self.control_pitch_gain,
                body_rate_damping: self.control_rate_damping,
                ..ControlGains::default()
            },
        }
    }

    #[must_use]
    pub fn ekf_tuning(&self) -> EkfTuning {
        EkfTuning {
            process_accel_noise: self.process_accel_noise,
            process_gyro_noise: self.process_gyro_noise,
            process_gyro_bias_noise: self.process_gyro_bias_noise,
            process_accel_bias_noise: self.process_accel_bias_noise,
            gps_position_noise: self.gps_position_noise,
            gps_velocity_noise: self.gps_velocity_noise,
            magnetometer_heading_noise: self.magnetometer_heading_noise,
        }
    }
}

impl Default for MissionConfig {
    fn default() -> Self {
        Self {
            platform: "host-sim".to_string(),
            ekf_rate_hz: 25,
            process_accel_noise: 0.35,
            process_gyro_noise: 0.04,
            process_gyro_bias_noise: 0.002,
            process_accel_bias_noise: 0.01,
            gps_position_noise: 2.5,
            gps_velocity_noise: 0.35,
            magnetometer_heading_noise: 0.12,
            sim_duration_s: 6.0,
            imu_period_ms: 20,
            gps_period_ms: 200,
            magnetometer_period_ms: 100,
            heartbeat_timeout_ms: 250,
            guidance_position_gain: 0.012,
            guidance_velocity_gain: 0.11,
            guidance_yaw_gain: 0.85,
            control_roll_gain: 1.35,
            control_pitch_gain: 1.1,
            control_rate_damping: 0.35,
        }
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

#[cfg(test)]
mod tests {
    use super::MissionConfig;

    #[test]
    fn parses_file_style_config() {
        let config = MissionConfig::parse(
            "platform = host-sim\nekf_rate_hz = 40\nprocess_accel_noise = 0.5\nimu_period_ms = 10\ngps_period_ms = 100\nmagnetometer_period_ms = 50\nheartbeat_timeout_ms = 300\nsim_duration_s = 3.0\n",
        )
        .expect("config");

        assert_eq!(config.platform, "host-sim");
        assert_eq!(config.ekf_rate_hz, 40);
        assert_eq!(config.process_accel_noise, 0.5);
        assert_eq!(config.imu_period_ms, 10);
        assert_eq!(config.sim_duration_s, 3.0);
    }

    #[test]
    fn rejects_ekf_rates_above_1khz() {
        let result = MissionConfig::parse(
            "platform = host-sim\nekf_rate_hz = 1001\nimu_period_ms = 10\ngps_period_ms = 100\nmagnetometer_period_ms = 50\nheartbeat_timeout_ms = 300\n",
        );

        assert!(result.is_err());
    }
}
