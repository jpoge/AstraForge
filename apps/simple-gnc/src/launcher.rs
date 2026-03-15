// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{DurationMs, MessageBus, SdkError, TimeSource};
use fsw_sdk_fdir::ClassMapPolicy;
use fsw_sdk_hal::{HalPlatform, HostSimPlatform};
use fsw_sdk_runtime::{Runtime, SchedulerConfig};

use crate::{
    new_shared_gnc_component, GpsReading, ImuReading, MagnetometerReading, MissionConfig,
    NavigationSolution, GNC_COMPONENT_ID, GNC_SOLUTION_TOPIC,
};

/// Summary returned by the mission launcher.
#[derive(Debug, Clone, PartialEq)]
pub struct LaunchReport {
    pub final_solution: NavigationSolution,
    pub last_topic_payload: Option<String>,
    pub platform: String,
}

/// Runs the configured GNC mission on the selected platform backend.
pub fn run_mission(config: &MissionConfig) -> Result<LaunchReport, SdkError> {
    match config.platform.as_str() {
        "host-sim" => run_host_sim(config),
        _ => Err(SdkError::InvalidConfig),
    }
}

fn run_host_sim(config: &MissionConfig) -> Result<LaunchReport, SdkError> {
    let platform = HostSimPlatform::new();
    let (component, proxy) = new_shared_gnc_component(config.gnc_config());

    let policy = ClassMapPolicy::conservative();
    let mut runtime = Runtime::new(policy);
    runtime.set_scheduler_config(SchedulerConfig {
        period_ms: config.imu_period_ms,
        heartbeat_timeout_ms: config.heartbeat_timeout_ms,
    });
    runtime.register_with_manifest(Box::new(proxy), include_str!("../module.yaml"))?;
    runtime.start_all()?;

    let ctx = AppContext {
        bus: platform.message_bus(),
        time: platform.time_source(),
    };

    let mut truth_position = [0.0_f64, 0.0_f64, 0.0_f64];
    let mut truth_velocity = [0.0_f64, 0.0_f64, 0.0_f64];
    let mut truth_attitude = [0.0_f64, 0.0_f64, 0.0_f64];
    let nav_magnetic_field = [0.8_f64, 0.1_f64, -0.2_f64];
    let steps = ((config.sim_duration_s * 1000.0) / config.imu_period_ms as f64).round() as u32;

    {
        let mut app = component.lock().map_err(|_| SdkError::BackendFailure)?;
        app.init(&ctx)?;
    }

    for step in 0..steps {
        platform.clock().tick(DurationMs(config.imu_period_ms));
        let now = platform.clock().now_monotonic();
        let dt_s = config.imu_period_ms as f64 / 1000.0;

        let body_accel = if step < steps / 3 {
            [0.25, 0.02, -0.01]
        } else if step < (2 * steps) / 3 {
            [0.05, -0.03, 0.02]
        } else {
            [0.0, 0.0, 0.0]
        };
        let body_rates = if step < steps / 2 {
            [0.01, -0.006, 0.02]
        } else {
            [0.0, 0.004, 0.0]
        };

        truth_attitude[0] += body_rates[0] * dt_s;
        truth_attitude[1] += body_rates[1] * dt_s;
        truth_attitude[2] += body_rates[2] * dt_s;

        let rotation = rotation_body_to_nav(truth_attitude);
        let nav_accel = multiply_matrix_vec3(&rotation, &body_accel);
        for axis in 0..3 {
            truth_velocity[axis] += nav_accel[axis] * dt_s;
            truth_position[axis] += truth_velocity[axis] * dt_s;
        }
        let magnetic_field_body =
            multiply_matrix_vec3(&transpose_matrix3(&rotation), &nav_magnetic_field);

        {
            let mut app = component.lock().map_err(|_| SdkError::BackendFailure)?;
            app.submit_imu(ImuReading {
                timestamp: now,
                accel_body_mps2: body_accel,
                gyro_body_rps: body_rates,
            });

            if now.0 % config.gps_period_ms == 0 {
                app.submit_gps(GpsReading {
                    timestamp: now,
                    position_m: truth_position,
                    velocity_mps: truth_velocity,
                });
            }

            if now.0 % config.magnetometer_period_ms == 0 {
                app.submit_magnetometer(MagnetometerReading {
                    timestamp: now,
                    magnetic_field_body,
                });
            }

            app.step(&ctx)?;
        }

        runtime.mark_heartbeat(&fsw_sdk_core::ComponentId(GNC_COMPONENT_ID.to_string()), now)?;
        let _responses = runtime.supervisor_step(now)?;
    }

    let last_topic_payload = platform
        .bus()
        .receive(&GNC_SOLUTION_TOPIC)?
        .map(|payload| String::from_utf8_lossy(&payload).to_string());

    let final_solution = component
        .lock()
        .map_err(|_| SdkError::BackendFailure)?
        .solution();

    {
        let mut app = component.lock().map_err(|_| SdkError::BackendFailure)?;
        app.shutdown(&ctx)?;
    }
    runtime.stop_all()?;

    Ok(LaunchReport {
        final_solution,
        last_topic_payload,
        platform: config.platform.clone(),
    })
}

fn rotation_body_to_nav(attitude: [f64; 3]) -> [[f64; 3]; 3] {
    let (roll, pitch, yaw) = (attitude[0], attitude[1], attitude[2]);
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let (sy, cy) = yaw.sin_cos();
    [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]
}

fn transpose_matrix3(matrix: &[[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]
}

fn multiply_matrix_vec3(matrix: &[[f64; 3]; 3], vector: &[f64; 3]) -> [f64; 3] {
    let mut result = [0.0; 3];
    for row in 0..3 {
        for col in 0..3 {
            result[row] += matrix[row][col] * vector[col];
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::run_mission;
    use crate::MissionConfig;

    #[test]
    fn host_sim_launcher_runs_from_config() {
        let config = MissionConfig {
            sim_duration_s: 0.5,
            ..MissionConfig::default()
        };

        let report = run_mission(&config).expect("launch");
        assert_eq!(report.platform, "host-sim");
        assert!(report.last_topic_payload.is_some());
        assert!(report.final_solution.timestamp.0 > 0);
    }
}
