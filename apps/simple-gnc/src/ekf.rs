// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_core::MissionTime;

const STATE_DIM: usize = 15;
const IDX_X: usize = 0;
const IDX_Y: usize = 1;
const IDX_Z: usize = 2;
const IDX_VX: usize = 3;
const IDX_VY: usize = 4;
const IDX_VZ: usize = 5;
const IDX_ROLL: usize = 6;
const IDX_PITCH: usize = 7;
const IDX_YAW: usize = 8;
const IDX_GYRO_BIAS_X: usize = 9;
const IDX_GYRO_BIAS_Y: usize = 10;
const IDX_GYRO_BIAS_Z: usize = 11;
const IDX_ACCEL_BIAS_X: usize = 12;
const IDX_ACCEL_BIAS_Y: usize = 13;
const IDX_ACCEL_BIAS_Z: usize = 14;

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct EkfTuning {
    pub process_accel_noise: f64,
    pub process_gyro_noise: f64,
    pub process_gyro_bias_noise: f64,
    pub process_accel_bias_noise: f64,
    pub gps_position_noise: f64,
    pub gps_velocity_noise: f64,
    pub magnetometer_heading_noise: f64,
}

impl Default for EkfTuning {
    fn default() -> Self {
        Self {
            process_accel_noise: 0.35,
            process_gyro_noise: 0.04,
            process_gyro_bias_noise: 0.002,
            process_accel_bias_noise: 0.01,
            gps_position_noise: 2.5,
            gps_velocity_noise: 0.35,
            magnetometer_heading_noise: 0.12,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImuReading {
    pub timestamp: MissionTime,
    pub accel_body_mps2: [f64; 3],
    pub gyro_body_rps: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GpsReading {
    pub timestamp: MissionTime,
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct MagnetometerReading {
    pub timestamp: MissionTime,
    pub magnetic_field_body: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct NavigationSolution {
    pub timestamp: MissionTime,
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
    pub attitude_rad: [f64; 3],
    pub gyro_bias_rps: [f64; 3],
    pub accel_bias_mps2: [f64; 3],
}

#[derive(Debug, Clone)]
pub struct ExtendedKalmanFilter {
    state: [f64; STATE_DIM],
    covariance: [[f64; STATE_DIM]; STATE_DIM],
    tuning: EkfTuning,
    solution_time: MissionTime,
}

impl ExtendedKalmanFilter {
    #[must_use]
    pub fn new() -> Self {
        Self::with_tuning(EkfTuning::default())
    }

    #[must_use]
    pub fn with_tuning(tuning: EkfTuning) -> Self {
        Self {
            state: [0.0; STATE_DIM],
            covariance: diagonal_matrix([
                25.0, 25.0, 25.0, 9.0, 9.0, 9.0, 0.5, 0.5, 1.0, 0.1, 0.1, 0.1, 0.2, 0.2, 0.2,
            ]),
            tuning,
            solution_time: MissionTime(0),
        }
    }

    pub fn reset(&mut self) {
        *self = Self::with_tuning(self.tuning);
    }

    pub fn predict(&mut self, imu: &ImuReading, dt_s: f64) {
        let attitude = [
            self.state[IDX_ROLL],
            self.state[IDX_PITCH],
            self.state[IDX_YAW],
        ];
        let body_to_nav = rotation_body_to_nav(attitude);
        let accel_body = subtract_vec3(
            imu.accel_body_mps2,
            [
                self.state[IDX_ACCEL_BIAS_X],
                self.state[IDX_ACCEL_BIAS_Y],
                self.state[IDX_ACCEL_BIAS_Z],
            ],
        );
        let gyro_body = subtract_vec3(
            imu.gyro_body_rps,
            [
                self.state[IDX_GYRO_BIAS_X],
                self.state[IDX_GYRO_BIAS_Y],
                self.state[IDX_GYRO_BIAS_Z],
            ],
        );
        let accel_nav = multiply_matrix_vec3(&body_to_nav, &accel_body);
        let euler_rates = body_rates_to_euler_rates(attitude, gyro_body);

        self.state[IDX_X] += self.state[IDX_VX] * dt_s + 0.5 * accel_nav[0] * dt_s * dt_s;
        self.state[IDX_Y] += self.state[IDX_VY] * dt_s + 0.5 * accel_nav[1] * dt_s * dt_s;
        self.state[IDX_Z] += self.state[IDX_VZ] * dt_s + 0.5 * accel_nav[2] * dt_s * dt_s;
        self.state[IDX_VX] += accel_nav[0] * dt_s;
        self.state[IDX_VY] += accel_nav[1] * dt_s;
        self.state[IDX_VZ] += accel_nav[2] * dt_s;
        self.state[IDX_ROLL] = wrap_angle(self.state[IDX_ROLL] + euler_rates[0] * dt_s);
        self.state[IDX_PITCH] = wrap_angle(self.state[IDX_PITCH] + euler_rates[1] * dt_s);
        self.state[IDX_YAW] = wrap_angle(self.state[IDX_YAW] + euler_rates[2] * dt_s);

        let mut f = identity_matrix();
        f[IDX_X][IDX_VX] = dt_s;
        f[IDX_Y][IDX_VY] = dt_s;
        f[IDX_Z][IDX_VZ] = dt_s;
        for axis in 0..3 {
            f[IDX_VX + axis][IDX_ACCEL_BIAS_X + axis] = -dt_s;
            f[IDX_X + axis][IDX_ACCEL_BIAS_X + axis] = -0.5 * dt_s * dt_s;
            f[IDX_ROLL + axis][IDX_GYRO_BIAS_X + axis] = -dt_s;
        }

        let accel_var = self.tuning.process_accel_noise * self.tuning.process_accel_noise;
        let gyro_var = self.tuning.process_gyro_noise * self.tuning.process_gyro_noise;
        let gyro_bias_var =
            self.tuning.process_gyro_bias_noise * self.tuning.process_gyro_bias_noise;
        let accel_bias_var =
            self.tuning.process_accel_bias_noise * self.tuning.process_accel_bias_noise;
        let dt2 = dt_s * dt_s;
        let dt4 = dt2 * dt2;
        let mut q = [[0.0; STATE_DIM]; STATE_DIM];
        for axis in 0..3 {
            q[IDX_X + axis][IDX_X + axis] = 0.25 * accel_var * dt4;
            q[IDX_VX + axis][IDX_VX + axis] = accel_var * dt2;
            q[IDX_ROLL + axis][IDX_ROLL + axis] = gyro_var * dt2;
            q[IDX_GYRO_BIAS_X + axis][IDX_GYRO_BIAS_X + axis] =
                gyro_bias_var * dt_s.max(1.0e-3);
            q[IDX_ACCEL_BIAS_X + axis][IDX_ACCEL_BIAS_X + axis] =
                accel_bias_var * dt_s.max(1.0e-3);
        }

        self.covariance = add_matrix(&multiply_fpf_t(&f, &self.covariance), &q);
        self.solution_time = imu.timestamp;
    }

    pub fn update_gps(&mut self, gps: &GpsReading) {
        let position_var = self.tuning.gps_position_noise * self.tuning.gps_position_noise;
        let velocity_var = self.tuning.gps_velocity_noise * self.tuning.gps_velocity_noise;
        for axis in 0..3 {
            self.update_scalar(IDX_X + axis, gps.position_m[axis], position_var, false);
            self.update_scalar(IDX_VX + axis, gps.velocity_mps[axis], velocity_var, false);
        }
        self.solution_time = gps.timestamp;
    }

    pub fn update_magnetometer(&mut self, mag: &MagnetometerReading) {
        let heading = heading_from_magnetometer(
            mag.magnetic_field_body,
            [self.state[IDX_ROLL], self.state[IDX_PITCH], self.state[IDX_YAW]],
        );
        self.update_scalar(
            IDX_YAW,
            heading,
            self.tuning.magnetometer_heading_noise * self.tuning.magnetometer_heading_noise,
            true,
        );
        self.solution_time = mag.timestamp;
    }

    #[must_use]
    pub fn solution(&self) -> NavigationSolution {
        NavigationSolution {
            timestamp: self.solution_time,
            position_m: [self.state[IDX_X], self.state[IDX_Y], self.state[IDX_Z]],
            velocity_mps: [self.state[IDX_VX], self.state[IDX_VY], self.state[IDX_VZ]],
            attitude_rad: [self.state[IDX_ROLL], self.state[IDX_PITCH], self.state[IDX_YAW]],
            gyro_bias_rps: [
                self.state[IDX_GYRO_BIAS_X],
                self.state[IDX_GYRO_BIAS_Y],
                self.state[IDX_GYRO_BIAS_Z],
            ],
            accel_bias_mps2: [
                self.state[IDX_ACCEL_BIAS_X],
                self.state[IDX_ACCEL_BIAS_Y],
                self.state[IDX_ACCEL_BIAS_Z],
            ],
        }
    }

    fn update_scalar(
        &mut self,
        measurement_index: usize,
        measurement: f64,
        variance: f64,
        wrap: bool,
    ) {
        let predicted = self.state[measurement_index];
        let innovation = if wrap {
            wrap_angle(measurement - predicted)
        } else {
            measurement - predicted
        };
        let s = self.covariance[measurement_index][measurement_index] + variance;
        if s <= f64::EPSILON {
            return;
        }

        let mut kalman_gain = [0.0; STATE_DIM];
        let mut h_covariance = [0.0; STATE_DIM];
        for index in 0..STATE_DIM {
            kalman_gain[index] = self.covariance[index][measurement_index] / s;
            h_covariance[index] = self.covariance[measurement_index][index];
        }

        for index in 0..STATE_DIM {
            self.state[index] += kalman_gain[index] * innovation;
        }
        self.state[IDX_ROLL] = wrap_angle(self.state[IDX_ROLL]);
        self.state[IDX_PITCH] = wrap_angle(self.state[IDX_PITCH]);
        self.state[IDX_YAW] = wrap_angle(self.state[IDX_YAW]);

        let mut updated = self.covariance;
        for row in 0..STATE_DIM {
            for col in 0..STATE_DIM {
                updated[row][col] -= kalman_gain[row] * h_covariance[col];
            }
        }

        for row in 0..STATE_DIM {
            for col in 0..STATE_DIM {
                self.covariance[row][col] = 0.5 * (updated[row][col] + updated[col][row]);
            }
        }
    }
}

impl Default for ExtendedKalmanFilter {
    fn default() -> Self {
        Self::new()
    }
}

fn wrap_angle(angle: f64) -> f64 {
    let two_pi = 2.0 * core::f64::consts::PI;
    let wrapped = (angle + core::f64::consts::PI).rem_euclid(two_pi) - core::f64::consts::PI;
    if wrapped == -core::f64::consts::PI {
        core::f64::consts::PI
    } else {
        wrapped
    }
}

fn subtract_vec3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[0] - right[0],
        left[1] - right[1],
        left[2] - right[2],
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

fn body_rates_to_euler_rates(attitude: [f64; 3], body_rates: [f64; 3]) -> [f64; 3] {
    let roll = attitude[0];
    let pitch = attitude[1].clamp(-1.55, 1.55);
    let pitch_cos = pitch.cos();
    let pitch_cos_safe = if pitch_cos.abs() < 1.0e-3 {
        if pitch_cos.is_sign_negative() {
            -1.0e-3
        } else {
            1.0e-3
        }
    } else {
        pitch_cos
    };
    let (sr, cr) = roll.sin_cos();
    let tp = pitch.tan();
    let p = body_rates[0];
    let q = body_rates[1];
    let r = body_rates[2];
    [
        p + sr * tp * q + cr * tp * r,
        cr * q - sr * r,
        (sr / pitch_cos_safe) * q + (cr / pitch_cos_safe) * r,
    ]
}

fn heading_from_magnetometer(magnetic_field_body: [f64; 3], attitude: [f64; 3]) -> f64 {
    let roll = attitude[0];
    let pitch = attitude[1];
    let (sr, cr) = roll.sin_cos();
    let (sp, cp) = pitch.sin_cos();
    let xh = magnetic_field_body[0] * cp
        + magnetic_field_body[1] * sr * sp
        + magnetic_field_body[2] * cr * sp;
    let yh = magnetic_field_body[1] * cr - magnetic_field_body[2] * sr;
    wrap_angle((-yh).atan2(xh))
}

fn identity_matrix() -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut matrix = [[0.0; STATE_DIM]; STATE_DIM];
    for (index, row) in matrix.iter_mut().enumerate() {
        row[index] = 1.0;
    }
    matrix
}

fn diagonal_matrix(diagonal: [f64; STATE_DIM]) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut matrix = [[0.0; STATE_DIM]; STATE_DIM];
    for (index, value) in diagonal.into_iter().enumerate() {
        matrix[index][index] = value;
    }
    matrix
}

fn multiply_fpf_t(
    f: &[[f64; STATE_DIM]; STATE_DIM],
    p: &[[f64; STATE_DIM]; STATE_DIM],
) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut fp = [[0.0; STATE_DIM]; STATE_DIM];
    for row in 0..STATE_DIM {
        for col in 0..STATE_DIM {
            for inner in 0..STATE_DIM {
                fp[row][col] += f[row][inner] * p[inner][col];
            }
        }
    }

    let mut fpf_t = [[0.0; STATE_DIM]; STATE_DIM];
    for row in 0..STATE_DIM {
        for col in 0..STATE_DIM {
            for inner in 0..STATE_DIM {
                fpf_t[row][col] += fp[row][inner] * f[col][inner];
            }
        }
    }
    fpf_t
}

fn add_matrix(
    left: &[[f64; STATE_DIM]; STATE_DIM],
    right: &[[f64; STATE_DIM]; STATE_DIM],
) -> [[f64; STATE_DIM]; STATE_DIM] {
    let mut result = [[0.0; STATE_DIM]; STATE_DIM];
    for row in 0..STATE_DIM {
        for col in 0..STATE_DIM {
            result[row][col] = left[row][col] + right[row][col];
        }
    }
    result
}

#[cfg(test)]
mod tests {
    use super::{EkfTuning, ExtendedKalmanFilter, GpsReading, ImuReading, MagnetometerReading};
    use fsw_sdk_core::MissionTime;

    #[test]
    fn predict_advances_state_from_imu() {
        let mut ekf = ExtendedKalmanFilter::new();
        let imu = ImuReading {
            timestamp: MissionTime(100),
            accel_body_mps2: [1.0, 0.0, -0.2],
            gyro_body_rps: [0.0, 0.0, 0.1],
        };

        ekf.predict(&imu, 1.0);
        let solution = ekf.solution();

        assert!(solution.position_m[0] > 0.4);
        assert!(solution.velocity_mps[0] > 0.9);
        assert!(solution.position_m[2] < -0.05);
        assert!(solution.attitude_rad[2] > 0.05);
    }

    #[test]
    fn gps_and_magnetometer_pull_solution_toward_measurements() {
        let mut ekf = ExtendedKalmanFilter::with_tuning(EkfTuning {
            gps_position_noise: 1.0,
            gps_velocity_noise: 0.2,
            magnetometer_heading_noise: 0.08,
            ..EkfTuning::default()
        });
        let imu = ImuReading {
            timestamp: MissionTime(100),
            accel_body_mps2: [0.0, 0.0, 0.0],
            gyro_body_rps: [0.05, -0.02, 0.4],
        };
        ekf.predict(&imu, 1.0);
        ekf.update_gps(&GpsReading {
            timestamp: MissionTime(120),
            position_m: [100.0, -30.0, 15.0],
            velocity_mps: [5.0, 1.0, -0.4],
        });
        ekf.update_magnetometer(&MagnetometerReading {
            timestamp: MissionTime(130),
            magnetic_field_body: [0.969, -0.247, 0.0],
        });

        let solution = ekf.solution();
        assert!(solution.position_m[0] > 75.0);
        assert!(solution.position_m[1] < -20.0);
        assert!(solution.position_m[2] > 8.0);
        assert!(solution.velocity_mps[0] > 3.0);
        assert!(solution.attitude_rad[2].abs() < 0.35);
    }
}
