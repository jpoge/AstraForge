// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use std::sync::{Arc, Mutex};

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{
    ComponentHealth, ComponentId, ComponentLifecycleState, DurationMs, FswComponent, MissionTime,
    SdkError, TimeSource, TopicName,
};

use crate::ekf::{
    EkfTuning, ExtendedKalmanFilter, GpsReading, ImuReading, MagnetometerReading,
    NavigationSolution,
};

pub const GNC_COMPONENT_ID: &str = "gnc.app";
pub const GNC_SOLUTION_TOPIC: TopicName = TopicName("gnc.solution");

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GncConfig {
    pub ekf_rate_hz: u32,
    pub tuning: EkfTuning,
}

impl GncConfig {
    #[must_use]
    pub fn ekf_period(&self) -> DurationMs {
        let clamped_rate = self.ekf_rate_hz.max(1);
        DurationMs(1000 / u64::from(clamped_rate))
    }
}

impl Default for GncConfig {
    fn default() -> Self {
        Self {
            ekf_rate_hz: 20,
            tuning: EkfTuning::default(),
        }
    }
}

pub struct GncComponent {
    config: GncConfig,
    ekf: ExtendedKalmanFilter,
    latest_imu: Option<ImuReading>,
    latest_gps: Option<GpsReading>,
    latest_magnetometer: Option<MagnetometerReading>,
    last_predict_time: Option<MissionTime>,
    lifecycle_state: ComponentLifecycleState,
    health: ComponentHealth,
}

impl GncComponent {
    #[must_use]
    pub fn new(config: GncConfig) -> Self {
        Self {
            config,
            ekf: ExtendedKalmanFilter::with_tuning(config.tuning),
            latest_imu: None,
            latest_gps: None,
            latest_magnetometer: None,
            last_predict_time: None,
            lifecycle_state: ComponentLifecycleState::Created,
            health: ComponentHealth::Degraded,
        }
    }

    pub fn submit_imu(&mut self, reading: ImuReading) {
        self.latest_imu = Some(reading);
        self.health = ComponentHealth::Nominal;
    }

    pub fn submit_gps(&mut self, reading: GpsReading) {
        self.latest_gps = Some(reading);
    }

    pub fn submit_magnetometer(&mut self, reading: MagnetometerReading) {
        self.latest_magnetometer = Some(reading);
    }

    #[must_use]
    pub fn solution(&self) -> NavigationSolution {
        self.ekf.solution()
    }

    fn reset_internal(&mut self) {
        self.ekf = ExtendedKalmanFilter::with_tuning(self.config.tuning);
        self.latest_imu = None;
        self.latest_gps = None;
        self.latest_magnetometer = None;
        self.last_predict_time = None;
        self.health = ComponentHealth::Degraded;
    }

    fn encode_solution(solution: &NavigationSolution) -> Vec<u8> {
        format!(
            "t_ms={},x_m={:.3},y_m={:.3},z_m={:.3},vx_mps={:.3},vy_mps={:.3},vz_mps={:.3},roll_rad={:.4},pitch_rad={:.4},yaw_rad={:.4},gyro_bias_xyz_rps=({:.6},{:.6},{:.6})",
            solution.timestamp.0,
            solution.position_m[0],
            solution.position_m[1],
            solution.position_m[2],
            solution.velocity_mps[0],
            solution.velocity_mps[1],
            solution.velocity_mps[2],
            solution.attitude_rad[0],
            solution.attitude_rad[1],
            solution.attitude_rad[2],
            solution.gyro_bias_rps[0],
            solution.gyro_bias_rps[1],
            solution.gyro_bias_rps[2]
        )
        .into_bytes()
    }

    fn step_filter<T: TimeSource + ?Sized>(&mut self, time_source: &T) {
        let period_ms = self.config.ekf_period().0;
        let now = time_source.now_monotonic();
        let Some(imu) = self.latest_imu else {
            self.health = ComponentHealth::Degraded;
            self.last_predict_time.get_or_insert(now);
            return;
        };

        let last_time = self.last_predict_time.get_or_insert(imu.timestamp);
        while now.0.saturating_sub(last_time.0) >= period_ms {
            let next_time = MissionTime(last_time.0 + period_ms);
            let mut propagated_imu = imu;
            propagated_imu.timestamp = next_time;
            self.ekf.predict(&propagated_imu, period_ms as f64 / 1000.0);
            *last_time = next_time;
        }

        if let Some(gps) = self.latest_gps.take() {
            self.ekf.update_gps(&gps);
        }
        if let Some(magnetometer) = self.latest_magnetometer.take() {
            self.ekf.update_magnetometer(&magnetometer);
        }
        self.health = ComponentHealth::Nominal;
    }
}

impl MissionApp for GncComponent {
    fn name(&self) -> &'static str {
        GNC_COMPONENT_ID
    }

    fn init(&mut self, _ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.lifecycle_state = ComponentLifecycleState::Initialized;
        Ok(())
    }

    fn step(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.step_filter(ctx.time);
        let payload = Self::encode_solution(&self.solution());
        ctx.bus.publish(&GNC_SOLUTION_TOPIC, payload)?;
        Ok(())
    }

    fn shutdown(&mut self, _ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.lifecycle_state = ComponentLifecycleState::Stopped;
        Ok(())
    }
}

impl FswComponent for GncComponent {
    fn id(&self) -> ComponentId {
        ComponentId(GNC_COMPONENT_ID.to_string())
    }

    fn dependencies(&self) -> Vec<(ComponentId, fsw_sdk_core::DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        self.lifecycle_state = ComponentLifecycleState::Initialized;
        Ok(())
    }

    fn start(&mut self) -> Result<(), SdkError> {
        self.lifecycle_state = ComponentLifecycleState::Running;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        self.lifecycle_state = ComponentLifecycleState::Stopped;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.reset_internal();
        self.lifecycle_state = ComponentLifecycleState::Initialized;
        Ok(())
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        self.lifecycle_state = ComponentLifecycleState::Created;
        Ok(())
    }

    fn health(&self) -> ComponentHealth {
        self.health
    }
}

pub type SharedGncComponent = Arc<Mutex<GncComponent>>;

pub struct GncComponentProxy {
    inner: SharedGncComponent,
}

impl GncComponentProxy {
    #[must_use]
    pub fn new(inner: SharedGncComponent) -> Self {
        Self { inner }
    }
}

pub fn new_shared_gnc_component(config: GncConfig) -> (SharedGncComponent, GncComponentProxy) {
    let component = Arc::new(Mutex::new(GncComponent::new(config)));
    let proxy = GncComponentProxy::new(Arc::clone(&component));
    (component, proxy)
}

impl FswComponent for GncComponentProxy {
    fn id(&self) -> ComponentId {
        ComponentId(GNC_COMPONENT_ID.to_string())
    }

    fn dependencies(&self) -> Vec<(ComponentId, fsw_sdk_core::DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        let mut component = self
            .inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)?;
        FswComponent::init(&mut *component)
    }

    fn start(&mut self) -> Result<(), SdkError> {
        self.inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)?
            .start()
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        self.inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)?
            .stop()
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)?
            .reset()
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        self.inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)?
            .deinit()
    }

    fn health(&self) -> ComponentHealth {
        self.inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)
            .map_or(ComponentHealth::Failed, |component| component.health())
    }
}

#[cfg(test)]
mod tests {
    use fsw_sdk_app::{AppContext, MissionApp};
    use fsw_sdk_core::{DurationMs, MessageBus, TimeSource};
    use fsw_sdk_runtime::InMemoryBus;
    use fsw_sdk_time::StaticClock;

    use super::{GncComponent, GncConfig, GNC_SOLUTION_TOPIC};
    use crate::{GpsReading, ImuReading, MagnetometerReading};

    #[test]
    fn ekf_respects_configured_rate() {
        let mut app = GncComponent::new(GncConfig {
            ekf_rate_hz: 10,
            ..GncConfig::default()
        });
        let bus = InMemoryBus::new();
        let clock = StaticClock::new();
        let ctx = AppContext {
            bus: &bus,
            time: &clock,
        };

        app.submit_imu(ImuReading {
            timestamp: fsw_sdk_core::MissionTime(0),
            accel_body_mps2: [1.0, 0.0, -0.2],
            gyro_body_rps: [0.0, 0.0, 0.0],
        });
        app.init(&ctx).expect("init");

        clock.tick(DurationMs(50));
        app.step(&ctx).expect("step before period");
        let first = app.solution();
        assert!(first.position_m[0].abs() < 1.0e-9);

        clock.tick(DurationMs(50));
        app.step(&ctx).expect("step at period");
        let second = app.solution();
        assert!(second.position_m[0] > 0.0);
        assert!(second.position_m[2] < 0.0);
        assert!(bus.receive(&GNC_SOLUTION_TOPIC).expect("read bus").is_some());
    }

    #[test]
    fn app_applies_gps_and_magnetometer_corrections() {
        let mut app = GncComponent::new(GncConfig {
            ekf_rate_hz: 5,
            ..GncConfig::default()
        });
        let bus = InMemoryBus::new();
        let clock = StaticClock::new();
        let ctx = AppContext {
            bus: &bus,
            time: &clock,
        };

        app.submit_imu(ImuReading {
            timestamp: fsw_sdk_core::MissionTime(0),
            accel_body_mps2: [0.0, 0.0, 0.0],
            gyro_body_rps: [0.02, -0.01, 0.2],
        });
        app.submit_gps(GpsReading {
            timestamp: fsw_sdk_core::MissionTime(200),
            position_m: [25.0, -8.0, 3.0],
            velocity_mps: [3.0, 0.5, -0.1],
        });
        app.submit_magnetometer(MagnetometerReading {
            timestamp: fsw_sdk_core::MissionTime(200),
            magnetic_field_body: [0.995, -0.1, 0.0],
        });
        app.init(&ctx).expect("init");

        clock.tick(DurationMs(200));
        app.step(&ctx).expect("step");
        let solution = app.solution();

        assert!(solution.position_m[0] > 10.0);
        assert!(solution.position_m[1] < -2.0);
        assert!(solution.position_m[2] > 1.0);
        assert!(solution.attitude_rad[2].abs() < 0.2);
    }
}
