// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Standalone full-stack host simulation for AstraForge mission apps.

pub mod config;
pub mod transport;

use std::collections::BTreeSet;
use std::fmt::Write as _;
use std::sync::{Arc, Mutex, MutexGuard};

use communications_template::{
    CommunicationApp, CommunicationConfig, LinkStatus, COMM_COMPONENT_ID, COMM_DOWNLINK_TOPIC,
    COMM_STATUS_TOPIC,
};
use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{
    ComponentHealth, ComponentId, FswComponent, MessageBus, MissionTime, SdkError, TimeSource,
};
use fsw_sdk_fdir::ClassMapPolicy;
use fsw_sdk_hal::{HalPlatform, HostSimPlatform};
use fsw_sdk_runtime::{Runtime, SchedulerConfig};
use payload_interface_template::{
    PayloadInterfaceApp, PayloadMode, PAYLOAD_COMMAND_TOPIC, PAYLOAD_COMPONENT_ID,
    PAYLOAD_STATUS_TOPIC,
};
use power_management_template::{
    PowerManagementApp, PowerMode, PowerSnapshot, POWER_COMPONENT_ID, POWER_STATUS_TOPIC,
};
use simple_gnc::{
    new_shared_gnc_component, GncCommand, GncComponentProxy, GncConfig, GpsReading, GuidanceMode,
    GuidanceReference, ImuReading, MagnetometerReading, NavigationSolution, SharedGncComponent,
    GNC_COMPONENT_ID, GNC_SOLUTION_TOPIC,
};
use thermal_management_template::{
    ThermalManagementApp, ThermalMode, THERMAL_COMPONENT_ID, THERMAL_STATUS_TOPIC,
};
use vehicle_management_template::{
    VehicleEvent, VehicleManagementApp, VehicleMode, VEHICLE_MGMT_COMPONENT_ID, VEHICLE_MODE_TOPIC,
};

const STANDARD_GRAVITY_MPS2: f64 = 9.80665;
const EARTH_RADIUS_M: f64 = 6_371_000.0;
const ORIGIN_LAT_DEG: f64 = 34.7420;
const ORIGIN_LON_DEG: f64 = -120.5724;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MissionPhase {
    PadInitialization,
    Launch,
    Flight,
    Landing,
    Impact,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PhaseControl {
    Auto,
    Manual,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum SimAnomaly {
    GpsDropout,
    ImuBiasSpike,
    BatterySag,
    ThermalRunaway,
    CommLoss,
    PayloadFault,
    EngineFailure,
    HardLanding,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FlightComputerMode {
    Standby,
    Active,
    Safe,
    Failed,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct FlightComputerStatus {
    pub powered: bool,
    pub single_string: bool,
    pub resets: u32,
    pub mode: FlightComputerMode,
    pub cpu_load: f64,
    pub avionics_temp_c: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct SensorSnapshot {
    pub imu: ImuReading,
    pub gps: Option<GpsReading>,
    pub magnetometer: MagnetometerReading,
    pub barometric_altitude_m: f64,
    pub radar_altitude_m: f64,
    pub airspeed_mps: f64,
    pub vertical_speed_mps: f64,
    pub downrange_m: f64,
    pub battery_voltage_v: f64,
    pub battery_current_a: f64,
    pub battery_soc: f64,
    pub signal_strength_dbm: f64,
    pub avionics_temp_c: f64,
    pub payload_temp_c: f64,
    pub payload_current_a: f64,
    pub impact_detected: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TruthState {
    pub position_m: [f64; 3],
    pub velocity_mps: [f64; 3],
    pub attitude_rad: [f64; 3],
    pub body_accel_mps2: [f64; 3],
    pub body_rates_rps: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeodeticPosition {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetState {
    pub latitude_deg: f64,
    pub longitude_deg: f64,
    pub altitude_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LaunchVector {
    pub azimuth_deg: f64,
    pub elevation_deg: f64,
    pub altitude_m: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct TargetOffset {
    pub east_m: f64,
    pub north_m: f64,
    pub altitude_m: f64,
}

const LAUNCH_VECTOR_TARGET_DISTANCE_M: f64 = 1_000.0;

impl LaunchVector {
    fn local_target(&self) -> [f64; 3] {
        let az = self.azimuth_deg.to_radians();
        let el = self.elevation_deg.to_radians();
        let horizontal = LAUNCH_VECTOR_TARGET_DISTANCE_M * el.cos();
        [
            horizontal * az.sin(),
            horizontal * az.cos(),
            self.altitude_m,
        ]
    }

    fn velocity_along_vector(&self, speed_mps: f64) -> [f64; 3] {
        let az = self.azimuth_deg.to_radians();
        let el = self.elevation_deg.to_radians();
        let horizontal_speed = speed_mps * el.cos();
        [
            horizontal_speed * az.sin(),
            horizontal_speed * az.cos(),
            speed_mps * el.sin(),
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PropulsionState {
    pub throttle_percent: u32,
    pub max_thrust_kn: f64,
    pub isp_s: f64,
    pub dry_mass_kg: f64,
    pub propellant_mass_kg: f64,
    pub total_mass_kg: f64,
    pub current_thrust_kn: f64,
    pub mass_flow_kgps: f64,
    pub consumption_scale: f64,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ControlSurfaceState {
    pub roll_surface: f64,
    pub pitch_surface: f64,
    pub yaw_surface: f64,
    pub desired_attitude_rad: [f64; 3],
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct AeroState {
    pub air_density_kgpm3: f64,
    pub dynamic_pressure_pa: f64,
    pub mach: f64,
    pub alpha_rad: f64,
    pub beta_rad: f64,
    pub body_velocity_mps: [f64; 3],
    pub aerodynamic_force_body_n: [f64; 3],
    pub aerodynamic_moment_body_nm: [f64; 3],
}

#[derive(Debug, Clone, PartialEq, Eq, Default)]
pub struct TelemetrySnapshot {
    pub vehicle_mode_line: String,
    pub power_status_line: String,
    pub thermal_status_line: String,
    pub payload_status_line: String,
    pub comm_status_line: String,
    pub gnc_solution_line: String,
    pub latest_downlink_frame: String,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SystemSnapshot {
    pub vehicle_mode: VehicleMode,
    pub power_mode: PowerMode,
    pub thermal_mode: ThermalMode,
    pub payload_mode: PayloadMode,
    pub comm_link_status: LinkStatus,
    pub gnc_solution: NavigationSolution,
}

#[derive(Debug, Clone, PartialEq)]
pub struct SimulationSnapshot {
    pub mission_time: MissionTime,
    pub phase: MissionPhase,
    pub phase_control: PhaseControl,
    pub flight_computer: FlightComputerStatus,
    pub truth: TruthState,
    pub geodetic: GeodeticPosition,
    pub target: TargetState,
    pub launch_vector: LaunchVector,
    pub controlled_flight_enabled: bool,
    pub target_offset: TargetOffset,
    pub has_left_pad: bool,
    pub propulsion: PropulsionState,
    pub control_surfaces: ControlSurfaceState,
    pub aerodynamics: AeroState,
    pub sensors: SensorSnapshot,
    pub systems: SystemSnapshot,
    pub anomalies: Vec<SimAnomaly>,
    pub telemetry: TelemetrySnapshot,
    pub last_fault_responses: Vec<(String, String)>,
}

#[derive(Debug, Clone, Copy)]
pub struct SimConfig {
    pub step_ms: u64,
    pub scheduler: SchedulerConfig,
    pub gnc: GncConfig,
    pub vehicle: VehicleModelConfig,
    pub launch_vector: LaunchVector,
    pub target_offset: TargetOffset,
    pub controlled_flight_enabled: bool,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct VehicleModelConfig {
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

impl Default for VehicleModelConfig {
    fn default() -> Self {
        Self {
            reference_area_m2: 1.85,
            reference_span_m: 2.8,
            reference_chord_m: 0.92,
            inertia_roll_scale: 0.22,
            inertia_pitch_scale: 0.18,
            inertia_yaw_scale: 0.12,
            drag_base: 0.18,
            drag_alpha2: 0.85,
            drag_beta2: 0.35,
            drag_mach_gain: 0.06,
            side_force_beta: -0.75,
            side_force_yaw_surface: 0.42,
            lift_alpha: -1.8,
            lift_pitch_surface: 0.65,
            roll_moment_beta: -0.22,
            roll_damping_p: -0.48,
            roll_surface_gain: 0.92,
            pitch_moment_alpha: -1.15,
            pitch_damping_q: -0.62,
            pitch_surface_gain: 1.08,
            yaw_moment_beta: 0.24,
            yaw_damping_r: -0.36,
            yaw_surface_gain: 0.88,
            launch_thrust_axis_factor: 1.0,
            flight_thrust_axis_factor: 0.62,
            landing_thrust_axis_factor: 0.78,
            axial_damping_gain: 0.028,
            lateral_damping_gain: 0.028,
            vertical_damping_gain: 0.018,
            max_body_rate_rps: 2.5,
            max_body_accel_mps2: 60.0,
            max_angular_accel_rps2: 8.0,
        }
    }
}

impl Default for SimConfig {
    fn default() -> Self {
        let launch_vector = LaunchVector {
            azimuth_deg: 90.0,
            elevation_deg: 0.0,
            altitude_m: 0.0,
        };
        Self {
            step_ms: 100,
            scheduler: SchedulerConfig::default(),
            gnc: GncConfig::default(),
            vehicle: VehicleModelConfig::default(),
            launch_vector,
            target_offset: TargetOffset {
                east_m: 1_000.0,
                north_m: 0.0,
                altitude_m: 0.0,
            },
            controlled_flight_enabled: true,
        }
    }
}

impl SimConfig {
    #[must_use]
    pub fn initial_target_offset(&self) -> [f64; 3] {
        [
            self.target_offset.east_m,
            self.target_offset.north_m,
            self.target_offset.altitude_m,
        ]
    }
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SimulatorCommand {
    Step(u32),
    Reset,
    SetPhase(MissionPhase),
    SetPhaseControl(PhaseControl),
    SetTarget(TargetState),
    ConfigurePropulsion(PropulsionState),
    ConfigureLaunchVector {
        azimuth_deg: f64,
        elevation_deg: f64,
        altitude_m: Option<f64>,
    },
    ConfigureTargetOffset {
        east_m: f64,
        north_m: f64,
        altitude_m: f64,
    },
    SetControlledFlight(bool),
    Inject(SimAnomaly),
    Clear(SimAnomaly),
    ClearAllAnomalies,
    QueueVehicleEvent(VehicleEvent),
}

struct AppProxy<T> {
    id: &'static str,
    inner: Arc<Mutex<T>>,
}

impl<T> AppProxy<T> {
    fn new(id: &'static str, inner: Arc<Mutex<T>>) -> Self {
        Self { id, inner }
    }
}

impl<T> FswComponent for AppProxy<T>
where
    T: FswComponent + Send + 'static,
{
    fn id(&self) -> ComponentId {
        ComponentId(self.id.to_string())
    }

    fn dependencies(&self) -> Vec<(ComponentId, fsw_sdk_core::DependencyLevel)> {
        self.inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)
            .map_or_else(|_| Vec::new(), |component| component.dependencies())
    }

    fn init(&mut self) -> Result<(), SdkError> {
        self.inner
            .lock()
            .map_err(|_| SdkError::BackendFailure)?
            .init()
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

pub struct FullStackSim {
    config: SimConfig,
    platform: HostSimPlatform,
    runtime: Runtime<ClassMapPolicy>,
    phase: MissionPhase,
    phase_control: PhaseControl,
    anomalies: BTreeSet<SimAnomaly>,
    flight_computer: FlightComputerStatus,
    truth: TruthState,
    geodetic: GeodeticPosition,
    target: TargetState,
    target_offset: TargetOffset,
    propulsion: PropulsionState,
    control_surfaces: ControlSurfaceState,
    aerodynamics: AeroState,
    sensors: SensorSnapshot,
    telemetry: TelemetrySnapshot,
    last_fault_responses: Vec<(String, String)>,
    gnc: SharedGncComponent,
    comms: Arc<Mutex<CommunicationApp>>,
    vehicle: Arc<Mutex<VehicleManagementApp>>,
    power: Arc<Mutex<PowerManagementApp>>,
    thermal: Arc<Mutex<ThermalManagementApp>>,
    payload: Arc<Mutex<PayloadInterfaceApp>>,
    has_left_pad: bool,
    controlled_flight_enabled: bool,
}

impl FullStackSim {
    pub fn new(config: SimConfig) -> Result<Self, SdkError> {
        let platform = HostSimPlatform::new();
        let mut runtime = Runtime::new(ClassMapPolicy::conservative());
        runtime.set_scheduler_config(config.scheduler);

        let (gnc, gnc_proxy): (SharedGncComponent, GncComponentProxy) =
            new_shared_gnc_component(config.gnc);
        let comms = Arc::new(Mutex::new(CommunicationApp::new(
            CommunicationConfig::default(),
        )));
        let vehicle = Arc::new(Mutex::new(VehicleManagementApp::new()));
        let power = Arc::new(Mutex::new(PowerManagementApp::new()));
        let thermal = Arc::new(Mutex::new(ThermalManagementApp::new()));
        let payload = Arc::new(Mutex::new(PayloadInterfaceApp::new()));

        runtime.register(Box::new(gnc_proxy))?;
        runtime.register(Box::new(AppProxy::new(
            COMM_COMPONENT_ID,
            Arc::clone(&comms),
        )))?;
        runtime.register(Box::new(AppProxy::new(
            VEHICLE_MGMT_COMPONENT_ID,
            Arc::clone(&vehicle),
        )))?;
        runtime.register(Box::new(AppProxy::new(
            POWER_COMPONENT_ID,
            Arc::clone(&power),
        )))?;
        runtime.register(Box::new(AppProxy::new(
            THERMAL_COMPONENT_ID,
            Arc::clone(&thermal),
        )))?;
        runtime.register(Box::new(AppProxy::new(
            PAYLOAD_COMPONENT_ID,
            Arc::clone(&payload),
        )))?;
        runtime.start_all()?;

        let truth = TruthState {
            position_m: [0.0, 0.0, 0.0],
            velocity_mps: [0.0, 0.0, 0.0],
            attitude_rad: [0.0, 0.0, 0.0],
            body_accel_mps2: [0.0, 0.0, 0.0],
            body_rates_rps: [0.0, 0.0, 0.0],
        };
        let initial_time = MissionTime(0);
        let geodetic = geodetic_from_local(truth.position_m);
        let target_offset = config.target_offset;
        let target_local = [
            target_offset.east_m,
            target_offset.north_m,
            target_offset.altitude_m,
        ];
        let target_geodetic = geodetic_from_local(target_local);
        let target = TargetState {
            latitude_deg: target_geodetic.latitude_deg,
            longitude_deg: target_geodetic.longitude_deg,
            altitude_m: target_geodetic.altitude_m,
        };
        let propulsion = PropulsionState {
            throttle_percent: 85,
            max_thrust_kn: 180.0,
            isp_s: 260.0,
            dry_mass_kg: 1_450.0,
            propellant_mass_kg: 620.0,
            total_mass_kg: 2_070.0,
            current_thrust_kn: 0.0,
            mass_flow_kgps: 0.0,
            consumption_scale: 1.0,
        };
        let control_surfaces = ControlSurfaceState {
            roll_surface: 0.0,
            pitch_surface: 0.0,
            yaw_surface: 0.0,
            desired_attitude_rad: [0.0; 3],
        };
        let aerodynamics = AeroState {
            air_density_kgpm3: 1.225,
            dynamic_pressure_pa: 0.0,
            mach: 0.0,
            alpha_rad: 0.0,
            beta_rad: 0.0,
            body_velocity_mps: [0.0; 3],
            aerodynamic_force_body_n: [0.0; 3],
            aerodynamic_moment_body_nm: [0.0; 3],
        };
        let sensors = sensor_from_truth(
            initial_time,
            truth,
            MissionPhase::PadInitialization,
            &BTreeSet::new(),
            propulsion,
        );

        let mut sim = Self {
            config,
            platform,
            runtime,
            phase: MissionPhase::PadInitialization,
            phase_control: PhaseControl::Auto,
            anomalies: BTreeSet::new(),
            flight_computer: FlightComputerStatus {
                powered: true,
                single_string: true,
                resets: 0,
                mode: FlightComputerMode::Standby,
                cpu_load: 0.18,
                avionics_temp_c: sensors.avionics_temp_c,
            },
            truth,
            geodetic,
            target,
            target_offset,
            propulsion,
            control_surfaces,
            aerodynamics,
            sensors,
            telemetry: TelemetrySnapshot::default(),
            last_fault_responses: Vec::new(),
            gnc,
            comms,
            vehicle,
            power,
            thermal,
            payload,
            has_left_pad: false,
            controlled_flight_enabled: config.controlled_flight_enabled,
        };
        sim.init_mission_apps()?;
        sim.step_apps(false)?;
        sim.collect_telemetry()?;
        Ok(sim)
    }

    fn set_target_offset(&mut self, offset: TargetOffset) {
        let target_geodetic =
            geodetic_from_local([offset.east_m, offset.north_m, offset.altitude_m]);
        self.target = TargetState {
            latitude_deg: target_geodetic.latitude_deg,
            longitude_deg: target_geodetic.longitude_deg,
            altitude_m: target_geodetic.altitude_m,
        };
        self.target_offset = offset;
        self.config.target_offset = offset;
    }

    pub fn apply_command(&mut self, command: SimulatorCommand) -> Result<(), SdkError> {
        match command {
            SimulatorCommand::Step(count) => self.step(count),
            SimulatorCommand::Reset => {
                let config = self.config;
                let resets = self.flight_computer.resets.saturating_add(1);
                *self = Self::new(config)?;
                self.flight_computer.resets = resets;
                Ok(())
            }
            SimulatorCommand::SetPhase(phase) => {
                self.phase = phase;
                self.refresh(true)
            }
            SimulatorCommand::SetPhaseControl(control) => {
                self.phase_control = control;
                self.refresh(false)
            }
            SimulatorCommand::SetTarget(target) => {
                let offset = local_from_target(target);
                self.set_target_offset(TargetOffset {
                    east_m: offset[0],
                    north_m: offset[1],
                    altitude_m: offset[2],
                });
                self.refresh(false)
            }
            SimulatorCommand::ConfigurePropulsion(config) => {
                self.propulsion.throttle_percent = config.throttle_percent.min(100);
                self.propulsion.max_thrust_kn = config.max_thrust_kn.max(1.0);
                self.propulsion.isp_s = config.isp_s.max(10.0);
                self.propulsion.dry_mass_kg = config.dry_mass_kg.max(100.0);
                self.propulsion.propellant_mass_kg = config.propellant_mass_kg.max(0.0);
                self.propulsion.consumption_scale = config.consumption_scale.clamp(0.05, 5.0);
                self.refresh(false)
            }
            SimulatorCommand::ConfigureLaunchVector {
                azimuth_deg,
                elevation_deg,
                altitude_m,
            } => {
                let mut vector = self.config.launch_vector;
                vector.azimuth_deg = azimuth_deg;
                vector.elevation_deg = elevation_deg;
                if let Some(value) = altitude_m {
                    vector.altitude_m = value;
                }
                self.config.launch_vector = vector;
                self.refresh(false)
            }
            SimulatorCommand::ConfigureTargetOffset {
                east_m,
                north_m,
                altitude_m,
            } => {
                self.set_target_offset(TargetOffset {
                    east_m,
                    north_m,
                    altitude_m,
                });
                self.refresh(false)
            }
            SimulatorCommand::SetControlledFlight(enabled) => {
                self.controlled_flight_enabled = enabled;
                self.refresh(false)
            }
            SimulatorCommand::Inject(anomaly) => {
                self.anomalies.insert(anomaly);
                self.refresh(false)
            }
            SimulatorCommand::Clear(anomaly) => {
                self.anomalies.remove(&anomaly);
                self.refresh(false)
            }
            SimulatorCommand::ClearAllAnomalies => {
                self.anomalies.clear();
                self.refresh(false)
            }
            SimulatorCommand::QueueVehicleEvent(event) => {
                lock_arc(&self.vehicle)?.queue_event(event);
                self.refresh(false)
            }
        }
    }

    pub fn step(&mut self, count: u32) -> Result<(), SdkError> {
        let steps = count.max(1);
        for _ in 0..steps {
            self.step_once()?;
        }
        Ok(())
    }

    #[must_use]
    pub fn snapshot(&self) -> SimulationSnapshot {
        let vehicle_mode = lock_arc(&self.vehicle)
            .map(|vehicle| vehicle.mode())
            .unwrap_or(VehicleMode::Fault);
        let power_mode = lock_arc(&self.power)
            .map(|power| power.mode())
            .unwrap_or(PowerMode::Survival);
        let thermal_mode = lock_arc(&self.thermal)
            .map(|thermal| thermal.mode())
            .unwrap_or(ThermalMode::Survival);
        let payload_mode = lock_arc(&self.payload)
            .map(|payload| payload.mode())
            .unwrap_or(PayloadMode::Fault);
        let comm_link_status = lock_arc(&self.comms)
            .map(|comms| comms.link_status())
            .unwrap_or(LinkStatus::Offline);
        let gnc_solution =
            lock_arc(&self.gnc)
                .map(|gnc| gnc.solution())
                .unwrap_or(NavigationSolution {
                    timestamp: MissionTime(0),
                    position_m: [0.0; 3],
                    velocity_mps: [0.0; 3],
                    attitude_rad: [0.0; 3],
                    gyro_bias_rps: [0.0; 3],
                    accel_bias_mps2: [0.0; 3],
                });

        SimulationSnapshot {
            mission_time: self.platform.clock().now_monotonic(),
            phase: self.phase,
            phase_control: self.phase_control,
            flight_computer: self.flight_computer,
            truth: self.truth,
            geodetic: self.geodetic,
            target: self.target,
            launch_vector: self.config.launch_vector,
            controlled_flight_enabled: self.controlled_flight_enabled,
            target_offset: self.target_offset,
            has_left_pad: self.has_left_pad,
            propulsion: self.propulsion,
            control_surfaces: self.control_surfaces,
            aerodynamics: self.aerodynamics,
            sensors: self.sensors,
            systems: SystemSnapshot {
                vehicle_mode,
                power_mode,
                thermal_mode,
                payload_mode,
                comm_link_status,
                gnc_solution,
            },
            anomalies: self.anomalies.iter().copied().collect(),
            telemetry: self.telemetry.clone(),
            last_fault_responses: self.last_fault_responses.clone(),
        }
    }

    #[must_use]
    pub fn platform(&self) -> &HostSimPlatform {
        &self.platform
    }

    fn step_once(&mut self) -> Result<(), SdkError> {
        self.platform
            .clock()
            .tick(fsw_sdk_core::DurationMs(self.config.step_ms));
        let previous_phase = self.phase;

        self.advance_truth(self.config.step_ms as f64 / 1000.0);
        self.maybe_transition_phase();
        self.refresh(previous_phase != self.phase)
    }

    fn init_mission_apps(&mut self) -> Result<(), SdkError> {
        self.update_gnc_reference()?;
        let ctx = self.app_context();
        MissionApp::init(&mut *lock_arc(&self.gnc)?, &ctx)?;
        MissionApp::init(&mut *lock_arc(&self.comms)?, &ctx)?;
        MissionApp::init(&mut *lock_arc(&self.vehicle)?, &ctx)?;
        MissionApp::init(&mut *lock_arc(&self.power)?, &ctx)?;
        MissionApp::init(&mut *lock_arc(&self.thermal)?, &ctx)?;
        MissionApp::init(&mut *lock_arc(&self.payload)?, &ctx)?;
        Ok(())
    }

    fn step_apps(&mut self, phase_changed: bool) -> Result<(), SdkError> {
        let ctx = self.app_context();

        {
            let mut vehicle = lock_arc(&self.vehicle)?;
            if phase_changed {
                queue_phase_events(&mut vehicle, self.phase);
            }
            if self.anomalies.contains(&SimAnomaly::BatterySag) {
                vehicle.queue_event(VehicleEvent::PowerEmergency);
            }
            if self.anomalies.contains(&SimAnomaly::ThermalRunaway) {
                vehicle.queue_event(VehicleEvent::ThermalEmergency);
            }
            if self.anomalies.contains(&SimAnomaly::EngineFailure) {
                vehicle.queue_event(VehicleEvent::FaultDetected);
            }
            MissionApp::step(&mut *vehicle, &ctx)?;
        }

        {
            let mut gnc = lock_arc(&self.gnc)?;
            gnc.set_guidance_reference(GuidanceReference {
                target_position_m: local_from_target(self.target),
                target_velocity_mps: match self.phase {
                    MissionPhase::Launch => [0.0, 0.0, 28.0],
                    MissionPhase::Flight => [0.0, 0.0, 0.0],
                    MissionPhase::Landing => [0.0, 0.0, -8.0],
                    MissionPhase::PadInitialization | MissionPhase::Impact => [0.0; 3],
                },
                mode: match self.phase {
                    MissionPhase::PadInitialization | MissionPhase::Impact => GuidanceMode::Hold,
                    MissionPhase::Launch => GuidanceMode::Ascent,
                    MissionPhase::Flight => GuidanceMode::Cruise,
                    MissionPhase::Landing => GuidanceMode::Landing,
                },
            });
            gnc.submit_imu(self.sensors.imu);
            if let Some(gps) = self.sensors.gps {
                gnc.submit_gps(gps);
            }
            gnc.submit_magnetometer(self.sensors.magnetometer);
            MissionApp::step(&mut *gnc, &ctx)?;
        }

        {
            let mut power = lock_arc(&self.power)?;
            power.update_snapshot(PowerSnapshot {
                bus_voltage_v: self.sensors.battery_voltage_v,
                bus_current_a: self.sensors.battery_current_a,
                battery_soc: self.sensors.battery_soc,
            });
            MissionApp::step(&mut *power, &ctx)?;
        }

        {
            let mut thermal = lock_arc(&self.thermal)?;
            thermal.update_zone_temperature("avionics", self.sensors.avionics_temp_c);
            thermal.update_zone_temperature("payload", self.sensors.payload_temp_c);
            thermal.update_zone_temperature("battery", self.sensors.avionics_temp_c - 4.0);
            MissionApp::step(&mut *thermal, &ctx)?;
        }

        {
            let mut payload = lock_arc(&self.payload)?;
            if self.anomalies.contains(&SimAnomaly::PayloadFault) {
                payload.inject_fault();
            } else {
                payload.clear_fault();
                if matches!(self.phase, MissionPhase::Flight) {
                    ctx.bus
                        .publish(&PAYLOAD_COMMAND_TOPIC, b"sample_payload_capture".to_vec())?;
                }
            }
            MissionApp::step(&mut *payload, &ctx)?;
        }

        {
            let mut comms = lock_arc(&self.comms)?;
            let link_status = if self.anomalies.contains(&SimAnomaly::CommLoss) {
                LinkStatus::Offline
            } else if matches!(self.phase, MissionPhase::Launch | MissionPhase::Flight) {
                LinkStatus::Online
            } else {
                LinkStatus::Standby
            };
            comms.set_link_status(link_status);
            comms.queue_downlink_frame(render_downlink_frame(
                self.phase,
                &self.anomalies,
                &self.sensors,
            ));
            MissionApp::step(&mut *comms, &ctx)?;
        }

        let now = ctx.time.now_monotonic();
        for component_id in [
            GNC_COMPONENT_ID,
            COMM_COMPONENT_ID,
            VEHICLE_MGMT_COMPONENT_ID,
            POWER_COMPONENT_ID,
            THERMAL_COMPONENT_ID,
            PAYLOAD_COMPONENT_ID,
        ] {
            self.runtime
                .mark_heartbeat(&ComponentId(component_id.to_string()), now)?;
        }

        Ok(())
    }

    fn collect_telemetry(&mut self) -> Result<(), SdkError> {
        self.telemetry.vehicle_mode_line =
            read_latest_line(self.platform.bus(), &VEHICLE_MODE_TOPIC)?;
        self.telemetry.power_status_line =
            read_latest_line(self.platform.bus(), &POWER_STATUS_TOPIC)?;
        self.telemetry.thermal_status_line =
            read_latest_line(self.platform.bus(), &THERMAL_STATUS_TOPIC)?;
        self.telemetry.payload_status_line =
            read_latest_line(self.platform.bus(), &PAYLOAD_STATUS_TOPIC)?;
        self.telemetry.comm_status_line =
            read_latest_line(self.platform.bus(), &COMM_STATUS_TOPIC)?;
        self.telemetry.gnc_solution_line =
            read_latest_line(self.platform.bus(), &GNC_SOLUTION_TOPIC)?;
        self.telemetry.latest_downlink_frame =
            read_latest_line(self.platform.bus(), &COMM_DOWNLINK_TOPIC)?;
        Ok(())
    }

    fn maybe_transition_phase(&mut self) {
        let altitude = self.truth.position_m[2];
        if altitude <= 0.0
            && self.phase != MissionPhase::PadInitialization
            && (self.phase != MissionPhase::Launch || self.has_left_pad)
        {
            self.phase = MissionPhase::Impact;
            return;
        }

        if self.phase_control == PhaseControl::Manual {
            return;
        }

        let vertical_speed = self.truth.velocity_mps[2];
        let elapsed = self.platform.clock().now_monotonic().0;
        self.phase = match self.phase {
            MissionPhase::PadInitialization if elapsed >= 5_000 => MissionPhase::Launch,
            MissionPhase::Launch if altitude >= 750.0 || elapsed >= 20_000 => MissionPhase::Flight,
            MissionPhase::Flight if vertical_speed < -8.0 && altitude <= 400.0 => {
                MissionPhase::Landing
            }
            MissionPhase::Landing if altitude <= 0.0 => MissionPhase::Impact,
            phase => phase,
        };
    }

    fn advance_truth(&mut self, dt_s: f64) {
        let accel_nav;
        let gnc_command = lock_arc(&self.gnc)
            .map(|gnc| gnc.control_command())
            .unwrap_or(GncCommand {
                timestamp: self.platform.clock().now_monotonic(),
                target_position_m: [0.0; 3],
                target_velocity_mps: [0.0; 3],
                desired_attitude_rad: [0.0; 3],
                control_surfaces: [0.0; 3],
            });
        let command_surfaces = if self.controlled_flight_enabled {
            gnc_command.control_surfaces
        } else {
            [0.0; 3]
        };
        self.control_surfaces = ControlSurfaceState {
            roll_surface: command_surfaces[0],
            pitch_surface: command_surfaces[1],
            yaw_surface: command_surfaces[2],
            desired_attitude_rad: gnc_command.desired_attitude_rad,
        };
        self.propulsion.total_mass_kg =
            (self.propulsion.dry_mass_kg + self.propulsion.propellant_mass_kg).max(1.0);
        let thrust_factor = if self.anomalies.contains(&SimAnomaly::EngineFailure) {
            0.32
        } else {
            1.0
        };
        let phase_throttle_factor = match self.phase {
            MissionPhase::PadInitialization => 0.0,
            MissionPhase::Launch => 1.0,
            MissionPhase::Flight => 0.55,
            MissionPhase::Landing => 0.42,
            MissionPhase::Impact => 0.0,
        };
        let effective_throttle =
            f64::from(self.propulsion.throttle_percent) / 100.0 * phase_throttle_factor;
        let base_thrust_n =
            self.propulsion.max_thrust_kn * 1_000.0 * effective_throttle * thrust_factor;
        let base_mass_flow_kgps = if self.propulsion.isp_s <= f64::EPSILON {
            0.0
        } else {
            base_thrust_n / (self.propulsion.isp_s * STANDARD_GRAVITY_MPS2)
                * self.propulsion.consumption_scale
        };
        let propellant_used = (base_mass_flow_kgps * dt_s).min(self.propulsion.propellant_mass_kg);
        let thrust_scale = if base_mass_flow_kgps <= f64::EPSILON {
            0.0
        } else {
            (propellant_used / (base_mass_flow_kgps * dt_s).max(f64::EPSILON)).clamp(0.0, 1.0)
        };
        let current_thrust_n = base_thrust_n * thrust_scale;
        self.propulsion.current_thrust_kn = current_thrust_n / 1_000.0;
        self.propulsion.mass_flow_kgps = base_mass_flow_kgps * thrust_scale;
        self.propulsion.propellant_mass_kg -= propellant_used;
        self.propulsion.total_mass_kg =
            (self.propulsion.dry_mass_kg + self.propulsion.propellant_mass_kg).max(1.0);
        let mass_kg = self.propulsion.total_mass_kg;

        match self.phase {
            MissionPhase::PadInitialization => {
                self.truth.position_m[2] = 0.0;
                self.truth.velocity_mps = [0.0; 3];
                self.truth.body_rates_rps = [0.0; 3];
                self.truth.body_accel_mps2 = [0.0; 3];
                self.aerodynamics = AeroState {
                    air_density_kgpm3: air_density(self.truth.position_m[2]),
                    dynamic_pressure_pa: 0.0,
                    mach: 0.0,
                    alpha_rad: 0.0,
                    beta_rad: 0.0,
                    body_velocity_mps: [0.0; 3],
                    aerodynamic_force_body_n: [0.0; 3],
                    aerodynamic_moment_body_nm: [0.0; 3],
                };
                return;
            }
            MissionPhase::Impact => {
                self.truth.position_m[2] = 0.0;
                self.truth.velocity_mps = [0.0; 3];
                self.truth.body_accel_mps2 = [0.0; 3];
                self.truth.body_rates_rps = [0.0; 3];
                return;
            }
            MissionPhase::Launch | MissionPhase::Flight | MissionPhase::Landing => {}
        }

        let body_to_nav = rotation_body_to_nav(self.truth.attitude_rad);
        let nav_to_body = transpose3(body_to_nav);
        let body_velocity = multiply_matrix_vec3(&nav_to_body, &self.truth.velocity_mps);
        let aero = aerodynamic_state(
            body_velocity,
            self.truth.body_rates_rps,
            self.control_surfaces,
            self.truth.position_m[2],
            self.config.vehicle,
        );
        self.aerodynamics = aero;

        let thrust_axis_factor = match self.phase {
            MissionPhase::Launch => self.config.vehicle.launch_thrust_axis_factor,
            MissionPhase::Flight => self.config.vehicle.flight_thrust_axis_factor,
            MissionPhase::Landing => self.config.vehicle.landing_thrust_axis_factor,
            MissionPhase::PadInitialization | MissionPhase::Impact => 0.0,
        };
        let thrust_body_n = [current_thrust_n * thrust_axis_factor, 0.0, 0.0];
        let gravity_body = multiply_matrix_vec3(&nav_to_body, &[0.0, 0.0, -STANDARD_GRAVITY_MPS2]);
        let damping_force_body_n = [
            -body_velocity[0] * self.config.vehicle.axial_damping_gain * mass_kg,
            -body_velocity[1] * self.config.vehicle.lateral_damping_gain * mass_kg,
            -body_velocity[2] * self.config.vehicle.vertical_damping_gain * mass_kg,
        ];
        let total_force_body_n = add_vec3(
            add_vec3(aero.aerodynamic_force_body_n, thrust_body_n),
            damping_force_body_n,
        );
        let coriolis_term = cross3(self.truth.body_rates_rps, body_velocity);
        let body_accel = [
            total_force_body_n[0] / mass_kg + gravity_body[0] - coriolis_term[0],
            total_force_body_n[1] / mass_kg + gravity_body[1] - coriolis_term[1],
            total_force_body_n[2] / mass_kg + gravity_body[2] - coriolis_term[2],
        ];
        let body_accel = [
            sanitize_clamp(
                body_accel[0],
                -self.config.vehicle.max_body_accel_mps2,
                self.config.vehicle.max_body_accel_mps2,
            ),
            sanitize_clamp(
                body_accel[1],
                -self.config.vehicle.max_body_accel_mps2,
                self.config.vehicle.max_body_accel_mps2,
            ),
            sanitize_clamp(
                body_accel[2],
                -self.config.vehicle.max_body_accel_mps2,
                self.config.vehicle.max_body_accel_mps2,
            ),
        ];

        let inertia = vehicle_inertia_kgm2(mass_kg, self.config.vehicle);
        let angular_accel = angular_acceleration(
            self.truth.body_rates_rps,
            aero.aerodynamic_moment_body_nm,
            inertia,
        );
        let angular_accel = [
            sanitize_clamp(
                angular_accel[0],
                -self.config.vehicle.max_angular_accel_rps2,
                self.config.vehicle.max_angular_accel_rps2,
            ),
            sanitize_clamp(
                angular_accel[1],
                -self.config.vehicle.max_angular_accel_rps2,
                self.config.vehicle.max_angular_accel_rps2,
            ),
            sanitize_clamp(
                angular_accel[2],
                -self.config.vehicle.max_angular_accel_rps2,
                self.config.vehicle.max_angular_accel_rps2,
            ),
        ];
        let next_body_rates = [
            sanitize_clamp(
                self.truth.body_rates_rps[0] + angular_accel[0] * dt_s,
                -self.config.vehicle.max_body_rate_rps,
                self.config.vehicle.max_body_rate_rps,
            ),
            sanitize_clamp(
                self.truth.body_rates_rps[1] + angular_accel[1] * dt_s,
                -self.config.vehicle.max_body_rate_rps,
                self.config.vehicle.max_body_rate_rps,
            ),
            sanitize_clamp(
                self.truth.body_rates_rps[2] + angular_accel[2] * dt_s,
                -self.config.vehicle.max_body_rate_rps,
                self.config.vehicle.max_body_rate_rps,
            ),
        ];
        let next_body_velocity = [
            sanitize_finite(body_velocity[0] + body_accel[0] * dt_s),
            sanitize_finite(body_velocity[1] + body_accel[1] * dt_s),
            sanitize_finite(body_velocity[2] + body_accel[2] * dt_s),
        ];
        let next_nav_velocity = multiply_matrix_vec3(&body_to_nav, &next_body_velocity);
        accel_nav = multiply_matrix_vec3(&body_to_nav, &body_accel);
        let euler_rates = body_rates_to_euler_rates(self.truth.attitude_rad, next_body_rates);

        for axis in 0..3 {
            self.truth.position_m[axis] +=
                self.truth.velocity_mps[axis] * dt_s + 0.5 * accel_nav[axis] * dt_s * dt_s;
            self.truth.velocity_mps[axis] = next_nav_velocity[axis];
            self.truth.attitude_rad[axis] =
                wrap_angle(self.truth.attitude_rad[axis] + euler_rates[axis] * dt_s);
        }
        self.truth.position_m[2] = self.truth.position_m[2].max(0.0);
        if self.truth.position_m[2] > 0.0 {
            self.has_left_pad = true;
        }
        if self.truth.position_m[2] <= 0.0 && self.phase != MissionPhase::PadInitialization {
            self.truth.velocity_mps = [0.0; 3];
            self.truth.body_rates_rps = [0.0; 3];
        }
        self.geodetic = geodetic_from_local(self.truth.position_m);
        self.truth.body_accel_mps2 = body_accel;
        self.truth.body_rates_rps = next_body_rates;
    }

    fn update_flight_computer(&mut self) {
        self.flight_computer.cpu_load = match self.phase {
            MissionPhase::PadInitialization => 0.24,
            MissionPhase::Launch => 0.68,
            MissionPhase::Flight => 0.57,
            MissionPhase::Landing => 0.51,
            MissionPhase::Impact => 0.09,
        };
        if self.anomalies.contains(&SimAnomaly::ThermalRunaway) {
            self.flight_computer.mode = FlightComputerMode::Safe;
        }
        self.flight_computer.avionics_temp_c = self.sensors.avionics_temp_c;
    }

    fn app_context(&self) -> AppContext<'_> {
        AppContext {
            bus: self.platform.message_bus(),
            time: self.platform.time_source(),
        }
    }

    fn refresh(&mut self, phase_changed: bool) -> Result<(), SdkError> {
        let now = self.platform.clock().now_monotonic();
        self.geodetic = geodetic_from_local(self.truth.position_m);
        self.sensors = sensor_from_truth(
            now,
            self.truth,
            self.phase,
            &self.anomalies,
            self.propulsion,
        );
        self.update_flight_computer();
        self.step_apps(phase_changed)?;
        self.collect_telemetry()?;

        let responses = self.runtime.supervisor_step(now)?;
        self.last_fault_responses = responses
            .into_iter()
            .map(|(component, response)| (component.0, format!("{response:?}")))
            .collect();
        self.flight_computer.mode = if self.last_fault_responses.is_empty() {
            FlightComputerMode::Active
        } else {
            FlightComputerMode::Safe
        };
        Ok(())
    }

    fn update_gnc_reference(&mut self) -> Result<(), SdkError> {
        const LAUNCH_SPEED_MPS: f64 = 28.0;
        let (target_position_m, target_velocity_mps, mode) = match self.phase {
            MissionPhase::PadInitialization | MissionPhase::Impact => {
                (local_from_target(self.target), [0.0; 3], GuidanceMode::Hold)
            }
            MissionPhase::Launch => (
                self.config.launch_vector.local_target(),
                self.config
                    .launch_vector
                    .velocity_along_vector(LAUNCH_SPEED_MPS),
                GuidanceMode::Ascent,
            ),
            MissionPhase::Flight => (
                local_from_target(self.target),
                [0.0; 3],
                GuidanceMode::Cruise,
            ),
            MissionPhase::Landing => (
                local_from_target(self.target),
                [0.0, 0.0, -8.0],
                GuidanceMode::Landing,
            ),
        };
        lock_arc(&self.gnc)?.set_guidance_reference(GuidanceReference {
            target_position_m,
            target_velocity_mps,
            mode,
        });
        Ok(())
    }
}

fn sensor_from_truth(
    now: MissionTime,
    truth: TruthState,
    phase: MissionPhase,
    anomalies: &BTreeSet<SimAnomaly>,
    propulsion: PropulsionState,
) -> SensorSnapshot {
    let gps = if anomalies.contains(&SimAnomaly::GpsDropout) {
        None
    } else {
        Some(GpsReading {
            timestamp: now,
            position_m: truth.position_m,
            velocity_mps: truth.velocity_mps,
        })
    };

    let imu_bias = if anomalies.contains(&SimAnomaly::ImuBiasSpike) {
        [0.9, -0.6, 1.4]
    } else {
        [0.02, -0.01, 0.03]
    };
    let gyro_bias = if anomalies.contains(&SimAnomaly::ImuBiasSpike) {
        [0.03, -0.02, 0.04]
    } else {
        [0.002, 0.001, -0.001]
    };

    let altitude = truth.position_m[2];
    let speed = norm3(truth.velocity_mps);
    let battery_soc = if anomalies.contains(&SimAnomaly::BatterySag) {
        0.18
    } else {
        (0.98 - altitude / 50_000.0).clamp(0.32, 0.98)
    };
    let avionics_temp_c = if anomalies.contains(&SimAnomaly::ThermalRunaway) {
        81.0
    } else {
        22.0 + speed * 0.08
    };

    SensorSnapshot {
        imu: ImuReading {
            timestamp: now,
            accel_body_mps2: add_vec3(truth.body_accel_mps2, imu_bias),
            gyro_body_rps: add_vec3(truth.body_rates_rps, gyro_bias),
        },
        gps,
        magnetometer: MagnetometerReading {
            timestamp: now,
            magnetic_field_body: synthetic_magnetic_field(truth.attitude_rad),
        },
        barometric_altitude_m: altitude + 4.0,
        radar_altitude_m: altitude.max(0.0),
        airspeed_mps: speed,
        vertical_speed_mps: truth.velocity_mps[2],
        downrange_m: truth.position_m[0].hypot(truth.position_m[1]),
        battery_voltage_v: if anomalies.contains(&SimAnomaly::BatterySag) {
            20.5
        } else {
            27.4 * battery_soc
        },
        battery_current_a: match phase {
            MissionPhase::Launch => 16.0 + propulsion.current_thrust_kn * 0.02,
            MissionPhase::Flight => 12.0 + propulsion.current_thrust_kn * 0.015,
            MissionPhase::Landing => 9.0 + propulsion.current_thrust_kn * 0.01,
            MissionPhase::Impact => 2.0,
            MissionPhase::PadInitialization => 6.5,
        },
        battery_soc,
        signal_strength_dbm: if anomalies.contains(&SimAnomaly::CommLoss) {
            -120.0
        } else if matches!(phase, MissionPhase::Launch | MissionPhase::Flight) {
            -72.0
        } else {
            -58.0
        },
        avionics_temp_c,
        payload_temp_c: if anomalies.contains(&SimAnomaly::ThermalRunaway) {
            64.0
        } else {
            18.0 + speed * 0.05
        },
        payload_current_a: if matches!(phase, MissionPhase::Flight) {
            2.8
        } else {
            0.6
        },
        impact_detected: phase == MissionPhase::Impact,
    }
}

fn queue_phase_events(vehicle: &mut VehicleManagementApp, phase: MissionPhase) {
    match phase {
        MissionPhase::PadInitialization => vehicle.queue_event(VehicleEvent::BootComplete),
        MissionPhase::Launch => vehicle.queue_event(VehicleEvent::EnterOperational),
        MissionPhase::Flight => vehicle.queue_event(VehicleEvent::EnterOperational),
        MissionPhase::Landing | MissionPhase::Impact => {
            vehicle.queue_event(VehicleEvent::RecoverToSafe)
        }
    }
}

fn render_downlink_frame(
    phase: MissionPhase,
    anomalies: &BTreeSet<SimAnomaly>,
    sensors: &SensorSnapshot,
) -> Vec<u8> {
    let mut line = format!(
        "phase={phase:?},alt_m={:.1},vs_mps={:.1},soc={:.2},signal_dbm={:.1}",
        sensors.radar_altitude_m,
        sensors.vertical_speed_mps,
        sensors.battery_soc,
        sensors.signal_strength_dbm
    );
    if !anomalies.is_empty() {
        let _ = write!(&mut line, ",anomalies={anomalies:?}");
    }
    line.into_bytes()
}

fn read_latest_line(
    bus: &dyn MessageBus,
    topic: &fsw_sdk_core::TopicName,
) -> Result<String, SdkError> {
    let mut latest = String::new();
    while let Some(payload) = bus.receive(topic)? {
        latest = String::from_utf8_lossy(&payload).into_owned();
    }
    Ok(latest)
}

fn lock_arc<T>(arc: &Arc<Mutex<T>>) -> Result<MutexGuard<'_, T>, SdkError> {
    arc.lock().map_err(|_| SdkError::BackendFailure)
}

fn wrap_angle(angle: f64) -> f64 {
    let mut wrapped = angle;
    while wrapped > std::f64::consts::PI {
        wrapped -= std::f64::consts::TAU;
    }
    while wrapped < -std::f64::consts::PI {
        wrapped += std::f64::consts::TAU;
    }
    wrapped
}

fn add_vec3(lhs: [f64; 3], rhs: [f64; 3]) -> [f64; 3] {
    [lhs[0] + rhs[0], lhs[1] + rhs[1], lhs[2] + rhs[2]]
}

fn norm3(vector: [f64; 3]) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

fn multiply_matrix_vec3(matrix: &[[f64; 3]; 3], vector: &[f64; 3]) -> [f64; 3] {
    [
        matrix[0][0] * vector[0] + matrix[0][1] * vector[1] + matrix[0][2] * vector[2],
        matrix[1][0] * vector[0] + matrix[1][1] * vector[1] + matrix[1][2] * vector[2],
        matrix[2][0] * vector[0] + matrix[2][1] * vector[1] + matrix[2][2] * vector[2],
    ]
}

fn transpose3(matrix: [[f64; 3]; 3]) -> [[f64; 3]; 3] {
    [
        [matrix[0][0], matrix[1][0], matrix[2][0]],
        [matrix[0][1], matrix[1][1], matrix[2][1]],
        [matrix[0][2], matrix[1][2], matrix[2][2]],
    ]
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

fn synthetic_magnetic_field(attitude: [f64; 3]) -> [f64; 3] {
    let nav_field = [0.23, 0.02, -0.41];
    let nav_to_body = transpose3(rotation_body_to_nav(attitude));
    multiply_matrix_vec3(&nav_to_body, &nav_field)
}

fn geodetic_from_local(position_m: [f64; 3]) -> GeodeticPosition {
    let east_m = position_m[0];
    let north_m = position_m[1];
    let latitude_deg = ORIGIN_LAT_DEG + north_m / EARTH_RADIUS_M * 180.0 / std::f64::consts::PI;
    let longitude_deg = ORIGIN_LON_DEG
        + east_m / (EARTH_RADIUS_M * ORIGIN_LAT_DEG.to_radians().cos()) * 180.0
            / std::f64::consts::PI;
    GeodeticPosition {
        latitude_deg,
        longitude_deg,
        altitude_m: position_m[2].max(0.0),
    }
}

fn local_from_target(target: TargetState) -> [f64; 3] {
    let north_m = (target.latitude_deg - ORIGIN_LAT_DEG).to_radians() * EARTH_RADIUS_M;
    let east_m = (target.longitude_deg - ORIGIN_LON_DEG).to_radians()
        * EARTH_RADIUS_M
        * ORIGIN_LAT_DEG.to_radians().cos();
    [east_m, north_m, target.altitude_m]
}

fn aerodynamic_state(
    body_velocity_mps: [f64; 3],
    body_rates_rps: [f64; 3],
    control_surfaces: ControlSurfaceState,
    altitude_m: f64,
    vehicle: VehicleModelConfig,
) -> AeroState {
    let air_density_kgpm3 = air_density(altitude_m);
    let airspeed_mps = norm3(body_velocity_mps).max(1.0);
    let axial_speed = body_velocity_mps[0];
    let alpha_rad = (-body_velocity_mps[2]).atan2(axial_speed.abs().max(1.0));
    let beta_rad = (body_velocity_mps[1] / airspeed_mps)
        .clamp(-0.99, 0.99)
        .asin();
    let dynamic_pressure_pa = 0.5 * air_density_kgpm3 * airspeed_mps * airspeed_mps;
    let mach = airspeed_mps / speed_of_sound(altitude_m).max(1.0);

    let reference_area_m2 = vehicle.reference_area_m2;
    let reference_span_m = vehicle.reference_span_m;
    let reference_chord_m = vehicle.reference_chord_m;

    let cd = vehicle.drag_base
        + vehicle.drag_alpha2 * alpha_rad * alpha_rad
        + vehicle.drag_beta2 * beta_rad * beta_rad
        + vehicle.drag_mach_gain * mach.min(3.0);
    let cy = vehicle.side_force_beta * beta_rad
        + vehicle.side_force_yaw_surface * control_surfaces.yaw_surface;
    let cz = vehicle.lift_alpha * alpha_rad
        + vehicle.lift_pitch_surface * control_surfaces.pitch_surface;
    let cx = -cd + 0.08 * control_surfaces.pitch_surface.abs();

    let aerodynamic_force_body_n = [
        dynamic_pressure_pa * reference_area_m2 * cx,
        dynamic_pressure_pa * reference_area_m2 * cy,
        dynamic_pressure_pa * reference_area_m2 * cz,
    ];

    let p_hat = body_rates_rps[0] * reference_span_m / (2.0 * airspeed_mps);
    let q_hat = body_rates_rps[1] * reference_chord_m / (2.0 * airspeed_mps);
    let r_hat = body_rates_rps[2] * reference_span_m / (2.0 * airspeed_mps);
    let roll_moment_coeff = vehicle.roll_moment_beta * beta_rad
        + vehicle.roll_damping_p * p_hat
        + vehicle.roll_surface_gain * control_surfaces.roll_surface;
    let pitch_moment_coeff = vehicle.pitch_moment_alpha * alpha_rad
        + vehicle.pitch_damping_q * q_hat
        + vehicle.pitch_surface_gain * control_surfaces.pitch_surface;
    let yaw_moment_coeff = vehicle.yaw_moment_beta * beta_rad
        + vehicle.yaw_damping_r * r_hat
        + vehicle.yaw_surface_gain * control_surfaces.yaw_surface;
    let aerodynamic_moment_body_nm = [
        dynamic_pressure_pa * reference_area_m2 * reference_span_m * roll_moment_coeff,
        dynamic_pressure_pa * reference_area_m2 * reference_chord_m * pitch_moment_coeff,
        dynamic_pressure_pa * reference_area_m2 * reference_span_m * yaw_moment_coeff,
    ];

    AeroState {
        air_density_kgpm3,
        dynamic_pressure_pa,
        mach,
        alpha_rad,
        beta_rad,
        body_velocity_mps,
        aerodynamic_force_body_n,
        aerodynamic_moment_body_nm,
    }
}

fn vehicle_inertia_kgm2(mass_kg: f64, vehicle: VehicleModelConfig) -> [f64; 3] {
    [
        vehicle.inertia_roll_scale * mass_kg,
        vehicle.inertia_pitch_scale * mass_kg,
        vehicle.inertia_yaw_scale * mass_kg,
    ]
}

fn angular_acceleration(
    body_rates_rps: [f64; 3],
    moment_body_nm: [f64; 3],
    inertia_kgm2: [f64; 3],
) -> [f64; 3] {
    [
        (moment_body_nm[0]
            - (inertia_kgm2[2] - inertia_kgm2[1]) * body_rates_rps[1] * body_rates_rps[2])
            / inertia_kgm2[0].max(1.0),
        (moment_body_nm[1]
            - (inertia_kgm2[0] - inertia_kgm2[2]) * body_rates_rps[2] * body_rates_rps[0])
            / inertia_kgm2[1].max(1.0),
        (moment_body_nm[2]
            - (inertia_kgm2[1] - inertia_kgm2[0]) * body_rates_rps[0] * body_rates_rps[1])
            / inertia_kgm2[2].max(1.0),
    ]
}

fn air_density(altitude_m: f64) -> f64 {
    1.225 * (-altitude_m.max(0.0) / 8_500.0).exp()
}

fn speed_of_sound(altitude_m: f64) -> f64 {
    (340.0 - altitude_m.max(0.0) * 0.003).max(295.0)
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

fn cross3(lhs: [f64; 3], rhs: [f64; 3]) -> [f64; 3] {
    [
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    ]
}

fn sanitize_finite(value: f64) -> f64 {
    if value.is_finite() {
        value
    } else {
        0.0
    }
}

fn sanitize_clamp(value: f64, min: f64, max: f64) -> f64 {
    sanitize_finite(value).clamp(min, max)
}

#[cfg(test)]
mod tests {
    use super::{
        FullStackSim, MissionPhase, PhaseControl, PropulsionState, SimAnomaly, SimConfig,
        SimulatorCommand, TargetState,
    };

    fn sub_vec3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
        [left[0] - right[0], left[1] - right[1], left[2] - right[2]]
    }

    fn norm3(value: [f64; 3]) -> f64 {
        (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]).sqrt()
    }

    fn local_from_target(target: TargetState) -> [f64; 3] {
        const EARTH_RADIUS_M: f64 = 6_371_000.0;
        const ORIGIN_LAT_DEG: f64 = 34.7420;
        const ORIGIN_LON_DEG: f64 = -120.5724;

        let north_m = (target.latitude_deg - ORIGIN_LAT_DEG).to_radians() * EARTH_RADIUS_M;
        let east_m = (target.longitude_deg - ORIGIN_LON_DEG).to_radians()
            * EARTH_RADIUS_M
            * ORIGIN_LAT_DEG.to_radians().cos();
        [east_m, north_m, target.altitude_m]
    }

    #[test]
    fn auto_mode_advances_through_launch() {
        let mut sim = FullStackSim::new(SimConfig::default()).expect("build sim");
        sim.step(60).expect("run sim");
        let snapshot = sim.snapshot();
        assert_ne!(snapshot.phase, MissionPhase::PadInitialization);
    }

    #[test]
    fn reset_clears_anomalies_and_manual_phase() {
        let mut sim = FullStackSim::new(SimConfig::default()).expect("build sim");
        sim.apply_command(SimulatorCommand::SetPhaseControl(PhaseControl::Manual))
            .expect("set control");
        sim.apply_command(SimulatorCommand::SetPhase(MissionPhase::Flight))
            .expect("set phase");
        sim.apply_command(SimulatorCommand::Inject(SimAnomaly::CommLoss))
            .expect("inject");
        sim.apply_command(SimulatorCommand::Reset).expect("reset");

        let snapshot = sim.snapshot();
        assert_eq!(snapshot.phase, MissionPhase::PadInitialization);
        assert_eq!(snapshot.phase_control, PhaseControl::Auto);
        assert!(snapshot.anomalies.is_empty());
    }

    #[test]
    fn can_set_target_and_propulsion() {
        let mut sim = FullStackSim::new(SimConfig::default()).expect("build sim");
        sim.apply_command(SimulatorCommand::SetTarget(TargetState {
            latitude_deg: 34.91,
            longitude_deg: -120.2,
            altitude_m: 120.0,
        }))
        .expect("target");
        sim.apply_command(SimulatorCommand::ConfigurePropulsion(PropulsionState {
            throttle_percent: 63,
            max_thrust_kn: 210.0,
            isp_s: 275.0,
            dry_mass_kg: 1_500.0,
            propellant_mass_kg: 700.0,
            total_mass_kg: 0.0,
            current_thrust_kn: 0.0,
            mass_flow_kgps: 0.0,
            consumption_scale: 1.2,
        }))
        .expect("propulsion");
        let snapshot = sim.snapshot();
        assert_eq!(snapshot.target.latitude_deg, 34.91);
        assert_eq!(snapshot.propulsion.throttle_percent, 63);
        assert_eq!(snapshot.propulsion.max_thrust_kn, 210.0);
    }

    #[test]
    fn launch_generates_aero_and_control_activity() {
        let mut sim = FullStackSim::new(SimConfig::default()).expect("build sim");
        sim.apply_command(SimulatorCommand::SetPhaseControl(PhaseControl::Manual))
            .expect("manual");
        sim.apply_command(SimulatorCommand::SetPhase(MissionPhase::Launch))
            .expect("launch");
        sim.step(30).expect("step launch");

        let snapshot = sim.snapshot();
        assert!(snapshot.aerodynamics.dynamic_pressure_pa > 0.0);
        assert!(snapshot.aerodynamics.mach > 0.0);
        assert!(
            snapshot.control_surfaces.roll_surface.abs()
                + snapshot.control_surfaces.pitch_surface.abs()
                + snapshot.control_surfaces.yaw_surface.abs()
                > 0.0
        );
    }

    #[test]
    fn vehicle_model_config_changes_aero_response() {
        let mut config = SimConfig::default();
        config.vehicle.reference_area_m2 = 3.6;
        config.vehicle.drag_base = 0.42;
        config.vehicle.pitch_surface_gain = 1.8;
        let mut sim = FullStackSim::new(config).expect("build sim");
        sim.apply_command(SimulatorCommand::SetPhaseControl(PhaseControl::Manual))
            .expect("manual");
        sim.apply_command(SimulatorCommand::SetPhase(MissionPhase::Launch))
            .expect("launch");
        sim.step(20).expect("step");

        let snapshot = sim.snapshot();
        assert!(snapshot.aerodynamics.dynamic_pressure_pa > 0.0);
        assert!(snapshot.aerodynamics.aerodynamic_force_body_n[0].abs() > 0.0);
        assert!(snapshot.aerodynamics.aerodynamic_moment_body_nm[1].abs() > 0.0);
    }

    #[test]
    fn flight_guidance_reduces_range_to_target() {
        let mut sim = FullStackSim::new(SimConfig::default()).expect("build sim");
        sim.apply_command(SimulatorCommand::SetPhaseControl(PhaseControl::Manual))
            .expect("manual");
        sim.apply_command(SimulatorCommand::SetPhase(MissionPhase::Launch))
            .expect("launch");
        sim.step(80).expect("climb");

        let target_local = local_from_target(sim.snapshot().target);
        let initial_range = norm3(sub_vec3(target_local, sim.snapshot().truth.position_m));

        sim.apply_command(SimulatorCommand::SetPhase(MissionPhase::Flight))
            .expect("flight");
        sim.step(120).expect("track");

        let final_range = norm3(sub_vec3(target_local, sim.snapshot().truth.position_m));
        assert!(
            final_range < initial_range,
            "{final_range} !< {initial_range}"
        );
    }

    #[test]
    fn configure_launch_vector_preserves_altitude_unless_overridden() {
        let mut sim = FullStackSim::new(SimConfig::default()).expect("build sim");
        sim.apply_command(SimulatorCommand::ConfigureLaunchVector {
            azimuth_deg: 45.0,
            elevation_deg: 20.0,
            altitude_m: None,
        })
        .expect("configure az/el");
        assert_eq!(sim.config.launch_vector.altitude_m, 0.0);
        sim.apply_command(SimulatorCommand::ConfigureLaunchVector {
            azimuth_deg: 60.0,
            elevation_deg: 10.0,
            altitude_m: Some(500.0),
        })
        .expect("configure with altitude");
        assert_eq!(sim.config.launch_vector.altitude_m, 500.0);
    }
}
