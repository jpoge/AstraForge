// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Simple GNC mission application built on top of AstraForge.

mod component;
mod config;
mod ekf;
mod launcher;

pub use component::{
    new_shared_gnc_component, ControlGains, GncCommand, GncComponent, GncComponentProxy, GncConfig,
    GuidanceMode, GuidanceReference, SharedGncComponent, GNC_COMMAND_TOPIC, GNC_COMPONENT_ID,
    GNC_SOLUTION_TOPIC,
};
pub use config::MissionConfig;
pub use ekf::{
    EkfTuning, ExtendedKalmanFilter, GpsReading, ImuReading, MagnetometerReading,
    NavigationSolution,
};
pub use launcher::{run_mission, LaunchReport};
