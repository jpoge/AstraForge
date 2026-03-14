// Copyright (c) 2026 James Pogemiller/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Simple GNC mission application built on top of AstraForge.

mod component;
mod config;
mod ekf;

pub use component::{
    new_shared_gnc_component, GncComponent, GncComponentProxy, GncConfig, SharedGncComponent,
    GNC_COMPONENT_ID, GNC_SOLUTION_TOPIC,
};
pub use config::MissionConfig;
pub use ekf::{
    EkfTuning, ExtendedKalmanFilter, GpsReading, ImuReading, MagnetometerReading,
    NavigationSolution,
};
