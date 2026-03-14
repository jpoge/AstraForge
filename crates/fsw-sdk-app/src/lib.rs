// Copyright (c) 2026 James Pogemiller/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Mission application interface crate.

use fsw_sdk_core::{MessageBus, SdkError, TimeSource};

/// Narrow host context passed to mission applications.
pub struct AppContext<'a> {
    /// Shared message bus.
    pub bus: &'a dyn MessageBus,
    /// Shared time source.
    pub time: &'a dyn TimeSource,
}

/// Mission-level application contract.
pub trait MissionApp: Send {
    /// Returns stable application name.
    fn name(&self) -> &'static str;

    /// Called once on app deployment.
    fn init(&mut self, _ctx: &AppContext<'_>) -> Result<(), SdkError> {
        Ok(())
    }

    /// Called at a scheduler-controlled cadence.
    fn step(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError>;

    /// Called once before app removal.
    fn shutdown(&mut self, _ctx: &AppContext<'_>) -> Result<(), SdkError> {
        Ok(())
    }
}
