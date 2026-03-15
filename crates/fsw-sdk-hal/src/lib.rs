// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! HAL-style platform boundary for AstraForge applications.

use fsw_sdk_core::{MessageBus, TimeSource};
use fsw_sdk_runtime::InMemoryBus;
use fsw_sdk_time::StaticClock;

/// Minimal platform abstraction used by mission launchers.
pub trait HalPlatform {
    /// Returns the shared clock for the platform.
    fn time_source(&self) -> &dyn TimeSource;

    /// Returns the shared message bus for the platform.
    fn message_bus(&self) -> &dyn MessageBus;
}

/// Host simulation platform using in-memory runtime services.
#[derive(Default)]
pub struct HostSimPlatform {
    clock: StaticClock,
    bus: InMemoryBus,
}

impl HostSimPlatform {
    /// Creates a host simulation platform with in-memory services.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Returns the concrete static clock used by the platform.
    #[must_use]
    pub fn clock(&self) -> &StaticClock {
        &self.clock
    }

    /// Returns the concrete in-memory bus used by the platform.
    #[must_use]
    pub fn bus(&self) -> &InMemoryBus {
        &self.bus
    }
}

impl HalPlatform for HostSimPlatform {
    fn time_source(&self) -> &dyn TimeSource {
        &self.clock
    }

    fn message_bus(&self) -> &dyn MessageBus {
        &self.bus
    }
}
