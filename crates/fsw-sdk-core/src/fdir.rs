// Copyright (c) 2026 James Pogemiller/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Fault-related contracts consumed by runtime and applications.

use crate::{component::ComponentId, time::MissionTime};

/// Fault severity class used by policy engines.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub enum FaultClass {
    /// Recoverable and expected in nominal operations.
    Transient,
    /// Repeated or long-lived failure requiring isolation.
    Persistent,
    /// Safety-critical fault requiring immediate safe response.
    Critical,
}

/// Normalized fault event record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FaultEvent {
    /// Fault source component.
    pub source: ComponentId,
    /// Fault class for policy selection.
    pub class: FaultClass,
    /// Mission time of detection.
    pub detected_at: MissionTime,
    /// Short machine-readable reason.
    pub reason: &'static str,
}

/// Action requested by FDIR policy.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultResponse {
    /// Attempt local retry.
    Retry,
    /// Restart the failed component.
    Restart,
    /// Isolate component from mission graph.
    Isolate,
    /// Enter system safe mode.
    EnterSafeMode,
}
