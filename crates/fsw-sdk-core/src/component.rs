// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Component lifecycle and dependency contracts.

use crate::error::SdkError;

/// Stable component identifier.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ComponentId(pub String);

/// Dependency importance for fault isolation decisions.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DependencyLevel {
    /// Optional dependency; component can run without it.
    Optional,
    /// Required dependency; component should be isolated if absent.
    Required,
}

/// Runtime lifecycle state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentLifecycleState {
    /// Constructed but not initialized.
    Created,
    /// Initialized and ready to run.
    Initialized,
    /// Actively running.
    Running,
    /// Temporarily stopped.
    Stopped,
    /// Isolated by FDIR supervisor.
    Isolated,
}

/// Health status reported to FDIR.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ComponentHealth {
    /// Component reports nominal behavior.
    Nominal,
    /// Degraded behavior but still operational.
    Degraded,
    /// Component is non-operational.
    Failed,
}

/// Lifecycle contract all modules must implement.
pub trait FswComponent: Send {
    /// Returns stable component id.
    fn id(&self) -> ComponentId;

    /// Returns dependencies and required level.
    fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)>;

    /// Initializes component resources.
    fn init(&mut self) -> Result<(), SdkError>;

    /// Starts component execution.
    fn start(&mut self) -> Result<(), SdkError>;

    /// Stops execution but keeps state.
    fn stop(&mut self) -> Result<(), SdkError>;

    /// Performs internal reset for recovery.
    fn reset(&mut self) -> Result<(), SdkError>;

    /// Deinitializes resources.
    fn deinit(&mut self) -> Result<(), SdkError>;

    /// Reports current health status.
    fn health(&self) -> ComponentHealth;
}
