// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Generic vehicle-management template with a mission-mode state machine.

use std::collections::VecDeque;

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{ComponentHealth, ComponentId, DependencyLevel, SdkError, TopicName};
use fsw_sdk_template_support::{publish_status_line, TemplateComponent, TemplateComponentState};

pub const VEHICLE_MGMT_COMPONENT_ID: &str = "vehicle_management.app";
pub const VEHICLE_MODE_TOPIC: TopicName = TopicName("vehicle_management.mode");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleMode {
    Boot,
    Safe,
    Standby,
    Operational,
    Survival,
    Fault,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VehicleEvent {
    BootComplete,
    EnterOperational,
    FaultDetected,
    PowerEmergency,
    ThermalEmergency,
    RecoverToSafe,
}

pub struct VehicleManagementApp {
    template: TemplateComponentState,
    mode: VehicleMode,
    pending_events: VecDeque<VehicleEvent>,
}

impl VehicleManagementApp {
    #[must_use]
    pub fn new() -> Self {
        Self {
            template: TemplateComponentState::new(
                VEHICLE_MGMT_COMPONENT_ID,
                ComponentHealth::Nominal,
            ),
            mode: VehicleMode::Boot,
            pending_events: VecDeque::new(),
        }
    }

    pub fn queue_event(&mut self, event: VehicleEvent) {
        self.pending_events.push_back(event);
    }

    #[must_use]
    pub fn mode(&self) -> VehicleMode {
        self.mode
    }

    fn collect_mode_inputs(&mut self) {
        // Connect subsystem health and autonomy inputs here.
        // Typical sources:
        // - power-management status
        // - thermal-management limits
        // - communications arm/disarm commands
        // - payload readiness / inhibit signals
        // Convert those signals into `VehicleEvent` values and queue them.
    }

    fn apply_event(&mut self, event: VehicleEvent) {
        self.mode = match (self.mode, event) {
            (VehicleMode::Boot, VehicleEvent::BootComplete) => VehicleMode::Safe,
            (VehicleMode::Safe, VehicleEvent::EnterOperational) => VehicleMode::Standby,
            (VehicleMode::Standby, VehicleEvent::EnterOperational) => VehicleMode::Operational,
            (_, VehicleEvent::PowerEmergency | VehicleEvent::ThermalEmergency) => VehicleMode::Survival,
            (_, VehicleEvent::FaultDetected) => VehicleMode::Fault,
            (_, VehicleEvent::RecoverToSafe) => VehicleMode::Safe,
            (mode, _) => mode,
        };
    }

    fn publish_mode(&self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        publish_status_line(ctx, &VEHICLE_MODE_TOPIC, format!("mode={:?}", self.mode))
    }
}

impl Default for VehicleManagementApp {
    fn default() -> Self {
        Self::new()
    }
}

impl MissionApp for VehicleManagementApp {
    fn name(&self) -> &'static str {
        self.template.component_name()
    }

    fn step(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.collect_mode_inputs();
        while let Some(event) = self.pending_events.pop_front() {
            self.apply_event(event);
        }

        // Add mode-entry/exit actions here.
        // This is the right location to trigger enables, inhibits, safing actions,
        // or component command broadcasts associated with each vehicle mode.

        self.publish_mode(ctx)
    }
}

impl TemplateComponent for VehicleManagementApp {
    fn template_state(&self) -> &TemplateComponentState {
        &self.template
    }
}

impl fsw_sdk_core::FswComponent for VehicleManagementApp {
    fn id(&self) -> ComponentId {
        self.template.component_id()
    }

    fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        self.mode = VehicleMode::Boot;
        Ok(())
    }

    fn start(&mut self) -> Result<(), SdkError> {
        self.pending_events.push_back(VehicleEvent::BootComplete);
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        self.mode = VehicleMode::Safe;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.pending_events.clear();
        self.mode = VehicleMode::Boot;
        self.template.set_health(ComponentHealth::Nominal);
        Ok(())
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        self.mode = VehicleMode::Boot;
        Ok(())
    }

    fn health(&self) -> ComponentHealth {
        self.template.health()
    }
}

#[cfg(test)]
mod tests {
    use super::{VehicleEvent, VehicleManagementApp, VehicleMode};

    #[test]
    fn state_machine_reaches_operational_via_safe_and_standby() {
        let mut app = VehicleManagementApp::new();
        app.queue_event(VehicleEvent::BootComplete);
        app.queue_event(VehicleEvent::EnterOperational);
        app.queue_event(VehicleEvent::EnterOperational);

        while let Some(event) = app.pending_events.pop_front() {
            app.apply_event(event);
        }

        assert_eq!(app.mode, VehicleMode::Operational);
    }
}
