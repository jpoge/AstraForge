// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Generic power-management template for AstraForge missions.

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{ComponentHealth, ComponentId, DependencyLevel, SdkError, TopicName};
use fsw_sdk_template_support::{publish_status_line, TemplateComponent, TemplateComponentState};

pub const POWER_COMPONENT_ID: &str = "power_management.app";
pub const POWER_STATUS_TOPIC: TopicName = TopicName("power_management.status");

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PowerSnapshot {
    pub bus_voltage_v: f64,
    pub bus_current_a: f64,
    pub battery_soc: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PowerMode {
    Nominal,
    LoadShedding,
    Survival,
}

pub struct PowerManagementApp {
    template: TemplateComponentState,
    mode: PowerMode,
    last_snapshot: Option<PowerSnapshot>,
}

impl PowerManagementApp {
    #[must_use]
    pub fn new() -> Self {
        Self {
            template: TemplateComponentState::new(POWER_COMPONENT_ID, ComponentHealth::Degraded),
            mode: PowerMode::Nominal,
            last_snapshot: None,
        }
    }

    pub fn update_snapshot(&mut self, snapshot: PowerSnapshot) {
        self.last_snapshot = Some(snapshot);
        self.template.set_health(ComponentHealth::Nominal);
    }

    #[must_use]
    pub fn mode(&self) -> PowerMode {
        self.mode
    }

    #[must_use]
    pub fn last_snapshot(&self) -> Option<PowerSnapshot> {
        self.last_snapshot
    }

    fn evaluate_power_mode(&mut self) {
        // Replace these placeholder thresholds with mission-specific policy.
        // This is also the place to call a real EPS/BMS driver or power board HAL.
        if let Some(snapshot) = self.last_snapshot {
            self.mode = if snapshot.battery_soc < 0.15 {
                PowerMode::Survival
            } else if snapshot.battery_soc < 0.35 {
                PowerMode::LoadShedding
            } else {
                PowerMode::Nominal
            };
        }
    }
}

impl Default for PowerManagementApp {
    fn default() -> Self {
        Self::new()
    }
}

impl MissionApp for PowerManagementApp {
    fn name(&self) -> &'static str {
        self.template.component_name()
    }

    fn step(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.evaluate_power_mode();

        // Add commanded power-rail switching, load shedding, or battery balancing here.
        // In a real mission this hook is where you fan out actions to EPS hardware or
        // publish inhibit commands for non-essential subsystems.

        publish_status_line(
            ctx,
            &POWER_STATUS_TOPIC,
            format!(
                "mode={:?},snapshot={:?},health={:?}",
                self.mode,
                self.last_snapshot,
                self.template.health()
            ),
        )
    }
}

impl TemplateComponent for PowerManagementApp {
    fn template_state(&self) -> &TemplateComponentState {
        &self.template
    }
}

impl fsw_sdk_core::FswComponent for PowerManagementApp {
    fn id(&self) -> ComponentId {
        self.template.component_id()
    }

    fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        self.mode = PowerMode::Nominal;
        Ok(())
    }

    fn start(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        self.mode = PowerMode::Nominal;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.last_snapshot = None;
        self.template.set_health(ComponentHealth::Degraded);
        self.mode = PowerMode::Nominal;
        Ok(())
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn health(&self) -> ComponentHealth {
        self.template.health()
    }
}
