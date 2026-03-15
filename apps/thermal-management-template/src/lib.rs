// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Generic thermal-management template for AstraForge missions.

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{ComponentHealth, ComponentId, DependencyLevel, SdkError, TopicName};
use fsw_sdk_template_support::{publish_status_line, TemplateComponent, TemplateComponentState};

pub const THERMAL_COMPONENT_ID: &str = "thermal_management.app";
pub const THERMAL_STATUS_TOPIC: TopicName = TopicName("thermal_management.status");

#[derive(Debug, Clone, PartialEq)]
pub struct ThermalZone {
    pub name: String,
    pub temperature_c: f64,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ThermalMode {
    Nominal,
    Heating,
    Cooling,
    Survival,
}

pub struct ThermalManagementApp {
    template: TemplateComponentState,
    mode: ThermalMode,
    zones: Vec<ThermalZone>,
}

impl ThermalManagementApp {
    #[must_use]
    pub fn new() -> Self {
        Self {
            template: TemplateComponentState::new(THERMAL_COMPONENT_ID, ComponentHealth::Degraded),
            mode: ThermalMode::Nominal,
            zones: Vec::new(),
        }
    }

    pub fn update_zone_temperature(&mut self, zone_name: impl Into<String>, temperature_c: f64) {
        let zone_name = zone_name.into();
        if let Some(zone) = self.zones.iter_mut().find(|zone| zone.name == zone_name) {
            zone.temperature_c = temperature_c;
        } else {
            self.zones.push(ThermalZone {
                name: zone_name,
                temperature_c,
            });
        }
        self.template.set_health(ComponentHealth::Nominal);
    }

    #[must_use]
    pub fn mode(&self) -> ThermalMode {
        self.mode
    }

    #[must_use]
    pub fn zones(&self) -> &[ThermalZone] {
        &self.zones
    }

    fn evaluate_mode(&mut self) {
        // Replace these placeholder limits with your actual thermal policy.
        // This is also where a mission should ingest heater controller data,
        // radiator loop telemetry, and survival heater inhibits.
        let hottest = self
            .zones
            .iter()
            .map(|zone| zone.temperature_c)
            .reduce(f64::max)
            .unwrap_or(20.0);
        let coldest = self
            .zones
            .iter()
            .map(|zone| zone.temperature_c)
            .reduce(f64::min)
            .unwrap_or(20.0);

        self.mode = if hottest > 60.0 {
            ThermalMode::Cooling
        } else if coldest < -10.0 {
            ThermalMode::Heating
        } else {
            ThermalMode::Nominal
        };
    }
}

impl Default for ThermalManagementApp {
    fn default() -> Self {
        Self::new()
    }
}

impl MissionApp for ThermalManagementApp {
    fn name(&self) -> &'static str {
        self.template.component_name()
    }

    fn step(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.evaluate_mode();

        // Hook thermal actuators here.
        // Typical integrations:
        // - heater PWM or relay outputs
        // - louver or valve commands
        // - thermal safing notifications to vehicle management

        publish_status_line(
            ctx,
            &THERMAL_STATUS_TOPIC,
            format!(
                "mode={:?},zones={:?},health={:?}",
                self.mode,
                self.zones,
                self.template.health()
            ),
        )
    }
}

impl TemplateComponent for ThermalManagementApp {
    fn template_state(&self) -> &TemplateComponentState {
        &self.template
    }
}

impl fsw_sdk_core::FswComponent for ThermalManagementApp {
    fn id(&self) -> ComponentId {
        self.template.component_id()
    }

    fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        self.mode = ThermalMode::Nominal;
        Ok(())
    }

    fn start(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        self.mode = ThermalMode::Nominal;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.zones.clear();
        self.mode = ThermalMode::Nominal;
        self.template.set_health(ComponentHealth::Degraded);
        Ok(())
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn health(&self) -> ComponentHealth {
        self.template.health()
    }
}
