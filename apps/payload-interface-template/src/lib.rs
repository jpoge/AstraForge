// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Generic payload-interface template for AstraForge missions.

use std::collections::VecDeque;

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{ComponentHealth, ComponentId, DependencyLevel, SdkError, TopicName};
use fsw_sdk_template_support::{publish_status_line, TemplateComponent, TemplateComponentState};

pub const PAYLOAD_COMPONENT_ID: &str = "payload_interface.app";
pub const PAYLOAD_COMMAND_TOPIC: TopicName = TopicName("payload.command");
pub const PAYLOAD_STATUS_TOPIC: TopicName = TopicName("payload.status");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PayloadMode {
    Idle,
    Configuring,
    Active,
    Fault,
}

pub struct PayloadInterfaceApp {
    template: TemplateComponentState,
    mode: PayloadMode,
    pending_commands: VecDeque<Vec<u8>>,
}

impl PayloadInterfaceApp {
    #[must_use]
    pub fn new() -> Self {
        Self {
            template: TemplateComponentState::new(PAYLOAD_COMPONENT_ID, ComponentHealth::Nominal),
            mode: PayloadMode::Idle,
            pending_commands: VecDeque::new(),
        }
    }

    fn poll_payload_link(&mut self) {
        // Connect the physical payload transport here.
        // Examples:
        // - SPI/UART/CAN transaction layer
        // - file or socket proxy for lab simulations
        // - payload-specific packet framing / parsing
    }

    fn service_commands(&mut self) {
        while let Some(_command) = self.pending_commands.pop_front() {
            // Replace this stub with payload-specific command translation.
            // Typical work:
            // - decode command opcodes / arguments
            // - validate payload mode and power/thermal interlocks
            // - send one or more transactions to the payload controller
            self.mode = PayloadMode::Active;
        }
    }

    #[must_use]
    pub fn mode(&self) -> PayloadMode {
        self.mode
    }

    pub fn inject_fault(&mut self) {
        self.mode = PayloadMode::Fault;
        self.template.set_health(ComponentHealth::Failed);
    }

    pub fn clear_fault(&mut self) {
        if self.mode == PayloadMode::Fault {
            self.mode = PayloadMode::Idle;
        }
        self.template.set_health(ComponentHealth::Nominal);
    }
}

impl Default for PayloadInterfaceApp {
    fn default() -> Self {
        Self::new()
    }
}

impl MissionApp for PayloadInterfaceApp {
    fn name(&self) -> &'static str {
        self.template.component_name()
    }

    fn step(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.poll_payload_link();
        while let Some(command) = ctx.bus.receive(&PAYLOAD_COMMAND_TOPIC)? {
            self.pending_commands.push_back(command);
        }
        self.service_commands();

        publish_status_line(
            ctx,
            &PAYLOAD_STATUS_TOPIC,
            format!("mode={:?},health={:?}", self.mode, self.template.health()),
        )
    }
}

impl TemplateComponent for PayloadInterfaceApp {
    fn template_state(&self) -> &TemplateComponentState {
        &self.template
    }
}

impl fsw_sdk_core::FswComponent for PayloadInterfaceApp {
    fn id(&self) -> ComponentId {
        self.template.component_id()
    }

    fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        self.mode = PayloadMode::Idle;
        Ok(())
    }

    fn start(&mut self) -> Result<(), SdkError> {
        self.mode = PayloadMode::Configuring;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        self.mode = PayloadMode::Idle;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.pending_commands.clear();
        self.mode = PayloadMode::Idle;
        self.template.set_health(ComponentHealth::Nominal);
        Ok(())
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        self.mode = PayloadMode::Idle;
        Ok(())
    }

    fn health(&self) -> ComponentHealth {
        self.template.health()
    }
}
