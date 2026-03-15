// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Generic communications app template for AstraForge missions.

use std::collections::VecDeque;

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{ComponentHealth, ComponentId, DependencyLevel, SdkError, TopicName};
use fsw_sdk_template_support::{publish_status_line, TemplateComponent, TemplateComponentState};

pub const COMM_COMPONENT_ID: &str = "communications.app";
pub const COMM_UPLINK_TOPIC: TopicName = TopicName("communications.uplink");
pub const COMM_DOWNLINK_TOPIC: TopicName = TopicName("communications.downlink");
pub const COMM_STATUS_TOPIC: TopicName = TopicName("communications.status");

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkStatus {
    Offline,
    Standby,
    Online,
    Degraded,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CommunicationConfig {
    pub max_downlink_frames_per_step: usize,
}

impl Default for CommunicationConfig {
    fn default() -> Self {
        Self {
            max_downlink_frames_per_step: 4,
        }
    }
}

pub struct CommunicationApp {
    config: CommunicationConfig,
    template: TemplateComponentState,
    link_status: LinkStatus,
    uplink_frames: VecDeque<Vec<u8>>,
    downlink_frames: VecDeque<Vec<u8>>,
}

impl CommunicationApp {
    #[must_use]
    pub fn new(config: CommunicationConfig) -> Self {
        Self {
            config,
            template: TemplateComponentState::new(COMM_COMPONENT_ID, ComponentHealth::Nominal),
            link_status: LinkStatus::Standby,
            uplink_frames: VecDeque::new(),
            downlink_frames: VecDeque::new(),
        }
    }

    pub fn queue_downlink_frame(&mut self, frame: Vec<u8>) {
        self.downlink_frames.push_back(frame);
    }

    pub fn take_uplink_frame(&mut self) -> Option<Vec<u8>> {
        self.uplink_frames.pop_front()
    }

    #[must_use]
    pub fn link_status(&self) -> LinkStatus {
        self.link_status
    }

    pub fn set_link_status(&mut self, status: LinkStatus) {
        self.link_status = status;
        self.template.set_health(
            if matches!(status, LinkStatus::Degraded | LinkStatus::Offline) {
                ComponentHealth::Degraded
            } else {
                ComponentHealth::Nominal
            },
        );
    }

    fn poll_external_link(&mut self) {
        // Connect your radio/modem/socket driver here.
        // Typical pattern:
        // 1. poll the physical or transport link
        // 2. validate framing / CRC / authentication
        // 3. push accepted frames into `uplink_frames`
        // 4. set `link_status` and `health` based on link quality
    }

    fn flush_downlink(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        // Replace this bus publication with the real transmission path
        // once your mission has a flight or ground communications driver.
        for _ in 0..self.config.max_downlink_frames_per_step {
            let Some(frame) = self.downlink_frames.pop_front() else {
                break;
            };
            ctx.bus.publish(&COMM_DOWNLINK_TOPIC, frame)?;
        }
        Ok(())
    }

    fn publish_status(&self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        publish_status_line(
            ctx,
            &COMM_STATUS_TOPIC,
            format!(
                "link_status={:?},health={:?}",
                self.link_status,
                self.template.health()
            ),
        )
    }
}

impl MissionApp for CommunicationApp {
    fn name(&self) -> &'static str {
        self.template.component_name()
    }

    fn step(&mut self, ctx: &AppContext<'_>) -> Result<(), SdkError> {
        self.poll_external_link();

        while let Some(frame) = ctx.bus.receive(&COMM_UPLINK_TOPIC)? {
            // This hook is where a mission would route validated uplink frames
            // from a driver task or test harness into higher-level parsing logic.
            self.uplink_frames.push_back(frame);
        }

        self.flush_downlink(ctx)?;
        self.publish_status(ctx)?;
        Ok(())
    }
}

impl TemplateComponent for CommunicationApp {
    fn template_state(&self) -> &TemplateComponentState {
        &self.template
    }
}

impl fsw_sdk_core::FswComponent for CommunicationApp {
    fn id(&self) -> ComponentId {
        self.template.component_id()
    }

    fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        self.link_status = LinkStatus::Standby;
        Ok(())
    }

    fn start(&mut self) -> Result<(), SdkError> {
        self.link_status = LinkStatus::Online;
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        self.link_status = LinkStatus::Offline;
        Ok(())
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.uplink_frames.clear();
        self.downlink_frames.clear();
        self.link_status = LinkStatus::Standby;
        self.template.set_health(ComponentHealth::Nominal);
        Ok(())
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        self.link_status = LinkStatus::Offline;
        Ok(())
    }

    fn health(&self) -> ComponentHealth {
        self.template.health()
    }
}
