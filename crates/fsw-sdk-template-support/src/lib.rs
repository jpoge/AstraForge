// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Shared support utilities for AstraForge application templates.

use fsw_sdk_app::AppContext;
use fsw_sdk_core::{ComponentHealth, ComponentId, SdkError, TopicName};

/// Common lifecycle state carried by template applications.
#[derive(Debug, Clone)]
pub struct TemplateComponentState {
    component_id: &'static str,
    health: ComponentHealth,
}

impl TemplateComponentState {
    /// Creates template state with the provided component id and initial health.
    #[must_use]
    pub fn new(component_id: &'static str, initial_health: ComponentHealth) -> Self {
        Self {
            component_id,
            health: initial_health,
        }
    }

    /// Returns the component id as a typed `ComponentId`.
    #[must_use]
    pub fn component_id(&self) -> ComponentId {
        ComponentId(self.component_id.to_string())
    }

    /// Returns the raw component id string.
    #[must_use]
    pub fn component_name(&self) -> &'static str {
        self.component_id
    }

    /// Updates template health.
    pub fn set_health(&mut self, health: ComponentHealth) {
        self.health = health;
    }

    /// Returns current template health.
    #[must_use]
    pub fn health(&self) -> ComponentHealth {
        self.health
    }
}

/// Publishes a string status payload to a topic.
pub fn publish_status_line(
    ctx: &AppContext<'_>,
    topic: &TopicName,
    payload: impl Into<String>,
) -> Result<(), SdkError> {
    ctx.bus.publish(topic, payload.into().into_bytes())
}

/// Trait exposing shared template state to application-specific implementations.
pub trait TemplateComponent: Send {
    /// Returns shared state for the template.
    fn template_state(&self) -> &TemplateComponentState;
}
