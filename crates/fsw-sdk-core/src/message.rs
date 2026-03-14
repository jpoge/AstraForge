// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Message bus abstractions.

use crate::error::SdkError;

/// Strongly-typed message topic.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct TopicName(pub &'static str);

/// Minimal bus API for command and telemetry exchange.
pub trait MessageBus: Send + Sync {
    /// Publishes opaque payload bytes.
    fn publish(&self, topic: &TopicName, payload: Vec<u8>) -> Result<(), SdkError>;

    /// Receives the next payload from a topic.
    fn receive(&self, topic: &TopicName) -> Result<Option<Vec<u8>>, SdkError>;
}
