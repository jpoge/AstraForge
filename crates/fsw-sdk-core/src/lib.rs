// Copyright (c) 2026 James Pogemiller/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Core interfaces and strongly-typed contracts for AstraForge.

pub mod component;
pub mod error;
pub mod fdir;
pub mod message;
pub mod time;

pub use component::{ComponentHealth, ComponentId, ComponentLifecycleState, DependencyLevel, FswComponent};
pub use error::SdkError;
pub use fdir::{FaultClass, FaultEvent, FaultResponse};
pub use message::{MessageBus, TopicName};
pub use time::{DurationMs, MissionTime, TimeSource};
