// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! FDIR policy and supervisor primitives.

use std::collections::HashMap;

use fsw_sdk_core::{ComponentId, FaultClass, FaultEvent, FaultResponse};

/// Policy contract for mapping faults to responses.
pub trait FdirPolicy: Send + Sync {
    /// Returns response action for an observed fault.
    fn select_response(&self, event: &FaultEvent) -> FaultResponse;
}

/// Static class-to-response map policy.
pub struct ClassMapPolicy {
    map: HashMap<FaultClass, FaultResponse>,
}

impl ClassMapPolicy {
    /// Creates default policy with conservative behavior.
    #[must_use]
    pub fn conservative() -> Self {
        let mut map = HashMap::new();
        map.insert(FaultClass::Transient, FaultResponse::Retry);
        map.insert(FaultClass::Persistent, FaultResponse::Isolate);
        map.insert(FaultClass::Critical, FaultResponse::EnterSafeMode);
        Self { map }
    }

    /// Overrides one class behavior.
    #[must_use]
    pub fn with_rule(mut self, class: FaultClass, response: FaultResponse) -> Self {
        self.map.insert(class, response);
        self
    }
}

impl FdirPolicy for ClassMapPolicy {
    fn select_response(&self, event: &FaultEvent) -> FaultResponse {
        self.map
            .get(&event.class)
            .copied()
            .unwrap_or(FaultResponse::Isolate)
    }
}

/// Fault counter for coarse isolation decisions.
#[derive(Default)]
pub struct FaultCounter {
    counts: HashMap<ComponentId, u32>,
}

impl FaultCounter {
    /// Increments and returns fault count for component.
    pub fn record(&mut self, source: &ComponentId) -> u32 {
        let entry = self.counts.entry(source.clone()).or_insert(0);
        *entry += 1;
        *entry
    }

    /// Returns current count for component.
    #[must_use]
    pub fn count(&self, source: &ComponentId) -> u32 {
        self.counts.get(source).copied().unwrap_or(0)
    }
}
