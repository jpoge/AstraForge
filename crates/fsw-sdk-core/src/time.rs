// Copyright (c) 2026 James Pogemiller/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Time primitives shared across all layers.

/// Mission-relative monotonic timestamp in milliseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct MissionTime(pub u64);

/// Duration in milliseconds.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct DurationMs(pub u64);

/// Abstract clock contract.
pub trait TimeSource: Send + Sync {
    /// Returns current monotonic mission time.
    fn now_monotonic(&self) -> MissionTime;

    /// Sleeps until absolute mission time.
    fn sleep_until(&self, target: MissionTime);

    /// Advances time by a fixed amount. No-op for real clocks.
    fn tick(&self, delta: DurationMs);
}
