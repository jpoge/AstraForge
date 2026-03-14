// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Time source implementations.

use std::sync::atomic::{AtomicU64, Ordering};
use std::thread;
use std::time::{Duration, Instant};

use fsw_sdk_core::{DurationMs, MissionTime, TimeSource};

/// Deterministic clock advanced only by explicit ticks.
pub struct StaticClock {
    now_ms: AtomicU64,
}

impl StaticClock {
    /// Creates a new static clock at zero.
    #[must_use]
    pub fn new() -> Self {
        Self {
            now_ms: AtomicU64::new(0),
        }
    }
}

impl Default for StaticClock {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeSource for StaticClock {
    fn now_monotonic(&self) -> MissionTime {
        MissionTime(self.now_ms.load(Ordering::Relaxed))
    }

    fn sleep_until(&self, target: MissionTime) {
        loop {
            if self.now_monotonic().0 >= target.0 {
                break;
            }
            thread::yield_now();
        }
    }

    fn tick(&self, delta: DurationMs) {
        self.now_ms.fetch_add(delta.0, Ordering::Relaxed);
    }
}

/// Host wall-clock backed implementation.
pub struct SystemClock {
    start: Instant,
}

impl SystemClock {
    /// Creates a new system clock anchored at creation.
    #[must_use]
    pub fn new() -> Self {
        Self {
            start: Instant::now(),
        }
    }
}

impl Default for SystemClock {
    fn default() -> Self {
        Self::new()
    }
}

impl TimeSource for SystemClock {
    fn now_monotonic(&self) -> MissionTime {
        let elapsed = self.start.elapsed();
        let ms = elapsed.as_millis().min(u128::from(u64::MAX)) as u64;
        MissionTime(ms)
    }

    fn sleep_until(&self, target: MissionTime) {
        let now = self.now_monotonic().0;
        if target.0 > now {
            thread::sleep(Duration::from_millis(target.0 - now));
        }
    }

    fn tick(&self, _delta: DurationMs) {}
}
