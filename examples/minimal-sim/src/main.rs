// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_core::{
    ComponentHealth, ComponentId, DependencyLevel, DurationMs, FaultClass, FaultEvent,
    FswComponent, MissionTime, SdkError, TimeSource,
};
use fsw_sdk_fdir::ClassMapPolicy;
use fsw_sdk_runtime::{Runtime, SchedulerConfig};
use fsw_sdk_time::StaticClock;

struct DemoComponent {
    health: ComponentHealth,
}

impl DemoComponent {
    fn new() -> Self {
        Self {
            health: ComponentHealth::Nominal,
        }
    }
}

impl FswComponent for DemoComponent {
    fn id(&self) -> ComponentId {
        ComponentId("demo.component".to_string())
    }

    fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)> {
        Vec::new()
    }

    fn init(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn start(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn stop(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn reset(&mut self) -> Result<(), SdkError> {
        self.health = ComponentHealth::Nominal;
        Ok(())
    }

    fn deinit(&mut self) -> Result<(), SdkError> {
        Ok(())
    }

    fn health(&self) -> ComponentHealth {
        self.health
    }
}

fn main() -> Result<(), SdkError> {
    let clock = StaticClock::new();
    let policy = ClassMapPolicy::conservative();

    let mut runtime = Runtime::new(policy);
    runtime.set_scheduler_config(SchedulerConfig {
        period_ms: 100,
        heartbeat_timeout_ms: 150,
    });

    let manifest = include_str!("../module.yaml");
    runtime.register_with_manifest(Box::new(DemoComponent::new()), manifest)?;
    runtime.start_all()?;

    runtime.mark_heartbeat(&ComponentId("demo.component".to_string()), MissionTime(0))?;

    clock.tick(DurationMs(200));
    let watchdog_responses = runtime.supervisor_step(clock.now_monotonic())?;
    println!("Watchdog responses: {:?}", watchdog_responses);

    let fault = FaultEvent {
        source: ComponentId("demo.component".to_string()),
        class: FaultClass::Transient,
        detected_at: MissionTime(200),
        reason: "demo_fault",
    };

    let response = runtime.handle_fault(&fault)?;
    println!("FDIR response: {:?}", response);
    println!("Safe mode: {}", runtime.in_safe_mode());

    runtime.stop_all()?;
    Ok(())
}
