// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Component runtime with lifecycle management, manifest registration, and supervision.

use std::collections::{HashMap, VecDeque};
use std::sync::{Arc, Mutex};

use fsw_sdk_core::{
    ComponentHealth, ComponentId, ComponentLifecycleState, DependencyLevel, DurationMs, FaultClass,
    FaultEvent, FaultResponse, FswComponent, MessageBus, MissionTime, SdkError, TimeSource,
    TopicName,
};
use fsw_sdk_fdir::FdirPolicy;

/// Runtime scheduler and watchdog parameters.
#[derive(Debug, Clone, Copy)]
pub struct SchedulerConfig {
    /// Fixed supervisor period in milliseconds.
    pub period_ms: u64,
    /// Maximum heartbeat age before a fault is raised.
    pub heartbeat_timeout_ms: u64,
}

impl Default for SchedulerConfig {
    fn default() -> Self {
        Self {
            period_ms: 100,
            heartbeat_timeout_ms: 500,
        }
    }
}

/// Parsed module manifest data.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ModuleManifest {
    /// Module id, expected to match component id.
    pub id: String,
    /// Required dependencies.
    pub requires: Vec<String>,
    /// Optional dependencies.
    pub optional: Vec<String>,
}

impl ModuleManifest {
    /// Parses a constrained YAML subset for module declarations.
    pub fn parse_yaml(input: &str) -> Result<Self, SdkError> {
        let mut id: Option<String> = None;
        let mut requires: Vec<String> = Vec::new();
        let mut optional: Vec<String> = Vec::new();
        let mut section: Option<&str> = None;

        for raw_line in input.lines() {
            let line = raw_line.trim();
            if line.is_empty() || line.starts_with('#') {
                continue;
            }

            if let Some(rest) = line.strip_prefix("id:") {
                id = Some(rest.trim().to_string());
                section = None;
                continue;
            }

            if line == "requires:" {
                section = Some("requires");
                continue;
            }

            if line == "optional:" {
                section = Some("optional");
                continue;
            }

            if let Some(value) = line.strip_prefix("-") {
                let dep = value.trim().to_string();
                match section {
                    Some("requires") => requires.push(dep),
                    Some("optional") => optional.push(dep),
                    _ => return Err(SdkError::ManifestParse),
                }
                continue;
            }

            return Err(SdkError::ManifestParse);
        }

        let module_id = id.ok_or(SdkError::ManifestParse)?;
        if module_id.is_empty() {
            return Err(SdkError::ManifestParse);
        }

        Ok(Self {
            id: module_id,
            requires,
            optional,
        })
    }
}

struct ComponentEntry {
    state: ComponentLifecycleState,
    component: Box<dyn FswComponent>,
    manifest: Option<ModuleManifest>,
}

/// In-memory message bus for simulation and unit tests.
#[derive(Default)]
pub struct InMemoryBus {
    topics: Arc<Mutex<HashMap<TopicName, VecDeque<Vec<u8>>>>>,
}

impl InMemoryBus {
    /// Creates a new empty in-memory bus.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }
}

impl MessageBus for InMemoryBus {
    fn publish(&self, topic: &TopicName, payload: Vec<u8>) -> Result<(), SdkError> {
        let mut map = self.topics.lock().map_err(|_| SdkError::BackendFailure)?;
        map.entry(topic.clone()).or_default().push_back(payload);
        Ok(())
    }

    fn receive(&self, topic: &TopicName) -> Result<Option<Vec<u8>>, SdkError> {
        let mut map = self.topics.lock().map_err(|_| SdkError::BackendFailure)?;
        Ok(map.get_mut(topic).and_then(VecDeque::pop_front))
    }
}

/// Runtime kernel hosting components and enforcing policy.
pub struct Runtime<P: FdirPolicy> {
    components: HashMap<ComponentId, ComponentEntry>,
    heartbeats: HashMap<ComponentId, MissionTime>,
    policy: P,
    safe_mode: bool,
    scheduler: SchedulerConfig,
}

impl<P: FdirPolicy> Runtime<P> {
    /// Creates a new runtime with the provided FDIR policy.
    #[must_use]
    pub fn new(policy: P) -> Self {
        Self {
            components: HashMap::new(),
            heartbeats: HashMap::new(),
            policy,
            safe_mode: false,
            scheduler: SchedulerConfig::default(),
        }
    }

    /// Replaces scheduler and watchdog settings.
    pub fn set_scheduler_config(&mut self, config: SchedulerConfig) {
        self.scheduler = config;
    }

    /// Registers a component in created state.
    pub fn register(&mut self, component: Box<dyn FswComponent>) -> Result<(), SdkError> {
        self.register_internal(component, None)
    }

    /// Registers a component with a YAML module manifest.
    pub fn register_with_manifest(
        &mut self,
        component: Box<dyn FswComponent>,
        manifest_yaml: &str,
    ) -> Result<(), SdkError> {
        let manifest = ModuleManifest::parse_yaml(manifest_yaml)?;
        let component_id = component.id();
        if manifest.id != component_id.0 {
            return Err(SdkError::InvalidConfig);
        }
        self.register_internal(component, Some(manifest))
    }

    fn register_internal(
        &mut self,
        component: Box<dyn FswComponent>,
        manifest: Option<ModuleManifest>,
    ) -> Result<(), SdkError> {
        let id = component.id();
        if self.components.contains_key(&id) {
            return Err(SdkError::InvalidConfig);
        }

        self.components.insert(
            id,
            ComponentEntry {
                state: ComponentLifecycleState::Created,
                component,
                manifest,
            },
        );
        Ok(())
    }

    /// Validates dependency graph using component contracts plus manifest rules.
    pub fn validate_dependency_graph(&self) -> Result<(), SdkError> {
        for entry in self.components.values() {
            for (dep_id, level) in entry.component.dependencies() {
                if level == DependencyLevel::Required && !self.components.contains_key(&dep_id) {
                    return Err(SdkError::DependencyMissing);
                }
            }

            if let Some(manifest) = &entry.manifest {
                for dep in &manifest.requires {
                    if !self.components.contains_key(&ComponentId(dep.clone())) {
                        return Err(SdkError::DependencyMissing);
                    }
                }
            }
        }

        Ok(())
    }

    /// Initializes and starts all registered components.
    pub fn start_all(&mut self) -> Result<(), SdkError> {
        self.validate_dependency_graph()?;

        for (id, entry) in &mut self.components {
            entry.component.init()?;
            entry.state = ComponentLifecycleState::Initialized;
            entry.component.start()?;
            entry.state = ComponentLifecycleState::Running;
            self.heartbeats.insert(id.clone(), MissionTime(0));
        }
        Ok(())
    }

    /// Stops and deinitializes all components.
    pub fn stop_all(&mut self) -> Result<(), SdkError> {
        for entry in self.components.values_mut() {
            entry.component.stop()?;
            entry.state = ComponentLifecycleState::Stopped;
            entry.component.deinit()?;
            entry.state = ComponentLifecycleState::Created;
        }
        self.heartbeats.clear();
        Ok(())
    }

    /// Marks a component heartbeat at the provided mission time.
    pub fn mark_heartbeat(&mut self, id: &ComponentId, now: MissionTime) -> Result<(), SdkError> {
        if !self.components.contains_key(id) {
            return Err(SdkError::ComponentNotFound);
        }
        self.heartbeats.insert(id.clone(), now);
        Ok(())
    }

    /// Runs one supervisor pass: health checks and heartbeat timeout checks.
    pub fn supervisor_step(
        &mut self,
        now: MissionTime,
    ) -> Result<Vec<(ComponentId, FaultResponse)>, SdkError> {
        let timeout_ms = self.scheduler.heartbeat_timeout_ms;
        let mut pending_faults: Vec<FaultEvent> = Vec::new();

        for (id, entry) in &self.components {
            if entry.state != ComponentLifecycleState::Running {
                continue;
            }

            match entry.component.health() {
                ComponentHealth::Nominal => {}
                ComponentHealth::Degraded => pending_faults.push(FaultEvent {
                    source: id.clone(),
                    class: FaultClass::Persistent,
                    detected_at: now,
                    reason: "component_degraded",
                }),
                ComponentHealth::Failed => pending_faults.push(FaultEvent {
                    source: id.clone(),
                    class: FaultClass::Critical,
                    detected_at: now,
                    reason: "component_failed",
                }),
            }

            let hb = self.heartbeats.get(id).copied().unwrap_or(MissionTime(0));
            let age = now.0.saturating_sub(hb.0);
            if age > timeout_ms {
                pending_faults.push(FaultEvent {
                    source: id.clone(),
                    class: FaultClass::Persistent,
                    detected_at: now,
                    reason: "heartbeat_timeout",
                });
            }
        }

        let mut responses = Vec::new();
        for event in pending_faults {
            let source = event.source.clone();
            let response = self.handle_fault(&event)?;
            responses.push((source, response));
        }

        Ok(responses)
    }

    /// Runs supervisor loop for a fixed number of cycles.
    pub fn run_supervisor_cycles<T: TimeSource>(
        &mut self,
        clock: &T,
        cycles: u32,
    ) -> Result<(), SdkError> {
        for _ in 0..cycles {
            let now = clock.now_monotonic();
            let _responses = self.supervisor_step(now)?;
            let target = MissionTime(now.0 + self.scheduler.period_ms);
            clock.tick(DurationMs(self.scheduler.period_ms));
            clock.sleep_until(target);
        }
        Ok(())
    }

    /// Applies FDIR response to a fault event.
    pub fn handle_fault(&mut self, event: &FaultEvent) -> Result<FaultResponse, SdkError> {
        let response = self.policy.select_response(event);
        let entry = self
            .components
            .get_mut(&event.source)
            .ok_or(SdkError::ComponentNotFound)?;

        match response {
            FaultResponse::Retry => {
                entry.component.reset()?;
                entry.component.start()?;
                entry.state = ComponentLifecycleState::Running;
            }
            FaultResponse::Restart => {
                entry.component.stop()?;
                entry.state = ComponentLifecycleState::Stopped;
                entry.component.deinit()?;
                entry.state = ComponentLifecycleState::Created;
                entry.component.init()?;
                entry.state = ComponentLifecycleState::Initialized;
                entry.component.start()?;
                entry.state = ComponentLifecycleState::Running;
            }
            FaultResponse::Isolate => {
                entry.component.stop()?;
                entry.state = ComponentLifecycleState::Isolated;
            }
            FaultResponse::EnterSafeMode => {
                self.safe_mode = true;
                entry.component.stop()?;
                entry.state = ComponentLifecycleState::Isolated;
            }
        }

        Ok(response)
    }

    /// Returns true when the runtime entered safe mode.
    #[must_use]
    pub fn in_safe_mode(&self) -> bool {
        self.safe_mode
    }

    /// Returns current health snapshot for all components.
    #[must_use]
    pub fn health_snapshot(&self) -> Vec<(ComponentId, ComponentHealth)> {
        self.components
            .iter()
            .map(|(id, entry)| (id.clone(), entry.component.health()))
            .collect()
    }
}

#[cfg(test)]
mod tests {
    use super::{ModuleManifest, Runtime, SchedulerConfig};
    use std::sync::{
        atomic::{AtomicU32, Ordering},
        Arc,
    };

    use fsw_sdk_core::{
        ComponentHealth, ComponentId, ComponentLifecycleState, DependencyLevel, DurationMs,
        FswComponent, MissionTime, SdkError, TimeSource,
    };
    use fsw_sdk_fdir::ClassMapPolicy;
    use fsw_sdk_time::StaticClock;

    #[derive(Default)]
    struct LifecycleCounters {
        init: AtomicU32,
        start: AtomicU32,
        stop: AtomicU32,
        reset: AtomicU32,
        deinit: AtomicU32,
    }

    struct TestComponent {
        id: String,
        deps: Vec<(ComponentId, DependencyLevel)>,
        health: ComponentHealth,
        counters: Arc<LifecycleCounters>,
    }

    impl TestComponent {
        fn new(id: &str) -> Self {
            Self {
                id: id.to_string(),
                deps: Vec::new(),
                health: ComponentHealth::Nominal,
                counters: Arc::new(LifecycleCounters::default()),
            }
        }

        fn with_required_dep(mut self, dep: &str) -> Self {
            self.deps
                .push((ComponentId(dep.to_string()), DependencyLevel::Required));
            self
        }

        fn with_counters(mut self, counters: Arc<LifecycleCounters>) -> Self {
            self.counters = counters;
            self
        }
    }

    impl FswComponent for TestComponent {
        fn id(&self) -> ComponentId {
            ComponentId(self.id.clone())
        }

        fn dependencies(&self) -> Vec<(ComponentId, DependencyLevel)> {
            self.deps.clone()
        }

        fn init(&mut self) -> Result<(), SdkError> {
            self.counters.init.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn start(&mut self) -> Result<(), SdkError> {
            self.counters.start.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn stop(&mut self) -> Result<(), SdkError> {
            self.counters.stop.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn reset(&mut self) -> Result<(), SdkError> {
            self.counters.reset.fetch_add(1, Ordering::Relaxed);
            self.health = ComponentHealth::Nominal;
            Ok(())
        }

        fn deinit(&mut self) -> Result<(), SdkError> {
            self.counters.deinit.fetch_add(1, Ordering::Relaxed);
            Ok(())
        }

        fn health(&self) -> ComponentHealth {
            self.health
        }
    }

    #[test]
    fn manifest_parser_accepts_minimal_yaml() {
        let yaml = "id: nav.app\nrequires:\n- imu.driver\noptional:\n- gps.driver\n";
        let manifest = ModuleManifest::parse_yaml(yaml).expect("valid yaml");
        assert_eq!(manifest.id, "nav.app".to_string());
        assert_eq!(manifest.requires, vec!["imu.driver".to_string()]);
        assert_eq!(manifest.optional, vec!["gps.driver".to_string()]);
    }

    #[test]
    fn dependency_validation_fails_when_required_missing() {
        let policy = ClassMapPolicy::conservative();
        let mut runtime = Runtime::new(policy);
        runtime
            .register(Box::new(TestComponent::new("nav").with_required_dep("imu")))
            .expect("register");

        let err = runtime.validate_dependency_graph().expect_err("must fail");
        assert_eq!(err, SdkError::DependencyMissing);
    }

    #[test]
    fn watchdog_raises_fault_when_heartbeat_stale() {
        let policy = ClassMapPolicy::conservative();
        let mut runtime = Runtime::new(policy);
        runtime.set_scheduler_config(SchedulerConfig {
            period_ms: 50,
            heartbeat_timeout_ms: 100,
        });
        runtime
            .register(Box::new(TestComponent::new("demo.component")))
            .expect("register");
        runtime.start_all().expect("start");
        runtime
            .mark_heartbeat(&ComponentId("demo.component".to_string()), MissionTime(0))
            .expect("heartbeat");

        let clock = StaticClock::new();
        clock.tick(DurationMs(200));
        let responses = runtime
            .supervisor_step(clock.now_monotonic())
            .expect("supervisor step");

        assert_eq!(responses.len(), 1);
    }

    struct RestartPolicy;

    impl fsw_sdk_fdir::FdirPolicy for RestartPolicy {
        fn select_response(
            &self,
            _event: &fsw_sdk_core::FaultEvent,
        ) -> fsw_sdk_core::FaultResponse {
            fsw_sdk_core::FaultResponse::Restart
        }
    }

    #[test]
    fn supervisor_cycles_advance_static_clock_without_deadlock() {
        let policy = ClassMapPolicy::conservative();
        let mut runtime = Runtime::new(policy);
        runtime
            .register(Box::new(TestComponent::new("demo.component")))
            .expect("register");
        runtime.start_all().expect("start");

        let clock = StaticClock::new();
        runtime
            .run_supervisor_cycles(&clock, 2)
            .expect("supervisor cycles");

        assert_eq!(clock.now_monotonic(), MissionTime(200));
    }

    #[test]
    fn restart_response_runs_full_lifecycle() {
        let counters = Arc::new(LifecycleCounters::default());
        let mut runtime = Runtime::new(RestartPolicy);
        runtime
            .register(Box::new(
                TestComponent::new("demo.component").with_counters(Arc::clone(&counters)),
            ))
            .expect("register");
        runtime.start_all().expect("start");

        let response = runtime
            .handle_fault(&fsw_sdk_core::FaultEvent {
                source: ComponentId("demo.component".to_string()),
                class: fsw_sdk_core::FaultClass::Persistent,
                detected_at: MissionTime(10),
                reason: "forced_restart",
            })
            .expect("handle fault");

        assert_eq!(response, fsw_sdk_core::FaultResponse::Restart);

        let entry = runtime
            .components
            .get(&ComponentId("demo.component".to_string()))
            .expect("component entry");

        assert_eq!(entry.state, ComponentLifecycleState::Running);
        assert_eq!(entry.component.health(), ComponentHealth::Nominal);
        assert_eq!(counters.init.load(Ordering::Relaxed), 2);
        assert_eq!(counters.start.load(Ordering::Relaxed), 2);
        assert_eq!(counters.stop.load(Ordering::Relaxed), 1);
        assert_eq!(counters.reset.load(Ordering::Relaxed), 0);
        assert_eq!(counters.deinit.load(Ordering::Relaxed), 1);
    }
}
