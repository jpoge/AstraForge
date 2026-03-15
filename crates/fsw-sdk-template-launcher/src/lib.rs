// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

//! Shared launcher helpers for AstraForge template applications.

use fsw_sdk_app::{AppContext, MissionApp};
use fsw_sdk_core::{FswComponent, SdkError};
use fsw_sdk_fdir::ClassMapPolicy;
use fsw_sdk_hal::{HalPlatform, HostSimPlatform};
use fsw_sdk_runtime::Runtime;

/// Runs a template app in host simulation, providing a configured runtime and context.
pub fn run_host_sim_example<T, B, F>(build: B, scenario: F) -> Result<HostSimPlatform, SdkError>
where
    T: MissionApp + FswComponent + 'static,
    B: Fn() -> T,
    F: FnOnce(&mut T, &AppContext<'_>, &HostSimPlatform) -> Result<(), SdkError>,
{
    let platform = HostSimPlatform::new();
    let ctx = AppContext {
        bus: platform.message_bus(),
        time: platform.time_source(),
    };

    let policy = ClassMapPolicy::conservative();
    let mut runtime = Runtime::new(policy);
    runtime.register(Box::new(build()))?;
    runtime.start_all()?;

    let mut app = build();
    MissionApp::init(&mut app, &ctx)?;
    scenario(&mut app, &ctx, &platform)?;

    Ok(platform)
}
