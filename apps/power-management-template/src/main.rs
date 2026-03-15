// Copyright (c) 2026 AstraForge Team/Mad Mountain, LLC
// SPDX-License-Identifier: Apache-2.0

use fsw_sdk_app::MissionApp;
use fsw_sdk_core::MessageBus;
use fsw_sdk_template_launcher::run_host_sim_example;
use power_management_template::{PowerManagementApp, PowerSnapshot, POWER_STATUS_TOPIC};

fn main() -> Result<(), fsw_sdk_core::SdkError> {
    let platform = run_host_sim_example(PowerManagementApp::new, |app, ctx, _platform| {
        app.update_snapshot(PowerSnapshot {
            bus_voltage_v: 27.8,
            bus_current_a: 3.4,
            battery_soc: 0.42,
        });
        app.step(ctx)
    })?;

    if let Some(status) = platform.bus().receive(&POWER_STATUS_TOPIC)? {
        println!("{}", String::from_utf8_lossy(&status));
    }

    Ok(())
}
